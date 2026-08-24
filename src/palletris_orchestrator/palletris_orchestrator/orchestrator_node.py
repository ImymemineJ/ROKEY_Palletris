#!/usr/bin/env python3
"""
orchestrator_node.py — 관제탑 (1차: B구역 전용)

박스 하나를 처리하는 7단계를 순서대로 진행한다.
"지금 몇 단계인가"를 self.state 하나에 적어두고, 응답이 오면 다음 단계로 넘긴다.

★ 설계 원칙 (이거 지키지 않으면 조용히 멈춘다)
  1. 모든 서비스 호출은 call_async + done_callback. 절대 spin_until_future_complete 금지
     → 콜백 안에서 응답을 기다리면 executor가 자기 자신을 기다리는 꼴이 된다
  2. MultiThreadedExecutor + ReentrantCallbackGroup 사용
  3. 로봇에게 명령하는 노드는 여기 하나뿐. algo도 detector도 로봇을 부르지 않는다
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
# ── 팀 공용 인터페이스 ───────────────────────────────────────
# ★ 이 import 가 ModuleNotFoundError 로 죽으면 packing_msgs 가 아직 안 깔린 것이다.
#   cd ~/palletris_ws && colcon build --packages-select packing_msgs
#   source install/setup.bash
from packing_msgs.msg import BoxAtPickup, PlacePose, RobotCmd, RobotEvent, ZoneStatus
from packing_msgs.srv import (
    CancelPlan,
    DespawnBox,
    GetBoxInfo,
    MarkPlaced,
    PlanPlacement,
    SetStatus,
    SetVelocity,
    UpdateHeightmap,
)


# ─────────────────────────────────────────────────────────────
# 상태 정의
# ─────────────────────────────────────────────────────────────
IDLE     = "IDLE"       # ① 벨트 가동, 박스 도착 대기
FETCHING = "FETCHING"   # ② 벨트 정지 + DB에 박스 정보 조회
PLANNING = "PLANNING"   # ③ algo에 놓을 자리 요청
PICKING  = "PICKING"    # ④ 로봇에게 집기 지시
PLACING  = "PLACING"    # ⑤ 로봇에게 놓기 지시
UPDATING = "UPDATING"   # ⑥ algo에 heightmap 갱신
DONE     = "DONE"       # ⑦ 팔레트 만재 → 종료

ZONE = "b"              # 1차는 B구역만

# ── 이 노드가 부르는 서비스 8개: 이름 → 타입 ──
# 표로 뺀 이유: 기존 호출부가 call_async("/db/get_box_info", {...}, cb) 처럼
# 서비스를 "문자열"로 넘긴다. 그 형태를 안 바꾸려면 이름→타입 사전이 필요하다.
SERVICES = {
    "/db/get_box_info":       GetBoxInfo,
    "/db/set_status":         SetStatus,
    "/db/mark_placed":        MarkPlaced,
    "/algo/plan_placement":   PlanPlacement,
    "/algo/update_heightmap": UpdateHeightmap,
    "/algo/cancel_plan":      CancelPlan,
    "/conveyor/set_velocity": SetVelocity,
    "/sim/despawn_box":       DespawnBox,
}

# ── 단위 · 좌표 변환 ─────────────────────────────────────────
# ⚠ 원래 자리는 packing_common/units.py · frames.py 이고 B·C 가 같은 것을
#   import 해야 한다 (CLAUDE.md §1 · §8). 그런데 packing_common 은 아직
#   빈 디렉터리다 (.gitkeep 뿐, package.xml 도 없음). 들어오면 여기를 지우고
#   import 로 바꾼다 — packing_robot/cycle.py 의 pallet_place_to_world 와 같은 처지다.
def m_to_mm(v):
    """m(float) → mm(int). ★ 반올림한다. 잘라내면 1mm 씩 계속 아래로 쏠린다."""
    return int(round(float(v) * 1000.0))


def quat_to_yaw_deg(q):
    """쿼터니언 → yaw(도). Z-up 이라 Z축 회전만 뽑는다.

    BoxAtPickup.pose 는 geometry_msgs/Pose 라 방향이 쿼터니언으로 온다.
    로봇에게 넘길 건 yaw 하나뿐이라 여기서 뽑아 쓴다.
    """
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


# ★ 로봇 모션 제한 시간. 집기·놓기는 5~10초 걸리므로 서비스 타임아웃(2초)과
#   전혀 다른 값이다. 이걸 짧게 잡으면 정상 동작 중에 계획을 버린다.
ROBOT_TIMEOUT_S  = 20.0

CALL_TIMEOUT_S   = 2.0    # 서비스 응답 제한. 넘으면 계획을 버리고 IDLE 로 돌아간다
WAIT_SERVER_S    = 2.0    # 서버가 아직 안 떴을 때 기다려 볼 시간
TIMEOUT_SCAN_S   = 0.1    # 타임아웃 감시 주기


class Orchestrator(Node):

    def __init__(self):
        super().__init__("orchestrator")

        # ReentrantCallbackGroup: 콜백이 서로를 막지 않게 한다.
        # 이게 없으면 서비스 응답 콜백이 타이머 콜백을 기다리다 멈춘다.
        self.cbg = ReentrantCallbackGroup()

        self.state = IDLE
        # reset_to_idle() 재진입 방지.
        # 벨트 서비스가 없으면 call_async 가 reset_to_idle 을 부르고,
        # reset_to_idle 이 다시 벨트를 불러 무한 재귀가 된다. 그걸 막는다.
        self._resetting = False
        self.box = None          # 지금 처리 중인 박스 {box_id, x_mm, y_mm, yaw_deg, w, d, h}
        self.plan = None         # algo가 준 놓을 자리
        self.t_start = time.time()
        self.placed = 0
        self.rejected = 0
        self.fill_rate = 0.0        # ZoneStatus 가 매번 읽으므로 미리 만들어 둔다
        self.max_height_mm = 0

        # ★ 명령 일련번호. 로봇이 event 에 이 값을 그대로 실어 보내야
        #   "지금 온 결과가 방금 그 명령의 것"임이 보장된다.
        #   없으면 타임아웃 뒤 늦게 온 이전 실패 응답을 새 박스 결과로 오인한다.
        self.seq = 0
        self.t_robot = None         # PICKING/PLACING 진입 시각. 타임아웃 판정용

        # ── 구독: 박스 도착 알림 (detector_zone_b가 발행) ──
        # ★ std_msgs/String JSON 이 아니라 packing_msgs/BoxAtPickup 이다
        #   (작업명세서 §2.1 · §2.3). 발행자는 트랙 D 의 detector_zone_b.
        self.create_subscription(
            BoxAtPickup, f"/zone_{ZONE}/box_at_pickup",
            self.on_box_arrived, 10, callback_group=self.cbg)

        # ── 구독: 로봇 결과 ──
        self.create_subscription(
            RobotEvent, "/robot/event",
            self.on_robot_event, 10, callback_group=self.cbg)

        # ── 발행: 로봇 명령 ──
        self.pub_robot = self.create_publisher(RobotCmd, "/robot/cmd", 10)

        # ── 발행: 구역 상태 (모니터링용, 2Hz) ──
        self.pub_status = self.create_publisher(ZoneStatus, f"/zone_{ZONE}/status", 10)
        self.create_timer(0.5, self.publish_status, callback_group=self.cbg)

        # ── 서비스 클라이언트 8개 ──
        # ★ callback_group=self.cbg 를 반드시 준다.
        #   이걸 빼면 응답 콜백이 기본 그룹(상호배제)에 묶인다. 그러면 구독 콜백
        #   안에서 부른 서비스의 답이 영원히 안 온다 — 에러 한 줄 없이 조용히 멈춘다.
        # ※ create_client 는 서버가 없어도 그냥 성공한다. 여기서는 기다리지 않는다.
        #   (기다리면 노드 시작이 8번 × 2초 = 16초 늦어진다)
        self.cli = {
            name: self.create_client(srv_type, name, callback_group=self.cbg)
            for name, srv_type in SERVICES.items()
        }

        # ── 진행 중인 서비스 호출 장부 ──
        # 호출마다 타이머를 새로 만들지 않는 이유: 타이머 콜백 안에서 자기 자신을
        # destroy_timer 하는 건 위험하다. 대신 감시 타이머 하나가 장부를 훑는다.
        self._lock = threading.Lock()     # 여러 스레드가 동시에 장부를 만진다
        self._pending = {}                # call_id → {srv, deadline, on_response, future}
        self._next_call_id = 0
        self.create_timer(TIMEOUT_SCAN_S, self._check_timeouts, callback_group=self.cbg)

        # 벨트 재가동 속도. ★ 숫자를 코드에 박지 않는다.
        #   출처: src/packing_bringup/config/system.yaml · conveyor.belt_velocity_mps
        self.declare_parameter("belt_velocity_mps", 0.5)

        self.get_logger().info("관제탑 시작 — 상태: IDLE")

    # ══════════════════════════════════════════════════════════
    # ① IDLE → ② FETCHING : 박스가 스토퍼에 도착했다
    # ══════════════════════════════════════════════════════════
    def on_box_arrived(self, msg):
        """BoxAtPickup 수신. ★ 여기서는 판단만 하고 무거운 일을 하지 않는다 (§1.5)."""
        if self.state != IDLE:
            # 처리 중에 또 도착 알림이 오면 무시한다.
            # (detector 는 5Hz 폴링이라 같은 박스를 여러 번 알린다)
            return

        # ── ① 이탈 알림은 걸러낸다 ──
        # 박스가 집혀서 스토퍼를 떠나면 present=false 가 온다 (작업명세서 §5.6).
        # 이걸 안 거르면 "없는 박스"로 벨트를 세우고 사이클을 시작한다.
        if not msg.present:
            return

        # ── ② box_id 가 비었으면 아무것도 못 한다 ──
        # 다음 단계가 /db/get_box_info(box_id) 인데 빈 문자열로는 조회가 안 된다.
        # detector 가 색 → box_id 매칭에 실패하면 이렇게 온다.
        if not msg.box_id:
            self.get_logger().warn("box_at_pickup 에 box_id 가 비어 있다 — 무시")
            return

        # ── ③ m → mm ──
        # ★ BoxAtPickup.pose 는 월드 좌표 m, 박스 "중심" 이다 (작업명세서 §2.1).
        #   orchestrator 내부와 /robot/cmd 는 mm 라서 여기서 한 번만 바꾼다.
        pos = msg.pose.position
        self.box = {
            "box_id":   msg.box_id,
            "x_mm":     m_to_mm(pos.x),
            "y_mm":     m_to_mm(pos.y),
            "yaw_deg":  quat_to_yaw_deg(msg.pose.orientation),
        }

        # ── ④ 단위 점검 (경고만) ──
        # 스토퍼는 x = 10.0 m = 10000 mm 다 (CLAUDE.md §1).
        # 100m 를 넘으면 발행자가 pose 에 m 가 아니라 mm 를 담은 것이다.
        # 막지는 않는다 — 막으면 진짜 원인이 안 보인다. 대신 크게 남긴다.
        if abs(self.box["x_mm"]) > 100_000:
            self.get_logger().warn(
                f"x={pos.x} → {self.box['x_mm']}mm — 스토퍼는 10000mm 다. "
                f"발행자가 pose 에 mm 를 담고 있지 않은지 확인할 것 (계약은 m)")

        self.get_logger().info(
            f"[①→②] 박스 도착: {self.box['box_id']} "
            f"({self.box['x_mm']}, {self.box['y_mm']}) yaw {self.box['yaw_deg']:.1f}°")
        self.state = FETCHING

        # 벨트를 먼저 세운다. 안 세우고 흡착하면 박스가 끌려간다.
        self.call_set_velocity(0.0, then=self.fetch_box_info)

    # ══════════════════════════════════════════════════════════
    # ② FETCHING : DB에 "이 박스 크기 뭐야?" 물어본다
    # ══════════════════════════════════════════════════════════
    def fetch_box_info(self):
        def on_response(future):
            res = future.result()
            if not res.found:
                # DB에 없는 박스 = 인식이 실패했다. 집지 않고 버린다.
                self.get_logger().warn(f"DB에 없음: {self.box['box_id']} → 폐기")
                self.discard_box("NOT_IN_DB")
                return
            self.box.update({
                "w": res.info.size_w_mm,
                "d": res.info.size_d_mm,
                "h": res.info.size_h_mm,
            })
            self.get_logger().info(
                f"[②→③] 크기 {res.info.size_w_mm}×{res.info.size_d_mm}×{res.info.size_h_mm}")
            self.state = PLANNING
            self.request_plan()

        self.call_async("/db/get_box_info", {"box_id": self.box["box_id"]}, on_response)

    # ══════════════════════════════════════════════════════════
    # ③ PLANNING : algo에 "어디 놓을까?" 물어본다
    # ══════════════════════════════════════════════════════════
    def request_plan(self):
        def on_response(future):
            res = future.result()

            # ── 이탈 경로 A: 팔레트 만재 → 시나리오 종료 ──
            if res.message == "PALLET_FULL":
                self.get_logger().info("PALLET_FULL — 시나리오 종료")
                self.finish()
                return

            # ── 이탈 경로 B: 이 박스만 자리 없음 → 박스만 버리고 계속 ──
            # ★ 여기서 끝내면 안 된다. 다음 박스는 들어갈 수 있다.
            if res.message == "NO_FEASIBLE_POSE" or not res.ok:
                self.get_logger().info(f"NO_FEASIBLE_POSE: {self.box['box_id']} → 폐기")
                self.discard_box("NO_FEASIBLE_POSE")
                return

            self.plan = res.pose
            self.get_logger().info(
                f"[③→④] 놓을 자리 ({res.pose.x_mm}, {res.pose.y_mm}, {res.pose.z_mm})")
            self.state = PICKING
            self.t_robot = time.time()      # 집기 구간 타임아웃 시작
            self.send_robot_cmd(RobotCmd.PICK)

        self.call_async("/algo/plan_placement", {
            "zone": ZONE,
            "box_id": self.box["box_id"],
            "size_w_mm": self.box["w"],
            "size_d_mm": self.box["d"],
            "size_h_mm": self.box["h"],
        }, on_response)

    # ══════════════════════════════════════════════════════════
    # ④⑤ PICKING / PLACING : 로봇에게 명령하고 결과를 기다린다
    # ══════════════════════════════════════════════════════════
    def send_robot_cmd(self, action):
        """로봇에게 명령 하나를 던진다. 결과는 /robot/event 로 돌아온다.

        ★ 보낼 때마다 seq 를 올린다. 로봇이 그 seq 를 되돌려줘야
          늦게 온 지난 명령의 결과와 구분할 수 있다.
        """
        self.seq += 1
        cmd = RobotCmd()
        cmd.action = action
        cmd.seq = self.seq

        if action == RobotCmd.PICK:
            cmd.box_id = self.box["box_id"]
            cmd.x_mm = self.box["x_mm"]
            cmd.y_mm = self.box["y_mm"]
            # 집을 z = 벨트 상면(800) + 박스 높이. 그리퍼 여유는 트랙 B가 뺀다.
            cmd.z_mm = 800 + self.box["h"]
            cmd.yaw_deg = float(self.box["yaw_deg"])
            cmd.size_w_mm = self.box["w"]
            cmd.size_d_mm = self.box["d"]
            cmd.size_h_mm = self.box["h"]

        elif action == RobotCmd.PLACE:
            cmd.box_id = self.box["box_id"]
            cmd.x_mm = self.plan.x_mm
            cmd.y_mm = self.plan.y_mm
            cmd.z_mm = self.plan.z_mm   # ★ 밑면 기준. 중심으로 쓰면 팔레트에 파묻힌다
            cmd.yaw_deg = float(self.plan.yaw_deg)
            cmd.size_w_mm = self.box["w"]
            cmd.size_d_mm = self.box["d"]
            cmd.size_h_mm = self.box["h"]

        # HOME / STOP 은 box_id 도 좌표도 비운다 (기본값 그대로).

        # TODO: approach_z_mm — 수직 하강을 시작할 수 있는 높이.
        #   /algo/plan_placement 응답에 통로 높이가 없어서 채울 값이 없다.
        #   0 = 로봇 재량. algo 가 stack_top 을 실어 주면 그때 채운다.
        cmd.approach_z_mm = 0

        self.pub_robot.publish(cmd)

    def on_robot_event(self, ev):
        # ── ① seq 대조 ── ★ 무엇보다 먼저 한다.
        #   타임아웃으로 손을 뗀 뒤 로봇이 뒤늦게 답하면, 그 실패가 다음 박스의
        #   결과로 둔갑한다. 에러도 안 나고 엉뚱하게 도는 버그다.
        if ev.seq != self.seq:
            self.get_logger().warn(
                f"지난 명령의 결과 무시: seq {ev.seq} (지금은 {self.seq})")
            return

        # 처리 중이 아닐 때 온 것도 버린다.
        # (self.box 가 None 인데 ["box_id"] 를 읽으면 노드가 죽는다)
        if self.state not in (PICKING, PLACING) or self.box is None:
            return

        # ── 이탈 경로 C: 로봇 실패 → 계획 취소하고 처음으로 ──
        if not ev.ok:
            self.get_logger().warn(f"로봇 실패({self.state}): {ev.reason}")
            self.call_async("/algo/cancel_plan",
                            {"zone": ZONE, "box_id": self.box["box_id"]}, None)
            self.call_async("/db/set_status",
                            {"box_id": self.box["box_id"], "status": "PICK_FAILED",
                             "reason": ev.reason}, None)
            self.reset_to_idle()
            return

        if self.state == PICKING:
            self.get_logger().info(f"[④→⑤] 집기 성공 ({ev.duration_s:.1f}초)")
            self.call_async("/db/set_status",
                            {"box_id": self.box["box_id"], "status": "PICKED",
                             "reason": ""}, None)
            self.state = PLACING
            self.t_robot = time.time()          # 놓기 구간 타임아웃 다시 시작
            self.send_robot_cmd(RobotCmd.PLACE)

        elif self.state == PLACING:
            self.get_logger().info(f"[⑤→⑥] 놓기 성공 ({ev.duration_s:.1f}초)")
            self.state = UPDATING
            self.t_robot = None
            self.update_heightmap(self._actual_pose(ev))

    def _actual_pose(self, ev):
        """heightmap 에 넣을 pose 를 고른다.

        ★ 계획 pose 가 아니라 로봇이 잰 실측 pose 를 쓴다. 계획값을 되보내면
          오차가 누적돼 알고리즘은 840mm 로 끝났다고 믿는데 실제 더미는 850mm 를 넘는다.
        로봇이 아직 실측을 못 하면(has_actual=false) 계획값으로라도 진행한다 —
        여기서 멈추면 로봇 쪽 미구현 때문에 사이클 전체가 안 돈다.
        """
        if not ev.has_actual:
            self.get_logger().warn(
                "로봇이 실측 pose 를 안 줬다 (has_actual=false) — 계획값으로 갱신한다")
            return self.plan
        return {"x_mm": ev.actual_x_mm, "y_mm": ev.actual_y_mm,
                "z_mm": ev.actual_z_mm, "yaw_deg": ev.actual_yaw_deg}

    def check_robot_timeout(self):
        """로봇이 20초 안에 답하지 않으면 손을 뗀다. 0.5초 타이머가 부른다.

        ★ 이게 없으면 로봇 노드가 죽었을 때 관제탑이 PICKING 인 채로 영원히 선다.
          벨트도 멈춰 있어서 라인 전체가 조용히 정지한다.
        """
        if self.state not in (PICKING, PLACING) or self.t_robot is None:
            return
        held = time.time() - self.t_robot
        if held < ROBOT_TIMEOUT_S:
            return

        bid = self.box["box_id"] if self.box else ""
        self.get_logger().error(
            f"로봇 응답 없음 — {self.state} 에서 {held:.1f}초 (제한 {ROBOT_TIMEOUT_S}초). 계획을 버린다")
        self.t_robot = None
        self.call_async("/algo/cancel_plan", {"zone": ZONE, "box_id": bid}, None)
        self.call_async("/db/set_status",
                        {"box_id": bid, "status": "PICK_FAILED", "reason": "TIMEOUT"}, None)
        self.reset_to_idle()

    # ══════════════════════════════════════════════════════════
    # ⑥ UPDATING : algo에 "여기 놨다" 알려준다
    # ══════════════════════════════════════════════════════════
    def update_heightmap(self, actual_pose):
        def on_response(future):
            res = future.result()
            if not res.ok:
                # ok=false = yaw 오차 5° 초과 경고. 로그만 남기고 계속 진행한다.
                self.get_logger().warn("yaw 오차 초과 — 계속 진행")
            self.placed += 1
            self.fill_rate = res.fill_rate
            self.max_height_mm = res.max_height_mm
            self.get_logger().info(
                f"[⑥→①] 적재 {self.placed}개 · 충전율 {res.fill_rate:.1%}"
                f" · 최고 {res.max_height_mm}mm")
            # ★ MarkPlaced 는 zone 과 actual_pose 를 받는다. 계획 pose 가 아니다.
            self.call_async("/db/mark_placed",
                            {"box_id": self.box["box_id"], "zone": ZONE,
                             "actual_pose": actual_pose}, None)
            self.reset_to_idle()

        self.call_async("/algo/update_heightmap", {
            "zone": ZONE,
            "box_id": self.box["box_id"],
            "size_w_mm": self.box["w"],
            "size_d_mm": self.box["d"],
            "size_h_mm": self.box["h"],
            "actual_pose": actual_pose,
        }, on_response)

    # ══════════════════════════════════════════════════════════
    # 공통 처리
    # ══════════════════════════════════════════════════════════
    def discard_box(self, reason=""):
        """1차 전용: 놓을 수 없는 박스를 씬에서 지운다.
        2차에는 Return line으로 반송하지만 1차엔 그 길이 없다.
        스토퍼에 두면 뒤 박스가 밀려 라인이 막힌다.

        reason 은 기본값이 있어 기존 호출부 discard_box() 는 그대로 동작한다.
        SetStatus 에 reason 필드가 생겨서, 왜 버렸는지를 DB 에 남기려고 받는다."""
        bid = self.box["box_id"]
        self.rejected += 1
        self.call_async("/algo/cancel_plan", {"zone": ZONE, "box_id": bid}, None)
        self.call_async("/db/set_status",
                        {"box_id": bid, "status": "REJECTED", "reason": reason}, None)
        self.call_async("/sim/despawn_box", {"box_id": bid}, None)
        self.reset_to_idle()

    def reset_to_idle(self):
        """벨트를 다시 돌리고 다음 박스를 기다린다.

        ★ 재진입 가드가 필요한 이유:
          벨트 서비스가 안 떠 있으면 call_async 가 reset_to_idle 을 부르고,
          reset_to_idle 이 다시 벨트를 부른다 → 무한 재귀로 노드가 죽는다.
          한 번 들어오면 안쪽 호출은 그냥 돌려보낸다."""
        if self._resetting:
            return
        self._resetting = True
        try:
            self.box = None
            self.plan = None
            self.state = IDLE
            self.t_robot = None
            # 숫자를 코드에 박지 않는다 — system.yaml 의 conveyor.belt_velocity_mps
            self.call_set_velocity(
                self.get_parameter("belt_velocity_mps").value, then=None)
            self.get_logger().info("[→①] 대기")
        finally:
            self._resetting = False

    def finish(self):
        """PALLET_FULL — 최종 지표를 찍고 멈춘다."""
        self.state = DONE
        self.call_set_velocity(0.0, then=None)
        self.send_robot_cmd_home()
        elapsed = time.time() - self.t_start
        self.get_logger().info(
            "═══ 시나리오 종료 ═══\n"
            f"  적재: {self.placed}개 / 폐기: {self.rejected}개\n"
            f"  충전율: {self.fill_rate:.1%} · 최고 {self.max_height_mm}mm\n"
            f"  소요: {elapsed:.1f}초")

    def send_robot_cmd_home(self):
        self.send_robot_cmd(RobotCmd.HOME)

    def publish_status(self):
        # 로봇 타임아웃도 이 타이머로 같이 본다 (0.5초면 20초 판정에 충분하다).
        # 상태를 먼저 정리하고 나서 발행해야 방금 끊긴 것이 바로 보인다.
        self.check_robot_timeout()

        msg = ZoneStatus()
        msg.zone = ZONE
        msg.state = self.state
        # queue_len 은 /db/zone_queue 가 유일한 출처인데(§6.7) 관제탑은 그 서비스를
        # 안 쓴다. 씬에서 직접 세면 DB 값과 어긋나므로 0 으로 둔다 — 2차에 채운다.
        msg.queue_len = 0
        msg.placed_count = self.placed
        msg.rejected_count = self.rejected
        msg.fill_rate = float(self.fill_rate)
        msg.max_height_mm = int(self.max_height_mm)
        # ★ 중재자는 이 한 칸만 보고 "이 구역에 더 보낼지"를 정한다.
        msg.accepting = (self.state != DONE)
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_status.publish(msg)

    # ── 서비스 호출 헬퍼 ──────────────────────────────────────
    def call_set_velocity(self, mps, then):
        """벨트 속도 변경. 응답 ok를 확인한 뒤 다음 단계로 간다."""
        def on_response(future):
            res = future.result()
            if not res.ok:
                # ★ 벨트가 안 멈춘 채 흡착하면 박스가 끌려간다. 다음 단계로 안 넘어간다.
                self.get_logger().error(
                    f"벨트 속도 변경 실패 — 요청 {mps} m/s / 실제 {res.actual_mps} m/s")
                # then 이 있는 쪽(= 집기 전 정지)만 되돌린다.
                # then 이 없는 쪽은 reset_to_idle 안의 재가동이라,
                # 여기서 또 reset 을 부르면 서로 계속 부르는 고리가 된다.
                if then is not None:
                    self.reset_to_idle()
                return
            if then:
                then()
        self.call_async("/conveyor/set_velocity",
                        {"seg": "b_pickup", "velocity_mps": mps}, on_response)

    def call_async(self, srv_name, req_dict, on_response):
        """서비스 1회 호출. ★ call_async + add_done_callback 만 쓴다.

        spin_until_future_complete 를 쓰면 executor 가 자기 자신을 기다린다.
        에러도 안 나고 그냥 멈춘다 — 가장 찾기 어려운 고장이다.

        on_response=None 이면 "보내고 잊는" 호출이다(로그·기록용).
        이런 호출은 실패해도 상태기계를 건드리지 않는다.
        """
        cli = self.cli.get(srv_name)
        if cli is None:
            self.get_logger().error(f"모르는 서비스: {srv_name} — SERVICES 표에 없다")
            return

        # ── 서버가 떠 있나 ──
        # 먼저 service_is_ready() 로 물어보고, 아닐 때만 2초 기다린다.
        # wait_for_service 는 executor 가 아니라 rclpy 의 별도 graph 스레드가
        # 처리하므로 콜백 안에서 불러도 데드락은 안 난다. 다만 그 2초 동안
        # 이 스레드는 멈추므로 꼭 필요할 때만 쓴다.
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=WAIT_SERVER_S):
            self.get_logger().error(
                f"서비스 없음: {srv_name} — {WAIT_SERVER_S}초 기다렸지만 서버가 안 떴다. 대기로 복귀")
            self.reset_to_idle()     # ★ 노드는 죽이지 않는다. 다음 박스 때 다시 시도한다
            return

        # ── dict → ROS 요청 객체 ──
        try:
            req = self._fill_request(cli.srv_type.Request(), req_dict)
        except Exception as e:
            self.get_logger().error(f"{srv_name} 요청 변환 실패: {e}")
            self.reset_to_idle()
            return

        # 장부에 먼저 올리고 나서 보낸다.
        # 순서를 바꾸면 응답이 너무 빨리 와서 장부에 없는 call_id 를 찾게 된다.
        with self._lock:
            call_id = self._next_call_id
            self._next_call_id += 1
            call = {"srv": srv_name, "on_response": on_response,
                    "deadline": time.time() + CALL_TIMEOUT_S, "future": None}
            self._pending[call_id] = call

        future = cli.call_async(req)
        call["future"] = future
        future.add_done_callback(lambda f, cid=call_id: self._on_service_done(cid, f))

    # ── call_async 내부 도우미 ────────────────────────────────
    def _fill_request(self, req, req_dict):
        """파이썬 dict 를 ROS 요청 객체로 채운다.

        rclpy 는 타입이 조금만 달라도 거절한다 (int32 자리에 float 를 넣으면 예외).
        특히 로봇이 JSON 으로 준 값은 int 인지 float 인지 그때그때 다르므로
        필드 타입을 보고 여기서 한 번 맞춰준다.
        """
        types = req.get_fields_and_field_types()
        for key, val in req_dict.items():
            if key not in types:
                # ★ 조용히 넘어가면 "보냈는데 왜 반영이 안 되지"로 30분을 태운다
                self.get_logger().warn(f"요청에 없는 필드 무시: {key} (있는 필드: {list(types)})")
                continue
            t = types[key]
            if t.endswith("PlacePose"):
                setattr(req, key, self._to_place_pose(val))
            elif t in ("float", "double"):
                setattr(req, key, float(val))
            elif t.startswith("int") or t.startswith("uint"):
                setattr(req, key, int(val))
            elif t == "boolean":
                setattr(req, key, bool(val))
            else:
                setattr(req, key, str(val))
        return req

    def _to_place_pose(self, val):
        """PlacePose 로 만든다. 이미 PlacePose 면 그대로, dict 면 변환한다.

        ★ z_mm 은 박스 "밑면". 로봇이 중심값을 보내오면 여기서는 못 잡는다 —
          트랙 B 와 맞춘 규약을 믿는다 (작업명세서 §3.3).
        """
        if isinstance(val, PlacePose):
            return val
        pose = PlacePose()
        pose.x_mm    = int(val["x_mm"])
        pose.y_mm    = int(val["y_mm"])
        pose.z_mm    = int(val["z_mm"])
        pose.yaw_deg = float(val.get("yaw_deg", 0.0))
        return pose

    def _on_service_done(self, call_id, future):
        """응답 도착. 장부에서 빼는 데 성공한 쪽만 처리한다."""
        with self._lock:
            call = self._pending.pop(call_id, None)
        if call is None:
            return                      # 이미 타임아웃으로 처리됐다. 늦게 온 답은 버린다

        srv_name = call["srv"]
        on_response = call["on_response"]
        try:
            future.result()             # 통신 자체가 실패했는지 먼저 본다
        except Exception as e:
            self.get_logger().error(f"{srv_name} 호출 실패: {e}")
            if on_response is not None:
                self.reset_to_idle()
            return

        if on_response is None:
            return                      # 보내고 잊는 호출

        try:
            on_response(future)
        except Exception as e:
            # ★ 콜백 안에서 난 예외는 rclpy 가 삼켜서 아무 일도 없던 것처럼 보인다.
            #   여기서 잡아 로그로 남기지 않으면 "박스 하나 처리하다 그냥 멈춤"이 된다.
            self.get_logger().error(f"{srv_name} 응답 처리 중 예외: {e}")
            self.reset_to_idle()

    def _check_timeouts(self):
        """감시 타이머(0.1초). 제한 시간을 넘긴 호출을 걷어낸다."""
        now = time.time()
        with self._lock:
            expired = [cid for cid, c in self._pending.items() if now >= c["deadline"]]
            calls = [self._pending.pop(cid) for cid in expired]
        for call in calls:
            self._handle_timeout(call)

    def _handle_timeout(self, call):
        srv_name = call["srv"]
        future = call["future"]
        if future is not None:
            future.cancel()                          # 더 안 기다린다
            cli = self.cli.get(srv_name)
            try:
                cli.remove_pending_request(future)   # 클라이언트 내부 대기표도 지운다
            except Exception:
                pass

        self.get_logger().error(f"{srv_name} 응답 없음 — {CALL_TIMEOUT_S}초 초과")

        if call["on_response"] is None:
            return                                   # 보내고 잊는 호출은 상태를 안 건드린다
        if self.state in (IDLE, DONE):
            return                                   # 처리 중인 박스가 없으면 되돌릴 것도 없다

        # ★ 이탈할 때는 계획을 반드시 버린다. 안 버리면 그 자리가 영원히 예약된 채 남는다.
        #   단, 타임아웃 난 게 cancel_plan 자신이면 또 부르지 않는다 (무한 반복).
        if self.box and srv_name != "/algo/cancel_plan":
            self.call_async("/algo/cancel_plan",
                            {"zone": ZONE, "box_id": self.box["box_id"]}, None)
        self.reset_to_idle()


def main():
    rclpy.init()
    node = Orchestrator()
    # ★ MultiThreadedExecutor가 아니면 서비스 응답 콜백이 서로를 막는다.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGTERM(ros2 launch 가 자식을 끌 때)으로 죽으면 rclpy 신호 처리기가
        # context 를 이미 닫아 놓는다. 그 상태에서 또 부르면 traceback 이 뜬다.
        # 정상 종료인데 죽은 것처럼 보이므로 막는다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
