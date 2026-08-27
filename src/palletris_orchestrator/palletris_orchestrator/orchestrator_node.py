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

import os
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
    ResetHeightmap,
    SetSpawnEnabled,
    SetVelocity,
    SwapPallet,
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

# ★ 예전에 여기 있던 모듈 전역 ZONE / ZONE_ID 는 없앴다 (2026-08-25).
#   관제탑은 **하나뿐이고 구역 전부를 맡는다** (CLAUDE.md §8). 전역에 구역을 두면
#   구역이 둘 이상인 순간 A 구역 사이클이 C 구역 값을 읽는다 — 에러 없이 엉뚱한
#   팔레트에 쌓는다. 구역별 값은 전부 ZoneCycle 인스턴스가 들고 있다.

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
    # ★ 2026-08-26 추가 — 팔레트 교체(§7 사양 경로). 서버는 셋 다 있었는데
    #   **부르는 쪽이 없어서** PALLET_FULL 이 그냥 종료로 끝나고 있었다.
    "/sim/set_spawn_enabled": SetSpawnEnabled,
    "/sim/swap_pallet":       SwapPallet,
    "/algo/reset_heightmap":  ResetHeightmap,
}

# ★ 로봇 모션 제한 시간. 집기·놓기는 5~10초 걸리므로 서비스 타임아웃(2초)과
#   전혀 다른 값이다. 이걸 짧게 잡으면 정상 동작 중에 계획을 버린다.
ROBOT_TIMEOUT_S  = 20.0

CALL_TIMEOUT_S   = 2.0    # 서비스 응답 제한. 넘으면 계획을 버리고 IDLE 로 돌아간다
# ★★ 2026-08-26 — **서비스별 예외.** 2.0 초는 "plan() < 200ms"(명세서 §7.9) 를
#   전제로 정한 값인데, `lookahead k:4` 를 켠 뒤 plan 한 번이 786~859 ms 다
#   (algo 노드 자신이 ERROR 로 찍는다). 구역이 셋이면 파이썬 GIL 때문에 사실상
#   줄을 서서 2.4 초가 되고, **세 구역이 동시에 타임아웃 난다** — 로봇이 집은 채
#   멈춘다 (실측 2026-08-26 error4).
#   여기서 늘리는 것은 **증상 완화**다. 근본은 둘 중 하나로 정해야 한다:
#     · lookahead 를 줄여 §7.9 를 지킨다 (k:1 이면 1 ms)
#     · §7.9 기준을 실제 비용에 맞춰 개정한다 (정정인님 결정 사항)
CALL_TIMEOUT_BY_SRV = {
    "/algo/plan_placement":   6.0,
    "/algo/update_heightmap": 6.0,
    "/algo/cancel_plan":      6.0,
    "/algo/reset_heightmap":  6.0,
}
WAIT_SERVER_S    = 2.0    # 서버가 아직 안 떴을 때 기다려 볼 시간
TIMEOUT_SCAN_S   = 0.1    # 타임아웃 감시 주기
# ★ IDLE 인데 스토퍼에 박스가 이만큼 서 있으면 치운다.
#   로봇 타임아웃(20초)보다 짧게 둔다 — 이쪽은 아무 작업도 진행 중이 아니라
#   오래 기다릴 이유가 없고, 그동안 뒤 박스가 계속 밀려온다.
STUCK_TIMEOUT_S  = 12.0


class ZoneCycle:
    """구역 하나의 상태머신. **노드가 아니다** — 관제탑 노드가 구역마다 하나씩 든다.

    ★ 관제탑은 프로세스도 노드도 하나다 (CLAUDE.md §8). 구역이 셋이어도 늘어나는
      것은 이 객체뿐이다. 로봇 명령의 유일한 발신자라는 규약이 그래야 지켜진다 —
      프로세스를 셋으로 나누면 누가 먼저 보냈는지 알 수 없게 된다.
    """

    def __init__(self, node, zone, belt_mps, belt_top_mm):
        self.node = node
        self.zone = str(zone).lower()
        # ★ 대소문자 규약: 서비스 인자의 zone 은 **대문자**, 토픽 경로만 소문자다
        #   (작업명세서 §2.1 "★ 대문자. 경로에서만 소문자 zone_b").
        #   소문자로 보내면 algo 의 packer 사전(키가 A·B·C)에서 안 잡혀
        #   빈 팔레트인데도 NO_FEASIBLE_POSE 가 돌아온다. 실측으로 확인한 버그다 —
        #   에러 로그는 algo 쪽에만 찍히고 관제탑은 "자리가 없구나" 로 오해한다.
        self.zone_id = self.zone.upper()
        # ★ 구역별 **자식 로거**를 쓴다. 로그 앞에 구역이 붙어 누가 한 말인지 보인다.
        #
        # ★ 래퍼로 감싸면 안 된다. 한때 __getattr__ + 람다로 감쌌다가 노드가 죽었다:
        #     ValueError: Logger severity cannot be changed between calls.
        #   rclpy 는 `once` · `throttle_duration_sec` 를 **호출 위치**(파일·줄·함수)로
        #   캐시하는데, 래퍼를 쓰면 모든 로그가 람다 한 줄에서 나온 것으로 보인다.
        #   심각도가 섞이는 순간 예외가 나고, 그 예외가 구독 콜백을 타고 올라가
        #   **프로세스가 통째로 죽는다.** 실측으로 확인했다.
        self.log = node.get_logger().get_child(self.zone)
        self.belt_mps = float(belt_mps)
        # 벨트 상면 z [mm]. 벨트 위 박스의 **밑면** 높이이기도 하다.
        # ★ 숫자를 코드에 박지 않는다 — system.yaml · conveyor.belt_top_z_mm
        self.belt_top_mm = int(belt_top_mm)

        # ReentrantCallbackGroup: 콜백이 서로를 막지 않게 한다.
        # 이게 없으면 서비스 응답 콜백이 타이머 콜백을 기다리다 멈춘다.
        self.cbg = node.cbg

        self.state = IDLE
        # reset_to_idle() 재진입 방지.
        # 벨트 서비스가 없으면 call_async 가 reset_to_idle 을 부르고,
        # reset_to_idle 이 다시 벨트를 불러 무한 재귀가 된다. 그걸 막는다.
        self._resetting = False
        # ★ 방금 버린 박스. 씬이 despawn 을 반영하기 전까지 detector 는 같은 박스를
        #   계속 present=True 로 알린다. 그대로 두면 버린 박스를 또 집으려 하고,
        #   또 버리고, 또 집으려 하는 무한 루프가 된다 — 에러 없이 라인만 선다.
        #   실측으로 확인했다(ROS 그래프 검증). 같은 id 가 다시 오면 무시한다.
        self._rejected_id = None
        # ★ IDLE 인데 스토퍼에 박스가 계속 서 있는 것을 감시한다.
        #   cam1 이 놓쳐 등록이 안 됐거나(box_id 빈 문자열), camera_info·tf 가 없어
        #   좌표를 못 내면 관제탑은 "무시" 하고 끝난다. 그러면 **아무도 그 박스를
        #   치우지 않는다** — 뒤 박스가 밀려 라인이 통째로 막힌다 (CLAUDE.md §7
        #   "스토퍼에 박스를 두면 라인이 막히므로 반드시 치운다").
        #   로봇 타임아웃(check_robot_timeout)은 PICKING/PLACING 만 본다.
        self._stuck_id = None            # 못 쓰고 있는 박스 id ("" 면 식별 실패)
        self._stuck_since = None
        # ★ 방금 **적재를 끝낸** 박스. 버린 박스와 같은 이유로 필요하다 —
        #   PLACE 를 답한 직후에도 detector 가 보내 둔 이전 프레임이 큐에 남아 있어
        #   present=true·같은 box_id 로 도착한다. 그대로 두면 **같은 박스를 두 번
        #   쌓는다.** 실측으로 확인했다(3구역 9박스): BOX00006 이 두 번 적재되고
        #   그 바람에 BOX00007 이 통째로 밀려났다 — 에러는 한 줄도 안 났다.
        #   present=false(스토퍼가 빈 것)를 한 번 봐야 풀린다.
        self._done_id = None
        self.box = None          # 지금 처리 중인 박스 {box_id, x_mm, y_mm, yaw_deg, w, d, h}
        self.plan = None         # algo가 준 놓을 자리
        self.t_start = time.time()
        self.placed = 0
        self.rejected = 0
        self.fill_rate = 0.0        # ZoneStatus 가 매번 읽으므로 미리 만들어 둔다
        self.max_height_mm = 0
        # ★ 판갈이 — PALLET_FULL 이 나면 몇 판까지 이어갈지. 1 이면 예전 동작
        #   (첫 만재에 종료). 데모로 트럭 상차·재시작을 보려면 2 이상.
        self._last_ev = None      # 마지막으로 처리한 (seq, action)
        self.pallets_done = 0
        self.max_pallets = int(os.environ.get("PL_MAX_PALLETS", "3"))

        # ★ 명령 일련번호. 로봇이 event 에 이 값을 그대로 실어 보내야
        #   "지금 온 결과가 방금 그 명령의 것"임이 보장된다.
        #   없으면 타임아웃 뒤 늦게 온 이전 실패 응답을 새 박스 결과로 오인한다.
        self.seq = 0
        self.t_robot = None         # PICKING/PLACING 진입 시각. 타임아웃 판정용

        # ── 구독: 박스 도착 알림 (zone_detector 가 발행) ──
        # ★ std_msgs/String JSON 이 아니라 packing_msgs/BoxAtPickup 이다
        #   (작업명세서 §2.1 · §2.3). 발행자는 트랙 D 의 zone_detector (노드 하나가 구역 전부를 본다).
        self.node.create_subscription(
            BoxAtPickup, f"/zone_{self.zone}/box_at_pickup",
            self.on_box_arrived, 10, callback_group=self.cbg)

        # ── 구독: 신원 대역 (PL_GTPICK=2 일 때만 씬이 낸다) ──
        # ★ 풀 비전에서 존 인식이 QR 을 못 읽어 box_id 가 빈 채 오면 (사양상
        #   픽업 카메라 몫이 아니다 — §5 는 "픽업 좌표 실측"만), 씬 실측이
        #   /zone_*/box_ident 로 낸 신원을 붙여 준다 (2026-08-27 김병직 설계).
        #   대역 토픽이 없으면 이 구독은 조용히 놀고 기존 동작 그대로다.
        self._ident = None               # (box_id, 받은 시각) — 스토퍼 박스 신원
        self.node.create_subscription(
            BoxAtPickup, f"/zone_{self.zone}/box_ident",
            self._on_ident, 10, callback_group=self.cbg)

        # ── 구독: 로봇 결과 ──
        # ★ 로봇 토픽도 구역별이다: /zone_a/robot/cmd · /zone_b/... · /zone_c/...
        #   RobotCmd 에는 구역 칸이 없다. 구역이 셋인데 토픽이 하나면 A 구역 명령을
        #   C 구역 로봇도 받아 세 대가 동시에 같은 자리로 간다 — 충돌한다.
        #   msg 를 고치면 팀 전원이 재빌드해야 하므로 토픽 이름으로 가른다.
        self.node.create_subscription(
            RobotEvent, f"/zone_{self.zone}/robot/event",
            self.on_robot_event, 10, callback_group=self.cbg)

        # ── 발행: 로봇 명령 ──
        self.pub_robot = self.node.create_publisher(
            RobotCmd, f"/zone_{self.zone}/robot/cmd", 10)

        # ── 발행: 구역 상태 (모니터링용, 2Hz) ──
        self.pub_status = self.node.create_publisher(ZoneStatus, f"/zone_{self.zone}/status", 10)
        self.node.create_timer(0.5, self.publish_status, callback_group=self.cbg)

        # ── 서비스 클라이언트 8개 ──
        # ★ callback_group=self.cbg 를 반드시 준다.
        #   이걸 빼면 응답 콜백이 기본 그룹(상호배제)에 묶인다. 그러면 구독 콜백
        #   안에서 부른 서비스의 답이 영원히 안 온다 — 에러 한 줄 없이 조용히 멈춘다.
        # ※ create_client 는 서버가 없어도 그냥 성공한다. 여기서는 기다리지 않는다.
        #   (기다리면 노드 시작이 8번 × 2초 = 16초 늦어진다)
        # ★ 클라이언트는 노드가 하나만 만들어 구역이 나눠 쓴다. 구역마다 새로
        #   만들면 같은 서비스에 클라이언트가 3 벌씩 붙는다.
        self.cli = node.cli

        # ── 진행 중인 서비스 호출 장부 ──
        # 호출마다 타이머를 새로 만들지 않는 이유: 타이머 콜백 안에서 자기 자신을
        # destroy_timer 하는 건 위험하다. 대신 감시 타이머 하나가 장부를 훑는다.
        self._lock = threading.Lock()     # 여러 스레드가 동시에 장부를 만진다
        self._pending = {}                # call_id → {srv, deadline, on_response, future}
        self._next_call_id = 0
        self.node.create_timer(TIMEOUT_SCAN_S, self._check_timeouts, callback_group=self.cbg)

        self.log.info("사이클 준비 — 상태: IDLE")

    # ══════════════════════════════════════════════════════════
    # ① IDLE → ② FETCHING : 박스가 스토퍼에 도착했다
    # ══════════════════════════════════════════════════════════
    def _on_ident(self, msg):
        """신원 대역 수신 — 스토퍼 박스의 box_id 만 기억한다 (좌표는 안 쓴다).

        (box_id, 최근 수신, 최초 수신) 3짝이다. 최초 수신은 check_stuck_box 의
        "비전 침묵" 감시가 이 박스가 언제부터 서 있었는지 재는 데 쓴다."""
        if msg.present and msg.box_id:
            now = time.time()
            if self._ident is not None and self._ident[0] == msg.box_id:
                self._ident = (msg.box_id, now, self._ident[2])
            else:
                self._ident = (msg.box_id, now, now)
        elif not msg.present:
            self._ident = None           # 스토퍼가 비었다 — 낡은 신원을 안 붙인다

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
            # 스토퍼가 비었다 = 씬이 정말로 치웠다. 이제 같은 id 가 다시 오면
            # 그건 진짜 새 박스다 (재순환). 빗장을 푼다.
            self._rejected_id = None
            self._done_id = None
            self._stuck_id = self._stuck_since = None
            return

        # ── ①-b 이 박스를 지금 처리하지 못한다면 언제부터인지 기록한다 ──
        #   아래 어느 관문에 걸려 return 하든 스토퍼에는 박스가 남는다.
        if self._stuck_since is None or msg.box_id != self._stuck_id:
            self._stuck_id, self._stuck_since = msg.box_id, time.time()

        # ── ② box_id 가 비었으면 아무것도 못 한다 ──
        # 다음 단계가 /db/get_box_info(box_id) 인데 빈 문자열로는 조회가 안 된다.
        # detector 가 색 → box_id 매칭에 실패하면 이렇게 온다.
        if not msg.box_id:
            # ★ 신원 대역 (2026-08-27): /zone_*/box_ident 가 방금(3초 안) 준
            #   스토퍼 박스 신원이 있으면 그걸 붙인다 — 좌표는 카메라 실측 그대로.
            _id = self._ident
            if _id is not None and time.time() - _id[1] < 3.0:
                msg.box_id = _id[0]
                self.log.info("box_id 를 신원 대역에서 보충: %s" % msg.box_id,
                              throttle_duration_sec=5.0)
            else:
                # ★ once 로 억누르지 않는다. 이 상태가 이어지면 라인이 막히는데
                #   경고가 한 번만 뜨면 조용히 선 것처럼 보인다.
                self.log.warn("box_at_pickup 에 box_id 가 비어 있다 — 무시",
                              throttle_duration_sec=5.0)
                return

        # ── ②-b 방금 버렸거나 방금 다 쌓은 박스면 씬이 치울 때까지 건드리지 않는다 ──
        if msg.box_id in (self._rejected_id, self._done_id):
            return

        # ── ③ 좌표는 이미 mm 정수다 ──
        # ★ BoxAtPickup 은 mm 정수로 온다 (CLAUDE.md §3-2). 예전엔 여기서
        #   Pose(m) → mm 변환과 쿼터니언 → yaw 추출을 했는데, 그 두 단계가
        #   통째로 없어졌다. 발행자(pick_cam)가 이미 mm 로 바꿔 보낸다.
        self.box = {
            "box_id":   msg.box_id,
            "x_mm":     int(msg.x_mm),
            "y_mm":     int(msg.y_mm),
            "yaw_deg":  float(msg.yaw_deg),
        }

        # ── ④ 단위 점검 (경고만) ──
        # 스토퍼는 x = 10000 mm 다 (CLAUDE.md §4).
        # 100m 를 넘으면 발행자가 mm 가 아니라 µm 나 다른 단위를 담은 것이다.
        # 막지는 않는다 — 막으면 진짜 원인이 안 보인다. 대신 크게 남긴다.
        if abs(self.box["x_mm"]) > 100_000:
            self.log.warn(
                f"x={self.box['x_mm']}mm — 스토퍼는 10000mm 다. "
                f"발행자가 단위를 잘못 담고 있지 않은지 확인할 것 (계약은 mm 정수)")

        self.log.info(
            f"[①→②] 박스 도착: {self.box['box_id']} "
            f"({self.box['x_mm']}, {self.box['y_mm']}) yaw {self.box['yaw_deg']:.1f}°")
        self.state = FETCHING

        # 벨트를 먼저 세운다. 안 세우고 흡착하면 박스가 끌려간다.
        self.call_set_velocity(0.0, then=self.fetch_box_info)

    # ══════════════════════════════════════════════════════════
    # ② FETCHING : DB에 "이 박스 크기 뭐야?" 물어본다
    # ══════════════════════════════════════════════════════════
    def fetch_box_info(self):
        # ★ 2026-08-26 — 응답이 올 때쯤 self.box 가 비어 있을 수 있다 (경합).
        box = self.box
        if box is None:
            self.log.warn("조회할 박스가 없다 — 대기로 복귀")
            self.reset_to_idle()
            return
        box_id = box["box_id"]

        def on_response(future):
            res = future.result()
            if not res.found:
                # DB에 없는 박스 = 인식이 실패했다. 집지 않고 버린다.
                self.log.warn(f"DB에 없음: {box_id} → 폐기")
                self.discard_box("NOT_IN_DB")
                return
            if self.box is None:          # 그새 손을 뗐다
                return
            self.box.update({
                "w": res.info.size_w_mm,
                "d": res.info.size_d_mm,
                "h": res.info.size_h_mm,
            })
            self.log.info(
                f"[②→③] 크기 {res.info.size_w_mm}×{res.info.size_d_mm}×{res.info.size_h_mm}")
            self.state = PLANNING
            self.request_plan()

        self.call_async("/db/get_box_info", {"box_id": box_id}, on_response)

    # ══════════════════════════════════════════════════════════
    # ③ PLANNING : algo에 "어디 놓을까?" 물어본다
    # ══════════════════════════════════════════════════════════
    def request_plan(self):
        def on_response(future):
            res = future.result()

            # ── 이탈 경로 A: 팔레트 만재 → 시나리오 종료 ──
            if res.message == "PALLET_FULL":
                # ★★ 2026-08-26 — 여기서 **끝내지 않는다.**
                #   §7 사양 경로가 "PALLET_FULL -> 트럭 상차 -> 팔레트 교체" 인데
                #   관제탑이 그냥 DONE 으로 갔다. 서버(/sim/swap_pallet ·
                #   /sim/set_spawn_enabled · /algo/reset_heightmap)는 셋 다 있었고
                #   **부르는 쪽이 없었을 뿐**이다 (계약 감사 2026-08-26).
                #   PL_MAX_PALLETS 판을 채우면 그때 종료한다.
                self.pallets_done += 1
                if self.pallets_done >= self.max_pallets:
                    self.log.info("PALLET_FULL — %d 판째, 시나리오 종료"
                                  % self.pallets_done)
                    self.finish()
                else:
                    self.log.info("PALLET_FULL — %d 판째. 팔레트를 간다"
                                  % self.pallets_done)
                    self.swap_pallet()
                return

            # ── 이탈 경로 B: 이 박스만 자리 없음 → 박스만 버리고 계속 ──
            # ★ 여기서 끝내면 안 된다. 다음 박스는 들어갈 수 있다.
            if res.message == "NO_FEASIBLE_POSE" or not res.ok:
                _b = self.box
                # ★ 연속 폐기 가드 (2026-08-27 실측). A존이 62%/1260mm 에서
                #   **가장 작은 3호까지 19연속 NO_FEASIBLE** 인데 PALLET_FULL 은
                #   안 나왔다 — algo 의 만재 판정은 높이만 보고, 지지율·통로가
                #   다 막은 상태는 못 본다. 그 사이 오는 박스를 전부 버리며
                #   존이 사실상 죽는다. 서로 다른 박스가 이만큼 연속으로
                #   거부되면 만재로 간주하고 판갈이 경로를 탄다.
                self._nfp_streak = getattr(self, "_nfp_streak", 0) + 1
                if self._nfp_streak >= int(os.environ.get("PL_NFP_FULL", "6")):
                    self.log.warn("NO_FEASIBLE %d연속 — 사실상 만재. 판갈이"
                                  % self._nfp_streak)
                    self._nfp_streak = 0
                    self.pallets_done += 1
                    if self.pallets_done >= self.max_pallets:
                        self.log.info("PALLET_FULL(연속폐기) — %d 판째, 시나리오 종료"
                                      % self.pallets_done)
                        self.finish()      # 정상 만재 경로와 동일 (박스는 안 버린다)
                    else:
                        self.log.info("PALLET_FULL(연속폐기) — %d 판째. 팔레트를 간다"
                                      % self.pallets_done)
                        self.swap_pallet()
                    return
                self.log.info("NO_FEASIBLE_POSE: %s → 폐기 (연속 %d)"
                              % (_b["box_id"] if _b else "?", self._nfp_streak))
                self.discard_box("NO_FEASIBLE_POSE")
                return
            self._nfp_streak = 0           # 배치가 계획되면 연속 카운트 리셋

            self.plan = res.pose
            self.log.info(
                f"[③→④] 놓을 자리 ({res.pose.x_mm}, {res.pose.y_mm}, {res.pose.z_mm})")
            self.state = PICKING
            self.t_robot = time.time()      # 집기 구간 타임아웃 시작
            self.send_robot_cmd(RobotCmd.PICK)

        # ★ 같은 이유로 여기서도 한 번만 읽는다 (2026-08-26).
        box = self.box
        if box is None:
            self.log.warn("계획을 요청할 박스가 없다 — 대기로 복귀")
            self.reset_to_idle()
            return
        self.call_async("/algo/plan_placement", {
            "zone": self.zone_id,
            "box_id": box["box_id"],
            "size_w_mm": box["w"],
            "size_d_mm": box["d"],
            "size_h_mm": box["h"],
        }, on_response)

    # ══════════════════════════════════════════════════════════
    # ④⑤ PICKING / PLACING : 로봇에게 명령하고 결과를 기다린다
    # ══════════════════════════════════════════════════════════
    def send_robot_cmd(self, action):
        """로봇에게 명령 하나를 던진다. 결과는 /robot/event 로 돌아온다.

        ★ 보낼 때마다 seq 를 올린다. 로봇이 그 seq 를 되돌려줘야
          늦게 온 지난 명령의 결과와 구분할 수 있다.
        """
        # ★ 2026-08-26 — self.box 를 **한 번만** 읽는다. 이 메서드는 콜백 안에서
        #   불리는데, 그동안 타임아웃 타이머가 self.box 를 비울 수 있다.
        #   HOME/STOP 은 박스가 없어도 보내야 하므로 그 둘만 통과시킨다.
        box = self.box
        if box is None and action in (RobotCmd.PICK, RobotCmd.PLACE):
            self.log.warn("로봇 명령을 낼 박스가 없다 — 보내지 않는다")
            return
        self.seq += 1
        cmd = RobotCmd()
        cmd.action = action
        cmd.seq = self.seq

        if action == RobotCmd.PICK:
            cmd.box_id = box["box_id"]
            cmd.x_mm = box["x_mm"]
            cmd.y_mm = box["y_mm"]
            # ★ z_mm 은 **박스 밑면**이다 (RobotCmd.msg 정본 · PLACE 도 밑면을 보낸다).
            #   벨트 위에 놓인 박스의 밑면 = 벨트 상면이다.
            #   ⚠ 2026-08-26 수정 (김병직): 여기가 `800 + h`(=박스 **상면**)였다.
            #   한 필드가 PICK 에서는 상면, PLACE 에서는 밑면이라 뜻이 갈렸다.
            #   로봇이 정본대로 읽으면 중심을 h/2 만큼 높게 잡아 3호 기준 **210mm 위**
            #   허공으로 내려간다 — CLAUDE.md §10 "박스를 400mm 위에서 놓는다"와 같은 부류.
            #   숫자는 system.yaml 의 conveyor.belt_top_z_mm 이 정본이다.
            cmd.z_mm = self.belt_top_mm
            cmd.yaw_deg = float(box["yaw_deg"])
            cmd.size_w_mm = box["w"]
            cmd.size_d_mm = box["d"]
            cmd.size_h_mm = box["h"]

        elif action == RobotCmd.PLACE:
            cmd.box_id = box["box_id"]
            cmd.x_mm = self.plan.x_mm
            cmd.y_mm = self.plan.y_mm
            cmd.z_mm = self.plan.z_mm   # ★ 밑면 기준. 중심으로 쓰면 팔레트에 파묻힌다
            cmd.yaw_deg = float(self.plan.yaw_deg)
            cmd.size_w_mm = box["w"]
            cmd.size_d_mm = box["d"]
            cmd.size_h_mm = box["h"]

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
        # ★★ 2026-08-26 — **action 을 본다.** 아래 분기가 `ev.action` 이 아니라
        #   `self.state` 로 갈리기 때문에, PLACING 중에 **PICK 이벤트가 다시 오면
        #   그걸 "놓기 성공" 으로 처리한다.** Isaac 의 EvtPub 은 새 이벤트가 없어도
        #   마지막 값을 다시 쏘는 구조라 이게 실제로 일어났다 (실측 2026-08-26):
        #     update[B] BOX005 가 3 번 · db placed BOX005 가 2 번 ·
        #     algo 가 "이미 반영된 박스다 — 재시도로 보고 무시했다" 로 막아 줬다.
        #   팔이 아직 박스를 들고 있는데 사이클이 끝난 것으로 처리되니
        #   **집자마자 멈춘 것처럼 보인다.**
        want = {PICKING: RobotCmd.PICK, PLACING: RobotCmd.PLACE}.get(self.state)
        if want is not None and ev.action != want:
            self.log.warn(
                f"{self.state} 인데 action={ev.action} 이벤트가 왔다 — 무시",
                throttle_duration_sec=5.0)
            return

        # ★ 같은 (seq, action) 을 두 번 처리하지 않는다. seq 검사만으로는 재방송을
        #   못 막는다 — 재방송은 seq 가 **같아서** 그 검사를 그대로 통과한다.
        key = (int(ev.seq), int(ev.action))
        if key == self._last_ev:
            return
        self._last_ev = key

        if ev.seq != self.seq:
            # ★ throttle 을 건다. Isaac 의 EvtPub 은 새 이벤트가 없어도 마지막 값을
            #   다시 쏘는 구조라(ros_graph `_event_gate_script` 주석), 이 경고가
            #   정상 동작 중에도 반복된다. 안 누르면 로그가 통째로 덮인다.
            self.log.warn(
                f"지난 명령의 결과 무시: seq {ev.seq} (지금은 {self.seq})",
                throttle_duration_sec=5.0)
            return

        # 처리 중이 아닐 때 온 것도 버린다.
        # (self.box 가 None 인데 ["box_id"] 를 읽으면 노드가 죽는다)
        #
        # ★★ 2026-08-26 — **여기서 지역 변수로 붙잡는다.** 검사만으로는 부족했다.
        #   MultiThreadedExecutor + ReentrantCallbackGroup 이라 이 콜백이 도는 도중에
        #   0.5 초 상태 타이머의 `check_robot_timeout()` 이 `reset_to_idle()` 을 불러
        #   `self.box` 를 None 으로 만든다. 검사는 통과했는데 **일곱 줄 뒤에서** 죽었다:
        #       TypeError: 'NoneType' object is not subscriptable   (실측: ABORTED 직후)
        #   관제탑이 죽으면 `/robot/cmd` 를 낼 사람이 없어져 **라인 전체가 멈춘다.**
        #   아래에서는 self.box 를 다시 읽지 않는다 — 한 번 잡은 값만 쓴다.
        box = self.box
        if self.state not in (PICKING, PLACING) or box is None:
            return
        box_id = box["box_id"]

        # ── 이탈 경로 C: 로봇 실패 ──
        if not ev.ok:
            self.log.warn(f"로봇 실패({self.state}): {ev.reason}")
            # ★ 같은 박스 재시도 상한 (2026-08-27 실측). 스토퍼에서 뒤차가 앞차
            #   위로 올라타 2단이 되면(패드가 정확히 810+210+210=1226 에서 막힘)
            #   로봇은 아래 박스를 영원히 못 집는데, 관제탑은 같은 박스를 계속
            #   재시도했다 — B존이 잔차 226mm 실패를 **100회 넘게** 반복하며 섰다.
            #   3회 실패하면 그 박스를 치운다(despawn) — 위에 얹힌 박스가 스토퍼로
            #   떨어져 다음 픽 대상이 되므로 자가 회복된다.
            _fb, _fc = getattr(self, "_fail_box", (None, 0))
            _fc = _fc + 1 if _fb == box_id else 1
            self._fail_box = (box_id, _fc)
            if _fc >= int(os.environ.get("PL_PICK_RETRY_MAX", "3")):
                self.log.warn("%s %d회 연속 실패 — 집을 수 없는 박스로 보고 치운다"
                              % (box_id, _fc))
                self._fail_box = (None, 0)
                self.call_async("/algo/cancel_plan",
                                {"zone": self.zone_id, "box_id": box_id}, None)
                # ★ 팔을 홈으로 치운다 (2026-08-27). 실패한 팔이 스토퍼 위에 떠
                #   있으면 풀 비전에서 다음 박스가 카메라에 안 보인다. 재시도
                #   중에는 안 보낸다 — 어차피 같은 자리로 다시 내려간다.
                self.send_robot_cmd_home()
                self.discard_box("PICK_FAILED_x%d" % _fc)
                return
            self.call_async("/db/set_status",
                            {"box_id": box_id, "status": "PICK_FAILED",
                             "reason": ev.reason}, None)
            # ★★ 픽 실패는 **재인식 없이 바로 재시도한다** (2026-08-27 김병직 설계).
            #   스토퍼에서 한 번 인식했으면 좌표·신원·계획을 이미 쥐고 있고,
            #   흡착이 빗나가도 박스는 그 자리 그대로다 — 새 box_at_pickup 을
            #   기다리는 건 불필요할 뿐 아니라, 풀 비전에서는 실패한 팔이
            #   카메라를 가려 재인식이 **영영 안 와** 존이 죽는다 (실측 B·C).
            #   계획(self.plan)도 살아 있으므로 cancel_plan 도 부르지 않는다.
            if self.state == PICKING and self.box is not None:
                self.log.warn("픽 재시도 %d회째 — 같은 좌표로 바로 다시" % (_fc + 1))
                self.t_robot = time.time()
                self.send_robot_cmd(RobotCmd.PICK)
                return
            # ── PLACING 실패: 박스가 십중팔구 패드에 매달려 있다 ──
            # ★ 2026-08-27 실측 (A존): BOX054 를 2층에 놓다가 142mm 위에서 걸려
            #   (5kg 낙하 불가) ABORTED 났는데, 여기가 reset_to_idle 만 하고
            #   박스를 안 치웠다 — 문 채 IDLE 로 돌아가 이후 모든 PICK 이
            #   매달린 박스(높이 280)에 막혀 잔차 306mm ABORTED 폭풍, 존 영구
            #   고장. 치운다(despawn 이 sim 쪽에서 그리퍼 개방까지 한다) —
            #   박스를 이미 떨어뜨린 경우(DROPPED)도 같은 처리가 맞다:
            #   어디 떨어졌는지 모르는 박스는 라인에서 빼는 것뿐이다.
            self._fail_box = (None, 0)
            self.send_robot_cmd_home()
            self.discard_box("PLACE_FAILED:%s" % ev.reason)
            return

        if self.state == PICKING:
            self.log.info(f"[④→⑤] 집기 성공 ({ev.duration_s:.1f}초)")
            self.call_async("/db/set_status",
                            {"box_id": box_id, "status": "PICKED",
                             "reason": ""}, None)
            self.state = PLACING
            self.t_robot = time.time()          # 놓기 구간 타임아웃 다시 시작
            self.send_robot_cmd(RobotCmd.PLACE)

        elif self.state == PLACING:
            self.log.info(f"[⑤→⑥] 놓기 성공 ({ev.duration_s:.1f}초)")
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
            self.log.warn(
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

        _b = self.box                        # ★ 두 번 읽으면 그 사이에 비워진다
        bid = _b["box_id"] if _b else ""
        self.log.error(
            f"로봇 응답 없음 — {self.state} 에서 {held:.1f}초 (제한 {ROBOT_TIMEOUT_S}초). 계획을 버린다")
        self.t_robot = None
        self.call_async("/algo/cancel_plan", {"zone": self.zone_id, "box_id": bid}, None)
        self.call_async("/db/set_status",
                        {"box_id": bid, "status": "PICK_FAILED", "reason": "TIMEOUT"}, None)
        self.reset_to_idle()

    # ══════════════════════════════════════════════════════════
    # ⑥ UPDATING : algo에 "여기 놨다" 알려준다
    # ══════════════════════════════════════════════════════════
    def update_heightmap(self, actual_pose):
        # ★★ 2026-08-26 — 박스 정보를 **지금** 붙잡는다. 아래 `on_response` 는
        #   서비스 왕복 뒤에 도는데, 그 사이 타임아웃 타이머가 `reset_to_idle()` 로
        #   `self.box` 를 비울 수 있다 (on_robot_event 에서 실제로 죽었다).
        #   콜백 안에서 self.box 를 다시 읽으면 같은 자리에서 노드가 죽는다.
        box = self.box
        if box is None:
            self.log.warn("update_heightmap 인데 처리 중인 박스가 없다 — 건너뛴다")
            return
        box_id, bw, bd, bh = box["box_id"], box["w"], box["d"], box["h"]

        def on_response(future):
            res = future.result()
            # ★★ 배치 이상 연속 가드 (2026-08-27 실측, A존 전복).
            #   `ok=false` 는 "박스가 계획대로 안 앉았다" 는 algo 의 자백이다
            #   (yaw 가 허용을 넘었거나 팔레트 가장자리에 걸쳤거나).
            #   실측 로그가 보여준 붕괴 경로:
            #     BOX115 계획 (525,130) yaw 0 → 실측 yaw -37.4도  ok=false
            #     BOX128 계획 **같은 자리 (525,130)** → 실측 (861,123) 가장자리 ok=false
            #     → 33번째 박스가 1260mm 꼭대기에서 넘어졌다.
            #   앞 박스가 틀어진 걸 알고도 그 자리를 다시 내준 것이 원인이다.
            #   지형이 어긋난 위에 계속 쌓으면 무너지므로, 연속으로 어긋나면
            #   **더 쌓지 않고 판을 간다.** 지지율·통로를 손대는 것보다 안전하고,
            #   한 번만 어긋난 경우는 예전처럼 그냥 진행한다 (기본 임계 2).
            #   ⚠ 박스는 이미 물리적으로 놓였다 — placed·mark_placed 는 그대로 한다.
            if not res.ok:
                self._bad_streak = getattr(self, "_bad_streak", 0) + 1
                self.log.warn("배치 이상(ok=false) — 연속 %d" % self._bad_streak)
            else:
                self._bad_streak = 0
            self.placed += 1
            self.fill_rate = res.fill_rate
            self.max_height_mm = res.max_height_mm
            self.log.info(
                f"[⑥→①] 적재 {self.placed}개 · 충전율 {res.fill_rate:.1%}"
                f" · 최고 {res.max_height_mm}mm")
            # ★ MarkPlaced 는 zone 과 actual_pose 를 받는다. 계획 pose 가 아니다.
            self.call_async("/db/mark_placed",
                            {"box_id": box_id, "zone": self.zone_id,
                             "actual_pose": actual_pose}, None)
            # ★ 이 박스는 끝났다. IDLE 로 돌아간 직후에도 detector 가 보내 둔
            #   이전 프레임이 도착하므로, present=false 를 볼 때까지 빗장을 건다.
            self._done_id = box_id
            if getattr(self, "_bad_streak", 0) >= int(
                    os.environ.get("PL_BAD_FULL", "2")):
                self.log.warn("배치 이상 %d연속 — 지형이 어긋났다. 더 쌓지 않고 판갈이"
                              % self._bad_streak)
                self._bad_streak = 0
                self._nfp_streak = 0
                self.pallets_done += 1
                if self.pallets_done >= self.max_pallets:
                    self.log.info("PALLET_FULL(배치이상) — %d 판째, 시나리오 종료"
                                  % self.pallets_done)
                    self.finish()
                else:
                    self.log.info("PALLET_FULL(배치이상) — %d 판째. 팔레트를 간다"
                                  % self.pallets_done)
                    self.swap_pallet()   # box 를 비우고 IDLE 까지 스스로 잇는다
                return
            self.reset_to_idle()

        self.call_async("/algo/update_heightmap", {
            "zone": self.zone_id,
            "box_id": box_id,
            "size_w_mm": bw,
            "size_d_mm": bd,
            "size_h_mm": bh,
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
        # ★ None 을 견딘다. 같은 경합으로 여기서도 죽을 수 있다 (2026-08-26).
        _b = self.box                        # ★ 두 번 읽으면 그 사이에 비워진다
        bid = _b["box_id"] if _b else ""
        if not bid:
            self.log.warn("discard_box 인데 처리 중인 박스가 없다 — 상태만 되돌린다")
            self.reset_to_idle()
            return
        self.rejected += 1
        self._rejected_id = bid          # 씬이 치울 때까지 다시 안 잡는다
        self.call_async("/algo/cancel_plan", {"zone": self.zone_id, "box_id": bid}, None)
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
                self.belt_mps, then=None)
            self.log.info("[→①] 대기")
        finally:
            self._resetting = False

    def swap_pallet(self):
        """PALLET_FULL — 사양 경로대로 판을 간다. **노드는 안 죽는다.**

            스폰 정지 -> swap_pallet -> algo reset -> 스폰 재개 -> 벨트 재가동 -> IDLE

        ★ 네 호출을 **콜백으로 잇는다.** 하나라도 동기로 기다리면 executor 가
          자기 자신을 기다려 에러 없이 멎는다 (CLAUDE.md §9).
        ★ 중간에 실패해도 IDLE 로 돌아간다. 판을 못 갈았을 뿐이고, 다음 박스에서
          다시 PALLET_FULL 이 나면 또 시도한다 — 라인을 세우는 것보다 낫다.
        ★ 스폰 정지는 **판이 없는 그 한 순간**만이다. 반출까지 껴서 길게 끄면
          반출 안 하는 다른 존까지 굶는다 (트랙 A `c11a500` 과 같은 교훈).
        """
        self.state = UPDATING          # 새 박스를 안 받는 상태로 둔다

        def back_to_idle(_tag):
            self.log.info("팔레트 교체 완료 — 대기로 복귀")
            self.box = None
            self.state = IDLE
            self.call_set_velocity(self.belt_mps, then=None)

        def on_spawn_on(fut):
            back_to_idle("spawn_on")

        def on_reset(fut):
            res = fut.result()
            if not getattr(res, "ok", False):
                self.log.error("algo reset_heightmap 실패 — 그래도 계속한다")
            # heightmap 을 비웠으니 우리 지표도 같이 비운다. 안 그러면 다음 판의
            # 충전율이 이전 판 값에서 이어져 보인다.
            self.placed = 0
            self.fill_rate = 0.0
            self.max_height_mm = 0
            self.call_async("/sim/set_spawn_enabled", {"enabled": True}, on_spawn_on)

        def on_swap(fut):
            res = fut.result()
            if not getattr(res, "ok", False):
                self.log.error("swap_pallet 실패 — 그래도 heightmap 은 비운다")
            # ★ zone 은 **대문자**(zone_id)로 보낸다 (2026-08-27 실측 사고).
            #   algo 의 packer 표와 스포너의 존 목록이 대문자 키다. 소문자 'b' 를
            #   보냈더니 reset 이 아무것도 안 비우고 ok=False 만 돌아왔고, 하이트맵이
            #   가득 찬 채 재계획 -> 즉시 또 PALLET_FULL -> **0.6초에 3판을 태우고
            #   시나리오가 끝났다** (plan 은 원래 zone_id 를 보내고 있어서 멀쩡했다).
            self.call_async("/algo/reset_heightmap", {"zone": self.zone_id}, on_reset)

        def on_spawn_off(fut):
            self.call_async("/sim/swap_pallet", {"zone": self.zone_id}, on_swap)

        self.call_async("/sim/set_spawn_enabled", {"enabled": False}, on_spawn_off)

    def finish(self):
        """PALLET_FULL — 최종 지표를 찍고 멈춘다."""
        self.state = DONE
        self.call_set_velocity(0.0, then=None)
        self.send_robot_cmd_home()
        elapsed = time.time() - self.t_start
        self.log.info(
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
        self.check_stuck_box()

        msg = ZoneStatus()
        msg.zone = self.zone
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
        msg.stamp = self.node.get_clock().now().to_msg()
        self.pub_status.publish(msg)

    def check_stuck_box(self):
        """IDLE 인데 스토퍼에 박스가 오래 서 있으면 치운다.

        ★ 안 치우면 뒤 박스가 밀려 라인이 통째로 막힌다 (CLAUDE.md §7).
          로봇 타임아웃은 PICKING/PLACING 만 보므로 이 구간은 감시가 없었다.

        ★ box_id 를 아는 경우에만 치울 수 있다 — DespawnBox 가 id 로 받는다.
          픽업에서도 QR 을 못 읽어 id 가 빈 경우는 씬 쪽 개입이 필요하다.
          그때는 **에러를 계속 낸다.** 조용히 서 있는 것이 제일 나쁘다.
        """
        if self.state != IDLE:
            return
        if self._stuck_since is None:
            # ★ 비전이 아예 침묵하는 경우 (2026-08-27 강의실 실측): 두 박스가 붙어
            #   스토퍼에 오면 실루엣이 병합돼(857x712 등) 사양 3종 어느 것과도 안
            #   맞고, detector 는 box_at_pickup 을 **한 번도 안 낸다** — 위의
            #   on_box_arrived 무장이 영영 안 된다. 점유 제어는 스토퍼가 찼다고
            #   지선을 세워 두므로 존 전체가 조용히 죽는다 (B존 실측).
            #   신원 대역(/zone_*/box_ident)은 씬 실물 기준이라 이 상태를 안다.
            ident = self._ident
            if (ident is not None
                    and time.time() - ident[1] < 3.0            # 지금도 서 있고
                    and time.time() - ident[2] >= STUCK_TIMEOUT_S):  # 오래 서 있다
                self.log.error(
                    f"{ident[0]} 가 스토퍼에 {STUCK_TIMEOUT_S:.0f}초째 서 있는데 "
                    "인식이 침묵한다 (실루엣 병합 의심) — 치워서 라인을 푼다")
                self._ident = None
                self.box = {"box_id": ident[0]}
                self.discard_box("STUCK_UNSEEN")
            return
        if time.time() - self._stuck_since < STUCK_TIMEOUT_S:
            return
        bid = self._stuck_id
        if not bid:
            self.log.error(
                f"스토퍼에 박스가 {STUCK_TIMEOUT_S:.0f}초째 서 있는데 식별이 안 된다 "
                "— 씬에서 치워야 라인이 풀린다 (QR·조명 확인)",
                throttle_duration_sec=10.0)
            return
        self.log.error(f"{bid} 가 {STUCK_TIMEOUT_S:.0f}초째 처리되지 않는다 — 치운다")
        self._stuck_id = self._stuck_since = None
        self.box = {"box_id": bid}
        self.discard_box("STUCK_AT_STOPPER")

    # ── 서비스 호출 헬퍼 ──────────────────────────────────────
    def call_set_velocity(self, mps, then):
        """벨트 속도 변경. 응답 ok를 확인한 뒤 다음 단계로 간다."""
        def on_response(future):
            res = future.result()
            if not res.ok:
                # ★ 벨트가 안 멈춘 채 흡착하면 박스가 끌려간다. 다음 단계로 안 넘어간다.
                self.log.error(
                    f"벨트 속도 변경 실패 — 요청 {mps} m/s / 실제 {res.actual_mps} m/s")
                # then 이 있는 쪽(= 집기 전 정지)만 되돌린다.
                # then 이 없는 쪽은 reset_to_idle 안의 재가동이라,
                # 여기서 또 reset 을 부르면 서로 계속 부르는 고리가 된다.
                if then is not None:
                    self.reset_to_idle()
                return
            if then:
                then()
        # ★ 2026-08-26 수정 (김병직): seg 가 "b_pickup" 고정이었다.
        #   ZoneCycle 은 구역별 인스턴스인데 벨트만 B 를 가리켜서, stage 2 에서
        #   **A 구역 사이클이 B 구역 벨트를 멈춘다.** 이 파일 상단 Orchestrator
        #   주석이 경고한 그 위험("A 구역 사이클이 C 구역 값을 읽는다")과 같은 종류다.
        #   구역 이름 규약은 packing_robot/cycle.py 의 belt_seg 와 같다.
        self.call_async("/conveyor/set_velocity",
                        {"seg": f"{self.zone}_pickup", "velocity_mps": mps}, on_response)

    def call_async(self, srv_name, req_dict, on_response):
        """서비스 1회 호출. ★ call_async + add_done_callback 만 쓴다.

        spin_until_future_complete 를 쓰면 executor 가 자기 자신을 기다린다.
        에러도 안 나고 그냥 멈춘다 — 가장 찾기 어려운 고장이다.

        on_response=None 이면 "보내고 잊는" 호출이다(로그·기록용).
        이런 호출은 실패해도 상태기계를 건드리지 않는다.
        """
        cli = self.cli.get(srv_name)
        if cli is None:
            self.log.error(f"모르는 서비스: {srv_name} — SERVICES 표에 없다")
            return

        # ── 서버가 떠 있나 ──
        # 먼저 service_is_ready() 로 물어보고, 아닐 때만 2초 기다린다.
        # wait_for_service 는 executor 가 아니라 rclpy 의 별도 graph 스레드가
        # 처리하므로 콜백 안에서 불러도 데드락은 안 난다. 다만 그 2초 동안
        # 이 스레드는 멈추므로 꼭 필요할 때만 쓴다.
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=WAIT_SERVER_S):
            self.log.error(
                f"서비스 없음: {srv_name} — {WAIT_SERVER_S}초 기다렸지만 서버가 안 떴다. 대기로 복귀")
            self.reset_to_idle()     # ★ 노드는 죽이지 않는다. 다음 박스 때 다시 시도한다
            return

        # ── dict → ROS 요청 객체 ──
        try:
            req = self._fill_request(cli.srv_type.Request(), req_dict)
        except Exception as e:
            self.log.error(f"{srv_name} 요청 변환 실패: {e}")
            self.reset_to_idle()
            return

        # 장부에 먼저 올리고 나서 보낸다.
        # 순서를 바꾸면 응답이 너무 빨리 와서 장부에 없는 call_id 를 찾게 된다.
        with self._lock:
            call_id = self._next_call_id
            self._next_call_id += 1
            _to = CALL_TIMEOUT_BY_SRV.get(srv_name, CALL_TIMEOUT_S)
            call = {"srv": srv_name, "on_response": on_response,
                    "deadline": time.time() + _to, "future": None}
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
                self.log.warn(f"요청에 없는 필드 무시: {key} (있는 필드: {list(types)})")
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
            self.log.error(f"{srv_name} 호출 실패: {e}")
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
            self.log.error(f"{srv_name} 응답 처리 중 예외: {e}")
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

        self.log.error("%s 응답 없음 — %.1f초 초과"
                       % (srv_name, CALL_TIMEOUT_BY_SRV.get(srv_name, CALL_TIMEOUT_S)))

        if call["on_response"] is None:
            return                                   # 보내고 잊는 호출은 상태를 안 건드린다
        if self.state in (IDLE, DONE):
            return                                   # 처리 중인 박스가 없으면 되돌릴 것도 없다

        # ★ 이탈할 때는 계획을 반드시 버린다. 안 버리면 그 자리가 영원히 예약된 채 남는다.
        #   단, 타임아웃 난 게 cancel_plan 자신이면 또 부르지 않는다 (무한 반복).
        _box = self.box                      # ★ 검사와 사용 사이에 비워질 수 있다
        if _box and srv_name != "/algo/cancel_plan":
            self.call_async("/algo/cancel_plan",
                            {"zone": self.zone_id, "box_id": _box["box_id"]}, None)
        self.reset_to_idle()


class Orchestrator(Node):
    """관제탑 — **프로세스도 노드도 하나다** (CLAUDE.md §8).

    ★ /robot/cmd 계열을 내는 유일한 노드다. 구역이 셋이어도 늘리지 않는다.
      늘리면 "로봇 명령의 유일한 발신자" 규약이 깨진다.

    파라미터
      zones               ["b"]   맡을 구역 목록. stage 2 는 ["a","b","c"]
      belt_velocity_mps   0.5     벨트 재가동 속도 (system.yaml 이 정본)

    ★ zone(단수) 도 계속 받는다 — 기존 launch·문서가 그걸 쓴다.
      zones 를 주면 그쪽이 이긴다.
    """

    def __init__(self):
        super().__init__("orchestrator")

        # ReentrantCallbackGroup 하나를 구역들이 나눠 쓴다.
        # 이게 없으면 서비스 응답 콜백이 타이머 콜백을 기다리다 멈춘다.
        self.cbg = ReentrantCallbackGroup()

        self.declare_parameter("zone", "b")
        self.declare_parameter("zones", [""])
        # 벨트 재가동 속도. ★ 숫자를 코드에 박지 않는다.
        #   출처: src/packing_bringup/config/system.yaml · conveyor.belt_velocity_mps
        self.declare_parameter("belt_velocity_mps", 0.5)
        # 벨트 상면 z [mm]. ★ 숫자를 코드에 박지 않는다.
        #   출처: src/packing_bringup/config/system.yaml · conveyor.belt_top_z_mm
        self.declare_parameter("belt_top_z_mm", 800)

        zs = [str(z).strip().lower()
              for z in (self.get_parameter("zones").value or []) if str(z).strip()]
        if not zs:
            zs = [str(self.get_parameter("zone").value).strip().lower()]
        # 중복 제거. 같은 구역을 두 번 맡으면 사이클 둘이 같은 팔레트에 명령을 낸다.
        seen, zones = set(), []
        for z in zs:
            if z and z not in seen:
                seen.add(z)
                zones.append(z)
        if not zones:
            raise ValueError("맡을 구역이 하나도 없다 — zone 또는 zones 를 줘야 한다")

        belt = float(self.get_parameter("belt_velocity_mps").value)
        belt_top = int(self.get_parameter("belt_top_z_mm").value)

        # ★ 서비스 클라이언트는 여기서 한 벌만 만들어 구역들이 나눠 쓴다.
        # ※ create_client 는 서버가 없어도 그냥 성공한다. 기다리지 않는다.
        self.cli = {
            name: self.create_client(srv_type, name, callback_group=self.cbg)
            for name, srv_type in SERVICES.items()
        }

        self.cycles = {z: ZoneCycle(self, z, belt, belt_top) for z in zones}
        self.get_logger().info(
            f"관제탑 시작 — 구역 {', '.join(z.upper() for z in zones)} "
            f"({len(zones)} 개) · 노드는 하나다")


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
