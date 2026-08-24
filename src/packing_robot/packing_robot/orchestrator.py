"""orchestrator — pick & place 상태머신 ★ 초안 (작성: 트랙 C 정정인 · 소유: 트랙 B 김병직).

**Isaac Sim 이 떠 있는 PC(김학민님 PC)에서 실행한다.** 공통가이드 §3 구조상
Isaac 프로세스("몸")와 ROS 프로세스("머리")가 같은 PC 안에 있고, 둘은
localhost:8765 개행구분 JSON 소켓으로 통신하기 때문이다.

두 가지 모드:
  robot_mode:=socket  ★ 실제 로봇을 움직인다. sim_server.py(:8765) 가 떠 있어야 한다.
  robot_mode:=mock       로봇 없이 /algo/* 왕복만 확인. 이동을 시간으로 흉내낸다.

철칙:
- 어떤 콜백에서도 spin_until_future_complete 를 쓰지 않는다. 상태머신을 타이머로 돌리고
  ROS 서비스는 call_async 후 다음 틱에 .done() 을 폴링한다 (§1.5 방법 2).
- move_ee 는 논블로킹이다. 목표만 넣고 motion_status 를 폴링한다 (§3 규칙 1).
- 좌표 변환은 packing_common.frames 만 쓴다 — 트랙 C 와 같은 파일이어야 좌표가 맞는다.
- 모든 이탈 경로에서 /algo/cancel_plan 을 부른다.

실행:
    ros2 run packing_robot orchestrator --ros-args -p robot_mode:=socket
"""
import random

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from packing_common.config import load
from packing_common.frames import PlacePose as PlacePoseData
from packing_common.frames import pallet_place_to_world, world_to_pallet_place
from packing_common.units import m_to_mm, mm_to_m
from packing_msgs.msg import PlacePose, ZoneStatus
from packing_msgs.srv import CancelPlan, PlanPlacement, SetVelocity, UpdateHeightmap
from packing_robot.sim_link import SimLink   # ★ Lock 있는 김병직님 것을 쓴다

TICK_HZ = 20.0


class Orchestrator(Node):
    def __init__(self):
        super().__init__("orchestrator")
        self.cfg = load()
        p = self.declare_parameter
        p("zone", "B"); p("robot_mode", "mock"); p("sim_host", "127.0.0.1"); p("sim_port", 8765)
        p("interval_s", 0.0); p("box_count", 0); p("mock_cycle_s", 1.0); p("place_noise_mm", 3)
        p("real_boxes", True)      # socket 모드에서 Isaac 에 진짜 박스를 스폰해 집는다
        p("pickup_timeout_s", 60.0)
        g = lambda k: self.get_parameter(k).value

        self.zone, self.mode = g("zone"), g("robot_mode")
        st1 = self.cfg["boxes"]["stage1"]
        self.interval = g("interval_s") or st1["spawn_interval_s"]
        self.total = g("box_count") or st1["total_count"]
        self.cycle_s, self.noise = g("mock_cycle_s"), g("place_noise_mm")

        rb_cfg, cv = self.cfg["robot"], self.cfg["conveyor"]
        self.origin = self.cfg["zones"]["pallet_origin_world_mm"][self.zone]
        self.approach = rb_cfg["approach_offset_mm"]
        self.stopper_x, self.belt_z = cv["stopper_x_mm"], cv["belt_top_z_mm"]
        self.belt_v = cv["belt_velocity_mps"]
        self.stop_speed = rb_cfg["release_speed_threshold_mm_s"] / 1000.0
        self.stop_timeout = rb_cfg["release_wait_timeout_s"]
        self.settle = rb_cfg["settle_wait_s"]

        self.types = [(b["name"], b["w_mm"], b["d_mm"], b["h_mm"]) for b in self.cfg["boxes"]["types"]]
        self.rb, self.rn = random.Random(self.cfg["boxes"]["random_seed"]), random.Random(7)

        cb = ReentrantCallbackGroup()
        self.cli_plan = self.create_client(PlanPlacement, "/algo/plan_placement", callback_group=cb)
        self.cli_upd = self.create_client(UpdateHeightmap, "/algo/update_heightmap", callback_group=cb)
        self.cli_cancel = self.create_client(CancelPlan, "/algo/cancel_plan", callback_group=cb)
        self.cli_belt = self.create_client(SetVelocity, "/conveyor/set_velocity", callback_group=cb)
        self.pub_status = self.create_publisher(ZoneStatus, f"/zone_{self.zone.lower()}/status", 10)

        self.link, self.sim_host, self.sim_port = None, g("sim_host"), int(g("sim_port"))
        if self.mode == "socket":
            try:
                self.link = SimLink(self.sim_host, self.sim_port)
                self.sim(cmd="ping")
                self.get_logger().info(f"Isaac 소켓 {self.sim_host}:{self.sim_port} 연결됨")
            except OSError as e:
                self.get_logger().error(
                    f"Isaac 소켓 연결 실패({e}) — sim_main.py 를 먼저 띄우세요. mock 으로 내려갑니다")
                self.mode = "mock"

        self.real_boxes = bool(g("real_boxes")) and self.mode == "socket"
        self.pickup_timeout = float(g("pickup_timeout_s"))
        self.pick_pose = None          # pickup_check 가 준 실측 파지 pose
        self.state, self.fut, self.steps, self.si = "WAIT_ALGO", None, [], 0
        self.box = self.pose = None
        self.t_next = self.t_deadline = 0.0
        self.idx = self.placed = self.dropped = self.failed = 0
        self.fill_rate = 0.0
        self.create_timer(1.0 / TICK_HZ, self.on_tick, callback_group=cb)
        self.create_timer(0.5, self.on_status, callback_group=cb)
        self.get_logger().info(
            f"orchestrator 시작 — zone={self.zone} · 모드={self.mode} · 박스 {self.total}개")

    # ---------- 유틸 ----------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def sim(self, _slow=None, **req):
        """소켓 호출은 전부 여기를 지난다.

        ★ Isaac 이 죽거나 창을 닫으면 SimLink 가 예외를 던진다. 그걸 그냥 두면
          타이머 콜백 밖으로 올라가 **노드가 통째로 죽는다.** 명세서 §1.5 대로
          state="BLOCKED" 를 발행하고 재연결을 시도해야 한다 (sim_link.py 주석).
        """
        if self.link is None:
            return {}
        try:
            return self.link.call_slow(_slow, **req) if _slow else self.link.call(**req)
        except (OSError, ConnectionError, ValueError) as e:
            self.get_logger().error(f"Isaac 소켓 끊김({e}) — state=BLOCKED, 재연결 시도")
            try:
                self.link.close()
            except OSError:
                pass
            self.link, self.state, self.t_next = None, "BLOCKED", self.now() + 2.0
            return {}

    def reconnect(self):
        try:
            self.link = SimLink(self.sim_host, self.sim_port)
            self.sim(cmd="ping")
            self.get_logger().info("Isaac 소켓 재연결 성공 — 사이클 재개")
            return True
        except OSError:
            self.link = None
            return False

    def belt(self, mps):
        """벨트는 ROS 서비스로만 (§3 규칙 4). 서비스가 없으면 경고만 하고 계속 간다."""
        if self.cli_belt.service_is_ready():
            self.cli_belt.call_async(SetVelocity.Request(
                seg=f"{self.zone.lower()}_pickup", velocity_mps=float(mps)))
        elif self.link is not None:
            # ★ 임시. §3 규칙 4 는 벨트를 ROS 서비스로 부르라고 한다. sim_bridge_node 가
            #   생기면 위 분기로 자동으로 넘어가므로 이 줄은 그때 지운다.
            self.sim(cmd="set_velocity", seg=f"{self.zone.lower()}_pickup", mps=float(mps))
        elif mps == 0.0:
            self.get_logger().warn("/conveyor/set_velocity 없음 — 벨트 정지를 건너뜁니다", once=True)

    def pick_world(self, w, d, h, lifted=False):
        """파지점(월드 m).

        pickup_check 가 준 실측 pose 가 있으면 그걸 쓴다 — 박스가 스토퍼에 정확히
        붙지 않거나 살짝 틀어져 있으면 계산값으로는 그리퍼가 모서리를 문다 (D5).
        없으면 계산값: 박스 중심 x = 스토퍼 − w/2 (함정 #16).
        """
        dz = mm_to_m(self.approach) if lifted else 0.0
        if self.pick_pose:
            x, y, z = self.pick_pose["pos_m"]
            return (x, y, z + dz)
        return (mm_to_m(self.stopper_x - w // 2), 0.0, mm_to_m(self.belt_z + h / 2) + dz)

    def pick_yaw(self):
        return float(self.pick_pose["yaw_deg"]) if self.pick_pose else 0.0

    def place_world(self, h, lifted=False):
        x, y, z = pallet_place_to_world(PlacePoseData.from_dict(self.pose), h, self.origin)
        return (x, y, z + (mm_to_m(self.approach) if lifted else 0.0))

    # ---------- 로봇 스텝 (True 를 돌려주면 다음 스텝으로) ----------
    def _move(self, target, yaw):
        """move_ee 는 논블로킹 — 목표를 한 번만 넣고 motion_status 를 폴링한다 (§3 규칙 1)."""
        if not self._sent:
            r = self.sim(cmd="move_ee", target_m=list(target), yaw_deg=float(yaw))
            if not r.get("accepted"):
                self.abort(f"move_ee 거부: {r}")
                return False
            self._sent = True
            return False
        st = self.sim(cmd="motion_status").get("state")
        if st == "REACHED":
            self._sent = False
            return True
        if st == "IK_FAIL":
            self.abort("IK_FAIL — 도달 불가")
        return False

    def build_steps(self):
        box_id, w, d, h = self.box
        yaw_pick, yaw_place = self.pick_yaw(), float(self.pose["yaw_deg"])
        S = []
        S.append(lambda: self._move(self.pick_world(w, d, h, True), yaw_pick))    # 픽 접근
        S.append(lambda: self._move(self.pick_world(w, d, h), yaw_pick))          # 하강
        S.append(lambda: self._grasp(box_id))
        S.append(lambda: self._move(self.pick_world(w, d, h, True), yaw_pick))    # 상승
        S.append(lambda: self._move(self.place_world(h, True), yaw_place))        # 배치 접근
        S.append(lambda: self._move(self.place_world(h), yaw_place))              # 하강
        S.append(self._wait_stop)                                                 # TCP 정지 대기
        S.append(self._release)
        S.append(self._settle)
        return S

    def _grasp(self, box_id):
        # grasp 는 그리퍼 재시도 때문에 오래 걸린다 — 전용 타임아웃 (sim_link.call_slow)
        if not self.sim(15.0, cmd="grasp", box_id=box_id).get("ok"):
            self.failed += 1
            self.abort("grasp 실패 (PICK_FAILED)")
            return False
        return True

    def _wait_stop(self):
        """release 는 '도착'이 아니라 '정지' 조건으로 (§6.5). 타임아웃 없으면 무한루프."""
        if self._t0 == 0.0:
            self._t0 = self.now()
        if self.sim(cmd="tcp_speed").get("mps", 1.0) < self.stop_speed:
            self._t0 = 0.0
            return True
        if self.now() - self._t0 > self.stop_timeout:
            self.get_logger().warn("release 정지 대기 타임아웃 — 강제 진행")
            self._t0 = 0.0
            return True
        return False

    def _release(self):
        self.sim(15.0, cmd="release")
        self._t0 = self.now()
        return True

    def _settle(self):
        return self.now() - self._t0 > self.settle

    def measure(self):
        """실측 pose → PlacePose. Isaac 이 없으면 계획값 ±noise 로 흉내낸다."""
        box_id, w, d, h = self.box
        if self.mode == "socket":
            r = self.sim(cmd="get_box_pose", box_id=box_id)
            if r.get("ok"):
                pp = world_to_pallet_place(r["pos_m"], float(r.get("yaw_deg", 0.0)), h, self.origin)
                return PlacePose(x_mm=pp.x_mm, y_mm=pp.y_mm, z_mm=pp.z_mm, yaw_deg=pp.yaw_deg)
            self.get_logger().warn(f"get_box_pose 실패({r}) — 계획값으로 갱신합니다")
        n = self.noise
        return PlacePose(x_mm=self.pose["x_mm"] + self.rn.randint(-n, n),
                         y_mm=self.pose["y_mm"] + self.rn.randint(-n, n),
                         z_mm=self.pose["z_mm"] + self.rn.randint(-n, n),
                         yaw_deg=float(self.pose["yaw_deg"]))

    def abort(self, why):
        """모든 이탈 경로 — cancel_plan 을 부르고 벨트를 되살린다."""
        self.get_logger().error(f"{self.box[0] if self.box else '?'}: {why}")
        if self.box:
            self.cli_cancel.call_async(CancelPlan.Request(zone=self.zone, box_id=self.box[0]))
        self.belt(self.belt_v)
        self.state, self.t_next = "IDLE", self.now() + self.interval

    # ---------- 상태머신 ----------
    def on_tick(self):
        s = self.state

        if s == "BLOCKED":
            if self.now() < self.t_next:
                return
            if self.reconnect():
                self.state = "IDLE"
            else:
                self.t_next = self.now() + 2.0
            return
        if s == "WAIT_ALGO":
            if self.cli_plan.service_is_ready() and self.cli_upd.service_is_ready():
                self.get_logger().info("/algo/* 연결됨 — 사이클 시작")
                self.state, self.t_next = "IDLE", self.now()
            return

        if s == "IDLE":
            if self.now() < self.t_next:
                return
            if self.idx >= self.total:
                return self.finish("박스 소진")
            name, w, d, h = self.rb.choice(self.types)
            box_id = f"BOX{self.idx:05d}"
            self.box, self.pick_pose = (box_id, w, d, h), None
            self.idx += 1
            if self.real_boxes:
                # Isaac 에 진짜 박스를 스폰한다. 크기는 우리가 고른 것이라 DB 없이도 안다.
                # TODO(트랙 B): sim_bridge_node 가 생기면 /zone_b/box_at_pickup 구독으로 바꾼다.
                r = self.sim(cmd="spawn_box", box_id=box_id, type=name, yaw_deg=0.0)
                if not r.get("ok"):
                    self.get_logger().error(f"spawn_box 실패: {r}")
                    self.state, self.t_next = "IDLE", self.now() + self.interval
                    return
                self.belt(self.belt_v)                # 벨트를 돌려 스토퍼까지 보낸다
                self.t_deadline = self.now() + self.pickup_timeout
                self.state = "WAIT_PICKUP"
                return
            self.belt(0.0)                            # 흡착 전 벨트 정지
            self.send_plan()
            return                                    # ★ 반드시 여기서 return

        if s == "WAIT_PICKUP":
            r = self.sim(cmd="pickup_check", zone=self.zone)
            if r.get("present") and r.get("box_id") == self.box[0]:
                self.pick_pose = r["pose"]            # 실측 파지 pose (pos_m · yaw_deg)
                self.belt(0.0)                        # ★ 흡착 전 정지. 안 멈추면 박스가 끌려간다
                x, y, _z = self.pick_pose["pos_m"]
                self.get_logger().info(f"{self.box[0]} 스토퍼 도착 — 파지 ({x:.3f}, {y:.3f}) "
                                       f"yaw={self.pick_pose['yaw_deg']:.1f}")
                self.send_plan()
                return
            if self.now() > self.t_deadline:
                self.get_logger().error(f"{self.box[0]} 스토퍼 도착 타임아웃 "
                                        f"({self.pickup_timeout:.0f}s) — 다음 박스로")
                self.state, self.t_next = "IDLE", self.now()
            return

        if s == "PLANNING":
            if not (self.fut and self.fut.done()):
                return
            res, self.fut = self.fut.result(), None
            box_id, w, d, h = self.box
            if not res.ok:
                self.cli_cancel.call_async(CancelPlan.Request(zone=self.zone, box_id=box_id))
                if res.message == "PALLET_FULL":
                    return self.finish("PALLET_FULL")
                self.dropped += 1                     # 이 박스만 버리고 계속 (트랙 D A1)
                self.get_logger().warn(f"{box_id} {w}x{d}x{h} → {res.message} · 버림 "
                                       f"(누적 {self.dropped})")
                # ★ 스토퍼에 그대로 두면 뒤 박스가 밀려 라인이 통째로 막힌다. 반드시 치운다.
                #   1차에는 Return line 이 없으므로 despawn 이 유일한 처리다 (CLAUDE.md §5).
                #   TODO(트랙 B): sim_bridge_node 가 생기면 /sim/despawn_box 서비스로 바꾼다.
                if self.link is not None:
                    r = self.sim(cmd="despawn_box", box_id=box_id)
                    if not r.get("ok"):
                        self.get_logger().error(f"despawn_box 실패({r}) — 라인이 막힐 수 있습니다")
                # TODO(트랙 B): /db/set_status(box_id, 'REJECTED')
                self.belt(self.belt_v)
                self.state, self.t_next = "IDLE", self.now() + self.interval
                return
            self.pose = dict(x_mm=res.pose.x_mm, y_mm=res.pose.y_mm,
                             z_mm=res.pose.z_mm, yaw_deg=res.pose.yaw_deg)
            wx, wy, wz = self.place_world(h)
            self.get_logger().info(
                f"{box_id} {w}x{d}x{h} → 팔레트({self.pose['x_mm']},{self.pose['y_mm']},"
                f"{self.pose['z_mm']}) yaw={self.pose['yaw_deg']:.0f} ⇒ 월드({wx:.3f},{wy:.3f},{wz:.3f})m")
            if self.mode == "socket":
                self._sent, self._t0 = False, 0.0
                self.steps, self.si = self.build_steps(), 0
                self.state = "ROBOT"
            else:
                self.t_deadline = self.now() + self.cycle_s
                self.state = "MOCK_MOVE"
            return

        if s == "ROBOT":
            if self.si >= len(self.steps):
                return self.do_update()
            if self.steps[self.si]():
                self.si += 1
            return

        if s == "MOCK_MOVE":
            if self.now() >= self.t_deadline:
                self.do_update()
            return

        if s == "UPDATING":
            if not (self.fut and self.fut.done()):
                return
            res, self.fut = self.fut.result(), None
            self.placed += 1
            self.fill_rate = res.fill_rate
            if not res.ok:
                self.get_logger().warn("update ok=false — yaw 오차 초과. 로그만 남기고 계속")
            self.get_logger().info(f"  적재 {res.placed_count}개 · 적재율 {res.fill_rate*100:.1f}%"
                                   f"  (버림 {self.dropped} · 픽실패 {self.failed})")
            # TODO(트랙 B): /db/mark_placed
            self.belt(self.belt_v)
            self.state, self.t_next = "IDLE", self.now() + self.interval
            return

    def send_plan(self):
        _bid, w, d, h = self.box
        self.fut = self.cli_plan.call_async(PlanPlacement.Request(
            zone=self.zone, box_id=self.box[0], size_w_mm=w, size_d_mm=d, size_h_mm=h))
        self.state = "PLANNING"

    def do_update(self):
        box_id, w, d, h = self.box
        self.fut = self.cli_upd.call_async(UpdateHeightmap.Request(
            zone=self.zone, box_id=box_id, size_w_mm=w, size_d_mm=d, size_h_mm=h,
            actual_pose=self.measure()))
        self.state = "UPDATING"

    def finish(self, why):
        self.state = "PALLET_FULL" if why == "PALLET_FULL" else "DONE"
        self.get_logger().info(
            f"═══ 종료 ({why}) — 투입 {self.idx} · 적재 {self.placed} · 버림 {self.dropped} · "
            f"픽실패 {self.failed} · 적재율 {self.fill_rate*100:.1f}% ═══")

    def on_status(self):
        busy = self.state in ("PLANNING", "ROBOT", "MOCK_MOVE", "UPDATING", "WAIT_PICKUP")
        st = ("BLOCKED" if self.state == "BLOCKED" else
              "BUSY" if busy else
              "PALLET_FULL" if self.state == "PALLET_FULL" else "IDLE")
        self.pub_status.publish(ZoneStatus(
            zone=self.zone, queue_len=0,          # TODO(트랙 B): /db/zone_queue 중계
            buffer_cap=int(self.cfg["zones"]["buffer_capacity"]),
            est_free_time_s=0.0,
            state=st,
            fill_rate=float(self.fill_rate)))


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        if node.link:
            node.link.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
