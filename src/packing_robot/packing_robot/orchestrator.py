"""orchestrator — pick & place 상태머신 ★ 통합 시연용 초안 (작성: 트랙 C 정정인).

**소유는 트랙 B(김병직)입니다.** DB(트랙 D)와 Isaac(트랙 A)이 아직 없어도
`/algo/*` 왕복이 ROS 로 실제로 도는 것을 보여주려고 먼저 만든 것입니다.

지금 하는 일:
  박스 생성(내부) → /algo/plan_placement → 좌표 변환 → (로봇 이동 흉내)
  → /algo/update_heightmap(실측 pose) → /zone_b/status 발행

김병직님이 채울 자리 3곳은 `# TODO(트랙 B)` 로 표시했습니다. 그 자리에 소켓 호출
(move_ee / grasp / release / get_box_pose)을 넣으면 실제 로봇이 움직입니다.

철칙:
- 어떤 콜백에서도 `spin_until_future_complete` 를 쓰지 않는다. 상태머신을 타이머로 돌리고
  서비스는 `call_async` 후 다음 틱에 `.done()` 을 폴링한다 (작업명세서 §1.5 방법 2).
- 좌표 변환은 `packing_common.frames` 만 쓴다 — 트랙 C 와 같은 파일이어야 좌표가 맞는다.
- 모든 이탈 경로에서 `/algo/cancel_plan` 을 부른다.

실행:
    ros2 run packing_robot orchestrator
    ros2 run packing_robot orchestrator --ros-args -p interval_s:=1.0 -p box_count:=30
"""
import random

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from packing_common.config import load
from packing_common.frames import PlacePose as PlacePoseData
from packing_common.frames import pallet_place_to_world
from packing_msgs.msg import PlacePose, ZoneStatus
from packing_msgs.srv import CancelPlan, PlanPlacement, UpdateHeightmap

TICK_HZ = 20.0        # 상태머신 틱


class Orchestrator(Node):
    def __init__(self):
        super().__init__("orchestrator")
        self.cfg = load()

        self.declare_parameter("zone", "B")
        self.declare_parameter("interval_s", 0.0)     # 0 = config 의 spawn_interval_s
        self.declare_parameter("box_count", 0)        # 0 = config 의 total_count
        self.declare_parameter("mock_cycle_s", 1.0)   # 로봇 1사이클 흉내 시간
        self.declare_parameter("place_noise_mm", 3)   # 실측 오차 흉내

        g = lambda k: self.get_parameter(k).value
        self.zone = g("zone")
        st1 = self.cfg["boxes"]["stage1"]
        self.interval = g("interval_s") or st1["spawn_interval_s"]
        self.total = g("box_count") or st1["total_count"]
        self.cycle_s = g("mock_cycle_s")
        self.noise = g("place_noise_mm")

        self.origin = self.cfg["zones"]["pallet_origin_world_mm"][self.zone]
        self.types = [(b["w_mm"], b["d_mm"], b["h_mm"]) for b in self.cfg["boxes"]["types"]]
        self.rb = random.Random(self.cfg["boxes"]["random_seed"])
        self.rn = random.Random(7)

        cb = ReentrantCallbackGroup()
        self.cli_plan = self.create_client(PlanPlacement, "/algo/plan_placement", callback_group=cb)
        self.cli_upd = self.create_client(UpdateHeightmap, "/algo/update_heightmap", callback_group=cb)
        self.cli_cancel = self.create_client(CancelPlan, "/algo/cancel_plan", callback_group=cb)
        self.pub_status = self.create_publisher(ZoneStatus, f"/zone_{self.zone.lower()}/status", 10)

        # 상태
        self.state = "WAIT_ALGO"       # WAIT_ALGO|IDLE|PLANNING|MOVING|UPDATING|PALLET_FULL|DONE
        self.fut = None
        self.box = None                # (box_id, w, d, h)
        self.pose = None               # 계획 PlacePose (dict)
        self.t_arrive = 0.0            # 다음 박스가 도착하는 시각
        self.t_move_end = 0.0
        self.idx = self.placed = self.dropped = 0
        self.fill_rate = 0.0

        self.create_timer(1.0 / TICK_HZ, self.on_tick, callback_group=cb)
        self.create_timer(0.5, self.on_status, callback_group=cb)      # 2 Hz
        self.get_logger().info(
            f"orchestrator 시작 — zone={self.zone} · 박스 {self.total}개 · "
            f"투입 간격 {self.interval}s · 사이클 흉내 {self.cycle_s}s · 오차 ±{self.noise}mm")

    # ---------- 유틸 ----------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def next_box(self):
        """★ 원래는 A 의 /zone_b/box_at_pickup 을 구독한다. 지금은 내부 생성."""
        if self.idx >= self.total:
            return None
        w, d, h = self.rb.choice(self.types)
        box = (f"BOX{self.idx:05d}", w, d, h)
        self.idx += 1
        return box

    # ---------- 상태머신 (타이머 — 절대 블로킹하지 않는다) ----------
    def on_tick(self):
        s = self.state

        if s == "WAIT_ALGO":
            if self.cli_plan.service_is_ready() and self.cli_upd.service_is_ready():
                self.get_logger().info("/algo/* 연결됨 — 사이클 시작")
                self.state, self.t_arrive = "IDLE", self.now()
            return

        if s == "IDLE":
            if self.now() < self.t_arrive:
                return
            self.box = self.next_box()
            if self.box is None:
                self.finish("박스 소진")
                return
            box_id, w, d, h = self.box
            # TODO(트랙 B): 여기서 /conveyor/set_velocity(b_pickup, 0.0) 을 부르고 ok 를 확인한다.
            # TODO(트랙 B): DB 가 붙으면 /db/get_box_info(box_id) 로 크기를 받아온다.
            req = PlanPlacement.Request(zone=self.zone, box_id=box_id,
                                        size_w_mm=w, size_d_mm=d, size_h_mm=h)
            self.fut = self.cli_plan.call_async(req)
            self.state = "PLANNING"
            return                                         # ★ 반드시 여기서 return

        if s == "PLANNING":
            if not (self.fut and self.fut.done()):
                return
            res, self.fut = self.fut.result(), None
            box_id, w, d, h = self.box
            if not res.ok:
                if res.message == "PALLET_FULL":
                    self.cli_cancel.call_async(CancelPlan.Request(zone=self.zone, box_id=box_id))
                    self.finish("PALLET_FULL")
                    return
                # NO_FEASIBLE_POSE = 이 박스만 자리가 없다 → 버리고 계속 (트랙 D 설계변경 A1)
                self.cli_cancel.call_async(CancelPlan.Request(zone=self.zone, box_id=box_id))
                self.dropped += 1
                self.get_logger().warn(f"{box_id} {w}x{d}x{h} → {res.message} · 버림 "
                                       f"(누적 {self.dropped})")
                # TODO(트랙 B): /db/set_status(REJECTED) + /sim/despawn_box(box_id)
                self.state, self.t_arrive = "IDLE", self.now() + self.interval
                return

            self.pose = dict(x_mm=res.pose.x_mm, y_mm=res.pose.y_mm,
                             z_mm=res.pose.z_mm, yaw_deg=res.pose.yaw_deg)
            # ★ 좌표 변환은 packing_common.frames 만 쓴다 (트랙 C 와 같은 파일)
            wx, wy, wz = pallet_place_to_world(PlacePoseData.from_dict(self.pose), h, self.origin)
            self.get_logger().info(
                f"{box_id} {w}x{d}x{h} → 팔레트({self.pose['x_mm']},{self.pose['y_mm']},"
                f"{self.pose['z_mm']}) yaw={self.pose['yaw_deg']:.0f}"
                f"  ⇒ 월드({wx:.3f}, {wy:.3f}, {wz:.3f})m")
            # TODO(트랙 B): 여기서 소켓으로 move_ee → 하강 → grasp → 이송 → release.
            #               지금은 cycle_s 만큼 기다리는 것으로 흉내낸다.
            self.t_move_end = self.now() + self.cycle_s
            self.state = "MOVING"
            return

        if s == "MOVING":
            if self.now() < self.t_move_end:
                return
            box_id, w, d, h = self.box
            # TODO(트랙 B): 소켓 get_box_pose(box_id) 로 실제 놓인 위치를 읽는다.
            #               지금은 계획값에 ±noise mm 를 섞어 흉내낸다.
            actual = PlacePose(
                x_mm=self.pose["x_mm"] + self.rn.randint(-self.noise, self.noise),
                y_mm=self.pose["y_mm"] + self.rn.randint(-self.noise, self.noise),
                z_mm=self.pose["z_mm"] + self.rn.randint(-self.noise, self.noise),
                yaw_deg=float(self.pose["yaw_deg"]))
            self.fut = self.cli_upd.call_async(UpdateHeightmap.Request(
                zone=self.zone, box_id=box_id, size_w_mm=w, size_d_mm=d, size_h_mm=h,
                actual_pose=actual))
            self.state = "UPDATING"
            return

        if s == "UPDATING":
            if not (self.fut and self.fut.done()):
                return
            res, self.fut = self.fut.result(), None
            self.placed += 1
            self.fill_rate = res.fill_rate
            if not res.ok:
                self.get_logger().warn("update ok=false — yaw 오차 초과. 로그만 남기고 계속")
            self.get_logger().info(
                f"  적재 {res.placed_count}개 · 적재율 {res.fill_rate * 100:.1f}%"
                f"  (버림 {self.dropped})")
            # TODO(트랙 B): /db/mark_placed + /conveyor/set_velocity(b_pickup, 0.5)
            self.state, self.t_arrive = "IDLE", self.now() + self.interval
            return

    def finish(self, why):
        self.state = "PALLET_FULL" if why == "PALLET_FULL" else "DONE"
        self.get_logger().info(
            f"═══ 종료 ({why}) — 투입 {self.idx} · 적재 {self.placed} · 버림 {self.dropped} · "
            f"적재율 {self.fill_rate * 100:.1f}% ═══")

    # ---------- /zone_b/status (2 Hz) ----------
    def on_status(self):
        busy = self.state in ("PLANNING", "MOVING", "UPDATING")
        self.pub_status.publish(ZoneStatus(
            zone=self.zone,
            queue_len=0,                 # TODO(트랙 B): /db/zone_queue 응답을 중계한다
            buffer_cap=int(self.cfg["zones"]["buffer_capacity"]),
            est_free_time_s=max(0.0, self.t_move_end - self.now()) if busy else 0.0,
            state="BUSY" if busy else ("PALLET_FULL" if self.state == "PALLET_FULL" else "IDLE"),
            fill_rate=float(self.fill_rate)))


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())      # ★ 철칙 4
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
