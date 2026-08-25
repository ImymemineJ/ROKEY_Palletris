"""orchestrator — 1차 구현 실행기 (초안 작성: 트랙 C 정정인 · 소유: 트랙 B 김병직).

**Isaac 이 떠 있는 PC 에서 실행한다** (소켓이 localhost:8765 다).

한 사이클의 알맹이는 전부 `cycle.Cycle` 이 한다 — 이 파일은 **껍데기**다:
  · 박스를 흘려보내고 (spawn_box → pickup_check)
  · `Cycle.run()` 을 워커 스레드에서 부르고
  · ROS 서비스를 `svc` 로 꽂아 주고
  · `/zone_b/status` 를 2 Hz 로 발행한다

★ 사이클 로직을 여기에 다시 짜지 않는다. `Cycle` 에는 실측으로 잡은 것들이 들어 있다 —
  픽 복귀 안전 고도(e701fc4: 호가 픽업 대기 박스를 쓸어 배치 오차 29.7mm), 이송 고도를
  적재 최상단 누적 최대치로(42690a9). 다시 짜면 그 사고가 그대로 재현된다.

실행:
    ros2 run packing_robot orchestrator --ros-args -p box_count:=70
"""
import random
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from packing_common.config import load
from packing_msgs.msg import PlacePose, ZoneStatus
from packing_msgs.srv import CancelPlan, PlanPlacement, SetVelocity, UpdateHeightmap
from packing_robot.cycle import Cycle
from packing_robot.sim_link import SimLink

SVC_TIMEOUT_S = 5.0


class RosServices:
    """`Cycle` 이 요구하는 svc 덕타이핑 구현 (rclpy 판).

    ★ 워커 스레드에서만 부른다. 여기서 future 를 기다리는 것은 executor 스레드가
      아니므로 데드락이 아니다 (§1.5). 콜백 안에서는 절대 부르지 않는다.
    """

    def __init__(self, node, link, log):
        self.n, self.link, self._log = node, link, log
        self.sizes = {}              # box_id -> (w,d,h). DB 가 없을 때 크기의 출처
        self.last_plan_msg = ""      # NO_FEASIBLE_POSE 인지 PALLET_FULL 인지 구분용

    def _wait(self, fut, what):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > SVC_TIMEOUT_S:
                raise TimeoutError(f"{what} 응답 없음 ({SVC_TIMEOUT_S}s)")
            time.sleep(0.005)
        return fut.result()

    # ── 트랙 D (DB) — 아직 없으므로 스폰 때 기록해 둔 크기를 쓴다 ──
    def get_box_info(self, box_id):
        wdh = self.sizes.get(box_id)
        return {"w_mm": wdh[0], "d_mm": wdh[1], "h_mm": wdh[2]} if wdh else None

    def set_status(self, box_id, status):
        return True          # TODO(트랙 D): /db/set_status

    def mark_placed(self, box_id, actual):
        return True          # TODO(트랙 D): /db/mark_placed

    # ── 트랙 C (algo) ──
    def plan_placement(self, zone, box_id, w, d, h):
        res = self._wait(self.n.cli_plan.call_async(PlanPlacement.Request(
            zone=zone, box_id=box_id, size_w_mm=w, size_d_mm=d, size_h_mm=h)), "plan_placement")
        self.last_plan_msg = res.message
        if not res.ok:
            return False, None, res.message
        return True, dict(x_mm=res.pose.x_mm, y_mm=res.pose.y_mm,
                          z_mm=res.pose.z_mm, yaw_deg=res.pose.yaw_deg), res.message

    def update_heightmap(self, zone, box_id, w, d, h, actual):
        res = self._wait(self.n.cli_upd.call_async(UpdateHeightmap.Request(
            zone=zone, box_id=box_id, size_w_mm=w, size_d_mm=d, size_h_mm=h,
            actual_pose=PlacePose(x_mm=int(actual["x_mm"]), y_mm=int(actual["y_mm"]),
                                  z_mm=int(actual["z_mm"]),
                                  yaw_deg=float(actual.get("yaw_deg", 0.0))))), "update_heightmap")
        self.n.fill_rate = res.fill_rate
        self.n.max_height_mm = res.max_height_mm     # ZoneStatus 로 그대로 중계한다
        self.n.placed_count = res.placed_count
        return res.ok, f"{res.fill_rate * 100:.1f}%"

    def cancel_plan(self, zone, box_id):
        # ★ future 를 버리면 쌓인다. 워커 스레드이므로 기다려도 데드락이 아니다.
        try:
            self._wait(self.n.cli_cancel.call_async(
                CancelPlan.Request(zone=zone, box_id=box_id)), "cancel_plan")
        except TimeoutError as e:
            self._log(f"[svc] {e}")
        return True

    # ── 트랙 A (씬) — 브릿지가 생기기 전까지는 소켓으로 내려간다 ──
    def set_velocity(self, seg, mps):
        if self.n.cli_belt.service_is_ready():
            res = self._wait(self.n.cli_belt.call_async(
                SetVelocity.Request(seg=seg, velocity_mps=float(mps))), "set_velocity")
            return res.ok
        # ★ 임시. §3 규칙 4 는 ROS 서비스를 쓰라고 한다. sim_bridge_node 가 생기면 위로 간다.
        return bool(self.n.sim(cmd="set_velocity", seg=seg, mps=float(mps)).get("ok"))


class Orchestrator(Node):
    def __init__(self):
        super().__init__("orchestrator")
        self.cfg = load()
        p = self.declare_parameter
        p("zone", "B"); p("sim_host", "127.0.0.1"); p("sim_port", 8765)
        p("box_count", 0); p("interval_s", 0.0); p("pickup_timeout_s", 60.0)
        g = lambda k: self.get_parameter(k).value

        self.zone = g("zone")
        st1 = self.cfg["boxes"]["stage1"]
        self.total = g("box_count") or st1["total_count"]
        self.interval = g("interval_s") or st1["spawn_interval_s"]
        self.pickup_timeout = float(g("pickup_timeout_s"))
        self.types = [(b["name"], b["w_mm"], b["d_mm"], b["h_mm"]) for b in self.cfg["boxes"]["types"]]
        self.rb = random.Random(self.cfg["boxes"]["random_seed"])

        cb = ReentrantCallbackGroup()
        self.cli_plan = self.create_client(PlanPlacement, "/algo/plan_placement", callback_group=cb)
        self.cli_upd = self.create_client(UpdateHeightmap, "/algo/update_heightmap", callback_group=cb)
        self.cli_cancel = self.create_client(CancelPlan, "/algo/cancel_plan", callback_group=cb)
        self.cli_belt = self.create_client(SetVelocity, "/conveyor/set_velocity", callback_group=cb)
        self.pub_status = self.create_publisher(ZoneStatus, f"/zone_{self.zone.lower()}/status", 10)

        self.sim_host, self.sim_port = g("sim_host"), int(g("sim_port"))
        self.link = SimLink(self.sim_host, self.sim_port)            # 실패하면 여기서 예외
        self.link.call(cmd="ping")
        self.get_logger().info(f"Isaac 소켓 {self.sim_host}:{self.sim_port} 연결됨")

        self.svc = RosServices(self, self.link, self.get_logger().info)
        self.cycle = Cycle(self.link, self.svc, self.cfg, self.zone, log=self.get_logger().info)

        self.state, self.finished = "IDLE", False
        self.accepting = True                        # §3-2: false = 더 보내지 마라
        self.fill_rate, self.placed_count = 0.0, 0
        self.max_height_mm = 0                       # UpdateHeightmap 응답에서 받는다
        self.placed = self.dropped = self.failed = 0
        self.create_timer(0.5, self.on_status, callback_group=cb)
        self.get_logger().info(f"orchestrator 시작 — zone={self.zone} · 박스 {self.total}개 · "
                               f"투입 간격 {self.interval}s")

    # ── 소켓 호출은 전부 여기를 지난다 ──
    def sim(self, _slow=None, **req):
        """★ 끊겨도 죽지 않는다. accepting=false 로 떨어뜨리고 재연결에 맡긴다.

        Cycle.run() 안쪽은 cycle.py 가 예외를 잡아 주지만, **사이클 밖**(spawn_box ·
        pickup_check · despawn_box · set_velocity)은 맨몸이었다. 거기서 예외가 나면
        워커 스레드만 죽고 rclpy.spin 은 메인 스레드에서 계속 돌아, 노드가
        **죽었는데 state=IDLE 을 계속 광고한다.** 중재자가 그 값으로 배정을 정하므로
        (§6.7) 크래시보다 나쁘다 — 상대가 알아차릴 방법이 없다.
        """
        if self.link is None:
            return {}
        try:
            return self.link.call_slow(_slow, **req) if _slow else self.link.call(**req)
        except (OSError, ConnectionError, ValueError) as e:
            self.get_logger().error(f"Isaac 소켓 끊김({e}) — accepting=false, 재연결 시도")
            try:
                self.link.close()
            except OSError:
                pass
            # ★ "BLOCKED" 는 §3-2 의 state 값이 아니다 (7상태뿐). 못 받는다는 건
            #   state 가 아니라 **accepting=false** 로 알린다 — 중재자는 그 칸만 본다.
            self.link, self.accepting = None, False
            return {}

    def ensure_link(self):
        """끊겼으면 다시 잡을 때까지 기다린다. 워커 스레드에서만 부른다."""
        while self.link is None and rclpy.ok():
            try:
                link = SimLink(self.sim_host, self.sim_port)
                link.call(cmd="ping")
            except OSError:
                time.sleep(2.0)
                continue
            # ★ Cycle 을 새로 만들지 않는다. _stack_top_m(누적 적재 최상단, 42690a9)이
            #   0 으로 돌아가면 재연결 뒤 첫 이송이 적재물 높이를 잊고 기둥을 지나간다.
            self.link = self.svc.link = self.cycle.link = link
            self.state, self.accepting = "IDLE", True
            self.get_logger().info("Isaac 소켓 재연결 — 사이클 재개")
        return self.link is not None

    # ── 워커 스레드 — 여기서만 블로킹한다 ──
    def loop(self):
        try:
            self._loop()
        except Exception as e:                       # noqa: BLE001 — 조용히 죽지 않게
            self.accepting = False
            self.get_logger().error(f"워커 스레드 중단: {e!r} — accepting=false 로 남깁니다")

    def _loop(self):
        log = self.get_logger()
        while not self.cli_plan.service_is_ready() or not self.cli_upd.service_is_ready():
            time.sleep(0.2)
        log.info("/algo/* 연결됨 — 사이클 시작")

        # ★ 메인 벨트는 켜 두고 계속 돌린다. 박스는 x=500(메인 구간 0~7500)에 스폰되므로
        #   이걸 안 켜면 스폰만 되고 영영 안 움직인다. Cycle 이 흡착 전에 멈추는 것은
        #   b_pickup 구간뿐이고, 앞 구간이 계속 돌아야 뒤 박스가 밀려와 대기열이 생긴다
        #   (설계결정서 §4.4 — 백압 조율에 필요한 버퍼가 물리적으로 생기는 지점).
        belt_v = float(self.cfg["conveyor"]["belt_velocity_mps"])
        if not self.svc.set_velocity("main", belt_v):
            log.error("메인 벨트 가동 실패 — 박스가 스토퍼까지 못 옵니다. "
                      "씬에 /World/ConveyorTrack 가 있는지 확인하세요")
        else:
            log.info(f"메인 벨트 {belt_v} m/s 가동")

        for i in range(self.total):
            if not rclpy.ok():
                return
            if not self.ensure_link():        # 끊겼으면 다시 잡고 나서 진행
                return
            name, w, d, h = self.rb.choice(self.types)
            box_id = f"BOX{i:05d}"
            self.svc.sizes[box_id] = (w, d, h)      # DB 대신 크기의 출처

            r = self.sim(cmd="spawn_box", box_id=box_id, type=name, yaw_deg=0.0)
            if not r.get("ok"):
                log.error(f"spawn_box 실패: {r}")
                continue
            self.svc.set_velocity(f"{self.zone.lower()}_pickup", belt_v)

            pick = self.wait_pickup(box_id)
            if pick is None:
                # 벨트에 남겨 두면 다음 박스와 겹친다 — 치우고 간다
                log.error(f"{box_id} 스토퍼 도착 타임아웃 — despawn 후 다음 박스로")
                self.sim(cmd="despawn_box", box_id=box_id)
                self.failed += 1
                continue

            # ★ §3-2 의 7상태만 쓴다. 사이클 알맹이는 Cycle 안에 있어 여기서
            #   PICKING/PLACING/UPDATING 을 나눠 볼 수 없다 — 대표값 하나로 둔다.
            #   (세분화는 소켓을 걷어낸 진짜 ROS 관제탑에서 한다. §2)
            self.state = "PICKING"
            res = self.cycle.run(box_id, pick)      # ★ 알맹이는 전부 Cycle 이 한다
            self.state = "IDLE"

            if res == "PLACED":
                self.placed += 1
            elif res == "PALLET_FULL":
                self.finish("PALLET_FULL"); return
            elif res == "BLOCKED" and self.svc.last_plan_msg == "NO_FEASIBLE_POSE":
                # 이 박스만 자리가 없다 → 치우고 계속 (트랙 D A1). 스토퍼에 두면 라인이 막힌다.
                self.dropped += 1
                self.sim(cmd="despawn_box", box_id=box_id)
                log.warn(f"{box_id} {w}x{d}x{h} → NO_FEASIBLE_POSE · 버림 (누적 {self.dropped})")
            else:
                self.failed += 1
                log.error(f"{box_id} → {res} (누적 실패 {self.failed})")
                self.sim(cmd="despawn_box", box_id=box_id)
            log.info(f"  적재 {self.placed} · 버림 {self.dropped} · 실패 {self.failed} "
                     f"· 적재율 {self.fill_rate * 100:.1f}%")
            time.sleep(self.interval)
        self.finish("박스 소진")

    def wait_pickup(self, box_id):
        """스토퍼에 그 박스가 설 때까지 기다린다. 실측 파지 pose 를 돌려준다."""
        t0 = time.time()
        while time.time() - t0 < self.pickup_timeout:
            if not rclpy.ok():
                return None
            r = self.sim(cmd="pickup_check", zone=self.zone)
            if r.get("present") and r.get("box_id") == box_id:
                x, y, _z = r["pose"]["pos_m"]
                self.get_logger().info(f"{box_id} 스토퍼 도착 — 파지 ({x:.3f}, {y:.3f}) "
                                       f"yaw={r['pose']['yaw_deg']:.1f}")
                return r["pose"]
            time.sleep(0.05)
        return None

    def finish(self, why):
        # ★ ZoneStatus.state 는 관제탑 7상태다 (CLAUDE.md §3-2):
        #   IDLE|FETCHING|PLANNING|PICKING|PLACING|UPDATING|DONE.
        #   옛 계약("BUSY"|"BLOCKED"|"PALLET_FULL")은 폐기됐다. 끝났으면 DONE 이고,
        #   "더 보내지 마라"는 state 가 아니라 **accepting=false** 로 알린다.
        self.state = "DONE"
        self.accepting = False
        self.finished = True
        self.get_logger().info(
            f"═══ 종료 ({why}) — 적재 {self.placed} · 버림 {self.dropped} · 실패 {self.failed} "
            f"· 적재율 {self.fill_rate * 100:.1f}% ═══")

    def on_status(self):
        self.pub_status.publish(ZoneStatus(
            zone=self.zone.lower(),                  # §3-2 는 소문자 "a"|"b"|"c"
            state=self.state,
            queue_len=0,                             # TODO(트랙 D): /db/zone_queue 중계
            placed_count=int(self.placed),
            rejected_count=int(self.dropped),
            fill_rate=float(self.fill_rate),
            max_height_mm=int(self.max_height_mm),
            accepting=bool(self.accepting),
            stamp=self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Orchestrator()
    except OSError as e:
        print(f"\n★ Isaac 소켓 연결 실패({e})\n"
              "   sim_main.py 를 먼저 띄우세요:\n"
              "   ~/isaacsim/python.sh src/packing_sim/isaac/sim_main.py --zone B --gui\n")
        rclpy.shutdown()
        return
    threading.Thread(target=node.loop, daemon=True).start()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.link.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
