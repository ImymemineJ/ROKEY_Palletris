#!/usr/bin/env python3
"""robot_node — 관제탑과 `Cycle` 사이를 메우는 노드 (트랙 B 김병직).

관제탑은 `RobotCmd(PICK/PLACE/HOME/STOP)` 이라는 **사이클 단위**로 말하고,
`cycle.py` 는 `move_ee`/`grasp`/`release` 라는 **동작 단위**로 말한다.
1:1 대응이 없어서 그 사이를 메우는 노드가 필요했다 — 이 파일이다.

    관제탑 ──RobotCmd──▶ robot_node ──SimLink(소켓)──▶ Isaac
           ◀─RobotEvent─┘

★ 소켓은 **Isaac 쪽만** 남아 있다 (CLAUDE.md §2 가 폐기 대상으로 적은 그 소켓이다).
  관제탑과의 사이는 ROS 2 전용 타입이다. Isaac 쪽을 Action Graph 로 바꾸는 것은
  트랙 A 와 함께 하는 다음 단계이고, 지금 같이 바꾸면 `demo_stack15.py` 기준선
  (시드 42 · 28/29 · 80.3%)이 깨져 무엇이 틀렸는지 구분할 수 없다.

## 반드시 지키는 것 3개 (CLAUDE.md §3-4)

1. `RobotEvent.seq` 는 `RobotCmd.seq` 를 **그대로 복사**한다. 안 하면 타임아웃 뒤
   늦게 온 답이 다음 박스의 결과로 둔갑한다 — 에러 없이 엉뚱하게 도는 버그다.
2. `has_actual` 은 **PLACE 성공 때만** true. 계획값을 되보내면 오차가 누적돼
   알고리즘은 1300 안이라 믿는데 실물이 넘는다.
3. **무슨 일이 있어도 답한다.** 예외로 조용히 죽으면 관제탑이 20초를 통째로 버린다.

## ★ Cycle 은 구역당 하나다. 명령마다 새로 만들면 안 된다

`Cycle` 은 상태를 들고 있다:
  `_stack_top_m`  이송 고도의 바닥. 0 이 되면 팔이 적재물 위를 낮게 지나
                  스택을 친다 — §10-3 이 기록한 knock 700.8mm 사고와 같은 경로다
  `_obstacles`    move_ee 의 IK 가지 선택용
  `_pick_yaw`     PLACE 의 yaw 기준. 없으면 손목이 먼 쪽으로 180° 돈다
  `_pick_c/_top`  이송 고도의 한 항

실행:
    ros2 run packing_robot robot_node --ros-args -p zones:="['b']"
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from packing_common.config import load
from packing_msgs.msg import RobotCmd, RobotEvent

from packing_robot.cycle import Cycle, MotionError
from packing_robot.sim_link import SimLink

ACTION_NAME = {RobotCmd.PICK: "PICK", RobotCmd.PLACE: "PLACE",
               RobotCmd.HOME: "HOME", RobotCmd.STOP: "STOP"}


class _NoSvc:
    """`Cycle` 이 요구하는 svc 자리의 빈 껍데기.

    ★ 왜 비었나: `pick()`/`place()` 가 실제로 부르는 svc 는 `_abort()` 의
      `cancel_plan` **하나뿐**이고, 그건 관제탑이 이미 한다 (`discard_box` →
      `/algo/cancel_plan`). 여기서 또 부르면 같은 계획을 두 번 취소한다.
      나머지(벨트·DB·plan·heightmap)는 애초에 반쪽 두 개의 바깥에 있다.
    """

    def cancel_plan(self, zone, box_id):
        return True

    def set_status(self, box_id, status):
        return True


class ZoneRobot:
    """구역 하나의 로봇. ★ `Cycle` 을 **한 번만** 만들어 계속 재사용한다."""

    def __init__(self, node, zone, cfg, host, port):
        self.node, self.zone, self.log = node, zone, node.get_logger()
        self.link = SimLink(host, port)
        self.link.call(cmd="ping")
        # ★ 여기서 만든 이 하나를 끝까지 쓴다. 명령마다 새로 만들면 _stack_top_m 이
        #   0 으로 돌아가 이송이 적재물 높이를 잊는다 (파일 상단 주석 참조).
        self.cycle = Cycle(self.link, _NoSvc(), cfg, zone.upper(),
                           log=lambda m: self.log.info(str(m)))
        self.origin_mm = list(cfg["zones"]["pallet_origin_world_mm"][zone.upper()])

        cb = ReentrantCallbackGroup()
        self.pub = node.create_publisher(RobotEvent, f"/zone_{zone}/robot/event", 10)
        node.create_subscription(RobotCmd, f"/zone_{zone}/robot/cmd",
                                 self.on_cmd, 10, callback_group=cb)
        # ★ 한 번에 한 명령만. 관제탑은 순차로 보내지만, 타임아웃 뒤 재시도가
        #   겹치면 워커 둘이 같은 팔을 동시에 움직인다.
        self.busy = threading.Lock()
        self.log.info(f"[{zone}] 로봇 준비 — Isaac {host}:{port} · 팔레트 원점 {self.origin_mm}")

    # ── 구독 콜백: 절대 블로킹하지 않는다 ──
    def on_cmd(self, c):
        # ★ 콜백에서 검증한다. rclpy 는 콜백 예외를 삼키므로, 잘못된 메시지 하나에
        #   로봇이 영영 안 움직이는데 에러는 안 뜬다 (§9).
        if c.action not in ACTION_NAME:
            self.log.warn(f"모르는 action={c.action} seq={c.seq} — 무시")
            return
        if not self.busy.acquire(blocking=False):
            self.log.error(f"이미 모션 중인데 {ACTION_NAME[c.action]} seq={c.seq} 가 왔다 "
                           f"— ok=false 로 답한다")
            self._answer(c, False, "ABORTED", None, 0.0)
            return
        threading.Thread(target=self._work, args=(c,), daemon=True).start()

    # ── 워커 스레드: 여기서만 블로킹한다 ──
    def _work(self, c):
        t0 = time.time()
        ok, reason, actual = False, "ABORTED", None
        try:
            ok, reason, actual = self._dispatch(c)
        except MotionError as e:
            reason = "IK_UNSOLVABLE" if e.state == "IK_FAIL" else "COLLISION"
            self.log.error(f"{ACTION_NAME[c.action]} seq={c.seq} 이동 실패 {e.state}")
        except Exception as e:                       # noqa: BLE001
            # ★ 여기서 삼키면 관제탑이 20초를 통째로 버린다. 반드시 답한다.
            reason = "ABORTED"
            self.log.error(f"{ACTION_NAME[c.action]} seq={c.seq} 예외 {e!r}")
        finally:
            self._answer(c, ok, reason, actual, time.time() - t0)
            self.busy.release()

    def _dispatch(self, c):
        name = ACTION_NAME[c.action]
        self.log.info(f"[{self.zone}] ← {name} seq={c.seq} {c.box_id} "
                      f"({c.x_mm}, {c.y_mm}, {c.z_mm}) yaw={c.yaw_deg:.1f}")

        if c.action == RobotCmd.PICK:
            h = int(c.size_h_mm)
            # ★ RobotCmd.z_mm 은 **박스 밑면**이다 (RobotCmd.msg 정본).
            #   Cycle.pick 은 박스 **중심**을 받으므로 h/2 를 더한다.
            pose = {"pos_m": [c.x_mm / 1000.0, c.y_mm / 1000.0, (c.z_mm + h / 2.0) / 1000.0],
                    "yaw_deg": float(c.yaw_deg)}
            r = self.cycle.pick(c.box_id, pose, h)
            return (r == "PICKED"), ("" if r == "PICKED" else "GRASP_FAILED"), None

        if c.action == RobotCmd.PLACE:
            # ★ PLACE 의 좌표는 **팔레트 기준**이다 (PlacePose 를 그대로 실어 보낸다).
            #   z 는 밑면. pallet_place_to_world 가 그 규약으로 월드 중심을 만든다.
            place = {"x_mm": int(c.x_mm), "y_mm": int(c.y_mm), "z_mm": int(c.z_mm),
                     "yaw_deg": float(c.yaw_deg)}
            r, actual, measured = self.cycle.place(
                c.box_id, place, int(c.size_w_mm), int(c.size_d_mm), int(c.size_h_mm))
            if r != "PLACED":
                return False, "DROPPED", None
            # ★ 씬에서 **실제로 읽은** 경우에만 실어 보낸다. 못 읽었으면 actual 은
            #   계획값이고, 그걸 has_actual=true 로 보내면 §3-4 규약 3 위반이다.
            if not measured:
                self.log.warn(f"{c.box_id} 실측 pose 를 못 읽었다 — has_actual=false 로 보낸다")
            return True, "", (actual if measured else None)

        if c.action == RobotCmd.HOME:
            # ★ 진짜 홈 관절값은 아직 없다 — 적재물 위로 팔만 뺀다.
            #   `Cycle.retreat_home()` 주석에 왜인지 적어 뒀다.
            self.cycle.retreat_home()
            return True, "", None

        # STOP — `robot_motion.handlers()` 에 stop 이 없다 (move_ee · motion_status ·
        # grasp · release · is_holding · tcp_speed 여섯뿐). 없는 API 를 부르지 않는다.
        # 지금 관제탑도 STOP 을 안 보낸다. 보내기 시작하면 여기와 robot_motion 을
        # 같이 채운다 — 조용히 ok 로 답하면 "멈췄다"고 오해하게 만든다.
        self.log.error(f"STOP seq={c.seq} — 아직 구현되지 않았다 (robot_motion 에 핸들러 없음)")
        return False, "ABORTED", None

    def _answer(self, c, ok, reason, actual, dt):
        ev = RobotEvent()
        ev.seq, ev.action, ev.box_id = c.seq, c.action, c.box_id   # ★ seq 그대로 복사
        ev.ok, ev.reason = bool(ok), ("" if ok else reason)
        # ★ PLACE 성공 때만 실측을 싣는다. 계획값을 되보내지 않는다 (§3-4 규약 3).
        if ok and c.action == RobotCmd.PLACE and actual is not None:
            ev.has_actual = True
            ev.actual_x_mm = int(actual["x_mm"])
            ev.actual_y_mm = int(actual["y_mm"])
            ev.actual_z_mm = int(actual["z_mm"])          # 밑면 기준
            ev.actual_yaw_deg = float(actual.get("yaw_deg", c.yaw_deg))
        ev.duration_s = float(dt)
        self.pub.publish(ev)
        self.log.info(f"[{self.zone}] → {ACTION_NAME[c.action]} seq={ev.seq} ok={ev.ok}"
                      + (f" {ev.reason}" if ev.reason else "")
                      + (f" 실측({ev.actual_x_mm},{ev.actual_y_mm},{ev.actual_z_mm})"
                         if ev.has_actual else "")
                      + f" {dt:.1f}s")


class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_node")
        self.declare_parameter("zones", [""])
        self.declare_parameter("zone", "b")
        self.declare_parameter("sim_host", "127.0.0.1")
        self.declare_parameter("sim_port", 8765)

        zs = [str(z).strip().lower()
              for z in (self.get_parameter("zones").value or []) if str(z).strip()]
        if not zs:
            zs = [str(self.get_parameter("zone").value).strip().lower()]
        seen, zones = set(), []
        for z in zs:
            if z not in seen:
                seen.add(z)
                zones.append(z)

        cfg = load()
        host = str(self.get_parameter("sim_host").value)
        port = int(self.get_parameter("sim_port").value)
        self.robots = {z: ZoneRobot(self, z, cfg, host, port) for z in zones}
        self.get_logger().info(
            f"robot_node 시작 — 구역 {', '.join(z.upper() for z in zones)}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RobotNode()
    except OSError as e:
        print(f"\n★ Isaac 소켓 연결 실패({e})\n"
              "   sim_main.py 를 먼저 띄우세요:\n"
              "   ~/isaacsim/python.sh src/packing_sim/isaac/sim_main.py --zone B --gui\n")
        rclpy.shutdown()
        return
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
