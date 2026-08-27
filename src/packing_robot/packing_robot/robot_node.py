#!/usr/bin/env python3
"""robot_node — 관제탑과 `Cycle` 사이를 메우는 노드 (트랙 B 김병직).

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : ⑦⑧ (과도기 경로) 관제탑 토픽 ↔ Cycle
    ║     불러 주는 쪽        : launch / ros2 run
    ║     이 파일이 부르는 쪽 : cmd_adapter → cycle → SimLink(소켓)
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

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

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from packing_common.config import load
from packing_msgs.msg import RobotCmd, RobotEvent

from packing_robot.cmd_adapter import ACTION_NAME, run_cmd
from packing_robot.cycle import Cycle
from packing_robot.sim_link import SimLink

# ★ 규칙(seq 복사 · has_actual 게이트 · 예외에도 답하기)은 여기 없다 —
#   `cmd_adapter.run_cmd` 하나에 있다. 최종 구조에서는 Isaac 안의 sim_main 이
#   같은 함수를 부른다. 두 군데서 유도하면 한쪽이 반드시 틀린다.


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
        """`/zone_*/robot/cmd` 구독 콜백. **절대 블로킹하지 않는다.**

        모션은 5~10초 걸린다. 여기서 기다리면 executor 스레드 하나가 그동안 묶여
        같은 그룹의 다른 콜백이 안 돈다. 그래서 워커 스레드로 넘기고 즉시 돌아온다.

        ★ 검증을 여기서 한다. rclpy 는 콜백 예외를 삼키므로, 잘못된 메시지 하나에
          로봇이 영영 안 움직이는데 에러는 안 뜬다 (CLAUDE.md §9).
        ★ 이미 모션 중이면 **조용히 버리지 않고 `ok=false` 로 답한다.** 답이 없으면
          관제탑은 20초를 통째로 버린 뒤에야 안다.
        """
        # ★ 콜백에서 검증한다. rclpy 는 콜백 예외를 삼키므로, 잘못된 메시지 하나에
        #   로봇이 영영 안 움직이는데 에러는 안 뜬다 (§9).
        if c.action not in ACTION_NAME:
            self.log.warn(f"모르는 action={c.action} seq={c.seq} — 무시")
            return
        if not self.busy.acquire(blocking=False):
            self.log.error(f"이미 모션 중인데 {ACTION_NAME[c.action]} seq={c.seq} 가 왔다 "
                           f"— ok=false 로 답한다")
            ev = run_cmd(self.cycle, {"action": -1, "seq": int(c.seq),
                                      "box_id": str(c.box_id)},
                         log=lambda m: self.log.warn(str(m)))
            ev["action"] = int(c.action)
            self._publish(ev)
            return
        threading.Thread(target=self._work, args=(c,), daemon=True).start()

    # ── 워커 스레드: 여기서만 블로킹한다 ──
    def _work(self, c):
        """워커 스레드 — **여기서만 블로킹한다.** 규약은 `cmd_adapter.run_cmd` 가 지킨다."""
        try:
            ev = run_cmd(self.cycle, _cmd_to_dict(c),
                         log=lambda m: self.log.info(str(m)))
            self._publish(ev)
        finally:
            # ★ 무슨 일이 있어도 락을 놓는다. 안 놓으면 다음 명령이 영영 안 돈다.
            self.busy.release()

    def _publish(self, ev):
        """dict → `RobotEvent` 발행. `seq` 는 어댑터가 이미 복사해 뒀다 (§3-4 규약 2)."""
        m = RobotEvent()
        m.seq = int(ev["seq"])
        m.action = int(ev["action"])
        m.box_id = str(ev["box_id"])
        m.ok = bool(ev["ok"])
        m.reason = str(ev["reason"])
        m.has_actual = bool(ev["has_actual"])
        m.actual_x_mm = int(ev["actual_x_mm"])
        m.actual_y_mm = int(ev["actual_y_mm"])
        m.actual_z_mm = int(ev["actual_z_mm"])
        m.actual_yaw_deg = float(ev["actual_yaw_deg"])
        m.duration_s = float(ev["duration_s"])
        self.pub.publish(m)


def _cmd_to_dict(c):
    """RobotCmd 메시지 -> 어댑터가 받는 순수 dict."""
    return {"action": int(c.action), "seq": int(c.seq), "box_id": str(c.box_id),
            "x_mm": int(c.x_mm), "y_mm": int(c.y_mm), "z_mm": int(c.z_mm),
            "yaw_deg": float(c.yaw_deg),
            "size_w_mm": int(c.size_w_mm), "size_d_mm": int(c.size_d_mm),
            "size_h_mm": int(c.size_h_mm),
            "approach_z_mm": int(c.approach_z_mm)}


class RobotNode(Node):
    """구역별 `ZoneRobot` 을 들고 `/zone_*/robot/{cmd,event}` 를 잇는다.

    ★ **과도기 노드다.** 최종 구조에서는 Isaac 안의 `sim_main --ros` 가 같은 일을
      소켓 없이 한다 (`ros_side.RosSide`). 둘 다 `cmd_adapter.run_cmd` 하나를 부르므로
      전송이 바뀌어도 규약(seq 복사 · has_actual 게이트 · 예외에도 답하기)은 안 바뀐다.
    """

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
    """`ros2 run packing_robot robot_node --ros-args -p zones:="['a','b','c']"`.

    ★ Isaac(`sim_main.py`)을 **먼저** 띄워야 한다 — 뜨자마자 소켓에 붙기 때문이다.
    """
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
