"""C5 — `/algo/*` ROS2 서비스 래퍼 (트랙 C).

**코어(`heightmap_core.HeightmapPacker`)는 한 줄도 건드리지 않는다.** 이 노드가 하는 일은
ROS 메시지 ↔ 파이썬 자료구조 **번역**과 구역별 패커 인스턴스 관리뿐이다.

철칙:
- 콜백 안에서 `spin_until_future_complete` 를 쓰지 않는다. 이 노드는 서비스를 부르지 않고
  받기만 하지만, 규약대로 `ReentrantCallbackGroup` + `MultiThreadedExecutor` 로 띄운다.
  (여러 구역 요청이 동시에 와도 서로를 막지 않는다.)
- 모든 수치는 `config/system.yaml` 에서 읽는다. 코드에 상수를 박지 않는다.
- 계획은 `box_id` 로 관리한다. 픽 실패·타임아웃 등 모든 이탈 경로에서 트랙 B 가
  `cancel_plan` 을 부르는 것을 전제로 설계돼 있다.

실행:
    ros2 run packing_algo algo_service_node
"""
import json
import os
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load
from packing_common.frames import PlacePose as PlacePoseData

try:
    from packing_msgs.msg import PlacePose
    from packing_msgs.srv import CancelPlan, PlanPlacement, ResetHeightmap, UpdateHeightmap
except ImportError as e:      # 트랙 D 의 packing_msgs 가 아직 빌드되지 않은 경우
    raise SystemExit(
        f"packing_msgs 를 import 하지 못했습니다: {e}\n"
        "  → 트랙 D 가 packing_msgs 를 push 했는지 확인하고,\n"
        "     colcon build --symlink-install && source install/setup.bash 를 다시 하세요."
    )


class AlgoService(Node):
    def __init__(self):
        super().__init__("algo_service_node")
        self.cfg = load()
        # 실시간 시각화용 상태 덤프. 빈 문자열이면 끈다 — 인터페이스가 아니라 파일이라
        # ROS 계약(D2: algo 는 아무것도 발행하지 않는다)을 건드리지 않는다.
        self.declare_parameter("snapshot_path", "")
        self.snap = os.path.expanduser(self.get_parameter("snapshot_path").value or "")
        cb = ReentrantCallbackGroup()

        # 구역별 패커 1개씩. stage 1 은 B 만 쓰지만, 3구역 확장 때 코드가 안 바뀐다.
        self.pk = {z: HeightmapPacker(self.cfg) for z in self.cfg["zones"]["list"]}

        self.create_service(PlanPlacement, "/algo/plan_placement", self.on_plan, callback_group=cb)
        self.create_service(UpdateHeightmap, "/algo/update_heightmap", self.on_update, callback_group=cb)
        self.create_service(CancelPlan, "/algo/cancel_plan", self.on_cancel, callback_group=cb)
        self.create_service(ResetHeightmap, "/algo/reset", self.on_reset, callback_group=cb)

        hm = next(iter(self.pk.values()))
        self.get_logger().info(
            f"algo_service_node 준비 — 구역 {list(self.pk)} · heightmap {hm.hm.shape} "
            f"(셀 {hm.cell}mm) · max_stack {hm.max_stack}mm")

    # ---------- 공통 ----------
    def _packer(self, zone):
        """알 수 없는 구역이면 None. 조용히 잘못된 팔레트에 쌓는 것보다 낫다."""
        pk = self.pk.get(zone)
        if pk is None:
            self.get_logger().error(f"알 수 없는 zone='{zone}' — config zones.list 를 확인하세요")
        return pk

    # ---------- /algo/plan_placement ----------
    def on_plan(self, req, res):
        t0 = time.perf_counter()
        pk = self._packer(req.zone)
        if pk is None:
            res.ok, res.score, res.message = False, 0.0, "NO_FEASIBLE_POSE"
            return res

        ok, pose, score, msg = pk.plan(req.box_id, req.size_w_mm, req.size_d_mm, req.size_h_mm)
        res.ok, res.score, res.message = bool(ok), float(score), msg
        if ok:
            res.pose = PlacePoseData.from_dict(pose).to_msg(PlacePose)

        dt = (time.perf_counter() - t0) * 1000
        line = (f"plan  [{req.zone}] {req.box_id} "
                f"{req.size_w_mm}x{req.size_d_mm}x{req.size_h_mm} → {msg}"
                + (f" ({pose['x_mm']},{pose['y_mm']}) z={pose['z_mm']} "
                   f"yaw={pose['yaw_deg']:.0f}" if ok else "")
                + f"  [{dt:.1f}ms]")
        # ★ rclpy 로거는 호출 지점(파일·줄)마다 심각도를 캐시한다. 같은 줄에서 info 와 warn 을
        #   번갈아 부르면 ValueError: Logger severity cannot be changed between calls 로 죽는다.
        #   삼항연산자로 로거 메서드를 고르지 말고, 줄을 갈라서 각각 부른다.
        if ok:
            self.get_logger().info(line)
        else:
            self.get_logger().warn(line)
        if dt > 200:      # 명세서 §7.9 요구
            self.get_logger().error(f"plan 이 {dt:.0f}ms — 200ms 요구 초과")
        return res

    # ---------- /algo/update_heightmap ----------
    def on_update(self, req, res):
        t0 = time.perf_counter()
        pk = self._packer(req.zone)
        if pk is None:
            res.ok, res.fill_rate, res.placed_count = False, 0.0, 0
            return res

        actual = PlacePoseData.from_msg(req.actual_pose).as_dict()   # ★ 실측 pose 로 갱신
        ok, fill, count = pk.update(req.box_id, req.size_w_mm, req.size_d_mm,
                                    req.size_h_mm, actual)
        res.ok, res.fill_rate, res.placed_count = bool(ok), float(fill), int(count)

        self.dump(req.zone)
        dt = (time.perf_counter() - t0) * 1000
        self.get_logger().info(
            f"update[{req.zone}] {req.box_id} → 적재 {count}개 · 적재율 {fill * 100:.1f}% · "
            f"최고 {int(pk.hm.max())}mm  [{dt:.1f}ms]")
        if not ok:
            # yaw 가 벗어났거나 팔레트 밖·한계 초과. 트랙 B 는 로그만 남기고 계속 진행.
            why = pk.last_warning or "실측 yaw/좌표가 계획을 벗어났습니다"
            self.get_logger().warn(f"update[{req.zone}] {req.box_id} ok=false — {why}")
        return res

    # ---------- 시각화용 스냅샷 ----------
    def dump(self, zone):
        """지금 적재 상태를 JSON 으로 떨군다. live_view 가 이 파일을 읽어 그린다.

        원자적으로 쓴다 — 뷰어가 반쯤 쓰인 파일을 읽으면 JSON 파싱이 깨진다.
        """
        if not self.snap:
            return
        pk = self.pk[zone]
        boxes = []
        for pose, w, d, h in pk.placed:
            ww, dd = (w, d) if round(pose["yaw_deg"] / 90) % 2 == 0 else (d, w)
            boxes.append([pose["x_mm"] - ww // 2, pose["y_mm"] - dd // 2, pose["z_mm"],
                          ww, dd, h, pose["yaw_deg"]])
        state = {"zone": zone, "count": len(pk.placed), "fill": pk.fill_rate(),
                 "cap": pk.capacity_rate(),          # 용량(max_stack) 기준 보조 지표
                 "hmax": int(pk.hm.max()), "max_stack": pk.max_stack,
                 "pallet": [pk.W * pk.cell, pk.D * pk.cell],
                 "hm": pk.hm.tolist(), "boxes": boxes}
        try:
            tmp = self.snap + ".tmp"
            with open(tmp, "w") as f:
                json.dump(state, f)
            os.replace(tmp, self.snap)
        except OSError as e:
            self.get_logger().warn(f"스냅샷 쓰기 실패({e}) — 시각화만 영향", once=True)

    # ---------- /algo/cancel_plan ----------
    def on_cancel(self, req, res):
        pk = self._packer(req.zone)
        if pk is not None:
            pk.cancel(req.box_id)
        res.ok = pk is not None
        self.get_logger().info(f"cancel[{req.zone}] {req.box_id}")
        return res

    # ---------- /algo/reset ----------
    def on_reset(self, req, res):
        pk = self._packer(req.zone)
        if pk is not None:
            pk.reset()
            self.dump(req.zone)
        res.ok = pk is not None
        self.get_logger().info(f"reset [{req.zone}] — heightmap 을 0 으로 초기화")
        return res


def main(args=None):
    rclpy.init(args=args)
    node = AlgoService()
    executor = MultiThreadedExecutor()          # ★ 철칙 4
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
