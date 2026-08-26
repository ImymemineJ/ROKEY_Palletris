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
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image

from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load
from packing_common.frames import PlacePose as PlacePoseData

try:
    from packing_msgs.msg import PlacePose
    from packing_msgs.srv import (CancelPlan, PlanPlacement, ResetHeightmap,
                                  UpdateHeightmap)
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
        # ── heightmap 발행 (시각화 전용, 기본 꺼짐) ──
        # ★ 왜 sensor_msgs/Image 인가: heightmap 은 그냥 120x80 2차원 배열이다.
        #   전용 타입을 새로 만들면 packing_msgs 가 바뀌어 **팀 4명이 전부**
        #   재빌드해야 한다 (CLAUDE.md §11). Image 는 이미 §3-2 에 있는 타입이라
        #   아무도 재빌드하지 않는다. 16UC1 = 밀리미터 정수 (max_stack 1300 이므로
        #   65535 안에 넉넉히 들어간다).
        # ★ 기본값이 빈 문자열인 이유: 이 노드는 원래 아무것도 발행하지 않는다.
        #   켜는 사람만 켠다 — 끄면 예전과 완전히 같다.
        self.declare_parameter("heightmap_topic", "")
        self.hm_topic = str(self.get_parameter("heightmap_topic").value or "")
        # ★ TRANSIENT_LOCAL — 뷰어를 **나중에** 띄워도 마지막 상태를 바로 받는다.
        #   VOLATILE 이면 뷰어가 붙기 전에 놓인 박스는 화면에 영영 안 나온다
        #   (live_view 를 ROS 로 옮기면서 실측으로 확인한 함정이다).
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cb = ReentrantCallbackGroup()

        # 구역별 패커 1개씩. stage 1 은 B 만 쓰지만, 3구역 확장 때 코드가 안 바뀐다.
        self.pk = {z: HeightmapPacker(self.cfg) for z in self.cfg["zones"]["list"]}
        # ★ 토픽 이름은 **소문자**다. config 의 zones.list 는 "A"/"B"/"C" 인데
        #   토픽 규약은 소문자다 (`/zone_b/status` — orchestrator 도 .lower() 를 쓴다).
        #   여기서 안 맞추면 뷰어가 `/algo/heightmap/b` 를 듣는데 발행은
        #   `/algo/heightmap/B` 로 나가 **에러 없이 아무것도 안 온다.**
        self.hm_pub = ({z: self.create_publisher(
            Image, f"{self.hm_topic}/{z.lower()}", qos) for z in self.pk}
            if self.hm_topic else {})
        # ★ 구역마다 락 하나. 왜 필요한가:
        #   - `MultiThreadedExecutor` + `ReentrantCallbackGroup` 이라 콜백이 **동시에** 돈다.
        #   - 관제탑은 1개지만(§8) 로봇 3대를 굴리므로 A·B·C 사이클이 겹쳐 돈다.
        #   - 게다가 §3-4 가 **재시도 1회**를 규약으로 정했다 — 첫 요청이 처리 중일 때
        #     같은 구역으로 두 번째가 들어올 수 있다.
        #   plan 은 hm 을 읽고 update 는 쓴다. 겹치면 반쯤 쓰인 heightmap 위에서 자리를
        #   고른다 — 에러 없이 박스가 겹쳐 놓인다.
        #   ※ 구역이 다르면 패커 객체가 달라 서로 안 막는다.
        self.lock = {z: threading.Lock() for z in self.pk}

        self.create_service(PlanPlacement, "/algo/plan_placement", self.on_plan, callback_group=cb)
        self.create_service(UpdateHeightmap, "/algo/update_heightmap", self.on_update, callback_group=cb)
        self.create_service(CancelPlan, "/algo/cancel_plan", self.on_cancel, callback_group=cb)
        # ★ 서비스 경로는 srv 이름과 맞춘다 — CLAUDE.md §3-3 이 타입만 정하고
        #   경로를 안 적어서, 이름이 다르면 클라이언트가 조용히 못 붙는다.
        self.create_service(ResetHeightmap, "/algo/reset_heightmap", self.on_reset,
                            callback_group=cb)

        # ── 소요시간 (평가항목 "검증 및 분석" — CLAUDE.md §13) ──
        #   첫 plan 이 들어온 순간부터 잰다. 노드를 미리 띄워 두고 기다린 시간은
        #   시나리오 시간이 아니다. 구역마다 따로 잰다.
        self.t0 = {}                 # zone -> 첫 요청 시각 (perf_counter)
        self.t_last = {}             # zone -> 마지막 반영 시각

        # ── use_sim_time 점검 (§3-4 규약 1번) ──
        #   이 노드는 tf 도 /clock 도 안 쓰지만 규약이라 맞춘다. 강제로 켜지는 않는다 —
        #   사용자가 일부러 false 로 준 걸 덮으면 그게 더 찾기 어려운 버그가 된다.
        if not self.get_parameter("use_sim_time").value:
            self.get_logger().warn(
                "use_sim_time 이 false 입니다 (§3-4 규약 1번은 모든 노드에 true). "
                "--ros-args -p use_sim_time:=true 로 띄우세요")

        hm = next(iter(self.pk.values()))
        self.get_logger().info(
            f"algo_service_node 준비 — 구역 {list(self.pk)} · heightmap {hm.hm.shape} "
            f"(셀 {hm.cell}mm) · max_stack {hm.max_stack}mm")
        if self.snap:
            self.get_logger().info(
                "스냅샷 — " + " · ".join(self._snap_path(z) for z in self.pk))
        if self.hm_topic:
            self.get_logger().info(
                "heightmap 발행 — "
                + " · ".join(f"{self.hm_topic}/{z.lower()}" for z in self.pk)
                + "  (16UC1 · TRANSIENT_LOCAL)")

    # ---------- 공통 ----------
    def _snap_path(self, zone):
        """스냅샷은 **구역마다 다른 파일**이어야 한다.

        heightmap 을 3장 소유하는데(CLAUDE.md §8) 파일이 하나면 A·B·C 가 서로 덮어써
        뷰어에 아무 구역이나 번갈아 뜬다. `/tmp/algo_state.json` → `/tmp/algo_state_b.json`.
        """
        stem, ext = os.path.splitext(self.snap)
        return f"{stem}_{zone.lower()}{ext or '.json'}"

    def _elapsed(self, zone):
        """이 구역의 시나리오 경과 [s]. 첫 plan 부터 마지막 반영까지."""
        if zone not in self.t0:
            return 0.0
        return self.t_last.get(zone, time.perf_counter()) - self.t0[zone]

    def _packer(self, zone):
        """알 수 없는 구역이면 None. 조용히 잘못된 팔레트에 쌓는 것보다 낫다."""
        pk = self.pk.get(zone)
        if pk is None:
            self.get_logger().error(f"알 수 없는 zone='{zone}' — config zones.list 를 확인하세요")
        return pk

    # ---------- /algo/plan_placement ----------
    def on_plan(self, req, res):
        t0 = time.perf_counter()
        self.t0.setdefault(req.zone, t0)          # 이 구역의 시나리오 시작 시각
        pk = self._packer(req.zone)
        if pk is None:
            res.ok, res.score, res.message = False, 0.0, "NO_FEASIBLE_POSE"
            return res

        with self.lock[req.zone]:
            ok, pose, score, msg = pk.plan(req.box_id, req.size_w_mm, req.size_d_mm,
                                           req.size_h_mm)
            warn = pk.last_warning
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
        if warn:
            # ★ 치수 자체가 잘못된 경우. 그냥 NO_FEASIBLE_POSE 만 보이면 "자리가 없나"
            #   로 오해하고 계속 박스를 넣는다 — 인식/DB 쪽을 봐야 하는 상황이다.
            self.get_logger().error(f"plan  [{req.zone}] {req.box_id} — {warn}")
        if dt > 200:      # 명세서 §7.9 요구
            self.get_logger().error(f"plan 이 {dt:.0f}ms — 200ms 요구 초과")
        if msg == "PALLET_FULL":
            # 이 구역은 여기서 끝난다. reset 이 안 올 수도 있으니 여기서도 남긴다.
            self.get_logger().info(self._summary(req.zone, pk))
        return res

    # ---------- /algo/update_heightmap ----------
    def on_update(self, req, res):
        t0 = time.perf_counter()
        pk = self._packer(req.zone)
        if pk is None:
            res.ok, res.fill_rate, res.placed_count = False, 0.0, 0
            res.max_height_mm = 0
            return res

        actual = PlacePoseData.from_msg(req.actual_pose).as_dict()   # ★ 실측 pose 로 갱신
        with self.lock[req.zone]:
            ok, fill, count = pk.update(req.box_id, req.size_w_mm, req.size_d_mm,
                                        req.size_h_mm, actual)
        res.ok, res.fill_rate, res.placed_count = bool(ok), float(fill), int(count)
        # ★ 관제탑이 ZoneStatus.max_height_mm (§3-2) 를 채우는 유일한 경로다.
        #   여기서 안 주면 관제탑은 실제 더미 높이를 알 방법이 없다.
        res.max_height_mm = int(pk.hm.max())

        self.t_last[req.zone] = time.perf_counter()
        self.dump(req.zone)
        dt = (time.perf_counter() - t0) * 1000
        self.get_logger().info(
            f"update[{req.zone}] {req.box_id} → 적재 {count}개 · "
            f"적재율 {fill * 100:.1f}%(실제높이) / {pk.capacity_rate() * 100:.1f}%(용량) · "
            f"최고 {int(pk.hm.max())}mm · 경과 {self._elapsed(req.zone):.0f}s  [{dt:.1f}ms]")
        if not ok:
            # yaw 가 벗어났거나 팔레트 밖·한계 초과. 트랙 B 는 로그만 남기고 계속 진행.
            why = pk.last_warning or "실측 yaw/좌표가 계획을 벗어났습니다"
            self.get_logger().warn(f"update[{req.zone}] {req.box_id} ok=false — {why}")
        elif pk.last_warning:
            # ok=true 인데 경고가 있다 = 재시도로 들어온 중복 요청을 멱등 처리한 것.
            # 조용히 넘기면 "왜 개수가 안 늘지?" 를 나중에 못 찾는다.
            # ★ 위 warn 과 **다른 줄**이어야 한다 — rclpy 로거는 호출 지점마다
            #   심각도를 캐시해서, 같은 줄에서 warn/info 를 섞으면 죽는다 (TS-012).
            self.get_logger().info(f"update[{req.zone}] {req.box_id} — {pk.last_warning}")
        return res

    # ---------- heightmap 발행 (시각화 전용) ----------
    def _pub_hm(self, zone):
        """heightmap 을 16UC1 Image 로 내보낸다. `heightmap_topic` 이 비면 아무것도 안 한다.

        ★ 여기서 예외가 나도 서비스 응답을 막으면 안 된다. 시각화는 부가 기능이고
          적재는 계속 돌아야 한다 — 스냅샷 쓰기 실패를 warn 으로만 넘기는 것과 같다.
        """
        pub = self.hm_pub.get(zone)
        if pub is None:
            return
        try:
            hm = self.pk[zone].hm
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            # frame_id 에 구역을 실어 둔다 — 뷰어가 토픽을 잘못 잡았는지 확인할 수 있다
            msg.header.frame_id = f"pallet_{zone}"
            # ★ 이미지 규약: height=행, width=열. 우리 hm 은 (W=Px, D=Py) 이므로
            #   height=W, width=D 가 된다. 뷰어가 그대로 되읽으려면 이 순서가 중요하다.
            msg.height, msg.width = hm.shape
            msg.encoding = "16UC1"
            msg.is_bigendian = 0
            msg.step = msg.width * 2
            msg.data = hm.clip(0, 65535).astype("<u2").tobytes()
            pub.publish(msg)
        except Exception as e:      # noqa: BLE001
            self.get_logger().warn(f"heightmap 발행 실패({e}) — 시각화만 영향",
                                   once=True)

    # ---------- 시각화용 스냅샷 ----------
    def dump(self, zone):
        """지금 적재 상태를 JSON 으로 떨군다. live_view 가 이 파일을 읽어 그린다.

        원자적으로 쓴다 — 뷰어가 반쯤 쓰인 파일을 읽으면 JSON 파싱이 깨진다.
        """
        self._pub_hm(zone)
        if not self.snap:
            return
        path = self._snap_path(zone)
        pk = self.pk[zone]
        boxes = []
        for pose, w, d, h in pk.placed:
            ww, dd = (w, d) if round(pose["yaw_deg"] / 90) % 2 == 0 else (d, w)
            boxes.append([pose["x_mm"] - ww // 2, pose["y_mm"] - dd // 2, pose["z_mm"],
                          ww, dd, h, pose["yaw_deg"]])
        state = {"zone": zone, "count": len(pk.placed), "fill": pk.fill_rate(),
                 "cap": pk.capacity_rate(),          # 용량(max_stack) 기준 보조 지표
                 "elapsed_s": self._elapsed(zone),   # 평가항목 "소요시간"
                 "hmax": int(pk.hm.max()), "max_stack": pk.max_stack,
                 "pallet": [pk.W * pk.cell, pk.D * pk.cell],
                 "hm": pk.hm.tolist(), "boxes": boxes}
        try:
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(state, f)
            os.replace(tmp, path)
        except OSError as e:
            self.get_logger().warn(f"스냅샷 쓰기 실패({e}) — 시각화만 영향", once=True)

    # ---------- /algo/cancel_plan ----------
    def on_cancel(self, req, res):
        pk = self._packer(req.zone)
        if pk is not None:
            with self.lock[req.zone]:
                pk.cancel(req.box_id)
        res.ok = pk is not None
        self.get_logger().info(f"cancel[{req.zone}] {req.box_id}")
        return res

    # ---------- /algo/reset_heightmap ----------
    def on_reset(self, req, res):
        pk = self._packer(req.zone)
        if pk is not None:
            # ★ 지우기 **전에** 최종 지표를 남긴다. 팔레트를 갈면(PALLET_FULL → SwapPallet)
            #   이 판의 결과가 영영 사라진다 — 평가항목 "검증 및 분석"이 이 숫자다.
            self.get_logger().info(self._summary(req.zone, pk))
            with self.lock[req.zone]:
                pk.reset()
            self.t0.pop(req.zone, None)
            self.t_last.pop(req.zone, None)
            self.dump(req.zone)
        res.ok = pk is not None
        self.get_logger().info(f"reset [{req.zone}] — heightmap 을 0 으로 초기화")
        return res

    def _summary(self, zone, pk):
        """종료 지표 한 줄 — 적재수 · 충전율 · 최고높이 · 소요시간 (CLAUDE.md §13)."""
        el = self._elapsed(zone)
        per = f" · 박스당 {el / len(pk.placed):.1f}s" if pk.placed else ""
        return (f"═══ [{zone}] 최종 — 적재 {len(pk.placed)}개 · "
                f"적재율 {pk.fill_rate() * 100:.1f}%(실제높이) / "
                f"{pk.capacity_rate() * 100:.1f}%(용량) · "
                f"최고 {int(pk.hm.max())}mm / {pk.max_stack}mm · "
                f"소요 {el:.0f}s{per} ═══")


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
