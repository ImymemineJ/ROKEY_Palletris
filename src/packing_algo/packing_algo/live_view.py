"""실시간 적재 뷰어 — 알고리즘이 "지금 어디에 쌓고 있는지" 를 Isaac 옆에 띄운다.

기본은 **ROS 2 구독**이다 (`--source ros`). 파일 폴링(`--source file`)은 ROS 없이
혼자 확인할 때만 쓰는 대비책으로 남겨 뒀다.

    # 1) 뷰어 (관제탑·로봇이 돌고 있으면 바로 붙는다)
    ros2 run packing_algo live_view --ros-args -p use_sim_time:=true
    ros2 run packing_algo live_view --ros-args -p use_sim_time:=true -p zone:=a

    # 2) ROS 없이 (algo 를 snapshot_path 로 띄운 경우)
    ros2 run packing_algo algo_service_node --ros-args -p snapshot_path:=/tmp/algo_state.json
    python3 -m packing_algo.live_view --source file -p /tmp/algo_state.json -z b

## ROS 모드가 무엇을 듣는가

| 토픽 | 쓰는 것 |
|---|---|
| `/robot/event` (`RobotEvent`) | PLACE 성공의 **실측 pose** — 3D 배치도와 heightmap 을 여기서 재구성한다 |
| `/zone_<z>/status` (`ZoneStatus`) | 관제탑이 발표하는 **공식** 적재수·적재율·최고높이·상태 |
| `/db/get_box_info` (`GetBoxInfo`) | box_id → 치수. RobotEvent 에 치수가 없다 |
| `/algo/heightmap/<z>` (`sensor_msgs/Image`) | **algo 의 진짜 heightmap** (켜져 있을 때만) |

★ **알고리즘 노드에 새 발행을 시키지 않았다.** CLAUDE.md §8 이 "알고리즘은 로봇
  명령을 내지 않는다" 를 정했고, 시각화를 위해 전용 토픽을 새로 파면 `packing_msgs`
  가 바뀌어 팀 4명이 전부 재빌드해야 한다 (§11). 이미 있는 인터페이스만 쓴다.

★ heightmap 은 두 경로가 있다.

  1. **algo 가 직접 보내 준 것** — algo 를 `-p heightmap_topic:=/algo/heightmap` 으로
     띄우면 16UC1 `sensor_msgs/Image` 로 나온다. **이게 진짜다.**
     전용 msg 를 새로 만들지 않은 이유: heightmap 은 그냥 120×80 2차원 배열이고,
     `packing_msgs` 를 건드리면 팀 4명이 전부 재빌드해야 한다 (§11).
     `TRANSIENT_LOCAL` 이라 **뷰어를 늦게 띄워도 마지막 상태를 바로 받는다.**
  2. **여기서 다시 계산한 것** — `/robot/event` 의 실측 pose 로 재구성한다.
     algo 발행이 꺼져 있으면 이쪽만 쓴다.

  둘이 다 있으면 **1을 그리고 2와 대조한다.** 어긋나면 화면에 경고를 띄운다 —
  `UpdateHeightmap` 에 계획값이 들어갔거나 이벤트를 놓쳤다는 뜻이다 (§3-4 규약 3).
  화면 제목에 지금 무엇을 보고 있는지("algo 발행" / "재구성")를 적는다.

## 알려진 한계

1. **`/robot/event` 는 지난 이벤트를 다시 주지 않는다.** 뷰어를 로봇보다 늦게 띄우면
   그 전에 놓인 박스는 화면에 없다. 관제탑의 `placed_count` 와 대조해 화면에 경고를 낸다.
2. `RobotEvent` 에 **zone 필드가 없다** (§3-2). 그래서 여러 구역이 동시에 도는 동안에는
어느 팔레트의 배치인지 이 토픽만으로 구분할 수 없다. 지금은 선택한 구역의 것으로
간주하고, 관제탑의 `placed_count` 와 어긋나면 화면에 경고를 낸다.
→ 3구역 동시 시연을 하려면 `RobotEvent` 에 zone 을 넣는 논의가 필요하다 (§15 로 올릴 것).
"""
import argparse
import json
import os
import threading
import time

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

for _name in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic", "Malgun Gothic"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _name
        break
plt.rcParams["axes.unicode_minus"] = False

COLORS = {210: "#4C9AFF", 280: "#FFAB00", 340: "#36B37E"}    # 3호/4호/5호


def read(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


# ══════════════════════════════════════════════════════════════════
#  ROS 소스
# ══════════════════════════════════════════════════════════════════
def make_ros_source(zone, argv):
    """rclpy 노드를 별도 스레드에서 돌리고, `snapshot()` 하나를 돌려준다.

    ★ rclpy 를 여기서 import 하는 이유: `--source file` 은 ROS 없이 돌아야 한다.
      모듈 최상단에서 import 하면 ROS 가 없는 PC 에서 파일 모드까지 죽는다.
    """
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from sensor_msgs.msg import Image

    from packing_msgs.msg import RobotCmd, RobotEvent, ZoneStatus
    from packing_msgs.srv import GetBoxInfo

    from packing_algo.heightmap_core import HeightmapPacker
    from packing_common.config import load

    class LiveViewNode(Node):
        def __init__(self):
            super().__init__("live_view")
            self.declare_parameter("zone", zone)
            self.zone = str(self.get_parameter("zone").value).lower()
            # algo 가 heightmap 을 내보내면 그걸 쓴다 (기본 경로). 빈 문자열이면 안 듣는다.
            self.declare_parameter("heightmap_topic", "/algo/heightmap")
            self.hm_topic = str(self.get_parameter("heightmap_topic").value or "")
            self.cfg = load()
            self.pk = HeightmapPacker(self.cfg)
            # ★ 콜백이 동시에 돈다 (MultiThreadedExecutor). pk 는 읽기/쓰기가 겹치므로
            #   락으로 감싼다 — algo_service_node 가 구역마다 락을 두는 것과 같은 이유다.
            self.lock = threading.Lock()
            self.t0 = None
            self.official = None          # 관제탑이 발표한 ZoneStatus (없을 수도 있다)
            self.notes = []               # 화면에 띄울 경고
            self.seen = set()             # 이미 반영한 (seq, box_id) — 중복 이벤트 방지
            self.pending = {}             # box_id -> 치수 조회 중인 이벤트
            self.algo_hm = None           # algo 가 보내 준 **진짜** heightmap

            cb = ReentrantCallbackGroup()
            self.create_subscription(RobotEvent, "/robot/event", self.on_event, 10,
                                     callback_group=cb)
            self.create_subscription(ZoneStatus, f"/zone_{self.zone}/status",
                                     self.on_status, 10, callback_group=cb)
            self.cli = self.create_client(GetBoxInfo, "/db/get_box_info",
                                          callback_group=cb)
            if self.hm_topic:
                # ★ TRANSIENT_LOCAL 로 맞춰야 한다. 발행 쪽이 TRANSIENT_LOCAL 인데
                #   VOLATILE 로 구독하면 **연결 자체가 안 된다** — 토픽은 보이는데
                #   메시지가 하나도 안 오는, 에러 없는 실패가 된다.
                qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(Image, f"{self.hm_topic}/{self.zone}",
                                         self.on_hm, qos, callback_group=cb)
            self.PLACE = RobotCmd.PLACE
            self.get_logger().info(
                f"live_view — zone {self.zone} · /robot/event 와 /zone_{self.zone}/status 를 듣는다")
            if not self.get_parameter("use_sim_time").value:
                # §3-4 규약 1. 강제로 켜지 않는다 — 켜지 않은 채 돌리는 실험이 있다.
                self.get_logger().warn("use_sim_time 이 false 다 (§3-4 규약 1)")

        # ---------- /zone_<z>/status ----------
        def on_status(self, msg):
            with self.lock:
                prev = self.official
                self.official = msg
                # 관제탑이 세는 적재수가 줄었다 = 팔레트를 갈았다. 우리도 지운다.
                if prev is not None and msg.placed_count < prev.placed_count:
                    self.pk.reset()
                    self.seen.clear()
                    self.t0 = None

        # ---------- /algo/heightmap/<z> ----------
        def on_hm(self, msg):
            """algo 의 heightmap 을 그대로 받는다 (16UC1 Image).

            ★ 재구성보다 이쪽이 **진짜**다. 다만 재구성을 버리지 않는다 —
              둘을 대조하는 것이 이 화면의 진단 가치이기 때문이다. 어긋나면
              `UpdateHeightmap` 에 계획값이 들어갔거나 이벤트를 놓친 것이다.
            """
            try:
                if msg.encoding != "16UC1":
                    self._note(f"heightmap encoding 이 {msg.encoding} 다 — 16UC1 이어야 한다")
                    return
                a = np.frombuffer(msg.data, dtype="<u2")
                # 발행 쪽이 height=W(Px) · width=D(Py) 로 넣었다 (algo_service_node._pub_hm)
                self.algo_hm = a.reshape(msg.height, msg.width).astype(np.int32)
            except Exception as e:      # noqa: BLE001 — 콜백은 절대 안 죽는다
                self._note(f"heightmap 수신 실패: {e}")

        # ---------- /robot/event ----------
        def on_event(self, ev):
            # ★ 구독 콜백은 입력을 검증한다. rclpy 는 콜백 예외를 삼키므로,
            #   잘못된 메시지 하나에 화면이 영원히 안 갱신되는데 에러는 안 뜬다 (§9).
            try:
                if ev.action != self.PLACE or not ev.ok or not ev.has_actual:
                    return
                key = (int(ev.seq), str(ev.box_id))
                with self.lock:
                    if key in self.seen or ev.box_id in self.pending:
                        return                       # 재시도로 같은 이벤트가 두 번 올 수 있다
                    self.pending[ev.box_id] = key
                    if self.t0 is None:
                        self.t0 = time.perf_counter()
                if not self.cli.service_is_ready():
                    self._note("db/get_box_info 가 없다 — 치수를 몰라 3D 를 못 그린다")
                    with self.lock:
                        self.pending.pop(ev.box_id, None)
                    return
                req = GetBoxInfo.Request()
                req.box_id = ev.box_id
                # ★ call_async + add_done_callback. spin_until_future_complete 를 쓰면
                #   executor 가 자기 자신을 기다려 **에러 없이 조용히 멈춘다** (§9).
                fut = self.cli.call_async(req)
                fut.add_done_callback(lambda f, e=ev: self._on_info(f, e))
            except Exception as e:                   # noqa: BLE001 — 콜백은 절대 안 죽는다
                self._note(f"이벤트 처리 실패: {e}")

        def _on_info(self, fut, ev):
            try:
                res = fut.result()
            except Exception as e:                   # noqa: BLE001
                self._note(f"get_box_info 실패: {e}")
                with self.lock:
                    self.pending.pop(ev.box_id, None)
                return
            with self.lock:
                key = self.pending.pop(ev.box_id, None)
                if key is None:
                    return
                if not res.found:
                    self._note(f"{ev.box_id}: DB 에 없다 — 건너뛴다", locked=True)
                    return
                i = res.info
                w, d, h = int(i.size_w_mm), int(i.size_d_mm), int(i.size_h_mm)
                actual = dict(x_mm=int(ev.actual_x_mm), y_mm=int(ev.actual_y_mm),
                              z_mm=int(ev.actual_z_mm),
                              yaw_deg=float(ev.actual_yaw_deg))
                # ★ plan 없이 update 를 부른다. 코어는 plans 에 없으면 actual 만으로
                #   반영한다 — 뷰어는 계획을 만든 적이 없으므로 이 경로가 맞다.
                self.pk.update(ev.box_id, w, d, h, actual)
                self.seen.add(key)
                if self.pk.last_warning:
                    self._note(self.pk.last_warning, locked=True)

        def _note(self, msg, locked=False):
            def put():
                if msg not in self.notes:
                    self.notes.append(msg)
                    del self.notes[:-3]              # 최근 3개만
            if locked:
                put()
            else:
                with self.lock:
                    put()

        # ---------- 그리기용 스냅샷 ----------
        def snapshot(self):
            """algo_service_node.dump() 와 **같은 모양**의 dict.

            같은 모양이라야 draw() 가 두 소스를 구분하지 않아도 된다 — 화면 코드를
            두 벌로 두면 한쪽만 고쳐지는 사고가 난다.
            """
            with self.lock:
                pk, off = self.pk, self.official
                boxes = []
                for pose, w, d, h in pk.placed:
                    ww, dd = ((w, d) if round(pose["yaw_deg"] / 90) % 2 == 0 else (d, w))
                    boxes.append([pose["x_mm"] - ww // 2, pose["y_mm"] - dd // 2,
                                  pose["z_mm"], ww, dd, h, pose["yaw_deg"]])
                notes = list(self.notes)
                n = len(pk.placed)
                # ★ algo 가 보내 준 heightmap 이 있으면 **그게 진짜**다. 재구성은
                #   버리지 않고 대조에만 쓴다 — 둘이 어긋나는 것이 곧 버그 신호다.
                hm, hm_src = pk.hm, "재구성"
                if self.algo_hm is not None:
                    if self.algo_hm.shape != pk.hm.shape:
                        notes.append(f"algo heightmap 크기 {self.algo_hm.shape} 가 "
                                     f"설정 {pk.hm.shape} 와 다르다 — config 불일치")
                    else:
                        hm, hm_src = self.algo_hm, "algo 발행"
                        gap = int(np.abs(self.algo_hm - pk.hm).max())
                        if gap > pk.cell:
                            notes.append(
                                f"algo heightmap 과 재구성이 최대 {gap}mm 어긋난다 — "
                                "UpdateHeightmap 에 계획값이 들어갔거나 이벤트를 놓쳤다")
                # 재구성과 공식 집계가 어긋났다면 숨기지 않고 화면에 띄운다.
                # ★ 단, **치수 조회가 도는 중**이면 어긋나는 게 정상이다 —
                #   get_box_info 왕복 동안 잠깐 하나 모자란다. 그때까지 경고를 내면
                #   박스마다 한 번씩 깜빡여서 진짜 경고를 못 알아본다.
                if off is not None and not self.pending and off.placed_count != n:
                    notes.append(
                        f"관제탑 적재수 {off.placed_count} ≠ 재구성 {n} — "
                        "뷰어를 늦게 띄웠거나(그전 이벤트는 못 받는다), "
                        "여러 구역이 같이 도는 중이다(RobotEvent 에 zone 이 없다)")
                return {"zone": self.zone, "count": n, "fill": pk.fill_rate(),
                        "cap": pk.capacity_rate(),
                        "elapsed_s": 0.0 if self.t0 is None else time.perf_counter() - self.t0,
                        "hmax": int(hm.max()), "max_stack": pk.max_stack,
                        "pallet": [pk.W * pk.cell, pk.D * pk.cell],
                        "hm": hm, "hm_src": hm_src, "boxes": boxes,
                        "state": "" if off is None else off.state,
                        "notes": notes[-3:]}

    rclpy.init(args=argv)
    node = LiveViewNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    # ★ matplotlib 은 메인 스레드에서만 그릴 수 있다. 그래서 spin 을 옆으로 보낸다.
    #   반대로 하면(그리기를 콜백에서) Tk/Qt 가 스레드 위반으로 죽는다.
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def shutdown():
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    return node.snapshot, shutdown


# ══════════════════════════════════════════════════════════════════
#  그리기
# ══════════════════════════════════════════════════════════════════
def draw(fig, axes, st, fills):
    ax3d, ax_hm, ax_fr = axes
    W, D = st["pallet"]
    ms = st["max_stack"]

    ax3d.clear()
    for x, y, z, w, d, h, _yaw in st["boxes"]:
        ax3d.bar3d(x, y, z, w, d, h, color=COLORS.get(h, "#8993A4"),
                   edgecolor="#172B4D", linewidth=0.4, alpha=0.92, shade=True)
    ax3d.set_xlim(0, W); ax3d.set_ylim(0, D); ax3d.set_zlim(0, ms)
    ax3d.set_box_aspect((W, D, ms))
    ax3d.set_xlabel("Px (mm) — 컨베이어 쪽이 0")
    ax3d.set_ylabel("Py (mm) — 로봇이 0 쪽")
    ax3d.set_zlabel("Pz (mm)")
    ax3d.view_init(elev=24, azim=-58)

    ax_hm.clear()
    hm = np.asarray(st["hm"])
    ax_hm.imshow(hm.T, origin="lower", cmap="viridis", vmin=0, vmax=ms,
                 extent=[0, W, 0, D], aspect="equal")
    # 어느 쪽 heightmap 을 보고 있는지 제목에 밝힌다 — "진짜냐 재구성이냐" 를
    # 모르면 화면을 믿을 수도 의심할 수도 없다.
    ax_hm.set_title(f"heightmap (mm) — {st.get('hm_src', '재구성')}", fontsize=9)
    ax_hm.set_xlabel("Px"); ax_hm.set_ylabel("Py")

    # 적재율 두 가지를 같이 그린다 — 기준이 다르면 곡선 모양이 아주 다르다.
    #   실제높이 기준(정본 §7.8): 층이 올라가는 순간 분모가 뛰어 **뚝 떨어진다**
    #   용량 기준(보조)         : 분모가 max_stack 고정이라 **단조 증가**
    # 하나만 보면 "왜 적재율이 내려가지?" 를 계속 설명해야 한다.
    ax_fr.clear()
    xs = range(1, len(fills) + 1)
    ax_fr.plot(xs, [f * 100 for f, _ in fills],
               color="#2a78d6", lw=2, marker="o", ms=3, label="실제 높이 기준 (§7.8)")
    ax_fr.plot(xs, [c * 100 for _, c in fills],
               color="#eb6834", lw=2, marker="o", ms=3, label=f"용량 기준 ({ms}mm)")
    ax_fr.axhline(70, color="#52514E", lw=1.2, ls=(0, (5, 4)))
    ax_fr.text(0.6, 71, "목표 70%", color="#52514E", fontsize=8)
    ax_fr.set_xlim(0.5, max(2, len(fills) + 0.5)); ax_fr.set_ylim(0, 85)
    ax_fr.set_title("적재율 vs 박스 수", fontsize=9)
    ax_fr.set_xlabel("적재된 박스 수"); ax_fr.grid(alpha=0.3)
    ax_fr.legend(fontsize=7, loc="lower right", framealpha=0.9)

    el = st.get("elapsed_s", 0.0)
    state = st.get("state") or ""
    fig.suptitle(f"zone {st['zone']}  ·  {state + '  ·  ' if state else ''}"
                 f"적재 {st['count']}개  ·  "
                 f"적재율 {st['fill'] * 100:.1f}% (실제높이) / "
                 f"{st.get('cap', 0.0) * 100:.1f}% (용량)  ·  "
                 f"최고 {st['hmax']}mm / {ms}mm  ·  소요 {el:.0f}s", fontsize=13)

    # 경고는 숨기지 않는다 — 조용히 틀린 그림을 보여주는 것이 제일 나쁘다.
    for t in list(getattr(fig, "_notes", [])):
        t.remove()
    fig._notes = []
    for k, msg in enumerate(st.get("notes") or []):
        fig._notes.append(fig.text(0.5, 0.035 + 0.028 * k, "⚠ " + msg, ha="center",
                                   fontsize=8.5, color="#B54708"))


def main():
    ap = argparse.ArgumentParser(description="실시간 적재 뷰어")
    ap.add_argument("--source", choices=("ros", "file"), default="ros",
                    help="ros = /robot/event 구독 (기본) · file = 스냅샷 JSON 폴링")
    ap.add_argument("-p", "--path", default="/tmp/algo_state.json",
                    help="file 모드에서만. algo 에 준 snapshot_path 와 같은 값")
    ap.add_argument("-z", "--zone", default="b", help="볼 구역 (a|b|c)")
    ap.add_argument("--save", default="", help="갱신마다 PNG 를 남길 디렉터리")
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--headless", action="store_true", help="창 없이 --save 만 (원격용)")
    # ★ `ros2 run ... --ros-args -p x:=y` 를 그대로 받기 위해 남은 인자를 흘려보낸다.
    args, ros_argv = ap.parse_known_args()

    if args.headless:
        matplotlib.use("Agg")
    if args.save:
        os.makedirs(args.save, exist_ok=True)

    snapshot = shutdown = None
    if args.source == "ros":
        snapshot, shutdown = make_ros_source(args.zone, ros_argv or None)
        print("[live_view] ROS 모드 — /robot/event 를 기다립니다. Ctrl+C 로 종료.")
    else:
        # ★ algo 가 구역마다 다른 파일에 쓴다 (heightmap 3장 · CLAUDE.md §8).
        #   여기서 같은 규칙으로 경로를 만든다 — 사용자가 두 곳에 같은 규칙을 손으로
        #   맞추게 하면 반드시 어긋난다.
        stem, ext = os.path.splitext(args.path)
        if not stem.endswith(f"_{args.zone.lower()}"):
            args.path = f"{stem}_{args.zone.lower()}{ext or '.json'}"
        print(f"[live_view] 파일 모드 — {args.path} 를 지켜봅니다. Ctrl+C 로 종료.")

    fig = plt.figure(figsize=(13, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1], hspace=0.45, wspace=0.25,
                          left=0.03, right=0.96, top=0.90, bottom=0.13)
    axes = (fig.add_subplot(gs[:, 0], projection="3d"),
            fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1]))
    if not args.headless:
        plt.ion(); plt.show(block=False)

    fills, last_count, last_mtime, shot = [], -1, 0.0, 0
    last_notes = None
    try:
        while True:
            st = None
            if args.source == "ros":
                st = snapshot()
            else:
                try:
                    mtime = os.path.getmtime(args.path)
                except OSError:
                    mtime = 0.0
                if mtime and mtime != last_mtime:
                    st = read(args.path)
                    if st:
                        last_mtime = mtime
            if st:
                if st["count"] < last_count:          # reset 됐다 — 곡선도 새로
                    fills = []
                # 옛 스냅샷(cap 없음)도 읽히게 — 없으면 fill 로 대신한다
                pair = (st["fill"], st.get("cap", st["fill"]))
                changed = st["count"] != last_count
                if changed:
                    fills.append(pair)
                    last_count = st["count"]
                # ★ ROS 모드는 매 틱 스냅샷이 오므로, **바뀐 때만** 다시 그린다.
                #   안 그러면 박스가 안 늘어도 0.5초마다 3D 를 통째로 다시 그려
                #   CPU 를 먹고 창이 버벅인다.
                # ★ 경고가 바뀐 것도 '바뀐 것'이다. 박스가 안 늘 때만 나는 경고
                #   (db 없음 등)가 있어서, 개수만 보면 그 경고가 영영 안 뜬다.
                notes_sig = (tuple(st.get("notes") or ()), st.get("hm_src"),
                             st.get("hmax"))
                if changed or notes_sig != last_notes or args.source == "file":
                    last_notes = notes_sig
                    draw(fig, axes, st, fills or [pair])
                    if args.save:
                        shot += 1
                        fig.savefig(os.path.join(args.save, f"{shot:04d}.png"), dpi=110)
                    if not args.headless:
                        fig.canvas.draw_idle()
            if args.headless:
                time.sleep(args.interval)
            else:
                plt.pause(args.interval)
    except KeyboardInterrupt:
        print(f"\n[live_view] 종료 — {shot}장 저장" if args.save else "\n[live_view] 종료")
    finally:
        if shutdown:
            shutdown()


if __name__ == "__main__":
    main()
