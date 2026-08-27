"""실시간 적재 뷰어 — 세 구역이 "지금 어디에 쌓고 있는지" 를 Isaac 옆에 띄운다.

기본은 **ROS 2 구독**이고 **3구역 동시**다 (`-z all`). 파일 폴링(`--source file`)은
ROS 없이 혼자 확인할 때만 쓰는 대비책으로 남겨 뒀다.

    # 1) 뷰어 (관제탑·로봇이 돌고 있으면 바로 붙는다) — 3구역 전부
    ros2 run packing_algo live_view --ros-args -p use_sim_time:=true
    # 한 구역만
    ros2 run packing_algo live_view -z b --ros-args -p use_sim_time:=true

    # 2) ROS 없이 (algo 를 snapshot_path 로 띄운 경우)
    python3 -m packing_algo.live_view --source file -p /tmp/algo_state.json -z b

## ROS 모드가 무엇을 듣는가

| 토픽 | 쓰는 것 |
|---|---|
| `/zone_<z>/robot/event` (`RobotEvent`) | PLACE 성공의 **실측 pose** — 3D 배치도와 heightmap 재구성 |
| `/zone_<z>/status` (`ZoneStatus`) | 관제탑이 발표하는 **공식** 적재수·적재율·상태 |
| `/db/get_box_info` (`GetBoxInfo`) | box_id → 치수. RobotEvent 에 치수가 없다 |
| `/algo/heightmap/<z>` (`sensor_msgs/Image`) | **algo 의 진짜 heightmap** (켜져 있을 때만) |

★★ **`/robot/event`(전역)를 듣지 않는다** (2026-08-27 수정). §3-2 표는 전역 단수형인데
  실제 Isaac 은 `/zone_{a|b|c}/robot/event` 로 낸다 — §15 "`/robot/cmd` 네임스페이스"
  와 같은 사연이다. 전역을 구독하면 발행자 0 이라 **화면이 영영 빈 채로 남는다**
  (에러도 안 난다 — 실측으로 밟았다). 구역별 토픽은 덤으로 §15 의
  "RobotEvent 에 zone 이 없다" 문제를 풀어 준다: **토픽 이름이 구역을 알려준다.**

★ **알고리즘 노드에 새 발행을 시키지 않았다.** CLAUDE.md §8 이 "알고리즘은 로봇
  명령을 내지 않는다" 를 정했고, 전용 토픽을 새로 파면 `packing_msgs` 가 바뀌어
  팀 4명이 전부 재빌드해야 한다 (§11). 이미 있는 인터페이스만 쓴다.

★ heightmap 은 두 경로가 있다.
  1. **algo 가 직접 보내 준 것** — algo 를 `-p heightmap_topic:=/algo/heightmap` 으로
     띄우면 16UC1 `sensor_msgs/Image` 로 나온다. TRANSIENT_LOCAL 이라 늦게 붙어도
     마지막 상태를 바로 받는다. **이게 진짜다.**
  2. **여기서 다시 계산한 것** — `/zone_*/robot/event` 의 실측 pose 로 재구성한다.
  둘이 다 있으면 1 을 그리고 2 와 대조한다. 어긋나면 화면에 경고를 띄운다.

## 알려진 한계

**`/zone_*/robot/event` 는 지난 이벤트를 다시 주지 않는다.** 뷰어를 로봇보다 늦게
띄우면 그 전에 놓인 박스는 화면에 없다 (algo heightmap 발행이 켜져 있으면 heightmap
만은 최신이 온다). 관제탑 `placed_count` 와 대조해 화면에 경고를 낸다.
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
ZONES_ALL = ("a", "b", "c")


def read(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


# ══════════════════════════════════════════════════════════════════
#  ROS 소스 — 구역 여러 개를 한 노드가 듣는다
# ══════════════════════════════════════════════════════════════════
def make_ros_source(zones, argv):
    """rclpy 노드를 별도 스레드에서 돌리고, `snapshot()` 하나를 돌려준다.

    ★ rclpy 를 여기서 import 하는 이유: `--source file` 은 ROS 없이 돌아야 한다.
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

    class _ZoneState:
        """구역 하나의 재구성 상태. 노드 락 하나로 전 구역을 지킨다 —
        구역마다 락을 두면 스냅샷이 구역 간에 찢어진 순간을 찍는다."""

        def __init__(self, cfg):
            self.pk = HeightmapPacker(cfg)
            self.official = None      # 관제탑 ZoneStatus
            self.seen = set()         # (seq, box_id) — 중복 이벤트 방지
            self.algo_hm = None       # algo 가 보내 준 진짜 heightmap
            self.t0 = None

    class LiveViewNode(Node):
        def __init__(self):
            super().__init__("live_view")
            self.declare_parameter("heightmap_topic", "/algo/heightmap")
            self.hm_topic = str(self.get_parameter("heightmap_topic").value or "")
            self.cfg = load()
            self.lock = threading.Lock()
            self.zs = {z: _ZoneState(self.cfg) for z in zones}
            self.notes = []               # 화면 경고 (구역 접두어 포함)
            self.pending = {}             # box_id -> (zone, key) 치수 조회 중

            cb = ReentrantCallbackGroup()
            for z in zones:
                # ★ 기본 인자 z=z 로 구역을 캡처한다 — 루프 변수를 그냥 쓰면
                #   콜백 셋이 전부 마지막 구역으로 붙는 고전적 사고가 난다.
                self.create_subscription(
                    RobotEvent, f"/zone_{z}/robot/event",
                    lambda ev, z=z: self.on_event(ev, z), 10, callback_group=cb)
                self.create_subscription(
                    ZoneStatus, f"/zone_{z}/status",
                    lambda m, z=z: self.on_status(m, z), 10, callback_group=cb)
                if self.hm_topic:
                    # ★ TRANSIENT_LOCAL 로 맞춰야 한다. VOLATILE 로 구독하면
                    #   연결 자체가 안 되는, 에러 없는 실패가 된다.
                    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                     reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                    self.create_subscription(
                        Image, f"{self.hm_topic}/{z}",
                        lambda m, z=z: self.on_hm(m, z), qos, callback_group=cb)
            self.cli = self.create_client(GetBoxInfo, "/db/get_box_info",
                                          callback_group=cb)
            self.PLACE = RobotCmd.PLACE
            self.get_logger().info(
                "live_view — 구역 %s · /zone_*/robot/event 를 듣는다" % ",".join(zones))
            if not self.get_parameter("use_sim_time").value:
                self.get_logger().warn("use_sim_time 이 false 다 (§3-4 규약 1)")

        # ---------- /zone_<z>/status ----------
        def on_status(self, msg, z):
            with self.lock:
                s = self.zs[z]
                prev, s.official = s.official, msg
                # 관제탑이 세는 적재수가 줄었다 = 팔레트를 갈았다. 우리도 지운다.
                if prev is not None and msg.placed_count < prev.placed_count:
                    s.pk.reset()
                    s.seen.clear()
                    s.t0 = None

        # ---------- /algo/heightmap/<z> ----------
        def on_hm(self, msg, z):
            try:
                if msg.encoding != "16UC1":
                    self._note(f"[{z}] heightmap encoding {msg.encoding} — 16UC1 이어야 한다")
                    return
                a = np.frombuffer(msg.data, dtype="<u2")
                with self.lock:
                    self.zs[z].algo_hm = a.reshape(msg.height, msg.width).astype(np.int32)
            except Exception as e:      # noqa: BLE001 — 콜백은 절대 안 죽는다
                self._note(f"[{z}] heightmap 수신 실패: {e}")

        # ---------- /zone_<z>/robot/event ----------
        def on_event(self, ev, z):
            # ★ 구독 콜백은 입력을 검증한다 (§9) — rclpy 는 콜백 예외를 삼킨다.
            try:
                if ev.action != self.PLACE or not ev.ok or not ev.has_actual:
                    return
                key = (int(ev.seq), str(ev.box_id))
                with self.lock:
                    s = self.zs[z]
                    if key in s.seen or ev.box_id in self.pending:
                        return                   # 재시도로 같은 이벤트가 두 번 올 수 있다
                    self.pending[ev.box_id] = (z, key)
                    if s.t0 is None:
                        s.t0 = time.perf_counter()
                if not self.cli.service_is_ready():
                    self._note("db/get_box_info 가 없다 — 치수를 몰라 3D 를 못 그린다")
                    with self.lock:
                        self.pending.pop(ev.box_id, None)
                    return
                req = GetBoxInfo.Request()
                req.box_id = ev.box_id
                # ★ call_async + add_done_callback (§9). spin_until_future_complete 금지.
                fut = self.cli.call_async(req)
                fut.add_done_callback(lambda f, e=ev: self._on_info(f, e))
            except Exception as e:               # noqa: BLE001
                self._note(f"[{z}] 이벤트 처리 실패: {e}")

        def _on_info(self, fut, ev):
            try:
                res = fut.result()
            except Exception as e:               # noqa: BLE001
                self._note(f"get_box_info 실패: {e}")
                with self.lock:
                    self.pending.pop(ev.box_id, None)
                return
            with self.lock:
                zk = self.pending.pop(ev.box_id, None)
                if zk is None:
                    return
                z, key = zk
                s = self.zs[z]
                if not res.found:
                    self._note(f"[{z}] {ev.box_id}: DB 에 없다 — 건너뛴다", locked=True)
                    return
                i = res.info
                w, d, h = int(i.size_w_mm), int(i.size_d_mm), int(i.size_h_mm)
                actual = dict(x_mm=int(ev.actual_x_mm), y_mm=int(ev.actual_y_mm),
                              z_mm=int(ev.actual_z_mm),
                              yaw_deg=float(ev.actual_yaw_deg))
                # plan 없이 update — 코어는 plans 에 없으면 actual 만으로 반영한다.
                s.pk.update(ev.box_id, w, d, h, actual)
                s.seen.add(key)
                if s.pk.last_warning:
                    self._note(f"[{z}] {s.pk.last_warning}", locked=True)

        def _note(self, msg, locked=False):
            def put():
                if msg not in self.notes:
                    self.notes.append(msg)
                    del self.notes[:-3]          # 최근 3개만
            if locked:
                put()
            else:
                with self.lock:
                    put()

        # ---------- 그리기용 스냅샷 ----------
        def snapshot(self):
            """{zone: algo_service_node.dump() 모양의 dict}. draw 가 소스를 구분 안 해도 되게."""
            with self.lock:
                out, notes = {}, list(self.notes)
                for z, s in self.zs.items():
                    pk, off = s.pk, s.official
                    boxes = []
                    for pose, w, d, h in pk.placed:
                        ww, dd = ((w, d) if round(pose["yaw_deg"] / 90) % 2 == 0
                                  else (d, w))
                        boxes.append([pose["x_mm"] - ww // 2, pose["y_mm"] - dd // 2,
                                      pose["z_mm"], ww, dd, h, pose["yaw_deg"]])
                    n = len(pk.placed)
                    hm, hm_src = pk.hm, "재구성"
                    if s.algo_hm is not None:
                        if s.algo_hm.shape != pk.hm.shape:
                            notes.append(f"[{z}] algo heightmap 크기 불일치 — config 확인")
                        else:
                            hm, hm_src = s.algo_hm, "algo 발행"
                            gap = int(np.abs(s.algo_hm - pk.hm).max())
                            if gap > pk.cell and n:
                                notes.append(f"[{z}] algo heightmap 과 재구성이 최대 "
                                             f"{gap}mm 어긋난다")
                    # 재구성과 공식 집계가 어긋나면 숨기지 않는다. 치수 조회 중은 제외.
                    if (off is not None and not self.pending
                            and off.placed_count != n):
                        notes.append(f"[{z}] 관제탑 적재수 {off.placed_count} ≠ 재구성 {n}"
                                     " — 뷰어를 늦게 띄웠다 (그전 이벤트는 못 받는다)")
                    out[z] = {
                        "zone": z, "count": n, "fill": pk.fill_rate(),
                        "cap": pk.capacity_rate(),
                        "elapsed_s": (0.0 if s.t0 is None
                                      else time.perf_counter() - s.t0),
                        "hmax": int(np.asarray(hm).max()), "max_stack": pk.max_stack,
                        "pallet": [pk.W * pk.cell, pk.D * pk.cell],
                        "hm": hm, "hm_src": hm_src, "boxes": boxes,
                        "state": "" if off is None else off.state}
                return {"zones": out, "notes": notes[-3:]}

    rclpy.init(args=argv)
    node = LiveViewNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    # ★ matplotlib 은 메인 스레드에서만 그린다. spin 을 옆 스레드로 보낸다.
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def shutdown():
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    return node.snapshot, shutdown


# ══════════════════════════════════════════════════════════════════
#  그리기 — 구역마다 세로 한 칸 (3D 위 · heightmap 아래)
# ══════════════════════════════════════════════════════════════════
def draw(fig, cols, states, notes):
    total = sum(st["count"] for st in states.values() if st)
    for z, (ax3d, ax_hm) in cols.items():
        st = states.get(z)
        ax3d.clear()
        ax_hm.clear()
        if not st:
            ax3d.set_title(f"{z.upper()} — 데이터 없음", fontsize=10)
            continue
        W, D = st["pallet"]
        ms = st["max_stack"]
        for x, y, zz, w, d, h, _yaw in st["boxes"]:
            ax3d.bar3d(x, y, zz, w, d, h, color=COLORS.get(h, "#8993A4"),
                       edgecolor="#172B4D", linewidth=0.4, alpha=0.92, shade=True)
        ax3d.set_xlim(0, W); ax3d.set_ylim(0, D); ax3d.set_zlim(0, ms)
        ax3d.set_box_aspect((W, D, ms))
        ax3d.view_init(elev=24, azim=-58)
        ax3d.tick_params(labelsize=6)
        state = st.get("state") or ""
        ax3d.set_title(f"{z.upper()}  {state}  ·  {st['count']}개  ·  "
                       f"{st['fill'] * 100:.0f}% / {st.get('cap', 0) * 100:.0f}%  ·  "
                       f"{st['hmax']}mm", fontsize=10)

        hm = np.asarray(st["hm"])
        ax_hm.imshow(hm.T, origin="lower", cmap="viridis", vmin=0, vmax=ms,
                     extent=[0, W, 0, D], aspect="equal")
        ax_hm.set_title(f"heightmap — {st.get('hm_src', '재구성')}", fontsize=8)
        ax_hm.tick_params(labelsize=6)

    fig.suptitle(f"적재 현황 — 합계 {total}개 · 적재율은 실제높이/용량 기준", fontsize=13)
    for t in list(getattr(fig, "_notes", [])):
        t.remove()
    fig._notes = []
    for k, msg in enumerate(notes or []):
        fig._notes.append(fig.text(0.5, 0.02 + 0.026 * k, "⚠ " + msg, ha="center",
                                   fontsize=8, color="#B54708"))


def main():
    ap = argparse.ArgumentParser(description="실시간 적재 뷰어 (기본: 3구역 동시)")
    ap.add_argument("--source", choices=("ros", "file"), default="ros",
                    help="ros = /zone_*/robot/event 구독 (기본) · file = 스냅샷 JSON 폴링")
    ap.add_argument("-p", "--path", default="/tmp/algo_state.json",
                    help="file 모드에서만. algo 에 준 snapshot_path 와 같은 값")
    ap.add_argument("-z", "--zone", default="all",
                    help="볼 구역: all(기본) 또는 a|b|c, 쉼표 목록(a,c)")
    ap.add_argument("--save", default="", help="갱신마다 PNG 를 남길 디렉터리")
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--headless", action="store_true", help="창 없이 --save 만 (원격용)")
    # ★ `ros2 run ... --ros-args -p x:=y` 를 그대로 받기 위해 남은 인자를 흘려보낸다.
    args, ros_argv = ap.parse_known_args()

    zsel = str(args.zone).lower()
    zones = list(ZONES_ALL) if zsel in ("all", "") else \
        [z.strip() for z in zsel.split(",") if z.strip() in ZONES_ALL]
    if not zones:
        raise SystemExit(f"구역 인자를 못 알아듣겠다: {args.zone!r} (all | a|b|c | a,c)")

    if args.headless:
        matplotlib.use("Agg")
    if args.save:
        os.makedirs(args.save, exist_ok=True)

    snapshot = shutdown = None
    paths = {}
    if args.source == "ros":
        snapshot, shutdown = make_ros_source(zones, ros_argv or None)
        print("[live_view] ROS 모드 — 구역 %s 의 /zone_*/robot/event 를 기다립니다."
              " Ctrl+C 로 종료." % ",".join(zones))
    else:
        # algo 가 구역마다 다른 파일에 쓴다 — 같은 규칙으로 경로를 만든다.
        stem, ext = os.path.splitext(args.path)
        for z in zones:
            paths[z] = (args.path if stem.endswith(f"_{z}")
                        else f"{stem}_{z}{ext or '.json'}")
        print(f"[live_view] 파일 모드 — {', '.join(paths.values())} 를 지켜봅니다.")

    n = len(zones)
    fig = plt.figure(figsize=(4.6 * n + 1, 7.2))
    gs = fig.add_gridspec(2, n, height_ratios=[1.7, 1], hspace=0.3, wspace=0.28,
                          left=0.04, right=0.97, top=0.88, bottom=0.11)
    cols = {z: (fig.add_subplot(gs[0, k], projection="3d"),
                fig.add_subplot(gs[1, k])) for k, z in enumerate(zones)}
    if not args.headless:
        plt.ion(); plt.show(block=False)

    last_sig, shot = None, 0
    try:
        while True:
            states, notes = {}, []
            if args.source == "ros":
                snap = snapshot()
                states, notes = snap["zones"], snap["notes"]
            else:
                for z, p in paths.items():
                    st = read(p)
                    if st:
                        st.setdefault("hm_src", "algo 스냅샷")
                        states[z] = st
            if states:
                # ★ 바뀐 때만 다시 그린다 — 매 틱 3D 세 개를 통째로 그리면
                #   창이 버벅인다. 경고 변화도 '바뀐 것'이다.
                sig = tuple(sorted((z, st["count"], st["hmax"], st.get("state", ""))
                                   for z, st in states.items())) + tuple(notes)
                if sig != last_sig:
                    last_sig = sig
                    draw(fig, cols, states, notes)
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
