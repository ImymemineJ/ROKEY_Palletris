"""실시간 적재 뷰어 — 알고리즘이 "지금 어디에 쌓고 있는지" 를 Isaac 옆에 띄운다.

`algo_service_node` 가 `snapshot_path` 로 떨구는 JSON 을 폴링해 다시 그린다.
ROS 를 쓰지 않으므로(파일만 읽는다) algo 가 아무것도 발행하지 않는다는 계약(D2)을
건드리지 않고, 노드를 껐다 켜도 뷰어는 그대로 살아 있다.

실행:
    # 1) algo 를 스냅샷 옵션으로 띄운다
    ros2 run packing_algo algo_service_node --ros-args -p snapshot_path:=/tmp/algo_state.json
    # 2) 옆 터미널에서 뷰어
    ros2 run packing_algo live_view                       # 기본 /tmp/algo_state.json
    ros2 run packing_algo live_view --save shots          # 매 갱신마다 PNG 도 남긴다
"""
import argparse
import json
import os
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
    ax_hm.set_title("heightmap (mm)", fontsize=9)
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

    fig.suptitle(f"zone {st['zone']}  ·  적재 {st['count']}개  ·  "
                 f"적재율 {st['fill'] * 100:.1f}% (실제높이) / "
                 f"{st.get('cap', 0.0) * 100:.1f}% (용량)  ·  "
                 f"최고 {st['hmax']}mm / {ms}mm", fontsize=13)


def main():
    ap = argparse.ArgumentParser(description="실시간 적재 뷰어")
    ap.add_argument("-p", "--path", default="/tmp/algo_state.json")
    ap.add_argument("--save", default="", help="갱신마다 PNG 를 남길 디렉터리")
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--headless", action="store_true", help="창 없이 --save 만 (원격용)")
    args = ap.parse_args()
    if args.headless:
        matplotlib.use("Agg")
    if args.save:
        os.makedirs(args.save, exist_ok=True)

    fig = plt.figure(figsize=(13, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1], hspace=0.45, wspace=0.25,
                          left=0.03, right=0.96, top=0.90, bottom=0.09)
    axes = (fig.add_subplot(gs[:, 0], projection="3d"),
            fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1]))
    if not args.headless:
        plt.ion(); plt.show(block=False)

    print(f"[live_view] {args.path} 를 지켜봅니다. algo 를 "
          f"-p snapshot_path:={args.path} 로 띄우세요. Ctrl+C 로 종료.")
    fills, last_count, last_mtime, shot = [], -1, 0.0, 0
    try:
        while True:
            try:
                mtime = os.path.getmtime(args.path)
            except OSError:
                mtime = 0.0
            if mtime and mtime != last_mtime:
                st = read(args.path)
                if st:
                    last_mtime = mtime
                    if st["count"] < last_count:      # reset 됐다 — 곡선도 새로
                        fills = []
                    # 옛 스냅샷(cap 없음)도 읽히게 — 없으면 fill 로 대신한다
                    pair = (st["fill"], st.get("cap", st["fill"]))
                    if st["count"] != last_count:
                        fills.append(pair)
                        last_count = st["count"]
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


if __name__ == "__main__":
    main()
