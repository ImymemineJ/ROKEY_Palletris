"""트랙 C 시각화 (C3) — 쌓이는 과정 동영상 · heightmap 히트맵 · 3D 배치도 · 적재율 곡선.

코어(HeightmapPacker)는 전혀 건드리지 않고 읽기만 한다. ROS 의존성 없음.

실행:
    cd ~/palletris_ws && source install/setup.bash
    python3 -m packing_algo.viz                      # packing_stack.mp4 생성
    python3 -m packing_algo.viz --noise 3            # 실측 오차 ±3mm 주입
    python3 -m packing_algo.viz -o /tmp/a.mp4 -n 20  # 저장 위치·박스 개수 지정
"""
import argparse
import random

import matplotlib
matplotlib.use("Agg")          # 화면 없는 환경(SSH 등)에서도 돌아가게 — import 순서 중요
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation, font_manager

from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load

# 한글 라벨이 네모(□)로 깨지지 않게 CJK 폰트를 찾아 쓴다. 없으면 기본 폰트로 그냥 진행.
for _name in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic", "Malgun Gothic"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _name
        break
plt.rcParams["axes.unicode_minus"] = False   # 마이너스 기호가 깨지는 것 방지

# 박스 높이별 색 — 3호(210) 파랑 / 4호(280) 주황 / 5호(340) 초록
COLORS = {210: "#4C9AFF", 280: "#FFAB00", 340: "#36B37E"}


def simulate(cfg, n=15, noise=0, seed_box=42, seed_noise=7):
    """박스를 하나씩 plan → update 하면서 매 단계 스냅샷을 남긴다.

    ★ NO_FEASIBLE_POSE 는 '이 박스만' 자리가 없다는 뜻이므로 그 박스만 버리고 계속 간다.
      시나리오를 끝내는 것은 PALLET_FULL 뿐이다 (트랙 D 설계변경 A1).
      첫 실패에서 멈추면 적재율을 크게 과소평가한다 — 49.8% vs 60.2%.

    ★ 난수 스트림을 박스용/노이즈용으로 분리한다. 하나를 공유하면 노이즈를 켜는 순간
      박스 등장 순서까지 바뀐다 (TS-006).
    """
    boxes = [(b["w_mm"], b["d_mm"], b["h_mm"]) for b in cfg["boxes"]["types"]]
    pk = HeightmapPacker(cfg)
    rb, rn = random.Random(seed_box), random.Random(seed_noise)
    steps = []
    placed = skipped = 0
    for k in range(n):
        w, d, h = rb.choice(boxes)
        box_id = f"BOX{k:05d}"
        ok, pose, _, msg = pk.plan(box_id, w, d, h)
        if not ok:
            pk.cancel(box_id)                       # ★ 이탈 경로에서는 반드시 계획 폐기
            if msg == "PALLET_FULL":
                steps.append(dict(placed=list(pk.placed), hm=pk.hm.copy(), fill=pk.fill_rate(),
                                  n_ok=placed, n_skip=skipped,
                                  label=f"{k + 1}번째 {w}x{d}x{h} → PALLET_FULL · 종료"))
                break
            skipped += 1
            steps.append(dict(placed=list(pk.placed), hm=pk.hm.copy(), fill=pk.fill_rate(),
                              n_ok=placed, n_skip=skipped,
                              label=f"{k + 1}번째 {w}x{d}x{h} → 자리 없음, 버리고 계속"))
            continue
        actual = dict(pose)
        for ax in ("x_mm", "y_mm", "z_mm"):
            actual[ax] += rn.randint(-noise, noise)
        pk.update(box_id, w, d, h, actual)
        placed += 1
        steps.append(dict(placed=list(pk.placed), hm=pk.hm.copy(), fill=pk.fill_rate(),
                          n_ok=placed, n_skip=skipped,
                          label=f"{k + 1}번째 {w}x{d}x{h} → ({pose['x_mm']}, {pose['y_mm']}) "
                                f"z={pose['z_mm']} yaw={int(pose['yaw_deg'])}°"))
    return pk, steps


def _footprint(entry):
    """pk.placed 한 칸 → 3D 막대 그릴 값 (좌하단 x, y, 밑면 z, 폭, 깊이, 높이) mm."""
    pose, w_mm, d_mm, h_mm = entry
    w, d = (w_mm, d_mm) if round(pose["yaw_deg"] / 90) % 2 == 0 else (d_mm, w_mm)
    return pose["x_mm"] - w // 2, pose["y_mm"] - d // 2, pose["z_mm"], w, d, h_mm


def draw_boxes3d(ax, placed, cfg):
    """3D 배치도. 박스를 놓인 순서대로 막대로 그린다."""
    pal = cfg["pallet"]
    ax.clear()
    for entry in placed:
        x, y, z, w, d, h = _footprint(entry)
        ax.bar3d(x, y, z, w, d, h, color=COLORS.get(h, "#8993A4"),
                 edgecolor="#172B4D", linewidth=0.4, alpha=0.92, shade=True)
    ax.set_xlim(0, pal["width_mm"]); ax.set_ylim(0, pal["depth_mm"])
    ax.set_zlim(0, pal["max_stack_mm"])
    ax.set_box_aspect((pal["width_mm"], pal["depth_mm"], pal["max_stack_mm"]))
    ax.set_xlabel("Px (mm)"); ax.set_ylabel("Py (mm)"); ax.set_zlabel("Pz (mm)")
    ax.view_init(elev=24, azim=-58)


def draw_heightmap(ax, hm, cfg):
    """heightmap 히트맵. hm[px][py] 라서 .T 로 그려야 화면 축과 맞는다."""
    pal = cfg["pallet"]
    ax.clear()
    im = ax.imshow(hm.T, origin="lower", cmap="viridis", vmin=0, vmax=pal["max_stack_mm"],
                   extent=[0, pal["width_mm"], 0, pal["depth_mm"]], aspect="equal")
    ax.set_title("heightmap (mm)", fontsize=9)
    ax.set_xlabel("Px"); ax.set_ylabel("Py")
    return im


def draw_fill_curve(ax, fills, k):
    """적재율 vs 박스 수. k 번째까지만 그려서 같이 자라게 한다."""
    ax.clear()
    ax.plot(range(1, len(fills) + 1), fills, color="#DFE1E6", lw=1.5)
    ax.plot(range(1, k + 2), fills[:k + 1], color="#0052CC", lw=2, marker="o", ms=3)
    ax.set_xlim(0.5, len(fills) + 0.5); ax.set_ylim(0, max(0.8, max(fills) * 1.15))
    ax.set_title("적재율 vs 박스 수", fontsize=9)
    ax.set_xlabel("박스 수"); ax.grid(alpha=0.3)


def render_video(cfg, steps, out_path, fps=2, hold=6):
    """매 단계를 프레임으로 찍어 mp4 로 저장. hold = 마지막 화면을 몇 프레임 더 유지할지."""
    fills = [s["fill"] for s in steps]
    fig = plt.figure(figsize=(12, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1], hspace=0.45, wspace=0.25,
                          left=0.03, right=0.96, top=0.90, bottom=0.09)
    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_hm = fig.add_subplot(gs[0, 1])
    ax_fr = fig.add_subplot(gs[1, 1])
    title = fig.suptitle("", fontsize=12)

    def frame(f):
        k = min(f, len(steps) - 1)              # hold 구간은 마지막 단계를 반복
        s = steps[k]
        draw_boxes3d(ax3d, s["placed"], cfg)
        draw_heightmap(ax_hm, s["hm"], cfg)
        draw_fill_curve(ax_fr, fills, k)
        title.set_text(f"{s['label']}   |   적재 {s['n_ok']}개 · 버림 {s['n_skip']}개   "
                       f"|   적재율 {s['fill'] * 100:.1f}%   "
                       f"최고 {int(s['hm'].max())}mm / {cfg['pallet']['max_stack_mm']}mm")
        return []

    anim = animation.FuncAnimation(fig, frame, frames=len(steps) + hold, blit=False)
    anim.save(out_path, writer=animation.FFMpegWriter(fps=fps, bitrate=2400))
    plt.close(fig)
    return out_path


def main():
    ap = argparse.ArgumentParser(description="패킹 과정 동영상 생성")
    ap.add_argument("-o", "--out", default="packing_stack.mp4", help="저장할 mp4 경로")
    ap.add_argument("-n", "--count", type=int, default=None, help="박스 개수 (기본: config 의 stage1)")
    ap.add_argument("--noise", type=int, default=0, help="실측 pose 에 주입할 오차 ±mm")
    ap.add_argument("--fps", type=int, default=2)
    args = ap.parse_args()

    cfg = load()
    n = args.count or cfg["boxes"]["stage1"]["total_count"]
    pk, steps = simulate(cfg, n=n, noise=args.noise)
    render_video(cfg, steps, args.out, fps=args.fps)

    print(f"저장: {args.out}")
    last = steps[-1] if steps else dict(n_ok=0, n_skip=0)
    print(f"투입 {n}개 → 적재 {last['n_ok']}개 · 버림 {last['n_skip']}개 · "
          f"적재율 {pk.fill_rate() * 100:.1f}% · "
          f"최고 {int(pk.hm.max())}mm / {cfg['pallet']['max_stack_mm']}mm")


if __name__ == "__main__":
    main()
