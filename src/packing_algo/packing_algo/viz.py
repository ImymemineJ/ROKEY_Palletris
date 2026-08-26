"""트랙 C 시각화 (C3) — 쌓이는 과정 동영상 · heightmap 히트맵 · 3D 배치도 · 적재율 곡선.
"""
import argparse
import copy
import random

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation, font_manager

from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load

def pick_korean_font():
    """한글 라벨이 네모(□)로 깨지지 않게 CJK 폰트를 찾아 쓴다."""
    candidates = (
        "Apple SD Gothic Neo", "AppleGothic", "NanumGothic", "Nanum Gothic",
        "Noto Sans CJK KR", "Noto Sans CJK JP", "Malgun Gothic"
    )
    for name in candidates:
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False


pick_korean_font()

# 박스 높이별 색 — 3호(210) 파랑 / 4호(280) 주황 / 5호(340) 초록
COLORS = {210: "#4C9AFF", 280: "#FFAB00", 340: "#36B37E"}


def simulate(cfg, n=15, noise=0, seed_box=42, seed_noise=7, packer_cls=None):
    """박스를 하나씩 plan → update 하면서 매 단계 스냅샷을 남긴다."""
    boxes = [(b["w_mm"], b["d_mm"], b["h_mm"]) for b in cfg["boxes"]["types"]]
    pk = (packer_cls or HeightmapPacker)(cfg)
    rb, rn = random.Random(seed_box), random.Random(seed_noise)
    
    steps = [dict(
        placed=[],
        hm=pk.hm.copy(),
        fill=0.0,
        cap=0.0,
        n_ok=0,
        n_skip=0,
        label="시작 · 빈 팔레트"
    )]
    
    placed = skipped = 0
    for k in range(n):
        w, d, h = rb.choice(boxes)
        box_id = f"BOX{k:05d}"
        ok, pose, _, msg = pk.plan(box_id, w, d, h)
        if not ok:
            pk.cancel(box_id)
            if msg == "PALLET_FULL":
                steps.append(dict(
                    placed=copy.deepcopy(pk.placed),
                    hm=pk.hm.copy(),
                    fill=pk.fill_rate(),
                    cap=pk.capacity_rate(),
                    n_ok=placed,
                    n_skip=skipped,
                    label=f"{k + 1}번째 {w}x{d}x{h} → PALLET_FULL · 종료"
                ))
                break
            skipped += 1
            steps.append(dict(
                placed=copy.deepcopy(pk.placed),
                hm=pk.hm.copy(),
                fill=pk.fill_rate(),
                cap=pk.capacity_rate(),
                n_ok=placed,
                n_skip=skipped,
                label=f"{k + 1}번째 {w}x{d}x{h} → 자리 없음, 버림"
            ))
            continue
        actual = dict(pose)
        for ax in ("x_mm", "y_mm", "z_mm"):
            actual[ax] += rn.randint(-noise, noise)
        pk.update(box_id, w, d, h, actual)
        placed += 1
        steps.append(dict(
            placed=copy.deepcopy(pk.placed),
            hm=pk.hm.copy(),
            fill=pk.fill_rate(),
            cap=pk.capacity_rate(),
            n_ok=placed,
            n_skip=skipped,
            label=f"{k + 1}번째 {w}x{d}x{h} 적재 성공"
        ))
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
                 edgecolor="#172B4D", linewidth=0.5, alpha=0.92, shade=True)
    ax.set_xlim(0, pal["width_mm"])
    ax.set_ylim(0, pal["depth_mm"])
    ax.set_zlim(0, pal["max_stack_mm"])
    ax.set_box_aspect((pal["width_mm"], pal["depth_mm"], pal["max_stack_mm"]))
    ax.set_xlabel("Px (mm)", fontsize=8)
    ax.set_ylabel("Py (mm)", fontsize=8)
    ax.set_zlabel("Pz (mm)", fontsize=8)
    ax.view_init(elev=24, azim=-58)


def draw_heightmap(ax, hm, cfg):
    """heightmap 히트맵."""
    pal = cfg["pallet"]
    ax.clear()
    im = ax.imshow(hm.T, origin="lower", cmap="viridis", vmin=0, vmax=pal["max_stack_mm"],
                   extent=[0, pal["width_mm"], 0, pal["depth_mm"]], aspect="equal")
    ax.set_title("Heightmap (mm)", fontsize=9)
    ax.set_xlabel("Px", fontsize=8)
    ax.set_ylabel("Py", fontsize=8)
    return im


def draw_fill_curve(ax, fills, caps, k):
    """적재율 vs 박스 수."""
    ax.clear()
    f_pc = [v * 100 for v in fills]
    c_pc = [v * 100 for v in caps]
    ax.plot(range(0, len(f_pc)), f_pc, color="#DFE1E6", lw=1.5)
    ax.plot(range(0, len(c_pc)), c_pc, color="#F5D0BF", lw=1.5)
    ax.plot(range(0, k + 1), f_pc[:k + 1], color="#2a78d6", lw=2, marker="o", ms=3,
            label="실제 높이 기준")
    ax.plot(range(0, k + 1), c_pc[:k + 1], color="#eb6834", lw=2, marker="o", ms=3,
            label="용량 기준")
    ax.axhline(70, color="#52514E", lw=1.2, ls=(0, (5, 4)))
    ax.text(0, 71.5, "목표 70%", color="#52514E", fontsize=8)
    ax.set_xlim(0, max(1, len(f_pc) - 1))
    ax.set_ylim(0, 85)
    ax.set_title("적재율 vs 박스 수 (%)", fontsize=9)
    ax.set_xlabel("박스 수", fontsize=8)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="lower right", framealpha=0.9)


def render_video(cfg, steps, out_path, fps=2, hold=6):
    """단독 패커 박스 적재 과정 애니메이션 동영상."""
    fills = [s["fill"] for s in steps]
    caps = [s["cap"] for s in steps]
    fig = plt.figure(figsize=(12, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1], hspace=0.45, wspace=0.25,
                          left=0.03, right=0.96, top=0.90, bottom=0.09)
    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_hm = fig.add_subplot(gs[0, 1])
    ax_fr = fig.add_subplot(gs[1, 1])
    title = fig.suptitle("", fontsize=12)

    def frame(f):
        k = min(f, len(steps) - 1)
        s = steps[k]
        draw_boxes3d(ax3d, s["placed"], cfg)
        draw_heightmap(ax_hm, s["hm"], cfg)
        draw_fill_curve(ax_fr, fills, caps, k)
        title.set_text(f"{s['label']}   |   적재 {s['n_ok']}개 · 버림 {s['n_skip']}개   "
                       f"|   적재율 {s['fill'] * 100:.1f}% / {s['cap'] * 100:.1f}%(용량)   "
                       f"최고 {int(s['hm'].max())}mm / {cfg['pallet']['max_stack_mm']}mm")
        return []

    anim = animation.FuncAnimation(fig, frame, frames=len(steps) + hold, blit=False)
    anim.save(out_path, writer=animation.FFMpegWriter(fps=fps, bitrate=2400))
    plt.close(fig)
    return out_path


def render_compare_video(cfg, steps_a, steps_b, out_path,
                         name_a="제안 (최종 튜닝 Heightmap)", name_b="Extreme Point (EP 3D 극점)",
                         fps=2, hold=8):
    """두 패커를 나란히(Side-by-Side) 재생하는 비교 동영상.
    하단에는 실시간 솟구침 곡선 대신 깔끔한 요약비교 바 차트(Bar Chart)를 표출한다.
    """
    n = max(len(steps_a), len(steps_b)) + hold
    fig = plt.figure(figsize=(13.0, 6.8))
    gs = fig.add_gridspec(2, 2, height_ratios=[3.2, 1], hspace=0.35, wspace=0.08,
                          left=0.04, right=0.96, top=0.88, bottom=0.10)
    axL = fig.add_subplot(gs[0, 0], projection="3d")
    axR = fig.add_subplot(gs[0, 1], projection="3d")
    axF = fig.add_subplot(gs[1, :])
    title = fig.suptitle("", fontsize=13, fontweight="bold")

    def frame(f):
        ka = min(f, len(steps_a) - 1)
        kb = min(f, len(steps_b) - 1)
        sa, sb = steps_a[ka], steps_b[kb]

        # 3D 배치도
        draw_boxes3d(axL, sa["placed"], cfg)
        axL.set_title(f"{name_a}\n적재 {sa['n_ok']}개 · 버림 {sa['n_skip']}개  |  적재율 {sa['fill'] * 100:.1f}%  최고 {int(sa['hm'].max())}mm"
                      + (" (종료)" if f >= len(steps_a) - 1 else ""), fontsize=9.5)

        draw_boxes3d(axR, sb["placed"], cfg)
        axR.set_title(f"{name_b}\n적재 {sb['n_ok']}개 · 버림 {sb['n_skip']}개  |  최고 {int(sb['hm'].max())}mm"
                      + (" (종료)" if f >= len(steps_b) - 1 else ""), fontsize=9.5)

        # 하단: 가로 바 차트 (적재 개수 비교)
        axF.clear()
        labels = [name_a, name_b]
        cnts = [sa['n_ok'], sb['n_ok']]
        colors = ["#2a78d6", "#9b59b6"]
        
        bars = axF.barh(labels, cnts, color=colors, height=0.45, alpha=0.9)
        axF.set_xlim(0, 35)
        axF.set_xlabel("현재 진행 단계 적재 박스 수 (개)", fontsize=9)
        axF.set_title("알고리즘별 현재 적재 완료 개수 비교", fontsize=9.5)
        axF.grid(axis='x', alpha=0.3)
        axF.set_axisbelow(True)
        
        for bar in bars:
            w = bar.get_width()
            axF.text(w + 0.5, bar.get_y() + bar.get_height()/2.0, f"{int(w)}개",
                     va='center', ha='left', fontsize=9, fontweight='bold')

        title.set_text(f"동일한 무작위 박스 공급 — 적재 과정 비교 (단계 {f}/{n - hold})")
        return []

    anim = animation.FuncAnimation(fig, frame, frames=n, blit=False)
    anim.save(out_path, writer=animation.FFMpegWriter(fps=fps, bitrate=2600))
    plt.close(fig)
    return out_path
