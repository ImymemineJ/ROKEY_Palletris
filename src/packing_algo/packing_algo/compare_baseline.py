"""C4 비교 실험 — 제안 알고리즘 vs 베이스라인, 여러 시드 평균 (명세서 §13).

적재율은 **박스 투입 순서에 크게 좌우된다** (시드에 따라 적재 6~15개까지 벌어진다).
따라서 실행 한 번의 숫자를 발표에 쓰면 안 된다. 이 스크립트는 같은 시드 집합을
두 패커에 똑같이 먹여 평균·표준편차·최소·최대를 낸다.

실행:
    cd ~/palletris_ws && source install/setup.bash
    python3 -m packing_algo.compare_baseline                      # 시드 30개, 투입 15/20/30
    python3 -m packing_algo.compare_baseline --seeds 50 --noise 3
"""
import argparse
import random
import statistics as st

import matplotlib
matplotlib.use("Agg")          # 화면 없는 환경에서도 돌아가게 — pyplot import 앞에 와야 한다
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from packing_algo.baseline import BaselinePacker
from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load

for _name in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic", "Malgun Gothic"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _name
        break
plt.rcParams["axes.unicode_minus"] = False

# 색: dataviz 기본 팔레트 categorical slot 1·2 (all-pairs 검증 통과 구간).
# 텍스트는 계열색을 입지 않는다 — 아래 잉크 토큰만 쓴다.
SERIES = {"제안 (heightmap)": "#2a78d6", "베이스라인 (순차)": "#eb6834"}
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#d8d7d2"
TARGET_INK = "#52514e"      # 목표선 — 계열색이 아닌 잉크색. 데이터가 아니라 기준선이다


def run_once(packer_cls, cfg, n, seed_box, seed_noise, noise_mm, cap=200, dry_limit=20):
    """박스를 흘려보낸다. n=None 이면 **팔레트가 포화될 때까지** (= 최대로 쌓기).

    → (투입 수, 적재 수, 버림 수, 적재율, 최고높이, 적재율 이력)

    ★ NO_FEASIBLE_POSE 는 그 박스만 버리고 계속. 종료는 PALLET_FULL 뿐 (트랙 D A1).
      다만 통로가 막혀 큰 박스만 계속 튕기는 상태가 있을 수 있어, 연속 실패 dry_limit 회면
      사실상 포화로 보고 끝낸다.
    ★ 난수는 박스 순서용 / 노이즈용으로 분리 (TS-006).
    """
    boxes = [(b["w_mm"], b["d_mm"], b["h_mm"]) for b in cfg["boxes"]["types"]]
    pk = packer_cls(cfg)
    rb, rn = random.Random(seed_box), random.Random(seed_noise)
    fed = placed = skipped = dry = 0
    curve = []
    for k in range(n if n else cap):
        fed += 1
        w, d, h = rb.choice(boxes)
        box_id = f"B{k}"
        ok, pose, _, msg = pk.plan(box_id, w, d, h)
        if not ok:
            pk.cancel(box_id)
            skipped += 1
            dry += 1
            if msg == "PALLET_FULL" or (n is None and dry >= dry_limit):
                break
            continue
        dry = 0
        actual = dict(pose)
        for ax in ("x_mm", "y_mm", "z_mm"):
            actual[ax] += rn.randint(-noise_mm, noise_mm)
        pk.update(box_id, w, d, h, actual)
        placed += 1
        curve.append(pk.fill_rate())
    return (fed, placed, skipped, pk.fill_rate(), int(pk.hm.max()), curve,
            pk.capacity_rate())


PACKERS = (("제안 (heightmap)", HeightmapPacker), ("베이스라인 (순차)", BaselinePacker))


def sweep(cfg, counts, seeds, noise_mm):
    """포화 조건(주 지표) + 투입 개수별(부록) 을 두 패커에 '같은 시드'로 돌린다."""
    sat = {lb: [run_once(cls, cfg, None, s, s + 1000, noise_mm) for s in seeds]
           for lb, cls in PACKERS}
    by_n = {lb: {n: [run_once(cls, cfg, n, s, s + 1000, noise_mm) for s in seeds] for n in counts}
            for lb, cls in PACKERS}
    return sat, by_n


def markdown_table(sat, by_n, counts, seeds, noise_mm, TARGET):
    L = ["# C4 비교 실험 — 제안 vs 베이스라인", "",
         f"시드 {len(seeds)}개(1~{max(seeds)}) · 실측 오차 ±{noise_mm}mm · 두 패커에 동일 시퀀스 투입", "",
         "## 주 지표 — 팔레트를 포화시킬 때까지 (= 최대로 쌓기)", "",
         "| 알고리즘 | 적재 개수 | 적재율(실제높이) | 최소~최대 | 적재율(용량) | 최고 높이 | 투입 | 버림 |",
         "|---|---|---|---|---|---|---|---|"]
    for lb in sat:
        rs = sat[lb]
        pl = [r[1] for r in rs]; fr = [r[3] * 100 for r in rs]; cp = [r[6] * 100 for r in rs]
        L.append(f"| {lb} | {st.mean(pl):.1f} ± {st.pstdev(pl):.1f} | "
                 f"**{st.mean(fr):.1f}% ± {st.pstdev(fr):.1f}** | {min(fr):.1f}~{max(fr):.1f}% | "
                 f"{st.mean(cp):.1f}% | "
                 f"{st.mean([r[4] for r in rs]):.0f}mm | {st.mean([r[0] for r in rs]):.1f} | "
                 f"{st.mean([r[2] for r in rs]):.1f} |")
    a, b = list(sat)
    fa = st.mean([r[3] for r in sat[a]]) * 100
    fb = st.mean([r[3] for r in sat[b]]) * 100
    L += ["", f"- 목표 {TARGET:.0f}% 까지: **{TARGET - fa:+.1f}%p**",
          f"- 베이스라인 대비: **+{fa - fb:.1f}%p** (+{(fa / fb - 1) * 100:.0f}%)",
          f"- 포화까지 박스 {st.mean([r[0] for r in sat[a]]):.0f}개를 흘려보내야 하고 "
          f"그중 {st.mean([r[2] for r in sat[a]]):.0f}개가 버려진다 "
          f"(스폰 간격 12초 기준 약 {st.mean([r[0] for r in sat[a]]) * 12 / 60:.0f}분).",
          "",
          "## 부록 — 투입 개수별 (몇 개를 흘려보낼지 정하는 근거)", "",
          "> 발표 지표는 위의 '포화' 표를 쓴다. 아래는 config 의 `stage1.total_count` 를",
          "> 정하기 위한 자료다. 투입이 모자라면 팔레트가 차기 전에 실험이 끝난다.", "",
          "| 투입 | 알고리즘 | 적재 개수 | 적재율 |", "|---|---|---|---|"]
    for n in counts:
        for lb in by_n:
            rs = by_n[lb][n]
            pl = [r[1] for r in rs]; fr = [r[3] * 100 for r in rs]
            L.append(f"| {n} | {lb} | {st.mean(pl):.1f} ± {st.pstdev(pl):.1f} | "
                     f"{st.mean(fr):.1f}% ± {st.pstdev(fr):.1f} |")
    return "\n".join(L) + "\n"


def _style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(axis="y", color=GRID, lw=0.8, alpha=0.9)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(colors=INK2, length=0, labelsize=9)


def plot(sat, seeds, noise_mm, out_png, target):
    labels = list(sat)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2), facecolor=SURFACE)

    # ── 왼쪽: 포화 시 적재율. 막대 = 평균, 오차막대 = ±1σ, 점 = 시드 하나하나 ──
    for si, lb in enumerate(labels):
        vals = [r[3] * 100 for r in sat[lb]]
        m, sd = st.mean(vals), st.pstdev(vals)
        ax1.bar(si, m, 0.5, yerr=sd, capsize=5, color=SERIES[lb],
                edgecolor=SURFACE, linewidth=2, error_kw=dict(ecolor=INK2, lw=1.2))
        ax1.scatter(np.full(len(vals), si) + np.linspace(-.11, .11, len(vals)), vals,
                    s=9, color=SURFACE, edgecolor=INK2, linewidth=0.5, zorder=3, alpha=.55)
        ax1.text(si, m + sd + 2.5, f"{m:.1f}%", ha="center", fontsize=12, color=INK)
        ax1.text(si, 2.5, f"평균 {st.mean([r[1] for r in sat[lb]]):.1f}개 적재",
                 ha="center", fontsize=9, color=SURFACE)
    ax1.axhline(target, color=TARGET_INK, lw=1.4, ls=(0, (5, 4)), zorder=1)
    ax1.text(len(labels) - 0.55, target + 1.2, f"목표 {target:.0f}%",
             color=TARGET_INK, fontsize=9, ha="right")
    ax1.set_xticks(range(len(labels))); ax1.set_xticklabels(labels)
    ax1.set_xlim(-0.6, len(labels) - 0.4)
    ax1.set_ylim(0, target + 12)
    ax1.set_ylabel("적재율 (%)", color=INK2, fontsize=10)
    ax1.set_title(f"팔레트 포화 시 적재율 — 시드 {len(seeds)}개 평균 ±1σ",
                  color=INK, fontsize=11, pad=12)
    _style(ax1)

    # ── 오른쪽: 적재 진행에 따른 적재율 (평균선 + ±1σ 밴드) ──
    for lb in labels:
        curves = [r[5] for r in sat[lb]]
        xs, mean, lo, hi = [], [], [], []
        for k in range(max(len(c) for c in curves)):
            vals = [c[k] * 100 for c in curves if len(c) > k]
            if len(vals) < len(curves) / 2:
                break
            m, sd = st.mean(vals), st.pstdev(vals)
            xs.append(k + 1); mean.append(m); lo.append(m - sd); hi.append(m + sd)
        ax2.fill_between(xs, lo, hi, color=SERIES[lb], alpha=0.14, linewidth=0)
        ax2.plot(xs, mean, color=SERIES[lb], lw=2)
        ax2.text(xs[-1], mean[-1], f"  {mean[-1]:.1f}%", color=INK, fontsize=9, va="center")
    ax2.axhline(target, color=TARGET_INK, lw=1.4, ls=(0, (5, 4)), zorder=1)
    ax2.set_xlabel("적재된 박스 수", color=INK2, fontsize=10)
    ax2.set_ylabel("적재율 (%)", color=INK2, fontsize=10)
    ax2.set_title("적재가 쌓이는 동안의 적재율", color=INK, fontsize=11, pad=12)
    _style(ax2)

    handles = [Patch(facecolor=SERIES[l], label=l) for l in labels]
    handles.append(Line2D([], [], color=TARGET_INK, lw=1.4, ls=(0, (5, 4)),
                          label=f"목표 {target:.0f}% (작업명세서 §13)"))
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False,
               fontsize=9, labelcolor=INK2, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle(f"C4 — 제안 알고리즘 vs 베이스라인  ·  팔레트 포화까지  ·  실측 오차 ±{noise_mm}mm",
                 color=INK, fontsize=13)
    fig.tight_layout(rect=[0, 0.06, 1, 0.94])
    fig.savefig(out_png, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="C4 비교 실험 — 여러 시드 평균")
    ap.add_argument("--seeds", type=int, default=30, help="시드 개수 (1..N)")
    ap.add_argument("--counts", default="15,20,30", help="투입 박스 개수, 쉼표 구분")
    ap.add_argument("--noise", type=int, default=3, help="실측 오차 ±mm")
    ap.add_argument("--target", type=float, default=70.0,
                    help="적재율 목표 %% — 작업명세서 §13 (기본 70)")
    ap.add_argument("-o", "--out", default="compare_baseline", help="출력 파일 접두사")
    args = ap.parse_args()

    cfg = load()
    counts = [int(c) for c in args.counts.split(",")]
    seeds = list(range(1, args.seeds + 1))
    sat, by_n = sweep(cfg, counts, seeds, args.noise)

    md = markdown_table(sat, by_n, counts, seeds, args.noise, args.target)
    open(f"{args.out}.md", "w").write(md)
    plot(sat, seeds, args.noise, f"{args.out}.png", args.target)
    print(md)
    print(f"저장: {args.out}.md · {args.out}.png")


if __name__ == "__main__":
    main()
