"""C4 비교 실험 — 제안 알고리즘 vs Extreme Point(EP) vs 베이스라인, 여러 시드 평균 (명세서 §13).

적재율은 **박스 투입 순서에 크게 좌우된다** (시드에 따라 적재 6~15개까지 벌어진다).
따라서 실행 한 번의 숫자를 발표에 쓰면 안 된다. 이 스크립트는 같은 시드 집합을
세 패커에 똑같이 먹여 평균·표준편차·최소·최대를 낸다.

실행:
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
from packing_algo.extreme_point import ExtremePointPacker
from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load

for _name in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic", "Malgun Gothic"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _name
        break
plt.rcParams["axes.unicode_minus"] = False

# 색상 지정
SERIES = {
    "제안 (Heightmap+Lookahead)": "#2a78d6",
    "Extreme Point (EP)": "#9b59b6",
    "베이스라인 (DBLF 순차)": "#eb6834"
}
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#d8d7d2"
TARGET_INK = "#52514e"


def run_once(packer_cls, cfg, n, seed_box, seed_noise, noise_mm, cap=200, dry_limit=20):
    """박스를 흘려보낸다. n=None 이면 **팔레트가 포화될 때까지** (= 최대로 쌓기)."""
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


PACKERS = (
    ("제안 (Heightmap+Lookahead)", HeightmapPacker),
    ("Extreme Point (EP)", ExtremePointPacker),
    ("베이스라인 (DBLF 순차)", BaselinePacker)
)


def sweep(cfg, counts, seeds, noise_mm):
    """포화 조건(주 지표) + 투입 개수별(부록) 을 세 패커에 '같은 시드'로 돌린다."""
    sat = {lb: [run_once(cls, cfg, None, s, s + 1000, noise_mm) for s in seeds]
           for lb, cls in PACKERS}
    by_n = {lb: {n: [run_once(cls, cfg, n, s, s + 1000, noise_mm) for s in seeds] for n in counts}
            for lb, cls in PACKERS}
    return sat, by_n


def markdown_table(sat, by_n, counts, seeds, noise_mm, TARGET):
    L = ["# C4 비교 실험 — 제안 vs Extreme Point vs 베이스라인", "",
         f"시드 {len(seeds)}개(1~{max(seeds)}) · 실측 오차 ±{noise_mm}mm · 세 패커에 동일 시퀀스 투입", "",
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
    a, b, c = list(sat)
    fa = st.mean([r[3] for r in sat[a]]) * 100
    fb = st.mean([r[3] for r in sat[b]]) * 100
    fc = st.mean([r[3] for r in sat[c]]) * 100
    L += ["", f"- 목표 {TARGET:.0f}% 까지: **{TARGET - fa:+.1f}%p**",
          f"- Extreme Point 대비: **+{fa - fb:.1f}%p**",
          f"- 베이스라인 대비: **+{fa - fc:.1f}%p**"]
    if not counts:
        return "\n".join(L) + "\n"
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--noise", type=int, default=0)
    a = ap.parse_args()

    cfg = load()
    seeds = list(range(1, a.seeds + 1))
    counts = [15, 20, 30]
    sat, by_n = sweep(cfg, counts, seeds, a.noise)

    md = markdown_table(sat, by_n, counts, seeds, a.noise, cfg["pallet"].get("target_fill_rate", 0.7) * 100)
    print(md)


if __name__ == "__main__":
    main()
