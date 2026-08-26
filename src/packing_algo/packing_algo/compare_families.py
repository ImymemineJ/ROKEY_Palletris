"""계열 비교 — "왜 heightmap 인가" 를 문헌의 대표 계열과 나란히 잰다 (명세서 §13).

`compare_baseline.py` 는 '아무 생각 없이 쌓기' 대조군 하나뿐이다. 그것만으로는
"우리가 나쁜 것보다 낫다" 밖에 못 보여준다. 여기서는 온라인 3D 적재 문헌의
대표 계열을 **같은 제약 · 같은 박스 시퀀스**로 돌린다 (families.py 참조).

실행:
    python3 -m packing_algo.compare_families
    python3 -m packing_algo.compare_families --quick
"""
import argparse
import pathlib
import statistics as st
from concurrent.futures import ProcessPoolExecutor

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

from packing_algo.baseline import BaselinePacker
from packing_algo.compare_baseline import run_once
from packing_algo.families import DBLFPacker, ExtremePointPacker, ShelfPacker
from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load

for _n in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic"):
    if any(f.name == _n for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _n
        break
plt.rcParams["axes.unicode_minus"] = False

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#d8d7d2"
# 계열색: 우리 것만 진하게, 대조군은 같은 계열의 회색조 — 비교 대상이지 경쟁 색이 아니다.
FAMILIES = [
    ("제안 (heightmap 점수화)", HeightmapPacker, "#2a78d6",
     "Wang & Hauser (2019) 변형 — 높이·지지·벽·먼쪽·평면"),
    ("DBLF", DBLFPacker, "#7b8794",
     "Karabulut & Inceoglu (2004) — 가장 낮은 곳 → 아래 → 왼쪽"),
    ("층 단위 (shelf)", ShelfPacker, "#9aa5b1",
     "산업 팔레타이징 관행 — 한 층을 채우고 다음 층"),
    ("극점 (Extreme Point)", ExtremePointPacker, "#b8c1cc",
     "Crainic, Perboli & Tadei (2008) — 놓인 박스의 모서리만 후보"),
    ("순차 (첫자리)", BaselinePacker, "#cbd2d9",
     "대조군 — 회전 없음 · 좌하단 첫 자리 · 지지율도 안 봄"),
]


def _job(args):
    name, cls, seeds = args
    rs = [run_once(cls, load(), None, s, s + 1000, 3) for s in seeds]
    return name, rs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", default="docs/트랙C_결과/07_families.md")
    ap.add_argument("--png", default="docs/트랙C_결과/07_families.png")
    a = ap.parse_args()

    seeds = list(range(1, 71))[::2] if a.quick else list(range(1, 71))
    print(f"계열 {len(FAMILIES)}개 x 시드 {len(seeds)}개 — 도는 중...")
    with ProcessPoolExecutor() as ex:
        got = dict(ex.map(_job, [(n, c, seeds) for n, c, _, _ in FAMILIES]))

    rows = []
    for name, _cls, color, src in FAMILIES:
        rs = got[name]
        fr = [r[3] for r in rs]
        rows.append(dict(name=name, color=color, src=src,
                         placed=st.mean(r[1] for r in rs),
                         fill=st.mean(fr), sd=st.pstdev(fr),
                         lo=min(fr), hi=max(fr),
                         cap=st.mean(r[6] for r in rs),
                         hmax=st.mean(r[4] for r in rs),
                         fed=st.mean(r[0] for r in rs)))

    ours = rows[0]
    out = ["# 계열 비교 — 왜 heightmap 점수화인가", "",
           f"시드 **{seeds[0]}~{seeds[-1]} ({len(seeds)}개)** · 팔레트 포화까지 · "
           "실측 오차 ±3mm · 모든 계열에 **같은 박스 시퀀스**", "",
           "## 실험 설계", "",
           "모든 계열에 **똑같은 물리 제약**을 건다 — 높이 제한 · 통로 검사 · 지지율 · "
           "계획 간격. 계열 차이는 **'놓을 수 있는 칸 중 무엇을 고르나'** 하나뿐이다.",
           "",
           "> 제약을 계열마다 다르게 주면 비교가 '전략 대 전략'이 아니라 "
           "'물리를 지킴 대 안 지킴'이 된다. `순차 (첫자리)` 만 예외로 지지율을 "
           "안 보는데, 그건 그 대조군의 정체성이 '아무 생각 없이 쌓기' 이기 때문이다.",
           "",
           "## 결과", "",
           "| 계열 | 적재 개수 | 적재율(실제높이) | 최소~최대 | 적재율(용량) | 제안 대비 | 근거 |",
           "|---|---|---|---|---|---|---|"]
    for r in rows:
        d = (r["fill"] - ours["fill"]) * 100
        diff = "—" if r is ours else f"**{d:+.1f}%p**"
        out.append(f"| {r['name']} | {r['placed']:.1f} | "
                   f"**{r['fill']*100:.1f}% ± {r['sd']*100:.1f}** | "
                   f"{r['lo']*100:.1f}~{r['hi']*100:.1f}% | {r['cap']*100:.1f}% | "
                   f"{diff} | {r['src']} |")
    out += ["", "## 읽는 법", ""]
    second = max((r for r in rows if r is not ours), key=lambda r: r["fill"])
    out += [f"- 문헌 계열 중 가장 좋은 것은 **{second['name']}** ({second['fill']*100:.1f}%) 이고, "
            f"제안은 그보다 **{(ours['fill']-second['fill'])*100:+.1f}%p** 높다.",
            f"- 가장 낮은 대조군과는 **{(ours['fill']-rows[-1]['fill'])*100:+.1f}%p** 차이다.",
            "- DBLF 는 우리 점수식에서 `w_height` 만 남기고 지지·벽·먼쪽·평면 항을 껐을 "
            "때와 같다. 그 차이가 **그 항들이 실제로 하는 일**의 크기다.",
            "- 극점(EP) 계열은 후보가 O(놓인 개수) 라 아주 싸다. 그런데도 격자 전수 "
            "점수화보다 낮다면, **그 계산을 더 쓸 값어치가 있다**는 뜻이다.",
            ""]

    fig, ax = plt.subplots(figsize=(9.5, 4.6), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    names = [r["name"] for r in rows][::-1]
    vals = [r["fill"] * 100 for r in rows][::-1]
    errs = [r["sd"] * 100 for r in rows][::-1]
    cols = [r["color"] for r in rows][::-1]
    bars = ax.barh(names, vals, xerr=errs, color=cols, height=0.6,
                   error_kw=dict(ecolor=INK2, lw=1.2, capsize=3))
    for b, v in zip(bars, vals):
        ax.text(v + 1.6, b.get_y() + b.get_height() / 2, f"{v:.1f}%",
                va="center", color=INK, fontsize=10.5, fontweight="bold")
    ax.set_xlabel("적재율 (실제 최고높이 기준, %)", color=INK2)
    ax.set_xlim(0, max(vals) * 1.28)
    ax.grid(axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(colors=INK2)
    ax.set_title(f"온라인 3D 적재 — 계열별 적재율 (시드 {len(seeds)}개 평균)",
                 color=INK, fontsize=12, pad=12)
    fig.tight_layout()
    pathlib.Path(a.png).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.png, dpi=150, facecolor=SURFACE)

    pathlib.Path(a.out).write_text("\n".join(out), encoding="utf-8")
    print("\n".join(out))
    print(f"\n저장: {a.out} · {a.png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
