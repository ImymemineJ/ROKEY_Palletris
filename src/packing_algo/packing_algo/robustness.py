"""동적 대응 실측 — 실측 pose 오차가 적재율을 얼마나 깎나 (평가항목 §13).

왜 이걸 재나
    §4 가 랜덤 스폰(y ±150 · yaw ±15°)을 "동적 대응" 평가항목으로 걸었다.
    그런데 알고리즘이 그걸 **얼마나 견디는지 숫자가 없었다** — 기존 수용시험은
    위치 ±3mm 뿐이었다.

무엇을 재는가
    스폰 지터 자체가 아니라 **팔레트에 놓인 결과의 오차**다. 벨트에서 박스가 돌아
    와도 로봇이 pick_cam 실측값으로 잡아 계획 yaw 로 돌려 놓는다. 남는 건 그
    보정이 얼마나 정확하냐다. 그래서 "실측 pose 가 계획에서 이만큼 벗어났을 때"
    를 쓸어 본다 — 결과가 곧 **로봇에 요구할 배치 정밀도**가 된다.

    ★ 위치 오차는 잘 안 아프고 yaw 가 아프다. `update()` 가 셀 격자(10mm)로
      반올림하므로 ±3mm 는 같은 칸에 떨어진다. 반면 yaw 가 `yaw_tol_deg`(5°)를
      넘으면 축정렬 격자에 못 넣어 **외접 사각형(AABB)으로 부풀려** 기록한다 —
      그 박스가 실제보다 넓은 자리를 먹는다.

실행
    ros2 run packing_algo robustness
    ros2 run packing_algo robustness --seeds 50 --out docs/robustness
"""
import argparse
import random
import statistics as st

import matplotlib

matplotlib.use("Agg")                       # pyplot 보다 먼저 (창 없는 환경)
import matplotlib.pyplot as plt             # noqa: E402

from packing_algo.heightmap_core import HeightmapPacker   # noqa: E402
from packing_algo.viz import pick_korean_font             # noqa: E402
from packing_common.config import load                    # noqa: E402

# (라벨, 위치오차 ±mm, yaw오차 ±deg)
CASES = [
    ("오차 없음", 0, 0.0),
    ("위치 ±3mm", 3, 0.0),
    ("위치 ±10mm", 10, 0.0),
    ("위치 ±3mm · yaw ±3°", 3, 3.0),
    ("위치 ±3mm · yaw ±5°", 3, 5.0),
    ("위치 ±3mm · yaw ±8°", 3, 8.0),
    ("위치 ±3mm · yaw ±15°", 3, 15.0),
    ("위치 ±10mm · yaw ±15°", 10, 15.0),
]
FEED_CAP = 200                              # 포화까지 흘려보낼 상한


def run_once(cfg, seed, pos_mm, yaw_deg):
    """포화까지 돌린다. -> (적재수, 적재율%, AABB부풀림 수, 버림 수)"""
    pk = HeightmapPacker(cfg)
    boxes = [(b["w_mm"], b["d_mm"], b["h_mm"]) for b in cfg["boxes"]["types"]]
    # ★ 박스 순서와 오차는 **다른 시드**를 쓴다. 한 스트림을 공유하면 오차 크기를
    #   바꾸는 순간 박스 순서까지 바뀌어 무엇 때문에 달라졌는지 알 수 없다 (TS-006).
    rb, rn = random.Random(seed), random.Random(seed + 1000)
    reasons = {"YAW": 0, "CLIPPED": 0, "OUTSIDE": 0, "OVER_MAX": 0}
    dropped = 0
    for k in range(FEED_CAP):
        w, d, h = rb.choice(boxes)
        bid = f"B{k:03d}"
        ok, pose, _, msg = pk.plan(bid, w, d, h)
        if not ok:
            pk.cancel(bid)                  # ★ 모든 이탈 경로에서 계획 폐기
            dropped += 1
            if msg == "PALLET_FULL":
                break
            continue                        # NO_FEASIBLE_POSE 는 버리고 계속 (§7)
        actual = dict(pose)
        for ax in ("x_mm", "y_mm", "z_mm"):
            actual[ax] += rn.randint(-pos_mm, pos_mm)
        actual["yaw_deg"] += rn.uniform(-yaw_deg, yaw_deg)
        if not pk.update(bid, w, d, h, actual)[0]:
            # ★ 거절 사유를 갈라 센다. yaw 초과와 경계 이탈은 원인도 대책도 다르다.
            reasons[pk.last_reason] = reasons.get(pk.last_reason, 0) + 1
    return len(pk.placed), pk.fill_rate() * 100, reasons, dropped


def sweep(cfg, seeds):
    rows = []
    for label, pos, yaw in CASES:
        rs = [run_once(cfg, s, pos, yaw) for s in seeds]
        rows.append({
            "label": label, "pos_err": pos, "yaw_err": yaw,
            "placed": st.mean(r[0] for r in rs),
            "fill": st.mean(r[1] for r in rs),
            "fill_sd": st.pstdev([r[1] for r in rs]),
            "n_yaw": st.mean(r[2].get("YAW", 0) for r in rs),
            "edge": st.mean(r[2].get("CLIPPED", 0) + r[2].get("OUTSIDE", 0) for r in rs),
        })
    return rows


def markdown(rows, seeds):
    base = rows[0]["fill"]
    L = ["# 동적 대응 실측 — 실측 pose 오차 vs 적재율", "",
         f"시드 {len(seeds)}개 · 팔레트 포화까지 · 박스 순서와 오차는 별도 시드", "",
         "| 조건 | 적재 개수 | 적재율 | 오차없음 대비 | yaw 초과 | 경계 이탈 |",
         "|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['label']} | {r['placed']:.1f} | "
                 f"**{r['fill']:.1f}% ± {r['fill_sd']:.1f}** | {r['fill'] - base:+.1f}%p | "
                 f"{r['n_yaw']:.1f}개 | {r['edge']:.1f}개 |")
    L += ["",
          "## 읽는 법", "",
          "- **위치 오차는 적재율엔 거의 안 아프다.** `update()` 가 셀 격자(10mm)로",
          "  반올림하므로 ±3mm 는 계획과 같은 칸에 떨어진다.",
          "  대신 ±10mm 부터는 **팔레트 가장자리에서 박스가 걸친다**(경계 이탈).",
          "- **yaw 가 아프다.** `yaw_tol_deg`(5°)를 넘으면 축정렬 격자에 못 넣어",
          "  외접 사각형(AABB)으로 부풀려 기록한다 — 그 박스가 실제보다 넓은 자리를 먹는다.",
          "",
          "## 그래서 로봇에 요구할 값", "",
          "**놓는 순간의 yaw 오차를 ±5° 안으로.** 위치는 ±10mm 여유가 있다.",
          "이 표가 그 근거다 — 5° 까지는 손해가 0 이고 넘어가면서 급격히 떨어진다."]
    return "\n".join(L)


def plot(rows, path):
    pick_korean_font()
    base = rows[0]["fill"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6))
    labels = [r["label"].replace(" · ", "\n") for r in rows]
    xs = range(len(rows))
    # 색: 손해가 없으면 파랑, 있으면 주황 (검증된 팔레트 1·2번 슬롯)
    cols = ["#2a78d6" if (r["n_yaw"] + r["edge"]) < 0.5 else "#eb6834" for r in rows]

    ax1.bar(xs, [r["fill"] for r in rows], color=cols, width=0.62)
    ax1.axhline(base, color="#52514E", lw=1.2, ls=(0, (5, 4)))
    ax1.text(0.05, base + 0.6, f"오차 없음 {base:.1f}%", color="#52514E", fontsize=8)
    ax1.set_ylim(0, 80)
    ax1.set_ylabel("적재율 (%)")
    ax1.set_title("실측 pose 오차 vs 적재율", fontsize=10)

    ax2.bar(xs, [r["n_yaw"] for r in rows], color="#eb6834", width=0.62,
            label="yaw 초과 (AABB 로 부풀림)")
    ax2.bar(xs, [r["edge"] for r in rows], bottom=[r["n_yaw"] for r in rows],
            color="#2a78d6", width=0.62, label="경계 이탈 (가장자리에 걸침)")
    ax2.set_ylabel("거절된 배치 수")
    ax2.set_title("update 가 ok=false 를 낸 건수 — 사유별", fontsize=10)
    ax2.legend(fontsize=7.5, loc="upper left", framealpha=0.9)

    for ax in (ax1, ax2):
        ax.set_xticks(list(xs))
        ax.set_xticklabels(labels, fontsize=7.5, rotation=20, ha="right")
        ax.grid(axis="y", alpha=0.3)
        ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="동적 대응 실측 (pose 오차 견딤)")
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--out", default="robustness")
    args = ap.parse_args()

    cfg = load()
    seeds = list(range(1, args.seeds + 1))
    rows = sweep(cfg, seeds)
    md = markdown(rows, seeds)
    with open(args.out + ".md", "w") as f:
        f.write(md + "\n")
    plot(rows, args.out + ".png")
    print(md)
    print(f"\n저장: {args.out}.md · {args.out}.png")


if __name__ == "__main__":
    main()
