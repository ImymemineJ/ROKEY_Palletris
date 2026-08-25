"""파라미터 튜닝 근거표 — "왜 이 값인가" (명세서 §13 검증 및 분석).

한 번에 **하나씩만** 바꿔 현행값과 비교한다(one-at-a-time). 그리고 반드시
**튜닝 시드와 검증 시드를 나눈다**:

    튜닝 시드 1~30   — 값을 고를 때 본 판
    검증 시드 31~70  — 고른 뒤 처음 보는 판

왜 나누나: 시드 30개 평균은 표준편차가 5%p 나 된다. 한 벌만 보고 고르면 **그 30판에만
맞는 값**을 고르게 된다. 실제로 `w_level` 을 0.15 로 키우면 튜닝 시드에서 61.5% 가
나오는데 검증 시드에서는 58.7% 로 주저앉는다 — 과적합이다. 채택 기준은
"튜닝에서 올랐다" 가 아니라 **"검증에서도 같이 올랐다"** 이다.

실행:
    python3 -m packing_algo.tuning                    # 전체 (몇 분)
    python3 -m packing_algo.tuning --quick            # 시드 절반
    python3 -m packing_algo.tuning --only w_level
"""
import argparse
import copy
import statistics as st
from concurrent.futures import ProcessPoolExecutor

from packing_algo.compare_baseline import run_once
from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load

# ── 스윕 정의 ──────────────────────────────────────────────────────
# (표시 이름, 바꿀 키들, 후보값들). 키가 여러 개면 묶어서 같이 바꾼다.
SWEEPS = [
    ("w_height / w_support", ("w_height", "w_support"),
     [(0.8, 0.2), (0.7, 0.3), (0.6, 0.4), (0.5, 0.5), (0.4, 0.6)],
     "높이 우선 vs 지지 우선의 균형"),
    ("w_wall", ("w_wall",), [(0.0,), (0.1,), (0.2,), (0.3,), (0.5,)],
     "벽 밀착 가점. 자투리를 가장자리로 몰아 가운데에 큰 박스 자리를 남긴다"),
    ("w_far", ("w_far",), [(0.0,), (0.05,), (0.1,), (0.2,), (0.4,)],
     "로봇에서 먼 쪽부터. 가까운 변을 마지막에 남겨야 팔이 안 걸린다"),
    ("w_level", ("w_level",), [(0.0,), (0.02,), (0.05,), (0.1,), (0.15,)],
     "평면 유지. 윗면이 이웃과 맞으면 가점 — 절벽을 처음부터 안 만든다"),
    ("level_ring_cells", ("level_ring_cells",), [(2,), (4,), (8,), (12,)],
     "평면 유지에서 '이웃'으로 볼 거리(셀)"),
    ("min_support_ratio", ("min_support_ratio",),
     [(0.4,), (0.5,), (0.6,), (0.7,), (0.8,), (0.9,)],
     "밑면이 이만큼은 받쳐져야 놓는다. 낮출수록 적재율은 오르고 외팔보가 생긴다"),
    ("support_tol_mm", ("support_tol_mm",), [(5,), (10,), (20,), (30,)],
     "받침으로 칠 높이차. cell_mm(10) 보다 작으면 2층부터 전부 실패한다"),
    ("topk_support_check", ("topk_support_check",), [(10,), (50,), (200,), (1000,)],
     "지지율을 정밀 계산할 상위 후보 수"),
    ("plan_clearance_mm", ("plan_clearance_mm",), [(0,), (5,), (10,), (20,), (30,)],
     "발자국을 부풀려 박스 사이에 실제 틈을 남긴다 (Isaac 실측 근거)"),
    ("lookahead.k", ("__la_k",), [(1,), (3,), (5,)],
     "후보 몇 개를 실제로 놓아 보고 다음 박스를 굴려 볼 것인가"),
]

TUNE = range(1, 31)          # 값을 고를 때 보는 판
HOLD = range(31, 71)         # 고른 뒤 처음 보는 판


def _cfg_with(cfg, keys, vals):
    c = copy.deepcopy(cfg)
    for k, v in zip(keys, vals):
        if k == "__la_k":
            c["heightmap"]["lookahead"] = {"k": v, "rollouts": 3, "depth": 4}
        else:
            c["heightmap"][k] = v
    return c


def _measure(args):
    """한 설정을 시드 집합에 돌린다 → (적재율 평균, 표준편차, 적재개수 평균, 최악 시드)"""
    cfg, seeds = args
    rs = [run_once(HeightmapPacker, cfg, None, s, s + 1000, 3) for s in seeds]
    fr = [r[3] for r in rs]
    return (st.mean(fr), st.pstdev(fr), st.mean(r[1] for r in rs), min(fr))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="시드를 절반만 쓴다")
    ap.add_argument("--only", default=None, help="이 이름의 스윕만")
    ap.add_argument("--out", default="docs/트랙C_결과/06_tuning.md")
    a = ap.parse_args()

    cfg = load()
    tune = list(TUNE)[::2] if a.quick else list(TUNE)
    hold = list(HOLD)[::2] if a.quick else list(HOLD)
    cur = {k: cfg["heightmap"].get(k) for k in
           ("w_height", "w_support", "w_wall", "w_far", "w_level", "level_ring_cells",
            "min_support_ratio", "support_tol_mm", "topk_support_check",
            "plan_clearance_mm")}
    cur["__la_k"] = (cfg["heightmap"].get("lookahead") or {}).get("k", 1)

    jobs, meta = [], []
    for name, keys, vals, why in SWEEPS:
        if a.only and a.only != name:
            continue
        for v in vals:
            jobs.append((_cfg_with(cfg, keys, v), tune))
            meta.append((name, keys, v, why, "tune"))
            jobs.append((_cfg_with(cfg, keys, v), hold))
            meta.append((name, keys, v, why, "hold"))

    print(f"설정 {len(jobs)//2}개 x 시드 {len(tune)}+{len(hold)}개 — 도는 중...")
    with ProcessPoolExecutor() as ex:
        res = list(ex.map(_measure, jobs, chunksize=1))

    # (name, vals) -> {"tune": r, "hold": r}
    table = {}
    for (name, keys, v, why, which), r in zip(meta, res):
        table.setdefault((name, keys, why), {}).setdefault(v, {})[which] = r

    out = ["# 파라미터 튜닝 근거 — 무엇을 바꿔 무엇이 나왔나", "",
           f"시드 **튜닝 {tune[0]}~{tune[-1]} ({len(tune)}개)** · "
           f"**검증 {hold[0]}~{hold[-1]} ({len(hold)}개)** · 팔레트 포화까지 · 실측 오차 ±3mm",
           "",
           "한 번에 하나만 바꿨다. **★ = 현행값.** 채택 기준은 튜닝 시드가 아니라 "
           "**검증 시드에서도 같이 오르는가** 이다.", ""]
    for (name, keys, why), rows in table.items():
        out += [f"## `{name}`", "", f"> {why}", "",
                "| 값 | 튜닝 적재율 | 검증 적재율 | 검증 최악 시드 | 적재 개수 | 판정 |",
                "|---|---|---|---|---|---|"]
        cur_v = tuple(cur[k] for k in keys)
        base = rows.get(cur_v, {}).get("hold")
        for v, r in rows.items():
            t, h = r.get("tune"), r.get("hold")
            if not t or not h:
                continue
            mark = " ★" if v == cur_v else ""
            label = "/".join(str(x) for x in v)
            if v == cur_v:
                verdict = "**현행**"
            elif base is None:
                verdict = "—"
            else:
                d = (h[0] - base[0]) * 100
                # 검증에서 유의미하게 올라야 채택 후보다. 0.5%p 는 시드 잡음 수준.
                verdict = ("채택 후보" if d > 0.5 else
                           "차이 없음" if d > -0.5 else "기각")
                verdict += f" ({d:+.1f}%p)"
            out.append(f"| {label}{mark} | {t[0]*100:.1f}% ± {t[1]*100:.1f} | "
                       f"**{h[0]*100:.1f}%** ± {h[1]*100:.1f} | {h[3]*100:.1f}% | "
                       f"{h[2]:.1f} | {verdict} |")
        out.append("")

    import pathlib
    p = pathlib.Path(a.out)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text("\n".join(out), encoding="utf-8")
    print("\n".join(out))
    print(f"\n저장: {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
