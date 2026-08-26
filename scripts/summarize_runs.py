#!/usr/bin/env python3
"""`results/*.txt` 실행 로그에서 **비교용 한 줄**을 뽑는다.

왜: 하루에 30번 넘게 돌리면 어떤 조건이 어떤 숫자를 냈는지 로그를 뒤져 찾게 된다.
    실제로 그러다 결론을 두 번 뒤집었다 (CLAUDE.md §10-1). 조건과 결과를 **같은 줄**에
    놓아야 "무엇이 무엇 때문인지" 를 안 헷갈린다.

    python3 scripts/summarize_runs.py results/pred130b_*.txt
    python3 scripts/summarize_runs.py --md results/*.txt > docs/표.md
"""
import argparse
import re
import sys
from pathlib import Path

# (열 이름, 정규식, 형변환)  — 로그에 없으면 "-" 로 남긴다
HEAD = [
    (r"이격 ([0-9.]+)", "이격"), (r"속도배율 ([0-9.]+)", "속도"), (r"시드 ([0-9]+)", "시드"),
    (r"밖으로 ([0-9.]+)\)", "밖"), (r"간격 ([0-9]+) mm", "간격"),
    (r"min_support_ratio\s+[0-9.]+ -> ([0-9.]+)", "지지율"),
    (r"lookahead\.k\s+[0-9]+ -> ([0-9]+)", "앞보기k"),
    (r"tower_ring_mm\s+[0-9]+ -> ([0-9]+)", "기둥억제"),
    (r"corridor_radius_mm\s+[0-9]+ -> ([0-9]+)", "통로반경"),
    (r"corridor_tier2_radius_mm\s+[0-9]+ -> ([0-9]+)", "2단반경"),
    (r"IK 가지  (\S+)", "IK가지"), (r"툴 기울기 ([0-9.]+) 도", "기울기"),
]
BODY = [
    (r"놓은 것\s+(\d+) / (\d+)", "적재/투입"),
    (r"\(적재 \d+ · 버림 (\d+) · 실패 (\d+)\)", "버림/실패"),
    (r"실측 최상단\s+(\d+) mm", "최상단"),
    (r"최대 knock\s+([0-9.]+) mm", "knock"),
    (r"팔레트 밖\s+([0-9.]+) mm", "밖으로"),
    (r"사이클 평균 ([0-9.]+) s", "사이클"),
    (r"RESULT: (\S+)", "판정"),
]


def one(path):
    try:
        t = Path(path).read_text(errors="replace")
    except OSError:
        return None
    # [diag] 줄은 양이 많고 요약과 무관하다
    t = "\n".join(l for l in t.splitlines() if not l.startswith("[diag]"))
    row = {"실행": Path(path).stem}
    for pat, name in HEAD + BODY:
        m = re.search(pat, t)
        row[name] = "/".join(m.groups()) if m else "-"
    # 적재율은 마지막 값이 정본이다 (매 사이클 찍힌다)
    fr = re.findall(r"적재율 (0\.[0-9]+)", t)
    row["적재율"] = "%.1f%%" % (float(fr[-1]) * 100) if fr else "-"
    row["낙하"] = str(len(re.findall(r"협곡 낙하", t)) or "-")
    row["접촉"] = str(len(re.findall(r"접촉 감지", t)) or "-")
    return row


def main():
    ap = argparse.ArgumentParser(description="실행 로그 -> 비교표")
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--md", action="store_true", help="마크다운 표로")
    a = ap.parse_args()
    rows = [r for r in (one(p) for p in a.logs) if r]
    if not rows:
        sys.exit("읽을 로그가 없다")
    cols = (["실행", "시드", "이격", "지지율", "앞보기k", "간격", "밖", "기둥억제",
             "통로반경", "2단반경", "적재/투입", "버림/실패", "적재율", "최상단",
             "knock", "밖으로", "낙하", "접촉", "사이클", "판정"])
    if a.md:
        print("| " + " | ".join(cols) + " |")
        print("|" + "---|" * len(cols))
        for r in rows:
            print("| " + " | ".join(str(r.get(c, "-")) for c in cols) + " |")
    else:
        w = {c: max(len(c), *(len(str(r.get(c, "-"))) for r in rows)) for c in cols}
        print("  ".join(c.ljust(w[c]) for c in cols))
        print("  ".join("-" * w[c] for c in cols))
        for r in rows:
            print("  ".join(str(r.get(c, "-")).ljust(w[c]) for c in cols))


if __name__ == "__main__":
    main()
