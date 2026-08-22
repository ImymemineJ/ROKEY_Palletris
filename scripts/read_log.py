#!/usr/bin/env python3
"""Isaac Sim Script Editor 출력을 터미널에서 읽는다.

Script Editor 콘솔은 Isaac 창 안에만 보이지만, `print()` 는 Kit 로그 파일에
`[py stdout]` 으로 남는다. 그래서 **결과를 복사해 옮길 필요가 없다.**

    python3 read_log.py            # 가장 최근 세션의 마지막 실행분
    python3 read_log.py -n 2       # 마지막에서 2번째 실행분
    python3 read_log.py --all      # 그 세션의 모든 출력
    python3 read_log.py --err      # stderr(예외 트레이스)도 같이

로그 위치: ~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/<버전>/kit_<날짜>.log
Isaac 을 껐다 켜면 새 파일이 생기므로 **가장 최근 파일**을 자동으로 고른다.
"""
import argparse
import re
import sys
from pathlib import Path

LOGDIR = Path.home() / ".nvidia-omniverse/logs/Kit"
# 실행 하나의 시작을 알리는 표식. 러너/진단기가 맨 끝에 찍는 문자열들이다.
STARTS = ("started", "diag started", "diag2 started", "probe started")


def newest_log():
    logs = sorted(LOGDIR.glob("*/*/kit_*.log"), key=lambda p: p.stat().st_mtime)
    if not logs:
        sys.exit(f"로그를 찾을 수 없다: {LOGDIR}")
    return logs[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=1, help="마지막에서 n번째 실행 (기본 1)")
    ap.add_argument("--all", action="store_true", help="세션 전체 출력")
    ap.add_argument("--err", action="store_true", help="stderr 도 포함")
    ap.add_argument("--log", help="로그 파일 직접 지정")
    a = ap.parse_args()

    path = Path(a.log) if a.log else newest_log()
    pat = r"\[py std(?:out|err)\]: (.*)$" if a.err else r"\[py stdout\]: (.*)$"

    out = []
    for ln in path.read_text(errors="replace").splitlines():
        m = re.search(pat, ln)
        if m:
            out.append(m.group(1))

    print(f"# {path.name}  ({len(out)} 줄)\n", file=sys.stderr)
    if a.all or not out:
        print("\n".join(out))
        return

    marks = [i for i, t in enumerate(out) if t.strip() in STARTS]
    if not marks:
        print("\n".join(out))
        return
    i = marks[max(0, len(marks) - a.n)]
    print("\n".join(out[i:]))


if __name__ == "__main__":
    main()
