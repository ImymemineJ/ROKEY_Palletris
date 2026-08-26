#!/usr/bin/env python3
"""소터 3개를 공칭 속도로 돌린다 — **돌고 있는 시뮬에 소켓으로 붙어서** (2026-08-26 김병직).

## 왜 필요한가

`demo_stack15_test1_1.py` 는 소터에 **방향만** 준다:

    kw = {"cmd": "set_sorter", "zone": zone, "divert": bool(want_div)}   # mps 없음

그런데 `conveyor_ctl.h_set_sorter` 규약은 "`mps` 를 주면 속도도 같이 바꾼다
(**안 주면 건드리지 않는다**)" 이고, A47 애셋의 `SorterSpeed` **기본값은 0.0** 이다.
`belt_graphs` 에도 소터는 없어서(16 세그먼트 전부 롤러) `set_velocity` 로도 안 켜진다.

-> 소터가 선 채로 남고, 본선 롤러의 소터 자리에는 구멍이 있어 **박스가 거기서 멈춘다.**
   `sim_main --autorun` 은 `cc.run_all()` 이 소터까지 켜므로 이 문제가 없다.

실측 2026-08-26 (test_3 · PL_ZONES=A,B,C · Videos/260826):

    대기 30 s  000(-6.12) 001(-4.77) 002(-3.15) 003(-1.54)   <- 4.6 m 에 흩어짐
    대기 45 s  000(-7.03) 001(-6.66) 002(-6.05) 003(-4.61)
    대기 60 s  000(-7.57) 001(-7.20) 002(-6.79) 003(-6.34)   <- 1.6 m 로 뭉침
    [투입] 보류 — 본선이 붐빈다 (소터 구간 최대 2 개) · 라인 0개

박스가 느려진 게 아니라 **존 A 분기(-8048~-7144)에서 막혀 뒤가 밀린** 것이다.

## 쓰는 법 — 시뮬을 끄지 않고

    python3 scripts/start_sorters.py               # 데모(8797)
    python3 scripts/start_sorters.py --port 8765   # sim_main

★ 근본 해결은 드라이버가 기동 때 소터를 켜는 것이다 (`run_all()` 처럼).
  이건 이미 돌고 있는 실행을 살리는 용도다.
"""
import argparse
import json
import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                                    # noqa: E402


def call(f, **req):
    f.write(json.dumps(req) + "\n")
    f.flush()
    return json.loads(f.readline())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8797,
                    help="8797 = demo_stack15_test1_1 · 8765 = sim_main")
    ap.add_argument("--zones", default="A,B,C")
    ap.add_argument("--mps", type=float, default=None,
                    help="기본은 system.yaml(+오버레이) 의 conveyor.belt_velocity_mps")
    a = ap.parse_args()

    mps = a.mps
    if mps is None:
        mps = float(_paths.system_config()["conveyor"]["belt_velocity_mps"])

    try:
        s = socket.create_connection(("127.0.0.1", a.port), timeout=3.0)
    except OSError as e:
        print("소켓 %d 에 못 붙었다 (%s) — 시뮬이 떠 있는지, 포트가 맞는지 확인" % (a.port, e))
        return 1
    s.settimeout(5.0)
    f = s.makefile("rw")
    print("소켓 127.0.0.1:%d · 소터 속도 %.3f m/s" % (a.port, mps))
    bad = 0
    for z in [x.strip().upper() for x in a.zones.split(",") if x.strip()]:
        # divert=False = 본선 그대로 통과. 분류는 드라이버가 박스마다 다시 건다.
        r = call(f, cmd="set_sorter", zone=z, divert=False, mps=mps)
        ok = bool(r.get("ok"))
        bad += 0 if ok else 1
        print("  존 %s -> %s%s" % (z, "OK" if ok else "실패", ""
                                   if ok else "  %s" % r.get("error")))
    f.close()
    s.close()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
