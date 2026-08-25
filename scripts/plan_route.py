#!/usr/bin/env python3
"""팔레트 → 배송 순서 → 실제 도로 경로.

    export NCP_API_KEY_ID=... NCP_API_KEY=...
    python3 scripts/plan_route.py --zone B

무엇을 하나:
  ① 적재된 팔레트를 읽어 **꺼내기 쉬운 순서**를 계산한다 (API 0 회)
  ② 그중 들어내기가 최소인 후보만 남긴다 (API 0 회)
  ③ 남은 후보를 **실제 도로거리**로 재서 제일 짧은 것을 고른다

★ 순서는 우리가 정한다. 네이버는 "이 순서로 가면 몇 km 인가" 만 답한다.
DB 는 **읽기 전용**으로 연다 — db_service_node 가 유일 writer 라는 규약을 지킨다.
"""
import argparse
import itertools
import math
import pathlib
import sqlite3
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src" / "packing_db"))

from packing_db.accessibility import _take_group, is_above          # noqa: E402
from packing_db.delivery_points import DEPOT                        # noqa: E402
from packing_db.route import RouteError, best_order                 # noqa: E402

DEF_DB = pathlib.Path(__file__).resolve().parents[1] / "palletris.db"
#: 순서 후보를 전부 셀 수 있는 배송지 수. 한 곳 늘 때마다 곱으로 는다.
MAX_ADDRS = 9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zone", default="B")
    ap.add_argument("--db", type=pathlib.Path, default=DEF_DB)
    a = ap.parse_args()
    if not a.db.exists():
        sys.exit(f"DB 가 없다: {a.db}")

    con = sqlite3.connect(f"file:{a.db}?mode=ro", uri=True)
    rows = con.execute(
        "SELECT s.box_id, s.place_x_mm, s.place_y_mm, s.place_z_mm, s.place_yaw_deg,"
        "       s.size_w_mm, s.size_d_mm, s.size_h_mm, m.address, m.lat, m.lon "
        "FROM boxes_sorted s JOIN boxes_master m USING(box_id) "
        "WHERE s.zone=? AND s.status='PLACED' AND s.place_z_mm IS NOT NULL",
        (a.zone,)).fetchall()
    con.close()
    if not rows:
        sys.exit(f"구역 {a.zone} 에 적재된 박스가 없다 — 먼저 시나리오를 돌려야 한다")

    boxes = [{"box_id": r[0], "x": r[1], "y": r[2], "z": r[3],
              "w": r[5], "d": r[6], "h": r[7], "yaw": r[4] or 0.0} for r in rows]
    addr = {r[0]: r[8] for r in rows}
    points = {r[8]: (r[9], r[10]) for r in rows}
    group = {}
    for b in boxes:
        group.setdefault(addr[b["box_id"]], []).append(b["box_id"])
    names = list(group)
    if len(names) > MAX_ADDRS:
        sys.exit(f"배송지가 {len(names)} 곳이라 순서를 전부 셀 수 없다 (상한 {MAX_ADDRS})")

    # ── ① 들어내기 횟수 — API 없이 계산된다 ──────────────────────
    above = {b["box_id"]: {o["box_id"] for o in boxes if is_above(o, b)} for b in boxes}

    def lifts(order):
        left = {b["box_id"] for b in boxes}
        return sum(_take_group(group[x], above, left) for x in order)

    scored = [(lifts(o), o) for o in itertools.permutations(names)]
    best_lift = min(n for n, _ in scored)
    cands = [o for n, o in scored if n == best_lift]

    print(f"구역 {a.zone} · 적재 {len(boxes)} 개 · 배송지 {len(names)} 곳")
    print(f"  순서 후보 {len(scored):,} 가지 → 들어내기 최소({best_lift} 회) 후보 "
          f"{len(cands):,} 가지")

    # ── ② 남은 후보를 실제 도로거리로 잰다 ───────────────────────
    D = (DEPOT[1], DEPOT[2])

    def hav(p, q):
        la1, lo1, la2, lo2 = map(math.radians, (p[0], p[1], q[0], q[1]))
        h = (math.sin((la2 - la1) / 2) ** 2
             + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2)
        return 2 * 6371000 * math.asin(math.sqrt(h))

    def straight(o):
        p = [D] + [points[x] for x in o] + [D]
        return sum(hav(p[i], p[i + 1]) for i in range(len(p) - 1))

    try:
        order, r = best_order(
            cands, D, points, prefilter=straight,
            on_try=lambda o, res, i, n: print(f"\r  도로거리 측정 {i}/{n}",
                                              end="", flush=True))
    except RouteError as e:
        sys.exit(f"\n경로 계산 실패:\n{e}")
    print(f"\r  도로거리 측정 {r['tried']} 회 완료"
          + (f" (직선거리로 {r['trimmed']} 개는 추려 냄)" if r["trimmed"] else ""))

    print(f"\n출발  {DEPOT[0]}")
    for i, x in enumerate(order, 1):
        print(f"  {i:2}. {x:34} 박스 {len(group[x])}개")
    print(f"  ↩ {DEPOT[0]} (복귀)")

    print(f"\n총 거리   {r['distance_m']/1000:.2f} km")
    print(f"총 시간   {r['duration_s']/60:.1f} 분")
    print(f"들어내기  {best_lift} 회")
    if r["toll_won"]:
        print(f"통행료    {r['toll_won']:,} 원")
    print(f"연료비    {r['fuel_won']:,} 원 · 경로점 {len(r['path'])} 개")


if __name__ == "__main__":
    main()
