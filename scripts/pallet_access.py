#!/usr/bin/env python3
"""팔레트 접근성 — 쌓인 더미에서 꺼내기 쉬운 순서와 그 정량 지표.

    python3 scripts/pallet_access.py            # 구역 B
    python3 scripts/pallet_access.py --zone A

★ API 키가 필요 없다. DB 의 실측 적재 위치만 읽는다.
DB 는 **읽기 전용**으로 연다 — db_service_node 가 유일 writer 라는 규약을 지킨다.
"""
import argparse
import pathlib
import sqlite3
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src" / "packing_db"))

from packing_db.accessibility import (best_visit_order, unload_order,  # noqa: E402
                                     visit_moves)
from packing_db.delivery_points import DEPOT  # noqa: E402
import math  # noqa: E402

DEF_DB = pathlib.Path(__file__).resolve().parents[1] / "palletris.db"


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
        "       s.size_w_mm, s.size_d_mm, s.size_h_mm, m.address, m.delivery_seq,"
        "       m.lat, m.lon "
        "FROM boxes_sorted s LEFT JOIN boxes_master m USING(box_id) "
        "WHERE s.zone=? AND s.status='PLACED' AND s.place_z_mm IS NOT NULL "
        "ORDER BY s.db_seq", (a.zone,)).fetchall()
    con.close()
    if not rows:
        sys.exit(f"구역 {a.zone} 에 적재된 박스가 없다 — 먼저 시나리오를 돌려야 한다")

    boxes, meta, geo = [], {}, {}
    for bid, x, y, z, yaw, w, d, h, addr, seq, lat, lon in rows:
        boxes.append({"box_id": bid, "x": x, "y": y, "z": z,
                      "w": w, "d": d, "h": h, "yaw": yaw or 0.0})
        addr = addr or "(주소 없음)"
        meta[bid] = (addr, seq if seq is not None else 10**9)
        # ★ 좌표가 없으면 거리 비교를 못 한다 — 물류센터 자리로 두어 0 으로 만든다.
        geo[addr] = (lat, lon) if lat is not None else (DEPOT[1], DEPOT[2])

    order = unload_order(boxes)
    print(f"구역 {a.zone} · 적재 {len(boxes)} 개 · 배송지 {len(set(m[0] for m in meta.values()))} 곳\n")

    print("  ── 팔레트 (위에서부터 벗겨지는 순서) ──")
    for b in order:
        print(f"  {b['seq']:>2} 겹{b['layer']}  {b['box_id']:10} 윗면 {b['z']+b['h']:>5}mm  "
              f"{meta[b['box_id']][0]}")

    def hav(p, q):
        la1, lo1, la2, lo2 = map(math.radians, (p[0], p[1], q[0], q[1]))
        h = (math.sin((la2 - la1) / 2) ** 2
             + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2)
        return 2 * 6371000 * math.asin(math.sqrt(h))

    D = (DEPOT[1], DEPOT[2])

    def dist_fn(addr_seq):
        """물류센터 왕복 총 이동거리(km). 순서를 고를 때만 쓰는 직선거리다 —
        실제 도로 거리는 그 뒤 scripts/plan_route.py 가 경로 API 로 잰다."""
        pts = [D] + [geo[x] for x in addr_seq] + [D]
        return sum(hav(pts[i], pts[i + 1]) for i in range(len(pts) - 1)) / 1000

    try:
        visits = best_visit_order(boxes, lambda b: meta[b][0], dist_fn)
    except ValueError as e:
        sys.exit(f"\n{e}")

    seq_order = sorted({m[0] for m in meta.values()},
                       key=lambda ad: min(m[1] for m in meta.values() if m[0] == ad))
    seq_visits = [{"address": ad, "box_ids": [b["box_id"] for b in boxes
                                              if meta[b["box_id"]][0] == ad], "seq": i}
                  for i, ad in enumerate(seq_order, 1)]

    print(f"\n  ── 배송 순서 (전수 탐색 · 한 곳은 한 번만) ──")
    for g in visits:
        print(f"  {g['seq']:>2}. {g['address']:34} 박스 {len(g['box_ids'])}개")

    n_best = visit_moves(boxes, visits)
    n_seq = visit_moves(boxes, seq_visits)
    d_best, d_seq = dist_fn([g["address"] for g in visits]), dist_fn(seq_order)
    print(f"\n★ 지표")
    print(f"    {'':22}{'들어내기':>9}{'직선거리':>11}")
    print(f"    {'전수 탐색 순서':22}{n_best:>7} 회{d_best:>9.2f} km")
    print(f"    {'delivery_seq 순서':22}{n_seq:>7} 회{d_seq:>9.2f} km")
    print("\n  ※ 실제 도로 거리·시간은 scripts/plan_route.py 가 경로 API 로 잰다.")


if __name__ == "__main__":
    main()
