#!/usr/bin/env python3
"""palletris.db 를 사람이 읽는 표로 찍는다. SQL 도 sqlite3 CLI 도 필요 없다.

    python3 scripts/db_peek.py              # 요약 + 박스 목록
    python3 scripts/db_peek.py --events     # 상태 전이 이력까지
    python3 scripts/db_peek.py --box BOX00003
    python3 scripts/db_peek.py --csv out.csv

이 스크립트는 **읽기 전용**이다. 그런데도 db_service_node 와 같이 떠 있어도
안전한 이유: 스키마가 WAL 모드라 reader 한 명은 writer 를 막지 않는다.
(writer 가 둘이면 `database is locked` 가 난다 — 그래서 쓰기는 노드 하나뿐이다.)
"""
import argparse
import csv
import os
import sqlite3
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))


def find_db(path=None):
    if path:
        return os.path.expanduser(path)
    return os.path.join(os.path.dirname(_HERE), "palletris.db")


def table(rows, headers):
    """의존성 없이 표를 그린다 (tabulate 를 새로 깔지 않으려고)."""
    if not rows:
        return "  (없음)"
    cells = [[("" if c is None else str(c)) for c in r] for r in rows]
    w = [max(len(h), *(len(r[i]) for r in cells)) for i, h in enumerate(headers)]
    line = "  " + "  ".join("─" * x for x in w)
    out = ["  " + "  ".join(h.ljust(w[i]) for i, h in enumerate(headers)), line]
    out += ["  " + "  ".join(r[i].ljust(w[i]) for i in range(len(headers))) for r in cells]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="palletris.db 들여다보기")
    ap.add_argument("--db-path", default=None)
    ap.add_argument("--events", action="store_true", help="상태 전이 이력도 출력")
    ap.add_argument("--box", default=None, help="박스 하나만 자세히")
    ap.add_argument("--csv", default=None, help="boxes_sorted 를 CSV 로 덤프")
    a = ap.parse_args()

    db_path = find_db(a.db_path)
    if not os.path.isfile(db_path):
        sys.exit(f"DB 가 없다: {db_path}\n  먼저: python3 scripts/gen_master_db.py")

    # ★ 읽기 전용으로 연다. 실수로 쓰는 코드가 들어가도 DB 를 못 건드린다.
    c = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    q = c.execute
    print(f"\n{db_path}\n")

    if a.box:
        row = q("SELECT box_id, zone, status, size_w_mm, size_d_mm, size_h_mm, mass_g,"
                " color, dest, qr_payload, place_x_mm, place_y_mm, place_z_mm,"
                " place_yaw_deg FROM boxes_sorted WHERE box_id=?", (a.box,)).fetchone()
        if not row:
            print(f"  boxes_sorted 에 {a.box} 가 없다 (아직 등록 전이거나 인식 실패)")
        else:
            names = ["box_id", "zone", "status", "w_mm", "d_mm", "h_mm", "mass_g",
                     "color", "dest", "qr", "place_x", "place_y", "place_z", "place_yaw"]
            for n, v in zip(names, row):
                print(f"  {n:10s} {v}")
        print("\n── 이력 ──")
        print(table(q("SELECT status, reason, ROUND(ts,2) FROM box_events "
                      "WHERE box_id=? ORDER BY id", (a.box,)).fetchall(),
                    ["status", "reason", "ts"]))
        return

    # ── 요약 ──
    total = q("SELECT COUNT(*) FROM boxes_master").fetchone()[0]
    seen = q("SELECT COUNT(*) FROM boxes_sorted").fetchone()[0]
    placed = q("SELECT COUNT(*) FROM boxes_sorted WHERE status='PLACED'").fetchone()[0]
    failed = q("SELECT COUNT(*) FROM boxes_sorted WHERE status='PICK_FAILED'").fetchone()[0]
    rejected = q("SELECT COUNT(*) FROM boxes_sorted WHERE status='REJECTED'").fetchone()[0]
    cyc = q("SELECT AVG(placed_at - picked_at) FROM boxes_sorted "
            "WHERE status='PLACED' AND picked_at IS NOT NULL").fetchone()[0]

    # ★ 인식률의 분모는 **라인에 올라온 박스 수**다 (2026-08-25 정정).
    #   정답지 풀을 분모로 쓰면 풀을 넉넉히 깔수록 인식률이 낮게 나온다 — 거꾸로다.
    #   실측: 풀 105 개에 15 개만 흘렸더니 14 % 로 찍혔다 (실제로는 100 %).
    miss1 = q("SELECT COUNT(*) FROM scan_misses WHERE where_at='cam1'").fetchone()[0]
    arrived = seen + miss1

    print("── 요약 ──")
    print(f"  정답지 풀(boxes_master) {total} 개   ← 미리 깔아 둔 운송장·주소")
    print(f"  라인에 올라옴          {arrived} 개")
    print(f"  인식 성공              {seen} / {arrived}"
          f"{f'  ({seen/arrived*100:.0f} %)' if arrived else ''}")
    if miss1:
        print(f"  인식 실패 (cam1)       {miss1} 개")
    print(f"  적재 완료              {placed}")
    print(f"  집기 실패              {failed}")
    print(f"  폐기                   {rejected}")
    if cyc:
        print(f"  평균 사이클            {cyc:.1f} s")

    print("\n── 상태 분포 ──")
    print(table(q("SELECT status, COUNT(*) FROM boxes_sorted GROUP BY status "
                  "ORDER BY COUNT(*) DESC").fetchall(), ["status", "개수"]))

    print("\n── 구역별 ──")
    print(table(q("SELECT zone, COUNT(*), SUM(status='PLACED') FROM boxes_sorted "
                  "GROUP BY zone").fetchall(), ["zone", "전체", "적재"]))

    print("\n── 박스 목록 ──")
    print(table(q("SELECT box_id, zone, status, size_w_mm||'x'||size_d_mm||'x'||size_h_mm,"
                  " mass_g, color, place_x_mm, place_y_mm, place_z_mm "
                  "FROM boxes_sorted ORDER BY db_seq").fetchall(),
                ["box_id", "zone", "status", "치수", "질량g", "색", "px", "py", "pz"]))

    miss = q("SELECT box_id, zone FROM boxes_master WHERE box_id NOT IN "
             "(SELECT box_id FROM boxes_sorted)").fetchall()
    if miss:
        print(f"\n── 아직 안 온 / 인식 못 한 박스 {len(miss)} 개 ──")
        print(table(miss, ["box_id", "zone"]))

    rec = q("SELECT box_id, retry_count, bypass_count FROM recirculation "
            "WHERE retry_count>0 OR bypass_count>0").fetchall()
    if rec:
        # ★ retry 는 인식 실패, bypass 는 정상적인 흐름 제어다. 절대 합쳐 보지 말 것.
        print("\n── 재순환 (retry=인식실패 / bypass=구역 포화, 실패 아님) ──")
        print(table(rec, ["box_id", "retry", "bypass"]))

    if a.events:
        print("\n── 상태 전이 이력 ──")
        print(table(q("SELECT box_id, status, reason, ROUND(ts,2) FROM box_events "
                      "ORDER BY id").fetchall(), ["box_id", "status", "reason", "ts"]))

    if a.csv:
        cur = q("SELECT * FROM boxes_sorted ORDER BY db_seq")
        with open(a.csv, "w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f)
            wr.writerow([d[0] for d in cur.description])
            wr.writerows(cur.fetchall())
        print(f"\nCSV 저장 → {a.csv}")
    print()


if __name__ == "__main__":
    main()
