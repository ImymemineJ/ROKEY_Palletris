#!/usr/bin/env python3
"""종료 지표 리포트 — 트랙 D 가이드 D3. 발표의 "결과 수치" 가 여기서 나온다.

    python3 scripts/report.py
    python3 scripts/report.py --csv results.csv
    python3 scripts/report.py --watch          # 2 초마다 새로 그린다

쿼리는 packing_db/metrics.py 에 있다. db_service_node 의 /metrics/summary 와
**같은 함수**를 쓴다 — 복붙하면 실시간 화면과 최종 보고서의 숫자가 갈린다.

읽기 전용(mode=ro)으로 연다. db_service_node 가 돌고 있어도 안전하다 —
스키마가 WAL 이라 reader 는 writer 를 막지 않는다. writer 가 둘일 때만
`database is locked` 가 난다 (작업명세서 §8.5).
"""
import argparse
import csv
import os
import sqlite3
import sys
import time

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src", "packing_db"))
sys.path.insert(0, os.path.join(_HERE, "..", "src", "packing_common"))

from packing_common.config import load                      # noqa: E402
from packing_db.metrics import collect, fmt_rate             # noqa: E402


def table(rows, headers):
    if not rows:
        return "  (없음)"
    cells = [[("" if c is None else str(c)) for c in r] for r in rows]
    w = [max(len(h), *(len(r[i]) for r in cells)) for i, h in enumerate(headers)]
    out = ["  " + "  ".join(h.ljust(w[i]) for i, h in enumerate(headers)),
           "  " + "  ".join("─" * x for x in w)]
    out += ["  " + "  ".join(r[i].ljust(w[i]) for i in range(len(headers))) for r in cells]
    return "\n".join(out)


def render(db, cfg, zone):
    m = collect(db, zone)
    stage = cfg["stage"]
    out = []
    out.append(f"\n{'═' * 62}")
    out.append(f"  Project Palletris — 실행 결과  (stage {stage} · 구역 {zone})")
    out.append("═" * 62)

    out.append("\n【 처리량 】")
    out.append(f"  투입 예정(정답지)     {m['total_boxes']:>6d} 개")
    out.append(f"  인식 성공             {m['scanned_count']:>6d} 개   "
               f"인식률 {fmt_rate(m['recognition_rate'])}")
    out.append(f"  적재 완료             {m['placed_count']:>6d} 개")
    out.append(f"  집기 실패             {m['pick_failed_count']:>6d} 개")
    out.append(f"  폐기(자리 없음 등)    {m['rejected_count']:>6d} 개")

    out.append("\n【 품질 】")
    # ★ 성공률의 분모는 PLACED + PICK_FAILED 다. 폐기(REJECTED)는 로봇 잘못이
    #   아니라 팔레트에 자리가 없던 것이라 로봇 성공률에서 뺀다.
    out.append(f"  pick&place 성공률     {fmt_rate(m['place_success_rate']):>7s}"
               f"   (분모 = 적재 + 집기실패)")
    out.append(f"  평균 사이클           "
               f"{fmt_rate(m['cycle_avg_s'], pct=False):>7s} s  (PICKED → PLACED)")
    out.append(f"  로봇 유휴율           {fmt_rate(m['robot_idle_rate']):>7s}"
               f"   (1 − 일한 시간 / 총 실행 시간)")
    out.append(f"  총 실행 시간          {m['elapsed_s']:>7.1f} s")

    fill = db.execute("SELECT COUNT(*) FROM boxes_sorted WHERE status='PLACED'").fetchone()[0]
    top = db.execute("SELECT MAX(place_z_mm + size_h_mm) FROM boxes_sorted "
                     "WHERE status='PLACED'").fetchone()[0]
    out.append("\n【 적재 】")
    out.append(f"  적재 개수             {fill:>6d} 개")
    # ★ 최고 높이는 DB 의 실측 pose 로 다시 잰다. algo 의 heightmap 과 어긋나면
    #   계획값을 되보내고 있다는 뜻이다 (CLAUDE.md §3-4 규약 3).
    out.append(f"  최고 높이(실측)       {(top or 0):>6d} mm  "
               f"(한계 {cfg['pallet']['max_stack_mm']})")

    out.append("\n【 상태 분포 】")
    out.append(table(db.execute("SELECT status, COUNT(*) FROM boxes_sorted "
                                "GROUP BY status ORDER BY COUNT(*) DESC").fetchall(),
                     ["status", "개수"]))

    rec = db.execute("SELECT box_id, retry_count, bypass_count FROM recirculation "
                     "WHERE retry_count>0 OR bypass_count>0").fetchall()
    if rec:
        # retry 는 인식 실패, bypass 는 구역 포화(정상 흐름). 합쳐 보지 말 것.
        out.append("\n【 재순환 】 retry=인식실패 · bypass=구역포화(실패 아님)")
        out.append(table(rec, ["box_id", "retry", "bypass"]))

    out.append("\n【 박스별 】")
    out.append(table(db.execute(
        "SELECT box_id, status, size_w_mm||'x'||size_d_mm||'x'||size_h_mm, "
        "place_x_mm, place_y_mm, place_z_mm, ROUND(placed_at-picked_at,1) "
        "FROM boxes_sorted ORDER BY db_seq").fetchall(),
        ["box_id", "status", "치수", "px", "py", "pz", "사이클s"]))
    out.append("")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="종료 지표 리포트 (트랙 D · D3)")
    ap.add_argument("--db-path", default=None)
    ap.add_argument("--csv", default=None, help="boxes_sorted 를 CSV 로 덤프")
    ap.add_argument("--watch", action="store_true", help="2 초마다 다시 그린다")
    a = ap.parse_args()

    cfg = load()
    zone = str(cfg["boxes"]["stage1"]["zone"]).upper()
    db_path = os.path.expanduser(a.db_path) if a.db_path else \
        os.path.join(os.path.dirname(_HERE), "palletris.db")
    if not os.path.isfile(db_path):
        sys.exit(f"DB 가 없다: {db_path}\n  먼저: python3 scripts/gen_master_db.py")

    db = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        while True:
            text = render(db, cfg, zone)
            if a.watch:
                print("\033[2J\033[H" + text)   # 화면 지우고 맨 위로
                time.sleep(2.0)
            else:
                print(text)
                break
    except KeyboardInterrupt:
        pass

    if a.csv:
        cur = db.execute("SELECT * FROM boxes_sorted ORDER BY db_seq")
        with open(a.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([d[0] for d in cur.description])
            w.writerows(cur.fetchall())
        print(f"CSV 저장 → {a.csv}\n")


if __name__ == "__main__":
    main()
