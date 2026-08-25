"""지표 계산 — 작업명세서 §8.6. ROS 의존성 0.

★ 왜 따로 빼나
  같은 숫자를 두 곳이 낸다 — 실행 중에는 db_service_node 가 /metrics/summary 로,
  끝난 뒤에는 scripts/report.py 가 표로. 쿼리를 양쪽에 복붙하면 한쪽만 고쳐져서
  "실시간 화면과 최종 보고서의 숫자가 다르다" 가 된다. 발표 자리에서 그러면 곤란하다.

★ 분모가 0 이면 0.0 이 아니라 -1.0 을 돌려준다
  "성공률 0 %" 와 "아직 잴 수 없음" 은 다르다. 0.0 으로 뭉개면 시작 직후 화면에
  성공률 0 % 가 크게 찍혀서 없는 문제를 쫓게 된다.
"""

UNKNOWN = -1.0


def _one(db, sql, args=()):
    row = db.execute(sql, args).fetchone()
    return row[0] if row else None


def collect(db, zone=None):
    """palletris.db 에서 지표를 한 번에 뽑는다. dict 로 돌려준다."""
    z = (zone or "").upper()

    # ★ 인식률의 분모 — 2026-08-25 바로잡음.
    #   boxes_sorted(등록된 박스)를 분모로 쓰면 놓친 박스는 안 세어져 **언제나 100 %** 다.
    #   boxes_master(정답지 풀)를 쓰면 풀을 넉넉히 깔수록 낮게 나온다 — 거꾸로다.
    #   실측: 풀 105 개에 15 개만 흘렸더니 15/105 = 14 % 였다. 실제로는 15/15 = 100 % 다.
    #   맞는 분모는 **라인에 실제로 올라온 박스 수** = 등록 성공 + cam1 인식 실패.
    #   cam1 은 QR 을 못 읽어도 "빨간 덩어리가 지나갔다"는 것은 보므로 이 값을 안다
    #   (/db/report_miss 로 들어온다).
    pool = _one(db, "SELECT COUNT(*) FROM boxes_master") or 0
    registered = _one(db, "SELECT COUNT(*) FROM boxes_sorted") or 0
    miss1 = _one(db, "SELECT COUNT(*) FROM scan_misses WHERE where_at='cam1'") or 0
    total = registered + miss1          # 라인에 올라온 박스 수
    scanned = _one(db, "SELECT COUNT(DISTINCT box_id) FROM box_events "
                       "WHERE status='SCANNED'") or 0
    placed = _one(db, "SELECT COUNT(*) FROM boxes_sorted WHERE status='PLACED'") or 0
    failed = _one(db, "SELECT COUNT(*) FROM boxes_sorted "
                      "WHERE status='PICK_FAILED'") or 0
    rejected = _one(db, "SELECT COUNT(*) FROM boxes_sorted "
                        "WHERE status='REJECTED'") or 0

    queue = _one(db, "SELECT COUNT(*) FROM boxes_sorted WHERE zone=? AND status IN "
                     "('DIVERTED','PICKED')", (z,)) or 0 if z else 0

    # 실행 시간 = 첫 이벤트부터 마지막 이벤트까지.
    t0 = _one(db, "SELECT MIN(ts) FROM box_events")
    t1 = _one(db, "SELECT MAX(ts) FROM box_events")
    elapsed = float(t1 - t0) if (t0 is not None and t1 is not None) else 0.0

    cycle = _one(db, "SELECT AVG(placed_at - picked_at) FROM boxes_sorted "
                     "WHERE status='PLACED' AND picked_at IS NOT NULL "
                     "AND placed_at IS NOT NULL")

    # 로봇이 실제로 일한 시간 = PICKED → PLACED 구간의 합.
    busy = _one(db, "SELECT SUM(placed_at - picked_at) FROM boxes_sorted "
                    "WHERE status='PLACED' AND picked_at IS NOT NULL "
                    "AND placed_at IS NOT NULL") or 0.0

    return {
        "total_boxes": int(total),      # 라인에 올라온 수 (인식률의 분모)
        "pool_size": int(pool),         # 정답지 풀 크기 — 분모가 아니다
        "scan_miss": int(miss1),
        "scanned_count": int(scanned),
        "placed_count": int(placed),
        "pick_failed_count": int(failed),
        "rejected_count": int(rejected),
        "queue_len": int(queue),
        "elapsed_s": elapsed,
        "recognition_rate": (scanned / total) if total else UNKNOWN,
        "place_success_rate": (placed / (placed + failed)) if (placed + failed)
                              else UNKNOWN,
        "cycle_avg_s": float(cycle) if cycle is not None else UNKNOWN,
        # ★ 유휴율은 "안 움직인 비율" 이다. 실행 시간이 0 이면 잴 수 없다.
        "robot_idle_rate": (1.0 - busy / elapsed) if elapsed > 0 else UNKNOWN,
    }


def fmt_rate(v, pct=True):
    """-1.0 은 숫자가 아니라 '—' 로 보여 준다."""
    if v is None or v <= UNKNOWN:
        return "—"
    return f"{v * 100:.1f}%" if pct else f"{v:.1f}"
