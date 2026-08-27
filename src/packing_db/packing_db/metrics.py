"""지표 계산 — 작업명세서 §8.6. ROS 의존성 0.

★ 왜 따로 빼나
  같은 숫자를 두 곳이 낸다 — 실행 중에는 db_service_node 가 /metrics/summary 로,
  끝난 뒤에는 scripts/report.py 가 표로. 쿼리를 양쪽에 복붙하면 한쪽만 고쳐져서
  "실시간 화면과 최종 보고서의 숫자가 다르다" 가 된다. 발표 자리에서 그러면 곤란하다.

★ 분모가 0 이면 0.0 이 아니라 -1.0 을 돌려준다
  "성공률 0 %" 와 "아직 잴 수 없음" 은 다르다. 0.0 으로 뭉개면 시작 직후 화면에
  성공률 0 % 가 크게 찍혀서 없는 문제를 쫓게 된다.

★★ 2026-08-27 — **실행 구간 한정**과 **성공률 분모 정정**

  실측으로 세 지표가 거짓말하고 있었다 (`palletris.db` 실행 31번 누적 상태):

      인식률       3.2 %        (실제로는 100 % 에 가깝다)
      배치 성공률  100.0 %      (PICK_FAILED 가 377 건인데도)
      유휴율       100.0 %      (경과가 17.9 억 초 = 약 56 년으로 잡혀서)

  원인 둘. 둘 다 "누적된 DB" 와 "현재값 한 칸" 이라는 같은 뿌리다.

  ① **실행이 갈린 자리를 아무도 안 표시한다.** `box_events`·`scan_misses` 에
     `run_id` 가 없고 자동 청소도 안 한다 (`db_service_node._warn_leftover_run` 은
     경고만 한다 — 남의 기록을 말없이 지우면 안 되기 때문이다). 그래서 지난 실행의
     행이 분모에 그대로 들어간다. 게다가 `_now()` 가 `/clock` 없으면 벽시계로
     떨어지므로 `ts` 에 sim 시각과 벽시계가 **같이** 들어 있다 — 실측 5,839 : 2,357 행.
     `MAX(ts) - MIN(ts)` 가 56 년이 되는 이유다.

     -> `_run_start()` 로 **시각이 뒤로 점프한 마지막 지점**을 찾아 거기서부터만 센다.
        시계가 섞여 있어도 "뒤로 갔다" 는 판정에는 영향이 없다. `--reset` 을 했으면
        점프가 없어 전체가 한 구간이 되므로 **예전 동작과 같다.**

  ② **성공률 분모가 이력이 아니라 현재 상태 한 칸이었다.** `boxes_sorted.status` 는
     재시도·거부로 즉시 덮어써진다 (`orchestrator` 가 픽 실패 후 재인식 없이 다시
     집는다). 실측: `box_events` 에 `PICK_FAILED` 377 건 · 박스 77 개인데
     `boxes_sorted` 에 남은 행은 **0 개** -> 72/(72+0) = 100 %.

     -> **시도 횟수는 `box_events` 에서 센다.** "몇 번 시도해 몇 번 성공했나" 는
        본디 이력의 질문이지 현재 상태의 질문이 아니다.
        "지금 팔레트에 몇 개 있나"(`placed_count`)는 그대로 현재 상태에서 센다 —
        그건 반대로 이력이 아니라 현재의 질문이다.

  ⚠ 남은 비대칭 하나: `scan_misses` 에 `box_id` 컬럼이 없어(`schema.sql`) 인식률
    분자는 박스 단위인데 분모의 miss 는 **건수 단위**다. 구간 한정으로 크게 완화되지만
    원리적으로는 아직 어긋난다. 컬럼 추가는 스키마 변경이라 팀 확인 사항으로 남긴다.
"""

UNKNOWN = -1.0

#: 시각이 이만큼(초) 뒤로 가면 **실행이 갈렸다**고 본다.
#: 한 실행 안에서는 시각이 단조 증가한다. 60 초는 sim/벽시계 어느 쪽으로 읽어도
#: 정상 진행과 헷갈릴 수 없는 폭이다.
RUN_GAP_S = 60.0

#: 실행 하나로 인정할 최소 이벤트 수. 이보다 짧은 구간은 **시계 뒤섞임**으로 본다
#: (노드 중복 실행 시 `ts` 가 두 계열로 들어와 가짜 경계가 생긴다 — `_run_start` 참조).
#: 박스 하나만 흘려도 SCANNED·VERIFIED·PICKED·PLACED 로 4 행이라 30 은 넉넉히 낮다.
RUN_MIN_EVENTS = 30

#: 이 값 이상이면 **벽시계**(1970 년부터의 초), 미만이면 **sim 시각**으로 본다.
#: `db_service_node._now()` 가 `/clock` 없을 때 벽시계로 떨어져 한 테이블에 두 계열이
#: 섞인다. 1e8 초는 sim 으로 치면 3 년이 넘어 절대 안 나오고, 벽시계로는 1973 년이라
#: 항상 넘는다 — 자릿수가 달라 확실히 갈린다.
EPOCH_MIN_S = 1e8


def _one(db, sql, args=()):
    row = db.execute(sql, args).fetchone()
    return row[0] if row else None


def _run_start(db, table):
    """`table` 에서 **현재 실행이 시작된 id**. 없으면 0 (= 전부 센다).

    id 순으로 훑으며 `ts` 가 `RUN_GAP_S` 이상 뒤로 점프한 지점을 모으고,
    그중 **뒤가 충분히 긴** 마지막 것을 고른다.

    ★ 왜 `ts` 최댓값 기준이 아니라 점프 기준인가: 벽시계 실행(1.78e9)과 sim 실행(17.6)이
      섞여 있으면 "큰 ts" 가 최신이라는 보장이 없다. 삽입 순서(id)만이 믿을 수 있다.

    ★★ 왜 "충분히 긴" 조건이 필요한가 (2026-08-27 실측):
      단순히 **마지막** 점프를 쓰면 노드가 중복 실행될 때 무너진다. 관제탑이 두 벌
      떠 있으면 시계가 둘이라 `ts` 가 두 계열로 뒤섞여 들어오고, 끝부분에서
      `1075 -> 50` 다음에 `306 -> 70` 같은 가짜 점프가 생긴다. 그 마지막 점프를
      실행 경계로 삼으면 **구간이 2 행 · 20 초**가 되어, 실제로 47 개를 적재했는데도
      화면이 "적재 0 · 인식률 0 %" 를 띄웠다.

      실행 하나는 이벤트가 수백 행이다. 뒤가 `RUN_MIN_EVENTS` 보다 짧은 경계는
      시계 뒤섞임으로 보고 그 앞 경계를 쓴다. 실측: 이 조건 하나로 같은 DB 가
      `2 행 / 적재 0` -> `403 행 / 적재 47` 로 바로잡혔다.

    ★ `--reset` 을 한 깨끗한 DB 에서는 점프가 없어 0 을 돌려준다 -> 예전과 같은 동작.
    """
    try:
        rows = db.execute(f"SELECT id, ts FROM {table} ORDER BY id").fetchall()
    except Exception:                       # noqa: BLE001 — 테이블이 없을 수도 있다
        return 0
    marks, prev = [0], None                 # 0 = "전부" (경계가 없을 때의 답)
    for rid, ts in rows:
        if ts is None:
            continue
        if prev is not None and ts < prev - RUN_GAP_S:
            marks.append(rid)
        prev = ts
    for b in reversed(marks):
        if sum(1 for r in rows if r[0] >= b) >= RUN_MIN_EVENTS:
            return b
    return 0


def collect(db, zone=None):
    """palletris.db 에서 지표를 한 번에 뽑는다. dict 로 돌려준다.

    ★ 전부 **현재 실행 구간**만 센다 (`_run_start` 참조).
    """
    z = (zone or "").upper()

    ev0 = _run_start(db, "box_events")      # 이번 실행의 첫 box_events.id
    sm0 = _run_start(db, "scan_misses")     # scan_misses 는 별도 테이블이라 따로 잡는다

    # ── 인식률 ────────────────────────────────────────────────────
    # ★ 분모 = **라인에 실제로 올라온 박스 수** = 등록 성공 + cam1 인식 실패.
    #   (2026-08-25 확정. boxes_sorted 를 쓰면 놓친 박스가 안 세어져 언제나 100 %,
    #    boxes_master 풀을 쓰면 풀이 클수록 낮아져 거꾸로다.)
    #   cam1 은 QR 을 못 읽어도 "덩어리가 지나갔다" 는 것은 보므로 이 값을 안다
    #   (/db/report_miss 로 들어온다).
    # ★ 2026-08-27: 등록 성공을 `COUNT(*) FROM boxes_sorted`(누적 전체) 가 아니라
    #   **이번 실행에 SCANNED 된 박스**로 센다. 그 둘은 깨끗한 DB 에서 같은 값이고,
    #   더러운 DB 에서만 갈린다.
    pool = _one(db, "SELECT COUNT(*) FROM boxes_master") or 0
    scanned = _one(db, "SELECT COUNT(DISTINCT box_id) FROM box_events "
                       "WHERE status='SCANNED' AND id>=?", (ev0,)) or 0
    miss1 = _one(db, "SELECT COUNT(*) FROM scan_misses "
                     "WHERE where_at='cam1' AND id>=?", (sm0,)) or 0
    total = scanned + miss1             # 라인에 올라온 박스 수

    # ── 이번 실행에 손댄 박스들 ───────────────────────────────────
    # 현재 상태(`boxes_sorted`)를 셀 때 이 집합으로 한정한다. 지난 실행에서 놓인 채
    # 남아 있는 행이 "지금 팔레트에 있다" 로 세어지는 것을 막는다.
    RUN_BOXES = "SELECT DISTINCT box_id FROM box_events WHERE id>=?"

    placed = _one(db, f"SELECT COUNT(*) FROM boxes_sorted "
                      f"WHERE status='PLACED' AND box_id IN ({RUN_BOXES})",
                  (ev0,)) or 0
    rejected = _one(db, f"SELECT COUNT(*) FROM boxes_sorted "
                        f"WHERE status='REJECTED' AND box_id IN ({RUN_BOXES})",
                    (ev0,)) or 0

    # ── 시도 횟수는 **이력**에서 센다 (2026-08-27 정정) ───────────
    # ★ `boxes_sorted.status` 로 세면 안 된다. 픽 실패 후 재시도해서 성공하면 그 칸이
    #   PLACED 로 덮여 실패한 적 없는 것이 된다 — 실측 377 건이 0 으로 사라졌다.
    place_ok = _one(db, "SELECT COUNT(*) FROM box_events "
                        "WHERE status='PLACED' AND id>=?", (ev0,)) or 0
    place_ng = _one(db, "SELECT COUNT(*) FROM box_events "
                        "WHERE status='PICK_FAILED' AND id>=?", (ev0,)) or 0
    attempts = place_ok + place_ng

    queue = (_one(db, f"SELECT COUNT(*) FROM boxes_sorted WHERE zone=? "
                      f"AND status IN ('DIVERTED','PICKED') "
                      f"AND box_id IN ({RUN_BOXES})", (z, ev0)) or 0) if z else 0

    # ── 실행 시간 = 이번 구간의 첫 이벤트부터 마지막 이벤트까지 ──
    # ★★ 한 구간 안에도 시계가 섞일 수 있다 (2026-08-27 실측).
    #   `_now()` 가 `/clock` 없으면 벽시계로 떨어지므로 같은 구간에 sim 시각(10)과
    #   벽시계(1,787,768,290)가 같이 들어온다. 그냥 MIN/MAX 를 내면
    #   **"10~1787768290초 (29,796,138분)"** 같은 값이 화면에 찍힌다 — 실제로 찍혔다.
    #   두 계열은 자릿수가 완전히 달라 확실히 가른다 (`EPOCH_MIN_S` 주석 참조).
    #   -> **행이 많은 쪽만** 쓴다. 소수 쪽은 시계가 잠깐 튄 것으로 본다.
    wall = _one(db, "SELECT COUNT(*) FROM box_events WHERE id>=? AND ts>=?",
                (ev0, EPOCH_MIN_S)) or 0
    sim = _one(db, "SELECT COUNT(*) FROM box_events WHERE id>=? AND ts<? AND ts IS NOT NULL",
               (ev0, EPOCH_MIN_S)) or 0
    clock_op, clock_ref = (">=", EPOCH_MIN_S) if wall > sim else ("<", EPOCH_MIN_S)
    t0 = _one(db, f"SELECT MIN(ts) FROM box_events WHERE id>=? AND ts{clock_op}?",
              (ev0, clock_ref))
    t1 = _one(db, f"SELECT MAX(ts) FROM box_events WHERE id>=? AND ts{clock_op}?",
              (ev0, clock_ref))
    elapsed = float(t1 - t0) if (t0 is not None and t1 is not None) else 0.0

    # ── 사이클 · 로봇이 실제로 일한 시간 ─────────────────────────
    # PICKED → PLACED 구간. 이번 실행 박스로 한정한다.
    # ★ 사이클도 같은 계열끼리만 뺀다. 한 박스의 picked_at 과 placed_at 이 서로 다른
    #   시계면 차이가 억 단위가 되어 평균을 통째로 망친다.
    CYCLE_WHERE = (f"FROM boxes_sorted WHERE status='PLACED' "
                   f"AND picked_at IS NOT NULL AND placed_at IS NOT NULL "
                   f"AND (picked_at>={EPOCH_MIN_S}) = (placed_at>={EPOCH_MIN_S}) "
                   f"AND box_id IN ({RUN_BOXES})")
    cycle = _one(db, f"SELECT AVG(placed_at - picked_at) {CYCLE_WHERE}", (ev0,))
    busy = _one(db, f"SELECT SUM(placed_at - picked_at) {CYCLE_WHERE}", (ev0,)) or 0.0

    return {
        "total_boxes": int(total),      # 라인에 올라온 수 (인식률의 분모)
        "pool_size": int(pool),         # 정답지 풀 크기 — 분모가 아니다
        "scan_miss": int(miss1),
        "scanned_count": int(scanned),
        "placed_count": int(placed),        # 지금 팔레트에 있는 수 (현재 상태)
        "pick_failed_count": int(place_ng),  # ★ 시도 실패 **횟수** (이력)
        "rejected_count": int(rejected),
        "queue_len": int(queue),
        "elapsed_s": elapsed,
        "recognition_rate": (scanned / total) if total else UNKNOWN,
        # ★ 분모가 이력이다. 재시도로 성공해도 실패 1 회는 실패로 남는다.
        "place_success_rate": (place_ok / attempts) if attempts else UNKNOWN,
        "cycle_avg_s": float(cycle) if cycle is not None else UNKNOWN,
        # ★ 유휴율은 "안 움직인 비율" 이다. 실행 시간이 0 이면 잴 수 없다.
        "robot_idle_rate": (1.0 - busy / elapsed) if elapsed > 0 else UNKNOWN,
        # ── 진단용 (화면에 "언제부터의 값인가" 를 띄우기 위한 것) ──
        # ★ 오염된 DB 를 조용히 넘어가지 않게 하는 장치다. 화면이 이걸 보여 주면
        #   "지금 몇 개째 실행의 숫자인가" 가 눈에 보인다.
        "run_start_id": int(ev0),
        "run_t0": float(t0) if t0 is not None else UNKNOWN,
        "run_t1": float(t1) if t1 is not None else UNKNOWN,
        "place_attempts": int(attempts),
    }


def fmt_rate(v, pct=True):
    """-1.0 은 숫자가 아니라 '—' 로 보여 준다."""
    if v is None or v <= UNKNOWN:
        return "—"
    return f"{v * 100:.1f}%" if pct else f"{v:.1f}"
