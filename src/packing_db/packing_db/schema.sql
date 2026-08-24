-- palletris.db 스키마 — 작업명세서 §8.2 기준.
--
-- ★ 명세서 §8.2 에서 늘어난 부분과 그 이유:
--   boxes_sorted 에 color · dest · qr_payload 를 더했다. 새 BoxInfo (CLAUDE.md §3-1)가
--   이 셋을 싣는데 명세서 스키마에는 담을 칸이 없었다. RegisterBox 요청이 주는 값이라
--   저장하지 않으면 GetBoxInfo 가 빈 문자열밖에 못 돌려준다.
--   box_events 에 reason 을 더했다. SetStatus 가 reason 을 받는데 status 컬럼만으로는
--   "왜 PICK_FAILED 인지"가 사라진다 — 실패 분석이 통째로 불가능해진다.
--
-- ★ 컬럼 이름은 mass_g 그대로 둔다. 인터페이스 필드는 BoxInfo.weight_g 로 바뀌었지만
--   스키마는 외부 계약이 아니고, 명세서 §8.6 의 지표 쿼리들이 이 이름을 참조한다.
--   매핑은 db_service_node 한 곳에서만 한다.

PRAGMA journal_mode=WAL;

-- 실행 전에 미리 만들어 두는 "정답지". gen_master_db.py 가 채운다.
CREATE TABLE IF NOT EXISTS boxes_master (
    box_id       TEXT PRIMARY KEY,
    address      TEXT NOT NULL,
    delivery_seq INTEGER NOT NULL,
    zone         TEXT NOT NULL
);

-- 실행 중 실제로 흘러간 박스. 인식이 실패하면 여기 행이 안 생긴다.
CREATE TABLE IF NOT EXISTS boxes_sorted (
    db_seq        INTEGER PRIMARY KEY AUTOINCREMENT,
    box_id        TEXT NOT NULL UNIQUE,
    zone          TEXT,
    size_w_mm     INTEGER, size_d_mm INTEGER, size_h_mm INTEGER,
    mass_g        INTEGER,                     -- → BoxInfo.weight_g
    color         TEXT,                        -- cam1 판독값
    dest          TEXT,                        -- QR 목적지 "A"|"B"|"C"
    qr_payload    TEXT,                        -- 등록 시점의 QR 원문
    status        TEXT NOT NULL,
      -- SPAWNED|SCANNED|VERIFIED|DIVERTED|PICKED|PLACED|PICK_FAILED|REJECTED
    place_x_mm    INTEGER, place_y_mm INTEGER, place_z_mm INTEGER,
    place_yaw_deg REAL,
    spawned_at REAL, scanned_at REAL, picked_at REAL, placed_at REAL
);

-- ★ 상태 전이 로그. 인식률·성공률·사이클타임이 전부 여기서 나온다.
--   status 단일 컬럼은 덮어써져 이력이 남지 않는다 (명세서 §8.6).
CREATE TABLE IF NOT EXISTS box_events (
    id     INTEGER PRIMARY KEY AUTOINCREMENT,
    box_id TEXT NOT NULL,
    status TEXT NOT NULL,
    reason TEXT,
    ts     REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_box_events_box ON box_events(box_id);

-- Return line 재순환. ★ retry 와 bypass 를 절대 같은 칸에 넣지 말 것 —
--   버퍼 우회는 정상 흐름이지 실패가 아니다. 합치면 멀쩡한 박스가 3바퀴 돌았다는
--   이유로 폐기된다 (명세서 §8.2 경고).
CREATE TABLE IF NOT EXISTS recirculation (
    box_id       TEXT PRIMARY KEY,
    retry_count  INTEGER NOT NULL DEFAULT 0,   -- 인식 실패. max_retry 초과 시 REJECTED
    bypass_count INTEGER NOT NULL DEFAULT 0,   -- 버퍼 우회. 실패 아님
    last_seen    REAL
);
