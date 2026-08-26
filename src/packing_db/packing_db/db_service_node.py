#!/usr/bin/env python3
"""db_service_node — palletris.db 의 유일한 writer + 중재자.

제공하는 것
  /db/register_box · /db/get_box_info · /db/verify_box
  /db/set_status   · /db/mark_placed  · /db/zone_queue      (CLAUDE.md §3-3)
  /diverter/request                                          (중재자)

★ 왜 중재자가 여기 같이 있나 (작업명세서 §8.5)
  중재자는 0.6 초 안에 답해야 한다 (CLAUDE.md §3-4 규약 4). 별 프로세스로 빼면
  판단에 필요한 retry_count·dest 를 /db/* 서비스로 왕복해서 가져와야 하고,
  그 왕복이 0.6 초를 잡아먹는다. 같은 프로세스에서 커넥션을 공유하면 조회가
  마이크로초다.

★ 왜 이 노드만 .db 를 여나 (CLAUDE.md §10)
  두 프로세스가 같은 파일을 열면 `database is locked` 가 뜬다. WAL 을 켜도
  writer 는 하나여야 한다.

★ sqlite3 와 MultiThreadedExecutor
  기본 커넥션은 만든 스레드 밖에서 쓰면 ProgrammingError 가 난다.
  check_same_thread=False 로 풀고, 쓰기는 Lock 으로 직렬화한다.

──────────────────────────────────────────────────────────────────────
⚠ 명세서와 달라진 점 — 인터페이스 개편(CLAUDE.md §3)의 결과다

1. SpawnEvent 가 없어졌다.
   명세서 §8.6 은 `/sim/spawn_event` 를 받아 SPAWNED 행을 먼저 만들라고 한다.
   그래야 "인식 실패한 박스"도 분모에 남는다. 그런데 개편에서 SpawnEvent.msg 가
   삭제됐고 CLAUDE.md §6 흐름은 DB 가 RegisterBox 에서 처음 박스를 안다.
   → 인식률 분모를 boxes_master 로 옮겼다. master 는 실행 전에 전량 깔리므로
     "스폰됐는데 등록 안 된 박스"를 그대로 셀 수 있다. 지표는 유지된다.

2. RegisterBox 가 질량을 안 준다.
   새 요청은 box_id·치수·color·dest·qr_payload 뿐이다. 그런데 BoxInfo.weight_g 는
   있어야 한다. → 치수로 system.yaml 의 박스 3종을 역인덱스해서 채운다.
   3종의 (w,d,h) 가 서로 겹치지 않아 유일하게 결정된다. 숫자를 코드에 박지 않는다.

3. zone 을 dest 에서 받는다.
   RegisterBox 에 zone 칸이 없다. QR 이 읽은 dest 가 곧 목적 구역이다.
   master.zone 과 다르면 "인식이 틀린 것"이므로 경고를 남기고 계속한다 —
   여기서 죽으면 박스 하나 때문에 라인 전체가 선다.
"""
import argparse
import os
import sqlite3
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from packing_common.config import load
from packing_common.paths import default_db_path, find_data
from packing_db.gen_master_db import dong_of
from packing_db.master_data import run_table_counts
from packing_db.metrics import collect, fmt_rate
from packing_msgs.msg import BoxInfo, MetricsSummary, ZoneStatus
from packing_msgs.srv import (DiverterDecide, DiverterRequest, GetBoxInfo,
                              MarkPlaced, RegisterBox, ReportMiss, SetStatus,
                              VerifyBox, ZoneQueue)

# CLAUDE.md §3-1 BoxInfo.status 가 정의한 8 개. 이 밖의 값은 받지 않는다.
VALID_STATUS = {"SPAWNED", "SCANNED", "VERIFIED", "DIVERTED",
                "PICKED", "PLACED", "PICK_FAILED", "REJECTED"}

# status → boxes_sorted 의 시각 컬럼. 없는 것은 box_events 에만 남는다.
TS_COLUMN = {"SPAWNED": "spawned_at", "SCANNED": "scanned_at",
             "PICKED": "picked_at", "PLACED": "placed_at"}


# ★ default_db_path 는 packing_common.paths 로 옮겼다 (2026-08-26).
#   여기와 gen_master_db 에 "config 경로에서 4 단계 위" 산술이 **두 벌** 있었는데,
#   find_config() 가 share 를 돌려주는 순간 둘 다 엉뚱한 곳을 가리킨다. DB 는 없으면
#   그냥 새로 만들어지므로 빈 DB 를 조용히 쓰게 된다 — "master 에 없는 박스" 증상이다.
#   이제 워크스페이스 표식으로 찾고, PALLETRIS_DB 로 덮어쓸 수 있다.


class DbService(Node):

    def __init__(self, db_path=None, config_path=None):
        super().__init__("db_service_node")
        self.cfg = load(config_path)
        self.db_path = os.path.expanduser(db_path) if db_path else default_db_path()

        # ── 치수 → 질량 역인덱스 (개편으로 RegisterBox 가 질량을 안 주게 됐다) ──
        self._mass_by_size = {
            (int(t["w_mm"]), int(t["d_mm"]), int(t["h_mm"])): int(t["mass_g"])
            for t in self.cfg["boxes"]["types"]}

        self._max_retry = int(self.cfg["recognition"]["max_retry"])

        # ── DB ──
        # ★★ 커넥션 접근 **전부** 직렬화 (2026-08-27, 김병직 — 세그폴트 실측).
        #   원래는 "쓰기만 Lock" 이었는데 읽기(SELECT)가 락 밖에서 같은 커넥션을
        #   동시에 썼다. 평소엔 경합이 드물어 버텼지만, cam1 SIZE_MISMATCH 홍수
        #   (report_miss 1200건+/10분)가 읽기·쓰기 동시 접근을 상시화하자
        #   프로세스가 exit -11 (네이티브 세그폴트) 로 죽었다 — DB 가 죽으면
        #   관제탑 3존이 전부 "서비스 없음" 으로 서고 라인이 통째로 멈춘다.
        #   커넥션을 RLock 프록시로 감싸 execute/commit 이 항상 락 안에서 돈다.
        #   기존 `with self._w:` 블록은 RLock 이라 그대로 둬도 재진입만 된다.
        _conn = sqlite3.connect(self.db_path, check_same_thread=False)
        _lk = threading.RLock()

        class _Rows:
            """락 **안에서** 미리 읽어 둔 결과 (2026-08-27 2차 수정).

            1차 수정은 execute 만 락으로 감쌌는데, 커서를 돌려주고 락을 푼 뒤
            호출부가 fetchone() 을 불렀다 — 그 틈에 다른 스레드의 commit() 이
            커서를 리셋해 COUNT(*) 의 fetchone 이 None 이 됐다 (int(None)
            TypeError 로 프로세스 사망, 실측). execute 시점에 결과를 다 읽어
            평범한 리스트로 들고 있으면 커서 수명 문제가 사라진다.
            이 파일의 사용처는 fetchone()/rowcount 뿐이다 (전수 확인)."""
            def __init__(self, rows, rowcount, lastrowid):
                self._rows, self.rowcount, self.lastrowid = rows, rowcount, lastrowid
                self._i = 0

            def fetchone(self):
                if self._i < len(self._rows):
                    self._i += 1
                    return self._rows[self._i - 1]
                return None

            def fetchall(self):
                out, self._i = self._rows[self._i:], len(self._rows)
                return out

        class _LockedConn:
            """sqlite3 Connection 의 스레드 직렬화 프록시 (위 주석 참조)."""
            def __init__(self, conn, lock):
                self._c, self._lk = conn, lock

            def execute(self, *a, **k):
                with self._lk:
                    cur = self._c.execute(*a, **k)
                    rows = cur.fetchall() if cur.description is not None else []
                    return _Rows(rows, cur.rowcount, cur.lastrowid)

            def executescript(self, *a, **k):
                with self._lk:
                    return self._c.executescript(*a, **k)

            def commit(self):
                with self._lk:
                    return self._c.commit()

            def __getattr__(self, n):       # close 등 나머지는 그대로 위임
                return getattr(self._c, n)

        self.db = _LockedConn(_conn, _lk)
        self.db.execute("PRAGMA journal_mode=WAL")
        self._w = _lk                       # 쓰기 직렬화 (커넥션 락과 같은 것)
        self._ensure_schema()

        n = self.db.execute("SELECT COUNT(*) FROM boxes_master").fetchone()[0]
        if n == 0:
            raise SystemExit(
                f"boxes_master 가 비어 있다 — gen_master_db 를 먼저 실행할 것.\n"
                f"  ros2 run packing_db gen_master_db\n  DB: {self.db_path}")

        cb = ReentrantCallbackGroup()
        for srv_type, name, fn in (
                (RegisterBox, "/db/register_box", self.on_register),
                (GetBoxInfo,  "/db/get_box_info", self.on_get_info),
                (VerifyBox,   "/db/verify_box",   self.on_verify),
                (SetStatus,   "/db/set_status",   self.on_set_status),
                (MarkPlaced,  "/db/mark_placed",  self.on_mark_placed),
                (ZoneQueue,   "/db/zone_queue",   self.on_zone_queue),
                (ReportMiss,  "/db/report_miss", self.on_report_miss),
                (DiverterRequest, "/diverter/request", self.on_diverter),
                (DiverterDecide,  "/diverter/decide",  self.on_diverter_decide)):
            self.create_service(srv_type, name, fn, callback_group=cb)

        # ── 구역 수용 여부 캐시 ──
        # ★ 중재자는 ZoneStatus 의 accepting 한 칸만 본다 (CLAUDE.md §3-2).
        #   버퍼 잔량·예상 여유시간을 직접 계산하던 옛 방식은 관제탑과 계산이
        #   어긋나면 조용히 잘못 보낸다. 판단 주체를 관제탑 하나로 몰아 둔다.
        self._accepting = {}                     # zone(대문자) → bool
        # ★ 분기기별 질의용 판단 캐시 (DiverterDecide.srv 주석 참조).
        #   한 바퀴에 한 번만 판단하기 위한 것이다.
        self._dv_cache = {}                      # box_id → (target, reason)
        self._n_diverters = len(self.cfg["zones"]["list"])
        # ★ fill_rate·max_height_mm 은 DB 가 모른다 — heightmap 을 가진 algo 만 안다.
        #   유일한 유입 경로가 관제탑의 ZoneStatus 다. 여기서 받아 지표에 얹는다.
        self._zone_cache = {}                    # zone → (fill_rate, max_height_mm)
        self._metrics_zone = str(self.cfg["boxes"]["stage1"]["zone"]).upper()
        for z in self.cfg["zones"]["list"]:
            self.create_subscription(
                ZoneStatus, f"/zone_{z.lower()}/status",
                self._on_zone_status, 10, callback_group=cb)

        # ★ /metrics/summary — 전용 타입이다. JSON 문자열이 아니다 (CLAUDE.md §11).
        self.pub_metrics = self.create_publisher(MetricsSummary, "/metrics/summary", 10)
        self.create_timer(1.0, self._publish_metrics, callback_group=cb)
        self._log_every, self._log_n = 10, 0
        self.get_logger().info(
            f"db_service_node 준비 — {self.db_path} · master {n} 행 · "
            f"max_retry {self._max_retry}")
        self._warn_leftover_run()

    def _warn_leftover_run(self):
        """이전 실행 기록이 남아 있으면 알린다. **지우지는 않는다.**

        ★ 왜 경고만 하나
          파일을 소유한 프로세스가 남의 기록을 말없이 지우면 안 된다. 지우는 것은
          사람이 `gen_master_db --reset` 으로 시킬 일이다.

        ★ 왜 알려야 하나 (2026-08-26 실측)
          metrics 의 elapsed_s 는 `MAX(ts) - MIN(ts)` 다. 어제 실행의 box_events 가
          남아 있으면 elapsed 가 며칠이 되고, robot_idle_rate 가 100 % 로 굳는다.
          실제로 그 상태였다 — box_events 시간 범위 17.8 억 초, 로봇 작업 0.1 초.
          지표가 조용히 쓰레기가 되는데 에러는 안 뜬다.
        """
        try:
            n = run_table_counts(self.db)
        except sqlite3.Error:
            return
        if not any(n.values()):
            return
        left = " · ".join(f"{k} {v} 행" for k, v in n.items() if v)
        self.get_logger().warn(
            f"이전 실행 기록이 남아 있다 — {left}. 지표가 섞인다 "
            f"(elapsed_s 가 MAX(ts)-MIN(ts) 라 어제 실행까지 포함된다). "
            f"새로 재려면: ros2 run packing_db gen_master_db --reset")

    # ══════════════════════════════════════════════════════════════
    # 공통
    # ══════════════════════════════════════════════════════════════
    def _ensure_schema(self):
        """schema.sql 을 적용한다. CREATE TABLE IF NOT EXISTS 라 여러 번 돌아도 안전하다.

        ★ 옛 코드는 `<here>/../share/packing_db/schema.sql` 을 봤다. 설치본에서
          share 는 모듈 기준 **4 단계 위**라 그 경로는 존재하지 않는다 —
          일반 colcon build 로는 여기서 죽었다. paths.find_data 가 그걸 안다.

        ★ 김학민님도 같은 날 같은 버그를 찾아 ament 인덱스로 고쳤다 (main ff5c5d0).
          여기서는 그 수정을 paths.find_data 로 흡수했다 — 같은 버그가
          gen_master_db · config.py · 정답지 CSV 에도 있어 한 곳으로 모아야 했다.
        """
        with open(find_data("packing_db", "schema.sql", __file__),
                  encoding="utf-8") as f, self._w:
            self.db.executescript(f.read())

    def _now(self):
        """타임스탬프. use_sim_time 이면 sim 시계를 따른다 (CLAUDE.md §3-4 규약 1).

        ⚠ use_sim_time 인데 /clock 발행자가 아직 없으면 ROS 시계가 0 에 멈춘다.
          그대로 쓰면 모든 사이클타임이 0 으로 찍혀 지표가 통째로 죽는다.
          0 이면 벽시계로 되돌린다 — 지표가 없는 것보다 낫다.
        """
        t = self.get_clock().now().nanoseconds / 1e9
        return t if t > 0.0 else time.time()

    def _set_status(self, box_id, status, reason=""):
        """상태 변경은 반드시 이 함수로. box_events 기록이 여기 묶여 있다.

        status 컬럼만 UPDATE 하면 덮어써져 이력이 사라지고, 그러면 인식률·성공률·
        사이클타임을 전부 못 잰다 (명세서 §8.6).
        """
        now = self._now()
        with self._w:
            self.db.execute("UPDATE boxes_sorted SET status=? WHERE box_id=?",
                            (status, box_id))
            self.db.execute(
                "INSERT INTO box_events(box_id, status, reason, ts) VALUES(?,?,?,?)",
                (box_id, status, reason or None, now))
            col = TS_COLUMN.get(status)
            if col:
                self.db.execute(
                    f"UPDATE boxes_sorted SET {col}=? WHERE box_id=?", (now, box_id))
            self.db.commit()

    def _master_zone(self, box_id):
        row = self.db.execute("SELECT zone FROM boxes_master WHERE box_id=?",
                              (box_id,)).fetchone()
        return row[0] if row else None

    def _zone_by_address(self, box_id):
        """운송장번호 -> 주소 -> 동 -> 담당 구역. 실제 택배 분류와 같은 순서다.

        ★ 구역 배정은 **배송지 기준**이다 (설계결정서 §5-3 확정).
          QR 에는 운송장번호만 있고 구역 글자는 없다 — 실제 송장 바코드가 그렇다.
          그래서 주소를 조회해 동을 뽑고, zone_map 이 담당 구역을 정한다.

        ★ 표에 없는 동이면 None 이다. 아무 구역에나 넣지 않는다 —
          모르는 주소를 넣으면 그 박스는 영영 잘못 배송된다. 중재자가 RETURN 시킨다.

        반환: (zone|None, address|None, dong|None) — 로그에 셋 다 필요하다
        """
        row = self.db.execute("SELECT address FROM boxes_master WHERE box_id=?",
                              (box_id,)).fetchone()
        if not row:
            return None, None, None
        addr = row[0]
        dong = dong_of(addr)
        if dong is None:
            return None, addr, None
        z = self.db.execute("SELECT zone FROM zone_map WHERE dong=?", (dong,)).fetchone()
        return (z[0] if z else None), addr, dong

    def _retry_count(self, box_id):
        row = self.db.execute("SELECT retry_count FROM recirculation WHERE box_id=?",
                              (box_id,)).fetchone()
        return int(row[0]) if row else 0

    def _bump(self, box_id, column):
        """recirculation 카운터 증가. retry 와 bypass 는 절대 합치지 않는다.

        버퍼 우회(bypass)는 정상 흐름이지 실패가 아니다. 합치면 멀쩡한 박스가
        세 바퀴 돌았다는 이유로 폐기된다 (명세서 §8.2 경고).
        """
        assert column in ("retry_count", "bypass_count")
        with self._w:
            self.db.execute(
                "INSERT INTO recirculation(box_id, last_seen) VALUES(?,?) "
                "ON CONFLICT(box_id) DO NOTHING", (box_id, self._now()))
            self.db.execute(
                f"UPDATE recirculation SET {column}={column}+1, last_seen=? "
                "WHERE box_id=?", (self._now(), box_id))
            self.db.commit()
        return self.db.execute(
            f"SELECT {column} FROM recirculation WHERE box_id=?", (box_id,)).fetchone()[0]

    def _on_zone_status(self, msg):
        z = msg.zone.upper()
        self._accepting[z] = bool(msg.accepting)
        self._zone_cache[z] = (float(msg.fill_rate), int(msg.max_height_mm))

    # ══════════════════════════════════════════════════════════════
    # /db/report_miss  — 인식 실패. **인식률의 분모**가 여기서 나온다
    # ══════════════════════════════════════════════════════════════
    def on_report_miss(self, req, res):
        at = (req.where_at or "").strip() or "unknown"
        with self._w:
            self.db.execute(
                "INSERT INTO scan_misses(where_at, reason, ts) VALUES(?,?,?)",
                (at, (req.reason or "").strip(), self._now()))
            self.db.commit()
        n = self.db.execute("SELECT COUNT(*) FROM scan_misses WHERE where_at=?",
                            (at,)).fetchone()[0]
        res.ok, res.miss_count = True, int(n)
        # ★ 경고로 남긴다. 이게 늘어나는데 조용하면 인식이 망가진 줄 모른다.
        self.get_logger().warn(f"인식 실패 [{at}] {req.reason} — 누계 {n}")
        return res

    # ══════════════════════════════════════════════════════════════
    # /db/register_box  — cam1 판독 결과. 여기서 boxes_sorted 행이 처음 생긴다
    # ══════════════════════════════════════════════════════════════
    def on_register(self, req, res):
        t0 = time.perf_counter()
        box_id = req.box_id.strip()

        # ★ 구독·서비스 콜백은 입력을 검증한다. rclpy 가 예외를 삼키므로
        #   잘못된 요청 하나에 라인이 서는데 에러는 안 뜬다 (CLAUDE.md §9).
        if not box_id:
            res.ok, res.message = False, "box_id 가 비었다"
            return res

        size = (int(req.size_w_mm), int(req.size_d_mm), int(req.size_h_mm))
        if min(size) <= 0:
            res.ok, res.message = False, f"치수가 잘못됐다 {size}"
            return res

        mass = self._mass_by_size.get(size)
        if mass is None:
            # 사양 3종이 아니다. 거부하지 않고 0 으로 남긴다 — 인식 오차로 치수가
            # 몇 mm 틀어졌을 뿐인데 박스를 버리면 라인이 막힌다.
            self.get_logger().warn(f"{box_id} 치수 {size} 가 사양 3종에 없다 — weight_g=0")
            mass = 0

        if self._master_zone(box_id) is None:
            # master 에 없는 id. 스포너와 운송장번호 규칙이 어긋난 것이다.
            res.ok, res.message = False, f"master 에 없는 박스: {box_id}"
            self.get_logger().error(res.message)
            return res

        # ★ 구역은 여기서 정해진다 — 주소의 동으로. cam1 이 보내온 req.dest 는
        #   쓰지 않는다. QR 에 구역 글자가 없기 때문이다 (설계결정서 §5-3).
        zone, addr, dong = self._zone_by_address(box_id)
        if zone is None:
            # 담당 구역이 없는 주소. 행은 만들되 zone 을 비워 둔다 —
            # 중재자가 zone 없는 박스를 RETURN 으로 돌린다.
            self.get_logger().warn(
                f"{box_id} 담당 구역 없음: 주소 {addr!r} · 동 {dong!r} "
                "— zone_map 에 없는 동이다")
        else:
            self.get_logger().info(
                f"{box_id} 주소 {addr!r} -> 동 {dong} -> 구역 {zone}")

        # req.dest 가 비어 있지 않으면 인식이 구역을 추측한 것이다. 기록만 한다.
        dest = (req.dest or "").strip().upper()
        if dest and dest != zone:
            self.get_logger().warn(
                f"{box_id} 인식이 보낸 dest {dest} 가 주소 기준 구역 {zone} 과 다르다")

        with self._w:
            self.db.execute(
                "INSERT INTO boxes_sorted(box_id, zone, size_w_mm, size_d_mm, size_h_mm,"
                " mass_g, color, dest, qr_payload, status) "
                "VALUES(?,?,?,?,?,?,?,?,?,'SCANNED') "
                "ON CONFLICT(box_id) DO UPDATE SET "
                "  zone=excluded.zone, size_w_mm=excluded.size_w_mm,"
                "  size_d_mm=excluded.size_d_mm, size_h_mm=excluded.size_h_mm,"
                "  mass_g=excluded.mass_g, color=excluded.color, dest=excluded.dest,"
                "  qr_payload=excluded.qr_payload,"
                # ★ 이전 바퀴의 흔적을 반드시 지운다 (2026-08-26).
                #   재등록은 "같은 박스가 라인을 다시 돈다" 는 뜻이다. 예전에는 status 만
                #   SCANNED 로 돌아가고 적재 좌표와 시각은 그대로 남았다 —
                #   실측: 재등록 뒤에도 place=(111,222,333) 이 살아 있었다.
                #   그러면 plan_route.py · pallet_access.py 가 **존재하지 않는 더미**를
                #   계산하고, cycle_avg_s 는 옛 placed_at-picked_at 을 계속 평균에 넣는다.
                "  status='SCANNED',"
                "  place_x_mm=NULL, place_y_mm=NULL, place_z_mm=NULL, place_yaw_deg=NULL,"
                "  scanned_at=NULL, picked_at=NULL, placed_at=NULL",
                (box_id, zone, size[0], size[1], size[2], mass,
                 req.color, dest, req.qr_payload))
            self.db.commit()
        self._set_status(box_id, "SCANNED")

        res.ok, res.message = True, (f"zone={zone} (동 {dong})" if zone
                                     else f"zone=없음 (동 {dong})")
        self.get_logger().debug(
            f"register_box {box_id} → {zone} ({(time.perf_counter()-t0)*1e3:.1f} ms)")
        return res

    # ══════════════════════════════════════════════════════════════
    # /db/get_box_info
    # ══════════════════════════════════════════════════════════════
    def on_get_info(self, req, res):
        row = self.db.execute(
            "SELECT size_w_mm, size_d_mm, size_h_mm, mass_g, color, dest,"
            " qr_payload, status FROM boxes_sorted WHERE box_id=?",
            (req.box_id,)).fetchone()
        res.found = row is not None
        if row:
            res.info = BoxInfo(
                box_id=req.box_id,
                size_w_mm=int(row[0] or 0), size_d_mm=int(row[1] or 0),
                size_h_mm=int(row[2] or 0),
                weight_g=int(row[3] or 0),          # ← 스키마는 mass_g. 여기서만 매핑한다
                color=row[4] or "", dest=row[5] or "", qr_payload=row[6] or "",
                retry_count=self._retry_count(req.box_id),
                status=row[7] or "")
        return res

    # ══════════════════════════════════════════════════════════════
    # /db/verify_box  — cam2 재판독. 등록값과 같은지 본다
    # ══════════════════════════════════════════════════════════════
    def on_verify(self, req, res):
        row = self.db.execute(
            "SELECT box_id FROM boxes_sorted WHERE qr_payload=? AND qr_payload<>''",
            (req.qr_payload,)).fetchone()
        res.registered_id = row[0] if row else ""
        res.match = bool(row) and row[0] == req.box_id

        if res.match:
            self._set_status(req.box_id, "VERIFIED")
        else:
            # 불일치는 실패로 센다. 어느 쪽이 틀렸는지는 box_events 로 되짚는다.
            self.get_logger().warn(
                f"verify 불일치: 요청 {req.box_id} · QR 주인 "
                f"{res.registered_id or '(등록 없음)'}")
        return res

    # ══════════════════════════════════════════════════════════════
    # /db/set_status  — 관제탑이 PICKED / PICK_FAILED 등을 올린다
    # ══════════════════════════════════════════════════════════════
    def on_set_status(self, req, res):
        status = (req.status or "").strip().upper()
        if status not in VALID_STATUS:
            self.get_logger().error(f"모르는 status: {req.status!r}")
            res.ok = False
            return res
        if self._master_zone(req.box_id) is None:
            self.get_logger().error(f"master 에 없는 박스: {req.box_id}")
            res.ok = False
            return res
        self._set_status(req.box_id, status, req.reason)
        res.ok = True
        return res

    # ══════════════════════════════════════════════════════════════
    # /db/mark_placed  — 실측 pose 를 남긴다
    # ══════════════════════════════════════════════════════════════
    def on_mark_placed(self, req, res):
        p = req.actual_pose
        with self._w:
            cur = self.db.execute(
                "UPDATE boxes_sorted SET zone=?, place_x_mm=?, place_y_mm=?,"
                " place_z_mm=?, place_yaw_deg=? WHERE box_id=?",
                (req.zone.upper(), int(p.x_mm), int(p.y_mm), int(p.z_mm),
                 float(p.yaw_deg), req.box_id))
            self.db.commit()
        if cur.rowcount == 0:
            self.get_logger().error(f"mark_placed: 등록되지 않은 박스 {req.box_id}")
            res.ok = False
            return res
        self._set_status(req.box_id, "PLACED")
        res.ok = True
        self.get_logger().info(
            f"placed {req.box_id} → {req.zone.upper()} "
            f"({p.x_mm}, {p.y_mm}, {p.z_mm}) yaw {p.yaw_deg:.1f}")
        return res

    # ══════════════════════════════════════════════════════════════
    # /db/zone_queue
    # ══════════════════════════════════════════════════════════════
    def on_zone_queue(self, req, res):
        z = req.zone.upper()
        # ★ 대기열은 DIVERTED·PICKED 만 센다. SCANNED 를 넣으면 아직 분기도 안 된
        #   박스가 구역 부하로 잡혀 중재자가 멀쩡한 구역을 막는다.
        res.count = self.db.execute(
            "SELECT COUNT(*) FROM boxes_sorted WHERE zone=? AND status IN "
            "('DIVERTED','PICKED')", (z,)).fetchone()[0]
        res.placed_count = self.db.execute(
            "SELECT COUNT(*) FROM boxes_sorted WHERE zone=? AND status='PLACED'",
            (z,)).fetchone()[0]
        return res

    # ══════════════════════════════════════════════════════════════
    # /diverter/request  — ★ 0.6 초 안에 답해야 한다
    # ══════════════════════════════════════════════════════════════
    def on_diverter(self, req, res):
        """목적지를 정한다. target 은 "A"|"B"|"C"|"RETURN" 네 가지뿐이다.

        씬이 한 번 묻고 게이트 3 개를 스스로 운전하는 방식이다.
        게이트마다 묻고 싶으면 `/diverter/decide` 를 쓴다.
        """
        t0 = time.perf_counter()
        box_id = req.box_id.strip()
        res.target, res.reason = self._decide(box_id)
        return self._diverter_done(res, box_id, t0)

    # ══════════════════════════════════════════════════════════════
    # /diverter/decide  — 분기기 한 대에 대한 답. ★ 0.6 초 안에
    # ══════════════════════════════════════════════════════════════
    def on_diverter_decide(self, req, res):
        """이 분기기로 뺄지 말지. 판단은 **첫 분기기에서 한 번만** 한다.

        ★ 분기기마다 새로 판단하면 retry_count 가 한 바퀴에 3 오른다 —
          멀쩡한 박스가 두 바퀴 만에 폐기된다. 그래서 캐시한다.
          마지막 분기기를 답한 뒤 버린다: 다음 바퀴에는 다시 판단해야 한다
          (반송 루프를 돌아 cam1 을 다시 지나 등록됐을 수 있다).
        """
        t0 = time.perf_counter()
        box_id = req.box_id.strip()
        did = int(req.diverter_id)

        cached = self._dv_cache.get(box_id)
        if did <= 1 or cached is None:
            cached = self._decide(box_id)      # 여기서만 부작용이 일어난다
            self._dv_cache[box_id] = cached
        target, reason = cached

        mine = self._zone_of_diverter(did)
        res.divert = bool(mine is not None and target == mine)
        res.target, res.reason = target, reason
        if did >= self._n_diverters:
            self._dv_cache.pop(box_id, None)   # 한 바퀴 끝 — 다음엔 다시 판단
        self.get_logger().info(
            f"분기기 {did}({mine or '?'}) {box_id} -> "
            f"{'빼낸다' if res.divert else '통과'} (목적지 {target} · {reason})"
            f"  [{(time.perf_counter() - t0) * 1000:.1f}ms]")
        return res

    def _zone_of_diverter(self, diverter_id):
        """분기기 번호 -> 담당 구역. 1 이 메인 라인에서 제일 앞이다.

        ★ 순서는 `zones.list` 를 따른다. 씬의 물리 배치와 이 목록의 순서가
          어긋나면 A 로 갈 박스가 C 지선으로 빠진다 — 에러 없이 틀린다.
          배치가 다르면 yaml 의 zones.list 순서를 씬에 맞출 것.
        """
        zs = [str(z).upper() for z in self.cfg["zones"]["list"]]
        i = int(diverter_id) - 1
        return zs[i] if 0 <= i < len(zs) else None

    def _decide(self, box_id):
        """중재자의 판단 규칙 5 단계. -> (target, reason)

        전부 로컬 sqlite 조회라 마이크로초에 끝난다. 여기서 다른 노드에 서비스를
        걸면 그 왕복이 0.6 초를 먹는다 — 그래서 중재자가 이 프로세스에 있다.

        ★ 부작용이 있다 (retry/bypass 증가 · status 기록). **한 바퀴에 한 번만**
          불러야 한다.
        """
        row = self.db.execute(
            "SELECT zone, status FROM boxes_sorted WHERE box_id=?", (box_id,)).fetchone()

        # ① 등록 자체가 안 됐다 = 투입부 인식 실패. 되돌려 다시 태운다.
        if row is None:
            n = self._bump(box_id, "retry_count")
            if n > self._max_retry:
                reason = f"REJECTED_RETRY_{n}"
                if self._master_zone(box_id) is not None:
                    self._set_status(box_id, "REJECTED", reason)
                return "RETURN", reason
            return "RETURN", f"NOT_REGISTERED_RETRY_{n}"

        zone, status = (row[0] or "").upper(), row[1]

        # ② 이미 폐기된 박스
        if status == "REJECTED":
            return "RETURN", "ALREADY_REJECTED"

        # ③ 목적 구역이 이상하다
        if zone not in ("A", "B", "C"):
            return "RETURN", f"BAD_ZONE_{zone or 'EMPTY'}"

        # ④ 구역이 더 못 받는다 → 되돌린다. ★ 실패가 아니므로 bypass 로 센다
        if self._accepting.get(zone) is False:
            n = self._bump(box_id, "bypass_count")
            return "RETURN", f"ZONE_{zone}_FULL_BYPASS_{n}"

        # ⑤ 정상 — 해당 구역으로 보낸다
        self._set_status(box_id, "DIVERTED")
        return zone, "OK"

    def _diverter_done(self, res, box_id, t0):
        dt = (time.perf_counter() - t0) * 1e3
        limit = float(self.cfg["arbiter"]["decision_timeout_s"]) * 1e3
        if dt > limit:
            # 넘으면 씬이 이미 NORMAL 로 통과시킨 뒤다. 답을 못 쓴 것이나 같다.
            self.get_logger().error(
                f"diverter 지연 {dt:.1f} ms > 제한 {limit:.0f} ms — {box_id}")
        else:
            self.get_logger().debug(
                f"diverter {box_id} → {res.target} ({res.reason}, {dt:.2f} ms)")
        return res

    # ══════════════════════════════════════════════════════════════
    # 지표 — /metrics/summary 로 1 Hz 발행 (작업명세서 §8.6)
    #   쿼리는 packing_db.metrics 에 있다. scripts/report.py 가 같은 것을 쓴다 —
    #   복붙하면 실시간 화면과 최종 보고서의 숫자가 갈린다.
    # ══════════════════════════════════════════════════════════════
    def _publish_metrics(self):
        try:
            m = collect(self.db, self._metrics_zone)
        except Exception as e:                      # noqa: BLE001
            # 지표 하나 때문에 노드가 죽으면 안 된다 (CLAUDE.md §9).
            self.get_logger().error(f"지표 계산 실패: {e}")
            return

        fill, maxh = self._zone_cache.get(self._metrics_zone, (0.0, 0))
        msg = MetricsSummary(
            stamp=self.get_clock().now().to_msg(),
            elapsed_s=float(m["elapsed_s"]),
            total_boxes=m["total_boxes"], scanned_count=m["scanned_count"],
            placed_count=m["placed_count"], pick_failed_count=m["pick_failed_count"],
            rejected_count=m["rejected_count"], queue_len=m["queue_len"],
            recognition_rate=float(m["recognition_rate"]),
            place_success_rate=float(m["place_success_rate"]),
            cycle_avg_s=float(m["cycle_avg_s"]),
            robot_idle_rate=float(m["robot_idle_rate"]),
            fill_rate=fill, max_height_mm=maxh, zone=self._metrics_zone.lower())
        self.pub_metrics.publish(msg)

        # 로그는 10 번에 한 번만. 1 Hz 로 찍으면 다른 로그가 안 보인다.
        self._log_n += 1
        if self._log_n % self._log_every == 0:
            self.get_logger().info(
                f"[지표] 인식 {m['scanned_count']}/{m['total_boxes']} "
                f"({fmt_rate(m['recognition_rate'])}) · 적재 {m['placed_count']} "
                f"· 성공률 {fmt_rate(m['place_success_rate'])} "
                f"· 사이클 {fmt_rate(m['cycle_avg_s'], pct=False)}s "
                f"· 유휴 {fmt_rate(m['robot_idle_rate'])} "
                f"· 충전율 {fill * 100:.1f}% · 최고 {maxh}mm")

    def destroy_node(self):
        try:
            self.db.close()
        finally:
            super().destroy_node()


def main(args=None):
    ap = argparse.ArgumentParser(description="palletris.db 서비스 노드 + 중재자")
    ap.add_argument("--db-path", default=None)
    ap.add_argument("--config", default=None)
    a, ros_args = ap.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = DbService(db_path=a.db_path, config_path=a.config)
    ex = MultiThreadedExecutor()          # ReentrantCallbackGroup 과 짝이다
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
