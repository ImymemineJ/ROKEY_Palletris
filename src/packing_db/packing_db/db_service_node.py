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

from packing_common.config import find_config, load
from packing_db.metrics import collect, fmt_rate
from packing_msgs.msg import BoxInfo, MetricsSummary, ZoneStatus
from packing_msgs.srv import (DiverterRequest, GetBoxInfo, MarkPlaced, RegisterBox,
                              SetStatus, VerifyBox, ZoneQueue)

# CLAUDE.md §3-1 BoxInfo.status 가 정의한 8 개. 이 밖의 값은 받지 않는다.
VALID_STATUS = {"SPAWNED", "SCANNED", "VERIFIED", "DIVERTED",
                "PICKED", "PLACED", "PICK_FAILED", "REJECTED"}

# status → boxes_sorted 의 시각 컬럼. 없는 것은 box_events 에만 남는다.
TS_COLUMN = {"SPAWNED": "spawned_at", "SCANNED": "scanned_at",
             "PICKED": "picked_at", "PLACED": "placed_at"}


def default_db_path():
    """워크스페이스 루트의 palletris.db.

    system.yaml 이 <ws>/src/packing_bringup/config/system.yaml 이므로 4 단계 위가 루트다.
    상대경로를 쓰면 실행 디렉터리마다 다른 빈 DB 가 생겨 "master 에 없는 박스" 만
    찍힌다 (트랙 D 가이드 D1 경고).
    """
    cfg_path = os.path.realpath(find_config())
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(cfg_path))))
    return os.path.join(ws, "palletris.db")


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
        self.db = sqlite3.connect(self.db_path, check_same_thread=False)
        self.db.execute("PRAGMA journal_mode=WAL")
        self._w = threading.Lock()          # 쓰기 직렬화
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
                (DiverterRequest, "/diverter/request", self.on_diverter)):
            self.create_service(srv_type, name, fn, callback_group=cb)

        # ── 구역 수용 여부 캐시 ──
        # ★ 중재자는 ZoneStatus 의 accepting 한 칸만 본다 (CLAUDE.md §3-2).
        #   버퍼 잔량·예상 여유시간을 직접 계산하던 옛 방식은 관제탑과 계산이
        #   어긋나면 조용히 잘못 보낸다. 판단 주체를 관제탑 하나로 몰아 둔다.
        self._accepting = {}                     # zone(대문자) → bool
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

    # ══════════════════════════════════════════════════════════════
    # 공통
    # ══════════════════════════════════════════════════════════════
    def _ensure_schema(self):
        """schema.sql 을 적용한다. CREATE TABLE IF NOT EXISTS 라 여러 번 돌아도 안전하다."""
        here = os.path.dirname(os.path.realpath(__file__))
        for cand in (os.path.join(here, "schema.sql"),
                     os.path.join(here, "..", "share", "packing_db", "schema.sql")):
            if os.path.isfile(cand):
                with open(cand, encoding="utf-8") as f, self._w:
                    self.db.executescript(f.read())
                return
        raise SystemExit("schema.sql 을 찾지 못했다")

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

        master_zone = self._master_zone(box_id)
        if master_zone is None:
            # master 에 없는 id. 스포너와 id 규칙이 어긋난 것이다 (BOX%05d).
            res.ok, res.message = False, f"master 에 없는 박스: {box_id}"
            self.get_logger().error(res.message)
            return res

        dest = (req.dest or "").strip().upper()
        zone = dest if dest in ("A", "B", "C") else master_zone
        if dest and dest != master_zone:
            # 인식이 틀렸다. 죽이지 않고 기록만 한다 — 이 값이 곧 오분류율 지표다.
            self.get_logger().warn(
                f"{box_id} dest 불일치: 읽은 값 {dest} · master {master_zone}")

        with self._w:
            self.db.execute(
                "INSERT INTO boxes_sorted(box_id, zone, size_w_mm, size_d_mm, size_h_mm,"
                " mass_g, color, dest, qr_payload, status) "
                "VALUES(?,?,?,?,?,?,?,?,?,'SCANNED') "
                "ON CONFLICT(box_id) DO UPDATE SET "
                "  zone=excluded.zone, size_w_mm=excluded.size_w_mm,"
                "  size_d_mm=excluded.size_d_mm, size_h_mm=excluded.size_h_mm,"
                "  mass_g=excluded.mass_g, color=excluded.color, dest=excluded.dest,"
                "  qr_payload=excluded.qr_payload",
                (box_id, zone, size[0], size[1], size[2], mass,
                 req.color, dest, req.qr_payload))
            self.db.commit()
        self._set_status(box_id, "SCANNED")

        res.ok, res.message = True, f"zone={zone}"
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

        전부 로컬 sqlite 조회라 마이크로초에 끝난다. 여기서 다른 노드에 서비스를
        걸면 그 왕복이 0.6 초를 먹는다 — 그래서 중재자가 이 프로세스에 있다.
        """
        t0 = time.perf_counter()
        box_id = req.box_id.strip()
        row = self.db.execute(
            "SELECT zone, status FROM boxes_sorted WHERE box_id=?", (box_id,)).fetchone()

        # ① 등록 자체가 안 됐다 = 투입부 인식 실패. 되돌려 다시 태운다.
        if row is None:
            n = self._bump(box_id, "retry_count")
            if n > self._max_retry:
                res.target, res.reason = "RETURN", f"REJECTED_RETRY_{n}"
                if self._master_zone(box_id) is not None:
                    self._set_status(box_id, "REJECTED", res.reason)
            else:
                res.target, res.reason = "RETURN", f"NOT_REGISTERED_RETRY_{n}"
            return self._diverter_done(res, box_id, t0)

        zone, status = (row[0] or "").upper(), row[1]

        # ② 이미 폐기된 박스
        if status == "REJECTED":
            res.target, res.reason = "RETURN", "ALREADY_REJECTED"
            return self._diverter_done(res, box_id, t0)

        # ③ 목적 구역이 이상하다
        if zone not in ("A", "B", "C"):
            res.target, res.reason = "RETURN", f"BAD_ZONE_{zone or 'EMPTY'}"
            return self._diverter_done(res, box_id, t0)

        # ④ 구역이 더 못 받는다 → 되돌린다. ★ 실패가 아니므로 bypass 로 센다
        if self._accepting.get(zone) is False:
            n = self._bump(box_id, "bypass_count")
            res.target, res.reason = "RETURN", f"ZONE_{zone}_FULL_BYPASS_{n}"
            return self._diverter_done(res, box_id, t0)

        # ⑤ 정상 — 해당 구역으로 보낸다
        res.target, res.reason = zone, "OK"
        self._set_status(box_id, "DIVERTED")
        return self._diverter_done(res, box_id, t0)

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
