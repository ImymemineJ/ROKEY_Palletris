#!/usr/bin/env python3
"""관제탑 대시보드 백엔드 — palletris.db 를 **읽기만** 해서 JSON 으로 낸다.

    python3 scripts/dashboard/server.py            # http://localhost:8090
    python3 scripts/dashboard/server.py --port 9000 --host 0.0.0.0

## 무엇인가

정적 파일(`/` = `truck.html`) + `/api/*` JSON 을 내는 작은 HTTP 서버다.
**표준 라이브러리만 쓴다** — 저장소에 웹 프레임워크가 하나도 없고, 시연 PC 에
pip 설치를 요구하는 순간 위험이 늘기 때문이다.

## 지키는 것

★ **DB 는 `mode=ro` 로만 연다.** `db_service_node` 가 유일 writer 라는 규약
  (CLAUDE.md §10) 을 깨지 않는다. 저장소의 다른 스크립트 10곳이 이미 같은 방식이다
  (`scripts/pallet_access.py` 등). `journal_mode=WAL` 이라 읽기가 쓰기를 안 막는다.

★ **ROS 는 있으면 쓰고 없으면 만다.** `Live` 가 `rclpy` 로 `/zone_*/status` 를
  직접 구독한다 — **rosbridge 가 필요 없다.** rclpy 가 없으면 조용히 꺼진 채
  서버는 그대로 뜨므로 `colcon build` 전에도 DB 화면은 나온다.
  `use_sim_time` 을 켜지 않고 ROS 타이머도 안 써서 `/clock` 함정(§10)에 안 걸린다.

★ **숫자를 코드에 안 박는다.** 팔레트 치수·최대적재는 `scripts/_paths.system_config()`
  로 읽는다 — `PL_CONFIG_OVERLAY` 까지 합쳐 읽으므로 시연(test_3)에서
  팔레트가 90° 돌아 있는 것(800×1200)이 자동으로 반영된다.

★ **지표 쿼리를 복사하지 않는다.** `packing_db.metrics` 를 import 한다.
  실시간 화면과 `scripts/report.py` 의 숫자가 갈라지면 안 된다 (`metrics.py` 머리 주석).

## 알려진 것 — 화면이 숨기지 않고 보여줄 것

1. **팔레트 판갈이가 DB 에 안 적힌다.** 한 구역에 판이 여러 장 서면
   `boxes_sorted` 는 그걸 한 뭉치로 돌려준다. 여기서는 **실측 좌표로 판을 갈라**
   (`_split_pallets`) 현재 판만 3D 로 그린다. 근본 해결은 `pallet_seq` 컬럼이다.
2. **실행이 여러 번 섞인다.** `metrics.collect` 가 현재 실행 구간만 세도록
   고쳐졌다 (2026-08-27). 응답에 `run` 을 실어 화면이 "언제부터의 값인가" 를 띄운다.
"""
import argparse
import errno
import json
import os
import pathlib
import sqlite3
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent.parent                       # ~/palletris_ws

sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src" / "packing_db"))

import _paths                                            # noqa: E402
from packing_db import accessibility, metrics            # noqa: E402

DEF_DB = pathlib.Path(os.environ.get("PALLETRIS_DB") or (ROOT / "palletris.db"))
ZONES = ("a", "b", "c")

#: 박스 종류를 치수로 되찾는다. 3종 치수가 안 겹치므로 유일하게 정해진다.
#: ★ `boxes_sorted.color` 를 쓰지 않는 이유: 지금 전 행이 'red' 다
#:   (vision 의 폴백). CLAUDE.md §15 "색 팔레트 공유" 가 닫히면 그때 쓴다.
def _box_type(cfg, w, d, h):
    for t in cfg["boxes"]["types"]:
        if (t["w_mm"], t["d_mm"], t["h_mm"]) == (w, d, h):
            return t["name"]
    return "?"


def _footprint(b):
    """yaw 를 반영한 바닥 사각형. yaw 는 0/90 뿐이라 가로세로만 맞바꾸면 된다."""
    w, d = b["w"], b["d"]
    if abs(abs(float(b.get("yaw", 0.0))) % 180.0 - 90.0) < 45.0:
        w, d = d, w
    return b["x"] - w / 2, b["x"] + w / 2, b["y"] - d / 2, b["y"] + d / 2


def _hits(a, b, tol=20):
    """두 박스가 **몸으로** 겹치는가. 한 팔레트 안에서는 절대 일어날 수 없다."""
    ax0, ax1, ay0, ay1 = _footprint(a)
    bx0, bx1, by0, by1 = _footprint(b)
    return (min(ax1, bx1) - max(ax0, bx0) > tol
            and min(ay1, by1) - max(ay0, by0) > tol
            and min(a["z"] + a["h"], b["z"] + b["h"]) - max(a["z"], b["z"]) > tol)


def _out_of_pallet(boxes, pw, pd, tol=50):
    """팔레트 밖으로 나간 박스 수. `tol` 은 계획 여유(plan_overhang)와 배치 오차 몫.

    ★ 0 이 아니면 대개 설정이 틀린 것이지 로봇이 틀린 게 아니다 — `PL_CONFIG_OVERLAY`
      를 빼먹으면 팔레트가 1200×800 으로 잡혀 800×1200 씬의 박스가 죄다 삐져나온다.
    """
    n = 0
    for b in boxes:
        x0, x1, y0, y1 = _footprint(b)
        if x0 < -tol or y0 < -tol or x1 > pw + tol or y1 > pd + tol:
            n += 1
    return n


def _split_pallets(boxes):
    """시각 순으로 훑으며 **판이 갈린 자리**를 찾아 나눈다.

    ★ 왜 필요한가: DB 에 판 번호가 없다. 판을 갈면 좌표가 다시 높이 0 부터
      시작하므로, 한 뭉치로 그리면 박스가 서로 뚫고 들어간 더미가 된다.

    ★ 판정: 새 박스가 **이미 놓인 박스와 몸으로 겹치면** 그 앞에서 판이 갈렸다.
      물리적으로 불가능한 배치이므로 다른 판이라는 뜻이다. 실측(구역 B, 32개)에서
      겹침 16쌍이 이 방법으로 3판으로 갈리며 0쌍이 됐고, `PL_MAX_PALLETS` 기본값
      3 과 일치했다.

    ★ 이건 **임시방편이다.** 근본 해결은 `boxes_sorted.pallet_seq` 컬럼이다
      (계획서 §3-1). 그때 이 함수는 지우면 된다.
    """
    out, cur = [], []
    for b in sorted(boxes, key=lambda x: (x["t"] is None, x["t"] or 0)):
        if any(_hits(b, o) for o in cur):
            out.append(cur)
            cur = []
        cur.append(b)
    if cur:
        out.append(cur)
    return out


class Live:
    """`/zone_*/status` 를 **백엔드가 직접** 구독한다. rosbridge 가 필요 없다.

    ## 왜 rosbridge 를 안 쓰나

    브라우저가 ROS 를 직접 상대하면 토픽 이름과 QoS 를 JS 문자열로 적게 된다.
    기존 대시보드가 그래서 `/robot/event`(발행자 0 개)를 구독한 채 영영 빈 화면을
    띄웠다. 여기서 rclpy 로 한 번 정확히 쓰면 그 계통의 사고가 사라진다.
    게다가 rosbridge 는 어느 `package.xml` 에도 없는 apt 의존이라, 시연 PC 마다
    따로 깔아야 한다.

    ## ★ 없으면 없는 대로 돈다

    rclpy 를 못 불러오거나 실패하면 **조용히 꺼진 채** 서버는 그대로 뜬다.
    DB 화면은 ROS 와 무관하게 나오므로, `colcon build` 전이나 ROS 가 없는 PC 에서도
    쓸 수 있다는 성질을 잃지 않는다.

    ## /clock 함정을 피하는 법 (CLAUDE.md §10)

    `use_sim_time` 을 **켜지 않는다.** 켜면 `/clock` 발행자가 없을 때 ROS 시계가
    0 에 멈춰 타이머가 영영 안 뜬다. 여기서는 **ROS 타이머를 아예 안 쓴다** —
    구독 콜백은 메시지가 올 때 뜨므로 시계와 무관하다.
    """

    def __init__(self, zones):
        self.zones = zones
        self.data = {}          # zone -> dict(ZoneStatus 필드)
        self.ok = False
        self.reason = "시작 안 함"
        self._t = None

    def start(self):
        try:
            import rclpy                                   # noqa: F401
        except Exception as e:                             # noqa: BLE001
            self.reason = f"rclpy 없음 ({type(e).__name__}) — install/setup.bash 를 source 했나"
            return
        self._t = threading.Thread(target=self._spin, daemon=True)
        self._t.start()

    def _spin(self):
        try:
            import rclpy
            from rclpy.callback_groups import ReentrantCallbackGroup
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node
            from packing_msgs.msg import ZoneStatus
        except Exception as e:                             # noqa: BLE001
            self.reason = f"packing_msgs 없음 ({e}) — colcon build 후 source 할 것"
            return
        try:
            rclpy.init(args=[])
            node = Node("dashboard_live")
            cb = ReentrantCallbackGroup()        # CLAUDE.md §9

            def make(z):
                def on(msg):
                    self.data[z] = {
                        "zone": msg.zone, "state": msg.state,
                        "queue_len": msg.queue_len,
                        "placed_count": msg.placed_count,
                        "rejected_count": msg.rejected_count,
                        "fill_rate": float(msg.fill_rate),
                        "max_height_mm": msg.max_height_mm,
                        "accepting": bool(msg.accepting),
                        # 화면이 "얼마나 오래된 값인가" 를 알 수 있게 벽시계로 찍는다.
                        # 메시지의 stamp 는 sim 시각이라 브라우저 시계와 못 비교한다.
                        "recv_wall": time.time(),
                    }
                return on

            for z in self.zones:
                node.create_subscription(ZoneStatus, f"/zone_{z}/status",
                                         make(z), 10, callback_group=cb)
            self.ok, self.reason = True, "구독 중"
            ex = MultiThreadedExecutor()         # CLAUDE.md §9
            ex.add_node(node)
            ex.spin()
        except Exception as e:                             # noqa: BLE001
            self.ok = False
            self.reason = f"{type(e).__name__}: {e}"

    def snapshot(self, stale_s=5.0):
        """지금 값. ★ 오래된 것은 버린다 — 노드가 죽어도 마지막 값이 남으면
        "ROS 미연결" 이라 적힌 화면에 조금 전 값이 지금 값처럼 보인다."""
        now = time.time()
        fresh = {z: v for z, v in self.data.items()
                 if now - v.get("recv_wall", 0) <= stale_s}
        return {"ok": self.ok and bool(fresh), "reason": self.reason, "zones": fresh}


class Store:
    """DB 읽기. 요청마다 새로 연다 — 스레드마다 커넥션이 따로여야 한다."""

    def __init__(self, db_path, cfg):
        self.db_path = pathlib.Path(db_path)
        self.cfg = cfg

    def _con(self):
        if not self.db_path.exists():
            raise FileNotFoundError(f"DB 가 없다: {self.db_path}")
        # ★ mode=ro — 읽기 전용. 쓰기는 db_service_node 독점 (CLAUDE.md §10)
        return sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True,
                               timeout=2.0)

    # ---------- 팔레트 ----------
    def pallet(self, zone):
        z = zone.upper()
        con = self._con()
        try:
            ev0 = metrics._run_start(con, "box_events")
            rows = con.execute(
                "SELECT s.box_id, s.place_x_mm, s.place_y_mm, s.place_z_mm,"
                "       s.place_yaw_deg, s.size_w_mm, s.size_d_mm, s.size_h_mm,"
                "       s.mass_g, s.placed_at, m.address, m.lat, m.lon "
                "FROM boxes_sorted s LEFT JOIN boxes_master m USING(box_id) "
                "WHERE s.zone=? AND s.status='PLACED' AND s.place_z_mm IS NOT NULL "
                "  AND s.box_id IN (SELECT DISTINCT box_id FROM box_events WHERE id>=?) "
                "ORDER BY s.placed_at", (z, ev0)).fetchall()
        finally:
            con.close()

        boxes = [{"box_id": r[0], "x": r[1], "y": r[2], "z": r[3],
                  "yaw": round(r[4] or 0.0, 1),
                  "w": r[5], "d": r[6], "h": r[7], "mass_g": r[8],
                  "t": r[9], "address": r[10], "lat": r[11], "lon": r[12],
                  "type": _box_type(self.cfg, r[5], r[6], r[7])} for r in rows]

        pallets = _split_pallets(boxes)
        cur = pallets[-1] if pallets else []
        pal = self.cfg["pallet"]
        return {
            "zone": zone.lower(),
            # ★ 지금 팔레트 = 마지막 판. 앞의 판들은 트럭으로 나갔다
            "boxes": cur,
            "shipped": [{"index": i + 1, "count": len(p),
                         "max_height_mm": max((b["z"] + b["h"] for b in p), default=0)}
                        for i, p in enumerate(pallets[:-1])],
            "pallet_index": len(pallets),
            "max_height_mm": max((b["z"] + b["h"] for b in cur), default=0),
            # ★ 치수는 설정에서. test_3 오버레이면 800×1200 으로 뒤바뀐다
            "pallet_w_mm": pal["width_mm"], "pallet_d_mm": pal["depth_mm"],
            "max_stack_mm": pal["max_stack_mm"],
            "cell_mm": self.cfg["heightmap"]["cell_mm"],
            # ★ 진단: 판을 갈랐는데도 남은 겹침. 0 이 아니면 화면이 경고를 띄운다
            "overlaps": sum(1 for i, a in enumerate(cur) for b in cur[i + 1:]
                            if _hits(a, b)),
            # ★ 진단: 박스가 팔레트 밖으로 나가나.
            #   가장 흔한 원인은 **PL_CONFIG_OVERLAY 를 빼먹은 것**이다.
            #   test_3 씬은 지선이 +Y 라 팔레트가 90° 돌아 있어 800×1200 인데,
            #   오버레이 없이 읽으면 system.yaml 의 1200×800 이 잡힌다.
            #   그러면 조용히 틀린 그림이 나온다 — 그걸 화면이 말하게 한다.
            "out_of_pallet": _out_of_pallet(cur, pal["width_mm"], pal["depth_mm"]),
            "overlay": os.environ.get("PL_CONFIG_OVERLAY", ""),
        }

    def unload(self, zone):
        """꺼내기 쉬운 순서. 현재 판만 본다 — 나간 판은 이미 트럭에 있다."""
        cur = self.pallet(zone)["boxes"]
        order = accessibility.unload_order(cur)
        by_id = {b["box_id"]: b for b in cur}
        return [{"seq": b["seq"], "layer": b["layer"], "box_id": b["box_id"],
                 "top_mm": b["z"] + b["h"],
                 "address": by_id[b["box_id"]].get("address")} for b in order]

    # ---------- 박스 이력 ----------
    def box(self, box_id):
        con = self._con()
        try:
            ev0 = metrics._run_start(con, "box_events")
            info = con.execute(
                "SELECT box_id, zone, size_w_mm, size_d_mm, size_h_mm, mass_g,"
                "       color, dest, status, place_x_mm, place_y_mm, place_z_mm,"
                "       place_yaw_deg, spawned_at, scanned_at, picked_at, placed_at "
                "FROM boxes_sorted WHERE box_id=?", (box_id,)).fetchone()
            ev = con.execute(
                "SELECT status, reason, ts FROM box_events "
                "WHERE box_id=? AND id>=? ORDER BY id", (box_id, ev0)).fetchall()
            addr = con.execute(
                "SELECT address, delivery_seq, zone, lat, lon FROM boxes_master "
                "WHERE box_id=?", (box_id,)).fetchone()
        finally:
            con.close()
        if info is None:
            return None
        k = ("box_id zone size_w_mm size_d_mm size_h_mm mass_g color dest status "
             "place_x_mm place_y_mm place_z_mm place_yaw_deg spawned_at scanned_at "
             "picked_at placed_at").split()
        out = dict(zip(k, info))
        out["events"] = [{"status": s, "reason": r, "ts": t} for s, r, t in ev]
        out["master"] = (dict(zip(("address", "delivery_seq", "zone", "lat", "lon"),
                                  addr)) if addr else None)
        return out

    # ---------- 최근 사건 (이벤트 로그) ----------
    def events(self, limit=200):
        """이번 실행의 박스 사건을 최신순으로.

        ★ ROS 토픽이 아니라 **DB 의 `box_events`** 를 쓴다. 이유 둘:
          1. `/zone_*/robot/event` 는 같은 결과를 계속 다시 낸다 — rosbag 실측으로
             run3 에서 2,634 건 중 **실제 사건은 27 건**(1.0 %)이었다. 나머지는
             같은 `(seq, action)` 의 반복과 빈 하트비트다.
          2. 토픽은 **지난 것을 다시 주지 않는다.** 화면을 늦게 열면 그 전 사건이
             통째로 빈다. DB 는 처음부터 다 갖고 있다.
          -> 실시간 감각은 `/zone_*/status`(2 Hz)가 담당하고, **사건 목록은 DB** 가 낸다.
        """
        con = self._con()
        try:
            ev0 = metrics._run_start(con, "box_events")
            rows = con.execute(
                "SELECT e.ts, e.box_id, e.status, e.reason, s.zone,"
                "       s.size_w_mm, s.size_d_mm, s.size_h_mm, m.address "
                "FROM box_events e "
                "LEFT JOIN boxes_sorted s USING(box_id) "
                "LEFT JOIN boxes_master m USING(box_id) "
                "WHERE e.id>=? ORDER BY e.id DESC LIMIT ?", (ev0, int(limit))).fetchall()
            t0 = con.execute("SELECT MIN(ts) FROM box_events WHERE id>=?",
                             (ev0,)).fetchone()[0]
        finally:
            con.close()
        out = []
        for ts, bid, st, rs, zone, w, d, h, addr in rows:
            out.append({
                "ts": ts,
                # 실행 시작부터의 경과. 벽시계·sim 어느 쪽이어도 읽을 수 있게 한다
                "t_rel": (ts - t0) if (ts is not None and t0 is not None) else None,
                "box_id": bid, "status": st, "reason": rs or "",
                "zone": (zone or "").lower(),
                "type": _box_type(self.cfg, w, d, h) if w else None,
                "address": addr,
            })
        return out

    # ---------- 지표 ----------
    def metrics(self, zone=None):
        con = self._con()
        try:
            m = metrics.collect(con, zone)
            # 구역별 대기 수 — ZoneStatus.queue_len 이 상수 0 이라 여기서 센다
            ev0 = m["run_start_id"]
            m["queue_by_zone"] = {
                z: con.execute(
                    "SELECT COUNT(*) FROM boxes_sorted WHERE zone=? AND status IN "
                    "('DIVERTED','PICKED') AND box_id IN "
                    "(SELECT DISTINCT box_id FROM box_events WHERE id>=?)",
                    (z.upper(), ev0)).fetchone()[0] for z in ZONES}
        finally:
            con.close()
        return m

    # ---------- 트럭 1대 요약 ----------
    def truck(self):
        pallets = {z: self.pallet(z) for z in ZONES}
        m = self.metrics()
        return {
            "pallets": {z: {"count": len(p["boxes"]),
                            "max_height_mm": p["max_height_mm"],
                            "pallet_index": p["pallet_index"],
                            "shipped": len(p["shipped"]),
                            "overlaps": p["overlaps"]}
                        for z, p in pallets.items()},
            "placed_total": sum(len(p["boxes"]) for p in pallets.values()),
            "shipped_total": sum(len(p["shipped"]) for p in pallets.values()),
            "metrics": m,
            # ★ "언제부터의 값인가" 를 화면이 띄울 수 있게 같이 낸다.
            #   DB 가 오염돼 있으면 조용히 넘어가지 않는다 (계획서 §7 D2)
            "run": {"start_id": m["run_start_id"],
                    "t0": m["run_t0"], "t1": m["run_t1"],
                    "elapsed_s": m["elapsed_s"]},
        }


class Handler(BaseHTTPRequestHandler):
    store: Store = None
    live: "Live" = None
    server_version = "PalletrisDash/1.0"

    def log_message(self, fmt, *a):        # 접근 로그는 조용히
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False, default=str).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _static(self, name):
        # ★ 경로 탈출 방지 — 이 폴더 밖은 절대 안 준다
        p = (HERE / name).resolve()
        if HERE not in p.parents and p != HERE:
            return self.send_error(403)
        if not p.is_file():
            return self.send_error(404)
        ct = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
              ".css": "text/css; charset=utf-8", ".json": "application/json",
              ".svg": "image/svg+xml"}.get(p.suffix, "application/octet-stream")
        body = p.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path == "/api/truck":
                return self._json(self.store.truck())
            if path == "/api/metrics":
                return self._json(self.store.metrics())
            if path == "/api/live":
                return self._json(Handler.live.snapshot())
            if path == "/api/events":
                q = parse_qs(urlparse(self.path).query)
                n = int((q.get("limit") or ["200"])[0])
                return self._json(self.store.events(min(max(n, 1), 1000)))
            if path.startswith("/api/pallet/"):
                rest = path[len("/api/pallet/"):].strip("/").split("/")
                z = rest[0].lower()
                if z not in ZONES:
                    return self._json({"error": f"모르는 구역: {z}"}, 404)
                if len(rest) > 1 and rest[1] == "unload":
                    return self._json(self.store.unload(z))
                return self._json(self.store.pallet(z))
            if path.startswith("/api/box/"):
                b = self.store.box(path[len("/api/box/"):].strip("/"))
                return self._json(b if b else {"error": "그런 박스가 없다"},
                                  200 if b else 404)
            if path == "/api/health":
                return self._json({"ok": True, "db": str(self.store.db_path),
                                   "db_exists": self.store.db_path.exists()})
            # ★ 루트는 **이 서버의 화면**(truck.html)이다.
            #   같은 폴더의 index.html 은 어제 올라온 다른 대시보드(6043fdf)이고
            #   rosbridge 를 쓴다. 루트를 그쪽으로 두면 아무것도 모르고 들어온
            #   사람이 옛 화면을 보게 된다 — 실제로 그랬다.
            #   옛 화면은 /index.html 로 그대로 열린다 (남의 파일이라 안 건드린다).
            return self._static("truck.html" if path in ("/", "") else path.lstrip("/"))
        except FileNotFoundError as e:
            # ★ DB 가 없어도 서버는 안 죽는다. 화면이 이유를 보여 준다
            return self._json({"error": str(e)}, 503)
        except Exception as e:                     # noqa: BLE001
            return self._json({"error": f"{type(e).__name__}: {e}"}, 500)


def main():
    ap = argparse.ArgumentParser(description="관제탑 대시보드 백엔드 (읽기 전용)")
    ap.add_argument("--port", type=int, default=8090,
                    help="기본 8090. ★ 8765 는 쓰지 말 것 — 폐기된 소켓 포트이고 "
                         "sim_main 이 아직 그 포트를 쓴다")
    ap.add_argument("--host", default="127.0.0.1",
                    help="기본 로컬만. 다른 PC 에서 보려면 0.0.0.0")
    ap.add_argument("--db", type=pathlib.Path, default=DEF_DB)
    ap.add_argument("--no-ros", action="store_true",
                    help="ROS 구독을 아예 시도하지 않는다 (DB 화면만)")
    a = ap.parse_args()

    cfg = _paths.system_config()          # PL_CONFIG_OVERLAY 까지 합쳐 읽는다
    Handler.store = Store(a.db, cfg)
    # ★ ROS 는 **있으면 쓰고 없으면 만다.** 없어도 DB 화면은 그대로 나온다
    Handler.live = Live(ZONES)
    if not a.no_ros:
        Handler.live.start()

    pal = cfg["pallet"]
    print(f"관제탑 대시보드 — http://{a.host}:{a.port}/")
    print(f"  DB      : {a.db}  ({'있음' if a.db.exists() else '★ 없음'})")
    print(f"  팔레트  : {pal['width_mm']}(X) x {pal['depth_mm']}(Y) mm "
          f"· 최대 {pal['max_stack_mm']} mm")
    if os.environ.get("PL_CONFIG_OVERLAY"):
        print(f"  오버레이: {os.environ['PL_CONFIG_OVERLAY']}")
    time.sleep(0.6)                        # 구독이 붙을 짬
    lv = Handler.live.snapshot()
    print(f"  실시간  : {'구독 중' if Handler.live.ok else '꺼짐 — ' + lv['reason']}")
    print("  Ctrl+C 로 종료")
    try:
        srv = ThreadingHTTPServer((a.host, a.port), Handler)
    except OSError as e:
        # ★ 파이썬 traceback 을 그대로 뱉으면 "왜 안 되는지" 를 알기 어렵다.
        #   실제로 걸렸다 — 이미 띄워 둔 서버를 잊고 다시 실행했을 때다.
        if e.errno != errno.EADDRINUSE:
            raise
        sys.exit(
            f"\n!! 포트 {a.port} 를 이미 누가 쓰고 있다.\n"
            f"   쓰는 프로세스 보기 :  ss -ltnp | grep {a.port}\n"
            f"   내리기             :  pkill -f 'dashboard/server.py'\n"
            f"   아니면 다른 포트로 :  --port {a.port + 1}\n")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n종료")            # Ctrl+C 에 traceback 을 뱉지 않는다


if __name__ == "__main__":
    main()
