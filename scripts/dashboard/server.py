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
import array
import errno
import json
import os
import pathlib
import sqlite3
import struct
import sys
import threading
import time
import zlib
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


#: 카메라 프레임을 화면에 보낼 때 쓰는 PNG 인코더.
#: ★ 왜 직접 쓰나: 이 서버는 표준 라이브러리만 쓴다(머리 주석). Pillow·OpenCV 를
#:   요구하면 시연 PC 마다 pip 설치가 필요해지고, cv_bridge 는 ROS 가 없는 PC 에서
#:   못 쓴다. PNG 는 zlib 한 번이면 되므로 의존성을 늘릴 이유가 없다.
#: ★ JPEG 이 아니라 PNG 인 이유도 같다 — JPEG 인코더는 표준 라이브러리에 없다.
def _png(rgb, w, h, level=1):
    """RGB8 바이트 → PNG. level 1 = 빠른 쪽. 화면용이라 압축률보다 지연이 중요하다."""
    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)                       # 필터 타입 0 (None)
        raw += rgb[y * stride:(y + 1) * stride]

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(bytes(raw), level))
            + chunk(b"IEND", b""))


def _to_rgb(fr, max_w=640):
    """sensor_msgs/Image 원본 바이트 → (rgb8 바이트, w, h).

    ★ numpy 가 있으면 **줄여서** 보낸다. 1280x720 을 그대로 PNG 로 만들면 한 장에
      1 MB 가 넘어 4 Hz 로 보내는 순간 회선과 CPU 를 다 먹는다. 화면에서는 어차피
      가로 절반이면 충분하다. numpy 가 없으면 원본 그대로 간다 (동작은 한다).
    ★ 인코딩 종류를 코드가 **골라서** 받는다. Isaac 은 rgb8 을 내지만 다른 노드가
      bgr8 을 내면 색이 뒤집힌 채 그냥 보이므로, 여기서 한 번에 맞춘다.
    """
    w, h, enc, step = fr["w"], fr["h"], fr["enc"], fr["step"]
    data = fr["data"]
    try:
        import numpy as np
    except Exception:                       # noqa: BLE001 — 없으면 원본 rgb8 만
        return (data, w, h) if enc == "rgb8" else (None, 0, 0)

    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if not ch or len(data) < h * step:
        return None, 0, 0
    a = np.frombuffer(data, dtype=np.uint8)[:h * step].reshape(h, step)[:, :w * ch]
    a = a.reshape(h, w, ch)
    s = max(1, -(-w // max_w))              # 정수 배율로만 줄인다 (보간 없음 = 빠르다)
    a = a[::s, ::s]
    if ch == 1:
        a = np.repeat(a, 3, axis=2)
    elif enc.startswith("bgr"):
        a = a[:, :, [2, 1, 0]]
    else:
        a = a[:, :, :3]
    return np.ascontiguousarray(a).tobytes(), a.shape[1], a.shape[0]


def _zone_colors(cfg):
    """구역 → 화면 색. **씬이 박스를 칠하는 색 그대로** 쓴다 (A 빨강·B 초록·C 파랑).

    ★ 표를 새로 만들지 않는다. `routing.by_color`(색→존)와 `boxes.colors`(색→RGB)를
      이어 붙여 유도한다 — test_3.yaml 이 같은 이유로 네 번째 표를 금지하고 있다.
      화면 색이 씬과 다르면 "저 빨간 박스" 라는 말이 두 가지를 가리키게 된다.
    """
    out = {}
    try:
        by_color = cfg["routing"]["by_color"]        # {"red": "A", ...}
        rgb = cfg["boxes"]["colors"]                 # {"red": [0.9, 0.03, 0.03], ...}
        for name, zone in by_color.items():
            c = rgb.get(name)
            if not c:
                continue
            out[str(zone).lower()] = "#%02x%02x%02x" % tuple(
                max(0, min(255, int(round(float(v) * 255)))) for v in c[:3])
    except Exception:                       # noqa: BLE001 — 없으면 화면이 기본색을 쓴다
        return {}
    return out


def _poly_bbox(items, w, h):
    """찾은 QR 들을 감싸는 픽셀 사각형. 다음 프레임에서 여기만 다시 본다."""
    xs = [pt[0] * w for it in items for pt in it["poly"]]
    ys = [pt[1] * h for it in items for pt in it["poly"]]
    return (min(xs), min(ys), max(xs), max(ys)) if xs else None


def _qr_scan(fr, max_w=640, deep=True, roi=None):
    """프레임에서 QR 을 찾는다 → [{"text", "poly": [[x,y] x4]}], 좌표는 **0~1 비율**.

    ★ 왜 비율인가: 화면이 창 크기에 따라 영상을 늘렸다 줄였다 한다. 픽셀 좌표를
      보내면 화면 쪽에서 다시 배율을 맞춰야 하고, 한 번 틀리면 박스가 엉뚱한 데
      그려진다. 비율로 보내면 화면은 곱하기만 하면 된다.

    ★ 왜 인식 노드에서 안 받고 여기서 다시 읽나: `vision_node` 는 QR **문자열만**
      쓰고 위치는 버린다 (`detect.read_qr`). 위치를 받으려면 `packing_msgs` 에
      필드를 늘려야 하고 그건 팀 전원 재빌드다 (§11·§14). 화면에 표시하자고
      공용 인터페이스를 바꿀 일은 아니라, 보여주는 쪽에서 한 번 더 읽는다.
      ⚠ 그래서 이 표시는 **"대시보드가 읽은 것"이지 "인식 노드가 등록한 것"이 아니다.**
        둘이 다르면 인식 쪽 문제를 드러내는 단서가 된다 (같은 화면을 봤는데 다르다).

    ★ cv2 가 없으면 조용히 빈 목록. 표시가 없을 뿐 영상은 그대로 나온다.
    """
    try:
        import cv2
        import numpy as np
    except Exception:                       # noqa: BLE001
        return []
    w, h, enc, step = fr["w"], fr["h"], fr["enc"], fr["step"]
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if not ch or len(fr["data"]) < h * step:
        return []
    a = np.frombuffer(fr["data"], dtype=np.uint8)[:h * step].reshape(h, step)[:, :w * ch]
    a = a.reshape(h, w, ch)
    def scan(sub, off=(0, 0), step=1):
        """한 조각을 훑는다. 좌표는 **원본 전체 기준** 0~1 비율로 돌려준다."""
        gray = (sub[:, :, 0] if ch == 1
                else cv2.cvtColor(np.ascontiguousarray(sub[:, :, :3]), cv2.COLOR_RGB2GRAY))
        det = cv2.QRCodeDetector()
        try:
            ok, texts, pts, _ = det.detectAndDecodeMulti(gray)
        except Exception:                   # noqa: BLE001 — 버전에 따라 없다
            ok = False
        if not ok or pts is None:
            try:
                txt, p1, _ = det.detectAndDecode(gray)
            except Exception:               # noqa: BLE001
                return []
            if p1 is None:
                return []
            texts, pts = [txt], p1.reshape(1, -1, 2)
        out = []
        for txt, quad in zip(texts, pts):
            if quad is None or len(quad) < 4:
                continue
            out.append({"text": txt or "",
                        "poly": [[round((off[0] + float(x) * step) / w, 4),
                                  round((off[1] + float(y) * step) / h, 4)]
                                 for x, y in quad[:4]]})
        return out

    # ★ ① 직전에 찾은 자리 둘레만 먼저 본다 (2026-08-27).
    #   화면 전체를 훑는 건 비싸서 0.4 초에 한 번밖에 못 했고, 그 사이 벨트가
    #   0.5 m/s 로 움직여 **빨간 박스가 QR 을 뒤따라갔다.** 자리를 알고 있을 때는
    #   그 둘레(1.6 배)만 원본 해상도로 보면 되고, 그건 몇 ms 라 매 프레임 할 수 있다.
    #   놓치면 아래 전체 탐색이 다시 잡는다 — 잡고 나면 다시 싸진다.
    if roi:
        rx0, ry0, rx1, ry1 = roi
        pw, ph = (rx1 - rx0) * 0.6 + 8, (ry1 - ry0) * 0.6 + 8
        x0 = max(0, int(rx0 - pw)); x1 = min(w, int(rx1 + pw))
        y0 = max(0, int(ry0 - ph)); y1 = min(h, int(ry1 + ph))
        if x1 - x0 > 24 and y1 - y0 > 24:
            got = scan(a[y0:y1, x0:x1], off=(x0, y0))
            if got and all(it["text"] for it in got):
                return got

    # ★ 두 번 훑는다 — 줄인 그림 먼저, 실패하면 원본 (2026-08-27 실측).
    #   줄여서만 훑으면 **1280 폭에서 QR 이 160 px 보다 작을 때 못 읽는다**
    #   (실측: 240·160 성공 · 100 은 찾되 내용 못 읽음 · 70 이하 실패).
    #   투입부 카메라가 벨트를 넓게 잡으면 QR 이 그 크기로 들어온다.
    #   원본은 4 배 느리지만 **못 읽었을 때만** 하므로, 읽히는 동안은 비용이 그대로다.
    sc = max(1, -(-w // max_w))
    hit = scan(a[::sc, ::sc], step=sc) if sc > 1 else []
    # ★ 원본 재훑기는 **비싸다.** QR 이 없는 화면에서도 매번 하면 컨베이어 그림
    #   기준 한 장에 40 ms, 지저분한 화면은 120 ms 다 — 4 Hz 로 계속 하면 코어
    #   하나를 절반 넘게 먹는다. 렌더가 이미 병목인 시연 PC 에서 그럴 수는 없다.
    #   그래서 ① 뭔가 찾긴 했는데 내용을 못 읽었을 때는 바로 다시 훑고
    #        ② 아무것도 못 찾았을 때는 **가끔만** (deep=True 인 호출에서만) 다시 훑는다.
    partial = bool(hit) and any(not it["text"] for it in hit)
    if partial or (deep and not hit):
        full = scan(a)
        if full and not any(not it["text"] for it in full):
            return full
        hit = hit or full
    return hit


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

    def __init__(self, zones, hm_topic="/algo/heightmap", cams=()):
        self.zones = zones
        self.data = {}          # zone -> dict(ZoneStatus 필드)
        self.hm = {}            # zone -> dict(w, d, cell_mm, grid, recv_wall)
        self.hm_topic = hm_topic
        # ★ 카메라는 **원본 그대로** 들고 있다가 요청이 올 때 인코딩한다.
        #   콜백에서 매 프레임(30 Hz) PNG 를 만들면 아무도 안 보고 있을 때도
        #   CPU 를 태운다. 화면은 4 Hz 면 충분하다.
        self.cams = tuple(cams)
        self.frame = {}         # 이름 -> dict(data, w, h, enc, step, recv_wall)
        # ★ **아무도 안 볼 때는 프레임을 붙잡지도 않는다.** 1280x720 한 장이 2.7 MB 라
        #   30 Hz 로 복사하면 초당 80 MB 다 — 화면을 안 보는 동안 시연 PC 의 메모리
        #   대역만 먹는다. 마지막 요청 시각을 보고 그때만 저장한다.
        #   처음 몇 초는 무조건 받는다 (기동 로그에 해상도를 찍기 위해)
        self.cam_want = time.time()
        self._cam_last = {}     # 이름 -> 마지막으로 저장한 시각
        # ★ QR 은 **프레임당 한 번만** 읽는다. 화면이 4 Hz 로 영상과 표시를 따로
        #   가져가므로, 캐시가 없으면 같은 사진을 두 번 훑게 된다
        self._qr = {}           # 이름 -> (프레임시각, 결과, 훑은시각, 마지막성공시각)
        self._qr_deep = {}      # 이름 -> 마지막으로 원본까지 훑은 시각
        self._qr_roi = {}       # 이름 -> 직전에 QR 이 있던 자리 (픽셀 bbox)
        self.zone_colors = {}   # 구역 -> 화면 색. main() 이 설정에서 넣어 준다
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
            from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                                   ReliabilityPolicy)
            from sensor_msgs.msg import Image
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

            # ── heightmap (algo 의 진짜 격자) ───────────────────────
            # ★ **TRANSIENT_LOCAL 로 구독한다.** 발행이 TRANSIENT_LOCAL 이라
            #   VOLATILE 로도 연결은 되지만, 그러면 **늦게 띄웠을 때 마지막 상태를
            #   못 받아** 다음 박스가 놓일 때까지 화면이 빈다.
            # ★ 이 토픽은 algo 를 `-p heightmap_topic:=/algo/heightmap` 으로 띄워야
            #   나온다 (기본 꺼짐). 없으면 그냥 안 올 뿐 아무것도 안 깨진다.
            if self.hm_topic:
                hq = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)

                def mk_hm(z):
                    def on(msg):
                        try:
                            # 16UC1 · little-endian · height=W(Px), width=D(Py)
                            g = array.array("H")
                            g.frombytes(bytes(msg.data))
                            if sys.byteorder == "big":
                                g.byteswap()
                            W, D = msg.height, msg.width
                            if len(g) < W * D:
                                return
                            self.hm[z] = {"w": W, "d": D, "grid": list(g[:W * D]),
                                          "frame": msg.header.frame_id,
                                          "recv_wall": time.time()}
                        except Exception:            # noqa: BLE001 — 시각화는 부가 기능
                            pass
                    return on

                for z in self.zones:
                    node.create_subscription(Image, f"{self.hm_topic}/{z}",
                                             mk_hm(z), hq, callback_group=cb)

            # ── 카메라 영상 (cam1 등) ───────────────────────────────
            # ★ QoS 는 반드시 BEST_EFFORT (qos_profile_sensor_data).
            #   기본 RELIABLE 로 구독하면 이미지가 **하나도 안 온다** — 에러도 안 난다
            #   (CLAUDE.md §3-4 규약 5).
            if self.cams:
                from rclpy.qos import qos_profile_sensor_data

                def mk_cam(name):
                    def on(msg):
                        # 보는 사람이 없으면 버린다 · 있어도 12 Hz 를 넘겨 받지 않는다
                        # (화면은 4 Hz 로 가져간다 — 더 들고 있어 봐야 안 쓴다)
                        now = time.time()
                        if now - self.cam_want > 5.0:
                            return
                        if now - self._cam_last.get(name, 0) < 0.08:
                            return
                        self._cam_last[name] = now
                        try:
                            self.frame[name] = {
                                "data": bytes(msg.data), "w": msg.width,
                                "h": msg.height, "enc": msg.encoding,
                                "step": msg.step, "recv_wall": time.time()}
                        except Exception:        # noqa: BLE001 — 영상은 부가 기능
                            pass
                    return on

                for ns in self.cams:
                    node.create_subscription(Image, f"{ns}/rgb",
                                             mk_cam(ns.strip("/")),
                                             qos_profile_sensor_data,
                                             callback_group=cb)

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
        # ★ heightmap 은 **오래됐다고 버리지 않는다.** 박스가 놓일 때만 갱신되므로
        #   30 초 넘게 그대로인 게 정상이다 (ZoneStatus 는 2 Hz 라 판정이 다르다).
        cams = {n: {"w": f["w"], "h": f["h"], "enc": f["enc"],
                    "age_s": round(now - f["recv_wall"], 2)}
                for n, f in self.frame.items()}
        return {"ok": self.ok and bool(fresh), "reason": self.reason,
                "zones": fresh, "heightmap": dict(self.hm),
                # ★ 영상 자체는 /api/cam/<이름>.png 로 따로 받는다. 여기엔 "지금
                #   들어오고 있나 · 몇 초 전 것인가" 만 싣는다 — 이걸 안 주면
                #   화면이 멈춘 마지막 프레임을 실시간처럼 보여준다
                "cams": cams, "cam_names": list(self.cams)}

    def heightmap(self, zone):
        return self.hm.get(zone)

    #: QR 을 다시 훑는 최소 간격(초). 화면은 4 Hz 로 물어보지만 그대로 다 훑으면
    #: CPU 를 낭비한다 — 벨트가 0.5 m/s 라 0.4 초면 표시가 충분히 따라간다.
    QR_MIN_S = 0.4
    #: 화면 **전체**를 다시 훑는 간격. 두 가지를 위해 반드시 있어야 한다 —
    #: ① 작은 QR (줄인 그림에서는 안 보인다)  ② **새로 들어온 박스**.
    #: ★ 이게 없으면 ROI 추적이 처음 잡은 QR 만 영영 쳐다본다. 실측으로 확인했다:
    #:   QR 3 개가 나란히 있는데 1 개만 표시됐다. 라인에서는 박스가 계속 들어오므로
    #:   추적만 하면 두 번째 박스부터 화면에 안 뜬다.
    QR_SWEEP_S = 0.7
    #: 방금 읽은 QR 을 잠깐 붙잡아 둔다. 한 프레임 실패마다 표시가 깜빡이면
    #: 읽히고 있는데도 "안 읽힌다" 처럼 보인다.
    QR_HOLD_S = 1.2

    def qr(self, name):
        """최신 프레임의 QR.

        ★ 자리를 알고 있을 때(`roi`)는 **속도 제한을 걸지 않는다.** 둘레만 보는 건
          싸고, 여기서 아끼면 표시가 QR 을 뒤따라간다 — 그게 정확히 고치려던 것이다.
          자리를 모를 때만 전체 탐색이라 그건 제한을 건다.
        """
        now = self.cam_want = time.time()
        n = name.strip("/")
        fr = self.frame.get(n)
        if not fr:
            return None
        key, hit, when, last_ok = self._qr.get(n, (None, [], 0.0, 0.0))
        roi = self._qr_roi.get(n)
        sweep = now - self._qr_deep.get(n, 0.0) >= self.QR_SWEEP_S
        if key != fr["recv_wall"] and (roi or sweep or now - when >= self.QR_MIN_S):
            if sweep:
                # 전체 훑기 — 새로 들어온 QR·작은 QR 을 여기서 잡는다
                fresh = _qr_scan(fr, deep=True)
                self._qr_deep[n] = now
            else:
                # 아는 자리 둘레만 — 싸다. 표시가 QR 을 그대로 따라간다
                fresh = _qr_scan(fr, deep=False, roi=roi)
            if fresh:
                hit, last_ok = fresh, now
                self._qr_roi[n] = _poly_bbox(fresh, fr["w"], fr["h"])
            else:
                # 놓쳤다 — 다음 번엔 전체를 다시 본다. ROI 를 붙들고 있으면
                # 박스가 사라진 자리를 영영 쳐다보게 된다
                self._qr_roi.pop(n, None)
                if now - last_ok > self.QR_HOLD_S:
                    hit = []
            self._qr[n] = (fr["recv_wall"], hit, now, last_ok)
        return {"qrs": hit, "age_s": round(now - fr["recv_wall"], 2),
                "zone_colors": self.zone_colors}

    def png(self, name, max_w=640):
        """이름(cam1)의 최신 프레임을 PNG 로. 없으면 (None, 이유)."""
        self.cam_want = time.time()      # "지금 보고 있다" — 구독 콜백이 이걸 본다
        fr = self.frame.get(name.strip("/"))
        if not fr:
            if not self.ok:
                return None, f"ROS 구독이 꺼져 있다 — {self.reason}"
            return None, f"{name} 영상이 아직 한 장도 안 왔다"
        rgb, w, h = _to_rgb(fr, max_w)
        if not rgb:
            return None, f"처리할 수 없는 인코딩: {fr['enc']}"
        return _png(rgb, w, h), None


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
            # ★ algo 가 내는 **진짜** 격자. 없으면 None — 화면이 DB 좌표로 다시
            #   계산한 것을 그린다. 둘 다 있으면 대조해서 어긋남을 잡을 수 있다.
            "algo_heightmap": (Handler.live.heightmap(zone.lower())
                               if Handler.live else None),
            # ★ 그림이 "어느 쪽에서 본 것인가" 를 스스로 말하게 하는 표시.
            #   설정마다 로봇이 서는 변이 다르다 — 정본은 팔레트 −Y 쪽,
            #   test_3 오버레이는 +X 쪽이다. 화면에 박으면 한쪽이 반드시 틀린다
            "robot_local_mm": self._robot_local(zone),
        }

    def _robot_local(self, zone):
        """로봇 베이스를 팔레트 로컬 mm 으로. 설정에 없으면 None — 표시만 안 한다."""
        try:
            zc = self.cfg["zones"]
            o = zc["pallet_origin_world_mm"][zone.upper()]
            r = zc["robot_base_world_mm"][zone.upper()]
            return [int(r[0] - o[0]), int(r[1] - o[1])]
        except Exception:                    # noqa: BLE001 — 부가 표시라 조용히 끈다
            return None

    def box_brief(self, qr_text):
        """QR 원문(지금 규약은 box_id 그대로) → 화면에 한 줄 띄울 정보. 없으면 None."""
        if not qr_text:
            return None
        try:
            con = self._con()
        except FileNotFoundError:
            return None
        try:
            r = con.execute(
                "SELECT s.box_id, s.status, s.zone, m.address FROM boxes_sorted s "
                "LEFT JOIN boxes_master m USING(box_id) "
                "WHERE s.box_id=? OR s.qr_payload=? LIMIT 1",
                (qr_text, qr_text)).fetchone()
        finally:
            con.close()
        if not r:
            return None
        return {"box_id": r[0], "status": r[1], "zone": r[2], "address": r[3]}

    def unload(self, zone):
        """꺼내기 쉬운 순서. 현재 판만 본다 — 나간 판은 이미 트럭에 있다."""
        return self._unload_from(self.pallet(zone)["boxes"])

    def _unload_from(self, cur):
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

    # ---------- 화면 한 장을 한 번에 ----------
    def state(self, zone, live, events=200):
        """★ 화면이 필요한 걸 **한 요청에** 다 준다.

        전에는 한 주기에 `/api/truck`·`/api/pallet/{z}`·`/api/live`·`/api/events`·
        `/api/pallet/{z}/unload` 를 따로 불렀다 — 다섯 번이다. 화면 하나를 그리는 데
        왕복이 다섯이면 느린 것보다 **상태가 어긋나는 게** 문제다. 각 응답이 서로
        다른 순간의 DB 를 보게 되어, 카드의 적재 수와 3D 의 박스 수가 안 맞는
        순간이 생긴다.

        박스 이력(`/api/box/{id}`)만 따로 둔다 — 눌렀을 때만 필요하고, 여기 넣으면
        매 주기에 안 쓰는 데이터를 실어 나른다.
        """
        pal = self.pallet(zone)
        return {
            "live": live,
            "truck": self.truck(pallets={zone: pal}),
            "pallet": pal,
            "unload": self._unload_from(pal["boxes"]),
            "events": self.events(events),
        }

    # ---------- 트럭 1대 요약 ----------
    def truck(self, pallets=None):
        # ★ 이미 계산한 구역은 다시 안 읽는다. `state()` 가 선택 구역을 넘겨 준다
        pallets = {z: (pallets or {}).get(z) or self.pallet(z) for z in ZONES}
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
            if path == "/api/state":
                q = parse_qs(urlparse(self.path).query)
                z = (q.get("zone") or ["b"])[0].lower()
                if z not in ZONES:
                    return self._json({"error": f"모르는 구역: {z}"}, 404)
                n = int((q.get("events") or ["200"])[0])
                return self._json(self.store.state(
                    z, Handler.live.snapshot(), min(max(n, 1), 1000)))
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
            # QR 표시용. 영상과 따로 받는다 — <img> 는 헤더를 읽을 수 없기 때문이다.
            if path.startswith("/api/cam/") and path.endswith("/qr"):
                name = path[len("/api/cam/"):-len("/qr")].strip("/")
                q = Handler.live.qr(name) if Handler.live else None
                if q is None:
                    return self._json({"qrs": [], "age_s": None})
                # ★ 읽은 QR 이 **DB 에 있는 박스인가** 를 같이 본다. 시연에서
                #   "저 QR 이 뭐냐" 의 답은 번호가 아니라 배송지다.
                for it in q["qrs"]:
                    it["box"] = self.store.box_brief(it["text"])
                return self._json(q)
            # ★ 카메라 한 장. <img src> 가 직접 부르므로 JSON 이 아니라 이미지다.
            if path.startswith("/api/cam/"):
                name = path[len("/api/cam/"):].strip("/")
                if name.endswith(".png"):
                    name = name[:-4]
                q = parse_qs(urlparse(self.path).query)
                mw = int((q.get("w") or ["640"])[0])
                png, why = (Handler.live.png(name, max(160, min(mw, 1920)))
                            if Handler.live else (None, "ROS 가 꺼져 있다"))
                if not png:
                    # ★ 503 + JSON. 화면이 이유를 그대로 띄운다 — 빈 사진틀만
                    #   보여주면 "카메라가 없는 것" 인지 "서버가 죽은 것" 인지 모른다
                    return self._json({"error": why}, 503)
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(png)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return self.wfile.write(png)
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
    # ★ 이름이 아니라 **네임스페이스**다. 실제 토픽은 <ns>/rgb 다 (vision_node 규약).
    ap.add_argument("--cam", default="/cam1",
                    help="화면에 띄울 카메라 네임스페이스. 쉼표로 여러 개 "
                         "(예: /cam1,/cam2,/pick_cam_a). 빈 문자열이면 안 받는다")
    a = ap.parse_args()

    cfg = _paths.system_config()          # PL_CONFIG_OVERLAY 까지 합쳐 읽는다
    Handler.store = Store(a.db, cfg)
    # ★ ROS 는 **있으면 쓰고 없으면 만다.** 없어도 DB 화면은 그대로 나온다
    cams = [c.strip() for c in a.cam.split(",") if c.strip()]
    Handler.live = Live(ZONES, cams=cams)
    Handler.live.zone_colors = _zone_colors(cfg)
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
    if cams:
        got = ", ".join(f"{n}({v['w']}x{v['h']})" for n, v in lv["cams"].items())
        print(f"  카메라  : {', '.join(cams)} → {got or '아직 프레임 없음'}")
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
