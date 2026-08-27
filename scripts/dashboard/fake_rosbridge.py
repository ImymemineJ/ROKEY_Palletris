#!/usr/bin/env python3
"""rosbridge **대역** — 대시보드의 실시간 칸을 ROS 없이 확인하는 시험용 서버.

    python3 scripts/dashboard/fake_rosbridge.py            # ws://localhost:9090
    브라우저: http://localhost:8090/truck.html

## 무엇이고 무엇이 아닌가

★ **시험용이다. 시연에 쓰지 말 것.** 진짜는 `ros-jazzy-rosbridge-suite` 다.
  이건 rosbridge 프로토콜 중 대시보드가 쓰는 **두 가지만** 흉내낸다:
  클라이언트의 `{"op":"subscribe"}` 를 받고, `{"op":"publish"}` 를 내보낸다.

★ **왜 필요한가:** rosbridge 는 apt 패키지라 아무 PC 에나 깔려 있지 않고,
  깔려 있어도 관제탑이 돌지 않으면 실시간 칸을 확인할 수 없다. 그러면 화면 쪽
  버그(토픽 이름·필드 이름·렌더링)를 **시연 당일에야** 발견하게 된다.
  표준 라이브러리만으로 WebSocket 핸드셰이크와 프레임을 처리하면 그걸 앞당길 수 있다.

★ 내보내는 값은 `ZoneStatus` **필드 이름과 타입을 그대로** 따른다
  (`packing_msgs/msg/ZoneStatus.msg`). 이름이 하나라도 다르면 화면이 조용히
  빈 칸을 그리므로, 그 사고를 여기서 잡는 것이 이 파일의 목적이다.

★ 숫자는 **가짜라는 게 보이게** 만든다. 실측 문서값(사이클 10.6 s 같은 것)을
  흉내내지 않는다 — 기존 대시보드의 demo 모드가 그렇게 해서, 스크린샷을 실측
  근거로 오독할 여지를 남겼다.
"""
import argparse
import base64
import hashlib
import json
import socket
import struct
import threading
import time

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"     # RFC 6455
ZONES = ("a", "b", "c")
STATES = ["IDLE", "FETCHING", "PLANNING", "PICKING", "PLACING", "UPDATING"]


def _handshake(conn):
    """WebSocket 업그레이드. 실패하면 False."""
    data = b""
    while b"\r\n\r\n" not in data:
        chunk = conn.recv(4096)
        if not chunk:
            return False
        data += chunk
        if len(data) > 65536:
            return False
    key = None
    for line in data.split(b"\r\n"):
        if line.lower().startswith(b"sec-websocket-key:"):
            key = line.split(b":", 1)[1].strip()
    if not key:
        return False
    accept = base64.b64encode(hashlib.sha1(key + GUID.encode()).digest()).decode()
    conn.sendall(
        b"HTTP/1.1 101 Switching Protocols\r\n"
        b"Upgrade: websocket\r\nConnection: Upgrade\r\n"
        b"Sec-WebSocket-Accept: " + accept.encode() + b"\r\n\r\n")
    return True


def _send(conn, text):
    """서버 -> 클라이언트 텍스트 프레임 (마스크 없음)."""
    p = text.encode()
    n = len(p)
    if n < 126:
        head = struct.pack("!BB", 0x81, n)
    elif n < (1 << 16):
        head = struct.pack("!BBH", 0x81, 126, n)
    else:
        head = struct.pack("!BBQ", 0x81, 127, n)
    conn.sendall(head + p)


def _recv(conn):
    """클라이언트 -> 서버 프레임 하나. 마스크를 풀어 문자열로. 끊기면 None."""
    def need(n):
        b = b""
        while len(b) < n:
            c = conn.recv(n - len(b))
            if not c:
                return None
            b += c
        return b

    h = need(2)
    if not h:
        return None
    op, ln = h[0] & 0x0F, h[1] & 0x7F
    masked = bool(h[1] & 0x80)
    if op == 0x8:                      # close
        return None
    if ln == 126:
        ln = struct.unpack("!H", need(2))[0]
    elif ln == 127:
        ln = struct.unpack("!Q", need(8))[0]
    mask = need(4) if masked else b"\0\0\0\0"
    body = need(ln) or b""
    if masked:
        body = bytes(b ^ mask[i % 4] for i, b in enumerate(body))
    return body.decode("utf-8", "replace") if op == 0x1 else ""


def _zone_status(z, tick):
    """`ZoneStatus.msg` 와 **필드 이름·타입이 같아야 한다.**

    ★ 이름이 하나라도 다르면 화면이 에러 없이 빈 칸을 그린다. 이 대역의 존재
      이유가 그 사고를 미리 잡는 것이다.
    """
    i = ZONES.index(z)
    placed = (tick // 4 + i * 3) % 18
    return {
        "zone": z,
        "state": STATES[(tick // 3 + i) % len(STATES)],
        "queue_len": 0,                       # 관제탑도 상수 0 을 낸다 (실제와 같게)
        "placed_count": placed,
        "rejected_count": (tick // 25 + i) % 4,
        "fill_rate": round(min(0.95, placed / 20.0), 3),
        "max_height_mm": placed * 55,
        "accepting": True,
        "stamp": {"sec": int(time.time()), "nanosec": 0},
    }


def serve(client, addr, hz):
    if not _handshake(client):
        return client.close()
    subs = set()
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            msg = _recv(client)
            if msg is None:
                break
            try:
                m = json.loads(msg)
            except ValueError:
                continue
            if m.get("op") == "subscribe" and m.get("topic"):
                subs.add(m["topic"])
                print(f"  구독 요청: {m['topic']}  (type={m.get('type')})")
        stop.set()

    threading.Thread(target=reader, daemon=True).start()
    print(f"  붙음: {addr[0]}:{addr[1]}")
    tick = 0
    try:
        while not stop.is_set():
            for z in ZONES:
                topic = f"/zone_{z}/status"
                if topic in subs:
                    _send(client, json.dumps(
                        {"op": "publish", "topic": topic,
                         "msg": _zone_status(z, tick)}))
            tick += 1
            time.sleep(1.0 / hz)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
    finally:
        stop.set()
        client.close()
        print(f"  끊김: {addr[0]}:{addr[1]}")


def main():
    ap = argparse.ArgumentParser(description="rosbridge 대역 (시험용)")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--hz", type=float, default=2.0, help="관제탑과 같은 2 Hz")
    a = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((a.host, a.port))
    s.listen(5)
    print(f"rosbridge 대역 — ws://{a.host}:{a.port}  ({a.hz} Hz)")
    print("★ 시험용이다. 시연에는 ros-jazzy-rosbridge-suite 를 쓸 것")
    while True:
        c, addr = s.accept()
        threading.Thread(target=serve, args=(c, addr, a.hz), daemon=True).start()


if __name__ == "__main__":
    main()
