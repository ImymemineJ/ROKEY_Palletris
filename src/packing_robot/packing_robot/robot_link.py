"""Isaac 소켓 클라이언트 — 공통가이드 §3 프로토콜 (개행구분 JSON, 요청-응답).

orchestrator 가 이걸 통해 로봇을 움직인다. **rclpy 를 import 하지 않는다** — 순수 소켓이다.
소켓 직접 호출은 B 소유 명령(move_ee/motion_status/grasp/release/is_holding/tcp_speed)과
get_box_pose 로 제한한다 (§3 규칙 4). 벨트·팔레트·스폰은 ROS 서비스를 경유한다.
"""
import json
import socket


class RobotLink:
    def __init__(self, host="127.0.0.1", port=8765, timeout=1.0):
        self.addr, self.timeout = (host, port), timeout
        self.sock = None
        self.buf = b""

    def connect(self):
        """연결 성공하면 True. 서버(sim_server.py)가 아직 없으면 False — 예외를 던지지 않는다."""
        try:
            s = socket.create_connection(self.addr, timeout=self.timeout)
            s.settimeout(self.timeout)
            self.sock, self.buf = s, b""
            return self.call("ping").get("ok", False)
        except OSError:
            self.sock = None
            return False

    def call(self, cmd, **params):
        """요청 1건 → 응답 1건. 실패하면 {"ok": False, "error": ...} 를 돌려준다(예외 없음)."""
        if self.sock is None:
            return {"ok": False, "error": "not_connected"}
        try:
            self.sock.sendall((json.dumps({"cmd": cmd, **params}) + "\n").encode())
            while b"\n" not in self.buf:
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise OSError("closed")
                self.buf += chunk
            line, self.buf = self.buf.split(b"\n", 1)
            return json.loads(line.decode())
        except (OSError, ValueError) as e:
            self.sock = None                      # 다음 틱에 재연결을 시도할 수 있게 끊어 둔다
            return {"ok": False, "error": str(e)}

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None
