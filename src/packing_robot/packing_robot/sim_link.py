"""Isaac 프로세스와 붙는 개행 JSON 소켓 클라이언트 (공통가이드 §3).

    link = SimLink()
    link.call(cmd="move_ee", target_m=[10.0, 0.0, 1.01], yaw_deg=0.0)

트랙A 가이드의 `SimLink` 와 **같은 클래스다.** 원래 자리는 `packing_common` 인데
C0 가 아직 안 들어와서 각 패키지가 하나씩 들고 있다.
`packing_common` 이 생기면 그리로 옮기고 여기서는 import 만 남긴다.

## 왜 Lock 이 필요한가

`MultiThreadedExecutor` 에서 타이머 콜백과 서비스 콜백이 동시에 부르면 두 요청이
한 소켓에 섞여 나가고, 응답도 뒤바뀐 채 돌아온다. 요청-응답이 1:1 인 프로토콜이라
**호출 전체를 하나의 임계구역**으로 묶어야 한다.

## 왜 timeout 이 필요한가

Isaac 프로세스가 죽으면 `readline()` 이 **에러 없이 영원히** 기다린다. 오케스트레이터는
멈춘 줄도 모르고 서 있게 된다. 그래서 소켓에 타임아웃을 건다 — 끊기면 `socket.timeout`
이 올라오고, 호출자는 명세서 §1.5 대로 `state="BLOCKED"` 를 발행하면 된다.
"""
import json
import socket
import threading

DEFAULT_PORT = 8765
DEFAULT_TIMEOUT_S = 2.0          # 명세서 §1.5 service.timeout_s


class SimLink:
    def __init__(self, host="127.0.0.1", port=DEFAULT_PORT, timeout_s=DEFAULT_TIMEOUT_S):
        self.sock = socket.create_connection((host, port), timeout=timeout_s)
        self.sock.settimeout(timeout_s)
        self.f = self.sock.makefile("rw")
        self.lock = threading.Lock()

    def call(self, **req):
        """요청 하나를 보내고 응답 하나를 받는다. 타임아웃이면 `socket.timeout`."""
        with self.lock:
            self.f.write(json.dumps(req) + "\n")
            self.f.flush()
            line = self.f.readline()
        if not line:
            raise ConnectionError("sim 프로세스가 연결을 닫았다")
        return json.loads(line)

    def call_slow(self, timeout_s, **req):
        """오래 걸리는 명령(`grasp`, `release`)용. 그 호출에만 타임아웃을 늘린다.

        `grasp` 은 그리퍼가 닫기를 시도하는 총 시간(`retry_interval_s`, 기본 10 s)만큼
        걸릴 수 있다. 기본 2 s 로 부르면 정상 동작인데도 타임아웃으로 오판한다.
        """
        with self.lock:
            old = self.sock.gettimeout()
            self.sock.settimeout(timeout_s)
            try:
                self.f.write(json.dumps(req) + "\n")
                self.f.flush()
                line = self.f.readline()
            finally:
                self.sock.settimeout(old)
        if not line:
            raise ConnectionError("sim 프로세스가 연결을 닫았다")
        return json.loads(line)

    def close(self):
        try:
            self.f.close()
        finally:
            self.sock.close()
