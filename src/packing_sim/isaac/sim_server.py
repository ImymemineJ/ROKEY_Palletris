"""localhost 개행 JSON 명령 서버 (공통가이드 §3). **김학민 소유.**

    server = SimServer(port=8765)
    server.register(spawner.handlers())
    server.register(robot_motion.handlers())
    server.register({"ping": lambda req: {"ok": True}})
    ...
    server.close()

요청 하나 -> 응답 하나. push 없음. 브릿지가 폴링한다.

## ★ 왜 가이드의 `poll()` 골격을 안 쓰나 — 그대로 만들면 100% 데드락이다

트랙A 가이드(§A6~A8)의 `SimServer` 는 논블로킹 소켓을 메인 루프에서 `poll()` 하는
구조다. 즉 **핸들러가 메인(물리) 스레드에서 실행된다.** 그런데 B4 의
`robot_motion.h_grasp` / `h_release` 는 프로토콜 표가 `{"ok":bool}` 을 요구하므로
판정이 날 때까지 `threading.Event.wait()` 로 기다리는데, 그 Event 를 set 하는 것은
`rm.step()` 이고 `rm.step()` 은 **메인 스레드**에서 돈다.

    메인 스레드: world.step() -> srv.poll() -> h_grasp() -> evt.wait()
                                                              ^
                        이 스레드가 여기서 멈췄으므로 rm.step() 이 영영 안 돈다
                        -> evt 가 영영 set 되지 않는다 -> 서로를 기다린다

그래서 **연결마다 스레드**를 둔다. 핸들러는 소켓 스레드에서 돌고, 메인 스레드는
물리만 돌린다. 이것이 `scripts/verify_b4_cycle.py` 의 `StandInServer` 가 택한
구조이고, B4 가 실제로 완주한 유일한 배선이다. 공통가이드 §3 규칙 1
("핸들러는 절대 블로킹하지 않는다")이 막으려는 것은 **물리 루프가 서는 것**이지
소켓 스레드가 기다리는 것이 아니다 — `robot_motion.py` docstring 이 같은 말을 한다.

## 규칙 (공통가이드 §3)

  2. 모르는 cmd 에는 `{"ok":false,"error":"unknown_cmd"}`. **크래시 금지.**
  3. 클라이언트를 **동시에 여러 개** 받는다 (브릿지 + 오케스트레이터).
     단일 연결로 짜면 Day 3 에 오케스트레이터가 조용히 멈춘다.

## 스레드 안전

핸들러 표(`self.handlers`)는 `register()` 로만 바뀌고, 등록은 서버를 띄운 직후
메인 스레드에서 한 번에 끝난다. 그 뒤로는 읽기만 하므로 락이 필요 없다.
**핸들러 자신의 스레드 안전은 핸들러가 책임진다** — `RobotMotion` 은 내부 락을
들고 있고, `Spawner` 는 요청을 큐에 넣고 메인 스레드가 소비한다.
"""
import json
import socket
import threading

DEFAULT_PORT = 8765          # 공통가이드 §3. verify_b4_cycle.py 는 8799 를 쓴다


class SimServer:
    def __init__(self, port=DEFAULT_PORT, host="127.0.0.1", log=print):
        self.handlers = {}
        self._log = log
        self.port = port
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 이전 실행이 남긴 TIME_WAIT 때문에 못 뜨는 것을 막는다.
        # ("Address already in use" 가 뜨면 pkill -f sim_main 을 먼저 해라.)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(8)
        self.srv.settimeout(0.2)          # close() 가 걸릴 수 있게
        self._stop = threading.Event()
        self._conns = []
        self._t = threading.Thread(target=self._accept, daemon=True,
                                   name="sim_server-accept")
        self._t.start()
        self._log("[sim_server] %s:%d 대기" % (host, port))

    # ── 등록 ────────────────────────────────────────────────
    def register(self, table):
        """{cmd: fn} 을 통째로 등록한다. fn(req: dict) -> dict."""
        dup = set(table) & set(self.handlers)
        if dup:
            # 조용히 덮어쓰면 어느 트랙 핸들러가 이겼는지 알 수 없게 된다.
            raise ValueError("cmd 가 중복 등록됐다: %s" % sorted(dup))
        self.handlers.update(table)
        return self

    def cmds(self):
        return sorted(self.handlers)

    # ── 소켓 ────────────────────────────────────────────────
    def _accept(self):
        while not self._stop.is_set():
            try:
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break                      # close() 로 소켓이 닫혔다
            self._conns.append(conn)
            threading.Thread(target=self._serve, args=(conn,), daemon=True,
                             name="sim_server-conn").start()

    def _serve(self, conn):
        f = conn.makefile("rw")
        while not self._stop.is_set():
            try:
                line = f.readline()
            except OSError:
                break
            if not line:
                break                      # 상대가 끊었다
            try:
                req = json.loads(line)
                fn = self.handlers.get(req.get("cmd"))
                # 규칙 2 — 모르는 cmd 에 죽지 않는다
                res = fn(req) if fn else {"ok": False, "error": "unknown_cmd"}
            except Exception as e:
                # 핸들러가 던져도 서버는 산다. 안 그러면 박스 하나 잘못 집었다고
                # 소켓이 끊겨 오케스트레이터가 통째로 BLOCKED 로 간다.
                res = {"ok": False, "error": repr(e)}
                self._log("[sim_server] 핸들러 예외: %r" % (e,))
            try:
                f.write(json.dumps(res) + "\n")
                f.flush()
            except OSError:
                break
        try:
            conn.close()
        except OSError:
            pass

    def close(self):
        self._stop.set()
        for c in self._conns:
            try:
                c.close()
            except OSError:
                pass
        try:
            self.srv.close()
        except OSError:
            pass
