"""소켓 스레드 -> 메인(물리) 스레드 작업 큐.

    q = MainQueue()
    # 소켓 스레드
    res = q.run(lambda: stage.RemovePrim(path))
    # 메인 스레드, 매 스텝
    q.pump()

## 왜 필요한가 — USD 는 스레드 안전하지 않다

소켓 핸들러는 연결마다 다른 스레드에서 돈다 (`sim_server.py`). 거기서 USD 를
직접 고치면 메인 스레드의 `world.step()` 과 겹쳐 **행이 걸리거나 스테이지가 깨진다.**

2026-08-22 실측: `conveyor_ctl.set_velocity` 가 소켓 스레드에서
`graph:variable:Velocity` 를 그냥 `Set()` 했더니, 몇 번은 통과하고 그 다음
호출에서 응답이 영영 안 왔다 (프로세스는 살아 있고 소켓도 열려 있는데 응답만
안 온다). 한 번은 파이썬 트레이스백 없이 프로세스가 그대로 죽었다.
**증상이 매번 다르다는 것 자체가 경쟁 상태의 표시다.**

그래서 **USD 를 건드리는 일은 전부 여기를 거친다.** 읽기만 하는 핸들러
(`pickup_check`, `motion_status`)는 메인 스레드가 갱신해 둔 스냅샷을 보므로
큐가 필요 없다.

`robot_motion.py` 가 `grasp`/`release` 에서 Event 로 기다리는 것과 같은 구조다 —
**소켓 스레드만 기다리고 물리는 계속 돈다.**
"""
import threading

DEFAULT_TIMEOUT_S = 5.0


class MainQueue:
    def __init__(self, timeout_s=DEFAULT_TIMEOUT_S, log=print):
        self.timeout_s = timeout_s
        self._log = log
        self._q = []
        self._lk = threading.Lock()

    def run(self, fn):
        """메인 스레드에서 fn() 을 돌리고 결과를 받아 온다. **소켓 스레드에서 호출.**"""
        evt, out = threading.Event(), {}
        with self._lk:
            self._q.append((fn, evt, out))
        if not evt.wait(self.timeout_s):
            return {"ok": False, "error": "sim 메인 루프가 응답하지 않는다"}
        return out.get("res", {"ok": False, "error": "no result"})

    def pump(self):
        """쌓인 작업을 처리한다. **메인 스레드에서 매 물리 스텝 한 번.**"""
        with self._lk:
            pending, self._q = self._q, []
        for fn, evt, out in pending:
            try:
                out["res"] = fn()
            except Exception as e:            # 작업 하나가 죽어도 루프는 산다
                out["res"] = {"ok": False, "error": repr(e)}
                self._log("[mainq] 작업 예외: %r" % (e,))
            evt.set()
