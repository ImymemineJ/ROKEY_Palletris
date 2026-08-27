"""소켓 스레드 -> 메인(물리) 스레드 작업 큐.

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : (전 구간) 스레드 안전 — USD 쓰기를 메인 스레드로 넘긴다
    ║     불러 주는 쪽        : 소켓/워커 스레드 전부
    ║     이 파일이 부르는 쪽 : sim_main 의 q.pump()
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

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

# ★ **클라이언트보다 반드시 먼저 포기해야 한다.**
#   `sim_link.SimLink` 의 소켓 타임아웃은 system.yaml `service.timeout_s` = 2.0 s 다.
#   여기가 5.0 이면 메인 루프가 2~5 s 사이로 밀렸을 때 클라이언트가 **먼저**
#   socket.timeout 으로 빠져나오는데, 그 사이 sim 은 정상 처리하고 응답 한 줄을
#   소켓에 그대로 써 넣는다. SimLink 는 write 한 줄 · readline 한 줄로 짝을 맞추므로
#   버퍼에 남은 그 한 줄 때문에 **그 뒤 모든 호출이 직전 명령의 답을 받는다** —
#   set_velocity 가 ok=true 를 돌려주는데 실은 그 전 pickup_check 의 답인 식이다.
#   예외도 로그도 없이 조용히 어긋난다. 그래서 서버 예산을 클라이언트의 80 % 로 둔다.
#   숫자의 단일 출처는 system.yaml 이다 — `sim_main.py` 가 그 값을 넣어 준다.
DEFAULT_TIMEOUT_S = 1.6


class MainQueue:
    def __init__(self, timeout_s=DEFAULT_TIMEOUT_S, log=print):
        self.timeout_s = timeout_s
        self._log = log
        self._q = []
        self._lk = threading.Lock()
        # ★ 메인 스레드가 누구인지 기억한다. 생성자는 sim_main.main() 안이라
        #   보통 여기가 메인이고, pump() 가 매 스텝 갱신하므로 확실해진다.
        self._main_tid = threading.get_ident()

    def run(self, fn):
        """메인 스레드에서 fn() 을 돌리고 결과를 받아 온다.

        ★ **메인 스레드에서 불러도 안전하다** (2026-08-26 김병직).
          원래는 소켓 스레드 전용이었다. 그런데 ROS 2 로 옮기면서 서비스 핸들러가
          **Action Graph 의 ScriptNode 안**, 즉 **메인 스레드**에서 돌게 됐다
          (`ros_graph.py` docstring 3번). 거기서 여기를 부르면 큐에 넣고
          `evt.wait()` 로 기다리는데, 그 큐를 비우는 `pump()` 도 같은 메인
          스레드라 **자기가 자기를 기다린다.** 타임아웃까지 멎었다가
          "sim 메인 루프가 응답하지 않는다" 를 돌려준다 — 실측 2026-08-26,
          `/conveyor/set_velocity` 가 항상 ok=false 였다.
          이미 메인이면 큐를 거칠 이유가 없으므로 **그 자리에서 실행한다.**
        """
        if threading.get_ident() == self._main_tid:
            return fn()
        evt, out = threading.Event(), {}
        with self._lk:
            self._q.append((fn, evt, out))
        if not evt.wait(self.timeout_s):
            out["abandoned"] = True
            return {"ok": False,
                    "error": "sim 메인 루프가 응답하지 않는다 (%.1f s)" % self.timeout_s}
        return out.get("res", {"ok": False, "error": "no result"})

    def pump(self):
        """쌓인 작업을 처리한다. **메인 스레드에서 매 물리 스텝 한 번.**"""
        # 여기를 부르는 스레드가 메인이다 — run() 이 이 값으로 판별한다.
        self._main_tid = threading.get_ident()
        with self._lk:
            pending, self._q = self._q, []
        for fn, evt, out in pending:
            try:
                out["res"] = fn()
            except Exception as e:            # 작업 하나가 죽어도 루프는 산다
                # ★ repr(e) 만 남기면 **어디서 났는지가 사라진다.** 2026-08-24 실측:
                #   카메라 인식이 NameError("LOADER_DIR") 로 죽는데 그게 우리 코드인지
                #   Isaac 내부인지 알 수 없어 엉뚱한 곳을 두 번 고쳤다.
                #   호출자는 error 만 보므로 응답 크기는 그대로 두고, 트레이스백은
                #   stdout 에 한 번 찍는다.
                import traceback
                print("[mainq] 작업 실패: %r\n%s" % (e, traceback.format_exc()),
                      flush=True)
                out["res"] = {"ok": False, "error": repr(e)}
                self._log("[mainq] 작업 예외: %r" % (e,))
            if out.get("abandoned"):
                # 부수효과는 이미 났다 — 되돌릴 수 없다. 최소한 "실패라는데 실제로는
                # 일어난" 상태 불일치를 추적할 수 있게 남긴다.
                self._log("[mainq] 늦게 끝난 작업 — 호출자는 이미 실패로 처리했다: %r"
                          % (out.get("res"),))
            evt.set()
