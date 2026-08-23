"""`isaac-sim.sh --exec` 로 띄운 헤드리스 시험을 **확실히** 끝낸다.

## 왜 따로 필요한가

`--exec` 스크립트는 앱을 소유하지 않는다. B5 견본(`demo_stack15.py`)은
`SimulationApp` 을 자기가 만들어서 `finally: kit.close()` 로 닫지만, 여기서는
kit 이 먼저 뜨고 스크립트가 얹히는 구조라 그 손잡이가 없다.

그래서 지금까지는 결과를 찍은 뒤

    omni.timeline.get_timeline_interface().stop()
    omni.kit.app.get_app().post_quit()

만 불렀다. **이것으로는 안 죽는다.** 두 줄 다 물리 스텝 콜백
(`subscribe_physics_step_events`) 안에서 불리는데, 그 틱은 kit 의 업데이트
루프 한가운데다. 타임라인을 그 안에서 세우면 재진입이고, `post_quit()` 이
올린 종료 이벤트는 다음 업데이트에서 뽑혀야 하는데 그 업데이트가 안 온다.

실측 2026-08-23: 결과표까지 다 찍고 `RESULT:` 를 출력한 실행 3개가 각각
6 · 12 · 23 분째 살아서 CPU 를 113 / 138 / 218 % 먹고 있었다. 셋이 같은 GPU 를
나눠 쓰는 바람에 **다음 실행의 물리 타이밍이 흔들렸다** — 같은 시드·같은
코드로 knock 이 18.4 mm 와 67.3 mm 로 갈렸다. 재현이 안 되는 게이트는
게이트가 아니다. 죽는 것은 편의가 아니라 정확도 요건이다.

## 어떻게 죽이는가 — 세 겹

1. 물리 스텝 구독을 먼저 끊는다. 이걸 안 하면 종료 중에도 `on_step` 이 계속
   돌아 이미 닫힌 소켓·프림을 만진다.
2. 실제 종료는 **다음 앱 업데이트**로 미룬다. 물리 콜백 밖이라 재진입이 없다.
3. 그래도 안 죽으면 `HARD_S` 초 뒤 `os._exit()`. 헤드리스 시험에는 저장할
   상태가 없다 — 결과는 이미 stdout 으로 나갔다.
"""
import os
import sys
import threading
import time

import omni.kit.app
import omni.timeline

HARD_S = float(os.environ.get("PL_HARD_EXIT_S", "20"))   # 3겹째가 끊는 시각


def shutdown(code=0, physx_sub=None, log=print):
    """돌아오지 않는다고 가정하고 불러라. 부른 뒤의 코드는 돌 수도, 안 돌 수도 있다."""
    if physx_sub is not None:
        try:
            physx_sub.unsubscribe()
        except Exception:
            pass

    def _hard():
        time.sleep(HARD_S)
        try:
            log("[kitexit] %.0f s 안에 안 닫혔다 — 강제 종료한다 (os._exit %d)"
                % (HARD_S, code))
            sys.stdout.flush()
        except Exception:
            pass
        os._exit(code)

    threading.Thread(target=_hard, daemon=True, name="kitexit").start()

    app = omni.kit.app.get_app()
    state = {"sub": None, "n": 0}

    def _on_update(_e):
        state["n"] += 1
        if state["n"] == 1:
            try:
                omni.timeline.get_timeline_interface().stop()
            except Exception:
                pass
            app.post_quit()
        elif state["n"] > 120:            # 2 초쯤 지나도 안 닫히면 3겹째에 맡긴다
            try:
                state["sub"].unsubscribe()
            except Exception:
                pass

    state["sub"] = app.get_update_event_stream().create_subscription_to_pop(
        _on_update, name="palletris-kitexit")
    sys.stdout.flush()
