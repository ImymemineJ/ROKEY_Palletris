#!/usr/bin/env python3
"""Isaac 안의 ROS 2 쪽 배선 — `sim_main` 이 이걸 부른다. **트랙 B 김병직 소유.**

`ros_graph.py`(A) 가 Action Graph 를 **세우는** 파일이라면, 이 파일은 거기에
**무엇을 꽂을지**를 정한다. 나눈 이유는 소유가 다르기 때문이다 —
`sim_main.py` 는 김학민 파일이고, 여기 든 것(로봇 사이클·명령 규약)은 트랙 B 다.

    sim_main ──▶ RosSide.build()   그래프 세우기 전
                 RosSide.wire()    Play + 몇 프레임 뒤
                 RosSide.step()    매 틱 (논블로킹)

## 소켓이 없다

`Cycle` 은 원래 `SimLink`(개행 JSON 소켓 8765)로 Isaac 에 붙었다. 여기서는
같은 프로세스 안이므로 **핸들러를 직접 부른다** (`_InProcLink`).
CLAUDE.md §2 가 폐기한 그 소켓이 이걸로 사라진다.

## ⚠ 스레드 규칙 (§9)

`run_cmd` 는 모션이 끝날 때까지 **블로킹**한다 (5~10초). Isaac 메인 스레드에서
부르면 판정을 내리는 `rm.step()` 이 못 돌아 **데드락**이다. 그래서:

    메인 스레드 : step() 이 명령을 워커에 넘기고 결과만 회수한다 (논블로킹)
    워커 스레드 : run_cmd() -> Cycle -> 핸들러 (소켓 스레드가 하던 그 자리다)

핸들러들은 원래 소켓 스레드에서 불렸으므로 이미 스레드 안전하다
(`mainq.run()` 으로 USD 작업을 메인에 맡기고, `robot_motion` 은 `_lk` 로 잠근다).
"""
import threading

import ros_graph


class _InProcLink:
    """`SimLink` 와 같은 인터페이스인데 소켓이 없다. 핸들러 표를 직접 부른다.

    ★ `Cycle` 을 한 줄도 안 고치기 위해서다. `Cycle` 은 `link.call(cmd=...)` 만
      알면 되고, 그게 소켓 너머인지 같은 프로세스인지는 안 봐도 된다.
    """

    def __init__(self, handlers):
        self.h = dict(handlers)

    def call(self, **req):
        fn = self.h.get(req.get("cmd"))
        if fn is None:
            return {"ok": False, "error": "모르는 cmd: %r" % req.get("cmd")}
        return fn(req) or {}

    def call_slow(self, timeout_s, **req):
        # 소켓이 없으니 타임아웃이 의미 없다. 핸들러가 알아서 끝난다.
        return self.call(**req)

    def close(self):
        pass


class _NoSvc:
    """`Cycle` 이 요구하는 svc 자리의 빈 껍데기.

    ★ `pick()`/`place()` 가 실제로 부르는 svc 는 `_abort()` 의 `cancel_plan`
      하나뿐이고 **그건 관제탑이 이미 한다** (`discard_box` → `/algo/cancel_plan`).
      여기서 또 부르면 같은 계획을 두 번 취소한다.
    """

    def cancel_plan(self, zone, box_id):
        return True

    def set_status(self, box_id, status):
        return True


def _svc(fn, keep, req_map=None, res_map=None):
    """소켓 핸들러(`fn(req_dict)`) → `ros_graph` 핸들러(`fn(**srv필드)`).

    Args:
        fn:      소켓 핸들러. `req` dict 하나를 받는다
        keep:    srv **응답**에 실제로 있는 칸. 나머지는 버린다
        req_map: srv 이름 → 소켓 이름 (예: velocity_mps → mps)
        res_map: 소켓 이름 → srv 이름 (예: actual → actual_mps)

    ★ `keep` 이 필요한 이유: 소켓 규약은 실패할 때 `error` 를 얹는데 srv 에는
      그 칸이 없다. 그대로 넘기면 `ros_graph` 가 못 쓰는 칸을 쓰려다
      "응답 'error' 쓰기 실패" 를 뱉는다 — 진짜 원인(ok=false)이 그 소음에 묻힌다.
    """
    def wrapped(**kw):
        req = dict(kw)
        for src, dst in (req_map or {}).items():
            if src in req:
                req[dst] = req.pop(src)
        res = dict(fn(req) or {})
        for src, dst in (res_map or {}).items():
            if src in res:
                res[dst] = res.pop(src)
        # 버리기 전에 실패 사유는 로그로 남긴다 — 조용히 사라지면 안 된다.
        if not res.get("ok") and res.get("error"):
            print("[ros_side] %s 실패: %s" % (getattr(fn, "__name__", "?"), res["error"]))
        return {k: v for k, v in res.items() if k in keep}
    return wrapped


class RosSide:
    """Isaac 안의 ROS 2 배선 한 벌.

    Args:
        cfg:      system.yaml
        zone:     "A"|"B"|"C"
        handlers: sim_main 이 이미 만든 소켓용 {cmd: fn} 표
        log:      한 줄 로거
    """

    def __init__(self, cfg, zone, handlers, log=print):
        self.zone, self.log = str(zone).upper(), log
        self.link = _InProcLink(handlers)

        # ★ Cycle 은 **하나만** 만들어 끝까지 쓴다. 명령마다 새로 만들면
        #   _stack_top_m(이송 고도의 바닥)이 0 으로 돌아가 팔이 적재물 위를
        #   낮게 지나 스택을 친다 — §10-3 knock 700.8mm 와 같은 경로다.
        # ★ 여기는 **Isaac 안**이라 colcon 설치본(3.12)을 못 본다.
        #   `_paths.py` 가 붙여 준 평면 경로로 들어간다 (demo_stack15 와 같다).
        try:
            from cycle import Cycle
        except ImportError:                               # 밖에서 부를 때 (테스트)
            from packing_robot.cycle import Cycle
        self.cycle = Cycle(self.link, _NoSvc(), cfg, self.zone, log=log)

        self._busy = threading.Lock()
        self._done = []                  # 워커가 넣고 step() 이 비운다
        self._done_lk = threading.Lock()

        # ★ 서비스 핸들러는 **전역 한 벌**이라 여기서 안 건다 — 구역별 인스턴스가
        #   각자 걸면 마지막 것만 남는다. `install_services()` 를 sim_main 이 한 번 부른다.

    # ── sim_main 이 부르는 것 ───────────────────────────────
    # ★ `build()`/`wire()` 는 **그래프 전체**에 대해 한 번만 부른다 (아래 모듈 함수).
    #   구역이 셋이어도 그래프는 하나고 서비스도 한 벌이다 — `ros_graph.build`
    #   docstring 참조. 구역별 인스턴스는 `step()` 만 각자 돈다.

    def step(self):
        """매 틱. **절대 블로킹하지 않는다.**"""
        # ① 끝난 결과부터 내보낸다
        with self._done_lk:
            evs, self._done = self._done, []
        for ev in evs:
            ros_graph.send_event(self.zone, **ev)

        # ② 새 명령
        cmd = ros_graph.take_cmd(self.zone)
        if cmd is None:
            return
        if not self._busy.acquire(blocking=False):
            # ★ 조용히 버리지 않는다. 답이 없으면 관제탑은 20초를 버린 뒤에야 안다.
            self.log("[ros_side] 이미 모션 중인데 seq=%s 가 왔다 — ok=false 로 답한다"
                     % cmd.get("seq"))
            self._push(dict(seq=int(cmd.get("seq", 0)), action=int(cmd.get("action", 0)),
                            box_id=str(cmd.get("box_id", "")), ok=False, reason="ABORTED",
                            has_actual=False, actual_x_mm=0, actual_y_mm=0,
                            actual_z_mm=0, actual_yaw_deg=0.0, duration_s=0.0))
            return
        threading.Thread(target=self._work, args=(cmd,), daemon=True).start()

    # ── 워커 스레드 ────────────────────────────────────────
    def _work(self, cmd):
        try:
            try:
                from cmd_adapter import run_cmd
            except ImportError:
                from packing_robot.cmd_adapter import run_cmd
            # ★ 규약(seq 복사 · has_actual 게이트 · 예외에도 답하기)은 여기 없다 —
            #   `cmd_adapter.run_cmd` 하나에 있다. 과도기 노드
            #   (`packing_robot/robot_node.py`)도 같은 함수를 부른다.
            self._push(run_cmd(self.cycle, _clean(cmd), log=self.log))
        finally:
            self._busy.release()

    def _push(self, ev):
        with self._done_lk:
            self._done.append(ev)


def _clean(cmd):
    """Action Graph 가 준 값을 파이썬 기본형으로. numpy 스칼라가 섞여 온다."""
    out = {}
    for k, v in cmd.items():
        try:
            out[k] = v.item() if hasattr(v, "item") else v
        except Exception:                                       # noqa: BLE001
            out[k] = v
    return out


# ══ 그래프 전체에 한 번만 부르는 것들 ════════════════════════════════════════
# ★ 구역이 셋이어도 그래프는 하나고 서비스도 한 벌이다. 구역별 `RosSide` 가 각자
#   부르면 노드가 겹치거나 핸들러가 덮어써진다.

def install_services(handlers):
    """씬 서비스 4종을 그래프에 꽂는다. **한 번만** 부른다.

    `handlers` 는 sim_main 이 만든 소켓용 {cmd: fn} 표다. 소켓과 ROS 가 서로 다른
    함수를 부르면 둘이 어긋나므로 같은 표를 쓴다.
    """
    h = handlers
    ros_graph.set_handlers(
        # srv SetVelocity: seg · velocity_mps -> ok · actual_mps
        # 소켓          : seg · mps          -> ok · actual
        set_velocity=_svc(h["set_velocity"], ("ok", "actual_mps"),
                          req_map={"velocity_mps": "mps"},
                          res_map={"actual": "actual_mps"}),
        despawn_box=_svc(h["despawn_box"], ("ok",)),
        set_spawn_enabled=_svc(h["set_spawn_enabled"], ("ok",)),
        swap_pallet=_svc(h["swap_pallet"], ("ok",)),
    )


def build(zones, cfg=None, log=print):
    """그래프를 세운다. **Play 전에** 한 번 부른다.

    `cfg` 를 주면 `/clock`·`/tf` 도 **씬에 없을 때만** 같이 세운다. 없으면
    `use_sim_time` 노드들의 시계가 0 에 멎어 타이머가 하나도 안 돈다 (§10).
    """
    ros_graph.build(zones=list(zones))
    for z in zones:
        log("[ros_side] %s — %s / %s"
            % (z, ros_graph.cmd_topic(z), ros_graph.evt_topic(z)))
    if cfg is not None:
        ros_graph.build_clock_tf(cfg, log=log)


def wire(log=print):
    """동적 필드를 잇는다. **Play + 몇 프레임 뒤**에 한 번 부른다 (TS-022 2번)."""
    ros_graph.wire()
    log("[ros_side] 배선 완료 — 명령 대기")
