"""B4 pick & place 한 사이클 — **실제 씬 `test1-1.usd` 위에서** (트랙 A·B 통합).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec /home/rokey/palletris_ws/scripts/verify_b4_test1_1.py

`scripts/verify_b4_cycle.py` 는 합성 씬(받침대 두 개)에서 B4 를 검증한다.
이 스크립트는 같은 사이클을 **A 가 만든 진짜 라인 위에서** 돌린다. 새로 만드는 것이
없다 — 전부 실물을 그대로 쓴다:

    packing_robot/cycle.py              상태머신 (§2.5)
    packing_robot/gripper.py            흡착
    packing_robot/sim_link.py           개행 JSON 소켓 클라이언트
    packing_sim/isaac/robot_motion.py   B 명령 핸들러 6개
    packing_sim/isaac/sim_server.py     소켓 서버        <- A
    packing_sim/isaac/spawner.py        박스·픽업감지    <- A
    packing_sim/isaac/conveyor_ctl.py   벨트 속도        <- A
    scripts/verify_b4_cycle.MockServices  §2.7 Mock (/db/* /algo/*)

## 왜 `isaac-sim.sh` 인가

벨트가 `IsaacConveyor` OmniGraph 노드로 도는데, 그 노드는 렌더 틱에 물려 있다.
`python.sh` standalone 은 그래프를 안 돌리거나(=render False, 약 12.7 m/s)
프레임 delta 가 부풀어 틀리게 돌린다(=render True, 약 3 배). 실측 근거는
`packing_sim/isaac/conveyor_ctl.py` docstring 에 있다. 여기서는 박스가 사양 속도로
실려 와야 하므로 `isaac-sim.sh` 를 쓴다.

## 박스는 **씬의 스토퍼 벽**이 세운다 (2026-08-23)

드라이버가 벨트를 끊지 않는다. 씬(`layout_to_spec_test1_1.py`)이 stopper_x 에
콜라이더 벽을 세우고, 박스는 거기 닿아 선다. 그래서 여기 흐름이 실제 운용과 같다:

    spawn_box -> 벨트 on -> (벽이 세운다) -> pickup_check present -> Cycle.run

한때 벽을 지우고 "벨트를 멈춰서 세운다" 로 갔던 판이 있었는데, `pickup_check` 는
박스가 *이미 멎어야* present 를 주고 `cycle.py` 는 present 를 받은 *뒤에* 벨트를
세우므로 서로를 기다려 아무도 못 세웠다. 그때는 이 드라이버가 대신 끊었다.
"""
import json
import math
import os
import shutil
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from omni.physx import get_physx_interface

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src/packing_sim/isaac"))
sys.path.insert(0, str(ROOT / "src/packing_robot/packing_robot"))

import _paths                                            # noqa: E402
import import_h2017_usd as IMP                           # noqa: E402

STAGE = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")
PORT = int(os.environ.get("PL_PORT", "8799"))            # 8765 는 sim_main 것
ROBOT_PRIM = "/World/robot"
GRIPPER_PRIM = ROBOT_PRIM + "/SurfaceGripper"
EE_FRAME = "tool0"
ZONE = "B"
BOX_ID = "BOX00000"
BOX_TYPE = "4호"                                          # Mock get_box_info 와 같아야 한다
BOOT_STEPS = 20                                          # 물리가 자리 잡을 때까지
SETTLE_STEPS = 90                                        # READY 자세로 수렴 대기 (1.5 s)
TRAVEL_TIMEOUT_S = 90.0                                  # 9.5 m / 0.5 m/s = 19 s + 여유
WATCHDOG_S = 180.0

cfg = _paths.system_config()
STOPPER_X = cfg["conveyor"]["stopper_x_mm"] / 1000.0
BELT_MPS = float(cfg["conveyor"]["belt_velocity_mps"])
ORIGIN_MM = cfg["zones"]["pallet_origin_world_mm"][ZONE]
T4 = next(t for t in cfg["boxes"]["types"] if t["name"] == BOX_TYPE)

_S = {"phase": "boot", "n": 0, "rm": None, "sp": None, "cc": None,
      "q": None, "server": None, "link": None, "t0": time.time(),
      "result": None, "done": False}


def log(*a):
    print(*a, flush=True)


# ── 씬 열기 + 즉시 Play ─────────────────────────────────────
omni.usd.get_context().open_stage(STAGE)
stage = omni.usd.get_context().get_stage()
log("\n" + "=" * 70)
log("B4 pick & place 1 사이클 — 실제 씬")
log("=" * 70)
log("씬        %s" % STAGE)
log("존 %s     팔레트 원점 %s mm   스토퍼 x %.3f m   벨트 %.2f m/s"
    % (ZONE, ORIGIN_MM, STOPPER_X, BELT_MPS))
for p in (ROBOT_PRIM, GRIPPER_PRIM):
    ok = stage.GetPrimAtPath(p).IsValid()
    log("  %-28s %s" % (p, "있음" if ok else "!! 없음"))
    if not ok:
        log("  -> run_layout_to_spec.sh 를 먼저 돌려라.")
        omni.kit.app.get_app().post_quit()
        raise SystemExit
log("-" * 70)

omni.timeline.get_timeline_interface().play()


def setup():
    """물리가 돌기 시작한 뒤에 부른다 (아티큘레이션 초기화가 필요하다)."""
    import conveyor_ctl
    import mainq as MQ
    import robot_motion as RM
    import spawner as SP
    import sim_server as SS
    import isaacsim.robot_motion.motion_generation as mg
    from isaacsim.core.prims import SingleArticulation

    art = SingleArticulation(prim_path=ROBOT_PRIM, name="h2017")
    art.initialize()

    tmp = tempfile.mkdtemp(prefix="h2017_lula_")
    urdf, _n, missing = IMP.make_abs_urdf(tmp)
    if missing:
        raise RuntimeError("메시 없음: %s" % missing[:3])
    lula = mg.LulaKinematicsSolver(robot_description_path=str(_paths.DESCRIPTOR),
                                   urdf_path=str(urdf))
    pos, quat = art.get_world_pose()
    lula.set_robot_base_pose(np.array(pos), np.array(quat))
    log("로봇      베이스 %s mm  EE %s"
        % ([round(float(v) * 1000) for v in pos], EE_FRAME))

    q = MQ.MainQueue()
    rm = RM.RobotMotion(art, lula, cfg, ee=EE_FRAME, gripper_path=GRIPPER_PRIM)
    sp = SP.Spawner(stage, cfg, q, zone=ZONE)
    cc = conveyor_ctl.ConveyorCtl(stage, cfg, q)

    server = SS.SimServer(port=PORT)
    server.register({"ping": lambda r: {"ok": True}})
    server.register(sp.handlers())
    server.register(cc.handlers())
    server.register(rm.handlers())
    log("소켓      127.0.0.1:%d   cmd %s" % (PORT, server.cmds()))

    # ── READY 자세로 순간이동 ────────────────────────────────
    # ★ 이게 없으면 첫 이동에서 진다. 씬의 로봇은 zero pose(팔이 수직으로 선 자세)로
    #   들어와 있어서, 픽 접근점까지 관절이 179 도를 돌아야 한다. `cycle.py` 의 한 구간
    #   예산(MOTION_TIMEOUT_S 20 s)을 그냥 넘겨 MOTION_TIMEOUT 이 난다
    #   (실측 2026-08-22: "move -> [9.846, -0.001, 1.280] |dq| 179.1 deg n 102" -> BLOCKED).
    #   `verify_b4_cycle.py` 도 사이클 전에 READY 로 보낸다 (verify_lula_ik.teleport).
    #   READY = descriptor 의 default_q. `robot_motion` 의 IK 웜스타트도 이 자세 기준이다.
    import yaml
    from isaacsim.core.utils.types import ArticulationAction
    ready = np.array(yaml.safe_load(_paths.DESCRIPTOR.read_text())["default_q"], dtype=float)
    art.set_joint_positions(ready)
    art.set_joint_velocities(np.zeros(6))
    _S["ready_action"] = ArticulationAction(joint_positions=ready)
    log("자세      READY 로 순간이동 %s deg"
        % np.round(np.degrees(ready), 1).tolist())

    _S.update(rm=rm, sp=sp, cc=cc, q=q, server=server, art=art)
    cc.stop_all()


def worker():
    """머리 쪽. 소켓으로만 말한다 — 물리 객체를 직접 안 만진다."""
    import cycle as CY
    import sim_link as SL
    from verify_b4_cycle import MockServices

    try:
        link = SL.SimLink(port=PORT)
        _S["link"] = link
        log("-" * 70)

        # ── 1) 박스를 라인에 투입하고 사양 속도로 흘린다 ──────
        log("[1] 박스 투입 + 벨트 %0.2f m/s" % BELT_MPS)
        r = link.call(cmd="spawn_box", box_id=BOX_ID, type=BOX_TYPE, yaw_deg=0)
        if not r.get("ok"):
            raise RuntimeError("spawn_box 실패: %r" % r)
        link.call(cmd="set_velocity", seg="main", mps=BELT_MPS)
        link.call(cmd="set_velocity", seg="b_pickup", mps=BELT_MPS)

        # ── 2) 벽이 세울 때까지 기다린다. **벨트는 안 끈다** ──
        # 9.5 m 를 0.5 m/s 로 오므로 19 s + 정지 판정. 넉넉히 본다.
        half_w = T4["w_mm"] / 2000.0
        pick = None
        t0 = time.time()
        while time.time() - t0 < TRAVEL_TIMEOUT_S:
            c = link.call(cmd="pickup_check", zone=ZONE)
            if c.get("present"):
                pick = c
                break
            time.sleep(0.05)
        if pick is None:
            p = link.call(cmd="get_box_pose", box_id=BOX_ID)
            raise RuntimeError("pickup_check 가 present 를 안 준다 (박스 %s)"
                               % (np.round(p.get("pos_m", []), 3).tolist()))
        travel = time.time() - t0
        log("[2] 스토퍼 도달 %.1f s — 박스 앞면 %.4f m (스토퍼 %.3f, 오차 %+.1f mm)"
            % (travel, pick["pose"]["pos_m"][0] + half_w, STOPPER_X,
               (pick["pose"]["pos_m"][0] + half_w - STOPPER_X) * 1000.0))
        log("[3] pickup_check present  box=%s  pose=%s"
            % (pick["box_id"], {k: (np.round(v, 4).tolist() if isinstance(v, list) else round(v, 2))
                                for k, v in pick["pose"].items()}))

        # ── 4) 사이클 한 바퀴 — **실물 cycle.py 그대로** ───────
        svc = MockServices(log=lambda *a: log("            ", *a))
        cyc = CY.Cycle(link, svc, cfg, zone=ZONE, log=lambda *a: log("   [cycle]", *a))
        log("[4] Cycle.run 시작 (§2.5 정상 시퀀스)")
        t0 = time.time()
        res = cyc.run(pick["box_id"], pick["pose"])
        wall = time.time() - t0

        # ── 5) 결과 ───────────────────────────────────────────
        end = link.call(cmd="get_box_pose", box_id=BOX_ID)
        want = CY.pallet_place_to_world(svc.place, T4["h_mm"], ORIGIN_MM)
        got = np.asarray(end["pos_m"], dtype=float)
        err = (got - np.asarray(want)) * 1000.0
        _S["result"] = {"res": res, "wall": wall, "got": got, "want": want,
                        "err": err, "calls": [c[0] for c in svc.calls]}
    except BaseException as e:
        import traceback
        _S["result"] = {"res": "EXCEPTION", "trace": traceback.format_exc(), "exc": e}
    finally:
        _S["done"] = True


def report():
    r = _S.get("result") or {"res": "TIMEOUT"}
    log("-" * 70)
    if r["res"] == "EXCEPTION":
        log("!! 예외\n" + r["trace"])
        ok = False
    else:
        log("사이클 결과   %s   (%.1f s 벽시계)" % (r["res"], r.get("wall", 0.0)))
        if "got" in r:
            log("박스 최종     %s m" % np.round(r["got"], 4).tolist())
            log("목표          %s m" % [round(v, 4) for v in r["want"]])
            log("배치 오차     [%+.1f %+.1f %+.1f] mm" % tuple(r["err"]))
        need = ["/conveyor/set_velocity", "/db/get_box_info", "/algo/plan_placement",
                "/db/set_status", "/algo/update_heightmap", "/db/mark_placed"]
        missing = [n for n in need if n not in r.get("calls", [])]
        log("서비스 호출   %d 회%s"
            % (len(r.get("calls", [])), "" if not missing else "   !! 빠진 것 %s" % missing))
        placed = r["res"] == "PLACED"
        on_target = ("err" in r and float(np.linalg.norm(r["err"][:2])) < 30.0
                     and abs(r["err"][2]) < 20.0)
        ok = placed and on_target and not missing
        for cond, msg in ((placed, "PLACED 로 끝나야 한다 — %s" % r["res"]),
                          (on_target, "배치 오차 XY 30 mm · Z 20 mm 안"),
                          (not missing, "명세서 §2.5 서비스가 전부 불려야 한다")):
            if not cond:
                log("  !! %s" % msg)
    log("-" * 70)
    log("RESULT: %s" % ("통과" if ok else "실패"))
    log("=" * 70 + "\n")
    try:
        if _S.get("link"):
            _S["link"].close()
        if _S.get("server"):
            _S["server"].close()
    except Exception:
        pass
    omni.timeline.get_timeline_interface().stop()
    omni.kit.app.get_app().post_quit()


def on_step(dt):
    if _S["phase"] == "done":
        return
    if _S["phase"] == "boot":
        _S["n"] += 1
        if _S["n"] < BOOT_STEPS:
            return
        try:
            setup()
        except BaseException:
            import traceback
            log("!! setup 실패\n" + traceback.format_exc())
            _S["phase"] = "done"
            report()
            return
        _S["phase"] = "settle"
        _S["n"] = 0
        return
    if _S["phase"] == "settle":
        # 드라이브 목표를 계속 걸어 줘야 READY 에 머문다 (안 걸면 옛 목표로 튄다).
        _S["n"] += 1
        _S["art"].apply_action(_S["ready_action"])
        if _S["n"] >= SETTLE_STEPS:
            q = _S["art"].get_joint_positions()
            log("자세      수렴 %s deg" % np.round(np.degrees(q[:6]), 1).tolist())
            log("-" * 70)
            _S["phase"] = "run"
            _S["t0"] = time.time()
            threading.Thread(target=worker, daemon=True, name="b4-cycle").start()
        return
    _S["q"].pump()
    _S["rm"].step()
    _S["sp"].step()
    if _S["done"] or time.time() - _S["t0"] > WATCHDOG_S:
        _S["phase"] = "done"
        report()


_sub = get_physx_interface().subscribe_physics_step_events(on_step)
log("Play — 부팅 대기...\n")
