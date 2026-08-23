#!/usr/bin/env python3
"""PC1 Isaac 프로세스 — 씬을 열고 소켓 명령 서버를 띄운다 (공통가이드 §3). **김학민 소유.**

    ~/isaacsim/python.sh src/packing_sim/isaac/sim_main.py
    ~/isaacsim/python.sh src/packing_sim/isaac/sim_main.py --gui
    ~/isaacsim/python.sh src/packing_sim/isaac/sim_main.py --gui --autorun

여기가 "몸"이다. "머리"(ROS 노드 2개)는 소켓으로 붙는다:

    ros2 run packing_sim sim_bridge_node      # A. /conveyor/* /sim/* <-> 소켓
    ros2 run packing_robot orchestrator       # B. cycle.py 를 돌린다

머리가 아직 없어도 이 프로세스만으로 씬은 돈다. 손으로 찔러 보려면:

    python3 -c "
    import json, socket
    f = socket.create_connection(('127.0.0.1', 8765)).makefile('rw')
    def call(**r):
        f.write(json.dumps(r) + '\\n'); f.flush(); return json.loads(f.readline())
    print(call(cmd='ping'))
    print(call(cmd='spawn_box', box_id='BOX00000', type='4호', yaw_deg=0))
    print(call(cmd='pickup_check', zone='B'))
    "

## 등록되는 cmd (공통가이드 §3 프로토콜 표 전부)

    김학민  ping · spawn_box · spawn_status · set_spawn_enabled · despawn_box
            pickup_check · get_box_pose · set_velocity
    김병직  move_ee · motion_status · grasp · release · is_holding · tcp_speed
            (`packing_sim/isaac/robot_motion.py` — B 소유 파일이다. 여기서는 등록만 한다)

## ★ 스레드 배치 — 이 파일에서 제일 중요한 것

    메인 스레드 : q.pump() -> rm.step() -> sp.step() -> world.step()   물리·USD 쓰기
    소켓 스레드 : 명령 핸들러                                          연결마다 하나

`grasp` / `release` 는 판정이 날 때까지 **소켓 스레드만** 기다린다. 그동안 물리는
계속 돌아야 결과가 나온다. 그래서 트랙A 가이드의 단일 스레드 `poll()` 골격을
쓰지 않는다 — 그대로 만들면 서로를 기다려 100% 데드락이다. 근거는
`sim_server.py` docstring 에 적어 뒀다.

`rm.step()` 은 **`world.step()` 앞**에서 부른다 (robot_motion docstring). 이번 스텝에
쓸 드라이브 목표를 거기서 걸기 때문이다. 순서를 뒤집으면 한 스텝씩 늦게 따라간다.

## 씬은 test1-1.usd 다

배치(페데스탈·팔레트·반출·로봇)는 `layout_to_spec_test1_1.py` 가 이미 해 뒀다.
여기서는 **씬을 고치지 않는다** — 열고, 물린다. 배치를 바꿀 일이 있으면
`run_layout_to_spec.sh` 를 돌리고 다시 뜬다.
"""
import argparse
import math
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent                 # src/packing_sim/isaac
ROOT = HERE.parent.parent.parent                       # ~/palletris_ws
sys.path.insert(0, str(HERE))                          # robot_motion, spawner, ...
sys.path.insert(0, str(ROOT / "scripts"))              # _paths, import_h2017_usd
sys.path.insert(0, str(ROOT / "src/packing_robot/packing_robot"))   # gripper

DEFAULT_STAGE = os.environ.get(
    "PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")
# 흡착 그리퍼가 붙은 로봇 판이 씬에 참조돼 있어야 한다 (layout_to_spec 참조).
ROBOT_PRIM = "/World/robot"
GRIPPER_PRIM = ROBOT_PRIM + "/SurfaceGripper"
EE_FRAME = "tool0"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default=DEFAULT_STAGE)
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--gui", action="store_true", help="창을 띄운다")
    ap.add_argument("--no-robot", action="store_true",
                    help="로봇 없이 씬만 (벨트·스포너 시험용)")
    ap.add_argument("--autorun", action="store_true",
                    help="뜨자마자 벨트를 사양 속도로 돌린다")
    ap.add_argument("--no-render", action="store_true",
                    help="렌더를 끈다. ★ 벨트가 안 돈다 — 아래 주석 참조")
    a = ap.parse_args()

    import _paths                                       # noqa: E402
    _paths.check()
    cfg = _paths.system_config()

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})

    rm = server = None
    try:
        from isaacsim.core.api import World
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.stage import open_stage

        # ★★ 컨베이어 확장을 **씬을 열기 전에** 켠다.
        #
        # 안 켜면 OmniGraph 가 `IsaacConveyor` 노드 타입을 못 찾고
        # ("Could not find node type interface for
        #   'isaacsim.asset.gen.conveyor.IsaacConveyor'") 그래프가 통째로 죽는다.
        # 그러면 `graph:variable:Velocity` 를 아무리 써넣어도 아무 일도 안 일어나고,
        # 벨트는 USD 에 **남아 있던 옛 `physxSurfaceVelocity`** 로 돈다.
        #
        # 실측 2026-08-22 — 이걸 안 켜고 돌렸을 때:
        #     0.00s  x= 0.500  z=0.950   (스폰)
        #     0.12s  x= 6.088  z=0.940   <- 0.12 초에 5.6 m = 약 46 m/s
        #     0.36s  x=10.443  z=0.615   <- 라인 끝으로 튕겨 나가 정지
        # `set_velocity` 는 ok=true 를 돌려주는데 벨트는 안 듣는다. GUI(isaac-sim.sh)
        # 에서는 확장이 기본으로 켜져 있어 **이 버그가 안 보인다** — standalone 에서만
        # 터지는 종류다.
        enable_extension("isaacsim.asset.gen.conveyor")

        hz = float(cfg["physics"]["scene"]["time_steps_per_second"])
        if not Path(a.stage).exists():
            sys.exit("씬이 없다: %s" % a.stage)
        open_stage(a.stage)
        world = World(stage_units_in_meters=1.0,
                      physics_dt=1.0 / hz, rendering_dt=1.0 / hz)
        # ★ World() 를 sim_params 없이 부르면 PhysicsContext 가 기존 /physicsScene 에
        #   **자기 기본값을 덮어쓴다** (stabilization off · CPU 파이프라인이면 ccd on).
        #   씬 USD 에 뭘 써 넣든 매 기동마다 되살아나므로 여기서 사양값으로 되돌린다.
        #   enable_ccd 는 sim_params 키로 못 준다 — 명시 호출이어야 한다.
        _ps = cfg["physics"]["scene"]
        _pc = world.get_physics_context()
        _pc.enable_stablization(bool(_ps["enable_stabilization"]))   # 메서드명 오타는 Isaac 쪽
        _pc.enable_ccd(bool(_ps["enable_ccd"]))
        print("[sim_main] 물리 안정화 %s · CCD %s · %.0f Hz"
              % (_ps["enable_stabilization"], _ps["enable_ccd"], hz))
        stage = world.stage
        print("[sim_main] 씬 %s" % a.stage)

        import conveyor_ctl
        import mainq as MQ
        import spawner as SP
        import sim_server as SS

        # 소켓 스레드가 USD 를 직접 못 건드리게 한다 (mainq.py docstring).
        q = MQ.MainQueue(timeout_s=float(cfg["service"]["timeout_s"]) * 0.8)

        server = SS.SimServer(port=a.port)
        server.register({"ping": lambda req: {"ok": True}})

        sp = SP.Spawner(stage, cfg, q, zone=a.zone)
        server.register(sp.handlers())

        cc = conveyor_ctl.ConveyorCtl(stage, cfg, q)
        server.register(cc.handlers())

        # ── 로봇 (B 소유 핸들러) ─────────────────────────────
        if not a.no_robot:
            rm = _bring_up_robot(world, stage, cfg, a.zone)
            if rm is not None:
                server.register(rm.handlers())

        world.reset()
        if rm is not None:
            rm.art.initialize()
        # 벨트는 **멈춘 채로** 시작한다. 오케스트레이터가 set_velocity 로 돌린다.
        # (뜨자마자 돌면 머리가 붙기 전에 박스가 라인 끝까지 흘러가 버린다.)
        cc.run_all() if a.autorun else cc.stop_all()
        print("[sim_main] 벨트 %s" % ("가동" if a.autorun else "정지 — set_velocity 대기"))
        print("[sim_main] cmd %s" % server.cmds())
        print("[sim_main] 준비 완료. 소켓 127.0.0.1:%d" % a.port)

        # ── 메인 루프: 물리만 돌린다 ─────────────────────────
        while kit.is_running():
            q.pump()                       # 소켓 스레드가 맡긴 USD 작업
            if rm is not None:
                rm.step()                  # ★ world.step() 앞
            sp.step()
            # ★★ **헤드리스에서도 render=True 로 돈다.**
            #
            # `IsaacConveyor` 노드는 `OnTick` 으로 도는데 `OnTick` 은 **렌더 틱에**
            # 발화한다. `render=False` 면 그래프가 아예 안 돌아 벨트가 USD 에 남아
            # 있던 옛 표면속도로 간다 (실측 약 12.7 m/s, 요청과 무관).
            #
            # ⚠ 단, 렌더를 켜도 standalone 의 벨트 **속도는 못 믿는다** — 노드가
            #   프레임 delta 로 적분하는데 standalone 은 프레임이 실시간보다 느려서
            #   실측 약 1.5 m/s (사양 0.5 의 3 배) 가 나오고 실행마다 값이 다르다.
            #   **정지(0)는 정확하다.** 속도 검증은 `isaac-sim.sh` +
            #   `scripts/verify_belt_speed.py` 로 한다. conveyor_ctl docstring 참조.
            world.step(render=not a.no_render)
    except KeyboardInterrupt:
        print("\n[sim_main] 종료")
    finally:
        if server is not None:
            server.close()
        kit.close()


def _bring_up_robot(world, stage, cfg, zone):
    """씬에 이미 있는 로봇 프림에 Lula 와 그리퍼를 물린다.

    씬에 로봇을 **넣지 않는다** — `layout_to_spec_test1_1.py` 가 참조로 넣어 뒀다.
    여기서 또 넣으면 두 벌이 겹친다.
    """
    import numpy as np

    prim = stage.GetPrimAtPath(ROBOT_PRIM)
    if not prim or not prim.IsValid():
        print("[sim_main] !! %s 가 씬에 없다 — 로봇 없이 간다.\n"
              "   run_layout_to_spec.sh 를 먼저 돌려라." % ROBOT_PRIM)
        return None
    if not stage.GetPrimAtPath(GRIPPER_PRIM).IsValid():
        print("[sim_main] !! %s 가 없다 — 씬의 로봇 USD 에 흡착 그리퍼가 없다.\n"
              "   ~/isaac_assets/h2017_suction.usd 를 참조하도록\n"
              "   run_layout_to_spec.sh 를 다시 돌려라 (없으면\n"
              "   scripts/build_gripper_usd.py 로 만든다). 로봇 없이 간다."
              % GRIPPER_PRIM)
        return None

    import _paths
    import import_h2017_usd as IMP
    import isaacsim.robot_motion.motion_generation as mg
    from isaacsim.core.prims import SingleArticulation

    # Lula 는 URDF 로 기구학을 만든다. `package://` 를 못 읽어 절대경로 임시본을 쓴다.
    tmp = tempfile.mkdtemp(prefix="h2017_lula_")
    urdf, _n, missing = IMP.make_abs_urdf(tmp)
    if missing:
        shutil.rmtree(tmp, ignore_errors=True)
        print("[sim_main] !! 메시가 없다: %s — 로봇 없이 간다." % missing[:3])
        return None

    art = SingleArticulation(prim_path=ROBOT_PRIM, name="h2017")
    world.reset()
    art.initialize()

    lula = mg.LulaKinematicsSolver(robot_description_path=str(_paths.DESCRIPTOR),
                                   urdf_path=str(urdf))
    if EE_FRAME not in lula.get_all_frame_names():
        print("[sim_main] !! EE 프레임 %r 이 Lula 에 없다: %s"
              % (EE_FRAME, lula.get_all_frame_names()))
        return None
    pos, quat = art.get_world_pose()
    lula.set_robot_base_pose(np.array(pos), np.array(quat))     # ★ 반드시

    base_mm = cfg["zones"]["robot_base_world_mm"][zone]
    got_mm = [round(float(v) * 1000.0) for v in pos]
    if any(abs(g - s) > 1 for g, s in zip(got_mm, base_mm)):
        # 사양과 씬이 어긋나면 IK 는 풀리는데 박스가 엉뚱한 데 놓인다.
        # 조용히 넘어가면 원인을 찾는 데 하루가 든다.
        print("[sim_main] !! 로봇 베이스가 사양과 다르다: 씬 %s mm vs yaml %s mm"
              % (got_mm, base_mm))

    import robot_motion as RM
    rm = RM.RobotMotion(art, lula, cfg, ee=EE_FRAME, gripper_path=GRIPPER_PRIM)
    print("[sim_main] 로봇 %s  베이스 %s mm  EE %s  그리퍼 %s"
          % (ROBOT_PRIM, got_mm, EE_FRAME, GRIPPER_PRIM))
    return rm


if __name__ == "__main__":
    main()
