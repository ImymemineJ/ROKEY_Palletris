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
# ★ 로봇 프림. 씬에 따라 다르다 (2026-08-26 김병직):
#     test1-1 (1 구역)  /World/robot
#     test_3  (3 구역)  /World/robot_A · robot_B · robot_C
#   `demo_stack15_test1_1.py` 가 이미 같은 규약(`PL_ROBOT_FMT`)을 쓴다 — 맞춘다.
ROBOT_PRIM = os.environ.get("PL_ROBOT_PRIM", "/World/robot")
ROBOT_FMT = os.environ.get("PL_ROBOT_FMT", "/World/robot_%s")


def robot_prim(zone, multi):
    """구역 `zone` 의 로봇 prim. 구역이 하나면 `PL_ROBOT_PRIM` 을 그대로 쓴다."""
    return (ROBOT_FMT % str(zone).upper()) if multi else ROBOT_PRIM


def gripper_prim(zone, multi):
    return robot_prim(zone, multi) + "/SurfaceGripper"
EE_FRAME = "tool0"
SETTLE_STEPS = 90                  # READY 수렴 (60 Hz 에서 1.5 s). 게이트 판과 같다


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default=DEFAULT_STAGE)
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    # ★ 로봇 여러 대를 동시에 돌린다 (2026-08-26). `--zones A,B,C` 로 켠다.
    #   안 주면 `--zone` 하나뿐이라 기존 동작이 하나도 안 바뀐다 (test1-1 기준선).
    #   `demo_stack15_test1_1.py` 의 `PL_ZONES` 와 같은 규약이다.
    ap.add_argument("--zones", default=None,
                    help="구역 여럿 — 예: A,B,C (test_3 씬). 안 주면 --zone 하나")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--gui", action="store_true", help="창을 띄운다")
    ap.add_argument("--no-robot", action="store_true",
                    help="로봇 없이 씬만 (벨트·스포너 시험용)")
    # ★ ROS 2 통로를 연다 (CLAUDE.md §2 — Isaac 안팎을 잇는 통로는 ROS 2 하나뿐).
    #   켜면 Action Graph 가 /zone_<z>/robot/cmd 를 구독하고 씬 서비스 4종을 낸다.
    #   끄면 예전처럼 소켓만 연다 (demo_stack15 · 과도기 robot_node 용).
    ap.add_argument("--ros", action="store_true",
                    help="ROS 2 Action Graph 를 세운다 (관제탑과 직접 대화)")
    ap.add_argument("--autorun", action="store_true",
                    help="뜨자마자 벨트를 사양 속도로 돌린다")
    ap.add_argument("--no-render", action="store_true",
                    help="렌더를 끈다. ★ 벨트가 안 돈다 — 아래 주석 참조")
    # ★ 급송 (2026-08-26 김병직). 기본 0 = **꺼짐** — 지금까지 동작 그대로다.
    #   왜 필요한가: `--ros` 경로에는 **박스를 넣는 주체가 아무도 없다.**
    #   `spawn_box` 는 요청형 핸들러뿐이고 §3-3 계약에 스폰 서비스가 없다
    #   (CLAUDE.md flow 1 번 "박스 스폰 — 씬" 이 아직 씬에 없다). 그래서 전 구간을
    #   띄워도 관제탑이 IDLE 로 영원히 앉아 있는다. 씬이 스스로 낼 때까지의 임시다.
    ap.add_argument("--spawn-period", type=float, default=0.0, metavar="S",
                    help="S 초마다 박스 하나 투입 (0 = 안 넣는다)")
    ap.add_argument("--spawn-count", type=int, default=0, metavar="N",
                    help="총 N 개까지만 (0 = 무제한)")
    a = ap.parse_args()

    import _paths                                       # noqa: E402
    _paths.check()
    cfg = _paths.system_config()

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})

    server = None
    rms = {}                     # {zone: RobotMotion}. 예외로 빠져도 finally 가 안 죽게
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

        # ★ ROS 쪽(ros_side)도 같은 핸들러를 써야 하므로 한 표에 모아 둔다.
        #   소켓과 ROS 가 서로 다른 함수를 부르면 둘이 어긋난다.
        H = {"ping": lambda req: {"ok": True}}

        ZONES = [z.strip().upper() for z in (a.zones or a.zone).split(",") if z.strip()]
        MULTI = len(ZONES) > 1
        print("[sim_main] 구역 %s%s" % (", ".join(ZONES), " (다중)" if MULTI else ""))

        sp = SP.Spawner(stage, cfg, q, zone=ZONES[0], zones=ZONES)
        H.update(sp.handlers())
        server.register(sp.handlers())

        cc = conveyor_ctl.ConveyorCtl(stage, cfg, q)
        H.update(cc.handlers())
        server.register(cc.handlers())

        # ── 로봇 (B 소유 핸들러) ─────────────────────────────
        # ★ 지금 집으려는 박스. 워커가 없는 정본 서버에서는 `grasp` 요청이 곧
        #   그 신호다. B 코드를 고치지 않고 **등록할 때 감싸서** 채운다.
        # ★★ 구역마다 **핸들러 표를 따로 만든다** (2026-08-26). `move_ee`·`grasp` 는
        #   이름에 구역이 없어서 한 표에 몰면 마지막 로봇이 앞의 것을 덮는다 —
        #   A 구역 명령이 C 구역 팔을 움직인다. 소켓은 예전처럼 **첫 구역** 것만
        #   등록한다 (소켓 규약에 구역 칸이 없다. ROS 를 쓰면 구역이 갈린다).
        cur = {"box": None}
        HZ = {}                       # {zone: 그 구역 핸들러 표}
        if not a.no_robot:
            for zi, z in enumerate(ZONES):
                rm_z = _bring_up_robot(world, stage, cfg, z, spawner=sp, cur=cur,
                                       prim_path=robot_prim(z, MULTI),
                                       grip_path=gripper_prim(z, MULTI))
                if rm_z is None:
                    continue
                rms[z] = rm_z
                _h = dict(rm_z.handlers())
                _grasp = _h["grasp"]

                def h_grasp(req, _f=_grasp):
                    bid = (req or {}).get("box_id")
                    rec = sp.boxes.get(bid)
                    cur["box"] = (bid, rec["dims"][2] / 2.0) if rec else None
                    try:
                        return _f(req)
                    finally:
                        cur["box"] = None   # 사이클 밖에서 옛 박스로 재지 않는다

                _h["grasp"] = h_grasp
                HZ[z] = dict(H, **_h)     # 씬 핸들러 + 그 구역 로봇 핸들러
                if zi == 0:
                    H.update(_h)
                    server.register(_h)
        for z in ZONES:
            HZ.setdefault(z, H)           # 로봇이 없는 구역도 씬 서비스는 답한다

        # ── ROS 2 통로 (트랙 B — ros_side.py) ────────────────
        # ★ build() 는 **Play 전**, wire() 는 **Play + 몇 프레임 뒤**다.
        #   메시지 필드가 Play 후에야 생겨서 그 전에 이으면 죽는다 (TS-022 2번).
        # ★ 그래프는 하나, 서비스는 한 벌, 구역별 `RosSide` 는 step() 만 각자 돈다.
        ros = []
        if a.ros:
            for _e in ("isaacsim.core.nodes", "isaacsim.ros2.bridge",
                       "omni.graph.scriptnode"):
                enable_extension(_e)
            kit.update()
            import ros_side
            ros_side.install_services(H)
            ros_side.build(ZONES, cfg=cfg)   # cfg -> /clock·/tf 도 (씬에 없을 때만)
            ros = [ros_side.RosSide(cfg, z, HZ[z]) for z in ZONES]

        world.reset()
        if rms:
            # ★ READY 로 순간이동한 뒤 **정착시킨다.** 이게 없으면 씬의 로봇이
            #   zero pose(팔이 수직으로 선 자세)로 남아, 첫 이동이 179 도를 돌아야
            #   해서 구간 예산(20 s)을 넘겨 MOTION_TIMEOUT 이 난다
            #   (`verify_b4_test1_1.py` 의 실측). 게이트 스크립트에는 있는데
            #   정본 서버에만 빠져 있었다.
            # ★★ 로봇 **전부**를 같은 스텝에서 함께 정착시킨다 (2026-08-26).
            #   한 대씩 SETTLE_STEPS 를 돌리면 앞 로봇이 목표를 안 받는 동안
            #   옛 목표로 튄다 (바로 아래 apply_action 주석과 같은 이유).
            import numpy as np
            import yaml
            from isaacsim.core.utils.types import ArticulationAction
            _ready = np.array(
                yaml.safe_load(_paths.DESCRIPTOR.read_text())["default_q"], dtype=float)
            _act = ArticulationAction(joint_positions=_ready)
            for _z, _rm in rms.items():
                _rm.art.initialize()
                _rm.art.set_joint_positions(_ready)
                _rm.art.set_joint_velocities(np.zeros(6))
            for _ in range(SETTLE_STEPS):
                for _rm in rms.values():
                    _rm.art.apply_action(_act)     # 안 걸면 옛 목표로 튄다
                world.step(render=not a.no_render)
            for _z, _rm in rms.items():
                print("[sim_main] %s 자세 READY 수렴 %s deg"
                      % (_z, np.round(np.degrees(
                          _rm.art.get_joint_positions()[:6]), 1).tolist()))
        # 벨트는 **멈춘 채로** 시작한다. 오케스트레이터가 set_velocity 로 돌린다.
        # (뜨자마자 돌면 머리가 붙기 전에 박스가 라인 끝까지 흘러가 버린다.)
        if ros:
            # 정착 스텝(SETTLE_STEPS)에서 이미 프레임이 돌았지만, 로봇이 없으면
            # (--no-robot) 한 번도 안 돌았을 수 있어 여기서 확실히 채운다.
            for _ in range(60):
                world.step(render=not a.no_render)
            import ros_side
            ros_side.wire()

        feeder = None
        if a.spawn_period > 0:
            feeder = _Feeder(cfg, sp, a.spawn_period, a.spawn_count)
            print("[sim_main] 급송 %.1f 초 간격 · %s 개"
                  % (a.spawn_period, a.spawn_count or "무제한"))

        cc.run_all() if a.autorun else cc.stop_all()
        print("[sim_main] 벨트 %s" % ("가동" if a.autorun else "정지 — set_velocity 대기"))
        print("[sim_main] cmd %s" % server.cmds())
        print("[sim_main] 준비 완료. 소켓 127.0.0.1:%d%s"
              % (a.port, " · ROS 2 켬 (구역 %d)" % len(ros) if ros else ""))

        # ── 메인 루프: 물리만 돌린다 ─────────────────────────
        while kit.is_running():
            q.pump()                       # 소켓 스레드가 맡긴 USD 작업
            for _rm in rms.values():
                _rm.step()                 # ★ world.step() 앞. 로봇 전부
            sp.step()
            if feeder is not None:
                feeder.step(time.time())
            for _r in ros:
                _r.step()              # ★ 논블로킹. 모션은 워커가 돈다 (§9)
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
    except Exception:
        # ★ 여기서 직접 찍는다. 아래 `kit.close()` 가 **프로세스를 끝내 버려서**
        #   파이썬이 traceback 을 찍을 기회를 못 얻는다 — 화면에는
        #   "Simulation App Shutting Down" 한 줄만 남고 종료코드도 0 일 때가 있다.
        #   원인을 못 보고 씬·오버레이·경로를 헤매게 되는 종류의 침묵이다
        #   (2026-08-26 실측).
        import traceback
        print("\n[sim_main] !! 예외로 종료한다 ------------------------")
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        if server is not None:
            server.close()
        kit.close()


class _Feeder:
    """벨트에 박스를 하나씩 넣는다. **메인 스레드에서만** 돈다.

    ★ 이건 `demo_stack15_test1_1.py` 의 급송 스레드를 옮겨 온 것이 아니다.
      저쪽은 소켓으로 부르고 백압 판단이 훨씬 두껍다. 여기서는 같은 프로세스라
      핸들러를 **직접** 부른다 — 소켓도, 스레드도 안 쓴다 (§2 · §9).

    ★ 박스 구성은 `demo_stack15.manifest` 와 **같은 방식**으로 뽑는다
      (`np.random.default_rng(random_seed).choice`). 그 모듈을 import 하면
      Isaac 배선 스크립트가 줄줄이 딸려 와서 다섯 줄을 옮겨 적었다.
      ⚠ `random.Random(seed).choices` 로 바꾸면 **같은 seed 라도 결과가 다르다.**

    ★ 백압 — 라인에 이미 `cap` 개가 있으면 안 넣는다. 사이클이 투입 간격보다
      길면 큐가 계속 길어지고, 벨트가 뒤에서 밀어 박스끼리 눌리다 솔버가 터진다
      (2026-08-23 BOX00014 실측: 초속 수 km 로 날아갔다).
    """

    def __init__(self, cfg, sp, period, total, cap=3, log=print):
        import numpy as np
        bx = cfg["boxes"]
        rng = np.random.default_rng(int(bx.get("random_seed", 42)))
        pr = np.array(bx["size_distribution"], dtype=float)
        pr = pr / pr.sum()
        n = int(total) if total > 0 else 200
        self.types = [bx["types"][int(i)]["name"]
                      for i in rng.choice(len(bx["types"]), n, p=pr)]
        # 배송지는 크기와 **독립**으로 뽑는다 — 한 존에 3·4·5 호가 섞여야
        # 패커가 실제로 일한다 (demo_stack15_test1_1 의 근거 그대로).
        import random as _r
        dests = list(bx.get("dests") or [])
        dr = _r.Random(int(bx.get("random_seed", 42)))
        self.dests = [dr.choice(dests) for _ in range(n)] if dests else [None] * n
        self.h_spawn = sp.handlers()["spawn_box"]
        self.h_stat = sp.handlers()["spawn_status"]
        self.period, self.total, self.cap, self.log = float(period), int(total), int(cap), log
        self.i, self.next_t = 0, 0.0

    def step(self, now):
        if self.total and self.i >= self.total:
            return
        if self.i >= len(self.types) or now < self.next_t:
            return
        st = self.h_stat({}) or {}
        # 스폰 지점이 아직 안 비었으면 **겹쳐 스폰된다.** 다음 틱에 다시 본다.
        if st.get("on_belt", 0) >= self.cap or not st.get("spawn_clear", True):
            return
        bid = "BOX%05d" % self.i
        kw = {"box_id": bid, "type": self.types[self.i], "yaw_deg": 0}
        if self.dests[self.i]:
            kw["dest"] = self.dests[self.i]
        r = self.h_spawn(kw) or {}
        if r.get("ok"):
            self.i += 1
            self.next_t = now + self.period
            self.log("[투입] %s %s %s (라인 위 %s)"
                     % (bid, kw["type"], kw.get("dest", "-"),
                        (self.h_stat({}) or {}).get("on_belt", "?")))
        else:
            self.next_t = now + 1.0          # 실패는 조용히 재시도 (라인이 붐빈다)


def _bring_up_robot(world, stage, cfg, zone, spawner=None, cur=None,
                    prim_path=None, grip_path=None):
    """씬에 이미 있는 로봇 프림에 Lula 와 그리퍼를 물린다.

    씬에 로봇을 **넣지 않는다** — `layout_to_spec_test1_1.py` 가 참조로 넣어 뒀다.
    여기서 또 넣으면 두 벌이 겹친다.

    `spawner`·`cur` 를 주면 흡착 간극 실측(`gap_fn`)까지 배선한다. 안 주면
    예전처럼 상태만 보고 판정한다 — 게이트 스크립트에는 있는데 **정본 서버에만
    빠져 있던** 것이라 오케스트레이터가 붙는 순간 드러난다.
    """
    import numpy as np

    # 구역별로 prim 이 다르다 (test_3 은 robot_A/B/C). 안 주면 예전 기본값.
    prim_path = prim_path or ROBOT_PRIM
    grip_path = grip_path or (prim_path + "/SurfaceGripper")

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print("[sim_main] !! %s 가 씬에 없다 — 로봇 없이 간다.\n"
              "   run_layout_to_spec.sh 를 먼저 돌려라." % prim_path)
        return None
    if not stage.GetPrimAtPath(grip_path).IsValid():
        print("[sim_main] !! %s 가 없다 — 씬의 로봇 USD 에 흡착 그리퍼가 없다.\n"
              "   ~/isaac_assets/h2017_suction.usd 를 참조하도록\n"
              "   run_layout_to_spec.sh 를 다시 돌려라 (없으면\n"
              "   scripts/build_gripper_usd.py 로 만든다). 로봇 없이 간다."
              % grip_path)
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

    art = SingleArticulation(prim_path=prim_path, name="h2017_%s" % str(zone).upper())
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

    def _gap_mm():
        """흡착면~박스 윗면 간극 [mm]. 못 재면 None.

        ★ 이걸 안 주면 `gripper._grasp_verdict` 의 간극 검사(GRIP_TOL_MM 3 mm)가
          통째로 건너뛰어진다. 그러면 "is_holding 이 참이고 물린 프림 이름이 맞다"
          까지만 보고 성공으로 친다 — 박스가 몇 mm 떠서 매달려도 모른다.
          `cycle.py` 의 이송·배치 높이는 전부 "딱 붙었다" 를 가정하므로 매달린 만큼
          밑면이 계산보다 낮게 내려가 이웃 상면을 긁는다.
          `verify_suction.measure` 와 같은 식으로 툴 +Z 축에 투영한다 —
          `rm.suction_pose()` 는 위치만 주고 축을 안 줘서 여기 못 쓴다.
        rm.step() -> poll_grasp -> _grasp_verdict 경로라 **메인 스레드**다.
        """
        if spawner is None or cur is None or cur.get("box") is None:
            return None
        bid, half_h = cur["box"]
        e = spawner._snap.get(bid)
        if e is None:
            return None
        q6 = np.asarray(art.get_joint_positions()[:6], dtype=float)
        fk_p, fk_R = lula.compute_forward_kinematics(EE_FRAME, q6)
        zax = np.asarray(fk_R)[:, 2]
        face = np.asarray(fk_p, dtype=float) + zax * rm.grip_len
        c = np.asarray(e["pos_m"], dtype=float)
        return (float(np.dot(c - face, zax)) - half_h) * 1000.0

    import robot_motion as RM
    rm = RM.RobotMotion(art, lula, cfg, ee=EE_FRAME, gripper_path=grip_path,
                        gap_fn=_gap_mm)
    print("[sim_main] 로봇 %s  베이스 %s mm  EE %s  그리퍼 %s"
          % (prim_path, got_mm, EE_FRAME, grip_path))
    return rm


if __name__ == "__main__":
    main()
