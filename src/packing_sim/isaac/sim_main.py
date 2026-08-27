#!/usr/bin/env python3
"""PC1 Isaac 프로세스 — 씬을 열고 명령을 받는다 (공통가이드 §3). **김학민 소유.**

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : **몸 전체** — 씬을 열고 ①~⑩ 이 일어날 무대를 돌린다
    ║     불러 주는 쪽        : scripts/run_test3_ros.sh
    ║     이 파일이 부르는 쪽 : spawner · conveyor_ctl · robot_motion · camera_pick · ros_side
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

═══════════════════════════════════════════════════════════════════════════
★ 최종 구조 (2026-08-26 이후) — 아래 원문보다 **이쪽이 지금 도는 경로**다
═══════════════════════════════════════════════════════════════════════════

`--ros` 를 주면 **Action Graph 가 직접 ROS 2 로 말한다.** 소켓을 안 거친다
(CLAUDE.md §2 가 폐기한 그 소켓이다). 최종 실행은 껍데기 하나로 끝난다:

    ./scripts/run_test3_ros.sh              # A,B,C · 창 · 자동 Play

    # 손으로 돌릴 때 (환경변수는 kit 이 **시작 때만** 읽는다 — 안에서 고치면 늦다)
    source install/setup.bash && export ROS_DOMAIN_ID=115
    PL_STAGE=~/isaac_scenes/project_1/test_3.usd \
    PL_CONFIG_OVERLAY=$PWD/src/packing_bringup/config/test_3.yaml \
    ~/isaac-sim-5.1.0/python.sh src/packing_sim/isaac/sim_main.py \
        --zones A,B,C --ros --autorun --gui --spawn-period 8

이 프로세스가 ROS 2 로 내는 것 / 받는 것 (`ros_side.py` + `ros_graph.py`):

    낸다 : /clock · /tf · 카메라 Image  ·  /zone_{a,b,c}/robot/event
    받는다: /zone_{a,b,c}/robot/cmd     ·  씬 서비스 4종
            (/conveyor/set_velocity · /sim/despawn_box
             /sim/set_spawn_enabled · /sim/swap_pallet)

판단은 **하나도 여기서 안 한다.** 어디에 놓을지는 algo 가, 언제 집을지는 관제탑이
정한다. 이 파일은 몸이고 머리는 Isaac 밖 rclpy 노드들이다.

⚠ **Isaac 내장 ROS 2 를 쓰면 안 된다.** `packing_msgs` 는 `/opt/ros/jazzy` 로
  빌드돼 있어 내장 배포판에 물리면 fastcdr ABI 가 달라 `symbol lookup error`
  또는 SEGFAULT 로 죽는다. `source install/setup.bash` 한 줄이 셋
  (`AMENT_PREFIX_PATH` · `LD_LIBRARY_PATH` · 배포판 일치)을 한꺼번에 잡는다.

⚠ `--autorun` 을 빠뜨리면 벨트가 안 돈다. `--no-render` 도 벨트를 멈춘다.
  **둘 다 에러 없이** 박스만 안 온다.

⚠ `PL_CONFIG_OVERLAY` 를 안 주면 **경고 없이 어긋난다** — 관제탑·algo 는 test1-1
  좌표로 계획하고 Isaac 은 test_3 좌표로 움직인다. 뜰 때 찍히는 "로봇 베이스"
  줄이 사양과 다르면 거기서 멈추고 볼 것.

─────────────────────────────────────────────────────────────────────────
아래는 소켓 시절의 원문이다. 소켓 경로(`--ros` 없이)는 데모 스크립트
(`demo_stack15_test1_1.py`)를 위해 **아직 살아 있다** — 지우지 말 것.
─────────────────────────────────────────────────────────────────────────

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


def _add_camera_lights(cfg, log=print):
    """`PL_LIGHT` — 카메라용 조명을 런타임에 추가한다. **기본 꺼짐.**

    ★ 왜 (2026-08-27 실측, 10.10.0.2): 배경(`full_warehouse`)이 들어온 뒤
      **cam1 이 모든 박스를 가장 작은 3호로 축소 등록**했다. 4호(높이 280)를
      3호(210)로 알면 로봇이 70 mm 낮은 곳을 목표로 내려가 박스에 부딪히고
      (잔차 83 mm) `IK_FAIL` 로 3회 실패 뒤 폐기된다 — 겉보기 증상은 "흡착이
      안 된다" 지만 실제로는 **닿지를 못한 것**이다.
      같은 코드가 배경을 끈 판(`PL_BG=0`)에서는 3·4·5호를 전부 정확히 등록했다.

      원인은 조명이다. 씬 조명이 `DistantLight` 하나뿐인데(돔 라이트 없음)
      창고 지붕·벽이 그 빛을 가려 박스가 그늘에 들어가고, 채도 기반 색 마스크가
      박스 일부만 잡아 실루엣이 작아진다.

    ★ 돔 라이트만으로는 부족할 수 있다 — 돔은 무한 원방 조명이라 **지붕이 있으면
      그것도 막힌다.** 그래서 카메라 **바로 위**(건물 안)에 국소 구광원을 같이 둔다.
      카메라는 아래를 보므로 그 위의 광원은 화면에 안 들어온다.

    사용: `PL_LIGHT=1` (기본 세기) · `PL_LIGHT=3000` (돔 세기 지정)
    """
    _v = os.environ.get("PL_LIGHT", "")
    if _v in ("", "0"):
        return 0
    try:
        import omni.usd as _ou
        from pxr import Gf, UsdGeom, UsdLux
        st = _ou.get_context().get_stage()
        dome_i = 1000.0 if _v == "1" else float(_v)

        made = []
        # ① 돔 라이트 — 지붕이 없거나 열린 쪽을 채운다
        if not any(p.GetTypeName() == "DomeLight" for p in st.Traverse()):
            d = UsdLux.DomeLight.Define(st, "/World/PalletrisDome")
            d.CreateIntensityAttr(dome_i)
            made.append("Dome(%.0f)" % dome_i)

        # ② 카메라 위 국소 조명 — 지붕 아래라도 확실히 밝힌다.
        #   카메라 리그 경로는 설정이 정본이다 (`cameras.*`). 리그 프림의
        #   translate 를 읽어 그 **1 m 위**에 광원을 둔다.
        cams = (cfg.get("cameras") or {})
        paths = list((cams.get("pick_prim") or {}).values())
        for k in ("spawn_prim", "line2_prim"):
            if cams.get(k):
                paths.append(str(cams[k]))
        lamp_i = float(os.environ.get("PL_LIGHT_LAMP", "300000"))
        for i, lens in enumerate(paths):
            rig = "/" + "/".join(str(lens).strip("/").split("/")[:2])
            prim = st.GetPrimAtPath(rig)
            if not (prim and prim.IsValid()):
                continue
            t = None
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    t = op.Get()
                    break
            if t is None:
                continue
            p = "/World/PalletrisLamp_%d" % i
            if st.GetPrimAtPath(p).IsValid():
                continue
            s = UsdLux.SphereLight.Define(st, p)
            s.CreateRadiusAttr(0.15)
            s.CreateIntensityAttr(lamp_i)
            UsdGeom.Xformable(s.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(t[0], t[1], t[2] + 1.0))     # 카메라 1 m 위 = 시야 밖
            made.append(rig.rsplit("/", 1)[-1])
        log("[sim_main] PL_LIGHT — 조명 추가: %s"
            % (", ".join(made) if made else "없음(이미 있음)"))
        return len(made)
    except Exception as e:                                  # noqa: BLE001
        log("[sim_main] !! PL_LIGHT 조명 추가 실패 — %r (그대로 진행)" % (e,))
        return 0


def main():
    """씬을 열고 → 로봇을 물리고 → (`--ros` 면) ROS 2 를 배선하고 → 틱을 돈다.

    한 틱의 순서가 중요하다 (뒤집으면 한 스텝씩 늦게 따라간다):

        q.pump()      소켓/워커 스레드가 맡긴 USD 작업을 **메인에서** 처리
        rm.step()     이번 스텝에 걸 관절 드라이브 목표 ★ world.step() 앞이어야 한다
        sp.step()     스포너·컨베이어·소터
        ros.step()    명령 회수 + 결과 발행 (**논블로킹**)
        world.step()  물리 1스텝

    ★ `ros.step()` 은 절대 블로킹하면 안 된다. 모션 판정을 내리는 `rm.step()` 도
      메인 스레드라, 여기서 결과를 기다리면 서로를 기다려 **100 % 데드락**이다.
      그래서 실제 모션은 `ros_side` 의 워커 스레드가 돌린다.
    """
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
    # ★ 라우팅 (2026-08-26 김병직). `--ros` 경로에는 박스를 **꺾는 주체가 없다** —
    #   소터를 돌리는 것과 꺾으라고 지시하는 것은 별개고, 지시는 데모 스크립트
    #   안에만 있었다. 그래서 존 옆을 그냥 지나쳐 순환 루프만 돌았다
    #   (실측: 12 개 중 존 진입 0 개). router.py docstring 참조.
    ap.add_argument("--no-route", action="store_true",
                    help="배송지 분류를 끈다 (기본은 켬)")
    a = ap.parse_args()

    import _paths                                       # noqa: E402
    _paths.check()
    cfg = _paths.system_config()

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})
    # ★ vsync·프레임 제한 해제 (2026-08-27). GUI 모드에서 app.update() 가
    #   화면 주사율에 묶이면 물리가 아무리 빨라도 렌더 프레임이 벽시계를 먹는다
    #   (실측: 렌더 프레임당 33 ms → PL_RENDER_EVERY=2 로도 1.0 배속에 정체).
    #   이미 꺼져 있으면 아무 일도 없다.
    try:
        import carb.settings
        _cs = carb.settings.get_settings()
        _cs.set("/app/vsync", False)
        _cs.set("/app/runLoops/main/rateLimitEnabled", False)
    except Exception as _e:                                     # noqa: BLE001
        print("[sim_main] vsync 해제 실패 (무시): %r" % (_e,))

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
        # ★ PL_BG=0 — 장식 배경을 **끄고 연다** (2026-08-27). 새 씬(배경 추가본)을
        #   8 GB GPU(5060)에서 열면 로드 중 `Killed`(OOM) 로 죽는다 — 실측.
        #   `full_warehouse` 는 콜라이더 0 개인 **순수 시각** 프림이라 꺼도 물리·
        #   좌표·반출 경로가 하나도 안 바뀐다. 시연 PC(5080/16GB)는 켜고 돌린다.
        #   파일은 안 고친다 (`SetActive(False)` 는 열린 스테이지에만 적용).
        if os.environ.get("PL_BG") == "0":
            import omni.usd as _ou          # Isaac 런타임 전용 — 지역 import
            _st0 = _ou.get_context().get_stage()
            _off = []
            for _n in ("full_warehouse", "PlatformLuggageCart_A01_01",
                       "PlatformLuggageCart_A01_02"):
                _p = _st0.GetPrimAtPath("/World/" + _n)
                if _p and _p.IsValid() and _p.IsActive():
                    _p.SetActive(False)
                    _off.append(_n)
            print("[sim_main] PL_BG=0 — 배경 %d 개 껐다 (%s)"
                  % (len(_off), ", ".join(_off) if _off else "없음"))
        _add_camera_lights(cfg)
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

        # ★ despawn 래퍼 (2026-08-27 실측). PLACE 실패(걸림 142mm·낙하 불가)로
        #   박스가 패드에 매달린 채 관제탑이 despawn 을 부를 수 있다. 그리퍼
        #   조인트를 문 채 순간이동시키면 조인트가 로봇을 잡아채므로, 어느 팔이든
        #   그 박스를 물고 있으면 **먼저 패드를 열고** 치운다. 전부 mainq 안에서
        #   한 job 으로 돌아 순서가 보장된다. rms 는 아래 로봇 루프가 채운다 —
        #   같은 dict 객체라 클로저로 잡아도 나중 내용이 보인다.
        _despawn_orig = H["despawn_box"]

        def h_despawn_grip(req, _sp=sp, _rms=rms, _q=q):
            bid = (req or {}).get("box_id")
            rec = _sp.boxes.get(bid)
            if rec is None:
                return {"ok": False, "error": "없는 box_id %s" % bid}

            def _job():
                for _z, _rm in _rms.items():
                    try:
                        if rec["path"] in (_rm.grip.gripped() or []):
                            _rm.grip.open()
                            print("[sim_main] despawn 전 %s 그리퍼 개방 — %s 를 물고 있었다"
                                  % (_z, bid))
                    except Exception as e:      # 개방 실패해도 despawn 은 진행한다
                        print("[sim_main] despawn 전 그리퍼 개방 실패(%s): %s" % (_z, e))
                return _sp._despawn(bid)
            return _q.run(_job)

        H["despawn_box"] = h_despawn_grip
        # register() 는 중복을 거부한다 — 이건 의도된 덮어쓰기이므로 표에 직접 넣는다
        server.handlers["despawn_box"] = h_despawn_grip

        # ── 로봇 (B 소유 핸들러) ─────────────────────────────
        # ★ 지금 집으려는 박스. 워커가 없는 정본 서버에서는 `grasp` 요청이 곧
        #   그 신호다. B 코드를 고치지 않고 **등록할 때 감싸서** 채운다.
        # ★★ 구역마다 **핸들러 표를 따로 만든다** (2026-08-26). `move_ee`·`grasp` 는
        #   이름에 구역이 없어서 한 표에 몰면 마지막 로봇이 앞의 것을 덮는다 —
        #   A 구역 명령이 C 구역 팔을 움직인다. 소켓은 예전처럼 **첫 구역** 것만
        #   등록한다 (소켓 규약에 구역 칸이 없다. ROS 를 쓰면 구역이 갈린다).
        # ★★★ `cur` 는 **구역마다 하나**다 (2026-08-26 김학민 리뷰에서 잡힘).
        #   하나로 두면 로봇 3 대가 같은 칸을 쓴다. `_gap_mm` 이 여기를 읽어
        #   흡착면~박스 간극을 재는데, 그게 `_grasp_verdict` 3 mm 검사의 **유일한
        #   입력**이다. 동시에 돌면 두 가지로 깨진다:
        #     · A 가 집는 중에 C 가 grasp -> A 가 **수십 m 떨어진 C 의 박스**까지
        #       거리를 잰다 -> 제대로 물었는데 실패 판정
        #     · C 의 finally 가 먼저 돌면 `cur["box"]=None` -> A 는 간극 검사가
        #       **통째로 건너뛰어진다.** 박스가 몇 mm 떠서 매달려도 모른다.
        #   ★ 두 번째가 더 나쁘다 — **에러가 안 난다.** 배치 Z 오차가 0.0 으로
        #     나와도 검사가 꺼진 채 통과한 것일 수 있다.
        #   그래서 `cur_z` 를 루프 **안**에서 만들고, 클로저가 아니라
        #   **기본인자**로 묶는다 (클로저면 마지막 구역 것만 남는다).
        HZ = {}                       # {zone: 그 구역 핸들러 표}
        if not a.no_robot:
            for zi, z in enumerate(ZONES):
                cur_z = {"box": None}          # ← 구역 전용
                rm_z = _bring_up_robot(world, stage, cfg, z, spawner=sp, cur=cur_z,
                                       prim_path=robot_prim(z, MULTI),
                                       grip_path=gripper_prim(z, MULTI))
                if rm_z is None:
                    continue
                rms[z] = rm_z
                _h = dict(rm_z.handlers())
                _grasp = _h["grasp"]

                def h_grasp(req, _f=_grasp, _c=cur_z):
                    bid = (req or {}).get("box_id")
                    rec = sp.boxes.get(bid)
                    _c["box"] = (bid, rec["dims"][2] / 2.0) if rec else None
                    try:
                        return _f(req)
                    finally:
                        _c["box"] = None    # 사이클 밖에서 옛 박스로 재지 않는다

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
        gtpick = None
        if a.ros:
            # ★ omni.replicator.core 는 **카메라용**이다 — `ROS2CameraHelper` 가
            #   요구하는 renderProduct 를 여기서 만든다 (ros_graph.build_cameras).
            for _e in ("isaacsim.core.nodes", "isaacsim.ros2.bridge",
                       "omni.graph.scriptnode", "omni.replicator.core"):
                enable_extension(_e)
            kit.update()
            import ros_side
            ros_side.install_services(H)
            ros_side.build(ZONES, cfg=cfg)   # cfg -> /clock·/tf 도 (씬에 없을 때만)
            ros = [ros_side.RosSide(cfg, z, HZ[z]) for z in ZONES]
            # ★ 임시 대역 — 카메라가 없어 인식 노드가 못 내는 픽업 좌표를
            #   씬 실측으로 대신 낸다. PL_GTPICK=0 이면 안 만든다.
            import ros_graph as _RG
            if _RG.GT_PICK:
                gtpick = _GtPick(sp, ZONES, _RG, conveyor=cc, cfg=cfg)
                print("[sim_main] 픽업 좌표를 **씬 실측**으로 낸다 (임시 대역) — "
                      "카메라가 들어오면 PL_GTPICK=0")

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

        router = None
        if not a.no_route:
            import router as RT
            router = RT.Router(cfg, sp, cc)

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
        # ★ PL_RENDER_EVERY=N — N 프레임에 1 번만 그린다 (기본 1 = 매 프레임).
        #   GUI 병목은 렌더링이라 2 면 체감 약 2 배속. 물리(60 Hz)는 그대로다.
        #   채택 실행(demo_stack15 --render-every 2)과 같은 장치다.
        #   ★ 2026-08-27: 라우터·급송·gtpick 이 **시뮬 시간**으로 돌게 바꿔서
        #     이제 3 이상도 분기 창을 안 놓친다 (아래 _tsim 주석). 상한은 화면
        #     끊김을 얼마나 참을 수 있느냐 뿐이다 — 4 면 15 fps 로 보인다.
        _rn = max(1, int(os.environ.get("PL_RENDER_EVERY", "1")))
        if _rn > 1:
            print("[sim_main] 렌더 %d 프레임당 1 회 — 체감 약 %d 배속" % (_rn, _rn))
        _fr = 0
        # ★ PL_PROF=1 — 프레임 시간이 어디로 가는지 300 프레임마다 찍는다
        #   (2026-08-27). 렌더 스킵 3 에서도 0.8 배속이라 병목이 렌더가 아닌
        #   것으로 보인다 — §10-2 "어디서 쓰는지 먼저 재라".
        _prof = os.environ.get("PL_PROF", "0") == "1"
        _pt = {"rm": 0.0, "sp": 0.0, "etc": 0.0, "ros": 0.0,
               "step_r": 0.0, "step_p": 0.0, "n_r": 0, "n_p": 0}
        while kit.is_running():
            _t0 = time.perf_counter() if _prof else 0.0
            q.pump()                       # 소켓 스레드가 맡긴 USD 작업
            for _rm in rms.values():
                _rm.step()                 # ★ world.step() 앞. 로봇 전부
            if _prof:
                _t1 = time.perf_counter(); _pt["rm"] += _t1 - _t0
            sp.step()
            if _prof:
                _t2 = time.perf_counter(); _pt["sp"] += _t2 - _t1
            # ★★ 벽시계가 아니라 **시뮬 시간**을 준다 (2026-08-27). 벽시계로 주면
            #   배속(PL_RENDER_EVERY)에 따라 라우터 폴링 대비 분기 창이 좁아져
            #   3 배속부터 소터가 박스를 놓쳤다. 시뮬 시간이면 폴링·스폰 간격이
            #   전부 시뮬 기준이라 **배속과 무관하게 같은 동작**이 된다
            #   (급송 8 초도 시뮬 8 초 — 재현성에도 이게 맞다).
            _tsim = world.current_time
            if router is not None:
                router.step(_tsim)         # ★ 논블로킹 (§9). 급송보다 먼저.
            if feeder is not None:
                feeder.step(_tsim)
            if gtpick is not None:
                gtpick.step(_tsim)         # ★ ros.step 앞. 이번 틱에 반영되게
            if _prof:
                _t3 = time.perf_counter(); _pt["etc"] += _t3 - _t2
            for _r in ros:
                _r.step()              # ★ 논블로킹. 모션은 워커가 돈다 (§9)
            if _prof:
                _t4 = time.perf_counter(); _pt["ros"] += _t4 - _t3
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
            _fr += 1
            _rend = (not a.no_render) and (_fr % _rn == 0)
            world.step(render=_rend)
            if _prof:
                _t5 = time.perf_counter()
                if _rend:
                    _pt["step_r"] += _t5 - _t4; _pt["n_r"] += 1
                else:
                    _pt["step_p"] += _t5 - _t4; _pt["n_p"] += 1
                if _fr % 300 == 0 and _fr:
                    _tot = sum(v for k, v in _pt.items() if not k.startswith("n_"))
                    print("[prof] 300프레임 %.1fs (배속 %.2f) | 로봇 %.1f 스폰 %.1f "
                          "잡무 %.1f ros %.1f | step렌더 %.1fms x%d step물리 %.1fms x%d"
                          % (_tot, 300 / 60.0 / max(_tot, 1e-9),
                             _pt["rm"], _pt["sp"], _pt["etc"], _pt["ros"],
                             _pt["step_r"] * 1000 / max(_pt["n_r"], 1), _pt["n_r"],
                             _pt["step_p"] * 1000 / max(_pt["n_p"], 1), _pt["n_p"]))
                    for k in _pt:
                        _pt[k] = 0 if k.startswith("n_") else 0.0
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


# 마스터 DB. `db_service_node.default_db_path()` 와 같은 자리다 (워크스페이스 루트).
#   ★ **읽기 전용으로만 연다.** 유일한 writer 는 db_service_node 다 (CLAUDE.md §8) —
#     다른 프로세스가 쓰기로 열면 `database is locked` 가 난다.
_DB_PATH = str(Path(__file__).resolve().parents[3] / "palletris.db")


class _GtPick:
    """픽업 좌표를 **씬 실측(ground truth)** 으로 `/zone_*/box_at_pickup` 에 낸다.

    ★★ **임시 대역이다.** §3-2 정본은 이 토픽을 인식 노드(트랙 D)가 낸다.
      씬에 카메라 prim 이 아직 하나도 없어서(2026-08-26, 89 개 삭제) 그 노드가
      볼 이미지가 없다. 이 토픽 하나가 비면 관제탑이 **영원히 IDLE** 이라
      그 뒤 사슬(algo · db · 로봇 · 반출)을 통째로 못 잰다.
      카메라가 들어오면 `PL_GTPICK=0` 으로 끄고 인식 노드로 돌린다.
      ⚠ 둘 다 켜면 발행자가 둘이라 관제탑이 두 좌표를 번갈아 받는다.

    ★ 값은 `spawner.h_pickup_check` 가 그대로 준다 — 스토퍼에 **멎은** 박스만
      돌려주므로 도착 판정을 여기서 다시 하지 않는다.

    ★ 5 Hz 로 낸다. 관제탑이 같은 박스를 여러 번 받는 것을 전제로 만들어져 있고
      (`on_box_arrived`: "detector 는 5Hz 폴링"), 늦게 떠도 도착을 놓치지 않는다.
    """

    PERIOD_S = 0.2

    def __init__(self, spawner, zones, rg, log=print, conveyor=None, cfg=None):
        # ★ `ros_graph` 를 인자로 받는다. 그 모듈은 최상단에서 `omni.graph.core` 를
        #   import 하므로 **Isaac 이 뜬 뒤에만** 들어온다 — sim_main 상단에서 못 건다.
        self.sp, self.zones, self.rg, self.log = spawner, list(zones), rg, log
        # ★ PL_SPUR_STOP=2 — **점유 기반** 지선 제어 (2026-08-27 김병직 설계 완성형).
        #   모드 1("픽 중 정지")은 인식 대기 중이 무방비라, 앞차가 스토퍼에서
        #   인식을 기다리는 동안 뒤차가 내려와 붙었다 (실측: C존 3호+5호 병합
        #   실루엣 734x543mm → 인식 영구 실패). 여기서는 **스토퍼 점유 = 정지,
        #   비면(박스가 들려 나가면) = 재가동**을 5Hz 폴마다 직접 건다.
        #   메인 스레드라 conveyor_ctl 을 mainq 없이 바로 불러도 안전하다.
        self._cc = conveyor
        self._spur2 = (os.environ.get("PL_SPUR_STOP") == "2"
                       and conveyor is not None and cfg is not None)
        if self._spur2:
            self._spur_mps = float(cfg["conveyor"].get(
                "spur_velocity_mps", cfg["conveyor"]["belt_velocity_mps"]))
            self._spur_cur = {}          # zone -> 마지막 지시값 (스팸 방지)
            # ★ 진입부 보호 (2026-08-27 2차 — 영상 'A존 상자겹침' 프레임 추적).
            #   점유 정지가 **막 꺾여 들어오던 박스를 본선 교차 지점에서 얼렸다** —
            #   입구에 45도로 걸쳐 선 박스를 본선 뒤차들이 치고 지나가며 회전
            #   (yaw 60/180)·겹침·사이드 이탈이 났다. 진입부(교차 창 ~ 진입 판정
            #   +여유)에 박스가 있는 동안은 정지를 미룬다 — 교차로를 벗어난 뒤
            #   얼리면 의도대로 "지선 중간 대기"가 된다.
            self._cross = {str(k).upper(): (v[0] / 1000.0, v[1] / 1000.0)
                           for k, v in
                           (cfg["zones"].get("pickup_cross_mm") or {}).items()}
            self._entry_m = (float(cfg["routing"].get("divert_done_y_mm", 1200))
                             / 1000.0 + 0.4)
            log("[gtpick] 지선 점유 제어 켬 (PL_SPUR_STOP=2) — 스토퍼에 박스가"
                " 있는 동안 그 지선을 세운다 (진입부 %.1fm 까지는 통과 보장)"
                % self._entry_m)
        self.next_t = 0.0
        self._said = {}          # zone -> 마지막으로 알린 box_id (로그 도배 방지)
        # ★ 자리 잡음 판정 (2026-08-27 실측). 도착 직후 아직 돌고 있는 박스를
        #   보고했다 — BOX011 이 yaw -26.6 -> -20.2 -> -0.1 로 찍혔고, 관제탑은
        #   첫 값으로 집으러 가서 흡착이 4.1mm 떠서 실패했다. **연속 두 번의
        #   폴에서 같은 자리(5mm·1.5도 이내)여야** 보고한다.
        self._prev = {}          # zone -> (box_id, x_mm, y_mm, yaw_deg)
        self._first = {}         # zone -> (box_id, 처음 본 시각) — 필터 타임아웃용
        # ★ 스토퍼 창. 지선에 줄 선 **둘째 박스**(스토퍼 앞 200mm)를 보고해
        #   팔이 엉뚱한 자리로 간 사고 (잔차 79mm IK_FAIL). 가장 큰 박스가
        #   스토퍼에 닿았을 때의 중심보다 60mm 여유까지만 픽업으로 인정한다.
        _half = max(max(t["w_mm"], t["d_mm"]) for t in
                    self.sp.cfg["boxes"]["types"]) / 2000.0
        self._win_m = self.sp.stopper_pos_m - _half - 0.060

    def step(self, now):
        if now < self.next_t:
            return
        self.next_t = now + self.PERIOD_S
        for z in self.zones:
            r = self.sp.h_pickup_check({"zone": z}) or {}
            if r.get("error"):
                # 스포너가 모르는 존이다. 조용히 넘어가면 영영 안 온다.
                self.log("[gtpick] %s %s" % (z, r["error"]))
                continue
            pose = r.get("pose") or {}
            pos = pose.get("pos_m") or (0.0, 0.0, 0.0)
            present = bool(r.get("present"))
            bid = str(r.get("box_id") or "")
            # ── 점유 기반 지선 제어 (__init__ 의 _spur2 주석) ──
            if self._spur2:
                want = 0.0 if present else self._spur_mps
                if want == 0.0:
                    # 진입부 보호 — 교차 창에 박스가 있으면 아직 못 세운다
                    _cw = self._cross.get(str(z).upper())
                    if _cw and any(
                            _e.get("pos_m") is not None
                            and _cw[0] <= _e["pos_m"][0] <= _cw[1]
                            and 0.30 <= _e["pos_m"][1] <= self._entry_m
                            for _e in dict(self.sp._snap).values()):
                        want = self._spur_mps
                if self._spur_cur.get(z) != want:
                    try:
                        self._cc.set_velocity("%s_pickup" % str(z).lower(), want)
                        self._spur_cur[z] = want
                    except Exception as e:                      # noqa: BLE001
                        self.log("[gtpick] 지선 %s 속도 실패 (무시): %r" % (z, e))
            if present:
                # ★ 필터 타임아웃 (2026-08-27 실측 — B존 277초 침묵). 뒤집힌 박스가
                #   창 밖에 누워 길을 막으면 아래 두 필터가 **영원히** 보고를 안 해
                #   존이 죽고, 그 존행 박스들이 리턴만 돈다 (74바퀴 실측). 같은
                #   박스가 이만큼 버티면 필터를 무시하고 일단 보고한다 — 픽이
                #   실패해도 재시도 상한(3회)이 치워 주므로 지선이 뚫린다.
                _fk = self._first.get(z)
                if _fk is None or _fk[0] != bid:
                    self._first[z] = _fk = (bid, now)
                _force = (now - _fk[1]) > 15.0
                if _force and self._said.get(z) != bid:
                    self.log("[gtpick] %s %s 15초째 필터에 걸림(창밖/불안정) — "
                             "강제 보고 (실패하면 상한이 치운다)" % (z, bid))
                # ── 스토퍼 창 밖 = 아직 도착 안 한 박스. 이번 틱은 보고하지 않는다
                #    (present=false 를 보내면 관제탑이 "치워졌다"로 오인해 빗장을 푼다)
                if pos[self.sp.axis] < self._win_m and not _force:
                    continue
                # ── 자리 잡음: 직전 폴과 같은 자리일 때만 보고
                cur = (bid, round(pos[0] * 1000), round(pos[1] * 1000),
                       float(pose.get("yaw_deg") or 0.0))
                pv = self._prev.get(z)
                self._prev[z] = cur
                if (not _force and (pv is None or pv[0] != bid
                        or abs(pv[1] - cur[1]) > 5 or abs(pv[2] - cur[2]) > 5
                        or abs(pv[3] - cur[3]) > 1.5)):
                    continue
            else:
                self._prev[z] = None
                self._first[z] = None
            self.rg.send_pickup(
                zone=z, box_id=bid, present=present,
                # ★ mm 정수다 (§3-2). x·y 는 박스 **중심**.
                x_mm=int(round(pos[0] * 1000.0)),
                y_mm=int(round(pos[1] * 1000.0)),
                yaw_deg=float(pose.get("yaw_deg") or 0.0))
            if present and self._said.get(z) != bid:
                self._said[z] = bid
                self.log("[gtpick] %s 도착 %s (%d, %d) yaw %.1f"
                         % (z, bid, round(pos[0] * 1000), round(pos[1] * 1000),
                            float(pose.get("yaw_deg") or 0.0)))
            elif not present:
                self._said[z] = None


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
        # ★★ 2026-08-26 — **마스터 DB 가 구역의 정본이다.**
        #   정본 설계(트랙 D)는 운송장번호 -> boxes_master.address -> 동 ->
        #   zone_map -> 구역 이다. `by_dest`(SEOUL/BUSAN/DAEJEON)는 그 앞 단계의
        #   자리표시자라, 도시로 뿌리면 DB 가 정한 구역과 **어긋난다.**
        #   실측 2026-08-26: BOX002(천왕동 -> C)가 **A 팔레트에 실렸다.** 에러는 없었다.
        #   -> 여기서 box_id 의 DB 구역을 읽어, 그 구역으로 가는 dest 로 바꿔 준다.
        #      라우팅 표(by_dest)는 그대로 쓰므로 진리원은 하나(DB)로 남는다.
        #   ⚠ 임시다. 정본은 `/diverter/request`(db_service_node.on_diverter)를
        #     부르는 것인데, 이 코드는 Isaac 안이라 rclpy 를 못 쓴다. 중재자 사슬이
        #     붙으면 이 블록을 지운다.
        self._db_dest = {}
        try:
            import sqlite3
            _bd = (cfg.get("routing") or {}).get("by_dest") or {}
            zone_to_dest = {}
            for _d, _z in _bd.items():
                zone_to_dest.setdefault(str(_z).upper(), _d)
            if zone_to_dest:
                _c = sqlite3.connect("file:%s?mode=ro" % _DB_PATH, uri=True)
                for _bid, _z in _c.execute("SELECT box_id, zone FROM boxes_master"):
                    _dd = zone_to_dest.get(str(_z).upper())
                    if _dd:
                        self._db_dest[_bid] = _dd
                _c.close()
                log("[투입] 구역은 **마스터 DB** 를 따른다 (%d 건) — by_dest 는 표기용"
                    % len(self._db_dest))
        except Exception as e:                                  # noqa: BLE001
            # DB 를 못 읽어도 라인은 돌아야 한다. 다만 조용히 넘어가면 안 된다 —
            # 그 순간부터 구역 배정이 DB 와 어긋난 채로 계속 돈다.
            log("[투입] !! 마스터 DB 를 못 읽었다 (%r) — by_dest 로 뿌린다. "
                "DB 가 정한 구역과 어긋날 수 있다" % (e,))
        self.h_spawn = sp.handlers()["spawn_box"]
        self.h_stat = sp.handlers()["spawn_status"]
        self.period, self.total, self.cap, self.log = float(period), int(total), int(cap), log
        self.i, self.next_t = 0, 0.0
        # ★ 리턴 합류 **예측** 스폰 보류 (2026-08-27 김병직 지시).
        #   리턴에서 복귀하는 박스는 본선 사이드로 얹혀 들어오고, 그 순간 새로
        #   스폰된 박스와 간격이 압축돼 지선에서 추돌·비틀림이 생긴다 (영상
        #   '풀비전_리턴구간 이슈'). 감시 창(merge_watch_mm)은 2026-08-26 에
        #   설정만 들어가고 **소비 코드가 없었다** — 여기서 처음 작동한다.
        #   접근 창(merge_approach_mm, _04 커브부터)까지 보면 합류 몇 초 전에
        #   미리 스폰이 멈춰서, 복귀 박스가 본선에 자리 잡은 뒤 새 박스가 나온다.
        self.sp = sp
        _lp = (cfg["conveyor"].get("loop") or {})
        self._watch = []
        for _k in ("merge_watch_mm", "merge_approach_mm"):
            _w = _lp.get(_k)
            if _w:
                self._watch.append((_w["x"][0] / 1000.0, _w["x"][1] / 1000.0,
                                    _w["y"][0] / 1000.0, _w["y"][1] / 1000.0))
        self._held_log = 0.0
        # ★ 걸림 감시 (2026-08-27 강의실 실측). 합류 창 안에서 박스가 이음매에
        #   걸려 안 움직이면 스폰 보류가 **영영 안 풀린다** — 라인 전체가 말라붙는
        #   것을 화면으로 확인했다. "접근 중"의 정의에 움직임을 넣는다:
        #   8초째 제자리(20mm 미만 이동)면 보류 대상에서 빼고, 30초째면 낙하물로
        #   보고 치운다(despawn). 창 안에서 30초 정지는 정상 흐름에 없다 —
        #   갭핑 정지는 최대 2초, 본선·리턴 벨트는 여기서 세우지 않는다.
        self._move_hist = {}     # {box_id: [x, y, 마지막으로 움직인 시각]}

    def step(self, now):
        if self.total and self.i >= self.total:
            return
        if self.i >= len(self.types) or now < self.next_t:
            return
        st = self.h_stat({}) or {}
        # 스폰 지점이 아직 안 비었으면 **겹쳐 스폰된다.** 다음 틱에 다시 본다.
        if st.get("on_belt", 0) >= self.cap or not st.get("spawn_clear", True):
            return
        # ★ 리턴 박스가 합류에 접근 중이면 스폰을 미룬다 (__init__ 주석 참조).
        #   벨트는 안 세운다 — 투입만 잡는다.
        if self._watch:
            for _b, _e in dict(getattr(self.sp, "_snap", {})).items():
                _p = _e.get("pos_m")
                if not (_p and any(x0 <= _p[0] <= x1 and y0 <= _p[1] <= y1
                                   for x0, x1, y0, y1 in self._watch)):
                    self._move_hist.pop(_b, None)
                    continue
                # ── 걸림 검사 (__init__ 주석) — 제자리 박스는 "접근 중"이 아니다 ──
                _h = self._move_hist.get(_b)
                if _h is None or abs(_p[0] - _h[0]) + abs(_p[1] - _h[1]) > 0.02:
                    self._move_hist[_b] = [_p[0], _p[1], now]
                    _still = 0.0
                else:
                    _still = now - _h[2]
                if _still >= 30.0:
                    self.log("[투입] %s 가 합류 창에서 30초째 제자리 — 낙하물로 보고 치운다"
                             % _b)
                    self._move_hist.pop(_b, None)
                    self.sp._despawn(_b)       # 메인 스레드다 — 직접 불러도 된다
                    continue
                if _still >= 8.0:
                    continue                   # 걸린 박스 때문에 스폰을 영영 막지 않는다
                if now - self._held_log > 5.0:
                    self._held_log = now
                    self.log("[투입] 리턴 박스 %s 합류 접근 — 스폰 보류" % _b)
                return
        # ★★ **3 자리다** (`BOX001`). 5 자리로 내면 조용히 다 막힌다 (2026-08-26 실측):
        #   · 마스터 DB(`boxes_master`)가 3 자리라 관제탑이 "DB 에 없음 -> 폐기"
        #   · QR 라벨 PNG 도 `qr_labels/BOX001.png` 라 라벨이 안 붙는다
        #     -> 픽업 카메라가 QR 을 못 읽어 `box_id` 가 빈 채 나가고,
        #        관제탑이 "box_at_pickup 에 box_id 가 비어 있다 — 무시" 로 버린다
        #   ⚠ `demo_stack15_test1_1.py:134` 은 `BOX%05d` 인데 그쪽은 DB 를 안 쓴다.
        bid = "BOX%03d" % (self.i + 1)
        kw = {"box_id": bid, "type": self.types[self.i], "yaw_deg": 0}
        # ★ DB 가 아는 박스면 DB 구역을 따른다 (위 주석).
        _d = self._db_dest.get(bid) or self.dests[self.i]
        if _d:
            kw["dest"] = _d
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
