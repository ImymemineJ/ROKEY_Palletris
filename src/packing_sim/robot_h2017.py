# ⚠⚠ **구버전이다. 지금 씬과 안 맞는다.** (2026-08-23)
#
# 이 경로(`open_test1_1.py` + `spawn_feeder` + `belt_cycle` + `robot_h2017`)는
# **스토퍼 벽이 생기기 전** 설계다. 지금 씬에는 x 10000~10100 에 실물 벽이 서 있어
# "벨트를 브레이크로 쓴다" 는 전제가 성립하지 않는다.
# 통합 경로는 `scripts/demo_stack15_test1_1.py` 하나다 (cycle.py 상태머신 +
# robot_motion + 소켓 브릿지 + HeightmapPacker). 게이트 수치는 전부 그쪽 것이다.
#
# 트랙B 리뷰(2026-08-23)가 이 경로에서 다섯 가지를 지적했고 **전부 확인됐다.**
# 지금은 아무도 안 부르므로 잠복이지만, 되살리려면 먼저 고쳐야 한다:
#   1. `robot_h2017.ROBOT_PRIM = "/World/robot/h2017"` 가 씬에 없다. 씬 빌더는
#      defaultPrim 을 `/World/robot` 에 직접 합성한다 (EE 도 `/World/robot/root_joint/tool0`).
#   2. 이송 고도에 매달린 박스 높이(+h)가 빠졌다 — 여유가 approach(200) − h 라
#      3호 −10 / 4호 −80 / 5호 −140 mm 로 전부 음수다. 같은 층을 관통한다.
#   3. IK/FK 실패 시 웨이포인트를 건너뛰고 다음 액션을 그대로 실행한다 (공중 흡착).
#   4. HOME 복귀가 보간 없는 계단 입력이다.
#   5. `belt_cycle.pallet_slot` 이 층 높이를 현재 박스 높이로 전 층에 적용한다.
#
"""H2017 팔레타이징 드라이버 — belt_cycle 에 꽂아 쓴다.

belt_cycle 이 박스를 픽 지점에 세워 주면 여기서 집어 팔레트에 놓고 원위치한다.
belt_cycle 은 `is_idle()` 이 True 가 되는 순간 벨트를 다시 돌린다.

    import belt_cycle, robot_h2017
    robot_h2017.attach()      # belt_cycle.set_robot(...) 까지 해 준다
    belt_cycle.start()

IK 는 Lula 를 쓴다 (`isaacsim.robot_motion.motion_generation`).
로봇 서술자와 URDF 는 로봇 트랙이 관리하는 것을 그대로 읽는다
(`src/packing_robot/config/h2017_descriptor.yaml`, `urdf/h2017.urdf`).
여기에 링크 길이를 적지 않는다 — 두 벌이 되면 반드시 어긋난다.

## 흡착은 아직 가짜다

박스를 잡는 동안 **키네마틱으로 바꿔 tool0 를 따라가게** 한다. 놓을 때
다시 다이내믹으로 되돌린다. 진짜 흡착(5.1 SurfaceGripper)은 `suction.py` 에
있고, 그리퍼 프림이 로봇 USD 에 붙은 뒤에 갈아 끼우면 된다.
갈아 끼울 자리는 `_grip()` / `_release()` 둘뿐이다.

## 움직임은 **태스크 공간 보간**이다

웨이포인트에서만 IK 를 풀고 그 사이를 드라이브에 맡기면(= 관절 보간) 툴
자세가 양 끝에서만 맞고 중간은 보존되지 않는다. 손목이 크게 도는 구간에서
팔이 통째로 감긴다. 2026-08-22 실측: '이송' 하나에서 잔여가 **463도** 까지
벌어졌고 (한 바퀴 넘게 감겼다) 제한시간을 10 -> 30초로 늘려도 더 감기기만
했다. 박스는 팔레트가 아니라 바닥에 떨어졌다.

지금은 **흡착면 위치를 직선으로 쪼개고 매 물리 스텝마다 IK 를 푼다.**
자세는 매 스텝 수직 하향으로 지령되므로 경로 내내 유지된다. 웜스타트도
고정(HOME_Q)이라 해가 다른 가지로 튀지 않는다.

이 처방의 출처는 로봇 트랙이다 — `scripts/verify_cycle.py` (main) 의
"★ 보간은 태스크 공간이다" 와 `--warm ready`. 여기서 다시 유도하지 않는다.

## IK 해는 **현재 자세에서 가장 가까운 것**을 고른다

태스크 공간으로 쪼개도 그것만으로는 부족했다. `_solve` 가 시드 목록에서
**처음 풀린 해**를 그냥 돌려주면, 목표가 몇 mm 만 움직여도 다른 가지의 해가
먼저 풀려 팔이 통째로 돈다. 2026-08-22 실측:

    팔레트 (10600, 650) 1번 박스   정상
    팔레트 (10600, 1050) 2번 박스  정상
    팔레트 (11200, 650) 3번 박스   '이송' 시간 초과, 잔여 227.2도

세 목표 모두 도달범위 안이다 (최원 1158mm < 실효 1550mm). 리치가 아니라
가지 선택 문제다. 지금은 시드를 전부 풀어 보고 **현재 관절각과의 최대 차이가
가장 작은 해**를 고른다. joint_1/4/5/6 은 가동범위가 ±360도라 같은 자세를
±2pi 만큼 옮긴 등가해가 있으므로, 그중 현재값에 가까운 쪽으로 풀어 준다
(joint_2 ±125도 · joint_3 ±160도 는 2pi 여유가 없어 그대로 둔다).

## 드라이브 게인은 건드리지 않는다

한때 팔이 IK 지령을 못 따라가서 (잔여 46도) 게인을 의심했다. 아니었다.
**팔레트가 어깨 스윙 높이에 물려 joint_1 이 -47.9 도에서 기계적으로 막혀
있던 것**이다 (2026-08-22 실측: maxForce 를 372 -> 1e5 로 올려도 안 풀렸고,
/World/Pallet 충돌만 끄면 즉시 풀렸다). 환경 쪽에서 팔레트를 y 300 -> 450
으로 옮겨 해결했다. 씬의 게인(stiffness 1000 / damping 100) 그대로 잔여
1.8 도에 0.2~1.0 초로 따라간다. 로봇 설정은 그대로 둔다.

환경 변수
    PL_TCP_MPS       흡착면 직선 이송 속도 [m/s]. 기본 0.5.
    PL_WARM          ready(기본) | chain. IK 웜스타트를 고정할지 이어갈지.
    PL_APPROACH_MM   접근 여유 [mm]. 기본은 사양 robot.approach_offset_mm.
    PL_JOINT_TOL_DEG 웨이포인트 도달 판정 [도]. 기본 2.0.
    PL_WP_TIMEOUT_S  한 웨이포인트 최대 대기 [초]. 기본 6.0.
"""

import os

import numpy as np
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

import belt_cycle

ROBOT_PRIM = "/World/robot/h2017"
EE_FRAME = "tool0"

REPO = os.path.expanduser("~/palletris_ws")
DESCRIPTOR = f"{REPO}/src/packing_robot/config/h2017_descriptor.yaml"
URDF = f"{REPO}/src/packing_robot/urdf/h2017.urdf"

JOINT_TOL = np.radians(float(os.environ.get("PL_JOINT_TOL_DEG", "2.0")))
# 흡착면을 직선으로 끌고 가는 속도 [m/s]. 태스크 공간 보간에 쓴다.
TCP_MPS = float(os.environ.get("PL_TCP_MPS", "0.5"))
# IK 웜스타트. ready = 고정 기준자세(감김 안 함, main 의 처방) / chain = 직전 해.
WARM = os.environ.get("PL_WARM", "ready")
WP_TIMEOUT = float(os.environ.get("PL_WP_TIMEOUT_S", "10.0"))

# 홈 자세 — descriptor 의 default_q. 벨트와 팔레트 사이 어디에도 안 걸린다.
HOME_Q = np.array([0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0])

_s = {}


# ─────────────────────────────────────────────────────────────
def attach():
    """belt_cycle 에 자신을 꽂는다. belt_cycle.start() 전에 부른다.

    ★ 여기서 아티큘레이션을 초기화하지 않는다. Play 전에는 물리 뷰가 없다.
      자세한 이유는 _ensure_ready() 주석 참고.
    """
    _setup()
    belt_cycle.set_robot(on_pick=on_pick, is_idle=is_idle)
    # 웨이포인트 진행은 물리 스텝마다 봐야 한다. belt_cycle 은 PICK 단계에서
    # is_idle() 만 묻기 때문에 여기서 따로 구독을 건다.
    from omni.physx import get_physx_interface
    _s["sub"] = get_physx_interface().subscribe_physics_step_events(step)
    # Stop 하면 물리 핸들이 무효가 된다. 다시 Play 할 때 초기화를 되풀이해야 한다.
    _s["timeline_sub"] = (
        omni.timeline.get_timeline_interface()
        .get_timeline_event_stream()
        .create_subscription_to_pop(_on_timeline)
    )
    print("[robot] belt_cycle 에 연결됨 — Play 를 누르면 H2017 이 집는다.")


def detach():
    _s["sub"] = None
    _s["timeline_sub"] = None
    belt_cycle.set_robot(on_pick=None, is_idle=None)


def _setup():
    """물리 없이 되는 준비만 한다 — 프림·파일 확인, Lula, 베이스 포즈."""
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(ROBOT_PRIM):
        raise RuntimeError(
            f"[robot] {ROBOT_PRIM} 가 없다. 씬에 로봇이 안 올라가 있다 — "
            f"./run_layout_to_spec.sh 를 먼저 돌려라.")
    for p in (DESCRIPTOR, URDF):
        if not os.path.exists(p):
            raise RuntimeError(f"[robot] 없는 파일: {p}")

    # 생성자는 물리 뷰가 없어도 된다. initialize() 만 Play 뒤로 미룬다.
    art = SingleArticulation(prim_path=ROBOT_PRIM, name="h2017")
    ik = LulaKinematicsSolver(robot_description_path=DESCRIPTOR, urdf_path=URDF)
    ik.set_default_cspace_seeds(SEEDS)

    # Lula 는 로봇 로컬 프레임으로 푼다. 씬에서 로봇이 페데스탈 위에 올라가
    # 있으므로 베이스 월드 포즈를 알려 줘야 목표 월드 좌표가 맞는다.
    xc = UsdGeom.XformCache()
    m = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(ROBOT_PRIM))
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    base_p = np.array([t[0], t[1], t[2]])
    base_q = np.array([q.GetReal(), *q.GetImaginary()])
    ik.set_robot_base_pose(base_p, base_q)

    # 흡착면 = tool0 에서 툴 z 로 그리퍼 길이만큼. 사양에서 읽는다.
    spec = _spec()
    tool_len = spec["robot"]["gripper_length_mm"] / 1000.0
    approach = float(os.environ.get(
        "PL_APPROACH_MM", spec["robot"]["approach_offset_mm"])) / 1000.0

    _s.update(stage=stage, art=art, ik=ik, solver=None,
              tool_len=tool_len, approach=approach,
              wps=[], i=0, t=0.0, held=None, placed=0, busy=False, ready=False)
    print(f"[robot] Lula 준비 — 베이스 ({base_p[0]:.3f}, {base_p[1]:.3f}, "
          f"{base_p[2]:.3f})  그리퍼 {tool_len*1000:.0f}mm  "
          f"접근 여유 {approach*1000:.0f}mm")


def _ensure_ready():
    """아티큘레이션 초기화. Play 로 물리 뷰가 생긴 뒤에야 된다.

    ★ 예전에는 attach() 안에서 바로 art.initialize() 를 불렀다. 그러면
      SimulationManager.get_physics_sim_view() 가 아직 None 이라

          AttributeError: 'NoneType' object has no attribute
                          'create_articulation_view'

      가 나고, 그 예외가 open_test1_1.py 를 통째로 끊어서 belt_cycle.start()
      까지 실행이 안 됐다. 벨트가 안 서고 로봇이 가만히 있던 원인이 이것이다
      (2026-08-22 로그 kit_20260822_140959).

      물리 핸들은 Stop 하면 무효가 되므로(Articulation 이 STOP 에 구독해
      직접 버린다) 매 스텝 handles_initialized 를 확인해 다시 잡는다.
    """
    art = _s.get("art")
    if art is None:
        return False
    if _s.get("ready") and art.handles_initialized:
        return True

    from isaacsim.core.simulation_manager import SimulationManager
    if SimulationManager.get_physics_sim_view() is None:
        return False            # 아직 Play 전이다. 조용히 기다린다.

    from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver

    art.initialize()
    if list(_s["ik"].get_joint_names()) != list(art.dof_names):
        raise RuntimeError(
            f"[robot] 관절 순서가 다르다. Lula {_s['ik'].get_joint_names()} vs "
            f"아티큘레이션 {list(art.dof_names)}. _solve 의 2단계가 어긋난다.")
    _s["solver"] = ArticulationKinematicsSolver(art, _s["ik"], EE_FRAME)
    _s["ready"] = True
    print(f"[robot] 아티큘레이션 초기화 — {art.num_dof}축 {list(art.dof_names)}")
    _go(HOME_Q)
    return True


def _on_timeline(event):
    """Stop — 물리 핸들이 죽는다. 잡고 있던 박스를 놓고 사이클을 리셋한다."""
    if event.type != int(omni.timeline.TimelineEventType.STOP):
        return
    _release()
    _s.update(ready=False, solver=None, busy=False, held=None,
              wps=[], i=0, t=0.0, q_cmd=None, seg=None, placed=0)
    print("[robot] Stop — 사이클 리셋")


def _spec():
    import yaml
    with open(f"{REPO}/src/packing_bringup/config/system.yaml") as f:
        return yaml.safe_load(f)


# ─────────────────────────────────────────────────────────────
# belt_cycle 인터페이스
def on_pick(box_path, pose):
    """픽 지점에 박스가 섰다. 집어서 팔레트에 놓는 계획을 세운다."""
    stage = _s["stage"]
    prim = stage.GetPrimAtPath(box_path)
    dims = _dims(prim)
    top = pose[2] + dims[2] / 2.0                 # 박스 윗면
    slot = belt_cycle.pallet_slot(stage, _s["placed"], dims)
    drop_top = float(slot[2]) + dims[2] / 2.0     # 놓았을 때의 윗면

    a = _s["approach"]
    # 팔레트 위 이송 높이. 적재 상단에서 approach_offset_mm 만큼 띄운다.
    # ★ 예전에는 여기에 +0.20 이 더 붙어 있었다. system.yaml 어디에도 없는
    #   숫자였고 (철칙 3), 사양의 approach_offset_mm 200 을 이중으로 먹어
    #   이송 높이를 200mm 씩 밀어 올렸다. 그만큼 도달범위를 까먹는다 —
    #   2026-08-22 실측: 2층 5호 이송이 2080mm 로 올라가 y=1050 열이
    #   1760mm 를 요구해 IK 가 실패했다 (실효 리치 1550). 사양값만 쓴다.
    safe = max(top, drop_top) + a

    # ★ 벨트 위에서는 그 높이까지 못 든다. 픽 지점이 로봇에서 1143mm 떨어져
    #   있어서 z 1960mm 는 도달범위 밖이다 — 2026-08-22 실측:
    #
    #       [robot] ★ IK 실패 — '상승' (9757, -0, 1960)
    #
    #   목적지 (10600, 650, 1960) 자체는 1449mm 라 닿는다. 순전히 경로 문제라
    #   벨트 위에서는 박스를 벨트에서 떼는 만큼만 들고, 나머지 높이는 팔레트로
    #   가는 사선에서 번다. 이송은 태스크 공간 직선이라 중간점도 전부 도달범위
    #   안이고(최원 1291mm), 팔레트 앞 모서리를 지날 때 박스 밑면이 적재 상단
    #   보다 150mm 이상 위에 있다.
    lift = top + a

    # (목표 흡착면 z, 동작) 순서로. 흡착면이 박스 윗면에 닿아야 잡힌다.
    _s["wps"] = [
        ("이동",   (pose[0], pose[1], top + a),        None),
        ("하강",   (pose[0], pose[1], top),            None),
        ("흡착",   None,                               ("grip", box_path)),
        ("상승",   (pose[0], pose[1], lift),           None),
        ("이송",   (float(slot[0]), float(slot[1]), safe),     None),
        ("하강",   (float(slot[0]), float(slot[1]), drop_top), None),
        ("놓기",   None,                               ("release", None)),
        ("후퇴",   (float(slot[0]), float(slot[1]), safe),     None),
        ("원위치", None,                               ("home", None)),
    ]
    _s.update(i=0, t=0.0, busy=True, q_cmd=None, seg=None)
    _s.setdefault("log", [])
    print(f"[robot] 사이클 시작 — {box_path.split('/')[-1]} "
          f"({pose[0]*1000:.0f}, {pose[1]*1000:.0f}) 윗면 {top*1000:.0f}mm "
          f"-> 팔레트 ({slot[0]*1000:.0f}, {slot[1]*1000:.0f}) {drop_top*1000:.0f}mm "
          f"| 벨트 위 상승 {lift*1000:.0f}mm, 이송 {safe*1000:.0f}mm")


def is_idle():
    return not _s.get("busy", False)


# ─────────────────────────────────────────────────────────────
def step(dt):
    """물리 스텝마다 부른다 (attach() 가 구독을 걸어 준다)."""
    try:
        if not _ensure_ready():
            return              # Play 전 — 물리 뷰가 아직 없다
    except Exception as exc:
        if not _s.get("init_err"):
            _s["init_err"] = True
            print(f"[robot] ★ 초기화 실패 — {type(exc).__name__}: {exc}")
        return
    if not _s.get("busy"):
        return
    _follow()                       # 잡고 있는 박스를 tool0 에 붙여 둔다
    _s["t"] += dt

    name, target, action = _s["wps"][_s["i"]]
    if action is not None:
        kind, arg = action
        if kind == "grip":
            _grip(arg)
        elif kind == "release":
            _release()
        elif kind == "home":
            _go(HOME_Q)
            if not _reached(HOME_Q) and _s["t"] < WP_TIMEOUT:
                return
            _s.update(busy=False, placed=_s["placed"] + 1)
            print(f"[robot] 원위치 — 적재 {_s['placed']}개")
            return
        _next(name)
        return

    # ★ 태스크 공간 보간 — 흡착면을 직선으로 쪼개고 매 스텝 IK 를 푼다.
    #   자세(수직 하향)가 경로 내내 지령되므로 손목이 감기지 않는다.
    seg = _s.get("seg")
    if seg is None:
        p0 = _tcp_now()
        if p0 is None:
            _s.setdefault("log", []).append(
                (name, "FK실패", tuple(round(v * 1000) for v in target), None, 0.0, 0.0))
            _next(name)
            return
        p1 = np.array(target, dtype=float)
        seg = _s["seg"] = {
            "p0": p0, "p1": p1,
            "d": max(float(np.linalg.norm(p1 - p0)), 1e-6), "u": 0.0}

    seg["u"] = min(1.0, seg["u"] + TCP_MPS * dt / seg["d"])
    tcp = seg["p0"] + (seg["p1"] - seg["p0"]) * seg["u"]

    q = _solve(tuple(tcp))
    if q is None:
        # 중간점 하나를 못 풀었을 뿐이다. 그 자리를 지키고 다음 스텝에 다시 본다.
        if _s["t"] < WP_TIMEOUT:
            return
        _s.setdefault("log", []).append(
            (name, "IK실패", tuple(round(v * 1000) for v in target), None,
             round(_s["t"], 2), round(seg["u"], 2)))
        print(f"[robot] ★ IK 실패 — '{name}' "
              f"({target[0]*1000:.0f}, {target[1]*1000:.0f}, "
              f"{target[2]*1000:.0f}) 진행 {seg['u']*100:.0f}%. 건너뛴다.")
        _next(name)
        return
    _s["q_cmd"] = q
    _go(q)

    done = seg["u"] >= 1.0 and _reached(q)
    timeout = _s["t"] >= WP_TIMEOUT
    if done or timeout:
        cur = np.asarray(_s["art"].get_joint_positions())
        err = float(np.degrees(np.max(np.abs(cur - q))))
        _s.setdefault("log", []).append(
            (name, "도달" if done else "시간초과",
             tuple(round(v * 1000) for v in target), round(err, 1),
             round(_s["t"], 2), round(_s["seg"]["u"], 2)))
        if timeout:
            print(f"[robot] ※ '{name}' 시간 초과 ({WP_TIMEOUT}s, 잔여 {err:.1f}도)")
        _next(name)


def _next(name):
    _s["i"] += 1
    _s["t"] = 0.0
    _s["q_cmd"] = None
    _s["seg"] = None
    if _s["i"] >= len(_s["wps"]):
        _s["busy"] = False


# ─────────────────────────────────────────────────────────────
# Lula IK 는 시드(웜스타트)에 민감하다. 홈 자세에서 바로 벨트 쪽을 풀면
# 실패하는데, 같은 목표를 다른 자세에서 풀면 성공한다 — 도달범위 문제가
# 아니라 순전히 시드 문제다 (실측 2026-08-22: 홈에서 '이동'·'하강'
# 웨이포인트가 전부 실패해 로봇이 가만히 선 채 사이클만 헛돌았다).
# set_default_cspace_seeds() 로 후보를 미리 깔아 둔다.
SEEDS = np.array([
    [0.0,  0.0000, -1.5708, 0.0, -1.5708, 0.0],   # 홈 (descriptor default_q)
    [0.0, -1.5708, -1.0472, 0.0, -0.5236, 0.0],   # 앞으로 뻗은 자세
    [0.0,  0.6981,  2.0944, 0.0, -1.2217, 0.0],   # verify_reach 가 쓰는 시드
    [0.0, -0.7854, -1.5708, 0.0,  0.7854, 0.0],
    [1.5708, -1.0472, -1.5708, 0.0, 0.5236, 0.0],
])


def _tcp_now():
    """지금 흡착면의 월드 위치. FK 로 tool0 를 구해 그리퍼 길이만큼 내린다."""
    try:
        pos, _ = _s["solver"].compute_end_effector_pose()
    except Exception:
        return None
    return np.array([pos[0], pos[1], pos[2] - _s["tool_len"]], dtype=float)


def _limits():
    """관절 가동범위 (lower, upper) [rad]. 솔버에서 읽고, 없으면 URDF 값."""
    lim = _s.get("limits")
    if lim is not None:
        return lim
    lo = hi = None
    try:
        lo, hi = _s["ik"].get_cspace_position_limits()
        lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    except Exception:
        pass
    if lo is None or hi is None or len(lo) != 6:
        # h2017.urdf 의 <limit lower/upper>. 솔버가 안 알려줄 때만 쓴다.
        hi = np.array([6.2832, 2.1817, 2.7925, 6.2832, 6.2832, 6.2832])
        lo = -hi
    _s["limits"] = (lo, hi)
    return _s["limits"]


def _unwind(q, cur):
    """같은 자세의 등가해 중 현재 관절각에 가장 가까운 것.

    joint_1/4/5/6 은 ±360도라 q ± 2pi 도 가동범위 안이다. 그대로 두면
    "왼쪽으로 133도" 대신 "오른쪽으로 227도" 같은 지령이 나가 팔이 감긴다.
    """
    lo, hi = _limits()
    out = np.array(q, dtype=float)
    for j in range(len(out)):
        best = out[j]
        for k in (-1, 1):
            cand = out[j] + k * 2.0 * np.pi
            if lo[j] <= cand <= hi[j] and abs(cand - cur[j]) < abs(best - cur[j]):
                best = cand
        out[j] = best
    return out


def _solve(tcp):
    """흡착면을 tcp 에 두는 관절각. 툴 z 축은 아래를 향한다. 못 풀면 None.

    시드를 **전부** 풀어 보고 현재 자세에서 가장 가까운 해를 고른다.
    처음 풀린 해를 그냥 쓰면 목표가 몇 mm 만 움직여도 가지가 바뀌어 팔이
    통째로 돈다 (docstring 의 227도 실측 참고).
    """
    # tool0 는 흡착면보다 그리퍼 길이만큼 위에 있다 (툴 z 가 아래를 보므로).
    pos = np.array([tcp[0], tcp[1], tcp[2] + _s["tool_len"]])
    # 월드 -Z 를 향하는 자세. x 축 180도 회전.
    quat = np.array([0.0, 1.0, 0.0, 0.0])
    cur = np.asarray(_s["art"].get_joint_positions(), dtype=float)

    seeds = [HOME_Q if WARM == "ready" else _s.get("q_cmd", cur)]
    seeds += [x for x in SEEDS]
    # 직전 지령과 현재 자세도 시드로 넣는다 — 경로 중간에서는 이게 가장 가깝다.
    if _s.get("q_cmd") is not None:
        seeds.append(_s["q_cmd"])
    seeds.append(cur)

    best, best_trip = None, None
    for seed in seeds:
        q, ok = _s["ik"].compute_inverse_kinematics(
            frame_name=EE_FRAME, target_position=pos,
            target_orientation=quat, warm_start=np.asarray(seed, dtype=float))
        if not ok:
            continue
        q = _unwind(np.asarray(q, dtype=float), cur)
        trip = float(np.max(np.abs(q - cur)))
        if best_trip is None or trip < best_trip:
            best, best_trip = q, trip

    if best is None:
        _s["ik_fail"] = _s.get("ik_fail", 0) + 1
        return None
    _s["ik_trip"] = float(np.degrees(best_trip))
    return best


def _go(q):
    from isaacsim.core.utils.types import ArticulationAction
    _s["art"].apply_action(ArticulationAction(joint_positions=q))


def _reached(q):
    cur = _s["art"].get_joint_positions()
    return bool(np.max(np.abs(np.asarray(cur) - q)) < JOINT_TOL)


def _dims(prim):
    """박스 로컬 치수 (w, d, h) [m].

    extent 를 먼저 보고 scale 은 있으면 곱한다 — Cube(size=1)+scale 로 만든
    박스와 scale 없는 Mesh 박스를 둘 다 받는다. scale 만 읽으면 후자가 전부
    (0.4, 0.3, 0.3) 으로 잡혀 팔레트 슬롯과 접근 높이가 어긋난다.
    """
    ext = UsdGeom.Boundable(prim).GetExtentAttr().Get()
    d = ([float(ext[1][i] - ext[0][i]) for i in range(3)] if ext
         else [0.4, 0.3, 0.3])
    a = prim.GetAttribute("xformOp:scale")
    s = a.Get() if a else None
    if s:
        d = [d[i] * float(s[i]) for i in range(3)]
    return (d[0], d[1], d[2])


# ─────────────────────────────────────────────────────────────
# 흡착 — 지금은 키네마틱으로 tool0 를 따라가게 하는 가짜다.
# 진짜 흡착(suction.py, 5.1 SurfaceGripper)으로 갈아 끼울 자리가 여기 둘이다.
def _grip(box_path):
    stage = _s["stage"]
    prim = stage.GetPrimAtPath(box_path)
    if not prim or not prim.IsValid():
        return
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    xc = UsdGeom.XformCache()
    tool = xc.GetLocalToWorldTransform(
        stage.GetPrimAtPath(f"{ROBOT_PRIM}/{EE_FRAME}"))
    box = prim.GetAttribute("xformOp:translate").Get()
    t = tool.ExtractTranslation()
    _s["held"] = (box_path, Gf.Vec3d(box[0] - t[0], box[1] - t[1], box[2] - t[2]))
    print(f"[robot] 흡착 {box_path.split('/')[-1]}")


def _release():
    held = _s.get("held")
    if held is None:
        return
    prim = _s["stage"].GetPrimAtPath(held[0])
    if prim and prim.IsValid():
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
    _s["held"] = None
    print("[robot] 놓기")


def _follow():
    """잡고 있는 박스를 tool0 에 붙여 둔다."""
    held = _s.get("held")
    if held is None:
        return
    path, off = held
    stage = _s["stage"]
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        _s["held"] = None
        return
    xc = UsdGeom.XformCache()
    t = xc.GetLocalToWorldTransform(
        stage.GetPrimAtPath(f"{ROBOT_PRIM}/{EE_FRAME}")).ExtractTranslation()
    prim.GetAttribute("xformOp:translate").Set(
        Gf.Vec3d(t[0] + off[0], t[1] + off[1], t[2] + off[2]))
