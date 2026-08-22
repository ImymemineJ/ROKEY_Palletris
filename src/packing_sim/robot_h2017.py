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

환경 변수
    PL_APPROACH_MM   접근 여유 [mm]. 기본은 사양 robot.approach_offset_mm.
    PL_JOINT_TOL_DEG 웨이포인트 도달 판정 [도]. 기본 2.0.
    PL_WP_TIMEOUT_S  한 웨이포인트 최대 대기 [초]. 기본 6.0.
"""

import os

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

import belt_cycle

ROBOT_PRIM = "/World/robot/h2017"
EE_FRAME = "tool0"

REPO = os.path.expanduser("~/palletris_ws")
DESCRIPTOR = f"{REPO}/src/packing_robot/config/h2017_descriptor.yaml"
URDF = f"{REPO}/src/packing_robot/urdf/h2017.urdf"

JOINT_TOL = np.radians(float(os.environ.get("PL_JOINT_TOL_DEG", "2.0")))
WP_TIMEOUT = float(os.environ.get("PL_WP_TIMEOUT_S", "10.0"))

# 홈 자세 — descriptor 의 default_q. 벨트와 팔레트 사이 어디에도 안 걸린다.
HOME_Q = np.array([0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0])

_s = {}


# ─────────────────────────────────────────────────────────────
def attach():
    """belt_cycle 에 자신을 꽂는다. belt_cycle.start() 전에 부른다."""
    _setup()
    belt_cycle.set_robot(on_pick=on_pick, is_idle=is_idle)
    # 웨이포인트 진행은 물리 스텝마다 봐야 한다. belt_cycle 은 PICK 단계에서
    # is_idle() 만 묻기 때문에 여기서 따로 구독을 건다.
    from omni.physx import get_physx_interface
    _s["sub"] = get_physx_interface().subscribe_physics_step_events(step)
    print("[robot] belt_cycle 에 연결됨 — 스텁 대신 H2017 이 집는다.")


def detach():
    _s["sub"] = None
    belt_cycle.set_robot(on_pick=None, is_idle=None)


def _setup():
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.robot_motion.motion_generation import (
        ArticulationKinematicsSolver, LulaKinematicsSolver)

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(ROBOT_PRIM):
        raise RuntimeError(
            f"[robot] {ROBOT_PRIM} 가 없다. 씬에 로봇이 안 올라가 있다 — "
            f"./run_layout_to_spec.sh 를 먼저 돌려라.")
    for p in (DESCRIPTOR, URDF):
        if not os.path.exists(p):
            raise RuntimeError(f"[robot] 없는 파일: {p}")

    art = SingleArticulation(prim_path=ROBOT_PRIM, name="h2017")
    art.initialize()
    ik = LulaKinematicsSolver(robot_description_path=DESCRIPTOR, urdf_path=URDF)
    ik.set_default_cspace_seeds(SEEDS)
    solver = ArticulationKinematicsSolver(art, ik, EE_FRAME)
    if list(ik.get_joint_names()) != list(art.dof_names):
        raise RuntimeError(
            f"[robot] 관절 순서가 다르다. Lula {ik.get_joint_names()} vs "
            f"아티큘레이션 {list(art.dof_names)}. _solve 의 2단계가 어긋난다.")

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

    _s.update(stage=stage, art=art, ik=ik, solver=solver,
              tool_len=tool_len, approach=approach,
              wps=[], i=0, t=0.0, held=None, placed=0, busy=False)
    print(f"[robot] Lula 준비 — 베이스 ({base_p[0]:.3f}, {base_p[1]:.3f}, "
          f"{base_p[2]:.3f})  그리퍼 {tool_len*1000:.0f}mm  "
          f"접근 여유 {approach*1000:.0f}mm")
    _go(HOME_Q)


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
    safe = max(top, drop_top) + a + 0.20          # 이송 높이

    # (목표 흡착면 z, 동작) 순서로. 흡착면이 박스 윗면에 닿아야 잡힌다.
    _s["wps"] = [
        ("이동",   (pose[0], pose[1], top + a),        None),
        ("하강",   (pose[0], pose[1], top),            None),
        ("흡착",   None,                               ("grip", box_path)),
        ("상승",   (pose[0], pose[1], safe),           None),
        ("이송",   (float(slot[0]), float(slot[1]), safe),     None),
        ("하강",   (float(slot[0]), float(slot[1]), drop_top), None),
        ("놓기",   None,                               ("release", None)),
        ("후퇴",   (float(slot[0]), float(slot[1]), safe),     None),
        ("원위치", None,                               ("home", None)),
    ]
    _s.update(i=0, t=0.0, busy=True, q_cmd=None)
    _s.setdefault("log", [])
    print(f"[robot] 사이클 시작 — {box_path.split('/')[-1]} "
          f"({pose[0]*1000:.0f}, {pose[1]*1000:.0f}) 윗면 {top*1000:.0f}mm "
          f"-> 팔레트 ({slot[0]*1000:.0f}, {slot[1]*1000:.0f}) {drop_top*1000:.0f}mm")


def is_idle():
    return not _s.get("busy", False)


# ─────────────────────────────────────────────────────────────
def step(dt):
    """물리 스텝마다 부른다 (open_test1_1 이 구독을 걸어 준다)."""
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

    q = _s.get("q_cmd")
    if q is None:
        q = _solve(target)
        if q is None:
            _s.setdefault("log", []).append(
                (name, "IK실패", tuple(round(v * 1000) for v in target), None, 0.0))
            print(f"[robot] ★ IK 실패 — '{name}' "
                  f"({target[0]*1000:.0f}, {target[1]*1000:.0f}, "
                  f"{target[2]*1000:.0f}). 이 웨이포인트를 건너뛴다.")
            _next(name)
            return
        _s["q_cmd"] = q
        _go(q)

    done, timeout = _reached(q), _s["t"] >= WP_TIMEOUT
    if done or timeout:
        cur = np.asarray(_s["art"].get_joint_positions())
        err = float(np.degrees(np.max(np.abs(cur - q))))
        _s.setdefault("log", []).append(
            (name, "도달" if done else "시간초과",
             tuple(round(v * 1000) for v in target), round(err, 1),
             round(_s["t"], 2)))
        if timeout:
            print(f"[robot] ※ '{name}' 시간 초과 ({WP_TIMEOUT}s, 잔여 {err:.1f}도)")
        _next(name)


def _next(name):
    _s["i"] += 1
    _s["t"] = 0.0
    _s["q_cmd"] = None
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


def _solve(tcp):
    """흡착면을 tcp 에 두는 관절각. 툴 z 축은 아래를 향한다. 못 풀면 None."""
    # tool0 는 흡착면보다 그리퍼 길이만큼 위에 있다 (툴 z 가 아래를 보므로).
    pos = np.array([tcp[0], tcp[1], tcp[2] + _s["tool_len"]])
    # 월드 -Z 를 향하는 자세. x 축 180도 회전.
    quat = np.array([0.0, 1.0, 0.0, 0.0])

    # 1) 현재 자세에서 (관절 순서를 알아서 맞춰 준다)
    action, ok = _s["solver"].compute_inverse_kinematics(
        target_position=pos, target_orientation=quat)
    if ok:
        return np.array(action.joint_positions, dtype=float)

    # 2) 시드를 하나씩 명시해서 다시. Lula 관절 순서가 아티큘레이션과 같은
    #    것은 _setup() 에서 확인해 둔다.
    for seed in SEEDS:
        q, ok = _s["ik"].compute_inverse_kinematics(
            frame_name=EE_FRAME, target_position=pos,
            target_orientation=quat, warm_start=seed)
        if ok:
            _s["ik_reseed"] = _s.get("ik_reseed", 0) + 1
            return np.array(q, dtype=float)

    _s["ik_fail"] = _s.get("ik_fail", 0) + 1
    return None


def _go(q):
    from isaacsim.core.utils.types import ArticulationAction
    _s["art"].apply_action(ArticulationAction(joint_positions=q))


def _reached(q):
    cur = _s["art"].get_joint_positions()
    return bool(np.max(np.abs(np.asarray(cur) - q)) < JOINT_TOL)


def _dims(prim):
    a = prim.GetAttribute("xformOp:scale")
    s = a.Get() if a else None
    return (float(s[0]), float(s[1]), float(s[2])) if s else (0.4, 0.3, 0.3)


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
