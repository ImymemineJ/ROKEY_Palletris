"""공장 투입구 피더 — Play 하면 박스가 하나씩 벨트 위로 떨어진다 (기본 10개, 12초 간격).

Isaac Sim GUI 에서 돌린다.

    ./run_spawn_feeder.sh                       # test1.usd 를 열고 피더를 붙인다
    PL_N=20 PL_INTERVAL=2 ./run_spawn_feeder.sh
    PL_STAGE=~/isaac_scenes/project_1/test1-1.usd ./run_spawn_feeder.sh   # 벨트 1개 씬

이미 스테이지를 열어 둔 상태라면 Script Editor 에 이 파일 내용을 붙여넣어도 된다
(현재 열린 스테이지가 test1.usd 면 다시 열지 않는다).

동작
  · Play 중 물리 스텝에서 시간을 누적해 PL_INTERVAL 초마다 박스 1개 생성.
  · 호수는 3/4/5호 를 50:30:20 비율로 랜덤, 요(yaw)와 가로 위치도 랜덤.
    가로 지터는 회전 후 실제 발자국으로 상한을 계산해 실측한 벨트 폭 밖으로
    걸치지 않게 한다 (폭은 씬마다 다르다 — test1.usd 722mm, test1-1.usd 1546mm).
  · 색은 호수 고정 — 3호 빨강 / 4호 초록 / 5호 파랑.
    test1.usd 에 이미 있는 /World/Looks/BoxNo{3,4,5} 머티리얼을 쓰고,
    없으면 만든다. displayColor 도 같이 넣는다.
  · 박스는 **scale 없는 Mesh** 다. 치수를 지오메트리에 직접 굽는다 — _make_box 주석 참고.
  · Stop 하면 스폰한 박스를 전부 지우고 카운터를 리셋한다. 다시 Play 하면 처음부터.

흔들림 (PL_JITTER, 기본 1)
  0 으로 두면 가로 지터와 요 흔들림을 끈다. 박스가 항상 벨트 한가운데에
  0도 또는 90도 로 반듯하게 떨어진다. 픽 지점까지 벨트가 세워 주면 로봇이
  볼 자세가 완전히 고정되므로, 로봇 사이클을 붙일 때 이렇게 쓴다.

투입 위치 (PL_SPAWN)
  스테이지에서 벨트 메시 bbox 를 재서 폭 중심 · 상면 · 진입단을 그때그때 구한다.
  씬에서 벨트를 옮기거나 높이를 바꿔도 따라간다.
  cube   : 공장 투입구 (/World/ConveyorTrack, 기본). x=0 쪽에서 들어와 -X 로
           흐른다. 진입단에서 0.5m 안쪽에 떨어뜨린다.
             투입 -> -X 로 주행 -> x=-8 A03 커브 -> 북상 (본선 x=-9.50)
             -> SorterBlue -> SorterGreen -> SorterRed -> ConveyorTrack_01
  feeder : 본선 시작점 (/World/ConveyorTrack_03). +Y 로 흐른다. 커브를 건너뛰고
           바로 북상시켜 소터 구간만 볼 때 쓴다.

낙하 높이 (PL_DROP_H, 기본 0.30m)
  실측한 벨트 상면 기준이다. 0.30m 면 착지 2.4m/s 라 9kg 5호도 안 튄다.
  2026-08-21 실측: 라인 전 구간이 1780mm 계열이다 (A06 벨트 1780.5mm,
  소터 롤러 1756~1759mm, ConveyorTrack_04 롤러 1737.5mm). 예전 900mm 평탄화
  버전이 아니다. 그래서 상면을 상수로 박지 않고 매번 잰다.
"""

import math
import os
import random

import omni.timeline
import omni.usd
from omni.physx import get_physx_interface
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

# PL_STAGE 로 다른 USD 를 지정할 수 있다 (예: 벨트 1개짜리 test1-1.usd).
DEFAULT_STAGE = "/home/rokey/isaac_scenes/project_1/test1.usd"
STAGE_PATH = os.environ.get("PL_STAGE", DEFAULT_STAGE)

N_BOXES = int(os.environ.get("PL_N", 10))
# 박스 사이 간격 [초]. 사양 stage1.spawn_interval_s 값이다.
# 예전 기본값은 3초였는데, 벨트가 픽 지점에서 멈춰 로봇 사이클(10초)을
# 기다리는 방식으로 바뀌면서 3초로는 벨트에 박스가 계속 쌓였다.
# 사이클보다 길게 잡아야 줄이 늘어나지 않는다 (12 > 10).
INTERVAL = float(os.environ.get("PL_INTERVAL", 12.0))
SEED = os.environ.get("PL_SEED")                       # 없으면 매번 다른 랜덤
SPAWN = os.environ.get("PL_SPAWN", "cube")
DROP_H = float(os.environ.get("PL_DROP_H", "0.30"))   # 벨트면 위 낙하 높이 [m]
# 벨트 위에 동시에 올려둘 박스 수. 0 이면 무제한 (기존 동작).
# 1 로 두면 한 개씩 — 로봇이 그 박스를 집어 올려 벨트에서 빠져야 다음이 들어온다.
# 팔레타이징 사이클(집기 -> 놓기 -> 복귀)에 투입을 맞추는 용도다.
MAX_ON_BELT = int(os.environ.get("PL_MAX_ON_BELT", 0))
# 가로 위치와 요(yaw) 를 얼마나 흔들지. 1 이면 기존대로 랜덤, 0 이면 항상
# 벨트 한가운데에 반듯하게 떨어뜨린다. 로봇이 고정된 픽 자세 하나로 집게
# 하려면 0 이다 (belt_cycle 이 x 를 고정해 주고, 이게 y 와 yaw 를 고정한다).
JITTER = float(os.environ.get("PL_JITTER", "1"))

BOX_ROOT = "/World/Boxes"
LOOKS_ROOT = "/World/Looks"
CUBE_PATH = "/World/Cube"

BOX_SPECS = {
    "3호": (0.340, 0.250, 0.210),
    "4호": (0.410, 0.310, 0.280),
    "5호": (0.480, 0.380, 0.340),
}
BOX_MASS = {"3호": 3.0, "4호": 5.0, "5호": 9.0}
BOX_RATIO = {"3호": 0.50, "4호": 0.30, "5호": 0.20}
BOX_COLOR = {"3호": (1.0, 0.0, 0.0), "4호": (0.0, 1.0, 0.0), "5호": (0.0, 0.0, 1.0)}
BOX_SLUG = {"3호": "No3", "4호": "No4", "5호": "No5"}

# 투입 지점은 상수로 박지 않는다. 스테이지에서 벨트 메시를 직접 재서 구한다.
# 예전엔 벨트면 z=0.900, 폭 중심 (-0.478, 0.035) 로 박아 뒀는데 씬을 1780mm
# 높이 계열로 되돌린 뒤로 그 좌표가 벨트면보다 470mm 아래를 가리켰다. 박스가
# 벨트 안쪽에서 생성돼 튕겨 나가거나 바닥으로 떨어졌다 — 스폰이 안 되던 원인.
#
# 흐름 방향도 씬에서 읽는다. ConveyorNode 의 inputs:direction 을 데크 프림의
# 월드 회전으로 돌려서 쓴다. 아래 표의 축·부호는 그래프를 못 찾을 때만 쓰는
# 기본값이다.
#   test1.usd    /World/ConveyorTrack = 8.4m A06, 상면 1780mm, x=0 에서 -X 로 흐른다
#   test1-1.usd  /World/ConveyorTrack = 5m A05,   상면  800mm, x=0 에서 +X 로 흐른다
# 이름이 같은 프림이 씬마다 반대로 흐른다. 부호를 박아 두면 반드시 한쪽이
# 깨진다 — 실제로 +1 로 박아 두는 바람에 test1.usd 에서 박스가 투입구가 아니라
# 8m 떨어진 토출단(커브 앞)에 떨어졌다. 그게 "스폰이 안 된다" 의 정체였다.
#
#   이름: (조각 프림, (흐름 축, 부호 기본값), 진입단에서 안쪽으로 넣을 거리 [m])
SPAWN_POINTS = {
    # 공장 투입구. 진입단에서 0.50m 안쪽 = 사양의 spawn_point_mm 다.
    "cube":   ("/World/ConveyorTrack",    ("x", +1), 0.50),
    # 로봇까지 이어지는 본선 시작점. ConveyorTrack_03 은 +Y 로 흐른다.
    "feeder": ("/World/ConveyorTrack_03", ("y", +1), 0.40),
}

# 박스가 닿는 면의 하위 프림 이름. 에셋 계열마다 다르다 —
# A05/A02/A43 (770mm 계열) 은 Rollers, A06/A03 (1780mm 계열) 은 Belt.
DECK_NAMES = ("Rollers", "Belt")

# 벨트 실측값. start() -> _measure() 가 채운다.
# 예전에는 실측이 실패하면 기록해 둔 상수로 넘어갔는데, 그 상수가 다른 씬의
# 것이면 박스를 허공이나 벨트 밑에 만들어 놓고 조용히 지나간다. 씬을 바꿀
# 때마다 이걸로 깨졌다. 이제는 못 재면 끊는다.
BELT_TOP_Z = 0.0
# 게이팅 판정용 벨트 흐름축 범위 [m]. _measure() 가 실측값으로 채운다.
BELT_FLOW_RANGE = None
BELT_HALF_W = 0.0
EDGE_MARGIN = 0.08       # 벨트 가장자리 여유 [m]
YAW_JITTER_DEG = 15.0

_state = {}


# ─────────────────────────────────────────────────────────────
def start():
    stop()

    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if stage is None or stage.GetRootLayer().identifier != STAGE_PATH:
        print(f"[feeder] 스테이지 열기: {STAGE_PATH}")
        ctx.open_stage(STAGE_PATH)
        stage = ctx.get_stage()

    center, axis = _measure(stage)
    drop_z = BELT_TOP_Z + DROP_H

    # 투입구에 있던 테스트 큐브는 비활성화한다 (스폰 박스와 부딪히므로).
    # 파일을 고치는 게 아니라 세션에서만 꺼지고, Ctrl+S 하면 저장된다.
    cube = stage.GetPrimAtPath(CUBE_PATH)
    if cube and cube.IsActive():
        cube.SetActive(False)
        print(f"[feeder] {CUBE_PATH} 비활성화 (되살리려면 Property 창에서 active 체크)")

    _ensure_materials(stage)
    if not stage.GetPrimAtPath(BOX_ROOT):
        UsdGeom.Xform.Define(stage, BOX_ROOT)

    _state.update(
        stage=stage, center=center, axis=axis, drop_z=drop_z,
        rng=random.Random(int(SEED)) if SEED is not None else random.Random(),
        elapsed=INTERVAL, spawned=0, paths=[],
    )

    _state["physics_sub"] = get_physx_interface().subscribe_physics_step_events(_on_step)
    _state["timeline_sub"] = (
        omni.timeline.get_timeline_interface()
        .get_timeline_event_stream()
        .create_subscription_to_pop(_on_timeline)
    )

    print(f"[feeder] 투입구 '{SPAWN}' ({center[0]:.3f}, {center[1]:.3f}, {drop_z:.3f})  "
          f"낙하 {DROP_H:.2f}m -> 착지 {math.sqrt(2 * 9.81 * DROP_H):.1f}m/s")
    print(f"[feeder] Play 를 누르면 {INTERVAL:.1f}초마다 1개씩, 총 {N_BOXES}개 투입한다.")


def stop():
    """구독 해제 + 스폰한 박스 제거."""
    if _state.get("physics_sub") is not None:
        _state["physics_sub"] = None
    if _state.get("timeline_sub") is not None:
        _state["timeline_sub"] = None
    _clear()
    _state.clear()


# ─────────────────────────────────────────────────────────────
def _on_belt_count():
    """아직 벨트 위에 남아 있는 박스 수.

    로봇이 집어 올리면 z 가 벨트면 위로 뜨고, 팔레트에 놓으면 흐름축 범위
    밖으로 나간다. 둘 중 하나만 봐서는 안 된다 — 적재가 높아지면 팔레트 위
    박스도 벨트면(800mm)보다 높아지므로 흐름축 범위를 같이 본다.
    """
    stage = _state.get("stage")
    if stage is None or BELT_FLOW_RANGE is None:
        return 0
    lo, hi, i_flow = BELT_FLOW_RANGE
    n = 0
    for path in _state.get("paths", []):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        attr = prim.GetAttribute("xformOp:translate")
        t = attr.Get() if attr else None
        if t is None:
            continue
        # 낙하 중인 박스도 세야 한다 (벨트면 위 DROP_H 까지).
        if lo <= t[i_flow] <= hi and BELT_TOP_Z - 0.10 <= t[2] <= BELT_TOP_Z + DROP_H + 0.5:
            n += 1
    return n


def _on_step(dt):
    if _state.get("spawned", 0) >= N_BOXES:
        return
    _state["elapsed"] += dt
    if _state["elapsed"] < INTERVAL:
        return
    # 벨트에 이미 MAX_ON_BELT 개가 올라가 있으면 기다린다. 로봇이 하나 집어
    # 올려 벨트에서 빠져야 다음이 들어온다. elapsed 는 계속 쌓아 두므로
    # 자리가 나는 즉시 투입된다.
    if MAX_ON_BELT > 0:
        n = _on_belt_count()
        if n >= MAX_ON_BELT:
            if _state.get("waiting") != n:
                _state["waiting"] = n
                print(f"[feeder] 대기 — 벨트 위 {n}/{MAX_ON_BELT}개. "
                      f"하나 빠지면 다음 박스를 넣는다.")
            return
        _state["waiting"] = None
    _state["elapsed"] = 0.0
    _spawn_one()


def _on_timeline(event):
    if event.type == int(omni.timeline.TimelineEventType.STOP):
        # 되감기: 스폰한 박스를 지우고 처음부터 다시 시작할 수 있게 한다
        _clear()
        _state["spawned"] = 0
        _state["elapsed"] = INTERVAL
        print("[feeder] Stop — 투입 박스 제거, 카운터 리셋")


def _clear():
    stage = _state.get("stage")
    if stage is None:
        return
    for path in _state.get("paths", []):
        if stage.GetPrimAtPath(path):
            stage.RemovePrim(path)
    _state["paths"] = []


# ─────────────────────────────────────────────────────────────
def _flow_dir(stage, track_path, deck, default_axis, default_sign):
    """벨트가 실제로 어느 쪽으로 흐르는지 (축, 부호) 를 씬에서 읽는다.

    ConveyorNode 의 inputs:direction 은 conveyorPrim 의 로컬 프레임 기준이다.
    조각을 Z 180도 돌려 놓은 씬이 흔해서 같은 (1,0,0) 이 씬에 따라 +X 도 되고
    -X 도 된다. test1.usd 와 test1-1.usd 가 정확히 그 관계다.
    """
    node, local = None, None
    track = stage.GetPrimAtPath(track_path)
    for graph in track.GetChildren():
        if "ConveyorBeltGraph" not in graph.GetName():
            continue
        for child in graph.GetChildren():
            attr = child.GetAttribute("inputs:direction")
            v = attr.Get() if attr else None
            if v is not None and Gf.Vec3d(v[0], v[1], v[2]).GetLength() > 1e-9:
                node, local = child, Gf.Vec3d(v[0], v[1], v[2])
                break
        if node is not None:
            break

    if node is None:
        print(f"[feeder] {track_path} 에서 ConveyorNode 를 못 찾았다 — "
              f"기본 흐름 {default_axis}{'+' if default_sign > 0 else '-'} 를 쓴다")
        return default_axis, default_sign

    frame = deck if UsdGeom.Xformable(deck) else track
    world = UsdGeom.Xformable(frame).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).TransformDir(local)

    axis = "x" if abs(world[0]) >= abs(world[1]) else "y"
    sign = 1 if world[0 if axis == "x" else 1] >= 0 else -1
    arrow = f"{axis}{'+' if sign > 0 else '-'}"
    print(f"[feeder] 흐름 실측 {node.GetPath()}  "
          f"로컬 ({local[0]:+.0f},{local[1]:+.0f},{local[2]:+.0f}) -> 월드 {arrow}")
    if (axis, sign) != (default_axis, default_sign):
        print(f"[feeder]   표의 기본값 "
              f"{default_axis}{'+' if default_sign > 0 else '-'} 와 다르다 — 씬을 따른다")
    return axis, sign


def _measure(stage):
    """투입 벨트 메시를 재서 (중심 xy, 폭 방향 축) 을 돌려준다.

    같이 BELT_TOP_Z (벨트 상면) 와 BELT_HALF_W (반폭) 를 실측값으로 채운다.
    bbox 는 purpose 를 전부 넣고 잡는다 — default 만 넣으면 proxy 로 잡힌
    충돌 메시가 빠져서 빈 범위가 나온다.
    """
    global BELT_TOP_Z, BELT_HALF_W, BELT_FLOW_RANGE
    if SPAWN not in SPAWN_POINTS:
        raise ValueError(f"PL_SPAWN 은 {' | '.join(SPAWN_POINTS)}: {SPAWN!r}")

    track_path, (flow_axis, flow_sign), inset = SPAWN_POINTS[SPAWN]

    # 벨트가 없으면 좌표를 찍을 근거가 없다. 조용히 넘어가면 박스가 바닥으로
    # 떨어지므로 여기서 끊는다.
    if not stage.GetPrimAtPath(track_path):
        raise RuntimeError(
            f"[feeder] 투입 벨트 {track_path} 가 이 스테이지에 없다 "
            f"(PL_SPAWN={SPAWN!r}, 스테이지={STAGE_PATH}). "
            f"이 스테이지에 있는 투입구를 PL_SPAWN 으로 고르거나 "
            f"벨트가 있는 USD 를 PL_STAGE 로 열어라.")

    prim, prim_path = None, None
    for deck in DECK_NAMES:
        p = stage.GetPrimAtPath(f"{track_path}/{deck}")
        if p and p.IsValid():
            prim, prim_path = p, f"{track_path}/{deck}"
            break
    if prim is None:
        raise RuntimeError(
            f"[feeder] {track_path} 밑에 {' 도 '.join(DECK_NAMES)} 도 없다 — "
            f"박스가 닿을 면을 못 찾는다. 에셋 계열이 바뀌었으면 "
            f"DECK_NAMES 에 이름을 추가해라.")

    try:
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
             UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide])
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    except Exception as exc:
        raise RuntimeError(f"[feeder] {prim_path} 측정 실패 ({exc})") from exc
    if r.IsEmpty():
        # 예전엔 여기서 기록해 둔 상수로 넘어갔다. 그 상수가 다른 씬 것이면
        # 박스가 벨트 밑이나 허공에 생겨 조용히 사라진다. 끊는 게 낫다.
        raise RuntimeError(
            f"[feeder] {prim_path} 의 bbox 가 비었다 — 벨트 에셋(레퍼런스)이 "
            f"로드되지 않았을 가능성이 크다. 스테이지={STAGE_PATH}")

    # 흐름 방향은 씬에서 읽는다 (표의 부호는 못 읽을 때만 쓴다).
    flow_axis, flow_sign = _flow_dir(stage, track_path, prim, flow_axis, flow_sign)
    cross_axis = "y" if flow_axis == "x" else "x"

    mn, mx = r.GetMin(), r.GetMax()
    i_flow = 0 if flow_axis == "x" else 1
    i_cross = 1 - i_flow
    # 진입단 = 흐름의 반대쪽 끝. 거기서 inset 만큼 안쪽에 떨어뜨린다.
    entry = mx[i_flow] if flow_sign < 0 else mn[i_flow]
    along = float(entry + flow_sign * inset)
    cross_c = float((mn[i_cross] + mx[i_cross]) / 2.0)

    BELT_TOP_Z = float(mx[2])
    BELT_HALF_W = float((mx[i_cross] - mn[i_cross]) / 2.0)
    BELT_FLOW_RANGE = (float(mn[i_flow]), float(mx[i_flow]), i_flow)
    center = (along, cross_c) if flow_axis == "x" else (cross_c, along)

    print(f"[feeder] 벨트 실측 {prim_path}")
    print(f"[feeder]   {flow_axis} {mn[i_flow]:.3f}~{mx[i_flow]:.3f}m  "
          f"진입단 {entry:.3f}  상면 {BELT_TOP_Z * 1000:.0f}mm  "
          f"폭 {BELT_HALF_W * 2 * 1000:.0f}mm")
    print(f"[feeder]   투입 중심 ({center[0]:.3f}, {center[1]:.3f})")
    return center, cross_axis


def _spawn_one():
    stage, rng = _state["stage"], _state["rng"]
    cx, cy = _state["center"]
    axis = _state["axis"]

    name = rng.choices(list(BOX_SPECS), weights=[BOX_RATIO[k] for k in BOX_SPECS])[0]
    dims = BOX_SPECS[name]

    yaw = (90.0 if rng.random() < 0.5 else 0.0) + \
        rng.uniform(-YAW_JITTER_DEG, YAW_JITTER_DEG) * JITTER
    c, s = abs(math.cos(math.radians(yaw))), abs(math.sin(math.radians(yaw)))
    # 벨트를 가로지르는 방향의 반폭 (진행축에 따라 x/y 가 바뀐다)
    half = (dims[0] * c + dims[1] * s) / 2.0 if axis == "x" else (dims[0] * s + dims[1] * c) / 2.0
    jitter = max(0.0, BELT_HALF_W - EDGE_MARGIN - half)
    off = rng.uniform(-jitter, jitter) * JITTER

    # (cx, cy) 는 실측한 벨트 폭 중심이므로 off 만 걸면 항상 벨트 안이다.
    x = cx + off if axis == "x" else cx
    y = cy if axis == "x" else cy + off
    z = _state["drop_z"] + dims[2] / 2.0

    i = _state["spawned"]
    path = f"{BOX_ROOT}/Box_{i:02d}_{BOX_SLUG[name]}"
    _make_box(stage, path, name, dims, (x, y, z), yaw)

    _state["paths"].append(path)
    _state["spawned"] = i + 1
    print(f"[feeder] {i + 1:2d}/{N_BOXES}  {name}  "
          f"{dims[0] * 1000:.0f}x{dims[1] * 1000:.0f}x{dims[2] * 1000:.0f}mm  "
          f"{BOX_MASS[name]:.0f}kg  yaw={yaw:+6.1f}도  pos=({x:.3f}, {y:.3f}, {z:.3f})")


def _make_box(stage, path, name, dims, pos, yaw_deg):
    """박스 하나. dims = (w, d, h) [m].

    치수를 지오메트리에 직접 굽는다. 직육면체가 필요한데 `UsdGeom.Cube` 는 한 변만
    지정할 수 있어서, 점 8개를 직접 박은 `Mesh` 로 만들고 `xformOp:scale` 은 안 쓴다.

    ⚠ **2026-08-22 정정 — 원래 여기 적혀 있던 이유는 틀렸다.**
      "대상 프림에 scale 이 걸리면 흡착 시 박스가 42.8 mm 떠서 붙는다" 고 적었었다.
      흡착 그리퍼를 구 저장소 정본(`h2017_cell.usd`)대로 다시 만든 뒤 실측하니
      **scale 은 흡착에 아무 영향이 없다.** 같은 조건에서 소수점까지 같은 값이 나온다:

          3호  흡착 후 간극 +1.24 mm · 300 mm 상승 시 +299.8 mm   (Mesh = Cube+scale)
          5호  흡착 후 간극 +1.20 mm · 300 mm 상승 시 +299.0 mm   (Mesh = Cube+scale)

      42.8 mm 의 진짜 원인은 **그리퍼 쪽 패드 D6 구성**이었다 — `isaac:forwardAxis`
      미지정(기본이 X 라 옆을 본다)과 6축 하드 잠금(구속이 생긴 자세 그대로 굳는다).
      **흡착이 이상하면 박스 scale 이 아니라 그리퍼 패드부터 볼 것.**
      재현: `scripts/verify_suction.py`, 구성: `scripts/build_gripper_usd.py`

    이 코드를 되돌리지는 않는다. 치수를 지오메트리에 굽는 편이 어차피 깔끔하고,
    실측상 손해도 없다. 고칠 것은 이유뿐이다.
    """
    hx, hy, hz = dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0
    mesh = UsdGeom.Mesh.Define(stage, path)
    prim = mesh.GetPrim()

    pts = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
           (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    mesh.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1,  4, 5, 6, 7,  0, 1, 5, 4,
                                      1, 2, 6, 5,  2, 3, 7, 6,  3, 0, 4, 7])
    mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
    mesh.CreateSubdivisionSchemeAttr("none")     # 없으면 모서리가 둥글어진다
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*BOX_COLOR[name])])

    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    half = math.radians(yaw_deg) / 2.0
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half))))
    # ★ AddScaleOp 을 넣지 않는다. 넣어야 한다면 반드시 (1, 1, 1) 이어야 한다.

    UsdPhysics.CollisionAPI.Apply(prim)
    # 직육면체라 convexHull 근사 오차가 없다.
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    PhysxSchema.PhysxCollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(BOX_MASS[name])

    mat = UsdShade.Material(stage.GetPrimAtPath(f"{LOOKS_ROOT}/Box{BOX_SLUG[name]}"))
    if mat:
        UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(mat)


def _ensure_materials(stage):
    if not stage.GetPrimAtPath(LOOKS_ROOT):
        UsdGeom.Scope.Define(stage, LOOKS_ROOT)
    for name, rgb in BOX_COLOR.items():
        mat_path = f"{LOOKS_ROOT}/Box{BOX_SLUG[name]}"
        if stage.GetPrimAtPath(mat_path):
            continue
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        print(f"[feeder] 머티리얼 생성 {mat_path}")


start()
