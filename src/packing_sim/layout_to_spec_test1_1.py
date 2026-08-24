#!/usr/bin/env python3
"""test1-1.usd 를 system.yaml 좌표계로 재배치한다 (존 B 기준).

씬과 사양이 서로 다른 세계였다. 사양은 라인이 x=500 -> 10000 으로 +X 로
흐르고 로봇이 x=10900 에 있는데, 씬은 x=-8324 -> 0 으로 -X 로 흘렀다.
좌우가 뒤집힌 채 19 m 떨어져 있었다. 씬을 사양에 맞춘다.

하는 일
  1) 벨트 조각을 **사양이 정한 구간**에 맞춰 x 스케일을 다시 잡는다.
     조각 0 = stage1.main_range_mm (0~7500), 조각 1 = pickup_range_mm (7500~10000).
  2) 벨트를 x=0 에서 시작하도록 옮기고 흐름을 +X 로 돌린다.
  3) 스토퍼 벽을 라인 끝(stopper_x_mm)에 세운다 — 콜라이더만, RigidBody 없음.
     박스 앞면이 여기 닿아 선다. 벨트 정지는 흡착 직전 떨림을 없애는 용도다.
  4) 페데스탈 D400 x H(pedestal_height) 원기둥을 로봇 베이스 자리에 세운다.
  5) 팔레트를 pallet_origin 에 놓는다 (원점은 모서리, 상면이 origin.z).
  6) 팔레트 반출 컨베이어 상판을 **도크 바닥(z=0)** 과 같은 면에 깐다.
  7) H2017 을 페데스탈 위(로봇 베이스 좌표)에 올린다.

★★ z = 0 은 **도크 바닥 = 트럭 적재함 바닥**이다 (작업명세서 §3.1).
   팔레트 밑면 z=0 · 팔레트 상면 z=150 · 반출 롤러 상면 z=0 · 야드 지면 z=-850.
   `truck.bed_height_mm`(850) 은 **야드 지면 대비 도크 높이**이지 월드 z 가 아니다.
   ⚠ 2026-08-22 이전 판은 이 값을 팔레트 z 로 써서 팔레트를 z 850~1000 에 띄웠고,
     그러면 적재 상단이 도달범위 밖으로 나간다 (필요 2193 > 실효 1550).
     구 사양서 §5-1 이 "받침대 1,580 이 필요하다 -> 도크로 해결한다" 로 이미 기각한
     방향이다. 그래서 여기서는 bed_height_mm 을 높이로 쓰지 않는다.

★ 벨트를 왜 균등 분할하지 않나 — `set_velocity("b_pickup")` 이 세우는 구간이
   곧 조각 하나이기 때문이다. 사양 §3.2 의 픽업 구간은 7500~10000 (2500 mm) 이고
   그 폭에서 버퍼 4개 x 480 = 1920 ≤ 2500 이 성립한다. 균등 분할(5000/5000)로 두면
   벨트를 세울 때 5000 mm 가 통째로 서서 그 계산이 무의미해진다.

높이(800mm)와 속도(0.5m/s)는 set_belt_800_test1_1.py 가 이미 맞춰 놨다.
여기서는 건드리지 않는다.

돌리는 법 — system.yaml 을 읽어 넘겨줘야 하므로 래퍼를 쓴다.
    ./run_layout_to_spec.sh

    (Isaac 의 kit python 에는 yaml 이 없고, yaml 이 있는 시스템 python3 에는
     pxr 이 없다. 래퍼가 시스템 python3 로 값을 뽑아 PL_SPEC 으로 넘긴다.)

벨트 고유 치수는 실측 상수다. 에셋을 바꾸면 다시 재야 한다.

2026-08-22 개정 — 사용자가 GUI 에서 벨트 조각을 하나 더 붙이고 폭을 넓혀
저장했다 (scale.y 0.80 -> 1.71). 그 작업을 살리도록 고쳤다.
  * 조각이 여러 개면 사양 구간을 균등 분할해 이어 붙인다 (TRACKS).
  * 폭(scale.y)은 사용자가 정한 값을 그대로 쓰고, 모든 조각에 통일한다.
    벨트가 좁아 박스가 튕겨 나가던 문제의 대책이라 덮어쓰면 안 된다.
  * GUI 붙여넣기 사고로 /World 밖에 생긴 조각은 지운다 (STRAY).
"""

import json
import math
import os
import shutil
import time

from pxr import Gf, Sdf

USD_PATH = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")
ZONE = os.environ.get("PL_ZONE", "B")
# 로봇 베이스 y 를 사양값 대신 지정한다 [mm]. 벨트 정중앙(y=0)이 아니라
# 라인 옆에서 구동하고 싶을 때 쓴다. 사양 zones.robot_base_world_mm 와
# 어긋나므로 값을 바꾸면 팀에 알리고 system.yaml 도 맞춰야 한다.
ROBOT_Y_MM = os.environ.get("PL_ROBOT_Y")
# 팔레트 원점 y 도 같이 옮긴다. 로봇을 라인 옆으로 내리면 팔레트도 따라
# 내려야 네 모서리가 리치 안에 들어온다 (안 내리면 대각선 먼 모서리가
# 1780mm 로 리치 1700mm 밖이다 — 실측).
PALLET_Y_MM = os.environ.get("PL_PALLET_Y")

SPEC = json.loads(os.environ["PL_SPEC"])

# 라인을 이루는 조각들. **순서대로** 아래 SEG_KEYS 의 구간을 하나씩 맡는다.
#   /World/ConveyorTrack  = "main"      (0 ~ 7500)
#   /World/ConveyorTrack1 = "b_pickup"  (7500 ~ 10000)
# 이 순서가 conveyor_ctl.py 의 seg -> 그래프 경로 표와 짝이다. 바꾸면 둘 다 고친다.
TRACKS = ["/World/ConveyorTrack", "/World/ConveyorTrack1"]
SEG_KEYS = ["main", "b_pickup"]
# GUI 붙여넣기 사고로 /World 밖 루트에 생긴 조각. 지운다.
STRAY = ["/ConveyorTrack1"]
ENDSTOP_ROOT = "/World/EndStops"    # 옛 이름. 지우기만 한다.
STOPPER = "/World/Stopper"
PEDESTAL = "/World/Pedestal"
OUTFEED = "/World/PalletOutfeed"
ROBOT = "/World/robot"
CAM_PICK = "/World/CamPick"               # 픽업 하향 카메라 (하우징)
CAM_PRIM = CAM_PICK + "/Camera"           # 실제 Camera 프림
GRIPPER = ROBOT + "/gripper"              # 흡착 그리퍼 몸통 (link_6 에 고정조인트)
# H2017 **흡착 그리퍼가 붙은 판** — 로봇 트랙(B)이 만든 정본이다.
#   ~/isaac_assets/h2017_suction.usd
#     /h2017/SurfaceGripper  [IsaacSurfaceGripper]  coaxial 1e6 · maxGripDistance 20mm
#     /h2017/gripper/{body, visuals/pad_0..3, mount, pads/pad_0..3}
#   만드는 법은 scripts/build_gripper_usd.py (없으면 그걸 돌려 다시 만든다).
#
# ⚠ 예전 기본값이던 isaac_scenes/robot/h2017.usd 는 **그리퍼가 없는 맨팔**이다.
#   그걸 쓰면 robot_motion.py 가 SurfaceGripper 프림을 못 찾아 grasp 가 아예 안 된다
#   (실측 2026-08-22: 프림 69개에 SurfaceGripper·pad 0개).
#
# 참조하면 default prim(/h2017)의 내용이 ROBOT 아래에 그대로 깔린다:
#   /World/robot/root_joint        아티큘레이션 루트
#   /World/robot/root_joint/tool0  EE 프레임
#   /World/robot/SurfaceGripper    흡착
ROBOT_USD = os.environ.get(
    "PL_ROBOT_USD", os.path.expanduser("~/isaac_assets/h2017_suction.usd"))
PALLET = "/World/Pallet"
LOOKS = "/World/Looks"
PHYS_MAT = LOOKS + "/PhysicsMaterial"

# Isaac Sim 헤드리스 실측 (2026-08-22), A05 로 교체하고 scale=(4.180, 0.8019, 1.0399)
# 일 때의 /World/ConveyorTrack/Rollers 월드 bbox. 스케일당 길이를 뽑는 데 쓴다.
MEAS_SCALE_X = 4.180428897430741
MEAS_ROLLER_X0, MEAS_ROLLER_X1 = -8.3241, -0.0166   # orient 180 이라 음수쪽
ROLLER_LEN_PER_SCALE = (MEAS_ROLLER_X1 - MEAS_ROLLER_X0) / MEAS_SCALE_X
# 조각 원점에서 롤러 끝(토출단)까지의 거리 / 스케일
ROLLER_NEAR_PER_SCALE = -MEAS_ROLLER_X1 / MEAS_SCALE_X
# 같은 실측에서의 폭. scale.y 를 바꾸면 폭이 비례한다.
MEAS_SCALE_Y, MEAS_ROLLER_W = 0.8019239600482583, 0.7241
ROLLER_W_PER_SCALE = MEAS_ROLLER_W / MEAS_SCALE_Y

PED_DIAM = 0.40      # D400 원기둥

mm = lambda v: v / 1000.0

belt_x1 = mm(SPEC["conveyor"]["stopper_x_mm"])            # 벨트 토출단 = 스토퍼
belt_x0 = 0.0                                             # 사양 라인 시작
belt_len = belt_x1 - belt_x0

# 조각별 구간 [x0, x1] (m). 사양 §3.2 를 그대로 읽는다 — 균등 분할이 아니다.
#   main     = conveyor.stage1.main_range_mm   [0,    7500]
#   b_pickup = conveyor.pickup_range_mm        [7500, 10000]
_conv = SPEC["conveyor"]
SEG_RANGE_M = {
    "main": [mm(v) for v in _conv["stage1"]["main_range_mm"]],
    "b_pickup": [mm(v) for v in _conv["pickup_range_mm"]],
}
# 사양 자체가 어긋나 있으면 씬을 만들기 전에 멈춘다. 벨트에 틈이 생기거나
# 겹치면 박스가 그 자리에서 걸리는데, 씬을 다 만든 뒤에는 원인을 찾기 어렵다.
_m, _p = SEG_RANGE_M["main"], SEG_RANGE_M["b_pickup"]
if abs(_m[1] - _p[0]) > 1e-9:
    raise SystemExit(
        f"사양 불일치: main 끝 {_m[1]*1000:.0f} != pickup 시작 {_p[0]*1000:.0f} mm. "
        "system.yaml 의 conveyor.stage1.main_range_mm 와 pickup_range_mm 를 맞춰라.")
if abs(_p[1] - belt_x1) > 1e-9:
    raise SystemExit(
        f"사양 불일치: pickup 끝 {_p[1]*1000:.0f} != stopper_x {belt_x1*1000:.0f} mm.")
base = [mm(v) for v in SPEC["zones"]["robot_base_world_mm"][ZONE]]
spec_base_y = base[1]
if ROBOT_Y_MM is not None:
    base[1] = mm(float(ROBOT_Y_MM))
porg = [mm(v) for v in SPEC["zones"]["pallet_origin_world_mm"][ZONE]]
pal = SPEC["pallet"]
spec_pallet_y = porg[1]
if PALLET_Y_MM is not None:
    porg[1] = mm(float(PALLET_Y_MM))
ped_h = mm(SPEC["robot"]["pedestal_height_mm"])
BELT_MPS = float(SPEC["conveyor"]["belt_velocity_mps"])
# orient 180° 조각에서 월드 +X 로 흘리려면 로컬 direction.x 는 -1 이다.
DIR_LOCAL_X = -1.0

layer = Sdf.Layer.FindOrOpen(USD_PATH)
if layer is None:
    raise SystemExit(f"USD 를 못 열었다: {USD_PATH}")
backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}")
print(f"존 {ZONE}  벨트 {belt_x0*1000:.0f}~{belt_x1*1000:.0f}mm  "
      f"로봇베이스 {base}  팔레트원점 {porg}\n")


def define(path, type_name, api=None):
    spec = Sdf.CreatePrimInLayer(layer, Sdf.Path(path))
    spec.specifier = Sdf.SpecifierDef
    spec.typeName = type_name
    if api:
        spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(api))
    return spec


def put(spec, name, tn, value, uniform=False):
    """속성을 쓴다. **이미 있으면 값만 바꾼다.**

    ★ 예전 판은 무조건 `Sdf.AttributeSpec(...)` 을 만들어서, 같은 프림을 두 번
      건드리면 "already exists" 로 죽었다. `drop()` 으로 지웠다 다시 만드는
      프림(스토퍼·팔레트 등)은 괜찮았지만, 물리 머티리얼처럼 지우지 않는 것에서
      **빌더를 두 번 돌리면 터졌다** (2026-08-23 실측). 씬 빌더는 몇 번을 돌려도
      같은 결과가 나와야 한다.
    """
    a = spec.attributes.get(name)
    if a is None:
        var = Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying
        a = Sdf.AttributeSpec(spec, name, tn, var)
    a.default = value
    return a


def xform(spec, pos, scale, quat=(1, 0, 0, 0)):
    put(spec, "xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*pos))
    put(spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd, Gf.Quatd(*quat))
    put(spec, "xformOp:scale", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*scale))
    put(spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray,
        ["xformOp:translate", "xformOp:orient", "xformOp:scale"], uniform=True)


def bind_phys(path):
    """`path` 프림에 물리 머티리얼을 건다 (purpose=physics).

    ★ 씬에 PhysicsMaterial 프림이 **하나도 없었다.** /World/Looks 아래 있던 셋은
      전부 UsdPreviewSurface 시각 머티리얼이라 마찰과 무관하다. 즉 박스·팔레트·
      스토퍼·벨트가 전부 PhysX 기본 마찰로 돌고 있었고, system.yaml 의
      static 0.6 / dynamic 0.5 를 읽는 코드가 저장소에 없었다.
      그런데 위 스토퍼 전도 한계 검사(h > w/(2mu))는 그 mu 를 전제로 벽 높이
      100 mm 를 정당화한다 — 전제가 씬에서 성립하지 않고 있었다.
    """
    ps = layer.GetPrimAtPath(Sdf.Path(path))
    if ps is None:
        print(f"  물리 머티리얼 건너뜀 — {path} 가 이 레이어에 없다")
        return
    # ★ apiSchemas 는 **리스트옵**이다. 원래 형태를 반드시 지켜야 한다.
    #   /World/ConveyorTrack/Rollers 는 참조 에셋 위의 `over` 이고, 루트 레이어에는
    #   `prepended = [PhysxSurfaceVelocityAPI]` 만 들어 있다. 여기에 explicit 리스트를
    #   써 넣으면 **약한 레이어(참조 에셋)의 apiSchemas 를 통째로 막는다** — 롤러의
    #   RigidBody/Collision 이 사라져 벨트가 콜라이더를 잃는다.
    #   실측 2026-08-23: CreateExplicit 로 썼더니 OmniGraph 가
    #   "Selected Prim is not a Rigid Body" 를 내고 박스가 벨트를 뚫고 바닥(z 0.14)에
    #   떨어졌다. 그래서 explicit 이면 explicit 에, 아니면 prepended 에 덧붙인다.
    op = ps.GetInfo("apiSchemas") if ps.HasInfo("apiSchemas") else Sdf.TokenListOp()
    if "MaterialBindingAPI" not in op.GetAddedOrExplicitItems():
        if op.isExplicit:
            ps.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(
                list(op.explicitItems) + ["MaterialBindingAPI"]))
        else:
            new_op = Sdf.TokenListOp()
            new_op.prependedItems = list(op.prependedItems) + ["MaterialBindingAPI"]
            new_op.appendedItems = list(op.appendedItems)
            new_op.deletedItems = list(op.deletedItems)
            ps.SetInfo("apiSchemas", new_op)
    rel = ps.relationships.get("material:binding:physics")
    if rel is None:
        rel = Sdf.RelationshipSpec(ps, "material:binding:physics", False)
    rel.targetPathList.explicitItems = [Sdf.Path(PHYS_MAT)]


def drop(parent_path, name):
    parent = layer.GetPrimAtPath(Sdf.Path(parent_path))
    if parent is not None and name in parent.nameChildren:
        del parent.nameChildren[name]
        print(f"  기존 {parent_path}/{name} 제거")


# ── 1) 벨트 조각들을 사양 구간에 이어 붙인다 ────────────────
# GUI 사고로 /World 밖에 생긴 조각부터 치운다.
for stray in STRAY:
    sp = Sdf.Path(stray)
    parent = layer.GetPrimAtPath(sp.GetParentPath())
    if parent is not None and sp.name in parent.nameChildren:
        del parent.nameChildren[sp.name]
        print(f"루트 사고 조각 {stray} 제거")

tracks = [p for p in TRACKS if layer.GetPrimAtPath(Sdf.Path(p)) is not None]
if not tracks:
    raise SystemExit(f"벨트 조각이 하나도 없다: {TRACKS}")
if len(tracks) != len(SEG_KEYS):
    raise SystemExit(
        f"조각 {len(tracks)}개인데 사양 구간은 {len(SEG_KEYS)}개다 ({SEG_KEYS}). "
        "GUI 에서 조각을 더 붙였거나 지웠다면 TRACKS/SEG_KEYS 를 같이 고쳐라.")

# 폭은 사용자가 GUI 에서 정한 값(첫 조각)을 기준으로 모든 조각에 통일한다.
belt_sy = float(Gf.Vec3d(layer.GetAttributeAtPath(
    Sdf.Path(tracks[0]).AppendProperty("xformOp:scale")).default)[1])
belt_w = ROLLER_W_PER_SCALE * belt_sy
for _t in TRACKS:
    bind_phys(_t + "/Rollers")           # 박스가 실제로 닿는 면
print(f"조각 {len(tracks)}개 · 폭 {belt_w*1000:.0f}mm "
      f"(scale.y {belt_sy:.4f}, 사용자 값 유지)\n")

for i, tp in enumerate(tracks):
    path = Sdf.Path(tp)
    prim = layer.GetPrimAtPath(path)
    t = layer.GetAttributeAtPath(path.AppendProperty("xformOp:translate"))
    s = layer.GetAttributeAtPath(path.AppendProperty("xformOp:scale"))
    o = layer.GetAttributeAtPath(path.AppendProperty("xformOp:orient"))
    old_t, old_s = Gf.Vec3d(t.default), Gf.Vec3d(s.default)

    seg_key = SEG_KEYS[i]
    seg_x0, seg_x1 = SEG_RANGE_M[seg_key]     # 이 조각이 맡는 사양 구간
    seg_len = seg_x1 - seg_x0
    new_sx = seg_len / ROLLER_LEN_PER_SCALE
    # 월드 속도 = graph:variable:Velocity x scale.x 다 (실측 2026-08-22:
    # scale.x 2.5161 · Velocity 0.5 -> 1.2588 m/s, 이득 2.518 = scale.x).
    # 예전에 "A05 는 이득 1.0" 이라고 적어 둔 것은 오측이었다.
    # ★ 조각마다 scale.x 가 다르므로 Velocity 도 **조각마다** 달라야 한다.
    #   구간을 균등 분할하던 때에는 하나로 충분했지만 7500/2500 에서는 아니다.
    #   한 값으로 두면 픽업 벨트가 본선의 3배 속도로 돌아 박스가 튀어 나간다.
    belt_vel = round(BELT_MPS / new_sx, 6)

    # orient 180 이라 조각이 원점에서 -X 로 뻗는다. 토출단을 seg_x1 에 두려면
    # 원점을 그만큼 +X 로 민다.
    new_tx = seg_x1 + ROLLER_NEAR_PER_SCALE * new_sx

    t.default = Gf.Vec3d(new_tx, 0.0, 0.0)
    s.default = Gf.Vec3d(new_sx, belt_sy, old_s[2])
    # GUI 에서 조각을 복제하면 orient 가 틀어져 있을 수 있다. 180° 로 통일한다.
    if o is not None:
        o.default = Gf.Quatd(6.123233995736766e-17, Gf.Vec3d(0.0, 0.0, 1.0))
    print(f"{tp}   seg=\"{seg_key}\"")
    print(f"      롤러 {seg_x0*1000:.0f}~{seg_x1*1000:.0f}mm ({seg_len*1000:.0f})   "
          f"scale {old_s[0]:.4f},{old_s[1]:.4f} -> {new_sx:.4f},{belt_sy:.4f}   "
          f"translate.x {old_t[0]:.4f} -> {new_tx:.4f}")
    print(f"      속도 {BELT_MPS} m/s -> Velocity {belt_vel} (= {BELT_MPS} / {new_sx:.4f})")

    # 흐름을 월드 +X 로.
    #
    # 부호를 두 번 틀렸다. orient 가 180° 라 조각의 로컬 X 가 월드 -X 를
    # 향한다. inputs:direction 은 로컬 축 기준이므로 월드 +X 로 흘리려면
    # direction.x = -1 이다.
    # (실측 2026-08-22: +1 로 두면 박스가 -X 로 끌려가 입구 끝 x=0 으로
    #  떨어졌다. "박스가 벨트에서 튕겨 나간다" 던 증상의 정체가 이것이다.
    #  벨트 폭이 좁아서가 아니었다. -1 로 바꾸니 5호를 90° 돌려 놓아도,
    #  y 를 150mm 치우쳐 놓아도 밑면 z=800 을 유지하며 얌전히 흘러간다.)
    for g in prim.nameChildren:
        if g.typeName != "OmniGraph":
            continue
        gpath = path.AppendChild(g.name)
        v = layer.GetAttributeAtPath(gpath.AppendProperty("graph:variable:Velocity"))
        if v is not None:
            v.default = belt_vel
        for node in layer.GetPrimAtPath(gpath).nameChildren:
            npath = gpath.AppendChild(node.name)
            d = layer.GetAttributeAtPath(npath.AppendProperty("inputs:direction"))
            if d is None:
                continue
            old_d = Gf.Vec3f(d.default)
            d.default = Gf.Vec3f(DIR_LOCAL_X, old_d[1], old_d[2])
            # 롤러가 도는 그림(animateDirection)은 물리 방향과 부호를 맞춘다.
            # 따로 놀면 눈에는 반대로 도는 것처럼 보인다.
            a = layer.GetAttributeAtPath(npath.AppendProperty("inputs:animateDirection"))
            if a is not None:
                a.default = Gf.Vec2f(DIR_LOCAL_X, 0.0)
            print(f"      흐름 {tuple(old_d)} -> {tuple(d.default)} (월드 +X)")
print()

# ── 2) 스토퍼 벽 ─────────────────────────────────────────────
# 설계결정서 §4.4 · 작업명세서 §5.3 · 트랙A §A4 가 모두 "컨베이어 끝에 스토퍼
# (Cube 에 Collider 만, Rigid Body 없음)" 로 정해 둔 것이다.
#
# ⚠ 한때 이 자리에서 벽을 지우고 "벨트를 멈춰서 세운다" 로 갔었다. 두 가지가 틀렸다:
#
#   1) **아무도 박스를 못 세운다.** `spawner.pickup_check` 는 박스가 *이미 멎어야*
#      present 를 주고 `cycle.py:_run` 은 present 를 받은 *뒤에* 벨트를 세운다.
#      서로를 기다린다 (2026-08-22 실측: 검증 드라이버가 대신 끊어야 사이클이 돌았다).
#   2) **픽업 위치가 상수가 아니게 된다.** 감속으로 세우면 서는 자리가 감속거리만큼
#      매번 달라진다. 상수로 만들어 주는 것이 벽이다 (설계결정서 §4.4 표
#      "픽업 위치가 상수 — IK 목표가 고정값"). 벽을 지운 옛 주석은 이 인과를
#      거꾸로 적고 있었다.
#
# 벨트 정지는 벽을 대신하는 것이 아니라 벽과 **같이** 쓴다 — 벽이 세우고,
# `cycle.py` 가 흡착 전에 벨트를 꺼서 떨림을 없앤다. 벽은 정지가 늦어도 박스가
# 라인을 넘지 못하게 하는 안전망이다(설계결정서 §4.4 표).
drop("/World", "EndStops")          # 옛 이름. 남아 있으면 치운다.
drop("/World", "Stopper")
STOP_T = 0.10                       # 두께 100 mm (트랙A §A4)
# ★★ 높이는 §A4 의 300 이 아니라 **100** 이다. 300 으로 세우면 벽 상단이 z=1100 —
#    로봇 베이스(z=900)보다 200 높다. 그런데 로봇은 벽 **너머**(픽 지점 x≈9800)를
#    집어야 하므로 팔이 늘 벽 위를 지나간다. 팔레트에 놓고 다음 박스를 집으러 돌아올
#    때 호 경로가 z≈1.02 로 가라앉는데, 거기서 벽에 막혔다:
#      2026-08-23 실측 (300 mm 벽) — 1번째 박스는 READY 에서 수직 하강이라 통과.
#      2번째부터 매번 "도착 잔차 491.8 mm (허용 1.0)" 로 서고 전부 BLOCKED.
#      흡착면이 (10.284, 0.007, 1.020) 에서 멈췄다 = 그리퍼 앞끝이 벽면(10.100)에 닿는 자리.
#      놓아 둔 박스까지 79.6 mm 밀렸다.
#    §A4 의 300 은 로봇을 세워 보기 전에 적은 값이다 (그 때 페데스탈은 700 이라
#    벽이 베이스보다 250 높았으니 더 나빴다).
#
#    **막는 데 300 이 필요하지도 않다.** 벨트 마찰이 박스를 벽 위로 넘기려면
#    h_wall > w / (2·μ) 여야 한다 (벽 상단 모서리를 축으로 한 전도 조건).
#    μ=0.5 · 가장 좁은 박스 w=340 이면 한계는 340 mm 다 — 100 mm 는 한참 아래고,
#    오히려 **높을수록 위험하다**. 100 이면 벽 상단 z=900 = 로봇 베이스 평면이라
#    팔 경로와도 안 겹친다.
STOP_H = 0.10
# 폭은 트랙A §A4 가 700 이라고 적었는데, 그건 그 때 **벨트 폭이 720** 이었기 때문이다
# (scale.y 0.80). 사용자가 GUI 에서 벨트를 넓힌 뒤로는 700 이 벨트를 다 못 막는다.
# 원래 의도가 "벨트를 가로지르는 벽" 이므로 벨트 폭을 따라가되 사양값을 하한으로 둔다.
STOP_W = max(0.70, belt_w)
belt_top = mm(_conv["belt_top_z_mm"])
_bt = SPEC["boxes"]["types"]
_h_max, _d_max, _w_max = (max(t[k] for t in _bt) / 1000.0 for k in ("h_mm", "d_mm", "w_mm"))
_w_min = min(t["w_mm"] for t in _bt) / 1000.0
# ★ 기본값으로 조용히 떨어지면 안 된다. 셸이 `physics` 키를 안 넘기던 동안
#   이 검사는 system.yaml 이 아니라 리터럴 0.5 로 판정하고 있었다 — 마찰을
#   0.3 으로 낮춰도 스토퍼 전도 한계는 340 mm 그대로 나온다. 조용히 틀린 답을
#   내는 게이트는 없는 게이트보다 나쁘다 (철칙 3).
try:
    _mat = SPEC["physics"]["material"]
    _mu = float(_mat["dynamic_friction"])
    _mu_s = float(_mat["static_friction"])
except KeyError as _e:
    raise SystemExit(
        f"PL_SPEC 에 physics.material 이 없다 ({_e}) — "
        "run_layout_to_spec.sh 의 키 튜플을 확인하라.")
# 1) 너무 높으면 벨트 마찰이 박스를 벽 위로 **전도**시킨다 (벽 상단 모서리가 축).
_tip_h = _w_min / (2.0 * _mu)
if STOP_H >= _tip_h:
    raise SystemExit(
        f"스토퍼 높이 {STOP_H*1000:.0f} 가 전도 한계 {_tip_h*1000:.0f} mm 이상이다 "
        f"(가장 좁은 박스 {_w_min*1000:.0f} · 마찰 {_mu}) — 박스가 벽을 타고 넘는다.")
# 2) 벽 상단이 로봇 베이스 평면을 넘으면 팔이 픽 지점으로 돌아오다 벽에 막힌다
#    (2026-08-23 실측, 위 STOP_H 주석).
if belt_top + STOP_H > base[2] + 1e-9:
    raise SystemExit(
        f"스토퍼 상단 {(belt_top+STOP_H)*1000:.0f} 가 로봇 베이스 "
        f"{base[2]*1000:.0f} 보다 높다 — 팔이 벽 너머를 집으러 갈 때 막힌다.")
# 벽이 좁으면 y 로 치우친 박스가 옆으로 빠져나간다. 90° 돌아간 5호까지 본다.
if STOP_W < max(_d_max, _w_max):
    raise SystemExit(
        f"스토퍼 폭 {STOP_W*1000:.0f} 가 가장 큰 박스 최대 변"
        f"({max(_d_max, _w_max)*1000:.0f}) 보다 좁다.")
# ── 물리 씬 플래그 ──────────────────────────────────────────
# ★ 씬을 pxr 로 읽어 보니 사양과 **정반대**였다: enableStabilization=False
#   (사양 true) · enableCCD=True (사양 false). 그리고 이 두 속성을 쓰는 코드가
#   저장소에 한 줄도 없었다 — system.yaml 의 physics.scene 을 아무도 안 읽었다.
#   stabilization 을 켜라는 이유가 §5.4 상 정지 스택 붕괴 방지이고, G2 합격선이
#   "4단 30초 방치 후 위치변화 < 3 mm" 다. 그 조건이 이 씬에서 성립한 적이 없다.
#   라인 드라이버(demo_stack15_test1_1.py)는 World() 를 안 만들고 타임라인만
#   돌리므로, **여기 씬에 쓴 값이 곧 실행 값**이다.
_ps = SPEC["physics"]["scene"]
_pscene = layer.GetPrimAtPath(Sdf.Path("/physicsScene"))
if _pscene is None:
    _pscene = define("/physicsScene", "PhysicsScene",
                     ["PhysxSceneAPI"])
for _n, _v in (("physxScene:enableStabilization", bool(_ps["enable_stabilization"])),
               ("physxScene:enableCCD", bool(_ps["enable_ccd"]))):
    _a = _pscene.attributes.get(_n)
    if _a is None:
        _a = Sdf.AttributeSpec(_pscene, _n, Sdf.ValueTypeNames.Bool,
                               Sdf.VariabilityVarying)
    _old = _a.default
    _a.default = _v
    print(f"물리 씬 {_n.split(':')[-1]} {_old} -> {_v}")
_a = _pscene.attributes.get("physxScene:timeStepsPerSecond")
if _a is None:
    _a = Sdf.AttributeSpec(_pscene, "physxScene:timeStepsPerSecond",
                           Sdf.ValueTypeNames.UInt, Sdf.VariabilityVarying)
_a.default = int(_ps["time_steps_per_second"])
print(f"물리 씬 timeStepsPerSecond {_a.default}")

if layer.GetPrimAtPath(Sdf.Path(LOOKS)) is None:
    define(LOOKS, "Scope")
_pm = define(PHYS_MAT, "Material", ["PhysicsMaterialAPI"])
put(_pm, "physics:staticFriction", Sdf.ValueTypeNames.Float, _mu_s)
put(_pm, "physics:dynamicFriction", Sdf.ValueTypeNames.Float, _mu)
put(_pm, "physics:restitution", Sdf.ValueTypeNames.Float, 0.0)
print(f"물리 머티리얼 {PHYS_MAT}  정지 {_mu_s} · 운동 {_mu} · 반발 0.0")

spec = define(STOPPER, "Cube", ["PhysicsCollisionAPI", "PhysxCollisionAPI"])
put(spec, "size", Sdf.ValueTypeNames.Double, 1.0)
put(spec, "extent", Sdf.ValueTypeNames.Float3Array,
    [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
put(spec, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
    [Gf.Vec3f(0.85, 0.25, 0.12)])
put(spec, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, True)
# ★ RigidBodyAPI 를 붙이지 않는다 — 붙이면 박스가 벽을 밀고 나간다 (작업명세서 §5.3).
#   박스 **앞면**이 닿는 면이 stopper_x 여야 하므로 벽 중심은 두께의 반만큼 뒤다.
#   밑면은 벨트 상면에 얹는다.
xform(spec, (belt_x1 + STOP_T / 2.0, 0.0, belt_top + STOP_H / 2.0),
      (STOP_T, STOP_W, STOP_H))
bind_phys(STOPPER)
print(f"스토퍼 벽 x {belt_x1*1000:.0f}~{(belt_x1+STOP_T)*1000:.0f}  "
      f"y +-{STOP_W/2*1000:.0f}  z {belt_top*1000:.0f}~{(belt_top+STOP_H)*1000:.0f}  "
      f"(콜라이더만 · RigidBody 없음)")
print(f"      박스 앞면이 x={belt_x1*1000:.0f} 에 선다 -> 픽업 중심 = "
      f"{belt_x1*1000:.0f} - w/2 (5호 {belt_x1*1000 - _w_max*1000/2:.0f})")

# ── 3) 페데스탈 (D400 원기둥) ────────────────────────────────
drop("/World", "Pedestal")
spec = define(PEDESTAL, "Cylinder", ["PhysicsCollisionAPI", "PhysxCollisionAPI"])
put(spec, "axis", Sdf.ValueTypeNames.Token, "Z", uniform=True)
put(spec, "radius", Sdf.ValueTypeNames.Double, PED_DIAM / 2.0)
put(spec, "height", Sdf.ValueTypeNames.Double, ped_h)
put(spec, "extent", Sdf.ValueTypeNames.Float3Array,
    [Gf.Vec3f(-PED_DIAM / 2, -PED_DIAM / 2, -ped_h / 2),
     Gf.Vec3f(PED_DIAM / 2, PED_DIAM / 2, ped_h / 2)])
put(spec, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
    [Gf.Vec3f(0.35, 0.35, 0.38)])
put(spec, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, True)
# 바닥(0)부터 베이스 높이까지. 중심 z = 높이/2.
xform(spec, (base[0], base[1], ped_h / 2.0), (1.0, 1.0, 1.0))
bind_phys(PEDESTAL)
print(f"페데스탈 ({base[0]:.3f}, {base[1]:.3f})  D{PED_DIAM*1000:.0f} x H{ped_h*1000:.0f}  "
      f"상면 z={ped_h:.3f} (로봇 베이스)")


# 도달 판정은 여기서 하지 않는다. 예전에는 어깨축~목표점 유클리드 거리로
# 대충 봤는데, 로봇 트랙의 scripts/verify_reach.py 와 답이 갈렸다 (그쪽은
# URDF 관절 한계 + 충돌구체로 수치 IK 를 풀고 수직 하강 자세까지 제약한다).
# 판정 출처가 둘이면 반드시 어긋난다. 그쪽 하나만 쓴다.
if ROBOT_Y_MM is not None or PALLET_Y_MM is not None:
    print(f"  ※ system.yaml 값(로봇 y={spec_base_y*1000:.0f} · "
          f"팔레트 y={spec_pallet_y*1000:.0f})을 환경변수로 덮어썼다 -> "
          f"{base[1]*1000:.0f} · {porg[1]*1000:.0f}. 씬과 사양이 어긋난 상태다.")
print(f"  도달 판정은 로봇 트랙 게이트로: "
      f"python3 ~/palletris_ws/scripts/verify_reach.py --zone {ZONE}")

# ── 4) 팔레트 ────────────────────────────────────────────────
drop("/World", "Pallet")
spec = define(PALLET, "Cube", ["PhysicsCollisionAPI", "PhysxCollisionAPI"])
put(spec, "size", Sdf.ValueTypeNames.Double, 1.0)
put(spec, "extent", Sdf.ValueTypeNames.Float3Array,
    [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
put(spec, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
    [Gf.Vec3f(0.55, 0.40, 0.22)])
put(spec, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, True)
pw, pd, pt = mm(pal["width_mm"]), mm(pal["depth_mm"]), mm(pal["thickness_mm"])
# porg 는 모서리이고 z 는 상면이다. 중심으로 환산한다.
xform(spec, (porg[0] + pw / 2.0, porg[1] + pd / 2.0, porg[2] - pt / 2.0),
      (pw, pd, pt))
bind_phys(PALLET)
print(f"팔레트 모서리 ({porg[0]:.3f}, {porg[1]:.3f})  {pw*1000:.0f}x{pd*1000:.0f}x{pt*1000:.0f}  "
      f"상면 z={porg[2]:.3f}")

# ── 4b) 픽업 하향 카메라 (RealSense D455 상정) ────────────────
# ★ 에셋 라이브러리가 원격이라 이 PC 에서는 rsd455.usd 를 못 받아온다
#   (컨베이어 참조도 같은 이유로 못 푼다). 그래서 **같은 외형 치수의 하우징**을
#   그리고 그 안에 Camera 프림을 넣는다. 광학값(해상도·화각)은 system.yaml 이
#   정본이라 겉모습만 대역이고 계측은 진짜다.
# ⚠ 하우징에는 콜라이더를 안 건다. 팔이 바로 아래를 지나간다 — 물리적으로
#   부딪히면 안 된다. 높이는 팔의 최대 도달(TCP 1.75 + 그리퍼 0.15 = 1.90)보다
#   위여야 하고, system.yaml 이 2.30 으로 잡아 뒀다 (여유 400 mm).
_rec = SPEC.get("recognition", {})
_pc = _rec.get("pick_cam", {})
if _pc.get("enabled"):
    # 박스 **앞면**이 스토퍼에 닿아 서므로 중심은 종류마다 다르다
    # (5호 10000-240=9760 ~ 3호 10000-170=9830). 그 가운데를 본다.
    cam_x = belt_x1 - (_w_max + _w_min) / 4.0
    cam_y = mm(_pc.get("y_mm", 0))
    cam_z = mm(_pc.get("z_mm", 2300))
    if cam_z <= base[2] + 1.0:
        raise SystemExit(
            f"카메라 z {cam_z*1000:.0f} 가 팔이 지나는 높이에 걸린다 "
            f"(로봇 베이스 {base[2]*1000:.0f} + 이송 여유 1000).")
    hw, hd, hh = (v / 1000.0 for v in _pc.get("housing_mm", [124, 26, 29]))

    drop("/World", "CamPick")
    spec = define(CAM_PICK, "Xform")
    xform(spec, (cam_x, cam_y, cam_z), (1.0, 1.0, 1.0))

    # 하우징 — 보이기만 한다. 콜라이더 없음.
    spec = define(CAM_PICK + "/Body", "Cube")
    put(spec, "size", Sdf.ValueTypeNames.Double, 1.0)
    put(spec, "extent", Sdf.ValueTypeNames.Float3Array,
        [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    put(spec, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
        [Gf.Vec3f(0.10, 0.10, 0.12)])
    xform(spec, (0.0, 0.0, 0.0), (hw, hd, hh))

    # Camera — 회전을 안 주면 USD 규약상 **-Z 를 본다** = 똑바로 아래.
    #   광학 원점을 하우징 아래면에 둔다.
    spec = define(CAM_PRIM, "Camera")
    res = _rec.get("resolution", [1280, 720])
    hfov = float(_rec.get("hfov_deg", 60))
    # 수평 화각 -> 초점거리. 센서 가로를 USD 기본 36 mm 로 두고 환산한다.
    ap_h = 36.0
    focal = ap_h / (2.0 * math.tan(math.radians(hfov) / 2.0))
    put(spec, "focalLength", Sdf.ValueTypeNames.Float, focal)
    put(spec, "horizontalAperture", Sdf.ValueTypeNames.Float, ap_h)
    put(spec, "verticalAperture", Sdf.ValueTypeNames.Float,
        ap_h * float(res[1]) / float(res[0]))
    put(spec, "clippingRange", Sdf.ValueTypeNames.Float2, Gf.Vec2f(0.05, 10.0))
    xform(spec, (0.0, 0.0, -hh / 2.0), (1.0, 1.0, 1.0))

    _cover = 2.0 * (cam_z - belt_top) * math.tan(math.radians(hfov) / 2.0)
    print(f"픽업 카메라 ({cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f})  {_pc.get('model')} "
          f"{res[0]}x{res[1]} hfov {hfov:.0f} deg  f={focal:.1f}mm")
    print(f"      벨트면까지 {(cam_z-belt_top)*1000:.0f} mm · 가로 시야 "
          f"{_cover*1000:.0f} mm (가장 긴 박스 {_w_max*1000:.0f} 보다 넓다)"
          f"  콜라이더 없음")

# ── 5) 팔레트 반출 컨베이어 상판 ─────────────────────────────
# 적재한 팔레트를 **들지 않고 그대로 밀어** 트럭에 넣는다. 그래서 팔레트 밑면과
# 트럭 바닥의 높이차가 0 이어야 하는데, z=0 도크 규약이 그것을 이미 만족한다
# (작업명세서 §3.1) — 팔레트 밑면 z=0 이 곧 트럭 적재함 바닥이다.
# 반출 롤러 상면도 **z=0** 이다. 바닥에 얹는 구조물이 아니라 바닥과 같은 면이라
# 상판을 z 0 아래로 묻는다.
#
# ⚠ 여기에 truck.bed_height_mm(850) 을 쓰지 않는다. 그건 **야드 지면 대비**
#   도크 높이이지 월드 z 가 아니다. 예전 판이 이걸 높이로 써서 팔레트를
#   z 850~1000 에 띄웠고, 그러면 적재 상단이 도달범위 밖으로 나간다.
#   (구 사양서 §5-1 이 "받침대 1,580 필요 -> 도크로 해결" 로 기각한 방향이다.)
#
# 지금은 상판만 있는 정적 콜라이더다. 롤러/체인 구동은 아직 안 넣었다 —
# 반출 시퀀스를 만들 때 여기에 physxSurfaceVelocity 를 걸면 된다.
drop("/World", "PalletOutfeed")
DOCK_Z = 0.0             # 도크 바닥 = 트럭 적재함 바닥 = 반출 롤러 상면
pallet_bottom = porg[2] - mm(pal["thickness_mm"])
if abs(pallet_bottom - DOCK_Z) > 1e-6:
    raise SystemExit(
        f"팔레트 밑면이 {pallet_bottom*1000:.0f}mm 다 — 도크 바닥(0)이 아니다. "
        f"system.yaml 의 pallet_origin_world_mm[{ZONE}].z 는 "
        f"pallet.thickness_mm({pal['thickness_mm']}) 여야 한다 (작업명세서 §3.1).")
pw_, pd_ = mm(pal["width_mm"]), mm(pal["depth_mm"])
OF_T = 0.15          # 상판 두께
OF_MARGIN = 0.05     # 팔레트보다 옆으로 내민 양
OF_RUN = 1.00        # 반출 방향(+X)으로 더 뻗은 길이
ox0, ox1 = porg[0] - OF_MARGIN, porg[0] + pw_ + OF_RUN
oy0, oy1 = porg[1] - OF_MARGIN, porg[1] + pd_ + OF_MARGIN
spec = define(OUTFEED, "Cube", ["PhysicsCollisionAPI", "PhysxCollisionAPI"])
put(spec, "size", Sdf.ValueTypeNames.Double, 1.0)
put(spec, "extent", Sdf.ValueTypeNames.Float3Array,
    [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
put(spec, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
    [Gf.Vec3f(0.30, 0.32, 0.36)])
put(spec, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, True)
# 상면이 DOCK_Z 이므로 중심은 그보다 두께의 반만큼 아래다.
xform(spec, ((ox0 + ox1) / 2.0, (oy0 + oy1) / 2.0, DOCK_Z - OF_T / 2.0),
      (ox1 - ox0, oy1 - oy0, OF_T))
bind_phys(OUTFEED)
print(f"반출 컨베이어 x {ox0*1000:.0f}~{ox1*1000:.0f}  "
      f"y {oy0*1000:.0f}~{oy1*1000:.0f}  "
      f"상면 z={DOCK_Z*1000:.0f} (도크 바닥 = 트럭 적재함 바닥)")

# ── 6) H2017 ────────────────────────────────────────────────
# 베이스는 페데스탈 상면이다. base_link 가 프림 원점이므로 그대로 얹으면 된다.
drop("/World", "robot")
if os.path.exists(ROBOT_USD):
    spec = Sdf.CreatePrimInLayer(layer, Sdf.Path(ROBOT))
    spec.specifier = Sdf.SpecifierDef
    spec.typeName = "Xform"
    spec.referenceList.prependedItems.append(Sdf.Reference(ROBOT_USD))
    xform(spec, (base[0], base[1], base[2]), (1.0, 1.0, 1.0))
    print(f"로봇 {ROBOT} <- {ROBOT_USD}")
    print(f"      베이스 ({base[0]:.3f}, {base[1]:.3f}, {base[2]:.3f}) "
          f"= 페데스탈 상면")
else:
    print(f"로봇 건너뜀 — {ROBOT_USD} 가 없다. "
          f"h2017/HANDOFF.md 의 make_h2017_usd.py 로 만들어라.")

# ── 6b) 손목 카메라 (RealSense D455 상정) ────────────────────
# ★ 반드시 로봇 블록 **뒤**여야 한다 — 위 `drop("/World", "robot")` 이 /World/robot
#   스펙을 통째로 지우므로, 그 아래에 먼저 써 두면 같이 날아간다.
# ★ tool0 는 참조 에셋 안에 있는 링크다. 루트 레이어에서 그 아래에 프림을 새로
#   정의하면 합성 결과에 얹힌다. 물리는 안 붙인다 — 링크에 콜라이더나 질량을
#   더하면 아티큘레이션 동역학이 바뀐다.
# ★ 흡착 방향이 tool0 의 **+Z** 다 (`robot_motion` 이 흡착면을 p + R[:,2]*grip_len
#   으로 잡는다). USD 카메라는 자기 로컬 **-Z** 를 보므로, X 축으로 180도 돌려
#   카메라의 -Z 가 부모의 +Z 를 향하게 한다. 쿼터니언 (w,x,y,z) = (0,1,0,0).
_wc = _rec.get("wrist_cam", {})
if _wc.get("enabled") and os.path.exists(ROBOT_USD):
    CAM_PARENT = _wc.get("parent", GRIPPER)
    CAM_WRIST = CAM_PARENT + "/CamWrist"
    CAM_WRIST_PRIM = CAM_WRIST + "/Camera"
    wo = [v / 1000.0 for v in _wc.get("offset_mm", [170, 0, 30])]
    whw, whd, whh = (v / 1000.0 for v in _wc.get("housing_mm", [124, 26, 29]))
    # ★ 시선 기울기. 툴 +Y 축 둘레로 -tilt 만큼 돌리면 시선이 툴 축 쪽으로 기운다.
    #   쿼터니언 (w,x,y,z) = (cos(t/2), 0, -sin(t/2), 0).
    tilt = math.radians(float(_wc.get("tilt_deg", 18)))
    wq = (math.cos(tilt / 2.0), 0.0, -math.sin(tilt / 2.0), 0.0)

    drop(CAM_PARENT, "CamWrist")
    spec = define(CAM_WRIST, "Xform")
    xform(spec, (wo[0], wo[1], wo[2]), (1.0, 1.0, 1.0), quat=wq)

    spec = define(CAM_WRIST + "/Body", "Cube")
    put(spec, "size", Sdf.ValueTypeNames.Double, 1.0)
    put(spec, "extent", Sdf.ValueTypeNames.Float3Array,
        [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    put(spec, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
        [Gf.Vec3f(0.10, 0.10, 0.12)])
    # 하우징은 두께 방향이 툴 z (시선) 가 되게 눕힌다.
    # ★ 긴 변(124)을 **접선 방향**으로 돌린다. 반경 방향으로 두면 ±62 mm 라
    #   그리퍼 몸통(반경 122.8)을 파고든다 — 카메라 중심을 170 으로 빼도 하우징
    #   안쪽 끝이 108 이라 여전히 박힌다. 접선으로 눕히면 반경 폭이 ±14.5 다.
    if _wc.get("housing_tangential", True):
        xform(spec, (0.0, 0.0, 0.0), (whh, whw, whd))
    else:
        xform(spec, (0.0, 0.0, 0.0), (whw, whh, whd))

    spec = define(CAM_WRIST_PRIM, "Camera")
    res = _rec.get("resolution", [1280, 720])
    hfov = float(_rec.get("hfov_deg", 60))
    ap_h = 36.0
    focal = ap_h / (2.0 * math.tan(math.radians(hfov) / 2.0))
    put(spec, "focalLength", Sdf.ValueTypeNames.Float, focal)
    put(spec, "horizontalAperture", Sdf.ValueTypeNames.Float, ap_h)
    put(spec, "verticalAperture", Sdf.ValueTypeNames.Float,
        ap_h * float(res[1]) / float(res[0]))
    put(spec, "clippingRange", Sdf.ValueTypeNames.Float2, Gf.Vec2f(0.02, 6.0))
    xform(spec, (0.0, 0.0, whd / 2.0), (1.0, 1.0, 1.0), quat=(0.0, 1.0, 0.0, 0.0))

    print(f"손목 카메라 {CAM_WRIST}  부모 로컬 offset "
          f"({wo[0]*1000:.0f}, {wo[1]*1000:.0f}, {wo[2]*1000:.0f}) mm  "
          f"기울기 {math.degrees(tilt):.0f} deg  "
          f"{_wc.get('model')} {res[0]}x{res[1]} hfov {hfov:.0f} deg")
    print(f"      그리퍼 몸통(반경 122.8 · z 0~150 mm) 바깥에 두고 흡착면 쪽으로 "
          f"기울였다 · 콜라이더/질량 없음 (동역학 안 건드림)")

# ── 6c) 조명 (돔) ────────────────────────────────────────────
# ★ 왜 필요한가. 씬에는 /Environment/defaultLight (DistantLight 3000) 하나뿐인데,
#   **GUI 뷰포트는 그것만으로도 밝게 보인다** — 뷰포트가 자기 조명 리그를 얹기
#   때문이다. 그런데 오프스크린 렌더(ROS 로 내보내는 카메라, 인식용 render
#   product)에는 그 리그가 없다. 씬 조명만 받는다.
#   실측 2026-08-24, 한 실행에서 돔만 켜고 끄며 잰 화소 평균 / 순수 검정 비율:
#       픽업   180/0.00 -> 196/0.00      손목   94/0.60 -> 233/0.00
#       persp  220/0.00 -> 235/0.00
#   손목처럼 배경(하늘)이 많이 잡히는 시야는 돔이 없으면 화면의 60 %가 순수 검정이다.
#   물리에는 영향이 없다 (콜라이더도 질량도 없다).
#   ※ 손목 카메라가 아예 새까맣게(전부 0) 나왔던 것은 조명이 아니라 카메라가
#     그리퍼 몸통 안에 박혀 있었기 때문이다 — 그건 6b 의 offset 으로 고쳤다.
_lt = SPEC.get("lighting", {}).get("dome", {})
if _lt.get("enabled", True):
    drop("/Environment", "DomeLight")
    spec = define("/Environment/DomeLight", "DomeLight")
    put(spec, "inputs:intensity", Sdf.ValueTypeNames.Float,
        float(_lt.get("intensity", 1000)))
    _c = _lt.get("color", [1.0, 1.0, 1.0])
    put(spec, "inputs:color", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(*_c))
    put(spec, "inputs:exposure", Sdf.ValueTypeNames.Float, 0.0)
    # UsdLux 는 렌더 델리게이트에 이 토큰으로 조명 종류를 알린다.
    # 기존 defaultLight 도 light:shaderId 를 들고 있다 — 같은 형태로 맞춘다.
    put(spec, "light:shaderId", Sdf.ValueTypeNames.Token, "DomeLight",
        uniform=True)
    xform(spec, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    print(f"돔 조명 /Environment/DomeLight  intensity "
          f"{_lt.get('intensity', 1000)}  color {_c}")
    print(f"      오프스크린 렌더(ROS 카메라·인식)가 씬 조명만 받기 때문에 넣는다")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
