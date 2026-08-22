#!/usr/bin/env python3
"""test1-1.usd 를 system.yaml 좌표계로 재배치한다 (존 B 기준).

씬과 사양이 서로 다른 세계였다. 사양은 라인이 x=500 -> 10000 으로 +X 로
흐르고 로봇이 x=10900 에 있는데, 씬은 x=-8324 -> 0 으로 -X 로 흘렀다.
좌우가 뒤집힌 채 19 m 떨어져 있었다. 씬을 사양에 맞춘다.

하는 일
  1) 벨트 조각을 **사양이 정한 구간**에 맞춰 x 스케일을 다시 잡는다.
     조각 0 = stage1.main_range_mm (0~7500), 조각 1 = pickup_range_mm (7500~10000).
  2) 벨트를 x=0 에서 시작하도록 옮기고 흐름을 +X 로 돌린다.
  3) 스토퍼는 세우지 않는다 (있으면 지운다). 박스는 벨트를 멈춰서 세운다.
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
ENDSTOP_ROOT = "/World/EndStops"
PEDESTAL = "/World/Pedestal"
OUTFEED = "/World/PalletOutfeed"
ROBOT = "/World/robot"
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

# ── 2) 스토퍼는 안 세운다 ────────────────────────────────────
# 예전에는 사양 stopper_x_mm 자리에 벽을 세워 박스를 막았다. 지금은
# **벨트를 멈춰서** 세운다 (belt_cycle.py). 박스가 항상 같은 자리에 서므로
# 로봇이 고정된 픽 자세 하나로 집을 수 있고, 벽에 밀려 쌓이지도 않는다.
# 그래서 벽을 지운다. stopper_x_mm 는 이제 라인의 끝 좌표로만 쓴다.
drop("/World", "EndStops")

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
print(f"팔레트 모서리 ({porg[0]:.3f}, {porg[1]:.3f})  {pw*1000:.0f}x{pd*1000:.0f}x{pt*1000:.0f}  "
      f"상면 z={porg[2]:.3f}")

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

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
