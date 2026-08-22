#!/usr/bin/env python3
"""test1-1.usd (벨트 1개 씬) 벨트 끝에 끝막이를 세운다.

벨트가 /World/ConveyorTrack 하나뿐이라 박스가 토출단에서 그냥 바닥으로
떨어진다 (실측: Box_00 이 x=-9.09 에서 z=0 까지 낙하). 받는 벨트가 없으니
벽으로 잡아둔다. 박스는 벽에 막혀 벨트 위에 줄줄이 쌓인다.

필요 없어지면 /World/EndStops 를 통째로 지우면 된다.

  ~/isaacsim/python.sh add_endstop_test1_1.py

test1.usd 는 건드리지 않는다 — 이 스크립트는 test1-1.usd 전용이다.
"""

import os
import shutil
import time

from pxr import Gf, Sdf

USD_PATH = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")

ROOT = "/World/EndStops"
BELT = "/World/ConveyorTrack"

# Isaac Sim 헤드리스 실측 (2026-08-22). /World/ConveyorTrack/Rollers 월드 bbox.
# 원격 에셋을 받아야 나오는 값이라 상수로 박아 둔다.
# set_belt_800_test1_1.py 로 A06->A05 교체 + 800mm 로 맞춘 뒤의 값이다.
# A05 롤러는 A06 벨트보다 좁아서 폭이 1001 -> 724mm 로 줄었다.
#   x -8.3241 .. -0.0166   (흐름 -X, 토출단이 x 최소)
#   y -0.3620 ..  0.3620   (폭 724mm)
#   z 상면 0.80000
# 벨트 높이/에셋을 다시 바꾸면 이 값도 다시 재야 한다.
BELT_END_X = -8.3241
BELT_Y0, BELT_Y1 = -0.3620, 0.3620
BELT_TOP = 0.80000

WALL_T = 0.10        # 두께 (진행 방향)
WALL_H = 0.40        # 높이 — 가장 큰 5호(340mm) 보다 높게

layer = Sdf.Layer.FindOrOpen(USD_PATH)
if layer is None:
    raise SystemExit(f"USD 를 못 열었다: {USD_PATH}")

backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")


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


if layer.GetPrimAtPath(Sdf.Path(ROOT)) is not None:
    del layer.GetPrimAtPath(Sdf.Path("/World")).nameChildren["EndStops"]
    print(f"기존 {ROOT} 제거")
define(ROOT, "Xform")

# 정적 콜라이더. 리지드바디를 안 붙이면 고정 물체가 된다.
path = f"{ROOT}/Stop_End"
spec = define(path, "Cube", ["PhysicsCollisionAPI", "PhysxCollisionAPI"])
put(spec, "size", Sdf.ValueTypeNames.Double, 1.0)
put(spec, "extent", Sdf.ValueTypeNames.Float3Array,
    [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
put(spec, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
    [Gf.Vec3f(0.85, 0.85, 0.2)])
put(spec, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, True)

# 벽 안쪽 면을 벨트 토출단에 딱 맞춘다. 그래야 박스가 벨트 위에 온전히
# 올라간 채로 멈춘다 (벽을 더 바깥에 두면 박스 앞부분이 허공에 걸친다).
cx = BELT_END_X - WALL_T / 2.0
cy = (BELT_Y0 + BELT_Y1) / 2.0
cz = BELT_TOP + WALL_H / 2.0
put(spec, "xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(cx, cy, cz))
put(spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd, Gf.Quatd(1, 0, 0, 0))
put(spec, "xformOp:scale", Sdf.ValueTypeNames.Double3,
    Gf.Vec3d(WALL_T, BELT_Y1 - BELT_Y0, WALL_H))
put(spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray,
    ["xformOp:translate", "xformOp:orient", "xformOp:scale"], uniform=True)

print(f"  Stop_End  x={cx:.3f} (두께 {WALL_T*1000:.0f}mm)  "
      f"y {BELT_Y0:.3f}..{BELT_Y1:.3f}  z {BELT_TOP:.3f}..{BELT_TOP+WALL_H:.3f}")
layer.Save()
print(f"\n저장 완료: {USD_PATH}")
