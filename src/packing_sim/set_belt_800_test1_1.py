#!/usr/bin/env python3
"""test1-1.usd 의 벨트 접촉면을 지면 위 800 mm 로 맞춘다 (system.yaml 사양).

왜 교체하나 (set_belt_900.py 와 같은 이유)
    A06 은 벨트면이 원래 1780 mm 다. 이걸 800 으로 쓰려고 조각을 z=-0.98 로
    내리면 다리와 프레임이 지하에 묻힌다. 그래서 같은 형상의 낮은 짝인
    A05(ROLLER 계열, 고유 상면 약 770 mm)로 바꾸고, 원점 z=0 으로 다리를
    땅에 붙인 뒤 z 스케일로 상면을 800 에 맞춘다.

주의: 교체하면 하위 프림 이름이 Belt -> Rollers 로 바뀐다. 그래프의
inputs:conveyorPrim 을 같이 안 고치면 벨트가 통째로 죽는다.

두 단계로 돈다 (set_belt_900.py 와 같은 방식)
    PL_PHASE=swap   교체 + z=0 + scale.z=1.0 + 흐름 방향 보정 저장.
                    이 상태를 헤드리스로 재서 조각 고유 상면을 얻는다.
    PL_PHASE=scale  PL_NATIVE_TOP=<m> 를 받아 scale.z 를 잡는다.
    PL_PHASE=dir    흐름 방향만 다시 잡는다 (PL_DIR_X, 기본 -1).

흐름 방향 주의
    A05 의 Rollers 는 A06 의 Belt 와 로컬 프레임이 달라서, 같은
    inputs:direction=(1,0,0) 을 줘도 교체 후 박스가 반대로 흐른다
    (실측: 교체 직후 박스가 +X 로 밀려 투입단으로 되돌아 떨어졌다).
    그래서 교체할 때 direction 의 x 부호를 뒤집는다.

벨트 속도도 같이 맞춘다. 그래프 변수 Velocity 와 월드 속도의 관계가
에셋 계열마다 다르다 — 실측으로 확인했다.
    A06 / Belt     월드 = Velocity * xformOp:scale.x   (0.2 * 4.180 = 0.836 m/s)
    A05 / Rollers  월드 = Velocity                     (게인 실측 1.0032)
교체 후에는 A05 이므로 게인 1.0 이 기본이다. PL_VEL_GAIN 으로 바꿀 수 있다.
PL_BELT_MPS 는 system.yaml conveyor.belt_velocity_mps (0.5) 를 따른다.

  ~/isaacsim/python.sh set_belt_800_test1_1.py

test1.usd 는 건드리지 않는다 — test1-1.usd 전용이다.
"""

import os
import shutil
import time

from pxr import Gf, Sdf

USD_PATH = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")
PHASE = os.environ.get("PL_PHASE", "swap")
TARGET_TOP = float(os.environ.get("PL_TARGET_TOP", 0.800))
BELT_MPS = float(os.environ.get("PL_BELT_MPS", 0.5))
NATIVE_TOP = os.environ.get("PL_NATIVE_TOP")
DIR_X = float(os.environ.get("PL_DIR_X", -1.0))   # 월드 -X 로 흐르게
VEL_GAIN = float(os.environ.get("PL_VEL_GAIN", 1.0))  # A05/Rollers 실측 게인

TRACK = "/World/ConveyorTrack"
BASE = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
        "/Assets/Isaac/5.1/Isaac/Props/Conveyors/")
SWAP = {"ConveyorBelt_A06": "ConveyorBelt_A05",
        "ConveyorBelt_A03": "ConveyorBelt_A02"}
STALE_PREFIX = ("SM_ConveyorBelt_A06", "SM_ConveyorBelt_A03", "SM_ConveyorBelt_A37")

layer = Sdf.Layer.FindOrOpen(USD_PATH)
if layer is None:
    raise SystemExit(f"USD 를 못 열었다: {USD_PATH}")
backup = f"{USD_PATH}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(USD_PATH, backup)
print(f"백업: {backup}\n")

path = Sdf.Path(TRACK)
prim = layer.GetPrimAtPath(path)
if prim is None:
    raise SystemExit(f"{TRACK} 가 없다")


def asset_of(prim):
    if not prim.HasInfo("references"):
        return None, None
    refs = prim.GetInfo("references")
    for slot in ("prependedItems", "explicitItems", "appendedItems", "addedItems"):
        for r in getattr(refs, slot, []):
            name = r.assetPath.split("/")[-1].replace(".usd", "")
            if name.startswith("ConveyorBelt_"):
                return name, slot
    return None, None


def swap_ref(old, new, slot):
    refs = prim.GetInfo("references")
    items = [Sdf.Reference(BASE + new + ".usd")
             if r.assetPath.split("/")[-1].replace(".usd", "") == old else r
             for r in getattr(refs, slot, [])]
    op = Sdf.ReferenceListOp()
    setattr(op, slot, items)
    prim.SetInfo("references", op)

    # 그래프 타깃 Belt -> Rollers
    for g in prim.nameChildren:
        if g.typeName != "OmniGraph":
            continue
        for node in layer.GetPrimAtPath(path.AppendChild(g.name)).nameChildren:
            rel = layer.GetPropertyAtPath(
                path.AppendChild(g.name).AppendChild(node.name)
                    .AppendProperty("inputs:conveyorPrim"))
            if rel is None:
                continue
            cur = list(rel.targetPathList.GetAddedOrExplicitItems())
            fixed = [Sdf.Path(t.pathString.replace("/Belt", "/Rollers")) for t in cur]
            if fixed != cur:
                rel.targetPathList.ClearEditsAndMakeExplicit()
                for t in fixed:
                    rel.targetPathList.explicitItems.append(t)
                print(f"    그래프 타깃 {cur[0]} -> {fixed[0]}")

    for name in [c.name for c in prim.nameChildren
                 if c.name == "Belt" or c.name.startswith(STALE_PREFIX)]:
        del prim.nameChildren[name]
        print(f"    오버라이드 제거 {name}")

    # Rollers 오버라이드 + 표면속도 ON. 안 하면 조각은 살아 있는데 안 밀린다.
    roll = layer.GetPrimAtPath(path.AppendChild("Rollers"))
    if roll is None:
        roll = Sdf.PrimSpec(prim, "Rollers", Sdf.SpecifierOver)
    if roll.properties.get("physxSurfaceVelocity:surfaceVelocityEnabled") is None:
        a = Sdf.AttributeSpec(roll, "physxSurfaceVelocity:surfaceVelocityEnabled",
                              Sdf.ValueTypeNames.Bool)
        a.default = True
        print("    Rollers 표면속도 ON")


def set_direction(dir_x):
    """IsaacConveyor 의 흐름 방향을 잡는다.

    physics 방향은 inputs:direction, 눈에 보이는 롤러 애니메이션은
    animateDirection 이다. 둘이 어긋나면 박스는 A 로 가는데 롤러는 B 로 도는
    것처럼 보인다. 그래서 anim 의 x 부호를 direction 과 항상 같이 간다.

    (A05 계열의 다른 조각들이 anim=(0,1) 을 쓰길래 따라 했다가 롤러가 반대로
    도는 것처럼 보였다. 그 조각들은 direction 도 방위도 달라서 그대로 베끼면
    안 되는 값이었다.)
    """
    for g in prim.nameChildren:
        if g.typeName != "OmniGraph":
            continue
        for node in layer.GetPrimAtPath(path.AppendChild(g.name)).nameChildren:
            npath = path.AppendChild(g.name).AppendChild(node.name)
            d = layer.GetAttributeAtPath(npath.AppendProperty("inputs:direction"))
            if d is None:
                continue
            old_d = Gf.Vec3f(d.default)
            d.default = Gf.Vec3f(dir_x, old_d[1], old_d[2])
            a = layer.GetAttributeAtPath(npath.AppendProperty("inputs:animateDirection"))
            anim = None
            if a is not None:
                anim = Gf.Vec2f(1.0 if dir_x >= 0 else -1.0, 0.0)
                a.default = anim
            print(f"    흐름 방향 {tuple(old_d)} -> {tuple(d.default)}  anim {tuple(anim)}")


asset, slot = asset_of(prim)
print(f"=== {TRACK}  ({asset})")

t = layer.GetAttributeAtPath(path.AppendProperty("xformOp:translate"))
s = layer.GetAttributeAtPath(path.AppendProperty("xformOp:scale"))
scale = Gf.Vec3d(s.default)

if PHASE == "swap":
    if asset in SWAP:
        print(f"    참조 교체 {asset} -> {SWAP[asset]}")
        swap_ref(asset, SWAP[asset], slot)
    else:
        print(f"    이미 낮은 계열 ({asset}) — 교체 건너뜀")
    set_direction(DIR_X)
    o = Gf.Vec3d(t.default)
    t.default = Gf.Vec3d(o[0], o[1], 0.0)
    s.default = Gf.Vec3d(scale[0], scale[1], 1.0)
    print(f"    z {o[2]:+.4f} -> 0.0000   scale.z {scale[2]:.4f} -> 1.0000")
    print("\n측정 단계다. 헤드리스로 Rollers 상면을 재서 "
          "PL_PHASE=scale PL_NATIVE_TOP=<m> 로 다시 돌려라.")
elif PHASE == "scale":
    if not NATIVE_TOP:
        raise SystemExit("PL_NATIVE_TOP 을 줘야 한다 (scale.z=1.0 일 때의 상면 [m])")
    native = float(NATIVE_TOP)
    new_sz = round(TARGET_TOP / native, 6)
    s.default = Gf.Vec3d(scale[0], scale[1], new_sz)
    print(f"    고유상면 {native * 1000:.1f}mm -> 목표 {TARGET_TOP * 1000:.0f}mm   "
          f"scale.z {scale[2]:.4f} -> {new_sz:.4f}")
elif PHASE == "dir":
    set_direction(DIR_X)
else:
    raise SystemExit(f"PL_PHASE 는 swap | scale | dir: {PHASE!r}")

# 벨트 속도 — 그래프 변수는 로컬 값이라 x 스케일로 나눠 넣는다
vel_attr = layer.GetAttributeAtPath(
    Sdf.Path(f"{TRACK}/ConveyorBeltGraph").AppendProperty("graph:variable:Velocity"))
if vel_attr is not None:
    local = BELT_MPS / VEL_GAIN
    old = float(vel_attr.default)
    vel_attr.default = local
    print(f"    속도 Velocity {old:.4f} -> {local:.4f}  "
          f"(월드 {BELT_MPS:.3f} m/s, 게인 {VEL_GAIN:.4f})")

layer.Save()
print(f"\n저장 완료: {USD_PATH}")
