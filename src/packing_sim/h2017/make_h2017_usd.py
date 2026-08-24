#!/usr/bin/env python3
"""h2017.usd 를 URDF·system.yaml 값에 맞춰 교정하고 ~/isaac_scenes/robot/h2017.usd 로 낸다.

원래는 tools/import_h2017_usd.py 로 URDF 에서 새로 임포트하는 게 맞다. 그런데
URDF 가 참조하는 dsr_description2 메시(h2017_white/*.dae, h2017_collision/*.dae)가
이 PC 에 없어서 임포트가 불가능하다. 메시를 받으면 그쪽으로 갈아탄다.

대신 이미 임포트돼 있던 USD 를 쓴다. 확인해 보니 구조는 정확하다.
    revolute 6개 · tool0 존재 · articulation root 정상
    관절 한계도 URDF 와 일치 (joint_2 +-125.00 = +-2.1817 rad, joint_3 +-160.00)
어긋난 건 구동값뿐이라 그것만 URDF·명세 값으로 덮는다.

    maxForce   25800/25800/20400/9600/5400/2700  ->  URDF <limit effort>
    stiffness  716.67 (전 축)                    ->  1000  (임포터 기본값)
    damping     71.67 (전 축)                    ->   100
    solver     articulation 반복수                -> system.yaml (철칙 3)

joint_2 effort 는 URDF 에서 1000 이다. 상류 372 는 계열별 자리표시자라 적재 0kg
수평 자세만으로 한계의 99% 를 쓴다 (URDF 주석 참조).

    ~/isaacsim/python.sh make_h2017_usd.py
"""

import os
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# pxr 과 yaml 을 한 인터프리터에서 같이 쓰려면 SimulationApp 을 띄워야 한다.
# kit/python 은 yaml 이 없고, python.sh 는 SimulationApp 전에는 pxr 이 없다.
from isaacsim import SimulationApp

_app = SimulationApp({"headless": True})

import yaml  # noqa: E402

HERE = Path(__file__).resolve().parent
# 로봇 자산의 정본은 src/packing_robot/ 하나다. 여기 있던 사본은 지웠다 —
# rmpflow 설정의 페데스탈이 사본 -0.500 / 정본 -0.900 으로 갈라져 있었다.
URDF = HERE.parents[1] / "packing_robot/urdf/h2017.urdf"   # parents[1] = src/
SYSTEM_YAML = Path("/home/rokey/palletris_ws/src/packing_bringup/config/system.yaml")
SRC_USD = Path("/home/rokey/isaac_scenes/project_1/SubUSDs/h2017.usd")
OUT_USD = Path(os.environ.get(
    "PL_OUT", "/home/rokey/isaac_scenes/robot/h2017.usd"))

# 임포터 자신의 기본값. import_h2017_usd.py 가 쓰는 것과 같은 수다.
STIFFNESS = float(os.environ.get("PL_STIFFNESS", 1000.0))
DAMPING = float(os.environ.get("PL_DAMPING", 100.0))


def urdf_efforts(path):
    root = ET.parse(path).getroot()
    out = {}
    for j in root.iter("joint"):
        lim = j.find("limit")
        if lim is not None and lim.get("effort"):
            out[j.get("name")] = float(lim.get("effort"))
    return out


efforts = urdf_efforts(URDF)
art = yaml.safe_load(SYSTEM_YAML.read_text())["physics"]["articulation"]
print(f"URDF   {URDF}")
print(f"       effort {efforts}")
print(f"SYSTEM {SYSTEM_YAML}  articulation {art}")

if not SRC_USD.exists():
    sys.exit(f"원본 USD 가 없다: {SRC_USD}")
OUT_USD.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(SRC_USD, OUT_USD)
print(f"SRC    {SRC_USD}\nOUT    {OUT_USD}")

from pxr import PhysxSchema, Usd, UsdPhysics  # noqa: E402

stage = Usd.Stage.Open(str(OUT_USD))

joints, tool0, art_root = {}, None, None
for p in stage.Traverse():
    if p.GetTypeName() == "PhysicsRevoluteJoint":
        joints[p.GetName()] = p
    if p.GetName() == "tool0":
        tool0 = p
    if art_root is None and p.HasAPI(UsdPhysics.ArticulationRootAPI):
        art_root = p

if art_root is not None:
    api = PhysxSchema.PhysxArticulationAPI.Apply(art_root)
    api.CreateSolverPositionIterationCountAttr(art["solver_position_iterations"])
    api.CreateSolverVelocityIterationCountAttr(art["solver_velocity_iterations"])
    print(f"ARTICULATION {art_root.GetPath()}  solver "
          f"pos={art['solver_position_iterations']} vel={art['solver_velocity_iterations']}")

print(f"\n{'joint':<10s} {'lower':>8s} {'upper':>8s} "
      f"{'maxForce':>18s} {'stiffness':>16s} {'damping':>14s}")
for n in sorted(joints):
    j = joints[n]
    d = UsdPhysics.DriveAPI.Get(j, "angular")
    if not d:
        d = UsdPhysics.DriveAPI.Apply(j, "angular")
    old_f = d.GetMaxForceAttr().Get()
    old_s = d.GetStiffnessAttr().Get()
    old_d = d.GetDampingAttr().Get()
    new_f = efforts.get(n, old_f)
    d.CreateMaxForceAttr().Set(new_f)
    d.CreateStiffnessAttr().Set(STIFFNESS)
    d.CreateDampingAttr().Set(DAMPING)
    lo = j.GetAttribute("physics:lowerLimit").Get()
    hi = j.GetAttribute("physics:upperLimit").Get()
    print(f"{n:<10s} {lo:8.2f} {hi:8.2f} "
          f"{old_f:8.0f} -> {new_f:6.0f} {old_s:7.1f} -> {STIFFNESS:5.0f} "
          f"{old_d:6.1f} -> {DAMPING:4.0f}")

stage.GetRootLayer().Save()

ok = True
print("\nCHECKS")
print(f"  revolute joints  {len(joints)}  (want 6)   {'OK' if len(joints)==6 else 'FAIL'}")
ok &= len(joints) == 6
print(f"  tool0 prim       {tool0.GetPath() if tool0 else 'MISSING'}   "
      f"{'OK' if tool0 else 'FAIL'}")
ok &= tool0 is not None
j2 = joints.get("joint_2")
f2 = UsdPhysics.DriveAPI.Get(j2, "angular").GetMaxForceAttr().Get() if j2 else None
print(f"  joint_2 maxForce {f2}  (want 1000 = URDF effort)   "
      f"{'OK' if f2 and f2 >= 999 else 'FAIL'}")
ok &= bool(f2 and f2 >= 999)
# 한계는 URDF 와 맞는지 (여기가 틀리면 IK 가 도달 불가능한 해를 낸다 — 사양서 6-3)
import math  # noqa: E402
root = ET.parse(URDF).getroot()
lim_ok = True
for j in root.iter("joint"):
    n = j.get("name")
    lim = j.find("limit")
    if n not in joints or lim is None:
        continue
    want = math.degrees(float(lim.get("upper")))
    got = joints[n].GetAttribute("physics:upperLimit").Get()
    if abs(want - got) > 0.05:
        lim_ok = False
        print(f"  !! {n} upper {got:.2f} != URDF {want:.2f}")
print(f"  joint limits     {'URDF 와 일치' if lim_ok else '어긋남'}   "
      f"{'OK' if lim_ok else 'FAIL'}")
ok &= lim_ok
print(f"\nRESULT: {'OK' if ok else 'CHECK THE FAILS'}   -> {OUT_USD}")
sys.stdout.flush()
_app.close()
sys.exit(0 if ok else 1)
