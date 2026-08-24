"""벨트 이송속도 검증 — **`isaac-sim.sh --exec` 전용** (트랙 A).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec /home/rokey/palletris_ws/scripts/verify_belt_speed.py

## 왜 `python.sh` 가 아니라 `isaac-sim.sh` 인가

벨트는 `IsaacConveyor` OmniGraph 노드가 `graph:variable:Velocity` 를 읽어
`physxSurfaceVelocity` 로 옮겨야 돈다. 그 노드는 `OnTick` 으로 도는데,
**`OnTick` 은 앱이 렌더 틱을 돌 때만 발화한다.**

`python.sh` + `SimulationApp({"headless": True})` + `world.step(render=False)` 로는
그래프가 아예 안 돌아서, 벨트가 USD 에 남아 있던 옛 표면속도로 돈다.
`set_velocity` 가 ok 를 돌려주는데 실제 속도는 요청과 무관하다 (실측 2026-08-22:
요청 0.5 든 0.02 든 똑같이 약 12.7 m/s). **이 스크립트가 그 경로를 피해서 잰다.**

## 재는 것

  1. `system.yaml` 의 `conveyor.belt_velocity_mps` (사양, 기본 0.5)
  2. 박스를 스폰해 벨트 위(z ≈ 벨트상면 + h/2) 주행 구간의 dx/dt 실측
  3. 조각별로 따로 잰다 — main(0~7500) 과 b_pickup(7500~10000) 은 scale.x 가
     3 배 달라 `Velocity` 값도 다르다. 한쪽만 맞고 다른 쪽이 틀릴 수 있다.

합격선: 두 구간 모두 사양 대비 ±20 % 안.
"""
import json
import math
import os
import sys

import omni.kit.app
import omni.timeline
import omni.usd
from omni.physx import get_physx_interface
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

STAGE = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")
BOX_ROOT = "/World/boxes"
BOX_ID = "BELTCAL"
BOX_PATH = "%s/%s" % (BOX_ROOT, BOX_ID)
TOL = 0.20                                  # 사양 대비 허용 오차 20 %
SETTLE_STEPS = 30                           # 착지 안정화
MAX_STEPS = int(os.environ.get("PL_MAX_STEPS", "1800"))   # 60 Hz 에서 30 초


def load_cfg():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(root, "src/packing_bringup/config/system.yaml")
    try:
        import yaml
        return yaml.safe_load(open(path))
    except ImportError:
        # kit python 에 yaml 이 없을 수 있다. 필요한 값만 손으로 긁는다.
        cfg = {"conveyor": {}, "boxes": {"types": []}}
        for line in open(path, encoding="utf-8"):
            s = line.strip()
            if s.startswith("belt_velocity_mps:"):
                cfg["conveyor"]["belt_velocity_mps"] = float(s.split(":")[1])
            elif s.startswith("belt_top_z_mm:"):
                cfg["conveyor"]["belt_top_z_mm"] = float(s.split(":")[1].split("#")[0])
            elif s.startswith("spawn_point_mm:"):
                v = s.split(":", 1)[1].split("#")[0].strip()
                cfg["conveyor"]["spawn_point_mm"] = json.loads(v)
            elif s.startswith("- {name:"):
                d = {}
                for part in s.strip("- {}").split(","):
                    k, _, val = part.partition(":")
                    d[k.strip()] = val.strip().strip('"')
                cfg["boxes"]["types"].append(
                    {"name": d.get("name"),
                     "w_mm": float(d.get("w_mm", 0)), "d_mm": float(d.get("d_mm", 0)),
                     "h_mm": float(d.get("h_mm", 0)), "mass_g": float(d.get("mass_g", 0))})
        return cfg


def make_box(stage, path, dims, pos, mass):
    hx, hy, hz = dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0
    mesh = UsdGeom.Mesh.Define(stage, path)
    prim = mesh.GetPrim()
    pts = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
           (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    mesh.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4,
                                      1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
    mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.9, 0.1)])
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    PhysxSchema.PhysxCollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(24)
    rb.CreateSolverVelocityIterationCountAttr(4)
    return prim


# ── 준비 ────────────────────────────────────────────────────
omni.usd.get_context().open_stage(STAGE)
stage = omni.usd.get_context().get_stage()
cfg = load_cfg()
SPEC_MPS = float(cfg["conveyor"]["belt_velocity_mps"])
belt_top = float(cfg["conveyor"]["belt_top_z_mm"]) / 1000.0
sp_mm = cfg["conveyor"]["spawn_point_mm"]
t4 = next(t for t in cfg["boxes"]["types"] if t["name"] == "4호")
dims = (t4["w_mm"] / 1000.0, t4["d_mm"] / 1000.0, t4["h_mm"] / 1000.0)

print("\n" + "=" * 66)
print("벨트 이송속도 검증 — isaac-sim.sh (OmniGraph 가 도는 경로)")
print("=" * 66)
print("씬        %s" % STAGE)
print("사양      belt_velocity_mps = %.3f m/s" % SPEC_MPS)
print("벨트 상면 %.3f m   4호 %s mm" % (belt_top, [int(d * 1000) for d in dims]))

# 그래프 변수 확인 — layout_to_spec 이 조각별로 넣어 둔 값
for track in ("/World/ConveyorTrack", "/World/ConveyorTrack1"):
    g = stage.GetPrimAtPath(track + "/ConveyorBeltGraph")
    tp = stage.GetPrimAtPath(track)
    sx = float(tp.GetAttribute("xformOp:scale").Get()[0]) if tp else float("nan")
    v = g.GetAttribute("graph:variable:Velocity").Get() if g else None
    print("  %-24s scale.x %.4f  Velocity %.6f  -> 기대 월드 %.3f m/s"
          % (track, sx, v if v is not None else float("nan"),
             (v * sx) if v is not None else float("nan")))

if not stage.GetPrimAtPath(BOX_ROOT):
    UsdGeom.Xform.Define(stage, BOX_ROOT)
if stage.GetPrimAtPath(BOX_PATH):
    stage.RemovePrim(BOX_PATH)
spawn_pos = (sp_mm[0] / 1000.0, sp_mm[1] / 1000.0,
             sp_mm[2] / 1000.0 + dims[2] / 2.0)
make_box(stage, BOX_PATH, dims, spawn_pos, t4["mass_g"] / 1000.0)
print("박스      %s 스폰 %s" % (BOX_PATH, tuple(round(v, 3) for v in spawn_pos)))
print("-" * 66)

_state = {"n": 0, "t": 0.0, "samples": [], "done": False, "sub": None}
_cache = UsdGeom.XformCache()


def _on_step(dt):
    if _state["done"]:
        return
    _state["n"] += 1
    _state["t"] += dt
    if _state["n"] < SETTLE_STEPS:
        return
    prim = stage.GetPrimAtPath(BOX_PATH)
    if not prim or not prim.IsValid():
        return
    _cache.Clear()
    p = _cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    x, z = float(p[0]), float(p[2])
    on_belt = abs(z - (belt_top + dims[2] / 2.0)) < 0.02
    _state["samples"].append((_state["t"], x, z, on_belt))
    if _state["n"] % 60 == 0:
        print("  t=%5.1fs  x=%8.3f  z=%6.3f  %s"
              % (_state["t"], x, z, "벨트 위" if on_belt else "벨트 밖"))
    # 스토퍼(10 m) 근처에 오거나 벨트를 벗어나면 끝
    if x > 9.6 or _state["n"] > MAX_STEPS or (not on_belt and x > 1.0):
        _state["done"] = True
        report()


def seg_speed(lo, hi):
    """[lo, hi] m 구간을 벨트 위에서 지나간 속도. 못 재면 None."""
    pts = [(t, x) for (t, x, z, on) in _state["samples"] if on and lo <= x <= hi]
    if len(pts) < 5:
        return None
    (t1, x1), (t2, x2) = pts[0], pts[-1]
    return (x2 - x1) / (t2 - t1) if t2 > t1 else None


def report():
    print("-" * 66)
    ok = True
    for name, lo, hi in (("main     (0~7500)", 1.0, 7.0),
                         ("b_pickup (7500~10000)", 7.8, 9.5)):
        v = seg_speed(lo, hi)
        if v is None:
            print("  %-22s 샘플 부족 — 판정 불가" % name)
            ok = False
            continue
        err = (v - SPEC_MPS) / SPEC_MPS
        mark = "OK" if abs(err) <= TOL else "!! 불합격"
        print("  %-22s 실측 %6.3f m/s  (사양 %.3f, 오차 %+6.1f %%)  %s"
              % (name, v, SPEC_MPS, err * 100.0, mark))
        ok &= abs(err) <= TOL
    zs = [z for (_t, _x, z, on) in _state["samples"] if on]
    if zs:
        want = belt_top + dims[2] / 2.0
        print("  주행 높이              z %.4f ~ %.4f m  (기대 %.4f)"
              % (min(zs), max(zs), want))
    print("-" * 66)
    print("RESULT: %s" % ("통과" if ok else "실패"))
    print("=" * 66 + "\n", flush=True)
    # ★ `import omni.kit.app` 을 이 함수 안에 두면 `omni` 가 지역변수가 되어
    #   바로 윗줄의 `omni.timeline` 이 UnboundLocalError 를 낸다. 모듈 상단에서 받는다.
    omni.timeline.get_timeline_interface().stop()
    omni.kit.app.get_app().post_quit()          # --no-window 에서 스스로 빠져나온다


_state["sub"] = get_physx_interface().subscribe_physics_step_events(_on_step)
omni.timeline.get_timeline_interface().play()
print("Play — 물리 스텝 대기 중...\n", flush=True)
