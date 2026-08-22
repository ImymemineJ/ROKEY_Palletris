#!/usr/bin/env python3
"""B3 — 흡착이 붙는지, 그리고 **밀착해서** 붙는지 잰다.

    ~/isaac-sim-5.1.0/python.sh scripts/verify_suction.py
    ~/isaac-sim-5.1.0/python.sh scripts/verify_suction.py --box 5호
    ~/isaac-sim-5.1.0/python.sh scripts/verify_suction.py --gui

## 단계

    [0] 벨트에 박스를 놓고 안정화        [1] 흡착면을 박스 위 --approach-gap 까지 내린다
    [2] 흡착 — 상태·잡은 것·간극         [3] 300 mm 상승 — 딸려 오는가, 간극이 유지되는가
    [4] 놓기 — 떨어지는가

## 실측으로 밝혀진 것 (2026-08-22)

- **접근 간극은 2 mm 이하여야 한다.** 패드 D6 의 `transZ` 슬롯이 5 mm 라, 그보다 멀리서
  잡으면 그 자리에 구속이 서고 드라이브가 못 끌어당긴다.

      접근 0 mm -> 흡착 후 −0.03 mm      접근 2 mm -> +1.24 mm      접근 5 mm -> +6.40 mm

  구 저장소가 `PICK` 설계 간극을 2.0 mm 로 잡은 이유가 이것이다.
- **박스의 `xformOp:scale` 은 무관하다.** 제대로 만든 그리퍼에서는 scale 없는 `Mesh` 와
  옛 `Cube`+scale 이 **소수점까지 같은 값**을 낸다 (3호·5호 모두). 구 저장소가 42.8 mm 를
  scale 탓으로 본 것은 오진이고, 진짜 원인은 패드 D6 구성이었다 (build_gripper_usd.py 헤더).
- **질량은 병목이 아니다.** 9 kg 5호도 300 mm 중 299 mm 따라 올라온다.
  힘 상한은 1e6 이라 물리로는 안 걸린다 — 16 kgf 규제는 컨트롤러가 한다.

## 왜 간극을 축에 투영해서 재나

수직 하강 자세면 월드 z 차이와 같지만, 자세가 기울면 z 차이는 실제 간극보다 작게 나온다.
흡착 축(link_6 의 +Z)에 투영한 값이 진짜 간극이다.

## --geom

    mesh : 점을 직접 박은 UsdGeom.Mesh, scale 없음     (spawn_feeder._make_box 현재판)
    cube : UsdGeom.Cube(size=1) + xformOp:scale(dims)  (옛판)

위에 적었듯 지금은 **둘이 같은 결과**다. 회귀 감시용으로 남겨 둔다.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                          # noqa: E402
import verify_lula_ik as V                             # bring_up / goto / solve 재사용  # noqa: E402

LINK6 = "/World/h2017/root_joint/link_6"
GRIPPER = "/World/h2017/SurfaceGripper"
PADS = "/World/h2017/gripper/pads"
BELT = "/World/TestBelt"
BOX = "/World/Boxes/box"

LIFT_MM = 300.0             # 잡고 들어 올릴 높이


# ─────────────────────────────────────────────────────────────
def _vis_box(stage, path, center, size, rgb):
    """보이기만 하는 상자. **물리 없음** — 영상용이다."""
    from pxr import Gf, UsdGeom
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    xf = UsdGeom.Xformable(c.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
    xf.AddScaleOp().Set(Gf.Vec3d(*[float(v) for v in size]))


def _vis_cyl(stage, path, center, r, h, axis, rgb):
    """보이기만 하는 원기둥. **물리 없음.**"""
    from pxr import Gf, UsdGeom
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(float(r))
    c.CreateHeightAttr(float(h))
    c.CreateAxisAttr(axis)
    c.CreateExtentAttr([Gf.Vec3f(-r, -r, -h / 2.0), Gf.Vec3f(r, r, h / 2.0)])
    c.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    xf = UsdGeom.Xformable(c.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))


def make_belt(stage, top_z, cx, cy, size=(1.4, 1.2)):
    """롤러 컨베이어. **충돌은 안 보이는 판 하나가 전부 맡는다.**

    보이는 것(롤러·프레임·다리)에는 물리를 안 준다. 롤러마다 콜라이더를 주면 박스가
    롤러 사이에서 떨거나 끼고, 물리 비용도 는다. 상면이 평평한 판 하나가 정직하고 싸다.
    `MakeInvisible()` 은 렌더만 끈다 — `CollisionAPI` 는 그대로 산다.

    ★ size 는 팔레트와 안 겹치게 잡아야 한다. 겹치면 놓으러 내려갈 때 팔이 벨트에
      막혀서 "제어가 안 된다" 로 오독하게 된다 (verify_b4_cycle 에서 실제로 밟았다).
    """
    from pxr import Gf, UsdGeom, UsdPhysics
    lx, ly = float(size[0]), float(size[1])

    # 충돌 — 상면이 top_z 인 판. 안 보이게 한다.
    cube = UsdGeom.Cube.Define(stage, BELT)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, top_z - 0.05))
    xf.AddScaleOp().Set(Gf.Vec3d(lx, ly, 0.10))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()

    # 보이는 것 ── 롤러 · 옆 프레임 · 다리
    vis = "/World/BeltVisual"
    UsdGeom.Scope.Define(stage, vis)
    rr, pitch = 0.038, 0.095
    n = max(1, int(lx / pitch))
    x0 = cx - (n - 1) * pitch / 2.0
    for i in range(n):
        _vis_cyl(stage, "%s/roller_%02d" % (vis, i),
                 (x0 + i * pitch, cy, top_z - rr), rr, ly - 0.06, "Y", (0.55, 0.57, 0.60))
    for k, sgn in ((0, +1), (1, -1)):
        _vis_box(stage, "%s/frame_%d" % (vis, k),
                 (cx, cy + sgn * (ly / 2.0 - 0.015), top_z - 0.045),
                 (lx, 0.03, 0.13), (0.16, 0.34, 0.52))
    for k, (sx, sy) in enumerate(((+1, +1), (+1, -1), (-1, +1), (-1, -1))):
        _vis_cyl(stage, "%s/leg_%d" % (vis, k),
                 (cx + sx * (lx / 2.0 - 0.08), cy + sy * (ly / 2.0 - 0.05),
                  (top_z - 0.11) / 2.0), 0.028, top_z - 0.11, "Z", (0.30, 0.32, 0.35))


def make_pallet(stage, origin_mm, cfg):
    """목재 팔레트. 충돌은 안 보이는 판 하나, 보이는 것은 상판·블록·하판이다.

    놓은 박스가 얹힐 자리가 없으면 바닥까지 떨어진다. 상면이 `origin_mm[2]` 다.
    """
    from pxr import Gf, UsdGeom, UsdPhysics
    p = cfg["pallet"]
    w, d = p["width_mm"] / 1000.0, p["depth_mm"] / 1000.0
    t = p["thickness_mm"] / 1000.0
    ox, oy, oz = [v / 1000.0 for v in origin_mm]
    cx, cy = ox + w / 2.0, oy + d / 2.0

    cube = UsdGeom.Cube.Define(stage, "/World/TestPallet")
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, oz - t / 2.0))
    xf.AddScaleOp().Set(Gf.Vec3d(w, d, t))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()

    vis = "/World/PalletVisual"
    UsdGeom.Scope.Define(stage, vis)
    wood, dark = (0.70, 0.53, 0.31), (0.58, 0.42, 0.24)
    deck, blk = 0.025, t - 2 * 0.025           # 상판 25 · 블록 100 · 하판 25 = 150
    bw = 0.11                                  # 판재 폭
    n_top = 7
    for i in range(n_top):                     # 상판 — 폭(X) 방향으로 배열, 길이는 깊이(Y)
        x = ox + bw / 2.0 + i * (w - bw) / (n_top - 1)
        _vis_box(stage, "%s/top_%d" % (vis, i), (x, cy, oz - deck / 2.0),
                 (bw, d, deck), wood)
    for i in range(3):                         # 블록 3x3
        for j in range(3):
            _vis_box(stage, "%s/blk_%d%d" % (vis, i, j),
                     (ox + 0.06 + i * (w - 0.12) / 2.0,
                      oy + 0.06 + j * (d - 0.12) / 2.0, oz - deck - blk / 2.0),
                     (0.12, 0.12, blk), dark)
    for j in range(3):                         # 하판 — 깊이(Y) 방향으로 3장
        y = oy + bw / 2.0 + j * (d - bw) / 2.0
        _vis_box(stage, "%s/bot_%d" % (vis, j), (cx, y, oz - t + deck / 2.0),
                 (w, bw, deck), dark)


def make_pedestal(stage, base_xyz_m, cfg, diameter_m=0.4):
    """로봇 받침대. 바닥(z=0)에서 베이스까지 세운다.

    **콜라이더까지 준다.** 없으면 팔이 받침대를 통과해도 시뮬이 안 잡는다 — 실제로는
    부딪히는 물건이라 없는 편이 위험하다. 원기둥이라 scale 없이 치수가 나온다.
    """
    from pxr import Gf, UsdGeom, UsdPhysics
    h = cfg["robot"]["pedestal_height_mm"] / 1000.0
    cyl = UsdGeom.Cylinder.Define(stage, "/World/TestPedestal")
    cyl.CreateRadiusAttr(diameter_m / 2.0)
    cyl.CreateHeightAttr(h)
    cyl.CreateAxisAttr("Z")
    cyl.CreateExtentAttr([Gf.Vec3f(-diameter_m / 2.0, -diameter_m / 2.0, -h / 2.0),
                          Gf.Vec3f(diameter_m / 2.0, diameter_m / 2.0, h / 2.0)])
    cyl.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.37, 0.40)])
    xf = UsdGeom.Xformable(cyl.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(base_xyz_m[0], base_xyz_m[1], h / 2.0))
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())


def make_box(stage, geom, dims, pos, mass):
    """geom='mesh' 면 scale 없는 Mesh, 'cube' 면 옛 Cube+scale."""
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    if geom == "mesh":
        hx, hy, hz = dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0
        mesh = UsdGeom.Mesh.Define(stage, BOX)
        prim = mesh.GetPrim()
        pts = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
               (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
        mesh.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
        mesh.CreateFaceVertexCountsAttr([4] * 6)
        mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1,  4, 5, 6, 7,  0, 1, 5, 4,
                                          1, 2, 6, 5,  2, 3, 7, 6,  3, 0, 4, 7])
        mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.25, 0.20)])
        scale = None
    else:
        cube = UsdGeom.Cube.Define(stage, BOX)
        prim = cube.GetPrim()
        cube.CreateSizeAttr(1.0)
        cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.25, 0.20)])
        scale = Gf.Vec3d(*dims)

    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, Gf.Vec3d(0, 0, 0)))
    if scale is not None:
        xf.AddScaleOp().Set(scale)

    UsdPhysics.CollisionAPI.Apply(prim)
    if geom == "mesh":
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    PhysxSchema.PhysxCollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
    return prim


# ─────────────────────────────────────────────────────────────
def scale_of(stage, path):
    from pxr import UsdGeom
    p = stage.GetPrimAtPath(path)
    for o in UsdGeom.Xformable(p).GetOrderedXformOps():
        if "scale" in o.GetOpName():
            return "scale=%s" % (tuple(round(float(v), 4) for v in o.Get()),)
    return "scale 없음"


def xform_ops(stage, path):
    from pxr import UsdGeom
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        return ["<없음>"]
    return [o.GetOpName().split(":")[-1] for o in UsdGeom.Xformable(p).GetOrderedXformOps()]


def face_pose(stage, grip_len):
    """흡착면 중심(월드) 과 흡착 방향(link_6 의 +Z, 정규화)."""
    from pxr import Gf, Usd, UsdGeom
    m = UsdGeom.Xformable(stage.GetPrimAtPath(LINK6)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    face = np.array(m.Transform(Gf.Vec3d(0.0, 0.0, grip_len)))
    z = np.array(m.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)))
    return face, z / np.linalg.norm(z)


def measure(S, box_rp, half_h):
    """(간극 mm, 흡착면 z mm, 박스 상면 z mm).

    간극 = 흡착 축에 투영한 '흡착면 -> 박스 윗면' 거리. 양수면 떠 있다.

    ★ **둘 다 USD 로 읽으면 안 된다.** PhysX 는 강체 자세를 USD 가 아니라 Fabric 에
      쓴다. `ComputeLocalToWorldTransform` 은 스폰 당시 값을 그대로 돌려줘서, 300 mm
      들어 올려도 박스 상면이 1010.0 mm 로 안 움직인다 (실제로 그렇게 오독했다).
      박스는 `SingleRigidPrim.get_world_pose()`(물리 텐서), 흡착면은 **실제 관절값**
      FK 로 읽는다.
    """
    q = np.asarray(S.art.get_joint_positions()[:6], dtype=float)
    p, R = S.lula.compute_forward_kinematics(S.ee, q)          # 월드
    zax = np.asarray(R)[:, 2]                                  # 툴 +Z = 흡착 방향
    face = np.asarray(p, dtype=float) + zax * S.grip
    c = np.asarray(box_rp.get_world_pose()[0], dtype=float)
    gap = (float(np.dot(c - face, zax)) - half_h) * 1000.0
    return gap, face[2] * 1000.0, (c[2] + half_h) * 1000.0


def dump_pads(stage):
    """패드 D6 조인트가 런타임에 어떻게 채워졌는지 본다. 간극의 정체를 여기서 찾는다."""
    from pxr import UsdPhysics
    import usd.schema.isaac.robot_schema as rs
    scope = stage.GetPrimAtPath(PADS)
    if not scope.IsValid():
        print("    !! %s 가 없다" % PADS)
        return
    for jp in scope.GetChildren():
        j = UsdPhysics.Joint(jp)

        def rel(name):
            t = j.GetPrim().GetRelationship(name).GetTargets()
            return [str(x).split("/")[-1] for x in t] or ["-"]

        def vec(attr):
            v = attr.Get()
            return "(%+.4f %+.4f %+.4f)" % tuple(v) if v is not None else "-"

        print("    %-6s b0=%-10s b1=%-10s p0=%s p1=%s  clr=%s  fwd=%s  enabled=%s"
              % (jp.GetName(), ",".join(rel("physics:body0")), ",".join(rel("physics:body1")),
                 vec(j.GetLocalPos0Attr()), vec(j.GetLocalPos1Attr()),
                 jp.GetAttribute(rs.Attributes.CLEARANCE_OFFSET.name).Get(),
                 jp.GetAttribute(rs.Attributes.FORWARD_AXIS.name).Get(),
                 j.GetJointEnabledAttr().Get()))


# ─────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--geom", default="mesh", choices=["mesh", "cube"],
                    help="박스 지오메트리. cube 는 옛 Cube+scale 판(버그 재현용)")
    ap.add_argument("--box", default="3호", choices=["3호", "4호", "5호"])
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--approach-gap", type=float, default=2.0,
                    help="접근 시 남길 간극 [mm]. ★ 5 mm 로 벌리면 6~7 mm 뜬 채로 굳는다")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--hold", action="store_true", help="끝난 뒤 창을 열어 둔다")
    ap.add_argument("--coaxial", type=float, default=None,
                    help="축방향 힘 상한 [N] 덮어쓰기 (기본 system.yaml)")
    ap.add_argument("--shear", type=float, default=None, help="전단 힘 상한 [N] 덮어쓰기")
    ap.add_argument("--asset", default=None, help="쓸 USD (기본 h2017_suction.usd)")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    _paths.check()
    asset = Path(a.asset) if a.asset else V.ASSET_SUCTION
    if not asset.exists():
        sys.exit("no asset: %s\n  -> scripts/build_gripper_usd.py 를 먼저 돌릴 것" % asset)

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})
    tee = V.start_log(a.log or _paths.ROOT / "results" / ("verify_suction_%s.txt" % a.geom))

    import gripper as G                        # _paths 가 packing_robot 을 sys.path 에 넣는다

    ok_all = True
    try:
        S = V.bring_up(zone=a.zone, render=a.gui, asset=asset)
        stage = S.world.stage
        cfg = S.cfg
        spec = next(t for t in cfg["boxes"]["types"] if t["name"] == a.box)
        dims = [spec["w_mm"] / 1000.0, spec["d_mm"] / 1000.0, spec["h_mm"] / 1000.0]
        mass = spec["mass_g"] / 1000.0
        half_h = dims[2] / 2.0
        belt_z = cfg["conveyor"]["belt_top_z_mm"] / 1000.0
        pick_x = cfg["conveyor"]["stopper_x_mm"] / 1000.0
        sc = cfg["robot"]["suction"]

        print("ASSET   %s" % asset)
        print("GEOM    %s   박스 %s  %s mm  %.0f kg"
              % (a.geom, a.box, [int(v * 1000) for v in dims], mass))
        print("DOF     %d  %s" % (S.art.num_dof, S.art.dof_names))
        print("ZONE    %s   base %s mm   그리퍼 길이 %.0f mm"
              % (a.zone, list(np.round(S.base * 1000).astype(int)), S.grip * 1000))
        print("흡착    maxGripDistance %.0f mm  coaxial %.0f N  shear %.0f N  retry %.0f s"
              % (sc["max_grip_distance_mm"], sc["coaxial_force_limit_n"],
                 sc["shear_force_limit_n"], sc["retry_interval_s"]))

        # ── 씬 ─────────────────────────────────────────────
        make_belt(stage, belt_z, pick_x, 0.0)
        make_pedestal(stage, S.base, cfg)
        box_pos = (pick_x, 0.0, belt_z + half_h + 0.001)
        make_box(stage, a.geom, dims, box_pos, mass)

        # 박스는 world.reset() **뒤에** 생겼다. 물리 뷰를 다시 만들어야 텐서에 잡힌다.
        # reset_xform_properties=False ★ — True 면 박스에 scale 을 붙인다. 지금 재고 있는
        # 바로 그 변수를 측정 도구가 심는 꼴이 된다.
        from isaacsim.core.prims import SingleRigidPrim
        S.world.reset()
        S.art.initialize()
        box_rp = SingleRigidPrim(prim_path=BOX, name="box", reset_xform_properties=False)
        box_rp.initialize()
        S.world.step(render=False)

        # world.reset() 이 xform op 순서를 정규화하면서 scale=(1,1,1) 을 심는다.
        # 항등 스케일은 지오메트리를 안 바꾸므로 봐준다 — 문제는 비항등 스케일이다.
        print("\n박스 xformOps  %s  %s" % (xform_ops(stage, BOX), scale_of(stage, BOX)))
        if a.geom == "mesh" and "1.0, 1.0, 1.0" not in scale_of(stage, BOX) \
                and "없음" not in scale_of(stage, BOX):
            print("  !! mesh 판인데 비항등 scale 이 붙었다")
            ok_all = False

        V.teleport(S, S.ready, steps=int(0.3 * S.hz))
        for _ in range(int(0.7 * S.hz)):                 # 벨트 위에서 안정화
            S.world.step(render=S.render)

        # ★ 검증이 트랙B 모듈을 **그대로** 쓴다. 여기서만 되는 별도 배선이 있으면
        #   이 통과가 robot_motion.py 의 통과를 보장하지 못한다.
        grip = G.Gripper(
            GRIPPER, articulation=S.art, cfg=cfg,
            step=lambda: S.world.step(render=S.render),
            gap_fn=lambda: measure(S, box_rp, half_h)[0],
            tool_pose_fn=lambda: S.lula.compute_forward_kinematics(
                S.ee, np.asarray(S.art.get_joint_positions()[:6], dtype=float))[0],
            on_status=lambda st: print("             /db/set_status <- %s" % st))
        view = grip.view
        if a.coaxial is not None or a.shear is not None:
            view.set_surface_gripper_properties(
                coaxial_force_limit=[a.coaxial] if a.coaxial is not None else None,
                shear_force_limit=[a.shear] if a.shear is not None else None)
            print("힘 상한 덮어씀  coaxial %s N  shear %s N" % (a.coaxial, a.shear))
        print("SurfaceGripper xformOps  %s   (GripperView 가 붙인 것)"
              % xform_ops(stage, GRIPPER))
        print("초기 상태  %s   (APPROACH_GAP %.1f mm · GRIP_TOL %.1f mm)"
              % (grip.status(), G.APPROACH_GAP_MM, G.GRIP_TOL_MM))

        gap0, fz0, bz0 = measure(S, box_rp, half_h)
        print("\n[0] 안정화   박스 상면 %.1f mm (벨트 %.0f + 높이 %.0f = %.0f 기대)"
              % (bz0, belt_z * 1000, dims[2] * 1000, (belt_z + dims[2]) * 1000))

        # ── 접근 ───────────────────────────────────────────
        gapm = a.approach_gap / 1000.0
        tool = np.array([box_pos[0], box_pos[1], bz0 / 1000.0 + gapm + S.grip])
        ok, q, err, down, _, _ = V.solve(S, tool)
        if not ok:
            print("\n!! 접근 IK 실패 — 판정 불가")
            raise SystemExit(1)
        V.goto(S, q, secs=6.0)
        gap1, fz1, bz1 = measure(S, box_rp, half_h)
        print("[1] 접근     IK 오차 %.2f mm  하강정렬 %.3f" % (err * 1000, down))
        print("             간극 %+.2f mm   (목표 %+.1f)   흡착면 z %.1f  박스 상면 z %.1f"
              % (gap1, a.approach_gap, fz1, bz1))

        # ── 흡착 ───────────────────────────────────────────
        print("\n[2] 흡착 ★")
        grabbed = grip.grasp(BOX)
        gap2, fz2, bz2 = measure(S, box_rp, half_h)
        print("             상태 %s   잡은 것 %s" % (grip.status(), grip.gripped() or "없음"))
        print("             간극 %+.2f mm   (Δ %+.2f)   흡착면 z %.1f  박스 상면 z %.1f"
              % (gap2, gap2 - gap1, fz2, bz2))
        dump_pads(stage)
        ok_all &= grabbed

        # ── 들어 올리기 ────────────────────────────────────
        ok, q, err, _, _, _ = V.solve(S, tool + np.array([0.0, 0.0, LIFT_MM / 1000.0]))
        if ok:
            V.goto(S, q, secs=6.0)
            gap3, fz3, bz3 = measure(S, box_rp, half_h)
            rose = bz3 - bz2
            print("\n[3] 상승 %.0f mm   박스 상면 %.1f -> %.1f mm (%+.1f)"
                  % (LIFT_MM, bz2, bz3, rose))
            print("             간극 %+.2f mm   (Δ %+.2f)   상태 %s   is_holding %s"
                  % (gap3, gap3 - gap2, grip.status(), grip.is_holding()))
            held = (rose > LIFT_MM * 0.8 and abs(gap3) <= G.GRIP_TOL_MM
                    and grip.is_holding())
            if not held:
                print("             !! 딸려 올라오지 않았거나 간극이 벌어졌다")
            ok_all &= held
        else:
            print("\n[3] 상승 IK 실패 — 건너뜀")
            ok_all = False

        # ── 놓기 ───────────────────────────────────────────
        print("\n[4] 놓기 (명세서 6.5 순서)")
        released = grip.release()
        for _ in range(int(1.0 * S.hz)):                 # 자유낙하 시간을 준다
            S.world.step(render=S.render)
        _, _, bz4 = measure(S, box_rp, half_h)
        dropped = bz4 < bz3 - 50.0
        print("             박스 상면 %.1f -> %.1f mm   %s"
              % (bz3, bz4, "떨어졌다" if dropped else "!! 안 떨어졌다"))
        ok_all &= dropped and released

        # ── 과적 거절 (물리로는 안 걸린다 — 컨트롤러가 막는다) ──
        print("\n[5] 과적 거절  suction_limit %.0f kgf · box_mass_limit %.0f kg"
              % (cfg["robot"]["suction"]["controller_limits"]["suction_limit_kgf"],
                 cfg["robot"]["suction"]["controller_limits"]["box_mass_limit_kg"]))
        for m in (3.0, 5.0, 9.0, 12.0, 25.0):
            why = G.payload_reason(m, cfg)
            print("             %5.1f kg  %s" % (m, why or "통과"))
        gate = (G.payload_reason(9.0, cfg) is None
                and G.payload_reason(12.0, cfg) is not None
                and G.payload_reason(25.0, cfg) is not None)
        if not gate:
            print("             !! 3/5/9 는 통과하고 12/25 는 거절해야 한다")
        ok_all &= gate

        print("\nRESULT(%s): %s" % (a.geom, "통과" if ok_all else "실패"))
        if a.hold and a.gui:
            V.hold_open(S)
    except BaseException:
        # Kit 이 stdout 을 갈아치운 상태라 트레이스백이 조용히 사라진다. 로그로 남긴다.
        import traceback
        print("\n!! 예외\n" + traceback.format_exc())
        raise
    finally:
        tee.close()
        kit.close()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
