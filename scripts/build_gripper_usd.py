#!/usr/bin/env python3
"""흡착 그리퍼를 h2017.usd 에 붙여 **`~/isaac_assets/h2017_suction.usd`** 를 만든다. (B3 전제)

    ~/isaac-sim-5.1.0/python.sh scripts/build_gripper_usd.py
    ~/isaac-sim-5.1.0/python.sh scripts/build_gripper_usd.py --force     # 덮어쓰기
    ~/isaac-sim-5.1.0/python.sh scripts/build_gripper_usd.py --check     # 만든 것만 덤프

## 이 파일의 정본은 문서가 아니라 **어제 실제로 작동한 씬**이다

    ~/cobot3_ws/isaacpjt/H2017/jik/usd/h2017_cell.usd

여기 있는 구조와 숫자는 그 씬을 프림 단위로 덤프해서 옮긴 것이다.
`작업순서.md` 는 **계획서**라 최종값과 다른 곳이 있다 (coaxial 39 → 실제 1e6,
몸통을 link_6 하위 콜라이더로 → 실제로는 별도 강체 + mount). **씬이 이긴다.**

## 왜 스크립트인가

URDF 에는 그리퍼가 없다 — `tool0` 에서 끝난다. 구 저장소에서는 GUI 로 붙였지만
**USD 는 커밋할 수 없다**(작업명세서 §1.7). 파일 대신 **파일을 만드는 스크립트**를 커밋해야
4대에서 같은 그리퍼가 나온다. `import_h2017_usd.py` 와 같은 원칙이다.

    URDF ──(import_h2017_usd.py)──> h2017.usd ──(이 스크립트)──> h2017_suction.usd

## 구조 (h2017_cell.usd 그대로)

    /h2017/root_joint/link_6                     ← 아티큘레이션 링크
    /h2017/gripper            Xform  RigidBodyAPI + MassAPI 2 kg + FilteredPairsAPI
        └ body                Cylinder r 0.1225 h 0.15, 축 Z, translate (0,0,0.075), 콜라이더
        └ mount               FixedJoint  body0=link_6  body1=/h2017/gripper
                              excludeFromArticulation = **False**
        └ visuals/pad_0..3    Ø50 원기둥 4개 — **보이기만 한다.** 콜라이더 아님
        └ pads/pad_0..3       D6 4개 = 흡착점 4개
                              body0=/h2017/gripper  body1=link_6
                              localPos0 = localPos1 = (±0.08, ±0.055, 0.15)
                              excludeFromArticulation = **True**
    /h2017/SurfaceGripper     attachmentPoints -> pad_0..3

## ★ 반드시 지킬 것

- **패드 D6 의 6축을 전부 잠그면 안 된다.** `transZ` 는 `[0, 0.005]` 슬롯으로 열어 두고
  `drive:transZ`(stiffness 20000, damping 400) 스프링이 박스를 끌어당긴다.
  전부 잠그면 구속이 생긴 자세 그대로 굳어 **박스가 40 mm 떠서 붙는다** (실측).
  회전도 ±3° 는 열어 둬야 패드가 상판에 순응한다.
- **`isaac:forwardAxis` 는 `Z`.** 기본이 `X` 라 안 주면 옆을 보고 **아예 물체를 못 찾는다**.
- **몸통은 별도 강체 + `mount` 고정 조인트다.** `mount` 는 `excludeFromArticulation = **False**`
  (빼면 그리퍼가 팔을 55 mm 뒤처져 따라간다). **패드는 반대로 True** — 헷갈리는 부분이다.
- **`FilteredPairsAPI` 를 몸통에 얹는다.** 잡은 뒤 그리퍼-박스 충돌 필터를 런타임이 여기에 쓴다.
- **힘 상한은 물리로 못 건다.** `coaxialForceLimit`/`shearForceLimit` 은 1e6 이다.
  16 kgf 규제는 컨트롤러가 한다 (`suction.controller_limits`, 트러블슈팅 문제②).
- **`retryInterval` 은 "재시도 간격"이 아니라 닫기를 시도하는 총 시간(초)** 이다.
  0 이면 즉시 포기해 영영 `Open` 에 머문다.
- **충돌은 Ø245 원기둥 하나가 전부 맡는다.** 245 는 패드 4개의 외접원이다 —
  √(80² + 55²) + 25 = 122.1 mm. `check_place_plan.py` 가 같은 반지름을 쓰므로
  이 값을 줄이면 배치 여유 판정이 같이 헐거워진다. `visuals/` 는 렌더용일 뿐이다.
- 몸통에 `xformOp:scale` 을 걸지 않는다. `Cylinder` 는 radius/height 가 속성이라
  스케일 없이 치수가 나온다. (박스 쪽 scale 은 흡착과 무관한 것으로 판명됐다 — verify_suction.py)

숫자는 전부 `system.yaml` 의 `robot.gripper_*` / `robot.suction` 에서 읽는다 (철칙 3).
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                    # noqa: E402  경로 해결

ASSET_IN = Path.home() / "isaac_assets" / "h2017.usd"
ASSET_OUT = Path.home() / "isaac_assets" / "h2017_suction.usd"

LINK = "/h2017/root_joint/link_6"                # 그리퍼를 매다는 아티큘레이션 링크
BODY_ROOT = "/h2017/gripper"                     # 그리퍼 = 별도 강체
BODY = BODY_ROOT + "/body"
VISUALS = BODY_ROOT + "/visuals"                 # 보이기만 하는 패드. 물리 없음
MOUNT = BODY_ROOT + "/mount"
PADS_SCOPE = BODY_ROOT + "/pads"
GRIPPER = "/h2017/SurfaceGripper"


def build(stage, cfg, log):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    import usd.schema.isaac.robot_schema as rs

    g = cfg["robot"]
    sc = g["suction"]
    pj = sc["pad_joint"]
    grip_len = g["gripper_length_mm"] / 1000.0            # 플랜지~흡착면
    body_r = g["gripper_body_diameter_mm"] / 2000.0

    link = stage.GetPrimAtPath(LINK)
    if not link.IsValid():
        raise SystemExit("!! %s 가 없다. import_h2017_usd.py 를 먼저 돌릴 것" % LINK)

    # ── 1. 그리퍼 = 별도 강체 ────────────────────────────────
    #  link_6 하위 콜라이더로 두면 패드의 body0/body1 이 같은 강체가 돼서 구속이 안 선다.
    #  h2017_cell.usd 는 /World/gripper 를 따로 두고 mount 로 매달았다. 그대로 간다.
    #  초기 자세는 link_6 과 겹쳐야 한다 — 안 그러면 PhysX 가 "disjointed body
    #  transforms" 경고와 함께 스냅시킨다. 그래서 link_6 의 로컬-투-루트를 그대로 쓴다.
    m6 = UsdGeom.Xformable(link).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    gx = UsdGeom.Xform.Define(stage, BODY_ROOT)
    gp = gx.GetPrim()
    gx.AddTranslateOp().Set(m6.ExtractTranslation())
    gx.AddOrientOp().Set(Gf.Quatf(m6.RemoveScaleShear().ExtractRotationQuat()))
    UsdPhysics.RigidBodyAPI.Apply(gp)
    UsdPhysics.MassAPI.Apply(gp).CreateMassAttr(float(sc["body_mass_kg"]))
    UsdPhysics.FilteredPairsAPI.Apply(gp)      # 잡은 뒤 런타임이 충돌 필터를 여기에 쓴다
    log("그리퍼 %s  질량 %.1f kg  초기자세 %s"
        % (BODY_ROOT, sc["body_mass_kg"],
           tuple(round(v, 4) for v in m6.ExtractTranslation())))

    #  몸통 — Cylinder 는 radius/height 가 **속성**이라 scale 이 안 붙는다. 그게 핵심이다.
    cyl = UsdGeom.Cylinder.Define(stage, BODY)
    cyl.CreateRadiusAttr(body_r)
    cyl.CreateHeightAttr(grip_len)
    cyl.CreateAxisAttr("Z")
    cyl.CreateExtentAttr([Gf.Vec3f(-body_r, -body_r, -grip_len / 2.0),
                          Gf.Vec3f(body_r, body_r, grip_len / 2.0)])
    cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, grip_len / 2.0))
    cyl.CreateDisplayColorAttr([Gf.Vec3f(0.62, 0.64, 0.68)])
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    log("몸통  %s  r %.4f  h %.3f  (+Z 가 플랜지 밖, 끝면이 흡착면)"
        % (BODY, body_r, grip_len))

    #  보이는 패드 4개. **콜라이더도 강체도 아니다** — 영상에서 흡착 그리퍼로 읽히게
    #  하는 용도다. 충돌 판정은 위 원기둥 하나가 전부 맡는다 (그게 정본이고,
    #  check_place_plan.py 도 그 반지름을 쓴다).
    #  끝면이 몸통보다 1 mm 튀어나오게 둔다 — 같은 평면에 두면 z-파이팅이 난다.
    UsdGeom.Scope.Define(stage, VISUALS)
    pad_r = sc["pad_diameter_mm"] / 2000.0
    pad_h = 0.030
    for i, (x_mm, y_mm) in enumerate(sc["pad_xy_mm"]):
        v = UsdGeom.Cylinder.Define(stage, "%s/pad_%d" % (VISUALS, i))
        v.CreateRadiusAttr(pad_r)
        v.CreateHeightAttr(pad_h)
        v.CreateAxisAttr("Z")
        v.CreateExtentAttr([Gf.Vec3f(-pad_r, -pad_r, -pad_h / 2.0),
                            Gf.Vec3f(pad_r, pad_r, pad_h / 2.0)])
        v.AddTranslateOp().Set(Gf.Vec3d(x_mm / 1000.0, y_mm / 1000.0,
                                        grip_len + 0.001 - pad_h / 2.0))
        v.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.12, 0.14)])
    log("시각패드 %s  Ø%.0f x4 (물리 없음)" % (VISUALS, sc["pad_diameter_mm"]))

    # ── 2. mount — 손목에 매단다 ─────────────────────────────
    mt = UsdPhysics.FixedJoint.Define(stage, MOUNT)
    mt.CreateBody0Rel().SetTargets([LINK])
    mt.CreateBody1Rel().SetTargets([BODY_ROOT])
    mt.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    mt.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    mt.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    mt.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    mt.CreateExcludeFromArticulationAttr(False)       # ★ 패드와 반대다
    mt.CreateJointEnabledAttr(True)
    mt.CreateCollisionEnabledAttr(False)
    mt.CreateBreakForceAttr(float("inf"))
    mt.CreateBreakTorqueAttr(float("inf"))
    log("매달기 %s  FixedJoint  excludeFromArticulation=False" % MOUNT)

    # ── 3. 패드 = D6 조인트 4개 ──────────────────────────────
    UsdGeom.Scope.Define(stage, PADS_SCOPE)
    zlo, zhi = [v / 1000.0 for v in pj["trans_z_limit_mm"]]
    rot = float(pj["rot_limit_deg"])
    pad_paths = []
    for i, (x_mm, y_mm) in enumerate(sc["pad_xy_mm"]):
        path = "%s/pad_%d" % (PADS_SCOPE, i)
        j = UsdPhysics.Joint.Define(stage, path)
        jp = j.GetPrim()
        pos = Gf.Vec3f(x_mm / 1000.0, y_mm / 1000.0, grip_len)
        j.CreateBody0Rel().SetTargets([BODY_ROOT])
        j.CreateBody1Rel().SetTargets([LINK])
        j.CreateLocalPos0Attr(pos)
        j.CreateLocalPos1Attr(pos)                    # 두 프레임이 겹쳐 있어 같은 값이다
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)     # ★ 안 켜면 팔 관절로 오인돼 IK 가 망가진다
        j.CreateJointEnabledAttr(True)
        j.CreateCollisionEnabledAttr(False)
        j.CreateBreakForceAttr(float("inf"))
        j.CreateBreakTorqueAttr(float("inf"))

        # 리미트. transX/Y 만 잠그고(high < low) **transZ 는 슬롯으로 열어 둔다.**
        for axis, lo, hi in (("transX", 1.0, -1.0), ("transY", 1.0, -1.0),
                             ("transZ", zlo, zhi),
                             ("rotX", -rot, rot), ("rotY", -rot, rot), ("rotZ", -rot, rot)):
            lim = UsdPhysics.LimitAPI.Apply(jp, axis)
            lim.CreateLowAttr().Set(lo)
            lim.CreateHighAttr().Set(hi)

        # 드라이브. transZ 스프링이 박스를 흡착면으로 끌어당긴다 — 밀착의 정체가 이것이다.
        for axis, spec in (("transZ", pj["drive_trans_z"]),
                           ("rotX", pj["drive_rot_xy"]), ("rotY", pj["drive_rot_xy"]),
                           ("rotZ", pj["drive_rot_z"])):
            d = UsdPhysics.DriveAPI.Apply(jp, axis)
            d.CreateTypeAttr().Set(UsdPhysics.Tokens.force)
            d.CreateStiffnessAttr().Set(float(spec["stiffness"]))
            d.CreateDampingAttr().Set(float(spec["damping"]))
            d.CreateTargetPositionAttr().Set(0.0)
            d.CreateTargetVelocityAttr().Set(0.0)
            d.CreateMaxForceAttr().Set(float("inf"))

        rs.ApplyAttachmentPointAPI(jp)
        jp.GetAttribute(rs.Attributes.FORWARD_AXIS.name).Set(UsdPhysics.Tokens.z)
        jp.GetAttribute(rs.Attributes.CLEARANCE_OFFSET.name).Set(0.0)
        pad_paths.append(path)
        log("패드  %s  (%+.3f, %+.3f, %.3f)  transZ [%.3f, %.3f]  rot ±%.0f°  fwd Z"
            % (path, pos[0], pos[1], pos[2], zlo, zhi, rot))

    # ── 4. SurfaceGripper ────────────────────────────────────
    rs.CreateSurfaceGripper(stage, GRIPPER)
    p = stage.GetPrimAtPath(GRIPPER)
    p.GetRelationship(rs.Relations.ATTACHMENT_POINTS.name).SetTargets(pad_paths)
    p.GetAttribute(rs.Attributes.MAX_GRIP_DISTANCE.name).Set(
        sc["max_grip_distance_mm"] / 1000.0)
    p.GetAttribute(rs.Attributes.COAXIAL_FORCE_LIMIT.name).Set(
        float(sc["coaxial_force_limit_n"]))
    p.GetAttribute(rs.Attributes.SHEAR_FORCE_LIMIT.name).Set(
        float(sc["shear_force_limit_n"]))
    p.GetAttribute(rs.Attributes.RETRY_INTERVAL.name).Set(float(sc["retry_interval_s"]))
    log("흡착기 %s  흡착점 %d 개  maxGripDistance %.3f m  힘상한 %.0f/%.0f N  retry %.1f s"
        % (GRIPPER, len(pad_paths), sc["max_grip_distance_mm"] / 1000.0,
           sc["coaxial_force_limit_n"], sc["shear_force_limit_n"], sc["retry_interval_s"]))
    return pad_paths


def check(stage, log):
    """만든 것을 되읽어 확인한다. h2017_cell.usd 와 같은 모양인지."""
    from pxr import UsdGeom, UsdPhysics
    import usd.schema.isaac.robot_schema as rs

    ok = True
    for path in (BODY_ROOT, BODY, VISUALS, MOUNT, PADS_SCOPE, GRIPPER):
        p = stage.GetPrimAtPath(path)
        log("%-34s %s" % (path, "있음" if p.IsValid() else "!! 없음"))
        ok &= p.IsValid()

    body = stage.GetPrimAtPath(BODY)
    if body.IsValid():
        ops = [o.GetOpName() for o in UsdGeom.Xformable(body).GetOrderedXformOps()]
        scaled = [o for o in ops if "scale" in o]
        log("몸통 xformOps %s -> %s" % (ops, "!! scale 이 걸렸다" if scaled else "scale 없음 ✓"))
        ok &= not scaled

    vis = stage.GetPrimAtPath(VISUALS)
    if vis.IsValid():
        bad = [c.GetName() for c in vis.GetChildren()
               if any("Collision" in a or "RigidBody" in a for a in c.GetAppliedSchemas())]
        log("시각패드 %d 개  %s" % (len(vis.GetChildren()),
                                  "물리 없음 ✓" if not bad else "!! 물리가 붙었다 %s" % bad))
        ok &= len(vis.GetChildren()) == 4 and not bad

    mt = stage.GetPrimAtPath(MOUNT)
    if mt.IsValid():
        ex = UsdPhysics.Joint(mt).GetExcludeFromArticulationAttr().Get()
        log("mount excludeFromArticulation %s  %s" % (ex, "" if ex is False else "!! False 여야 한다"))
        ok &= ex is False

    gp = stage.GetPrimAtPath(GRIPPER)
    if gp.IsValid():
        tg = gp.GetRelationship(rs.Relations.ATTACHMENT_POINTS.name).GetTargets()
        log("흡착점 %d 개: %s" % (len(tg), [str(t).split("/")[-1] for t in tg]))
        ok &= len(tg) == 4
        for t in tg:
            jp = stage.GetPrimAtPath(t)
            j = UsdPhysics.Joint(jp)
            fa = jp.GetAttribute(rs.Attributes.FORWARD_AXIS.name).Get()
            zl = UsdPhysics.LimitAPI(jp, "transZ")
            zlo, zhi = zl.GetLowAttr().Get(), zl.GetHighAttr().Get()
            dz = UsdPhysics.DriveAPI(jp, "transZ")
            b0 = [str(x).split("/")[-1] for x in j.GetBody0Rel().GetTargets()]
            b1 = [str(x).split("/")[-1] for x in j.GetBody1Rel().GetTargets()]
            good = (fa == UsdPhysics.Tokens.z and zlo is not None and zhi > zlo
                    and (dz.GetStiffnessAttr().Get() or 0) > 0
                    and j.GetExcludeFromArticulationAttr().Get() is True)
            log("  %-6s b0=%-8s b1=%-8s fwd=%-2s transZ[%.4f, %.4f] driveK=%s  %s"
                % (str(t).split("/")[-1], ",".join(b0), ",".join(b1), fa, zlo, zhi,
                   dz.GetStiffnessAttr().Get(), "" if good else "!!"))
            ok &= good
        for a in (rs.Attributes.MAX_GRIP_DISTANCE, rs.Attributes.COAXIAL_FORCE_LIMIT,
                  rs.Attributes.SHEAR_FORCE_LIMIT, rs.Attributes.RETRY_INTERVAL):
            log("  %-28s %s" % (a.name, gp.GetAttribute(a.name).Get()))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true", help="출력 파일 덮어쓰기")
    ap.add_argument("--check", action="store_true", help="이미 만든 파일만 확인")
    a = ap.parse_args()

    _paths.check()
    if not ASSET_IN.exists():
        raise SystemExit("!! %s 가 없다. import_h2017_usd.py 를 먼저 돌릴 것" % ASSET_IN)
    if ASSET_OUT.exists() and not (a.force or a.check):
        raise SystemExit("!! %s 가 이미 있다. --force 로 덮어쓸 것" % ASSET_OUT)

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": True})

    # ⚠ Kit 이 시작하면서 sys.stdout 을 갈아치운다. print 가 삼켜지므로 파일로 쓴다.
    #   (verify_lula_ik.start_log 과 같은 이유)
    log_path = _paths.ROOT / "results" / "build_gripper.txt"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    fh = log_path.open("w", encoding="utf-8")

    def log(msg):
        fh.write(str(msg) + "\n")
        fh.flush()

    try:
        from pxr import Usd
        cfg = _paths.system_config()
        if a.check:
            stage = Usd.Stage.Open(str(ASSET_OUT))
            log("확인: %s" % ASSET_OUT)
            ok = check(stage, log)
        else:
            stage = Usd.Stage.Open(str(ASSET_IN))
            log("원본: %s" % ASSET_IN)
            build(stage, cfg, log)
            stage.GetRootLayer().Export(str(ASSET_OUT))
            log("저장: %s" % ASSET_OUT)
            ok = check(Usd.Stage.Open(str(ASSET_OUT)), log)
        log("RESULT: %s" % ("통과" if ok else "실패 — 위 !! 항목"))
    finally:
        fh.close()
        kit.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
