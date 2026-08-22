#!/usr/bin/env python3
"""흡착 그리퍼를 h2017.usd 에 붙여 **`~/isaac_assets/h2017_suction.usd`** 를 만든다. (B3 전제)

    ~/isaac-sim-5.1.0/python.sh scripts/build_gripper_usd.py
    ~/isaac-sim-5.1.0/python.sh scripts/build_gripper_usd.py --force     # 덮어쓰기
    ~/isaac-sim-5.1.0/python.sh scripts/build_gripper_usd.py --check     # 만든 것만 덤프

## 왜 스크립트인가

URDF 에는 그리퍼가 없다 — `tool0` 에서 끝난다. 구 저장소에서는 GUI 로 붙였지만
**USD 는 커밋할 수 없다**(작업명세서 §1.7). 파일 대신 **파일을 만드는 스크립트**를 커밋해야
4대에서 같은 그리퍼가 나온다. `import_h2017_usd.py` 와 같은 원칙이다.

    URDF ──(import_h2017_usd.py)──> h2017.usd ──(이 스크립트)──> h2017_suction.usd

## 구조 (구 저장소 `작업순서.md` 2-1~2-4 를 그대로 옮긴 것)

    /h2017/root_joint/link_6                     ← 이미 있는 강체
        └ gripper_body     Cylinder r 0.1225 h 0.15, 축 Z, translate (0,0,0.075)
                           Collider O, **RigidBody X** (link_6 이 이미 강체다)
                           MassAPI 2 kg
    /h2017/gripper_pads/pad_0..3                 D6 조인트 4개 = 흡착점 4개
                           body0 = link_6, body1 = 비움(런타임에 채워진다)
                           excludeFromArticulation = **True**
    /h2017/SurfaceGripper                        attachmentPoints -> pad_0..3

## ★ 반드시 지킬 것 (`docs/흡착_그리퍼_트러블슈팅.md`)

- **흡착 대상 프림에 `xformOp:scale` 을 걸지 않는다.** SurfaceGripper 의 런타임 구속
  앵커 계산이 스케일을 처리하지 못해 **박스가 42.8 mm 떨어진 채 붙는다.** 실측으로 확인된
  단일 변수다. 여기서 몸통을 `Cylinder`(radius/height 속성)로 만드는 이유도 같다 —
  `Cube` + scale 로 만들면 같은 버그를 그리퍼 쪽에 심는 것이다.
  박스도 `Cube`+scale 이 아니라 **점을 직접 박은 `Mesh`** 로 만들어야 한다.
- **몸통에 RigidBody 를 주지 않는다.** PhysX 는 강체 중첩을 허용하지 않는다.
  `link_6` 아래 콜라이더로 두면 조인트 없이 손목과 한 몸이 된다.
- **패드 D6 조인트는 `excludeFromArticulation = True`.** 안 켜면 팔 관절로 오인돼 IK 가 망가진다.
  (반대로 그리퍼를 매다는 `mount` 조인트는 False 여야 한다. 여기서는 몸통이 별도 강체가
  아니라 mount 자체가 없다.)
- **`retryInterval` 은 "재시도 간격"이 아니라 닫기를 시도하는 총 시간(초)** 이다.
  0 이면 즉시 포기해 영영 `Open` 에 머문다.
- **힘 상한은 물리로 못 건다.** `coaxialForceLimit` / `shearForceLimit` 을 넣어 두지만
  실제 16 kgf 규제는 컨트롤러가 한다 (트러블슈팅 문제②).

숫자는 전부 `system.yaml` 의 `robot.gripper_*` / `robot.suction` 에서 읽는다 (철칙 3).
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                    # noqa: E402  경로 해결

ASSET_IN = Path.home() / "isaac_assets" / "h2017.usd"
ASSET_OUT = Path.home() / "isaac_assets" / "h2017_suction.usd"

LINK = "/h2017/root_joint/link_6"                # 흡착 그리퍼가 붙는 강체
PADS_SCOPE = "/h2017/gripper_pads"
GRIPPER = "/h2017/SurfaceGripper"


def build(stage, cfg, log):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    import usd.schema.isaac.robot_schema as rs

    g = cfg["robot"]
    sc = g["suction"]
    grip_len = g["gripper_length_mm"] / 1000.0            # 플랜지~흡착면
    body_r = g["gripper_body_diameter_mm"] / 2000.0

    link = stage.GetPrimAtPath(LINK)
    if not link.IsValid():
        raise SystemExit("!! %s 가 없다. import_h2017_usd.py 를 먼저 돌릴 것" % LINK)

    # ── 1. 몸통 ──────────────────────────────────────────────────────
    #  Cylinder 는 radius/height 가 **속성**이라 scale 이 안 붙는다. 그게 핵심이다.
    body_path = LINK + "/gripper_body"
    cyl = UsdGeom.Cylinder.Define(stage, body_path)
    cyl.CreateRadiusAttr(body_r)
    cyl.CreateHeightAttr(grip_len)
    cyl.CreateAxisAttr("Z")
    cyl.CreateExtentAttr([Gf.Vec3f(-body_r, -body_r, -grip_len / 2.0),
                          Gf.Vec3f(body_r, body_r, grip_len / 2.0)])
    # 몸통 중심 = 플랜지에서 절반. +Z 가 플랜지 바깥이므로 far face 가 흡착면이다.
    cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, grip_len / 2.0))
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    UsdPhysics.MassAPI.Apply(cyl.GetPrim()).CreateMassAttr(float(sc["body_mass_kg"]))
    log("몸통  %s  r %.4f  h %.3f  질량 %.1f kg  (RigidBody 없음 — link_6 의 콜라이더다)"
        % (body_path, body_r, grip_len, sc["body_mass_kg"]))

    # ── 2. 패드 = D6 조인트 4개 ──────────────────────────────────────
    UsdGeom.Scope.Define(stage, PADS_SCOPE)
    pad_paths = []
    for i, (x_mm, y_mm) in enumerate(sc["pad_xy_mm"]):
        path = "%s/pad_%d" % (PADS_SCOPE, i)
        j = UsdPhysics.Joint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([LINK])
        j.CreateBody1Rel().SetTargets([])                 # 잡을 때 런타임이 채운다
        j.CreateLocalPos0Attr(Gf.Vec3f(x_mm / 1000.0, y_mm / 1000.0, grip_len))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)         # ★ 안 켜면 IK 가 망가진다
        j.CreateJointEnabledAttr(True)
        rs.ApplyAttachmentPointAPI(j.GetPrim())
        pad_paths.append(path)
        log("패드  %s  localPos0 (%+.3f, %+.3f, %.3f)"
            % (path, x_mm / 1000.0, y_mm / 1000.0, grip_len))

    # ── 3. SurfaceGripper ────────────────────────────────────────────
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
    log("그리퍼 %s  흡착점 %d 개  maxGripDistance %.3f m  retryInterval %.1f s"
        % (GRIPPER, len(pad_paths), sc["max_grip_distance_mm"] / 1000.0,
           sc["retry_interval_s"]))
    return pad_paths


def check(stage, log):
    """만든 것을 되읽어 확인한다. 특히 scale 이 안 붙었는지."""
    from pxr import UsdGeom
    import usd.schema.isaac.robot_schema as rs

    ok = True
    for path in (LINK + "/gripper_body", PADS_SCOPE, GRIPPER):
        p = stage.GetPrimAtPath(path)
        log("%-42s %s" % (path, "있음" if p.IsValid() else "!! 없음"))
        ok &= p.IsValid()

    body = stage.GetPrimAtPath(LINK + "/gripper_body")
    if body.IsValid():
        ops = [o.GetOpName() for o in UsdGeom.Xformable(body).GetOrderedXformOps()]
        scaled = [o for o in ops if "scale" in o]
        log("몸통 xformOps: %s  ->  %s" % (ops, "!! scale 이 걸렸다" if scaled else "scale 없음 ✓"))
        ok &= not scaled

    gp = stage.GetPrimAtPath(GRIPPER)
    if gp.IsValid():
        tg = gp.GetRelationship(rs.Relations.ATTACHMENT_POINTS.name).GetTargets()
        log("흡착점 %d 개: %s" % (len(tg), [str(t).split("/")[-1] for t in tg]))
        ok &= len(tg) == 4
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
