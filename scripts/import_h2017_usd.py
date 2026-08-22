#!/usr/bin/env python3
"""H2017 URDF -> USD 임포트. **`~/isaac_assets/h2017.usd` 를 만든다.**

    ~/isaac-sim-5.1.0/python.sh scripts/import_h2017_usd.py
    ~/isaac-sim-5.1.0/python.sh scripts/import_h2017_usd.py --force   # 덮어쓰기

김학민의 A3 배치 스크립트가 이 경로를 참조한다 (트랙B 가이드 Day 1).

## 왜 GUI 로 안 하고 스크립트로 하나

**USD 는 저장소에 못 넣는다** (`.gitignore`, 작업명세서 1.7). 그러면 4대에서
같은 로봇을 쓴다는 보장이 사라진다 — 각자 GUI 로 임포트하면 옵션 하나가 달라도
다른 로봇이 된다. **파일 대신 파일을 만드는 스크립트를 커밋한다.**
누구든 이걸 돌리면 비트 단위로 같은 결과가 나온다.

## 옵션 중 실수하면 안 되는 것

- **`merge_fixed_joints = False`** — `tool0` 는 `joint_6-tool0` 고정조인트로 붙어 있다.
  합치면 **tool0 프림이 사라진다.** 그러면 IK 의 TCP 프레임이 없어져 Lula 가
  `link_6`(플랜지 안쪽)을 잡게 되고, 흡착면이 150 mm 어긋난다.
- **`fix_base = True`** — 로봇은 페데스탈에 볼트로 고정돼 있다.
- **`distance_scale = 1.0`** — URDF 가 m 단위다. 여기가 틀리면 로봇이 1000배가 된다.

## 로그의 "Unresolved reference prim path .../visuals/tool0" 경고는 무해하다

`tool0` 과 `base` 는 **메시가 없는 링크**다(tool0 은 좌표 프레임, base 는 고정 루트).
임포터가 그래도 `visuals` 프림을 만들고 없는 대상을 참조해서 경고가 수십 줄 나온다.
프림 자체는 정상이고 아래 CHECKS 가 tool0 존재를 확인한다. 쫓아갈 필요 없다.

## package:// 는 임포터가 못 읽는다

커밋된 `h2017.urdf` 는 ROS 관례대로 `package://dsr_description2/meshes/...` 를 쓴다.
URDF Importer 는 이걸 못 찾으므로, 절대경로로 치환한 **임시 URDF** 를 만들어 넘긴다.
구 저장소는 이 치환본(`h2017_isaac_sim.urdf`)을 통째로 커밋해 뒀는데, 그 안에
`/home/byeongjik/...` 이 박혀서 남의 PC 에서 못 썼다. 그래서 여기서는 커밋하지 않고
매번 임시로 만든다.
"""
import argparse
import os
import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                          # noqa: E402

DEFAULT_OUT = Path.home() / "isaac_assets/h2017.usd"
PKG_PREFIX = "package://dsr_description2/"


def make_abs_urdf(tmpdir):
    """`package://` 를 저장소 절대경로로 치환한 임시 URDF 를 만든다.

    원본과 **같은 폴더**에 두지 않는다 — 저장소를 더럽히지 않기 위해서다.
    메시는 절대경로가 되므로 어디에 두든 해석된다.
    """
    src = _paths.URDF.read_text()
    base = str(_paths.ROBOT / "urdf") + "/"
    out = src.replace(PKG_PREFIX, base)
    n = src.count(PKG_PREFIX)
    p = Path(tmpdir) / "h2017_abs.urdf"
    p.write_text(out)
    missing = [ln.split('filename="')[1].split('"')[0]
               for ln in out.splitlines() if 'filename="' in ln]
    missing = [m for m in dict.fromkeys(missing) if not Path(m).exists()]
    return p, n, missing


def cfg_set(cfg, name, value):
    """있는 필드만 설정한다. 임포터 버전마다 이름이 조금씩 다르다.

    ⚠ `hasattr` 만으로는 부족하다. `set_collision_from_visuals` 처럼 **읽기 전용
      메서드**인 이름이 있어서 `setattr` 이 AttributeError 를 던진다.
    """
    if not hasattr(cfg, name):
        return False
    try:
        setattr(cfg, name, value)
        return True
    except AttributeError:
        fn = getattr(cfg, name)
        if callable(fn):                    # 세터 메서드 형태인 경우
            fn(value)
            return True
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--force", action="store_true", help="이미 있으면 지우고 다시 만든다")
    # 드라이브 게인. 기본값은 **URDF Importer 자신의 기본값**이다 — 내가 지어낸 수가 아니다.
    # GUI 로 기본 옵션 임포트했을 때와 같은 값이 나오게 하려는 것. B1/G5 에서 실측해 정한다.
    ap.add_argument("--stiffness", type=float, default=1000.0,
                    help="관절 위치 드라이브 stiffness (기본 1000 = 임포터 기본값)")
    ap.add_argument("--damping", type=float, default=100.0,
                    help="관절 위치 드라이브 damping (기본 100 = 임포터 기본값)")
    a = ap.parse_args()

    _paths.check()
    out = Path(a.out).expanduser().resolve()
    if out.exists() or (out.parent / "configuration").exists():
        if not a.force:
            sys.exit("ALREADY EXISTS: %s\n  -> use --force to overwrite" % out)
        out.unlink(missing_ok=True)
        shutil.rmtree(out.parent / "configuration", ignore_errors=True)
    out.parent.mkdir(parents=True, exist_ok=True)

    tmp = tempfile.mkdtemp(prefix="h2017_urdf_")
    urdf, n_sub, missing = make_abs_urdf(tmp)
    print("URDF   %s" % _paths.URDF)
    print("       package:// -> abs path, %d refs, temp %s" % (n_sub, urdf))
    if missing:
        print("MISSING MESHES (%d):" % len(missing))
        for m in missing[:5]:
            print("  %s" % m)
        sys.exit("  -> meshes not found. did you commit urdf/meshes/ ?")
    print("OUT    %s" % out)

    cfg_all = _paths.system_config()
    art_cfg = cfg_all["physics"]["articulation"]

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": True})

    import omni.kit.commands                                  # noqa: E402
    import omni.usd                                           # noqa: E402
    from pxr import PhysxSchema, Usd, UsdPhysics              # noqa: E402

    status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    applied = {}
    # ⚠ merge_fixed_joints 를 True 로 두면 tool0 이 사라진다. 위 docstring 참조.
    applied["merge_fixed_joints"] = cfg_set(cfg, "merge_fixed_joints", False)
    # ★ h2017.urdf 의 <dynamics> 에는 friction 만 있고 damping 이 없다.
    #   override_joint_dynamics=False(기본)면 임포터가 URDF 를 따라 damping=0 으로 만든다.
    applied["override_joint_dynamics"] = cfg_set(cfg, "override_joint_dynamics", True)
    #   ⚠ 이 두 필드는 **효과가 없다.** 1000/100 과 5000/500 으로 두 번 임포트해
    #     결과를 비교했더니 6축 전부 소수점까지 같았다 (stiffness 625, damping 0.005~4.9).
    #     임포터가 natural frequency 경로(K = m*f^2, D = 2*r*f*m, f 기본 25)로 계산해
    #     이 값들을 무시한다. 그래서 **임포트 후에 직접 박는다** (아래 참조).
    #     그래도 넣어 두는 이유: 버전이 바뀌어 살아나면 값이 일치하는 게 낫다.
    applied["default_drive_strength"] = cfg_set(cfg, "default_drive_strength", a.stiffness)
    applied["default_position_drive_damping"] = cfg_set(
        cfg, "default_position_drive_damping", a.damping)
    applied["fix_base"] = cfg_set(cfg, "fix_base", True)
    applied["distance_scale"] = cfg_set(cfg, "distance_scale", 1.0)
    applied["import_inertia_tensor"] = cfg_set(cfg, "import_inertia_tensor", True)
    applied["convex_decomp"] = cfg_set(cfg, "convex_decomp", False)
    applied["self_collision"] = cfg_set(cfg, "self_collision", False)
    applied["make_default_prim"] = cfg_set(cfg, "make_default_prim", True)
    applied["create_physics_scene"] = cfg_set(cfg, "create_physics_scene", False)
    # 충돌 메시가 따로 있으므로 시각 메시에서 만들지 않는다.
    for nm in ("collision_from_visuals", "set_collision_from_visuals"):
        applied[nm] = cfg_set(cfg, nm, False)
    try:
        from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
        applied["default_drive_type"] = cfg_set(
            cfg, "default_drive_type", UrdfJointTargetType.JOINT_DRIVE_POSITION)
    except Exception as e:
        print("NOTE  drive type enum: %s" % e)
    print("CONFIG %s" % ", ".join("%s=%s" % (k, "ok" if v else "N/A")
                                  for k, v in applied.items()))

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf),
        import_config=cfg,
        dest_path=str(out),
        get_articulation_root=True,
    )
    kit.update()
    print("IMPORT status=%s  articulation root=%s" % (status, prim_path))
    shutil.rmtree(tmp, ignore_errors=True)

    if not out.exists():
        kit.close()
        sys.exit("FAILED: %s was not written" % out)

    # ---- 저장된 USD 를 열어 검사하고 마무리 속성을 박는다.
    #      임포트 직후의 **라이브 스테이지**에 쓰면 세션 레이어로 가서 파일에 안 남는다.
    #      그래서 저장된 파일을 따로 연다.
    #
    #      ⚠ `kit.close()` 를 여기서 부르면 안 된다. `--/app/fastShutdown=True` 라
    #        프로세스가 그 자리에서 죽고, **버퍼에 있던 stdout 이 통째로 날아간다.**
    #        (실제로 exit 0 인데 출력이 한 줄도 안 남았다.) 검사를 다 끝내고 맨 마지막에 닫는다.
    stage = Usd.Stage.Open(str(out))
    root = stage.GetDefaultPrim()
    print("\nDEFAULT PRIM %s" % (root.GetPath() if root else "NONE"))

    joints, links = [], []
    tool0 = None
    for p in stage.Traverse():
        t = p.GetTypeName()
        if t == "PhysicsRevoluteJoint":
            joints.append(p)
        if p.GetName() == "tool0":
            tool0 = p
        if t == "Xform" and p.GetParent() == root:
            links.append(p.GetName())

    # 아티큘레이션 솔버 반복수 — system.yaml 이 유일한 출처 (철칙 3).
    art_root = None
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_root = p
            break
    if art_root is not None:
        api = PhysxSchema.PhysxArticulationAPI.Apply(art_root)
        api.CreateSolverPositionIterationCountAttr(art_cfg["solver_position_iterations"])
        api.CreateSolverVelocityIterationCountAttr(art_cfg["solver_velocity_iterations"])
        print("ARTICULATION %s  solver pos=%d vel=%d  (from system.yaml)"
              % (art_root.GetPath(), art_cfg["solver_position_iterations"],
                 art_cfg["solver_velocity_iterations"]))
    else:
        print("ARTICULATION ROOT NOT FOUND  <-- check fix_base")

    # ---- 드라이브 게인을 직접 박는다.
    #
    # 임포터가 자동으로 준 값(stiffness 625, damping 0.005~4.9)은 못 쓴다.
    # damping 0.67(joint_2)은 10 deg/s 에서 6.7 N.m 라 사실상 감쇠가 없다 —
    # 76 kg 짜리 팔이 중력 아래서 진동한다.
    #
    # 여기 값의 근거 두 가지가 겹친다.
    #   1) **임포터 자신의 기본값**이 stiffness 1000 / damping 100 이다.
    #      (`default_drive_strength` / `default_position_drive_damping` 의 초기값)
    #      즉 내가 지어낸 수가 아니라, 저 knob 이 동작했다면 나왔을 값이다.
    #   2) 임계감쇠 추정과 자릿수가 맞는다. joint_2 는 최악 정하중 약 680 N.m 인데
    #      (README 8), 처짐을 0.5 deg 이하로 두려면 K >= 1360 N.m/deg 이 필요하고,
    #      유효관성 약 60 kg.m^2 에 대한 임계감쇠는 D = 2*sqrt(K*m) 로 약 90 N.m.s/deg 다.
    #
    # ★ 그래도 이건 **시작점**이다. B1/G5 에서 실측(자세 유지 오차·진동)으로 확정한다.
    #   확정되면 system.yaml 에 robot.drive 로 올리고 여기서 읽게 바꾼다 (철칙 3).
    print("\nDRIVE  stiffness=%.0f damping=%.0f  (importer defaults; measured in G5)"
          % (a.stiffness, a.damping))
    print("\n%-16s %10s %10s %12s %12s %12s"
          % ("joint", "lower", "upper", "maxForce", "stiffness", "damping"))
    for j in sorted(joints, key=lambda p: p.GetName()):
        d = UsdPhysics.DriveAPI.Get(j, "angular")
        lo = j.GetAttribute("physics:lowerLimit").Get()
        hi = j.GetAttribute("physics:upperLimit").Get()
        d.CreateStiffnessAttr().Set(a.stiffness)
        d.CreateDampingAttr().Set(a.damping)
        f = d.GetMaxForceAttr().Get() if d else None
        s = d.GetStiffnessAttr().Get() if d else None
        dm = d.GetDampingAttr().Get() if d else None
        print("%-16s %10.2f %10.2f %12s %12s %12s"
              % (j.GetName(), lo if lo is not None else float("nan"),
                 hi if hi is not None else float("nan"),
                 "%.4g" % f if f is not None else "-",
                 "%.4g" % s if s is not None else "-",
                 "%.4g" % dm if dm is not None else "-"))
    stage.GetRootLayer().Save()

    ok = True
    print("\nCHECKS")
    print("  revolute joints  %d  (want 6)          %s"
          % (len(joints), "OK" if len(joints) == 6 else "FAIL"))
    ok &= len(joints) == 6
    print("  tool0 prim       %s   %s"
          % (tool0.GetPath() if tool0 else "MISSING",
             "OK" if tool0 else "FAIL <-- merge_fixed_joints must be False"))
    ok &= tool0 is not None
    bad_gain = [j.GetName() for j in joints
                if abs((UsdPhysics.DriveAPI.Get(j, "angular").GetDampingAttr().Get() or 0)
                       - a.damping) > 1e-6]
    print("  drive gains      %s  %s"
          % ("all %.0f/%.0f" % (a.stiffness, a.damping) if not bad_gain
             else "WRONG on %s" % bad_gain, "OK" if not bad_gain else "FAIL"))
    ok &= not bad_gain
    j2 = next((j for j in joints if j.GetName() == "joint_2"), None)
    if j2 is not None:
        f = UsdPhysics.DriveAPI.Get(j2, "angular").GetMaxForceAttr().Get()
        good = f is not None and f >= 999.0
        print("  joint_2 maxForce %s  (want 1000, URDF effort)  %s"
              % (f, "OK" if good else "FAIL <-- stale urdf?"))
        ok &= good
    size = sum(f.stat().st_size for f in out.parent.rglob("*") if f.is_file())
    print("  asset size       %.1f MB at %s" % (size / 1048576, out.parent))
    print("\nRESULT: %s" % ("OK - share the path with track A" if ok else "CHECK THE FAILS"))
    sys.stdout.flush()
    kit.close()                      # 맨 마지막. 이 줄 뒤로는 아무것도 실행되지 않는다.
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
