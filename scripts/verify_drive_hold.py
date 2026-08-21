#!/usr/bin/env python3
"""드라이브 게인 실측 — 팔이 자세를 **버티는가**.

    ~/isaac-sim-5.1.0/python.sh scripts/verify_drive_hold.py

`import_h2017_usd.py` 가 박은 stiffness/damping 이 쓸 만한지 숫자로 본다.
게인은 계산으로 고르고 **실측으로 확정한다** — B1/G5 의 준비물이다.

## 무엇을 재나

중력만 켜 놓고 관절 목표를 고정한 뒤 3초를 돌린다.

  · **처짐(sag)** — 목표와 실제 관절각의 정상상태 차이. 크면 stiffness 부족.
  · **진동(ripple)** — 마지막 1초 동안의 관절각 진폭. 크면 damping 부족.
  · **토크 포화** — joint_2 는 최악 자세에서 정하중만 680 N.m 가 필요하다 (README 8).

자세 두 가지를 본다. 접힌 자세는 쉽고, **수평 자세가 최악**이다.

## 판정

sag <= 1.0 deg, ripple <= 0.05 deg 면 통과로 본다.
수평 자세에서 joint_2 sag 가 1 deg 면 툴 끝에서 약 1.7 m * sin(1 deg) = 30 mm 다.
G1 여유가 115 mm 뿐이므로 이보다 크면 도달범위 판정까지 흔들린다.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                          # noqa: E402

ASSET = Path.home() / "isaac_assets/h2017.usd"
PRIM = "/World/h2017"

POSES_DEG = {
    # 접힌 자세 — 중력토크가 작다. 여기서도 못 버티면 게인이 크게 잘못된 것이다.
    "folded":     [0.0, 0.0, -90.0, 0.0, -90.0, 0.0],
    # 수평 완전 전개 — joint_2 최악. 사양서 도달범위의 먼 코너가 이 근처다.
    "horizontal": [0.0, 90.0, 0.0, 0.0, 0.0, 0.0],
}
SAG_TOL_DEG = 1.0
RIPPLE_TOL_DEG = 0.05


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()

    _paths.check()
    if not ASSET.exists():
        sys.exit("no asset: %s\n  -> run scripts/import_h2017_usd.py first" % ASSET)
    cfg = _paths.system_config()
    hz = cfg["physics"]["scene"]["time_steps_per_second"]

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": True})

    import omni.kit.app                                        # noqa: E402
    from isaacsim.core.api import World                        # noqa: E402
    from isaacsim.core.prims import SingleArticulation         # noqa: E402
    from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
    from isaacsim.core.utils.types import ArticulationAction   # noqa: E402
    from pxr import UsdPhysics                                 # noqa: E402

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / hz, rendering_dt=1.0 / hz)
    add_reference_to_stage(usd_path=str(ASSET), prim_path=PRIM)
    world.scene.add_default_ground_plane()
    world.reset()

    art = SingleArticulation(prim_path=PRIM, name="h2017")
    art.initialize()
    print("ASSET  %s" % ASSET)
    print("DOF    %d  %s" % (art.num_dof, art.dof_names))
    print("PHYSICS %.0f Hz (system.yaml)" % hz)

    stage = world.stage
    gains = []
    for j in stage.Traverse():
        if j.GetTypeName() == "PhysicsRevoluteJoint":
            d = UsdPhysics.DriveAPI.Get(j, "angular")
            gains.append((j.GetName(), d.GetStiffnessAttr().Get(),
                          d.GetDampingAttr().Get(), d.GetMaxForceAttr().Get()))
    gains.sort()
    print("DRIVE  %s" % ", ".join("%s=%g/%g" % (n, s, dmp) for n, s, dmp, _ in gains[:2]))

    n = int(a.secs * hz)
    tail = int(1.0 * hz)
    allok = True
    for name, deg in POSES_DEG.items():
        target = np.deg2rad(np.array(deg, dtype=float))
        art.set_joint_positions(target)
        art.set_joint_velocities(np.zeros(6))
        # ⚠ SingleArticulation 에는 set_joint_position_targets 가 없다.
        #   드라이브 목표는 apply_action 으로 준다.
        hold = ArticulationAction(joint_positions=target)
        art.apply_action(hold)

        hist = []
        for _ in range(n):
            art.apply_action(hold)
            world.step(render=False)
            hist.append(np.asarray(art.get_joint_positions())[:6].copy())
        hist = np.rad2deg(np.array(hist))

        sag = hist[-1] - np.rad2deg(target)
        ripple = hist[-tail:].max(axis=0) - hist[-tail:].min(axis=0)
        ok = (np.abs(sag).max() <= SAG_TOL_DEG) and (ripple.max() <= RIPPLE_TOL_DEG)
        allok &= ok
        print("\nPOSE %-11s target %s" % (name, deg))
        print("  %-10s %9s %9s %9s" % ("joint", "sag_deg", "ripple", "final"))
        for i in range(6):
            mark = ""
            if abs(sag[i]) > SAG_TOL_DEG:
                mark += "  <-- SAG"
            if ripple[i] > RIPPLE_TOL_DEG:
                mark += "  <-- RIPPLE"
            print("  joint_%-4d %+9.3f %9.3f %+9.3f%s"
                  % (i + 1, sag[i], ripple[i], hist[-1][i], mark))
        # 툴 끝에서의 오차로 환산해 본다 (1.7 m 팔).
        print("  worst sag %.3f deg  -> about %.0f mm at 1.7 m"
              % (np.abs(sag).max(), 1700 * np.sin(np.deg2rad(np.abs(sag).max()))))
        print("  %s" % ("OK" if ok else "FAIL"))

    print("\nTOL    sag <= %.1f deg, ripple <= %.2f deg" % (SAG_TOL_DEG, RIPPLE_TOL_DEG))
    print("RESULT: %s" % ("PASS" if allok else "FAIL - tune gains in import_h2017_usd.py"))
    sys.stdout.flush()
    kit.close()
    return 0 if allok else 1


if __name__ == "__main__":
    sys.exit(main())
