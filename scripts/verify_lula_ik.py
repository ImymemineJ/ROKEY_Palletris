#!/usr/bin/env python3
"""B1 — Lula 기동. **IK 를 한 점 푼다.** (작업명세서 6.1 B1, 게이팅 G5 착수)

    ~/isaac-sim-5.1.0/python.sh scripts/verify_lula_ik.py
    ~/isaac-sim-5.1.0/python.sh scripts/verify_lula_ik.py --zone B
    ~/isaac-sim-5.1.0/python.sh scripts/verify_lula_ik.py --target 11600 0 150

`scripts/verify_reach.py` 는 **책상 계산**(scipy 수치 IK)이다. 이 스크립트는 그 자리에
**Lula** 를 넣는다 — 실제로 제어에 쓸 솔버가 도는지, 프레임·좌표계 배선이 맞는지 본다.
G1 Isaac 판(`verify_reach_isaac.py`)이 아래 `bring_up()` 을 그대로 물려 11점을 돌린다.
**Isaac 배선은 여기 한 군데에만 있다** — 두 스크립트가 다른 배선으로 다른 답을 내면
어느 쪽이 맞는지 알 방법이 없다.

## 배선에서 틀리기 쉬운 곳 네 군데 (전부 실제로 밟았다)

1. **EE 프레임.** `tool0` 다. URDF 상 tool0 는 link_6 원점에 rpy 0 0 0 으로 붙어 있어
   지금은 위치가 같지만, 그리퍼를 tool0 밑에 다는 순간 `link_6` 을 쓰던 코드가
   조용히 150 mm 어긋난다.
2. **베이스 배치는 `world.reset()` 전에 프림에 직접 쓴다.** `SingleArticulation(position=)`
   으로 주면 안 옮겨진다 — `fix_base` 로 생긴 `root_joint` 가 PhysX 파싱 시점의 자세로
   로봇을 붙잡는다. 실측으로 (10.9, 0, 0.5) 가 무시되고 원점에 서 있었다.
3. **`set_robot_base_pose()` 를 부른 뒤 Lula 는 전부 월드로 말한다.** 목표도 월드로 주고,
   `compute_forward_kinematics()` 가 돌려주는 것도 **월드**다. 여기에 베이스를 또 더하면
   오프셋 10.9 m 가 통째로 오차로 잡힌다 (실제로 "오차 10911 mm" 를 봤다).
4. **그리퍼 길이.** IK 목표는 tool0 의 위치지 흡착면이 아니다. 수직 하강 자세에서
   흡착면은 tool0 보다 `gripper_length_mm` 만큼 **아래**이므로,
   집을 점(월드)에 +Z 로 그 길이를 더한 것이 tool0 목표다.

## 판정

트랙B 가이드 Day 2 의 G5 착수 기준 — "도달 가능한 점 하나에서 `ok=True` +
관절이 실제로 이동". 여기에 두 가지를 더 본다.

  · **왕복(round-trip)** — 지금 EE 자세를 그대로 목표로 주면 0 오차로 풀려야 한다.
    이게 안 되면 위 1·3 번이 틀린 것이다. 목표점 실패보다 **먼저** 잡힌다.
  · **수렴 시간** — 명세서 11 의 G5 합격 기준이 100 ms 다. 여기서는 한 점만 재고,
    11점 전체는 `verify_reach_isaac.py` 가 잰다.
"""
import argparse
import shutil
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                          # noqa: E402
import import_h2017_usd as IMP                         # make_abs_urdf 재사용  # noqa: E402

ASSET = Path.home() / "isaac_assets/h2017.usd"
# 흡착 그리퍼가 붙은 판 (scripts/build_gripper_usd.py 가 만든다). B3 이후는 이쪽을 쓴다.
ASSET_SUCTION = Path.home() / "isaac_assets/h2017_suction.usd"
PRIM = "/World/h2017"
EE_FRAME = "tool0"

POS_TOL = 0.005          # 5 mm — verify_reach.py 와 같은 기준
G5_SOLVE_MS = 100.0      # 명세서 11 G5


class Tee:
    """출력을 터미널과 파일에 **동시에** 쓴다.

    게이팅 결과는 파일이 정본이다 (명세서 11 "검증 스크립트는 scripts/ 에 커밋").
    터미널 스크롤백은 다음 실행이 밀어내면 사라지고, 남에게 보여줄 수도 없다.
    실제로 같은 검증을 두 번 돌리고도 결과를 못 찾아 세 번째를 돌린 적이 있다.

    `results/` 는 .gitignore 에 있다 — 결과 파일은 커밋되지 않는다.
    """

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = self.path.open("w", encoding="utf-8")
        self._out = sys.stdout

    def write(self, s):
        self._out.write(s)
        self._f.write(s)
        return len(s)

    def flush(self):
        self._out.flush()
        self._f.flush()

    def close(self):
        """stdout 을 되돌리고 파일을 닫는다. kit.close() **전에** 부를 것."""
        sys.stdout = self._out
        self._f.close()


def start_log(path):
    """sys.stdout 을 Tee 로 바꾸고 Tee 를 돌려준다."""
    tee = Tee(path)
    sys.stdout = tee
    return tee


def bring_up(zone="B", ee=EE_FRAME, prim=PRIM, ground=True, render=False, asset=None):
    """h2017 을 씬에 올리고 Lula 를 물린다. **SimulationApp 이 이미 떠 있어야 한다.**

    반환 네임스페이스: world, art, lula, ik, base(m), ready(rad), hz, grip(m), cfg,
    down_quat, ee, render.

    `render=True` 는 GUI(`--gui`)용이다. **배선은 헤드리스와 완전히 같다** — 창만 뜬다.
    눈으로 본 것과 표의 숫자가 다른 실행에서 나오면 둘 다 못 믿는다.
    """
    cfg = _paths.system_config()
    hz = cfg["physics"]["scene"]["time_steps_per_second"]
    grip = cfg["robot"]["gripper_length_mm"] / 1000.0
    base = np.array(cfg["zones"]["robot_base_world_mm"][zone], dtype=float) / 1000.0

    asset = Path(asset) if asset else ASSET
    if not asset.exists():
        sys.exit("no asset: %s\n  -> run scripts/import_h2017_usd.py first" % asset)

    # Lula 는 URDF 로 기구학을 만든다. `package://` 는 못 읽으므로 절대경로 임시본을 쓴다.
    tmp = tempfile.mkdtemp(prefix="h2017_lula_")
    urdf, n_sub, missing = IMP.make_abs_urdf(tmp)
    if missing:
        shutil.rmtree(tmp, ignore_errors=True)
        sys.exit("MISSING MESHES: %s" % missing[:3])

    import isaacsim.robot_motion.motion_generation as mg
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
    from isaacsim.core.utils.rotations import euler_angles_to_quat
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / hz, rendering_dt=1.0 / hz)
    add_reference_to_stage(usd_path=str(asset), prim_path=prim)
    SingleXFormPrim(prim).set_world_pose(position=base)      # ★ reset 전에. docstring 2번
    if ground:
        world.scene.add_default_ground_plane()

    art = SingleArticulation(prim_path=prim, name="h2017")
    world.reset()
    art.initialize()

    lula = mg.LulaKinematicsSolver(robot_description_path=str(_paths.DESCRIPTOR),
                                   urdf_path=str(urdf))
    if ee not in lula.get_all_frame_names():
        sys.exit("EE 프레임 '%s' 가 Lula 에 없다: %s" % (ee, lula.get_all_frame_names()))
    ik = mg.ArticulationKinematicsSolver(art, lula, ee)
    rob_pos, rob_quat = art.get_world_pose()
    lula.set_robot_base_pose(np.array(rob_pos), np.array(rob_quat))   # ★ docstring 3번

    ready = np.array(yaml.safe_load(_paths.DESCRIPTOR.read_text())["default_q"], dtype=float)

    # 수직 하향 — 툴 z 축이 월드 -Z. descriptor 의 default_q 주석과 **같은 값**이라
    # READY 자세에서 시작하면 손목을 돌릴 일이 없다 (둘 다 yaw +180).
    down_quat = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))

    if render:
        # 로봇이 월드 원점에서 10.9 m 떨어져 있다. 기본 카메라로는 화면에 안 잡힌다.
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=base + np.array([2.6, -3.6, 2.2]),
                        target=base + np.array([0.6, 0.0, 0.4]))

    return SimpleNamespace(world=world, art=art, lula=lula, ik=ik, base=base, ready=ready,
                           hz=hz, grip=grip, cfg=cfg, down_quat=down_quat, ee=ee,
                           render=render, urdf_refs=n_sub, tmpdir=tmp,
                           ArticulationAction=ArticulationAction)


def goto(S, q, secs=6.0, tol_deg=0.5):
    """관절 목표 q 를 지령하고 수렴할 때까지(최대 secs) 물리를 돌린다.

    ⚠ **계단 입력**이다. 실제 `robot_motion.py` 는 웨이포인트 보간을 거친다 (명세서 6.2).
      여기서는 "지령을 따라가긴 하는가" 만 보므로 계단으로 두되, **끝까지 기다린다** —
      joint_3 은 속도한계가 100 deg/s 라 190 deg 를 도는 데만 1.9 s 다. 고정 2 s 로
      잘랐다가 남은 14.5 deg 를 처짐으로 오독한 적이 있다.

    반환: (수렴시간 s 또는 None, 잔여속도 deg/s, 관절오차 deg)
    """
    action = S.ArticulationAction(joint_positions=np.asarray(q, dtype=float))
    t_conv = None
    n = int(secs * S.hz)
    for i in range(n):
        S.art.apply_action(action)
        S.world.step(render=S.render)
        err = np.abs(np.rad2deg(np.asarray(S.art.get_joint_positions()[:6]) - q)).max()
        if t_conv is None and err < tol_deg:
            t_conv = (i + 1) / S.hz
    q1 = np.asarray(S.art.get_joint_positions()[:6], dtype=float)
    vmax = float(np.abs(np.rad2deg(S.art.get_joint_velocities()[:6])).max())
    return t_conv, vmax, float(np.abs(np.rad2deg(q1 - np.asarray(q))).max())


def teleport(S, q, steps=3):
    """관절을 순간이동시킨다. 점마다 같은 조건에서 IK 를 풀려고 READY 로 되돌릴 때 쓴다.

    드라이브 목표도 같이 옮기지 않으면 다음 스텝에 옛 목표로 튄다.
    """
    q = np.asarray(q, dtype=float)
    S.art.set_joint_positions(q)
    S.art.set_joint_velocities(np.zeros(6))
    action = S.ArticulationAction(joint_positions=q)
    for _ in range(steps):
        S.art.apply_action(action)
        S.world.step(render=S.render)


def dwell(S, secs):
    """현재 자세로 secs 만큼 더 돌린다. GUI 에서 눈으로 볼 시간을 준다."""
    for _ in range(int(secs * S.hz)):
        S.world.step(render=S.render)


def hold_open(S):
    """창을 열어 둔 채 계속 돌린다. Ctrl-C 로 나간다."""
    print("\n창을 열어 둔다. 끝내려면 Ctrl-C.")
    try:
        while True:
            S.world.step(render=S.render)
    except KeyboardInterrupt:
        print("종료.")


def solve(S, tool_target, quat=None, n=5):
    """tool0 월드 목표를 푼다. 반환 (ok, q, 오차 m, 하강정렬, 중앙값 ms, 최대 ms)."""
    quat = S.down_quat if quat is None else quat
    ms = []
    action = ok = None
    for _ in range(n):
        t0 = time.perf_counter()
        action, ok = S.ik.compute_inverse_kinematics(target_position=np.asarray(tool_target),
                                                     target_orientation=quat)
        ms.append((time.perf_counter() - t0) * 1e3)
    if not ok:
        return False, None, float("nan"), float("nan"), float(np.median(ms)), max(ms)
    q = np.asarray(action.joint_positions, dtype=float)
    p, R = S.lula.compute_forward_kinematics(S.ee, q)          # 월드. docstring 3번
    return (True, q, float(np.linalg.norm(p - np.asarray(tool_target))), float(-R[2, 2]),
            float(np.median(ms)), max(ms))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--target", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                    help="흡착면이 닿을 월드 좌표 (mm). 기본은 팔레트 상면 중앙")
    ap.add_argument("--ee", default=EE_FRAME, help="EE 프레임명 (기본 tool0)")
    ap.add_argument("--settle", type=float, default=6.0, help="지령 후 물리 진행 최대 시간 (s)")
    ap.add_argument("--gui", action="store_true", help="창을 띄운다 (배선은 헤드리스와 동일)")
    ap.add_argument("--hold", action="store_true", help="끝난 뒤 창을 열어 둔다 (--gui 용)")
    ap.add_argument("--log", default=None,
                    help="결과를 남길 파일. 기본 results/verify_lula_ik_<zone>.txt")
    a = ap.parse_args()

    _paths.check()

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})
    # ★ Tee 는 **SimulationApp 이 뜬 뒤에** 건다. Kit 의 스크립팅 플러그인이 기동 중
    #   sys.stdout 을 자기 것으로 바꿔치기하므로(그래서 Kit 로그에 [py stdout] 이 남는다),
    #   앞에서 걸면 통째로 날아간다. 실측: 파일에 런처 배너만 남고 나머지가 전부 사라졌다.
    tee = start_log(a.log or _paths.ROOT / "results" / ("verify_lula_ik_%s.txt" % a.zone))

    from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats   # noqa: E402

    S = bring_up(zone=a.zone, ee=a.ee, render=a.gui)
    cfg = S.cfg

    if a.target is None:
        ox, oy, oz = cfg["zones"]["pallet_origin_world_mm"][a.zone]
        suction_mm = (ox + cfg["pallet"]["width_mm"] / 2.0,
                      oy + cfg["pallet"]["depth_mm"] / 2.0, float(oz))
    else:
        suction_mm = tuple(a.target)
    suction = np.array(suction_mm, dtype=float) / 1000.0
    target = suction + np.array([0.0, 0.0, S.grip])       # docstring 4번

    print("ASSET   %s" % ASSET)
    print("ZONE    %s   base world %s mm" % (a.zone, list(np.round(S.base * 1000).astype(int))))
    print("DOF     %d  %s" % (S.art.num_dof, S.art.dof_names))
    print("PHYSICS %.0f Hz (system.yaml)" % S.hz)
    print("URDF    %s  (package:// -> abs, %d refs)" % (_paths.URDF.name, S.urdf_refs))
    print("\nLULA    frames %s" % S.lula.get_all_frame_names())
    print("        joints %s" % S.lula.get_joint_names())
    if list(S.lula.get_joint_names()) != list(S.art.dof_names):
        # 이름 순서가 어긋나면 ArticulationKinematicsSolver 가 관절을 뒤섞어 넣는다.
        print("        ⚠ descriptor cspace 순서가 dof_names 와 다르다 — descriptor 를 고칠 것")

    teleport(S, S.ready, steps=int(0.5 * S.hz))
    ee_pos, ee_rot = S.ik.compute_end_effector_pose()
    print("\nSTART   q(READY) %s deg" % np.round(np.rad2deg(S.art.get_joint_positions()[:6]), 1))
    print("        %-6s world %s   툴 z %s"
          % (a.ee, np.round(ee_pos, 4), np.round(ee_rot[:, 2], 3)))

    ok_all = True

    # ---- 1) 왕복: 지금 자세를 그대로 목표로. 배선이 맞으면 0 오차.
    ok, _, rt_err, _, rt_ms, _ = solve(S, ee_pos, quat=rot_matrices_to_quats(ee_rot), n=1)
    good = ok and rt_err < POS_TOL
    print("\n[1] 왕복  ok=%s  %.1f ms  오차 %.2f mm   %s"
          % (ok, rt_ms, rt_err * 1000, "OK" if good else "FAIL"))
    if not good:
        ok_all = False
        print("    -> 프레임명·베이스 오프셋을 먼저 의심할 것 (docstring 1·3번)")

    # ---- 2) 실제 목표 한 점.
    print("\n[2] 목표  흡착면 %s mm  ->  %s 목표 %s mm (그리퍼 %.0f mm 보정)"
          % (np.round(suction * 1000), a.ee, np.round(target * 1000), S.grip * 1000))
    print("    베이스로부터 %.0f mm" % (np.linalg.norm(target - S.base) * 1000))
    ok, q, sol_err, down, ms, ms_max = solve(S, target)
    print("    ok=%s   %.1f ms (중앙값 5회, 최대 %.1f)   %s"
          % (ok, ms, ms_max, "OK" if ms < G5_SOLVE_MS else "FAIL <-- G5 100ms"))
    if not ok:
        print("    IK 수렴 실패 — 도달범위 밖이거나 자세 제약이 과하다\nRESULT: FAIL")
        sys.stdout.flush()
        kit.close()
        return 1
    print("    해 q   %s deg" % np.round(np.rad2deg(q), 1))
    print("    해 오차 %.2f mm   하강정렬 %.4f" % (sol_err * 1000, down))
    ok_all &= sol_err < POS_TOL and ms < G5_SOLVE_MS

    # ---- 3) 관절이 실제로 움직이는가 (가이드의 G5 착수 기준).
    q0 = np.asarray(S.art.get_joint_positions()[:6], dtype=float)
    t_conv, vmax, joint_err = goto(S, q, secs=a.settle)
    q1 = np.asarray(S.art.get_joint_positions()[:6], dtype=float)
    moved = float(np.abs(np.rad2deg(q1 - q0)).max())
    ee1, rot1 = S.ik.compute_end_effector_pose()
    real_err = float(np.linalg.norm(ee1 - target))
    print("\n[3] 이동  최대 %.1f s 물리 진행 (계단 입력)" % a.settle)
    print("    q      %s deg" % np.round(np.rad2deg(q1), 1))
    print("    최대 이동 %.1f deg   해 대비 관절오차 %.2f deg   잔여속도 %.2f deg/s"
          % (moved, joint_err, vmax))
    print("    수렴(0.5 deg) %s" % ("%.2f s" % t_conv if t_conv is not None else "미수렴"))
    print("    실제 %s 위치 %s   목표대비 %.1f mm   툴 z %s"
          % (a.ee, np.round(ee1, 4), real_err * 1000, np.round(rot1[:, 2], 3)))
    ok_all &= moved > 1.0 and t_conv is not None
    if real_err >= POS_TOL:
        print("    ⚠ 해 오차는 %.2f mm 인데 실제는 %.1f mm." % (sol_err * 1000, real_err * 1000))
        if vmax > 1.0:
            print("      잔여속도가 있다 = **아직 가는 중**이다. --settle 을 늘려 볼 것.")
        else:
            print("      멈춘 채 오차가 남았다 = 드라이브 추종 문제다."
                  " verify_drive_hold.py 로 게인을 볼 것 (IK 문제가 아니다)")

    print("\nRESULT: %s" % ("PASS — B1 Lula 기동 완료. 11점은 verify_reach_isaac.py (G1/G5)"
                            if ok_all else "FAIL — 위 항목"))
    sys.stdout.flush()
    if a.hold:
        hold_open(S)
    print("\n결과: %s" % tee.path)
    tee.close()
    shutil.rmtree(S.tmpdir, ignore_errors=True)
    kit.close()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
