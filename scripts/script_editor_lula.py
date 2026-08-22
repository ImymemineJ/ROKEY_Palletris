# Isaac Sim GUI 의 Script Editor 에 **통째로 붙여넣고 Ctrl+Enter**.
#
#   1. 터미널:  ~/isaac-sim-5.1.0/isaac-sim.sh
#      ⚠ `python.sh ... --gui` 로 뜨는 "Isaac Sim Python" 창에는 Script Editor 가 **없다**.
#        그건 apps/isaacsim.exp.base.python.kit ("A trimmed down app") 이고,
#        omni.kit.window.script_editor 는 isaacsim.exp.full.kit 에만 들어 있다.
#   2. File > Open  ->  ~/isaac_assets/h2017.usd     (또는 김학민의 A3 씬)
#   3. Window > Script Editor  ->  이 파일을 붙여넣고 **Ctrl+Enter**
#   4. 스크립트가 다음에 뭘 하라고 알려준다 (옮기기 -> Play -> 풀기). 시키는 대로
#      버튼을 누르고 Ctrl+Enter 를 다시 누르면 된다.
#
# 출력은 Script Editor 아래 콘솔에 뜨고, Kit 로그에도 `[py stdout]` 으로 남는다.
# 터미널에서 읽으려면:  python3 scripts/read_log.py
#
# ─────────────────────────────────────────────────────────────────────────────
# POINT 만 바꾸고 Ctrl+Enter 를 다시 누르면 팔이 그 점으로 간다.
#   0      = READY 자세 (descriptor 의 default_q)
#   1~11   = 작업명세서 6.3 의 도달범위 11점 (verify_reach.py 와 **같은 점**)
#
# 눈으로 볼 것: 7·8번에서 손목(joint_5)이 어떻게 꺾이는지. 거기 Lula 해가 j5 = +86.9 deg
# 인데 descriptor 의 default_q 는 j5 = -90 deg 다 — 부호가 반대다.
POINT = 7
ZONE = "B"
# ─────────────────────────────────────────────────────────────────────────────

print("started")                      # read_log.py 가 이 표식부터 잘라 보여준다

import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path.home() / "palletris_ws"   # README 의 clone 위치
if str(REPO / "scripts") not in sys.path:
    sys.path.insert(0, str(REPO / "scripts"))

import _paths
import fk as F
import import_h2017_usd as IMP
import verify_reach as VR
import yaml

import omni.timeline
import omni.usd
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdGeom, UsdPhysics


def main():
    cfg = _paths.system_config()
    grip = cfg["robot"]["gripper_length_mm"] / 1000.0
    ped_mm = cfg["robot"]["pedestal_height_mm"]
    base = np.array(cfg["zones"]["robot_base_world_mm"][ZONE], dtype=float) / 1000.0

    # ---- 1) 스테이지에서 아티큘레이션 루트를 찾는다.
    #      프림 경로를 손으로 적지 않는다. h2017.usd 를 그냥 열면 `/h2017` 이고
    #      (add_reference 로 붙이면 `/World/h2017`), A3 씬에서는 또 다를 수 있다.
    stage = omni.usd.get_context().get_stage()
    roots = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if not roots:
        print("아티큘레이션이 없다. File > Open > ~/isaac_assets/h2017.usd 를 먼저 하라.")
        return
    prim_path = str(roots[0].GetPath()).split("/root_joint")[0]
    print("ARTICULATION %s   (스테이지에서 %d 개 발견)" % (prim_path, len(roots)))

    # PhysicsScene 이 없어도 PhysX 가 기본 씬으로 돈다(확인함). 다만 타임스텝이
    # system.yaml 의 60 Hz 가 아니라 Kit 기본값이라, **시간이 걸린 수치는 못 믿는다.**
    n_scene = len([p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)])
    if n_scene == 0:
        print("NOTE  PhysicsScene 이 없다 (h2017.usd 는 create_physics_scene=False 로 만들었다).")
        print("      팔은 움직이지만 타임스텝이 system.yaml 60 Hz 가 아니다 —")
        print("      수렴 시간 같은 숫자는 verify_reach_isaac.py 로 재라.")

    tl = omni.timeline.get_timeline_interface()
    playing = tl.is_playing()

    # ---- 2) 로봇이 존 베이스에 있나. 없으면 옮긴다 — **정지 상태에서만 된다.**
    #      fix_base 로 생긴 root_joint 가 PhysX 파싱 시점의 자세로 로봇을 붙잡기 때문에,
    #      재생 중에 옮기면 시각만 움직이고 물리는 안 따라온다.
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    cur = np.array(xf.ComputeLocalToWorldTransform(0).ExtractTranslation(), dtype=float)
    if np.linalg.norm(cur - base) > 1e-3:
        print("\n로봇이 %s m 에 있다. 존 %s 베이스는 %s m 다."
              % (np.round(cur, 3), ZONE, np.round(base, 3)))
        if playing:
            print("-> 타임라인 **Stop(■)** 을 누르고 Ctrl+Enter 를 다시 눌러라. "
                  "재생 중에는 못 옮긴다.")
            return
        SingleXFormPrim(prim_path).set_world_pose(position=base)
        print("-> 베이스로 옮겼다. 이제 **Play(▶)** 를 누르고 Ctrl+Enter 를 다시 눌러라.")
        return

    if not playing:
        print("\n로봇은 베이스에 있다. -> **Play(▶)** 를 누르고 Ctrl+Enter 를 다시 눌러라.")
        print("   (SingleArticulation.initialize() 는 Play 이후에만 된다 — Isaac 문서 명시)")
        return

    # ---- 3) 여기부터는 재생 중 + 제자리. 실제 작업.
    art = SingleArticulation(prim_path=prim_path, name="h2017_se")
    art.initialize()
    print("DOF %d  %s" % (art.num_dof, art.dof_names))

    urdf, n_sub, missing = IMP.make_abs_urdf(tempfile.mkdtemp(prefix="h2017_se_"))
    if missing:
        print("메시를 못 찾겠다: %s" % missing[:3])
        return
    lula = mg.LulaKinematicsSolver(robot_description_path=str(_paths.DESCRIPTOR),
                                   urdf_path=str(urdf))
    ik = mg.ArticulationKinematicsSolver(art, lula, "tool0")
    rob_pos, rob_quat = art.get_world_pose()
    lula.set_robot_base_pose(np.array(rob_pos), np.array(rob_quat))
    print("BASE  %s m  — 이 뒤로 Lula 는 목표도 FK 결과도 전부 **월드**로 말한다"
          % np.round(rob_pos, 4))

    ready = np.array(yaml.safe_load(_paths.DESCRIPTOR.read_text())["default_q"], dtype=float)
    down_quat = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))   # 툴 z 가 월드 -Z

    if POINT == 0:
        q = ready
        print("\nTARGET  0. READY (default_q)")
    else:
        pts = VR.targets(cfg, ZONE, ped_mm)
        if not 1 <= POINT <= len(pts):
            print("POINT 는 0~%d" % len(pts))
            return
        name, (x, y, z) = pts[POINT - 1]
        suction = np.array([x, y, z], dtype=float) / 1000.0
        target = suction + np.array([0.0, 0.0, grip])   # IK 목표는 tool0, 흡착면이 아니다
        action, ok = ik.compute_inverse_kinematics(target_position=target,
                                                   target_orientation=down_quat)
        print("\nTARGET  %d. %s" % (POINT, name))
        print("        흡착면 %s mm  ->  tool0 %s mm"
              % (np.round(suction * 1000), np.round(target * 1000)))
        if not ok:
            print("        IK 수렴 실패")
            return
        q = np.asarray(action.joint_positions, dtype=float)
        p, R = lula.compute_forward_kinematics("tool0", q)   # 월드. 베이스를 또 더하지 말 것
        print("        해 오차 %.2f mm   하강정렬 %.4f"
              % (np.linalg.norm(p - target) * 1000, -R[2, 2]))
        clr, where = VR.pedestal_clearance(q, F.load_joints(), VR.load_spheres(),
                                           ped_mm / 1000.0)
        print("        페데스탈 여유 %.0f mm (%s)" % (clr * 1000, where))

    print("        q  %s deg" % np.round(np.rad2deg(q), 1))
    print("        j5 %+.1f deg   (default_q 의 j5 는 %+.1f deg — 0 이 손목 특이점)"
          % (np.rad2deg(q[4]), np.rad2deg(ready[4])))

    # ---- 4) 지령만 하고 손을 뗀다.
    #      여기서 world.step() 을 루프로 돌리면 **UI 가 얼어붙는다.**
    #      물리는 GUI 타임라인이 알아서 돌린다.
    art.apply_action(ArticulationAction(joint_positions=q))
    print("\n지령 완료. 뷰포트에서 팔이 움직인다.")
    print("POINT 를 바꾸고 Ctrl+Enter 를 다시 누르면 다음 점으로 간다.")


main()
