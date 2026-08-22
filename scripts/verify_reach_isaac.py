#!/usr/bin/env python3
"""G1 도달범위 11점 — **Isaac / Lula 판**. G5(100 ms 수렴)를 겸한다.

    ~/isaac-sim-5.1.0/python.sh scripts/verify_reach_isaac.py
    ~/isaac-sim-5.1.0/python.sh scripts/verify_reach_isaac.py --zone B --move
    ~/isaac-sim-5.1.0/python.sh scripts/verify_reach_isaac.py --ready-each

`verify_reach.py` 의 **책상 계산(scipy)** 자리에 실제 제어에 쓸 **Lula** 를 넣은 것이다.
점 11개는 같은 함수(`verify_reach.targets`)에서 가져온다 — 두 스크립트가 서로 다른
점을 보고 다른 판정을 내는 일을 원천 차단한다.

| | verify_reach.py (책상) | 이 스크립트 (Isaac) |
|---|---|---|
| IK | scipy 수치해, 재시작 60회 | Lula `compute_inverse_kinematics` |
| 자세 | 여러 해 중 페데스탈 여유 최대를 **고름** | Lula 가 준 **한 해** 그대로 |
| 물리 | 없음 | `--move` 시 실제로 지령하고 도달 확인 |

**자세를 고르지 않는다는 점이 핵심이다.** 6축 팔은 한 점을 여러 자세로 잡고, 책상판은
그중 페데스탈을 피하는 자세를 골라 "도달 가능"이라 판정했다. 런타임에는 그런 선택권이
없다 — Lula 가 주는 해 하나로 가야 한다. 그래서 여기서는 **Lula 의 해에 대해**
페데스탈 여유를 다시 잰다 (책상판의 `pedestal_clearance` 를 그대로 쓴다).

## 아직 씬 전체 충돌은 못 본다

명세서 6.3 이 말하는 G1 은 배치된 씬 안에서 도는 것인데, 김학민의 A3 배치 스크립트가
아직 없다. 여기서는 로봇 + 지면만 올리고, 페데스탈은 책상판과 같은 **해석적 원통**
(Ø400 가정)으로 본다. A3 가 들어오면 이 스크립트에 씬 로드를 붙인다.

## 판정 두 개

  · **G1** — 11점 전부 IK 수렴(5 mm) + 수직 하강(하강정렬 ≥ 0.999) + 페데스탈 여유 > 0
  · **G5** — 11점의 IK 수렴 시간이 전부 100 ms 미만 (명세서 11)
"""
import argparse
import math
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                            # noqa: E402
import fk as F                                           # noqa: E402
import verify_reach as VR                                # 11점·페데스탈 판정 재사용  # noqa: E402
import verify_lula_ik as B1                              # Isaac 배선 재사용  # noqa: E402

# 손목 특이점 경보 문턱. j5 = 0 이 특이점이다 (descriptor default_q 주석 참조).
J5_SINGULAR_DEG = 10.0
# j4·j6 가 이 값을 넘으면 손목을 반바퀴 돌린 해로 본다.
WRIST_FLIP_DEG = 150.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--move", action="store_true",
                    help="점마다 실제로 지령하고 물리로 도달 확인 (느리다)")
    ap.add_argument("--ready-each", action="store_true",
                    help="점마다 READY 로 되돌린 뒤 푼다. 점끼리 독립된 판정이 된다. "
                         "기본은 직전 해에서 이어 푼다 (런타임과 같은 조건)")
    ap.add_argument("--settle", type=float, default=6.0)
    ap.add_argument("--gui", action="store_true",
                    help="창을 띄운다. 눈으로 확인할 때. 배선은 헤드리스와 동일하다")
    ap.add_argument("--dwell", type=float, default=None,
                    help="--move 시 점마다 머무는 시간 (s). --gui 면 기본 1.0, 아니면 0")
    ap.add_argument("--hold", action="store_true", help="끝난 뒤 창을 열어 둔다")
    ap.add_argument("--corridor", action="store_true",
                    help="트랙 C 수직 통로(박스+여유) 기둥 밖으로 팔이 얼마나 나가는지 잰다")
    ap.add_argument("--log", default=None,
                    help="결과를 남길 파일. 기본 results/g1_isaac_<zone>_<모드>.txt "
                         "— 연쇄판과 독립판이 서로 덮어쓰지 않게 이름이 갈린다")
    a = ap.parse_args()
    if a.dwell is None:
        a.dwell = 1.0 if a.gui else 0.0
    if a.gui and not a.move:
        # 창만 띄우고 IK 만 풀면 팔이 READY 에서 안 움직인다 — 볼 게 없다.
        print("NOTE  --gui 는 --move 와 같이 쓰는 것이 낫다. 팔이 11점을 실제로 돈다.")

    _paths.check()

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})
    # ★ Tee 는 SimulationApp 뒤에 건다 (verify_lula_ik.py 의 같은 주석 참조).
    #   연쇄(런타임 조건)와 독립(--ready-each)은 **비교하려고** 돌리는 것이므로
    #   같은 파일에 쓰면 안 된다. 모드를 파일명에 박는다.
    mode = "readyeach" if a.ready_each else "chain"
    tee = B1.start_log(a.log or _paths.ROOT / "results" /
                       ("g1_isaac_%s_%s.txt" % (a.zone, mode)))

    S = B1.bring_up(zone=a.zone, render=a.gui)
    cfg = S.cfg
    ped_mm = cfg["robot"]["pedestal_height_mm"]
    joints = F.load_joints()
    spheres = VR.load_spheres()

    print("ASSET   %s" % B1.ASSET)
    print("ZONE    %s   base world %s mm   pedestal %s mm   gripper %.0f mm"
          % (a.zone, list(np.round(S.base * 1000).astype(int)), ped_mm, S.grip * 1000))
    print("EE      %s (Lula frames %s)" % (S.ee, S.lula.get_all_frame_names()))
    # ⚠ 거리 기준이 verify_reach.py 와 다르다. 책상판은 **흡착면**까지의 거리를
    #   실효 reach(카탈로그 - 그리퍼)와 비교한다 — 그리퍼가 항상 베이스 바깥쪽을 향한다고
    #   보는 보수적 근사다. 수직 하강에서는 그리퍼가 아래(-Z)를 향하지 반경 방향이 아니다.
    #   여기서는 **tool0(=플랜지)** 까지의 거리를 카탈로그 reach 와 직접 비교한다.
    #   그래서 같은 점의 '여유' 숫자가 두 스크립트에서 다르게 나온다 (7번: 책상 +115,
    #   여기 +186). 판정 자체는 IK 수렴이 내리므로 이 열은 참고값이다.
    print("REACH   catalog %.0f mm (플랜지=tool0 까지). 거리 열도 tool0 기준이다."
          % (VR.CATALOG_REACH * 1000))
    print("        verify_reach.py 는 흡착면 기준 거리 vs 실효 %.0f mm — 숫자가 다르다."
          % ((VR.CATALOG_REACH - S.grip) * 1000))
    # ★ 웜스타트가 해를 가른다. ArticulationKinematicsSolver 는 **현재 관절각**을
    #   웜스타트로 쓰므로, 같은 목표점이라도 직전에 어디 있었느냐에 따라 다른 가지로 수렴한다.
    #   실측(2026-08-21, Script Editor): 9번 픽업점을
    #     READY(j5 -90)에서 출발 -> j5 음수 가지
    #     7번 자세(j5 +87)에서 출발 -> j5 +79.5, j4 -180.0, j6 -179.4  (손목이 통째로 돌았다)
    #   런타임은 코너 -> 픽업 -> 코너 로 직전 자세를 물고 가므로 **기본을 연쇄로 둔다.**
    #   `--ready-each` 는 점끼리 독립인 판정(도달 가능성 자체)을 볼 때 쓴다.
    print("WARM    %s" % ("점마다 READY 로 리셋 (점끼리 독립)"
                          if a.ready_each else "직전 해에서 이어 푼다 (런타임과 같은 조건)"))
    print("PED     원통 r=%.0f mm (가정, verify_reach.py 와 동일)\n" % (VR.PEDESTAL_R * 1000))

    hdr = (f"{'#':>2} {'지점':<28} {'월드 mm':>22} {'거리':>7} {'오차':>6} "
           f"{'하강':>7} {'페데스탈':>9} {'IK ms':>7}  판정")
    print(hdr)

    g1 = g5 = True
    rows = []
    B1.teleport(S, S.ready, steps=int(0.5 * S.hz))
    for n, (name, (x, y, z)) in enumerate(VR.targets(cfg, a.zone, ped_mm), 1):
        if a.ready_each:
            B1.teleport(S, S.ready)
        suction = np.array([x, y, z], dtype=float) / 1000.0
        target = suction + np.array([0.0, 0.0, S.grip])     # tool0 목표 = 흡착면 + 그리퍼
        dist = float(np.linalg.norm(target - S.base))

        ok, q, err, down, ms, ms_max = B1.solve(S, target)
        if ok and not a.ready_each:
            # 다음 점이 이 해를 웜스타트로 물게 한다. 이걸 안 하면 팔이 READY 에
            # 그대로 서 있어서 **두 모드가 똑같아진다** (실제로 그랬다).
            B1.teleport(S, q)
        if ok:
            clr, where = VR.pedestal_clearance(q, joints, spheres, ped_mm / 1000.0)
            reach_ok = err < B1.POS_TOL and down >= VR.DOWN_TOL
            ped_ok = clr > 0
            good = reach_ok and ped_ok
            if good:
                verdict = "✓  여유 %+.0fmm" % ((VR.CATALOG_REACH - dist) * 1000)
            elif not reach_ok:
                verdict = "✗ 자세  오차 %.1fmm 하강 %.3f" % (err * 1000, down)
            else:
                verdict = "✗ 페데스탈(%s)" % where
        else:
            clr, where, good = float("nan"), "", False
            verdict = "✗ IK 미수렴  필요 %.0f vs 카탈로그 %.0f" % (dist * 1000, VR.CATALOG_REACH * 1000)
        slow = ms_max >= B1.G5_SOLVE_MS
        if slow:
            verdict += "  ⚠G5 %.0fms" % ms_max
        g1 &= good
        g5 &= not slow
        print("%2d %-28s (%5d,%5d,%5d) %7.0f %6.1f %7.4f %8.0fmm %7.2f  %s"
              % (n, name, x, y, z, dist * 1000, err * 1000, down, clr * 1000, ms, verdict))
        rows.append((n, name, target, q, good, dist))

    # ---- 물리 확인 (옵션). IK 해가 나왔다고 팔이 거기 서 있는 건 아니다.
    if a.move:
        print("\n물리 확인 — 점마다 계단 입력으로 지령하고 수렴까지 기다린다")
        print("%2s %-28s %9s %9s %11s" % ("#", "지점", "수렴 s", "실제 mm", "판정"))
        for n, name, target, q, good, _ in rows:
            if q is None:
                continue
            B1.teleport(S, S.ready)
            t_conv, vmax, _ = B1.goto(S, q, secs=a.settle)
            B1.dwell(S, a.dwell)
            ee, _ = S.ik.compute_end_effector_pose()
            real = float(np.linalg.norm(ee - target))
            move_ok = t_conv is not None and real < B1.POS_TOL
            g1 &= move_ok
            print("%2d %-28s %9s %9.1f %11s"
                  % (n, name, "%.2f" % t_conv if t_conv else "미수렴", real * 1000,
                     "OK" if move_ok else "FAIL(v=%.1f)" % vmax))

    print("\nG1 (Isaac/Lula): %s"
          % ("통과 — 11점 모두 Lula 해가 수직 하강 · 페데스탈 간섭 없음" if g1
             else "실패 — 위 ✗ 항목. 대응 순서는 작업명세서 6.3"))
    print("G5 (수렴 100ms): %s" % ("통과" if g5 else "실패 — 제어 스택 재선택 (명세서 11)"))
    if not g1:
        print("  1) 더 짧은 그리퍼  2) 페데스탈 600  3) 팔레트 100mm 당김"
              "  4) max_stack 하향 (정정인 협의)")
    # ---- 수직 통로 기둥 대비 팔 이탈량 (--corridor).
    #
    # 왜 재나: 명세서 6.2 가 경로계획을 뺀 근거가 "트랙 C 의 수직 통로 검사가
    # 충돌회피를 대신한다" 인데, 그 검사(7.5 corridor_clear)가 보는 기둥은
    # **박스 단면 + gripper_clearance_mm(30) 사방** 이다. 그 기둥이 팔까지 담는지
    # 여기서 숫자로 본다. 담지 못하면, 기둥이 뚫려 있어도 팔은 보장 밖이다.
    #
    # 기둥은 **가장 큰 박스(5호)** 기준으로 잡는다 — 가장 넉넉한 조건이다.
    # 그래도 팔이 밖으로 나간다면 작은 박스에서는 더 나간다.
    if a.corridor:
        big = max(cfg["boxes"]["types"], key=lambda b: b["w_mm"] * b["d_mm"])
        clr = cfg["heightmap"]["gripper_clearance_mm"]
        hx = (big["w_mm"] / 2.0 + clr) / 1000.0
        hy = (big["d_mm"] / 2.0 + clr) / 1000.0
        print("\n수직 통로 기둥 — 박스 %s(%dx%d) + 여유 %d mm 사방  ->  단면 %.0f x %.0f mm"
              % (big["name"], big["w_mm"], big["d_mm"], clr, hx * 2000, hy * 2000))
        print("  기둥 밖으로 나간 팔 충돌구체를 센다 (link_3~tool0, 놓을 높이 위쪽만)")
        print("  %2s %-28s %8s %10s  %s" % ("#", "지점", "밖/전체", "최대이탈", "가장 먼 구체"))
        for n, name, target, q, _, _ in rows:
            if q is None:
                continue
            T = F.fk(list(q), joints)
            base_z = target[2] - S.grip          # 흡착면 높이 = 놓을 높이
            out, tot, worst, where = 0, 0, -1e9, ""
            for link in ("link_3", "link_4", "link_5", "link_6", "tool0"):
                R, pl = T[link]
                for c, r in spheres.get(link, []):
                    w = np.array(pl) + np.array(R) @ c
                    wz = S.base[2] + w[2]
                    if wz + r <= base_z:         # 놓을 평면 아래는 기둥과 무관하다
                        continue
                    tot += 1
                    dx = max(abs(S.base[0] + w[0] - target[0]) - hx, 0.0)
                    dy = max(abs(S.base[1] + w[1] - target[1]) - hy, 0.0)
                    d = math.hypot(dx, dy) - r   # >0 이면 구체가 통째로 기둥 밖
                    if d > 0:
                        out += 1
                    if d > worst:
                        worst, where = d, link
            print("  %2d %-28s %4d/%-3d %8.0fmm  %s"
                  % (n, name, out, tot, worst * 1000, where))
        print("  ※ '최대이탈' 은 기둥 옆면에서 팔이 가장 멀리 벗어난 거리다.")
        print("     통로 검사는 이 바깥을 보지 않는다 — 거기 뭐가 있는지는 아무도 모른다.")

    # ---- 손목 특이점 점검. descriptor 의 default_q 주석이 지목한 지점이다.
    #   j5 = 0 이 손목 특이점이고, `default_q` 의 j5 를 -90 으로 뒤집은 이유가
    #   "이 셀의 작업 구간 j5 가 전부 음수라 0 을 통과시키면 안 된다" 였다.
    #   그 전제가 11점에서도 성립하는지 여기서 숫자로 확인한다. 깨지면 default_q 를
    #   다시 정해야 한다 (RMPflow 로 갈아탈 때 특히 — Lula IK 는 반응형이 아니라 덜 민감하다).
    j5 = [(n, name, float(np.rad2deg(q[4]))) for n, name, _, q, _, _ in rows if q is not None]
    near = [(n, name, v) for n, name, v in j5 if abs(v) < J5_SINGULAR_DEG]
    pos = [(n, name, v) for n, name, v in j5 if v > 0]
    print("\n손목 j5   범위 %.1f ~ %.1f deg  (0 = 특이점, default_q 는 -90)"
          % (min(v for _, _, v in j5), max(v for _, _, v in j5)))
    if near:
        print("  ⚠ |j5| < %.0f deg 인 점 %d 개 — 손목 특이점 근처다:" % (J5_SINGULAR_DEG, len(near)))
        for n, name, v in near:
            print("      %2d %-28s j5 %+.1f deg" % (n, name, v))
    if pos:
        print("  ⚠ j5 가 **양수**인 점 %d 개 — default_q(-90)가 이 점들을 특이점 너머로 당긴다:"
              % len(pos))
        for n, name, v in pos:
            print("      %2d %-28s j5 %+.1f deg" % (n, name, v))
    if not near and not pos:
        print("  OK — 11점 모두 j5 음수 · 특이점에서 %.0f deg 이상 떨어져 있다"
              % J5_SINGULAR_DEG)

    # 손목 뒤집힘. j4·j6 가 ±180 근처면 팔이 손목을 통째로 반바퀴 돌려 자세를 만든 것이다.
    # 하강정렬은 1.0000 으로 멀쩡해 보여도, 그 자세로 **가는 동안** 손목이 크게 돈다.
    flip = [(n, name, float(np.rad2deg(q[3])), float(np.rad2deg(q[5])))
            for n, name, _, q, _, _ in rows if q is not None
            and (abs(np.rad2deg(q[3])) > WRIST_FLIP_DEG or abs(np.rad2deg(q[5])) > WRIST_FLIP_DEG)]
    if flip:
        print("\n  ⚠ 손목 뒤집힘 (j4 또는 j6 가 ±%.0f deg 초과) %d 개:" % (WRIST_FLIP_DEG, len(flip)))
        for n, name, j4, j6 in flip:
            print("      %2d %-28s j4 %+.1f  j6 %+.1f deg" % (n, name, j4, j6))

    print("\n※ 씬 전체 충돌은 아직 못 본다 — 김학민 A3 배치 스크립트 대기 (docstring)")
    sys.stdout.flush()
    if a.hold:
        B1.hold_open(S)
    print("\n결과: %s" % tee.path)
    tee.close()
    shutil.rmtree(S.tmpdir, ignore_errors=True)
    kit.close()
    return 0 if (g1 and g5) else 1


if __name__ == "__main__":
    sys.exit(main())
