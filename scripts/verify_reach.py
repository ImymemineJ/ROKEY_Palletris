#!/usr/bin/env python3
"""G1 — 도달범위 11점 검증 (작업명세서 6.3). **책상 계산판.**

Isaac 없이, URDF 의 실제 링크 오프셋과 관절 한계를 넣고 수치 IK 를 돌린다.
흡착이므로 **툴 z축 수직 하강**을 제약으로 건다.

    python3 scripts/verify_reach.py            # B구역
    python3 scripts/verify_reach.py --zone A
    python3 scripts/verify_reach.py --pedestal 600   # 실패 시 대응안 감도

**모든 숫자는 `src/packing_bringup/config/system.yaml` 에서 읽는다** (철칙 3).
여기에 상수를 적지 않는다 — 페데스탈을 600 으로 올리기로 하면 yaml 만 고친다.

## 이게 G1 의 전부는 아니다

작업명세서 6.3 이 요구하는 G1 은 **Isaac 안에서 Lula IK** 로 도는 것이다
(`verify_reach_isaac.py`, Day 2, 김학민의 A3 배치 스크립트 이후).
이 스크립트는 그걸 기다리지 않고 **오늘 돌릴 수 있는** 사전 판정이다.
둘의 차이:

| | 이 스크립트 (책상) | Isaac 판 (Day 2) |
|---|---|---|
| IK | scipy 수치해 (재시작 60회) | Lula `compute_inverse_kinematics` |
| 충돌 | 페데스탈 원통 vs 충돌구체 22개 | 씬 전체 물리 |
| 실행 조건 | 지금 바로 | 씬 + 배치 스크립트 필요 |

책상에서 떨어지면 Isaac 에서도 떨어진다. 반대는 보장 못 한다.

## 페데스탈 형상은 가정이다

`system.yaml` 에 높이(500)만 있고 반지름이 없다. 구 저장소 실측치 **Ø400** 을 쓴다.
김학민이 확정하면 `PEDESTAL_R` 을 고칠 것 — 팔이 페데스탈 옆으로 내려가는
자세에서 이 값이 판정을 가른다 (구 셀에서 link_5 가 -17 mm 로 물렸던 적이 있다).
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import minimize

import _paths                      # 경로 해결 (scripts/_paths.py)
import fk as F

POS_TOL = 0.005               # 5 mm
DOWN_TOL = 0.999              # 툴 z축 아래 정렬도

# 페데스탈 원통 반지름 (m). ★ 가정값 — 위 docstring 참조.
PEDESTAL_R = 0.20

# H2017 카탈로그 reach (플랜지까지, m). 실효 reach = 이 값 - 그리퍼 길이.
CATALOG_REACH = 1.700


def targets(cfg, zone, pedestal_mm):
    """작업명세서 6.3 의 11점을 system.yaml 에서 유도한다. 월드 mm."""
    ox, oy, oz = cfg["zones"]["pallet_origin_world_mm"][zone]
    pw = cfg["pallet"]["width_mm"]        # Px, 월드 +X
    pd = cfg["pallet"]["depth_mm"]        # Py, 월드 +Y
    top = cfg["pallet"]["max_stack_mm"]
    app = cfg["robot"]["approach_offset_mm"]
    belt = cfg["conveyor"]["belt_top_z_mm"]
    stop = cfg["conveyor"]["stopper_x_mm"]

    out = []
    for label, dz in (("corner bottom", 0), ("corner top+app", top + app)):
        for px in (0, pw):
            for py in (0, pd):
                out.append((f"{label} P({px},{py})", (ox + px, oy + py, oz + dz)))
    # 픽업 접근점 — x 는 스토퍼가 아니라 **박스 중심**(스토퍼 - w/2), z 는 상판 + approach.
    for b in cfg["boxes"]["types"]:
        out.append((f"pickup {b['name']} ({b['w_mm']}x{b['d_mm']}x{b['h_mm']})",
                    (stop - b["w_mm"] // 2, 0, belt + b["h_mm"] + app)))
    return out


def solve(target_local, joints, tool_len, restarts=60, seed=0, spheres=None, ped_h=0.5):
    """target_local: 로봇 로컬 좌표(m). 툴 z축이 -Z 를 향하도록 푼다.

    6축 팔은 한 점을 여러 자세로 잡는다. 그중에는 받침대를 뚫고 들어가는 자세도 있다.
    `spheres` 를 주면 **도달하는 자세들 중 받침대 여유가 가장 큰 것**을 고른다.
    이걸 안 하면 무작위로 뽑힌 자세가 받침대를 뚫는 것을 보고 "도달 불가"로 오판한다.
    """
    lo = np.array([joints[f"joint_{i}"]["lower"] for i in range(1, 7)])
    hi = np.array([joints[f"joint_{i}"]["upper"] for i in range(1, 7)])
    lo = np.maximum(lo, -math.pi)   # 1회전 이상 도는 해는 의미가 없다
    hi = np.minimum(hi, math.pi)
    t = np.array(target_local)

    def cost(q):
        T = F.fk(list(q), joints)
        R, p = T["tool0"]
        tip = np.array(p) + np.array([R[0][2], R[1][2], R[2][2]]) * tool_len
        e = np.linalg.norm(tip - t)
        down = -R[2][2]
        return e * e + 0.05 * (1.0 - down) ** 2

    rng = np.random.default_rng(seed)
    best = None          # 도달 못 한 것 중 가장 가까운 것 (전부 실패했을 때 보고용)
    best_ok = None       # 도달한 것 중 받침대 여유가 가장 큰 것
    for k in range(restarts):
        q0 = rng.uniform(lo, hi) if k else np.array(
            [0.0, 0.6981, 2.0944, 0.0, -1.2217, 0.0]
        )
        r = minimize(cost, q0, method="L-BFGS-B", bounds=list(zip(lo, hi)),
                     options={"maxiter": 400})
        T = F.fk(list(r.x), joints)
        R, p = T["tool0"]
        tip = np.array(p) + np.array([R[0][2], R[1][2], R[2][2]]) * tool_len
        err = float(np.linalg.norm(tip - t))
        down = -R[2][2]

        if err < POS_TOL and down >= DOWN_TOL:
            clr = pedestal_clearance(r.x, joints, spheres, ped_h)[0] if spheres else 0.0
            if best_ok is None or clr > best_ok[0]:
                best_ok = (clr, err, down, r.x)
            if spheres is None or clr > 0:
                break                       # 받침대까지 피했으면 더 볼 것 없다
        else:
            score = err + (0 if down >= DOWN_TOL else 1.0)
            if best is None or score < best[0]:
                best = (score, err, down, r.x)

    if best_ok is not None:
        _, err, down, q = best_ok
        return (err, err, down, q)
    return best


def load_spheres():
    """descriptor 의 충돌 구체를 링크별로 읽는다."""
    path = _paths.DESCRIPTOR
    data = yaml.safe_load(path.read_text())
    out = {}
    for entry in data["collision_spheres"]:
        for link, sph in entry.items():
            out[link] = [(np.array(s["center"], dtype=float), float(s["radius"])) for s in sph]
    return out


def pedestal_clearance(q, joints, spheres, ped_h=0.5):
    """페데스탈 원통과 팔 충돌 구체 사이의 최소 여유(m). 음수면 파고든 것이다.

    `ped_h` 는 페데스탈 높이(m). 로봇 로컬 z 로 -ped_h ~ 0 구간을 차지한다.

    받침대는 로봇 자신의 z축에 붙어 있으므로 로봇 로컬 좌표에서 바로 계산한다.
    base_link · link_1 은 받침대 바로 위에 있어 항상 겹치므로 제외한다
    (rmpflow 의 body_collision_controllers 도 link_3 부터 본다).
    """
    T = F.fk(list(q), joints)
    worst = None
    where = ""
    for link in ("link_3", "link_4", "link_5", "link_6", "tool0"):
        R, p = T[link]
        for c, r in spheres.get(link, []):
            w = np.array(p) + np.array(R) @ c
            # 원통 표면까지의 거리 (옆면 · 윗면 중 가까운 쪽)
            radial = math.hypot(w[0], w[1]) - PEDESTAL_R
            above = w[2] - 0.0
            below = -ped_h - w[2]
            if radial >= 0 or above >= 0 or below >= 0:
                d = math.sqrt(sum(max(v, 0.0) ** 2 for v in (radial, above, below)))
            else:
                d = max(radial, above, below)   # 안쪽 — 가장 얕게 파고든 면
            d -= r
            if worst is None or d < worst:
                worst, where = d, link
    return worst, where

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--tool", type=float, default=None,
                    help="플랜지~흡착면 (m). 기본은 system.yaml 의 gripper_length_mm")
    ap.add_argument("--pedestal", type=float, default=None,
                    help="페데스탈 높이 (mm). 기본은 system.yaml. 실패 시 600 감도 확인용")
    a = ap.parse_args()

    _paths.check()
    cfg = _paths.system_config()
    tool = a.tool if a.tool is not None else cfg["robot"]["gripper_length_mm"] / 1000.0
    ped_mm = a.pedestal if a.pedestal is not None else cfg["robot"]["pedestal_height_mm"]
    bx, by, bz = cfg["zones"]["robot_base_world_mm"][a.zone]
    if a.pedestal is not None:
        bz = ped_mm                     # 페데스탈을 바꾸면 베이스 z 가 따라간다

    joints = F.load_joints()
    spheres = load_spheres()
    eff = CATALOG_REACH - tool

    print(f"zone {a.zone} | base ({bx}, {by}, {bz}) mm | pedestal {ped_mm} mm "
          f"| gripper {tool*1000:.0f} mm")
    print(f"catalog reach {CATALOG_REACH*1000:.0f} - gripper -> effective {eff*1000:.0f} mm")
    print(f"pedestal cylinder r={PEDESTAL_R*1000:.0f} mm (assumed), z -{ped_mm}~0 local\n")
    print(f"{'#':>2} {'지점':<28} {'월드 mm':>22} {'거리':>7} {'오차':>6} "
          f"{'하강':>7} {'페데스탈':>9}  판정")

    allok = True
    worst = None
    for n, (name, (x, y, z)) in enumerate(targets(cfg, a.zone, ped_mm), 1):
        local = ((x - bx) / 1000.0, (y - by) / 1000.0, (z - bz) / 1000.0)
        dist = math.sqrt(sum(v * v for v in local))
        score, err, down, q = solve(local, joints, tool, spheres=spheres,
                                    ped_h=ped_mm / 1000.0)
        clr, where = pedestal_clearance(q, joints, spheres, ped_mm / 1000.0)
        reach_ok = err < POS_TOL and down >= DOWN_TOL
        ped_ok = clr > 0
        ok = reach_ok and ped_ok
        allok &= ok
        if worst is None or dist > worst[0]:
            worst = (dist, name, ok)
        if ok:
            verdict = f"✓  여유 {(eff - dist)*1000:+.0f}mm"
        elif not reach_ok:
            verdict = f"✗ 미달  필요 {dist*1000:.0f} > 실효 {eff*1000:.0f}"
        else:
            verdict = f"✗ 페데스탈({where})"
        print(f"{n:2d} {name:<28} ({x:5d},{y:5d},{z:5d}) {dist*1000:7.0f} "
              f"{err*1000:6.1f} {down:7.4f} {clr*1000:8.0f}mm  {verdict}")

    print(f"\n최원거리: {worst[1]} = {worst[0]*1000:.0f} mm "
          f"(실효 reach {eff*1000:.0f}, 여유 {(eff - worst[0])*1000:+.0f} mm)")
    print("G1 (책상): " + ("통과 — 11점 모두 수직 하강 자세로 도달, 페데스탈 간섭 없음"
                          if allok else "실패 — 위 ✗ 항목. 대응 순서는 작업명세서 6.3"))
    if not allok:
        print("  1) 더 짧은 그리퍼  2) 페데스탈 600 (--pedestal 600)")
        print("  3) 팔레트 100mm 당김  4) max_stack 하향 (정정인 협의)")
    print("※ 책상 계산이다. Isaac Lula 판정은 verify_reach_isaac.py (Day 2).")
    return 0 if allok else 1


if __name__ == "__main__":
    sys.exit(main())
