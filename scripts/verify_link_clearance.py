#!/usr/bin/env python3
"""팔 링크 대 **적재물** 간섭 측정 — 특히 `link_2` (results/issues/18). 책상 계산판.

## 왜 이게 따로 필요한가

`verify_reach.pedestal_clearance()` 는 상대가 **페데스탈 원통**이고, 보는 링크도
`link_3~tool0` 뿐이다. `link_1`·`link_2` 는 "받침대 바로 위에 있어 항상 겹친다"는
이유로 아예 제외돼 있다. 그 판단은 상대가 받침대일 때만 맞다.

실제로 다층 적재를 깨뜨리는 접촉은 **link_2 가 이미 쌓인 근거리 기둥 위를 쓸고
지나가는 것**이다 (issues/18). 그런데 지금 그 판정은 `demo_stack15.py:331` 의

    lim = ped_mm - 150      # 책상 스윕에서 얻은 어림값

한 줄뿐이다. 어림값이라 "이격을 10 mm 밀면 나아지나"" 같은 질문에 답을 못 한다.
이 스크립트가 그 자리를 메운다 — `link_2` 충돌구체 6개(0~0.84 m)를 FK 로 월드에
놓고, 그 시점까지 **이미 쌓인 박스들의 AABB** 와의 최소거리를 직접 잰다.

    python3 scripts/verify_link_clearance.py results/stack15.json
    python3 scripts/verify_link_clearance.py results/stack15.json --base-dy -10
    python3 scripts/verify_link_clearance.py results/stack15.json --pedestal 1050
    python3 scripts/verify_link_clearance.py results/stack15.json --sweep-dy -60,-40,-20,0,20

`--base-dy` 는 로봇 베이스를 **월드 Y** 로 미는 양(mm). 음수 = 팔레트 반대쪽으로
물러남 = 장변 이격이 300 에서 그만큼 늘어난다. `system.yaml` 을 고치지 않고
감도만 본다 (공용 파일이라 §10·§11).

## 무엇을 재나

`cycle.py:184-195` 의 배치 구간을 그대로 따라간다. 팔은 배치 지점 XY 위에서
`transit` 부터 `put_top + GRASP_GAP_MM` 까지 **수직 하강**하므로, 그 z 구간을
샘플링해 각 점마다 수직 하강 자세 IK 를 풀고 링크별 최소여유를 낸다.

## 한계 — 이 숫자를 절대값으로 믿지 말 것

6축 팔은 같은 점을 여러 자세(팔꿈치 가지)로 잡는다. 여기서는 도달하는 자세들 중
**적재물 여유가 가장 큰 것**을 고른다. 즉 "제어기가 최선을 다했을 때" 값이다.
실기 Lula 해는 가지가 달라 더 나쁠 수 있다 — `demo_stack15.py:248` 이 같은 경고를
남겼다 ("책상 +70 도 실기에선 닿는다").

따라서 이 도구의 용도는 두 가지다:
  1. **여기서 음수면 실기에서도 확실히 닿는다** (필요조건 판정)
  2. **파라미터 A 대 B 의 상대 비교** — 같은 편향이 양쪽에 걸리므로 부호는 믿을 수 있다
"""
import argparse
import json
import math
import sys

import numpy as np
from scipy.optimize import minimize

import _paths                      # 경로 해결 (scripts/_paths.py)
import fk as F
import verify_reach as VR          # load_spheres / pedestal_clearance / 상수 재사용

GRASP_GAP_MM = 2.0                 # cycle.py:34 와 같아야 한다
# link_1 은 베이스 직상이라 팔레트에 닿을 일이 없고, 페데스탈과는 항상 겹쳐 보인다.
# 적재물 상대로는 link_2 부터가 의미 있다.
LINKS = ("link_2", "link_3", "link_4", "link_5", "link_6", "tool0")


# ─────────────────────────────────────────────────────────────
#  적재물 → 월드 AABB
# ─────────────────────────────────────────────────────────────
def box_dims(cfg, name):
    for b in cfg["boxes"]["types"]:
        if b["name"] == name:
            return b["w_mm"], b["d_mm"], b["h_mm"]
    sys.exit(f"system.yaml 에 없는 박스 종류: {name}")


def pallet_place_to_world_m(place, h_mm, origin_mm):
    """PlacePose(x/y=중심 · z=밑면) -> 월드 박스 **중심** (x, y, z) [m].

    ⚠ 정본은 `packing_common/frames.py` 다. 여기 사본이 있는 이유는 이 스크립트가
      **빌드 전에도 돌아야** 하기 때문이다 — `check_place_plan.py:52` 와 같은 사정,
      같은 규약이다. 둘이 어긋나면 박스가 팔레트에 반쯤 파묻힌다.
    """
    return ((place["x_mm"] + origin_mm[0]) / 1000.0,
            (place["y_mm"] + origin_mm[1]) / 1000.0,
            (place["z_mm"] + origin_mm[2] + h_mm // 2) / 1000.0)


def box_aabb_world(place, cfg, origin_mm):
    """PlacePose(팔레트 좌표, x/y=중심 · z=밑면) -> 월드 AABB (lo, hi) [m].

    yaw 는 0/90 뿐이다 (`heightmap.allowed_yaw_deg`). 90 이면 w·d 를 맞바꾼다 —
    규약은 `check_place_plan.footprint()` 와 같다.
    """
    w, d, h = box_dims(cfg, place["box_type"])
    if round(place.get("yaw_deg", 0)) % 180 == 90:
        w, d = d, w
    cx = place["x_mm"] + origin_mm[0]
    cy = place["y_mm"] + origin_mm[1]
    z0 = place["z_mm"] + origin_mm[2]
    lo = np.array([cx - w / 2.0, cy - d / 2.0, z0]) / 1000.0
    hi = np.array([cx + w / 2.0, cy + d / 2.0, z0 + h]) / 1000.0
    return lo, hi


def sphere_aabb_clearance(center, radius, lo, hi):
    """구와 AABB 사이 최소거리 [m]. 음수면 파고든 것이다.

    바깥이면 축별 초과분의 노름, 안쪽이면 가장 얕게 파고든 면까지의 거리다
    (`verify_reach.pedestal_clearance` 와 같은 규약).
    """
    over = np.maximum(lo - center, center - hi)      # >0 바깥, <0 안쪽
    if np.any(over > 0):
        d = float(np.linalg.norm(np.maximum(over, 0.0)))
    else:
        d = float(np.max(over))                      # 전부 음수 -> 얕은 면
    return d - radius


def stack_clearance(q, joints, spheres, base_m, obstacles):
    """지금 자세에서 팔 링크와 적재물 사이 최소여유 [m].

    반환 `(여유, 링크, 박스번호, per_link)`. `per_link` 은 링크별 최소여유 dict 다 —
    전체 최소값은 거의 항상 `tool0`(그리퍼 몸통 r 122.5, 놓는 자리 이웃과 붙는다)이
    잡아서, 그것만 보면 **`link_2` 가 영원히 안 보인다.** 링크별로 따로 낸다.
    """
    T = F.fk(list(q), joints)
    worst, where, which = None, "", -1
    per_link = {}
    for link in LINKS:
        R, p = T[link]
        Rm, pm = np.array(R), np.array(p)
        for c, r in spheres.get(link, []):
            w = base_m + pm + Rm @ c                 # 로봇 로컬 -> 월드
            for i, (lo, hi) in enumerate(obstacles):
                d = sphere_aabb_clearance(w, r, lo, hi)
                if link not in per_link or d < per_link[link]:
                    per_link[link] = d
                if worst is None or d < worst:
                    worst, where, which = d, link, i
    return worst, where, which, per_link


# ─────────────────────────────────────────────────────────────
#  IK — 도달하는 자세들 중 **적재물 여유가 가장 큰 것**
# ─────────────────────────────────────────────────────────────
def solve_max_clearance(target_local, joints, tool_len, spheres, ped_h,
                        base_m, obstacles, restarts=80, seed=0):
    """수직 하강 자세로 target_local(로봇 로컬 m)에 도달하는 해를 찾는다.

    `verify_reach.solve` 와 골격은 같지만 **선택 기준이 다르다.** 저쪽은 페데스탈
    여유를 최대화한다. 여기서는 적재물 여유를 최대화하되 페데스탈은 뚫지 않게 건다
    — 묻는 질문이 "적재물을 피할 자세가 있느냐" 이므로.
    """
    lo = np.array([joints[f"joint_{i}"]["lower"] for i in range(1, 7)])
    hi = np.array([joints[f"joint_{i}"]["upper"] for i in range(1, 7)])
    lo = np.maximum(lo, -math.pi)
    hi = np.minimum(hi, math.pi)
    t = np.array(target_local)

    def cost(q):
        T = F.fk(list(q), joints)
        R, p = T["tool0"]
        tip = np.array(p) + np.array([R[0][2], R[1][2], R[2][2]]) * tool_len
        e = np.linalg.norm(tip - t)
        return e * e + 0.05 * (1.0 - (-R[2][2])) ** 2

    rng = np.random.default_rng(seed)
    best = None            # (여유, 링크, 박스, q)  — 도달 + 페데스탈 통과한 것 중 최대
    nearest = None         # 전부 도달 실패했을 때 보고용
    for k in range(restarts):
        q0 = rng.uniform(lo, hi) if k else np.array([0.0, 0.6981, 2.0944, 0.0, -1.2217, 0.0])
        r = minimize(cost, q0, method="L-BFGS-B", bounds=list(zip(lo, hi)),
                     options={"maxiter": 400})
        T = F.fk(list(r.x), joints)
        R, p = T["tool0"]
        tip = np.array(p) + np.array([R[0][2], R[1][2], R[2][2]]) * tool_len
        err = float(np.linalg.norm(tip - t))
        down = -R[2][2]
        if err >= VR.POS_TOL or down < VR.DOWN_TOL:
            if nearest is None or err < nearest[0]:
                nearest = (err, r.x)
            continue
        if VR.pedestal_clearance(r.x, joints, spheres, ped_h)[0] <= 0:
            continue                                  # 받침대를 뚫는 가지는 버린다
        clr, link, which, per_link = stack_clearance(
            r.x, joints, spheres, base_m, obstacles)
        if best is None or clr > best[0]:
            best = (clr, link, which, r.x, per_link)
    return best, nearest


# ─────────────────────────────────────────────────────────────
#  계획 재생 — 순서대로 놓아 가며 매 배치의 여유를 낸다
# ─────────────────────────────────────────────────────────────
def _ik(cx, cy, z, base_m, joints, tool_m, spheres, ped_mm, obstacles, restarts, seed):
    local = (cx - base_m[0], cy - base_m[1], z - base_m[2])
    return solve_max_clearance(local, joints, tool_m, spheres, ped_mm / 1000.0,
                               base_m, obstacles, restarts=restarts, seed=seed)


def geometry(cfg, zone, base_dy_mm, ped_mm):
    """(팔레트 원점 mm, 로봇 베이스 m). base_dy 는 월드 Y 로 미는 양(mm)."""
    origin_mm = cfg["zones"]["pallet_origin_world_mm"][zone]
    bx, by, _ = cfg["zones"]["robot_base_world_mm"][zone]
    return origin_mm, np.array([bx, by + base_dy_mm, ped_mm]) / 1000.0


def run(plan, cfg, zone, base_dy_mm, ped_mm, tool_m, samples, restarts, verbose):
    """계획을 순서대로 재생한다. (최악전체, 최악link_2, rows) 반환."""
    origin_mm, base_m = geometry(cfg, zone, base_dy_mm, ped_mm)
    joints = F.load_joints()
    spheres = VR.load_spheres()
    approach_m = cfg["robot"]["approach_offset_mm"] / 1000.0

    rows, worst, worst_l2 = [], None, None
    for n, place in enumerate(plan, 1):
        obstacles = [box_aabb_world(p, cfg, origin_mm) for p in plan[:n - 1]]
        if not obstacles:
            if verbose:
                print("%3d  %-4s (%4d,%4d) z%4d   첫 박스 — 칠 것이 없다"
                      % (n, place["box_type"], place["x_mm"], place["y_mm"], place["z_mm"]))
            continue

        _, _, h = box_dims(cfg, place["box_type"])
        put_top = (place["z_mm"] + origin_mm[2] + h) / 1000.0
        cx = (place["x_mm"] + origin_mm[0]) / 1000.0
        cy = (place["y_mm"] + origin_mm[1]) / 1000.0
        stack_top = max(hi[2] for _, hi in obstacles)
        z_hi = max(put_top, stack_top) + approach_m + h / 1000.0
        z_lo = put_top + GRASP_GAP_MM / 1000.0

        row = None
        for si in range(samples):
            z = z_hi + (z_lo - z_hi) * (si / max(samples - 1, 1))
            best, _ = _ik(cx, cy, z, base_m, joints, tool_m, spheres, ped_mm,
                          obstacles, restarts, n * 100 + si)
            if best is None:
                row = (float("-inf"), "IK_FAIL", -1, float("-inf"))
                break
            clr, link, which, _q, per_link = best
            l2 = per_link.get("link_2", float("inf"))
            if row is None or clr < row[0]:
                row = (clr, link, which, l2)
            elif l2 < row[3]:
                row = (row[0], row[1], row[2], l2)

        rows.append((n, place, row))
        if worst is None or row[0] < worst[0]:
            worst = (row[0], n, row[1])
        if worst_l2 is None or row[3] < worst_l2[0]:
            worst_l2 = (row[3], n)
        if verbose:
            _print_row(n, place, row, plan)
    return worst, worst_l2, rows


def _print_row(n, place, row, plan):
    clr, link, which, l2 = row
    if link == "IK_FAIL":
        print("%3d  %-4s (%4d,%4d) z%4d      ---        IK 실패 (수직 하강 자세 없음)"
              % (n, place["box_type"], place["x_mm"], place["y_mm"], place["z_mm"]))
        return
    tgt = plan[which]
    print("%3d  %-4s (%4d,%4d) z%4d  %+8.1f %-7s %+9.1f   vs #%d (%4d,%4d) z%-4d%s"
          % (n, place["box_type"], place["x_mm"], place["y_mm"], place["z_mm"],
             clr * 1000, link, l2 * 1000, which + 1,
             tgt["x_mm"], tgt["y_mm"], tgt["z_mm"],
             "" if min(clr, l2) > 0 else "  !! 닿는다"))


# ─────────────────────────────────────────────────────────────
#  probe — `lim = ped - 150` 을 대체할 숫자를 직접 쓸어 낸다
# ─────────────────────────────────────────────────────────────
def near_column_aabb(cfg, zone, origin_mm, near_h_mm, box_type):
    """근거리 기둥의 월드 AABB (lo, hi) [m]. 높이만 파라미터다.

    팔레트 y 최소(=로봇 쪽) 변에 붙여 세우고, x 는 팔레트 중앙이다. 목표점과 x 가
    같아야 팔이 기둥 **정면**을 넘어가므로 이게 최악 정렬이다.

    ★ Isaac 판(`verify_link_clearance_isaac.py`)이 이 함수를 import 한다.
      두 스크립트가 서로 다른 기둥을 보고 다른 판정을 내는 일을 막는다.
    """
    pw = cfg["pallet"]["width_mm"]
    w, d, _h = box_dims(cfg, box_type)
    return (np.array([origin_mm[0] + pw / 2 - w / 2, origin_mm[1], origin_mm[2]]) / 1000.0,
            np.array([origin_mm[0] + pw / 2 + w / 2, origin_mm[1] + d,
                      origin_mm[2] + near_h_mm]) / 1000.0)


def far_target_suction(cfg, zone, origin_mm, target_z_mm, box_type):
    """far 쪽 목표에서 **흡착면**이 있어야 할 월드 좌표 (x, y, z) [m].

    먼 쪽(y 최대) 변, 밑면 팔레트 z = `target_z_mm`. z=0 이면 팔이 가장 펴진다.
    `cycle.py:195` 의 최종 하강점과 같은 규약이다 (박스 상면 + GRASP_GAP_MM).
    """
    pw, pd = cfg["pallet"]["width_mm"], cfg["pallet"]["depth_mm"]
    _w, d, h = box_dims(cfg, box_type)
    return ((origin_mm[0] + pw / 2) / 1000.0,
            (origin_mm[1] + pd - d / 2) / 1000.0,
            (origin_mm[2] + target_z_mm + h + GRASP_GAP_MM) / 1000.0)


def probe(cfg, zone, base_dy_mm, ped_mm, tool_m, near_h_mm, restarts, box_type,
          target_z_mm=0.0):
    """근거리 기둥 높이 `near_h_mm` 를 넘어 far 쪽에 놓을 때 link_2 여유 [mm].

    issues/18 의 기하를 그대로 만든다 — 로봇 쪽(팔레트 y 최소)에 기둥이 서 있고,
    그 너머 먼 쪽에 놓으러 간다. `target_z_mm` 은 놓는 박스 **밑면**의 팔레트 z 다.

    0(far 1층)이면 팔이 가장 펴지고 link_2 가 가장 눕는다. 하지만 실제로 데모를
    깨뜨리는 것은 **far 꼭대기**로 넘어가는 경우다 — 그때는 근거리도 이미 높다
    (`demo_stack15.py:240` "미뤄둔 far 꼭대기 … 근거리 680 은 link_2 가 넘는다").
    그래서 두 경우를 다 볼 수 있게 z 를 파라미터로 뺐다.

    하강 도중은 팔이 더 접혀 있어 여유가 더 크므로 최종점만 본다.
    """
    origin_mm, base_m = geometry(cfg, zone, base_dy_mm, ped_mm)
    joints = F.load_joints()
    spheres = VR.load_spheres()
    near = near_column_aabb(cfg, zone, origin_mm, near_h_mm, box_type)
    cx, cy, z = far_target_suction(cfg, zone, origin_mm, target_z_mm, box_type)

    best, nearest = _ik(cx, cy, z, base_m, joints, tool_m, spheres, ped_mm,
                        [near], restarts, int(near_h_mm + target_z_mm))
    if best is None:
        return None, nearest
    _clr, _link, _which, _q, per_link = best
    return per_link.get("link_2", float("inf")) * 1000.0, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("plan", nargs="?", help="계획 JSON (팔레트 좌표). 없으면 results/stack15.json")
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--base-dy", type=float, default=0.0,
                    help="로봇 베이스를 월드 Y 로 미는 양 [mm]. 음수 = 팔레트 반대쪽")
    ap.add_argument("--pedestal", type=float, default=None, help="페데스탈 높이 [mm]")
    ap.add_argument("--tool", type=float, default=None, help="플랜지~흡착면 [m]")
    ap.add_argument("--samples", type=int, default=4, help="하강 구간 z 샘플 수")
    ap.add_argument("--restarts", type=int, default=80, help="IK 재시작 횟수")
    ap.add_argument("--sweep-dy", default=None, help="쉼표로 구분한 base-dy 목록 [mm]")
    ap.add_argument("--sweep-ped", default=None, help="쉼표로 구분한 페데스탈 목록 [mm]")
    ap.add_argument("--probe", action="store_true",
                    help="계획 대신 근거리 기둥 높이를 쓸어 link_2 한계를 낸다")
    ap.add_argument("--probe-h", default="200,400,600,700,800,900,1000,1100,1200,1300",
                    help="probe 에서 쓸어볼 근거리 기둥 높이 [mm]")
    ap.add_argument("--probe-box", default="3호", help="probe 에서 놓을 박스 종류")
    ap.add_argument("--probe-z", default="0",
                    help="probe 목표의 밑면 팔레트 z [mm]. 쉼표로 여러 개 (far 꼭대기 검사)")
    a = ap.parse_args()

    _paths.check()
    cfg = _paths.system_config()
    tool_m = a.tool if a.tool is not None else cfg["robot"]["gripper_length_mm"] / 1000.0
    ped0 = a.pedestal if a.pedestal is not None else cfg["robot"]["pedestal_height_mm"]
    oy = cfg["zones"]["pallet_origin_world_mm"][a.zone][1]
    by0 = cfg["zones"]["robot_base_world_mm"][a.zone][1]
    dys = [float(v) for v in a.sweep_dy.split(",")] if a.sweep_dy else [a.base_dy]
    peds = [float(v) for v in a.sweep_ped.split(",")] if a.sweep_ped else [ped0]

    print(f"zone {a.zone} | 팔레트 원점 y {oy} · 로봇 베이스 y {by0} "
          f"-> 장변 이격 {oy - by0} mm | 그리퍼 {tool_m*1000:.0f} mm")

    # ── probe 모드 ───────────────────────────────────────────
    if a.probe:
        hs = [float(v) for v in a.probe_h.split(",")]
        tzs = [float(v) for v in a.probe_z.split(",")]
        print(f"probe: 근거리 기둥(가로축, mm)을 넘어 far 쪽 {a.probe_box} 에 놓을 때 "
              f"link_2 여유 [mm]")
        print(f"       tgt_z = 놓는 박스 밑면의 팔레트 z. "
              f"기존 어림값 lim = ped - 150 (demo_stack15.py:331)\n")
        print("%9s %6s %6s | %s" % ("base_dy", "ped", "tgt_z",
                                    "  ".join("%6.0f" % v for v in hs)))
        print("-" * (26 + 8 * len(hs)))
        for ped in peds:
            for dy in dys:
                for tz in tzs:
                    cells, limit = [], None
                    for hv in hs:
                        v, _ = probe(cfg, a.zone, dy, ped, tool_m, hv, a.restarts,
                                     a.probe_box, target_z_mm=tz)
                        if v is None:
                            cells.append("  IK✗")
                            continue
                        cells.append("%+6.0f" % v)
                        if v > 0:
                            limit = hv
                    print("%+9.0f %6.0f %6.0f | %s   한계 %s (어림 %.0f)"
                          % (dy, ped, tz, "  ".join(cells),
                             "%.0f" % limit if limit is not None else "없음", ped - 150))
        print("\n※ 각 칸은 그 높이의 근거리 기둥에 대한 link_2 여유다. 음수 = 쓸고 지나간다.")
        print("  '한계' = 여유가 양수인 가장 높은 기둥. 절대값보다 증감을 보라.")
        return 0

    path = a.plan or str(_paths.ROOT / "results/stack15.json")
    plan = json.loads(open(path).read())
    print(f"계획 {path} ({len(plan)} 박스) · approach {cfg['robot']['approach_offset_mm']} mm "
          f"· 하강 샘플 {a.samples} · IK 재시작 {a.restarts}\n")

    # ── 스윕 요약 ────────────────────────────────────────────
    if a.sweep_dy or a.sweep_ped:
        print("%9s %8s  %13s  %13s" % ("base_dy", "ped", "최악(전체)", "최악(link_2)"))
        for ped in peds:
            for dy in dys:
                worst, worst_l2, _ = run(plan, cfg, a.zone, dy, ped, tool_m,
                                         a.samples, a.restarts, verbose=False)
                print("%+9.0f %8.0f  %+8.1f #%-3d  %+8.1f #%-3d%s"
                      % (dy, ped, worst[0] * 1000, worst[1],
                         worst_l2[0] * 1000, worst_l2[1],
                         "  !! 닿는다" if min(worst[0], worst_l2[0]) <= 0 else ""))
        print("\n※ 절대값보다 **부호와 증감**을 보라 (docstring 한계 참조).")
        return 0

    # ── 상세 ─────────────────────────────────────────────────
    print("  #  종류   중심 mm   밑면z    최소여유 링크      link_2   상대")
    worst, worst_l2, _ = run(plan, cfg, a.zone, a.base_dy, ped0, tool_m,
                             a.samples, a.restarts, verbose=True)
    print(f"\n최악(전체): #{worst[1]} {worst[2]} {worst[0]*1000:+.1f} mm")
    print(f"최악(link_2): #{worst_l2[1]} {worst_l2[0]*1000:+.1f} mm")
    if min(worst[0], worst_l2[0]) <= 0:
        print("판정: 실패 — 팔이 적재물을 친다. 감도는 --sweep-dy / --sweep-ped / --probe")
        return 1
    print("판정: 통과 (책상 기준). 실기 Lula 가지는 더 나쁠 수 있다 — issues/18.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
