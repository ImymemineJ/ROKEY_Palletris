#!/usr/bin/env python3
"""★ **궤적 전체**를 쓸어 `link_2` 대 적재물 간섭을 잡는다 (Isaac/Lula).

    ~/isaac-sim-5.1.0/python.sh scripts/verify_transit_sweep_isaac.py
    ~/isaac-sim-5.1.0/python.sh scripts/verify_transit_sweep_isaac.py --gui --spawn --hold
    ~/isaac-sim-5.1.0/python.sh scripts/verify_transit_sweep_isaac.py --base-dy -100

## 왜 이 스크립트가 따로 필요한가 — 앞의 두 개가 못 잡는 것

`verify_link_clearance.py` / `..._isaac.py` 는 **웨이포인트 정지 자세**만 본다.
그래서 "근거리 기둥을 넘어 far 에 놓을 때" 같은 **끝점** 간섭은 잡지만,
**이동 중** 간섭은 원리적으로 못 잡는다.

실제로 데모를 깨뜨린 것은 이동 중이다 (`error1.mp4` 38~40 s, 김병직 2026-08-24):
**joint_1 이 회전하면서 link_2 가 로봇에서 가장 가까운 박스를 밀어버린다.**
link_2 는 joint_1 축에서 반경 방향으로 뻗어 있어서 회전하면 **수평 원호**를 쓴다.
끝점 두 개가 다 깨끗해도 그 사이에서 쓸고 지나간다.

## 경로를 추측하지 않는다 — 컨트롤러와 같은 호를 그린다

`robot_motion.py:309 _at()` 이 정본이다. 베이스 축 중심으로 **반경·z 는 선형,
각도는 짧은 쪽**으로 보간하고, 스텝마다 직전 해를 warm_start 로 IK 를 푼다.
스텝 수도 같은 식으로 정한다 (`robot_motion.py:348` — 관절 최대 변화 /
관절속도한계 x `joint_speed_scale`, x hz).

**warm_start 연쇄가 핵심이다.** 같은 호라도 직전 자세에 따라 다른 팔꿈치 가지로
가고, 그게 link_2 높이를 가른다. 스텝을 건너뛰면 다른 경로를 재게 된다.

## 웨이포인트는 cycle.py 를 그대로 따른다

`cycle.py:152-195` 의 순서다. 문제 구간은 4번(가로 이송)이다.

    1 픽 상공 상승        pick_air = pick_top + approach
    2 하강 · 흡착         pick_top + GRASP_GAP
    3 이송고도 상승        transit = max(pick_top, put_top, stack_top) + approach + h
    4 ★ 가로 이송         (pick_xy -> place_xy) @ transit     <- joint_1 회전
    5 배치 상공 하강        put_top + approach + h
    6 최종 하강           put_top + GRASP_GAP

## 무엇을 상대로 재나

그 시점까지 **이미 놓인 박스** 전부다 (지금 옮기는 박스는 뺀다 — 매달려 있다).
링크는 `link_2`~`tool0`. 매달린 박스 자체의 간섭은 `check_place_plan.py` 검사 5 다.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                            # noqa: E402
import fk as F                                           # noqa: E402
import verify_reach as VR                                # noqa: E402
import verify_lula_ik as B1                              # Isaac 배선 재사용  # noqa: E402
import verify_link_clearance as LC                       # 기하·간섭 계산 재사용  # noqa: E402

GRASP_GAP_MM = 2.0                 # cycle.py:34


def wrap(a):
    """[-pi, pi] 로 감는다. robot_motion._wrap 과 같은 규약 (각도는 짧은 쪽)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def arc_points(f0, f1, base_xy, steps, arc_min_r=0.15, direction="short"):
    """`robot_motion._at()` 과 같은 호를 samples 로 낸다. 흡착면 좌표 [m].

    `direction`:
      "short" — 컨트롤러 현행. `_wrap` 이 고르는 **짧은 쪽**.
      "long"  — 같은 끝점에 **반대로 돌아** 간다. 짧은 쪽이 적재물 위를 지날 때
                긴 쪽은 빈 공간(팔레트 반대편)을 지날 수 있다.

    ★ 긴 쪽은 joint_1 을 (360 − |dth|) 도 더 돌린다. 관절 한계를 넘으면 IK 가
      깨지므로 **한계 확인이 선행돼야 한다** — 이 함수는 기하만 낸다.
    """
    v0, v1 = np.asarray(f0)[:2] - base_xy, np.asarray(f1)[:2] - base_xy
    r0, r1 = float(np.linalg.norm(v0)), float(np.linalg.norm(v1))
    use_arc = r0 > arc_min_r and r1 > arc_min_r
    th0 = math.atan2(v0[1], v0[0])
    dth = wrap(math.atan2(v1[1], v1[0]) - th0)
    if direction == "long" and use_arc:
        # 같은 끝점, 반대 방향. 부호를 뒤집고 2pi 를 채운다.
        dth = dth - 2 * math.pi if dth > 0 else dth + 2 * math.pi
    out = []
    for k in range(steps + 1):
        u = k / steps
        if not use_arc:
            out.append(np.asarray(f0) + (np.asarray(f1) - np.asarray(f0)) * u)
            continue
        r = r0 + (r1 - r0) * u
        th = th0 + dth * u
        out.append(np.array([base_xy[0] + r * math.cos(th),
                             base_xy[1] + r * math.sin(th),
                             f0[2] + (f1[2] - f0[2]) * u]))
    return out, use_arc, math.degrees(dth)


def steps_for(S, q0, q1):
    """`robot_motion.py:348` 과 같은 스텝 수. 관절 최대변화 / (속도한계 x scale) x hz."""
    vlim = np.asarray(S.art.dof_properties["maxVelocity"][:6], dtype=float)
    scale = float(S.cfg["robot"]["joint_speed_scale"])
    dq = np.abs(np.asarray(q1) - np.asarray(q0))
    if not np.any(dq):
        return 1
    secs = float(np.max(dq / (vlim * scale)))
    return max(1, int(math.ceil(secs * S.hz)))


def sweep_segment(S, f0, f1, yaw0, yaw1, q_seed, obstacles, spheres, joints, paths,
                  steps=None, direction="short"):
    """한 구간을 호로 쓸며 매 스텝 IK -> 링크 간섭. (최악여유, 링크, l2최악, q_end, 스텝수).

    스텝마다 직전 해를 warm_start 로 쓴다 — 컨트롤러와 같은 조건이다.
    """
    # 스텝 수는 끝점 해로 정한다 (컨트롤러도 끝점 IK 로 페이싱을 먼저 잡는다)
    q_end, ok = solve_seeded(S, f1, yaw1, q_seed)
    if not ok:
        return None, "IK_FAIL", None, q_seed, 0, 0.0
    n = steps or steps_for(S, q_seed, q_end)
    # ★ 스텝 수는 **끝점 관절차**로 정해지므로 방향과 무관하다. 그런데 긴 쪽은 경로가
    #   3배까지 길어진다 — 같은 n 으로 쓸면 샘플이 성겨져 **간섭을 놓친다.**
    #   호 길이 비만큼 샘플을 늘린다 (컨트롤러 페이싱이 아니라 검사 해상도 문제다).
    if direction == "long":
        _, _, d_short = arc_points(f0, f1, S.base[:2], 1, direction="short")
        _, _, d_long = arc_points(f0, f1, S.base[:2], 1, direction="long")
        if abs(d_short) > 1e-6:
            n = int(math.ceil(n * abs(d_long) / abs(d_short)))
    pts, _is_arc, _dth = arc_points(f0, f1, S.base[:2], n, direction=direction)

    worst, where, l2 = None, "", None
    q = np.asarray(q_seed, dtype=float)
    for k, f in enumerate(pts):
        yaw = yaw0 + (yaw1 - yaw0) * (k / n)
        qk, ok = solve_seeded(S, f, yaw, q)
        if not ok:
            continue                        # 컨트롤러도 스텝 IK 실패는 건너뛴다
        q = qk
        if not obstacles:
            continue
        c, lk, c2 = clearance_at(S, q, obstacles, spheres, joints, paths)
        if worst is None or c < worst:
            worst, where = c, lk
        if l2 is None or c2 < l2:
            l2 = c2
    return worst, where, l2, q, n, _dth


def seed_bank(q, ready, which="multi"):
    """IK 가지를 갈아탈 후보 시드 목록. `[(이름, q), ...]`.

    ★ 왜 필요한가 — 6축 팔은 한 점을 최대 8가지 자세(어깨 좌/우 · 팔꿈치 상/하 ·
      손목 정/역)로 잡는다. 지금 컨트롤러는 **직전 해를 warm_start 로 물고** 가므로
      가지 하나에 갇힌다. 그 가지가 나쁘면 이송 내내 못 빠져나온다.

      실측 (results/branch_probe.txt, 5호 3단 #12 최종 자세):
        READY / 팔꿈치반대 / 어깨낮게 / 어깨높게 -> 전부 같은 해, 여유 +77.8 mm
        **j1 반대편  -> j1 239.4 · j2 -18.7 · j3 -126.4, 여유 +106.3 mm**
      즉 실제로 갈리는 것은 **어깨 반대편(j1 flip)** 하나뿐이었다. 나머지 시드는
      같은 해로 수렴한다 — 시드를 많이 넣는다고 가지가 많아지지 않는다.

    `which`:
      "keep"  — 현행. 직전 해만.
      "multi" — 직전 해 + READY + 어깨 반대편 + 손목 뒤집기.
    """
    q = np.asarray(q, dtype=float)
    out = [("keep", q)]
    if which == "keep":
        return out
    out.append(("ready", np.asarray(ready, dtype=float)))
    # 어깨 반대편 — j1 을 반 바퀴 돌리고 어깨·팔꿈치 부호를 뒤집는다.
    #   같은 점을 **팔레트 반대쪽에서** 잡는 자세다. 측정에서 유일하게 이득이 났다.
    flip = q.copy()
    flip[0] = wrap(q[0] + math.pi)
    flip[1], flip[2] = -q[1], -q[2]
    out.append(("j1flip", flip))
    # 손목 뒤집기 — j4/j6 반 바퀴, j5 부호 반전. 팔은 그대로고 손목만 다른 가지다.
    wr = q.copy()
    wr[3] = wrap(q[3] + math.pi)
    wr[4] = -q[4]
    wr[5] = wrap(q[5] + math.pi)
    out.append(("wflip", wr))
    return out


def solve_seeded(S, face, yaw_deg, seed):
    """흡착면 목표를 warm_start 로 푼다. `robot_motion._ik()` 와 같은 규약."""
    from isaacsim.core.utils.rotations import euler_angles_to_quat
    tgt = np.asarray(face, dtype=float) + np.array([0.0, 0.0, S.grip])
    quat = euler_angles_to_quat(np.array([0.0, np.pi, math.radians(yaw_deg)]))
    q, ok = S.lula.compute_inverse_kinematics(
        frame_name=S.ee, target_position=tgt, target_orientation=quat,
        warm_start=np.asarray(seed, dtype=float))
    return (np.asarray(q, dtype=float) if ok else np.asarray(seed)), bool(ok)


def clearance_at(S, q, obstacles, spheres, joints, paths):
    """자세 q 에서 링크 대 적재물 최소여유. (전체최소, 링크, link_2최소) [m].

    링크 포즈는 **Lula FK** 로 낸다 — 씬 프림을 쓰려면 관절을 실제로 옮겨야 해서
    한 구간에 수백 스텝을 도는 이 검사에는 못 쓴다. `..._isaac.py` 가 Lula FK 와
    씬 프림이 소수점까지 같음을 이미 확인했다 (results/link2_isaac_B.txt).
    """
    T = F.fk(list(q), joints)
    worst, where, l2 = None, "", float("inf")
    for link in LC.LINKS:
        R, p = np.asarray(T[link][0]), S.base + np.asarray(T[link][1])
        for c, r in spheres.get(link, []):
            w = p + R @ c
            for lo, hi in obstacles:
                d = LC.sphere_aabb_clearance(w, r, lo, hi)
                if link == "link_2" and d < l2:
                    l2 = d
                if worst is None or d < worst:
                    worst, where = d, link
    return worst, where, l2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("plan", nargs="?", help="계획 JSON. 기본 results/stack15.json")
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--base-dy", type=float, default=0.0,
                    help="로봇을 월드 Y 로 미는 양 [mm]. ★ 씬의 로봇을 실제로 옮긴다")
    ap.add_argument("--from-n", type=int, default=1, help="이 번째 박스부터 본다")
    ap.add_argument("--only-transit", action="store_true",
                    help="4번(가로 이송) 구간만 본다. 빠르다")
    ap.add_argument("--steps", type=int, default=None,
                    help="구간당 스텝 수 고정. 기본은 컨트롤러와 같은 식으로 계산")
    ap.add_argument("--tool", type=float, default=None, metavar="M",
                    help="플랜지~흡착면 [m]. 기본은 system.yaml 의 gripper_length_mm. "
                         "★ 익스텐션 효과를 보는 노브다 — 툴이 길면 팔뚝(link_4)이 "
                         "이웃 기둥 위로 뜬다. 대신 도달범위를 1:1 로 깎는다")
    ap.add_argument("--seeds", default="keep", choices=["keep", "multi"],
                    help="가로 이송 구간의 IK 시드. keep=현행(직전 해만) · "
                         "multi=여러 가지를 다 풀어 여유가 가장 큰 해를 고른다. "
                         "★ 이게 '팔을 유연하게' 의 실제 레버다 — 관절 한계나 자세 공차가 "
                         "아니라 **IK 가지 선택**이 묶여 있었다")
    ap.add_argument("--reseed", default="none", choices=["none", "ready", "auto"],
                    help="가로 이송 시작 전에 IK 시드를 바꾼다. none=현행(직전 해를 물고 간다) · "
                         "ready=READY 자세로 재시드 · auto=재시드 한 것과 안 한 것 중 여유가 큰 쪽. "
                         "★ 팔꿈치 가지가 나쁜 쪽에 갇히는 것을 깨는 노브다 — "
                         "고도·호방향으로 안 풀리는 박스(#12)가 이걸로 풀리는지 본다")
    ap.add_argument("--arc-dir", default="short", choices=["short", "long", "auto"],
                    help="가로 이송 호의 회전 방향. short=현행(짧은 쪽) · "
                         "long=반대로 돌아감 · auto=양쪽 다 계산해 여유가 큰 쪽을 고른다. "
                         "auto 가 곧 '닿을 것 같으면 호를 뒤집는다' 로직의 측정판이다")
    ap.add_argument("--extra-lift", type=float, default=0.0,
                    help="이송 고도를 이만큼 더 올린다 [mm]. 팔꿈치(link_3)가 적재물 "
                         "위로 뜨는지 본다. cycle.py 의 transit 계산에 더할 후보값이다")
    ap.add_argument("--spawn", action="store_true", help="적재물을 큐브로 그린다")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--hold", action="store_true")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    _paths.check()
    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})
    # ★ 파일명에 lift 도 넣는다. 안 넣으면 --extra-lift 여러 값이 전부 같은
    #   dy 파일에 덮어써져 마지막 것만 남는다 (터미널로 쫓아야 했던 원인).
    lift_tag = "_lift%+.0f" % a.extra_lift if a.extra_lift else ""
    lift_tag += "_arc%s" % a.arc_dir if a.arc_dir != "short" else ""
    lift_tag += "_seed%s" % a.reseed if a.reseed != "none" else ""
    lift_tag += "_%s" % a.seeds if a.seeds != "keep" else ""
    lift_tag += "_tool%.0f" % (a.tool * 1000) if a.tool is not None else ""
    tee = B1.start_log(a.log or _paths.ROOT / "results" /
                       ("transit_sweep_%s_dy%+.0f%s.txt"
                        % (a.zone, a.base_dy, lift_tag)))

    asset = B1.ASSET_SUCTION if B1.ASSET_SUCTION.exists() else None
    S = B1.bring_up(zone=a.zone, render=a.gui, asset=asset, base_dy_mm=a.base_dy)
    if a.tool is not None:
        S.grip = float(a.tool)          # 익스텐션 감도 — 씬의 그리퍼 메시는 그대로다
    cfg = S.cfg
    ped_mm = cfg["robot"]["pedestal_height_mm"]
    joints = F.load_joints()
    spheres = VR.load_spheres()
    origin_mm = cfg["zones"]["pallet_origin_world_mm"][a.zone]
    approach = cfg["robot"]["approach_offset_mm"] / 1000.0
    belt_z = cfg["conveyor"]["belt_top_z_mm"] / 1000.0
    stopper_x = cfg["conveyor"]["stopper_x_mm"] / 1000.0

    path = a.plan or str(_paths.ROOT / "results/stack15.json")
    plan = json.loads(Path(path).read_text())

    from isaacsim.core.utils.rotations import quat_to_rot_matrix     # noqa: F401
    link_paths = None            # 스텝 검사는 Lula FK 를 쓴다 (docstring 참조)

    print("ASSET   %s" % (asset or B1.ASSET))
    print("ZONE    %s   base world %s mm   pedestal %s mm   gripper %.0f mm"
          % (a.zone, list(np.round(S.base * 1000).astype(int)), ped_mm, S.grip * 1000))
    print("이격    팔레트 원점 y %d − 로봇 y %.0f = %.0f mm  (base-dy %+.0f)"
          % (origin_mm[1], S.base[1] * 1000, origin_mm[1] - S.base[1] * 1000, a.base_dy))
    print("계획    %s (%d 박스)  approach %.0f mm  hz %d  speed_scale %.2f"
          % (path, len(plan), approach * 1000, S.hz, cfg["robot"]["joint_speed_scale"]))
    print("경로    robot_motion._at() 과 같은 호 · 스텝마다 warm_start IK")
    if a.extra_lift:
        print("이송    고도 +%.0f mm 추가 (--extra-lift)" % a.extra_lift)
    print("호방향  %s%s" % (a.arc_dir,
                          "  (구간 4 에서 양방향 계산 후 여유 큰 쪽 선택)"
                          if a.arc_dir == "auto" else ""))
    print()

    seg_names = {3: "3 이송고도상승", 4: "4 ★가로이송", 5: "5 배치상공", 6: "6 최종하강"}
    hdr = "%3s %-4s %-13s %9s %8s %-7s %6s  판정" % (
        "#", "종류", "구간", "link_2", "전체최소", "링크", "스텝")
    print(hdr)
    print("-" * len(hdr))

    B1.teleport(S, S.ready)
    q = np.asarray(S.ready, dtype=float)
    stack_top_m = (origin_mm[2]) / 1000.0
    worst_all = None
    for n, place in enumerate(plan, 1):
        w, d, h = LC.box_dims(cfg, place["box_type"])
        obstacles = [LC.box_aabb_world(p, cfg, origin_mm) for p in plan[:n - 1]]
        if a.spawn and n == a.from_n:
            for i, ob in enumerate(obstacles):
                spawn_box(ob, i)
        # ★ `--from-n` 은 **출력만** 줄인다. 건너뛰면 안 된다 —
        #   warm_start 연쇄가 끊겨 다른 팔꿈치 가지로 가고 답이 바뀐다.
        #   실측: 같은 #11 이 --from-n 11 에서는 long +175.3, --from-n 5 에서는
        #   short +309.2 / long +172.5 로 **승자가 뒤집혔다**. 연쇄를 이어야 한다.
        #   여기서는 장애물을 빈 리스트로 줘서 간섭 계산(비싼 부분)만 건너뛴다.
        report = n >= a.from_n
        if not report:
            obstacles = []

        pick_x = stopper_x - w / 2000.0
        pick_top = belt_z + h / 1000.0
        pick_yaw = 0.0
        cx, cy, cz = LC.pallet_place_to_world_m(place, h, origin_mm)
        put_top = cz + h / 2000.0
        put_yaw = float(place.get("yaw_deg", 0.0))
        transit = (max(pick_top, put_top, stack_top_m) + approach + h / 1000.0
                   + a.extra_lift / 1000.0)          # --extra-lift: 호 전체를 올린다

        wps = [
            (3, [pick_x, 0.0, pick_top + approach], [pick_x, 0.0, transit], pick_yaw, pick_yaw),
            (4, [pick_x, 0.0, transit], [cx, cy, transit], pick_yaw, put_yaw),
            (5, [cx, cy, transit], [cx, cy, put_top + approach + h / 1000.0], put_yaw, put_yaw),
            (6, [cx, cy, put_top + approach + h / 1000.0],
                [cx, cy, put_top + GRASP_GAP_MM / 1000.0], put_yaw, put_yaw),
        ]
        if a.only_transit:
            wps = [s for s in wps if s[0] == 4]
        # 4번 구간 시작 자세를 만들려면 3번을 먼저 풀어야 한다 (warm_start 연쇄)
        q, _ = solve_seeded(S, wps[0][1], wps[0][3], q)

        for sid, f0, f1, y0, y1 in wps:
            # ★ 방향 선택은 **구간 4(가로 이송)** 에서만 의미가 있다. 나머지 구간은
            #   시작·끝의 반경/각도가 거의 같아 호가 사실상 직선이라 뒤집을 게 없다.
            dirs = [a.arc_dir]
            if a.arc_dir == "auto" and sid == 4:
                dirs = ["short", "long"]
            elif a.arc_dir != "short" and sid != 4:
                dirs = ["short"]
            # ★ 시드 후보. Lula 는 warm_start 로 팔꿈치 가지를 정하므로, 직전 해가
            #   나쁜 가지면 이송 내내 못 빠져나온다 (실측: #12 가 lift 0/200/310 과
            #   short/long 전부에서 -21.2 로 불변. 책상 IK 는 같은 자세에서 +195.8 을
            #   찾는다 = 좋은 가지가 존재하는데 Lula 가 못 간다).
            # ★ 가지는 **구간 시작에서 한 번 고르고 끝까지 유지**한다. 중간에 바꾸면
            #   관절이 180도 튄다 (실제 로그에 |dq| 180.0 deg 가 있었다).
            # ★ 시드 선택은 **모든 구간**에서 의미가 있다 (호 방향과 달리).
            #   실측: 구간 4 만 열었더니 병목이 구간 3 으로 옮겨갔을 뿐이었다
            #   (#11 구간4 +32.9 -> +115.0 인데 전체 최악은 +32.9 그대로).
            seeds = seed_bank(q, S.ready, a.seeds)
            if a.reseed in ("ready", "auto") and sid == 4:
                rd = ("ready", np.asarray(S.ready, dtype=float))
                seeds = [rd] if a.reseed == "ready" else [("keep", q), rd]
            best, tried = None, []
            for sname, sq in seeds:
              for dirn in dirs:
                r = sweep_segment(S, f0, f1, y0, y1, sq, obstacles, spheres,
                                  joints, link_paths, a.steps, direction=dirn)
                score = -1e9 if r[1] == "IK_FAIL" else (
                    1e9 if r[0] is None else min(r[0], r[2]))
                # ★ 진 쪽 값도 남긴다. 이긴 쪽만 보면 "왜 그쪽을 골랐는지" 를 못 본다 —
                #   실제로 각도 겹침 예측이 실측과 어긋난 것을 이걸로 잡았다.
                tried.append((dirn if sname == "keep" else dirn + "/" + sname,
                              score, r[5]))
                if best is None or score > best[0]:
                    best = (score, r, dirn if sname == "keep" else dirn + "/" + sname)
            c, lk, l2, q, ns, dth = best[1]
            chosen = best[2]
            if not report:
                continue                     # 연쇄만 잇고 보고는 안 한다
            if lk == "IK_FAIL":
                print("%3d %-4s %-13s %9s %8s %-7s %6s  ✗ 끝점 IK 실패"
                      % (n, place["box_type"], seg_names[sid], "-", "-", "", "-"))
                continue
            if c is None:
                print("%3d %-4s %-13s %9s %8s %-7s %6d  (칠 것이 없다)"
                      % (n, place["box_type"], seg_names[sid], "-", "-", "", ns))
                continue
            mark = "✓" if min(c, l2) > 0 else "!! 닿는다"
            tag = ""
            if sid == 4:
                # dth 는 arc_points 가 이미 **도**로 준다 (math.degrees 재적용 금지)
                if len(tried) > 1:
                    parts = []
                    for dn, sc, dd in tried:
                        v = "IK✗" if sc <= -1e8 else "%+.0f" % (sc * 1000)
                        parts.append("%s%+.0f°:%s%s"
                                     % (dn[0], dd, v, "←" if dn == chosen else ""))
                    tag = " [%s]" % " ".join(parts)
                else:
                    tag = " [%s %+.0f°]" % (chosen, dth)
            print("%3d %-4s %-13s %+9.1f %+8.1f %-7s %6d  %s%s"
                  % (n, place["box_type"], seg_names[sid], l2 * 1000, c * 1000, lk,
                     ns, mark, tag))
            if worst_all is None or min(c, l2) < worst_all[0]:
                worst_all = (min(c, l2), n, seg_names[sid], lk)
        stack_top_m = max(stack_top_m, put_top)

    if worst_all:
        print("\n최악: #%d %s — %s 여유 %+.1f mm"
              % (worst_all[1], worst_all[2], worst_all[3], worst_all[0] * 1000))
        print("판정: %s" % ("통과" if worst_all[0] > 0 else
                           "실패 — 이동 중 팔이 적재물을 친다"))
    print("\n※ 스텝 검사는 Lula FK 로 링크를 놓는다. 씬 프림과 일치함은"
          "\n  verify_link_clearance_isaac.py 가 확인했다 (results/link2_isaac_B.txt).")
    if a.hold:
        B1.hold_open(S)
    # ★ Tee 를 kit.close() **전에** 닫는다 (Tee.close docstring). 안 닫으면 파일이
    #   빈 채로 남는다 — kit.close() 가 프로세스를 끊어 버퍼가 그대로 날아간다.
    #   실제로 첫 실행에서 표가 통째로 사라졌다.
    tee.close()
    kit.close()
    return 0 if (worst_all is None or worst_all[0] > 0) else 1


def spawn_box(ob, i):
    """적재물을 큐브로 그린다. 숫자에는 영향이 없다 — 눈으로 볼 용도다."""
    from isaacsim.core.api.objects import VisualCuboid
    lo, hi = ob
    VisualCuboid(prim_path="/World/Stack_%02d" % i, name="stack_%02d" % i,
                 position=(lo + hi) / 2.0, scale=hi - lo,
                 color=np.array([0.85, 0.35, 0.35]))


if __name__ == "__main__":
    sys.exit(main())
