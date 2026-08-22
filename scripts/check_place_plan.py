#!/usr/bin/env python3
"""배치 계획 프리플라이트 — 알고리즘이 준 `PlacePose` 를 **실행 전에** 검사한다.

정정인(C)의 `/algo/plan_placement` 는 팔레트를 기하로 채우지 **로봇 팔을 모른다.**
그래서 받은 좌표를 그대로 실행하면 안 된다. 여기서 먼저 숫자로 거른다.

    python3 scripts/check_place_plan.py                  # 내장 예시 계획 (3호 1층)
    python3 scripts/check_place_plan.py plan.json        # 실제 계획
    python3 scripts/check_place_plan.py --zone A plan.json

`plan.json` 형식 — `/algo/plan_placement` 응답을 그대로 늘어놓은 것:

    [{"box_type": "3호", "x_mm": 170, "y_mm": 125, "z_mm": 0, "yaw_deg": 0}, ...]

좌표는 **팔레트 좌표**다 (작업명세서 3.3): x,y = 박스 중심, **z = 박스 밑면**.

## 검사 6가지

  1. 박스가 팔레트(800 x 1200) 밖으로 나가지 않는가
  2. `max_stack_mm` 를 넘지 않는가
  3. 같은 층 이웃과 겹치지 않고, 간격이 **배치 오차의 2배**보다 넓은가
  4. 그리퍼 몸통(r 122.5)이 더 높은 이웃을 스치지 않는가
  5. **이송 중 들고 있는 박스 밑면이 적재 최상단 위로 뜨는가** ← 아래 참조
  6. 배치점에 **수직 하강 자세로** 도달하고 페데스탈을 안 파고드는가

## 5번이 왜 따로 있나 (2026-08-21 실측)

안전 고도를 **흡착면**에 걸면 안 된다. 이송 중 쌓인 것을 스치는 건 패드가 아니라
그 아래 매달린 **박스 밑면**이고, 박스 높이만큼 낮다. 구 저장소에서 이걸 빼먹고
150 mm 여유를 흡착면에 걸었더니, 두 번째 박스를 옮기다 **앞서 놓은 박스를 73 mm
파고들며 1,061 mm 밀어내 팔레트 밖으로 떨어뜨렸다.**
박스 하나짜리 시험으로는 절대 안 나온다 — 칠 것이 없기 때문이다.

    h_safe(흡착면) = 적재최상단 + 드는박스높이 + 여유
"""
import json
import math
import sys
from pathlib import Path

import _paths                      # 경로 해결 (scripts/_paths.py)
import fk as F
import verify_reach as VR          # solve / load_spheres / pedestal_clearance 재사용

# 그리퍼 실측 (구 저장소 usd/h2017_cell.usd). Ø50 패드 4개, 바깥 210 x 160.
GRIPPER_R = 0.1225
GRIPPER_LEN = 0.150

PLACE_ERR_MM = 7.0                 # 실측 배치 오차 6.8 mm
RMP_SAG_MM = 35.0                  # 지령 대비 처짐. 안전 여유에서 이만큼 빠진다
TRAVERSE_MIN_MM = 50.0             # 처짐을 빼고도 남아야 하는 이송 여유
OVERHANG_LIMIT_MM = 20.0           # 배치 오차까지 얹어 팔레트 밖으로 나가도 되는 한계


def pallet_place_to_world(place, h_mm, origin_mm):
    """PlacePose(밑면) -> 월드 박스 **중심** (m). 작업명세서 3.3 계약.

    ⚠ 정본은 `packing_common/frames.py` 다 (정정인 C0). 그게 빌드되면 그걸 import 할 것.
      여기 사본이 있는 이유는 이 스크립트가 **빌드 전에도 돌아야** 하기 때문이다.
      두 개가 어긋나면 박스가 팔레트에 반쯤 파묻힌다.
    """
    return ((place["x_mm"] + origin_mm[0]) / 1000.0,
            (place["y_mm"] + origin_mm[1]) / 1000.0,
            (place["z_mm"] + origin_mm[2] + h_mm // 2) / 1000.0)


def footprint(place, box):
    """yaw 를 반영한 (x 반폭, y 반폭) mm. allowed_yaw_deg 는 0/90 뿐이다."""
    w, d = box["w_mm"], box["d_mm"]
    if round(place.get("yaw_deg", 0)) % 180 == 90:
        w, d = d, w
    return w / 2.0, d / 2.0


def demo_plan(cfg):
    """예시 — 3호 박스를 1층에 채운다. 실제 계획이 없을 때 스크립트가 도는지 보는 용도."""
    box = next(b for b in cfg["boxes"]["types"] if b["name"] == "3호")
    pw, pd = cfg["pallet"]["width_mm"], cfg["pallet"]["depth_mm"]
    gap = int(2 * PLACE_ERR_MM) + 6            # 20 mm
    out = []
    nx = (pw + gap) // (box["w_mm"] + gap)
    ny = (pd + gap) // (box["d_mm"] + gap)
    for ix in range(nx):
        for iy in range(ny):
            out.append({"box_type": "3호", "yaw_deg": 0, "z_mm": 0,
                        "x_mm": int(box["w_mm"] / 2 + ix * (box["w_mm"] + gap)),
                        "y_mm": int(box["d_mm"] / 2 + iy * (box["d_mm"] + gap))})
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    zone = "B"
    if "--zone" in sys.argv:
        zone = sys.argv[sys.argv.index("--zone") + 1]

    _paths.check()
    cfg = _paths.system_config()
    types = {b["name"]: b for b in cfg["boxes"]["types"]}
    origin = cfg["zones"]["pallet_origin_world_mm"][zone]
    base = cfg["zones"]["robot_base_world_mm"][zone]
    ped_mm = cfg["robot"]["pedestal_height_mm"]
    tool = cfg["robot"]["gripper_length_mm"] / 1000.0
    safe_mm = cfg["robot"]["approach_offset_mm"]
    pw, pd = cfg["pallet"]["width_mm"], cfg["pallet"]["depth_mm"]
    max_stack = cfg["pallet"]["max_stack_mm"]

    if args:
        plan = json.loads(Path(args[0]).read_text())
        src = args[0]
    else:
        plan = demo_plan(cfg)
        src = "내장 예시 (3호 1층)"

    joints = F.load_joints()
    spheres = VR.load_spheres()

    print("zone %s | pallet %d x %d, max_stack %d | origin %s"
          % (zone, pw, pd, max_stack, origin))
    print("base %s | gripper %.0f mm | approach %d mm | plan: %s (%d boxes)"
          % (base, tool * 1000, safe_mm, src, len(plan)))
    print("place err %.0f mm | RMP sag %.0f mm | traverse min %.0f mm\n"
          % (PLACE_ERR_MM, RMP_SAG_MM, TRAVERSE_MIN_MM))
    print("%3s %-6s %13s %5s %8s %8s %8s %8s  %s"
          % ("#", "type", "center mm", "yaw", "onPal", "nbrGap", "gripGap", "travGap", "reach"))

    fails = []
    placed = []            # (x_mm, y_mm, hx, hy, top_mm)
    for n, pl in enumerate(plan, 1):
        box = types.get(pl.get("box_type", "3호"))
        if box is None:
            fails.append("#%d: 모르는 박스 종류 %r" % (n, pl.get("box_type")))
            continue
        h = box["h_mm"]
        hx, hy = footprint(pl, box)
        top = pl["z_mm"] + h

        # 1. 팔레트 안. **계획 자체**와 **배치 오차를 얹었을 때**를 나눠 본다.
        #    가장자리에 딱 붙여 놓는 건 정상이다 (적재율). 오차만큼 걸치는 것까지
        #    실패로 치면 모든 가장자리 박스가 걸려서 검사가 소음이 된다.
        on = min(pl["x_mm"] - hx, pl["y_mm"] - hy,
                 pw - (pl["x_mm"] + hx), pd - (pl["y_mm"] + hy))
        if on < 0:
            fails.append("#%d: 계획 자체가 팔레트 밖으로 %.0f mm (알고리즘 버그)" % (n, -on))
        elif on - PLACE_ERR_MM < -OVERHANG_LIMIT_MM:
            fails.append("#%d: 배치 오차를 얹으면 %.0f mm 걸친다 (한계 %.0f)"
                         % (n, PLACE_ERR_MM - on, OVERHANG_LIMIT_MM))

        # 2. 적재 상한
        if top > max_stack:
            fails.append("#%d: 적재 상단 %d > max_stack %d" % (n, top, max_stack))

        # 3. 같은 층 이웃
        nbr = 1e9
        for ox, oy, ohx, ohy, otop in placed:
            if abs(otop - top) > 1e-6:
                continue
            nbr = min(nbr, max(abs(pl["x_mm"] - ox) - (hx + ohx),
                               abs(pl["y_mm"] - oy) - (hy + ohy)))
        if nbr < 1e8 and nbr < 2 * PLACE_ERR_MM:
            fails.append("#%d: 이웃 간격 %.0f mm < 배치오차 2배 %.0f mm"
                         % (n, nbr, 2 * PLACE_ERR_MM))

        # 4. 그리퍼 몸통이 더 높은 이웃을 스치나
        grip = 1e9
        for ox, oy, ohx, ohy, otop in placed:
            if otop <= top + 1e-6:
                continue
            grip = min(grip, math.hypot(pl["x_mm"] - ox, pl["y_mm"] - oy)
                       - max(ohx, ohy) - GRIPPER_R * 1000.0)
        if grip < 1e8 and grip < PLACE_ERR_MM:
            fails.append("#%d: 그리퍼 몸통이 더 높은 이웃과 %.0f mm 뿐" % (n, grip))

        # 5. 이송 여유 — 들고 있는 **박스 밑면** 기준
        obstacle = max([0] + [t for *_, t in placed])
        carry_bottom = max(obstacle, top) + safe_mm     # = h_safe - h
        trav = carry_bottom - obstacle - RMP_SAG_MM
        if trav < TRAVERSE_MIN_MM:
            fails.append("#%d: 이송 여유 %.0f mm < %.0f mm" % (n, trav, TRAVERSE_MIN_MM))

        # 6. 도달 + 페데스탈
        wx, wy, wz = pallet_place_to_world(pl, h, origin)
        contact_z = wz + h / 2000.0                     # 흡착면 = 박스 상판
        local = (wx - base[0] / 1000.0, wy - base[1] / 1000.0, contact_z - base[2] / 1000.0)
        r = VR.solve(local, joints, tool, spheres=spheres, ped_h=ped_mm / 1000.0)
        err, down, q = r[1], r[2], r[3]
        clr, where = VR.pedestal_clearance(q, joints, spheres, ped_mm / 1000.0)
        ok = err < VR.POS_TOL and down >= VR.DOWN_TOL
        if not ok:
            fails.append("#%d: 배치점에 수직 하강으로 못 닿는다 (오차 %.1f mm, 하강 %.4f)"
                         % (n, err * 1000, down))
        if clr <= 0:
            fails.append("#%d: 배치 자세가 페데스탈을 %s 로 %.0f mm 파고든다"
                         % (n, where, -clr * 1000))

        print("%3d %-6s (%5d,%5d) %5d %8.0f %8s %8s %8.0f  %s"
              % (n, box["name"], pl["x_mm"], pl["y_mm"], pl.get("yaw_deg", 0), on,
                 "-" if nbr > 1e8 else "%.0f" % nbr,
                 "-" if grip > 1e8 else "%.0f" % grip,
                 trav, "OK" if ok and clr > 0 else "FAIL"))
        placed.append((pl["x_mm"], pl["y_mm"], hx, hy, top))

    print()
    if fails:
        for f in fails:
            print("FAIL: %s" % f)
        print("\nRESULT: %d problem(s) — 실행하지 말 것" % len(fails))
        return 1
    print("RESULT: plan OK (%d boxes)" % len(plan))
    print("※ 책상 계산이다. 옆 박스와의 실제 접촉은 Isaac 실행으로 확인한다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
