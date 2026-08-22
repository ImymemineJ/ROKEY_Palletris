#!/usr/bin/env python3
"""시뮬레이션 A — 픽앤플레이스 **사이클 예행**. 박스는 안 띄우고 팔만 돈다.

    ~/isaac-sim-5.1.0/python.sh scripts/verify_cycle.py                 # 연쇄 웜스타트 (현행)
    ~/isaac-sim-5.1.0/python.sh scripts/verify_cycle.py --warm ready    # 고정 웜스타트 (처방)
    ~/isaac-sim-5.1.0/python.sh scripts/verify_cycle.py --gui --hold        # 영상 촬영용
    ~/isaac-sim-5.1.0/python.sh scripts/verify_cycle.py --transit line --yaw fixed  # 옛 방식(비교)
    ~/isaac-sim-5.1.0/python.sh scripts/verify_cycle.py --gui --boxes 3
    ~/isaac-sim-5.1.0/python.sh scripts/verify_cycle.py --plan plan.json

`verify_reach_isaac.py` 는 **점 11개를 따로따로** 본다. 실제 로봇은 점을 따로 가지 않는다 —
픽업과 배치를 번갈아 오가고, 적재가 자라고, 직전 자세가 다음 IK 의 웜스타트가 된다.
여기서 그 순서를 그대로 돌린다.

## 무엇을 재나 — 게이팅 표가 못 보는 것 넷

1. **j5 감김.** 연쇄 웜스타트에서 손목이 한 방향으로 감긴다. 11점 순회에서 이미 +287° 를 봤고
   URDF 한계는 ±360° 다. 15박스 사이클에서 한계에 닿는가?
2. **웜스타트 고정이 실제로 듣는가.** `--warm ready` 와 `--warm chain` 을 같은 계획으로 비교한다.
3. **이송 중 팔이 적재물 위로 얼마나 뜨는가.** ★ 이게 핵심이다.
   트랙 C 의 수직 통로 검사(명세서 7.5)는 **배치점 위 기둥 하나**만 본다.
   픽업에서 배치점으로 넘어가는 **이송 구간**은 그 정의상 대상이 아니다.
   `check_place_plan.py` 가 이걸 정적으로 검사하지만(검사 5), 실제로 팔이 어디를 지나는지는
   돌려봐야 안다. 여기서는 팔레트 위를 지나는 **모든 충돌구체의 최저점**을 매 스텝 샘플링해
   그때그때의 적재 최상단과 비교한다.
4. **사이클 타임.** `cycle_time_target_s` 대비.

## ★ 보간은 **태스크 공간**이다 (2026-08-21 수정)

처음에는 웨이포인트에서만 IK 를 풀고 그 사이를 **관절공간 선형 보간**으로 이었다. 틀렸다.
관절 보간은 툴 자세를 **양 끝점에서만** 맞추고 중간은 보존하지 않는다. 손목이 크게 도는
구간에서 툴이 통째로 기울었다 돌아온다 — 흡착으로 박스를 매단 채로는 그대로 쏟는 동작이다.
(GUI 로 눈에 띄어서 잡혔다. 숫자로는 안 보이던 결함이다.)

지금은 **흡착면 위치를 직선으로 쪼개고 매 스텝 IK 를 푼다.** 자세는 매 스텝 수직 하향으로
지령되므로 경로 내내 유지되고, 스텝 간격이 촘촘해 해가 다른 가지로 튀지도 않는다.
`--interp joint` 로 옛 방식을 재현할 수 있다 — 비교용으로 남겼다.

## 이 스크립트가 하지 않는 것

- 박스를 스폰하지 않는다. 흡착도 안 한다 (B3 이후). 팔 궤적만 본다.
- 배치 계획은 **더미**다 — 정정인(C)의 알고리즘이 아니라, 적재가 자라는 순서를 만들려고
  3호 박스를 격자로 층층이 쌓은 것뿐이다. `--plan` 으로 실제 계획을 넣을 수 있다.
- 박스 자체의 기울기는 못 잰다 (박스가 없다). **툴 기울기**로 대신 본다 — 흡착이 강체로
  붙는다면 둘이 같다.
"""
import argparse
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                            # noqa: E402
import fk as F                                           # noqa: E402
import check_place_plan as CP                            # demo_plan · 좌표 계약 재사용  # noqa: E402
import verify_reach as VR                                # 충돌구체  # noqa: E402
import verify_lula_ik as B1                              # Isaac 배선  # noqa: E402

ARM_LINKS = ("link_3", "link_4", "link_5", "link_6", "tool0")

# 박스를 매달고 있는 구간 — 이 이름의 웨이포인트로 **가는 동안** 흡착 중이다.
CARRY_WPS = ("carry_up", "carry_over", "place_app", "place")
# 베이스 반대편으로 넘어가는 두 구간. 여기만 호로 돌고 yaw 도 같이 돈다.
ARC_WPS = ("pick_app", "carry_over")


def _wrap(rad):
    """각을 (-pi, pi] 로 접는다 — 항상 짧은 쪽으로 돈다."""
    return math.atan2(math.sin(rad), math.cos(rad))


def _near(target_deg, near_deg, period=180.0):
    """직사각 박스는 yaw 와 yaw+180 이 같은 자리다. near 에 가장 가까운 대표각."""
    return target_deg + round((near_deg - target_deg) / period) * period


def _quat_of(R):
    """회전행렬 -> (w, x, y, z). Isaac 의 set_world_pose 규약이다.

    직접 짠다 — `rot_matrix_to_quat` 의 모듈 경로가 Isaac 판마다 달라서
    임포트로 묶어 두면 판이 바뀔 때 스크립트가 통째로 죽는다.
    """
    R = np.asarray(R, float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        w = math.sqrt(1.0 + tr) * 0.5
        k = 0.25 / w
        return np.array([w, (R[2, 1] - R[1, 2]) * k,
                         (R[0, 2] - R[2, 0]) * k, (R[1, 0] - R[0, 1]) * k])
    i = int(np.argmax(np.diag(R)))
    j, k2 = (i + 1) % 3, (i + 2) % 3
    t = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k2, k2])
    q = np.zeros(4)
    q[i + 1] = 0.5 * t
    t = 0.5 / t
    q[0] = (R[k2, j] - R[j, k2]) * t
    q[j + 1] = (R[j, i] + R[i, j]) * t
    q[k2 + 1] = (R[k2, i] + R[i, k2]) * t
    return q


_EULER = None


def _quat_down(yaw_rad):
    """툴 z 는 아래, yaw 만 준 쿼터니언. **Isaac 기동 뒤에만** 부를 것."""
    global _EULER
    if _EULER is None:
        from isaacsim.core.utils.rotations import euler_angles_to_quat
        _EULER = euler_angles_to_quat
    return _EULER(np.array([0.0, math.pi, float(yaw_rad)]))

# 박스를 매단 채 허용할 툴 기울기. ★ 가정값 — 흡착 유지 각도의 정본이 없다.
# 구 저장소에서 손목이 뒤집혔을 때 툴 기울기가 3.9 -> 91.5 deg 였다 (README 8).
# 실제 한계는 패드 전단력에서 나와야 하고, 정해지면 system.yaml 로 올릴 것.
TILT_TOL_DEG = 5.0


def mixed_plan(cfg, n):
    """**3·4·5호를 섞어** 층층이 쌓는 더미 계획. 선반(shelf) 방식 first-fit.

    ⚠ 정정인(C)의 배치 알고리즘이 아니다. 이송 중 적재 최상단이 자라고 **박스 크기가
      매번 달라지는** 상황을 만들려는 것뿐이다. 실제 계획은 `--plan plan.json` 으로 넣는다.

    크기를 섞는 게 중요하다. 3호만 돌리면 가장 유리한 경우만 보게 된다 —
    5호(480x380x340)는 흡착면 아래로 340 mm 매달려서 팔에 걸릴 여지가 전혀 다르고,
    통로 기둥 단면도 박스마다 다르다(명세서 7.5). 순서는 `boxes.random_seed` 로 고정한다.
    """
    types = cfg["boxes"]["types"]
    rng = np.random.default_rng(cfg["boxes"]["random_seed"])
    dist = np.array(cfg["boxes"]["size_distribution"], float)
    dist = dist / dist.sum()
    pw, pd = cfg["pallet"]["width_mm"], cfg["pallet"]["depth_mm"]
    max_stack = cfg["pallet"]["max_stack_mm"]
    gap = int(2 * CP.PLACE_ERR_MM) + 6             # 이웃 간격 = 배치오차 2배 + 여유

    # ★ 박스 종류를 **층마다** 뽑는다 (박스마다가 아니라).
    #
    #   같은 층에 높이가 다른 박스를 나란히 놓으면 통로 검사에 걸린다. 5호(h=340) 옆
    #   4호(h=280) 를 20 mm 간격으로 두면, 통로 링이 30 mm 라 창이 겹치고 **더 높은
    #   이웃**이 창 안에 들어온다. `corridor_clear` 는 그 자리를 후보에서 빼므로
    #   C 의 알고리즘이면 안 나올 계획인데, 더미가 그걸 만들어 냈다.
    #   실측: `check_place_plan.py` 가 "그리퍼 몸통이 더 높은 이웃과 4 mm 뿐" 으로 잡았다.
    #
    #   층마다 한 종류로 채우면 같은 층 높이가 같아져 이 상황이 안 생긴다.
    #   크기는 층 사이에서 섞이므로 3·4·5호를 다 보는 목적은 유지된다.
    out = []
    z = x = y = row_d = layer_h = 0                # 층 바닥 / 선반 커서 / 이 층의 최고 높이
    b = types[int(rng.choice(len(types), p=dist))]
    for _ in range(n):
        w, d, h = b["w_mm"], b["d_mm"], b["h_mm"]
        yaw = 0
        if x + w > pw and x + d <= pw:             # 돌리면 들어가면 돌린다 (yaw 0/90 만 허용)
            w, d, yaw = d, w, 90
        if x + w > pw:                             # 다음 줄
            x, y, row_d = 0, y + row_d + gap, 0
        if y + d > pd:                             # 다음 층 — 종류를 새로 뽑는다
            x = y = row_d = 0
            z += layer_h
            layer_h = 0
            b = types[int(rng.choice(len(types), p=dist))]
            w, d, h, yaw = b["w_mm"], b["d_mm"], b["h_mm"], 0
        if z + h > max_stack:                      # 더 못 쌓는다
            break
        out.append({"box_type": b["name"], "yaw_deg": yaw, "z_mm": int(z),
                    "x_mm": int(x + w / 2), "y_mm": int(y + d / 2)})
        x += w + gap
        row_d = max(row_d, d)
        layer_h = max(layer_h, h)
    return out


def box_arm_gap(q, joints, spheres, base, grip, w_mm, d_mm, h_mm):
    """**들고 있는 박스**와 팔 링크 사이 최소 거리 (m). 음수면 파고든 것이다.

    호로 돌 때 박스가 팔 자신에 걸리는지를 본다. 지금까지 아무도 안 봤다 —
    `import_h2017_usd.py` 가 `self_collision = False` 로 임포트하므로 **Isaac 물리도
    안 잡아준다.** 박스를 띄워도 팔을 그냥 통과한다. 숫자로 보는 수밖에 없다.

    박스는 흡착면에 강체로 붙어 툴 좌표계에서 매달린다 — 툴 z 방향으로 0~h,
    x·y 로 ±w/2, ±d/2. 툴이 기울면 박스도 같이 기우므로 **툴 프레임에서** 계산한다
    (월드 AABB 로 근사하면 기운 구간에서 틀린다).

    `tool0` 과 `link_6` 은 뺀다 — 둘 다 플랜지에 강체로 붙어 **구조상 박스 위**에 있다.
    특히 link_6 은 자세와 무관하게 항상 125.3 mm 로 나온다(구체 하단이 플랜지 아래 24.7 mm,
    박스 윗면이 그리퍼 길이 150 mm 아래). 그 상수가 최소값을 계속 차지하면
    **실제로 움직이는 링크가 안 보인다.**
    """
    T = F.fk(list(np.asarray(q, float)), joints)
    R6, p6 = T["tool0"]
    R6 = np.array(R6)
    face = base + np.array(p6) + R6[:, 2] * grip          # 흡착면 = 박스 윗면 중심
    half = np.array([w_mm, d_mm, h_mm], float) / 2000.0
    center = face + R6[:, 2] * (h_mm / 2000.0)            # 박스 중심
    best, where = 1e9, ""
    for link in ARM_LINKS:
        if link in ("tool0", "link_6"):
            continue
        Rl, pl = T[link]
        for c, r in spheres.get(link, []):
            wc = base + np.array(pl) + np.array(Rl) @ c
            loc = R6.T @ (wc - center)                    # 툴 프레임으로
            d = float(np.linalg.norm(np.maximum(np.abs(loc) - half, 0.0))) - r
            if d < best:
                best, where = d, link
    return best, where


def arm_spheres_world(q, joints, spheres, base):
    """관절각 q 에서 팔 충돌구체의 (월드중심, 반경) 목록."""
    T = F.fk(list(q), joints)
    out = []
    for link in ARM_LINKS:
        R, p = T[link]
        for c, r in spheres.get(link, []):
            out.append((base + np.array(p) + np.array(R) @ c, r, link))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--warm", default="chain", choices=["chain", "ready"],
                    help="chain = 직전 자세(현행 ArticulationKinematicsSolver) / "
                         "ready = 고정 기준자세(처방)")
    ap.add_argument("--boxes", type=int, default=None,
                    help="박스 수. 기본은 system.yaml 의 stage1.total_count")
    ap.add_argument("--plan", default=None, help="plan.json (없으면 더미 계획)")
    ap.add_argument("--vel", type=float, default=0.5, help="URDF 속도한계 대비 배율")
    ap.add_argument("--interp", default="task", choices=["task", "joint"],
                    help="task = 흡착면 직선 + 매 스텝 IK (자세 유지) / "
                         "joint = 웨이포인트에서만 IK + 관절 보간 (옛 방식, 자세 안 지켜짐)")
    ap.add_argument("--cell-yaw", type=float, default=0.0,
                    help="셀 배치 실험. 팔레트를 **로봇 베이스 둘레로** 이 각도만큼 돌린다 "
                         "(deg, 90 의 배수만 정확하다). 0 = system.yaml 그대로. "
                         "★ 90도 배치는 2026-08-22 확정돼 system.yaml 에 들어갔다 "
                         "(브랜치 layout/cell-90deg). 이 옵션은 그 뒤로는 배치 실험용이고, "
                         "회전 전 배치를 재현하려면 -90 을 준다")
    ap.add_argument("--transit", default="arc", choices=["arc", "line"],
                    help="이송 경로. arc = 베이스 둘레 호(반경·각도 보간) / "
                         "line = 흡착면 직선 (옛 방식)")
    ap.add_argument("--yaw", default="follow", choices=["follow", "fixed"],
                    help="이송 중 툴 yaw. follow = 호와 같이 돈다(손목 되감기 없음) / "
                         "fixed = 월드에 고정 (옛 방식, j1 이 도는 만큼 손목이 되감긴다)")
    ap.add_argument("--no-unwrap", action="store_true",
                    help="박스 사이 j5 등가각 되돌리기를 끈다")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--visual", action="store_true",
                    help="--gui 없이도 시각 프림을 만든다 (오프스크린 녹화·경로 점검용)")
    ap.add_argument("--no-visual", action="store_true",
                    help="--gui 에서 팔레트·벨트·박스 시각 프림을 안 띄운다. "
                         "시각 전용(물리 없음)이라 측정값에는 영향이 없다")
    ap.add_argument("--hold", action="store_true")
    ap.add_argument("--log", default=None)
    ap.add_argument("--dump-plan", default=None,
                    help="쓰인 계획을 plan.json 으로 저장. "
                         "`python3 scripts/check_place_plan.py plan.json` 으로 프리플라이트")
    a = ap.parse_args()

    _paths.check()

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})
    tee = B1.start_log(a.log or _paths.ROOT / "results" /
                       ("cycle_%s_%s_%s_%s.txt"
                        % (a.zone, a.interp, a.warm, a.transit)))

    S = B1.bring_up(zone=a.zone, render=a.gui)
    cfg = S.cfg
    joints = F.load_joints()
    spheres = VR.load_spheres()
    vlim = np.array([joints["joint_%d" % i]["velocity"] for i in range(1, 7)])
    jlo = np.array([joints["joint_%d" % i]["lower"] for i in range(1, 7)])
    jhi = np.array([joints["joint_%d" % i]["upper"] for i in range(1, 7)])

    types = {b["name"]: b for b in cfg["boxes"]["types"]}
    origin = np.array(cfg["zones"]["pallet_origin_world_mm"][a.zone], float)
    pw, pd = cfg["pallet"]["width_mm"], cfg["pallet"]["depth_mm"]
    belt = cfg["conveyor"]["belt_top_z_mm"]
    stop = cfg["conveyor"]["stopper_x_mm"]
    app_mm = cfg["robot"]["approach_offset_mm"]
    target_s = cfg["robot"]["cycle_time_target_s"]

    # ── 셀 회전 (배치 실험용) ────────────────────────────────────────
    #  팔레트를 **로봇 베이스 둘레로** 통째로 돌린다. 팔레트 로컬 축도 같이 돈다.
    #  0  = system.yaml 그대로.
    #  90 = 로봇이 팔레트의 **1200 변 바깥**에 서는 배치.
    #       팔레트만 돌리면 최원점이 1902 mm 로 도달범위(1700)를 203 mm 넘는다.
    #       로봇까지 같이 돌아야 1667 mm 로 유지된다 — 그게 이 옵션이다.
    #
    #  ★ 이 실험으로 90도 배치가 확정됐다 (2026-08-22). 확정 이유는 도달범위가 아니라
    #    **트럭 적재**다 — 존 방향으로 팔레트가 1200 이면 3개 붙여 3600 mm 라 트럭 바닥
    #    2800 을 넘는다. 800 이어야 2400 이 된다 (구 사양서 5-2).
    #    확정 좌표는 system.yaml 에 반영됐다. 이 옵션은 이후 배치 실험용으로 남긴다.
    _cy = math.radians(a.cell_yaw)
    CR = np.array([[math.cos(_cy), -math.sin(_cy)],
                   [math.sin(_cy), math.cos(_cy)]])
    base_xy = np.asarray(S.base[:2], float) * 1000.0
    origin_xy = base_xy + CR @ (origin[:2] - base_xy)        # 회전된 팔레트 원점 (mm)

    def pallet_to_world(x_mm, y_mm):
        """팔레트 로컬 (Px, Py) mm -> 월드 xy (m). 셀 회전을 포함한다."""
        return (origin_xy + CR @ np.array([float(x_mm), float(y_mm)])) / 1000.0

    _cor = np.array([pallet_to_world(x, y) for x, y in
                     ((0, 0), (pw, 0), (0, pd), (pw, pd))])
    pal_lo, pal_hi = _cor.min(axis=0), _cor.max(axis=0)       # 90 배수면 정확한 경계다

    n = a.boxes if a.boxes else cfg["boxes"]["stage1"]["total_count"]
    if a.plan:
        plan = json.loads(Path(a.plan).read_text())[:n]
        src = a.plan
    else:
        plan = mixed_plan(cfg, n)
        src = "더미 계획 (3·4·5호 혼합, seed %d)" % cfg["boxes"]["random_seed"]

    if a.dump_plan:
        Path(a.dump_plan).write_text(json.dumps(plan, ensure_ascii=False, indent=1))
        print("PLAN   -> %s (check_place_plan.py 로 프리플라이트 가능)" % a.dump_plan)

    # 어떤 박스가 몇 개인지 찍어 둔다 — 3호만 돌리면 가장 유리한 경우만 보게 된다.
    mix = {}
    for pl in plan:
        mix[pl.get("box_type", "3호")] = mix.get(pl.get("box_type", "3호"), 0) + 1

    print("ZONE   %s  base %s mm  gripper %.0f mm" %
          (a.zone, list(np.round(S.base * 1000).astype(int)), S.grip * 1000))
    print("WARM   %s" % ("직전 자세 — 현행 ArticulationKinematicsSolver" if a.warm == "chain"
                         else "고정 기준자세(READY) — LulaKinematicsSolver warm_start"))
    print("PLAN   %s, %d 박스 (%s)   팔레트 원점 %s mm"
          % (src, len(plan), ", ".join("%s x%d" % kv for kv in sorted(mix.items())),
             list(origin.astype(int))))
    print("MOTION %s 보간, URDF 속도한계의 %.0f%%"
          % ("태스크공간(흡착면 직선 + 매 스텝 IK)" if a.interp == "task"
             else "관절공간 선형 — ⚠ 중간 자세가 보존되지 않는다", a.vel * 100))
    if abs(a.cell_yaw) > 1e-9:
        _c = np.array([pallet_to_world(x, y) for x, y in
                       ((0, 0), (pw, 0), (0, pd), (pw, pd))]) * 1000.0
        print("CELL   ★ 배치 실험 — 팔레트를 베이스 둘레로 %+.0f도 돌렸다 "
              "(system.yaml 은 안 건드림)" % a.cell_yaw)
        print("       팔레트 월드 범위  x %.0f ~ %.0f   y %.0f ~ %.0f mm"
              % (_c[:, 0].min(), _c[:, 0].max(), _c[:, 1].min(), _c[:, 1].max()))
        print("       존 간격 %d mm 방향(=Y)으로 팔레트가 %.0f mm -> 3개 붙이면 %.0f mm "
              "(트럭 바닥 2800)"
              % (1500, _c[:, 1].max() - _c[:, 1].min(),
                 3 * (_c[:, 1].max() - _c[:, 1].min())))
    print("TRANSIT %s   |   툴 yaw %s   |   손목 되감기 %s"
          % ("베이스 둘레 **호** (반경·각도 보간)" if a.transit == "arc"
             else "흡착면 직선 — ⚠ 정반대 두 점이면 중점이 베이스 위(반경 0)다",
             "호를 따라 회전 — 손목이 되감을 일이 없다" if a.yaw == "follow"
             else "월드 고정 — ⚠ j1 이 도는 만큼 손목이 되감긴다",
             "끔" if a.no_unwrap else "박스 사이 j5 등가각 복원"))
    print("TARGET cycle_time_target_s = %.1f s\n" % target_s)

    # ---- 샘플링 상태. 팔레트 위를 지나는 팔의 최저점을 매 스텝 본다.
    sample = {"low": 1e9, "link": "", "at": 0.0, "tilt": 0.0, "tilt_carry": 0.0,
          "tilt_cmd": 0.0, "tilt_cmd_carry": 0.0, "at_wp": "",
          "boxgap": 1e9, "boxlink": "", "boxstack": 1e9, "boxbelt": 1e9}

    def _vis_follow(q_cmd):
        """매달린 시각 박스를 툴 자세에 맞춰 옮긴다. box_arm_gap 과 같은 기하다."""
        R6, p6 = F.fk(list(np.asarray(q_cmd, float)), joints)["tool0"]
        R6 = np.array(R6)
        face_w = np.asarray(S.base, float) + np.array(p6) + R6[:, 2] * S.grip
        vis_now[0].set_world_pose(
            position=face_w + R6[:, 2] * (cur_box[2] / 2000.0),
            orientation=_quat_of(R6))

    def probe(stack_top_m, corr, carrying, q_cmd=None, wp_name=""):
        """팔레트 위 · **통로 기둥 밖** 충돌구체의 최저점을 갱신한다.

        ⚠ 기둥 안쪽을 빼는 것이 핵심이다. 배치하려고 일부러 내려가는 구간까지 세면
          여유가 항상 음수로 나온다 — 그 하강은 트랙 C 의 통로 검사(명세서 7.5)가
          "기둥이 비었다"고 보장하는 구간이다. 여기서 재려는 것은 **그 보장 밖**,
          즉 기둥 옆에서 팔이 적재물 높이 아래로 내려오는가다.

        corr = (중심 x, 중심 y, 반폭 x, 반폭 y) m. 지금 목표점의 기둥.

        시각 박스도 여기서 툴에 붙여 따라가게 한다 (--gui 일 때만, 물리 없음).
        """
        cx, cy, hx, hy = corr
        q = np.asarray(S.art.get_joint_positions()[:6], float)
        if vis_now[0] is not None and carrying:
            _vis_follow(q)                 # 매달린 박스가 툴을 따라간다 (시각 전용)
        # 툴 기울기 — 툴 z 축과 월드 -Z 사이 각. 박스를 매단 채로는 이게 곧 박스 기울기다.
        #
        # ★ **지령**과 **실제**를 따로 잰다. 둘이 갈리는 자리가 원인을 가른다:
        #     지령도 기울면  -> IK / 보간이 자세를 안 지킨 것 (내 코드 문제)
        #     지령은 0 인데 실제만 기울면 -> 팔이 지령을 못 따라간 것 (속도한계·게인 문제)
        #   추측으로는 구분이 안 된다. 실제로 한 번 틀린 진단을 했다.
        tilt = _tilt(q)
        if tilt > sample["tilt"]:
            sample["tilt"] = tilt
        if carrying and tilt > sample["tilt_carry"]:
            sample["tilt_carry"] = tilt
        if carrying:
            g, gw = box_arm_gap(q, joints, spheres, S.base, S.grip,
                                cur_box[0], cur_box[1], cur_box[2])
            if g < sample["boxgap"]:
                sample.update(boxgap=g, boxlink=gw)
            # 들고 있는 박스 **밑면**이 적재물·벨트 위로 얼마나 떠 있나.
            # 팔 충돌구체 여유와 다른 값이다 — 부딪히는 건 패드가 아니라 매달린 박스다
            # (구 저장소 실측: 이걸 빼먹고 앞서 놓은 박스를 1,061 mm 밀어냈다).
            #
            # ★ **`carry_over` 구간에서만** 잰다. 이유 둘:
            #   · 배치 하강(place_app -> place)은 이웃보다 낮게 내려가는 게 **정상**이다.
            #     그 기둥이 비었음은 C 의 통로 검사가 보장한다. 그걸 세면 항상 음수가 나온다
            #     (실측 -333 mm — 4호를 1층에 놓는데 옆에 5호가 서 있던 경우다).
            #   · 픽업 직후에는 박스가 아직 벨트에 얹혀 있어 여유가 정의상 0 이다.
            #   남는 것이 **가로지르는 구간** 하나이고, 그게 통로 검사의 사각지대다.
            if wp_name == "carry_over":
                fc = face_of(q)
                bot = fc[2] - cur_box[2] / 1000.0
                if pal_lo[0] <= fc[0] <= pal_hi[0] and pal_lo[1] <= fc[1] <= pal_hi[1]:
                    sample["boxstack"] = min(sample["boxstack"], bot - stack_top_m)
                if fc[0] < pal_lo[0]:                 # 컨베이어 쪽
                    sample["boxbelt"] = min(sample["boxbelt"], bot - belt / 1000.0)
        if q_cmd is not None:
            tc = _tilt(np.asarray(q_cmd, float))
            if tc > sample["tilt_cmd"]:
                sample["tilt_cmd"] = tc
            if carrying and tc > sample["tilt_cmd_carry"]:
                sample["tilt_cmd_carry"] = tc
                sample["at_wp"] = wp_name
        for w, r, link in arm_spheres_world(q, joints, spheres, S.base):
            if not (pal_lo[0] <= w[0] <= pal_hi[0] and pal_lo[1] <= w[1] <= pal_hi[1]):
                continue                       # 팔레트 위가 아니면 관심 밖
            if abs(w[0] - cx) <= hx and abs(w[1] - cy) <= hy:
                continue                       # 통로 기둥 안 — C 가 보장하는 구간
            bottom = w[2] - r
            if bottom < sample["low"]:
                sample.update(low=bottom, link=link, at=stack_top_m)

    def move(face_from, face_to, q_end, stack_top_m, corr, carrying, wp="",
             yaw_from=0.0, yaw_to=0.0, arc=False):
        """흡착면 face_from -> face_to. (소요 시간 s, 실패한 스텝 수) 를 돌려준다.

        `--interp task`  : 위치를 직선으로 쪼개고 **매 스텝 IK**. 자세가 경로 내내 유지된다.
        `--interp joint` : 끝점 해로 관절 선형 보간. **중간 자세는 보존되지 않는다** — 옛 방식.

        스텝 수는 끝점까지의 관절 이동량을 URDF 속도한계로 나눠 잡는다.
        """
        q0 = np.asarray(S.art.get_joint_positions()[:6], float)
        dq = np.asarray(q_end, float) - q0
        secs = float(np.max(np.abs(dq) / (vlim * a.vel))) if np.any(dq) else 0.0
        steps = max(1, int(math.ceil(secs * S.hz)))
        f0, f1 = np.asarray(face_from, float), np.asarray(face_to, float)

        # ★ 이송 경로 — 직선이 아니라 **베이스 둘레 호**.
        #   픽업(θ=180°)과 팔레트(θ≈0°)를 직선으로 이으면 중점이 베이스 바로 위,
        #   즉 반경 0 이라 도달범위를 벗어난다 (실측: 경로 중간 IK 실패 73 스텝).
        #   반경과 높이는 선형으로, 각도만 짧은 쪽으로 돌리면 반경이 절대 0 이 안 된다.
        bxy = np.asarray(S.base[:2], float)
        v0, v1 = f0[:2] - bxy, f1[:2] - bxy
        r0, r1 = float(np.linalg.norm(v0)), float(np.linalg.norm(v1))
        use_arc = arc and r0 > 0.15 and r1 > 0.15      # 베이스 축 위면 각도가 무의미
        th0 = math.atan2(v0[1], v0[0])
        dth = _wrap(math.atan2(v1[1], v1[0]) - th0)

        def at(u):
            if not use_arc:
                return f0 + (f1 - f0) * u
            r = r0 + (r1 - r0) * u
            th = th0 + dth * u
            return np.array([bxy[0] + r * math.cos(th),
                             bxy[1] + r * math.sin(th),
                             f0[2] + (f1[2] - f0[2]) * u])

        def yaw_at(u):
            return yaw_from + (yaw_to - yaw_from) * u

        prev, miss, over = q0, 0, 0
        cap = vlim * a.vel / S.hz          # 물리 스텝 하나에 허용되는 관절 변화 (rad)

        if a.interp == "joint":
            for k in range(1, steps + 1):
                qk = q0 + dq * (k / steps)
                S.art.apply_action(S.ArticulationAction(joint_positions=qk))
                S.world.step(render=S.render)
                probe(stack_top_m, corr, carrying, qk, wp)
        else:
            # ★ 경로 매개변수 u 를 **속도한계에 맞춰** 전진시킨다.
            #
            #   스텝 수를 시작↔끝 관절차로만 잡으면 안 된다. 손목 특이점(j5≈0) 근처를
            #   지나는 구간에서는 툴 자세를 유지하느라 j4·j6 가 폭발적으로 돌아야 해서,
            #   같은 거리를 가도 필요한 관절 이동량이 몇 배가 된다. 그때 스텝당 지령이
            #   속도한계를 넘고, 드라이브가 못 따라가 **툴이 뒤처져 돈다**.
            #   실측: 지령 기울기 0.1° 인데 실제 179.5° 였다.
            #
            #   그래서 매 스텝 후보 해를 먼저 풀어 보고, 관절 변화가 한계를 넘으면
            #   u 증분을 반으로 줄인다. 특이점 근처에서 **경로가 알아서 느려진다** —
            #   실제 컨트롤러가 하는 일과 같다. 사이클 타임이 늘어나는 건 정직한 결과다.
            u, k = 0.0, 0
            du = 1.0 / steps
            lim = int(20 * S.hz)               # 한 구간 20 s 상한 (무한루프 방지)
            while u < 1.0 - 1e-9 and k < lim:
                step_u = min(du, 1.0 - u)
                qk, ok = None, False
                for _ in range(10):            # 최대 10회 이등분
                    cand, cok = solve(at(u + step_u), seed_for(prev),
                                      yaw_at(u + step_u))
                    if cok and np.all(np.abs(cand - prev) <= cap):
                        qk, ok = cand, True
                        break
                    if cok and qk is None:
                        qk = cand              # 한계는 넘지만 풀리긴 한 해 (마지막 수단)
                    step_u /= 2.0
                if qk is None:                 # 아예 못 풀었다
                    miss += 1
                    qk = prev
                elif not ok:                   # 풀렸지만 속도한계 초과
                    over += 1
                prev = qk
                u += step_u
                k += 1
                S.art.apply_action(S.ArticulationAction(joint_positions=qk))
                S.world.step(render=S.render)
                probe(stack_top_m, corr, carrying, qk, wp)
            steps = max(k, 1)
        q_cmd = np.asarray(prev if a.interp == "task" else q_end, float)
        for _ in range(int(0.15 * S.hz)):      # 정착
            S.art.apply_action(S.ArticulationAction(joint_positions=q_cmd))
            S.world.step(render=S.render)
            probe(stack_top_m, corr, carrying, q_cmd, wp)
        return steps / S.hz, miss, over, q_cmd

    def solve(face_world, seed, yaw=0.0):
        """흡착면 월드좌표 -> (q, ok). 툴 z 는 항상 아래, **yaw 는 호출자가 정한다.**

        yaw 를 월드에 고정하면 j1 이 도는 만큼 손목이 반대로 되감긴다 — 이것이
        j5 가 한계 ±360° 의 99% 까지 감긴 원인이다. 이송 중에는 박스 방향이
        아무래도 좋으므로 yaw 가 호를 따라 같이 돌면 손목은 돌 필요가 없다.

        `ArticulationKinematicsSolver` 를 쓰지 않는다 — 그건 현재 관절각을 시드로
        **강제**해서(`articulation_kinematics_solver.py:94`) 시드를 고를 수가 없다.
        """
        tgt = np.asarray(face_world, float) + np.array([0.0, 0.0, S.grip])
        quat = S.down_quat if a.yaw == "fixed" else _quat_down(yaw)
        q, ok = S.lula.compute_inverse_kinematics("tool0", tgt, quat,
                                                  warm_start=np.asarray(seed, float))
        return (np.asarray(q, float) if ok else None), ok

    def seed_for(prev):
        return prev if a.warm == "chain" else S.ready

    def dtheta(p_from, p_to):
        """베이스에서 본 두 점의 방위각 차 (짧은 쪽, rad)."""
        b = np.asarray(S.base[:2], float)
        return _wrap(math.atan2(p_to[1] - b[1], p_to[0] - b[0])
                     - math.atan2(p_from[1] - b[1], p_from[0] - b[0]))

    def unwrap_wrist():
        """박스를 놓은 뒤(빈 그리퍼) j5 를 등가각으로 되돌린다. 자세는 안 바뀐다.

        j5 는 ±2pi 가 같은 자세다. 감긴 채로 다음 사이클에 들어가면 누적돼
        한계에 붙고, 그때 손목이 더 못 돌아 **툴이 지령을 못 따라간다**.
        박스를 들고 있을 때는 절대 하지 않는다 — 360° 도는 동안 쏟는다.
        """
        if a.no_unwrap:
            return 0.0
        q = np.asarray(S.art.get_joint_positions()[:6], float)
        j, best = 4, float(q[4])
        for k in (-1, 1):
            cand = q[j] + k * 2.0 * math.pi
            if jlo[j] + 0.05 <= cand <= jhi[j] - 0.05 and abs(cand) < abs(best) - 1e-6:
                best = cand
        if abs(best - q[j]) < 1e-6:
            return 0.0
        tgt = q.copy()
        tgt[j] = best
        n = max(1, int(math.ceil(abs(best - q[j]) / (vlim[j] * a.vel) * S.hz)))
        for k in range(1, n + 1):
            S.art.apply_action(S.ArticulationAction(joint_positions=q + (tgt - q) * (k / n)))
            S.world.step(render=S.render)
        return n / S.hz

    def _tilt(q):
        """툴 z 축과 월드 -Z 사이 각 (deg)."""
        R, _ = F.fk(list(np.asarray(q, float)), joints)["tool0"]
        return math.degrees(math.acos(max(-1.0, min(1.0, -R[2][2]))))

    def face_of(q):
        """관절각 q 에서 흡착면의 월드 좌표. tool0 에서 툴 z 방향으로 그리퍼 길이만큼."""
        R, p = F.fk(list(np.asarray(q, float)), joints)["tool0"]
        tz = np.array([R[0][2], R[1][2], R[2][2]])
        return S.base + np.array(p) + tz * S.grip

    cur_box = [340, 250, 210]           # probe 가 참조한다. 사이클마다 갱신.
    vis_now = [None]                    # 지금 매달려 있는 시각 박스 (probe 가 갱신)

    def make_visuals():
        """영상용 시각 프림 — 팔레트 · 픽업 벨트 · 박스.

        ★ **물리가 없다.** VisualCuboid 는 콜라이더도 리지드바디도 없으므로
        측정값(도달·기울기·간격)에 전혀 영향을 주지 않는다. 실제 흡착·접촉은
        B3 이후에 본다. 여기서는 "무엇을 하는 동작인지"가 보이게만 한다.
        """
        try:
            from isaacsim.core.api.objects import VisualCuboid
        except Exception as exc:                      # 판이 다르면 그냥 생략한다
            print("VISUAL 생략 — VisualCuboid 임포트 실패: %s" % exc)
            return None
        th = cfg["pallet"]["thickness_mm"]
        qz = np.array([math.cos(_cy / 2.0), 0.0, 0.0, math.sin(_cy / 2.0)])

        # ── 페데스탈 ────────────────────────────────────────────────
        #  없으면 영상에서 로봇이 바닥에서 700 mm 떠 보인다.
        #  치수는 h2017_rmpflow_common.yaml 의 충돌 원통과 같은 것 — **D400 x 700**.
        ped = cfg["robot"]["pedestal_height_mm"] / 1000.0
        try:
            from isaacsim.core.api.objects import VisualCylinder
            VisualCylinder(prim_path="/World/vis/pedestal", name="vis_pedestal",
                           position=np.array([S.base[0], S.base[1], ped / 2.0]),
                           radius=0.20, height=ped,
                           color=np.array([0.35, 0.36, 0.38]))
        except Exception:                       # 원통이 없는 판이면 각기둥으로
            VisualCuboid(prim_path="/World/vis/pedestal", name="vis_pedestal",
                         position=np.array([S.base[0], S.base[1], ped / 2.0]),
                         scale=np.array([0.4, 0.4, ped]),
                         color=np.array([0.35, 0.36, 0.38]))

        # ── 팔레트 롤러 라인 ────────────────────────────────────────
        #  ★ 상면이 곧 **도크 바닥 z = 0** 이다 (구 사양서 5-1). 바닥에 얹는 구조물이
        #    아니라 바닥과 같은 면이다 — 팔레트가 트럭 적재함으로 **같은 높이로**
        #    밀려 들어가야 하기 때문. 팔레트를 49 mm 만 올려도 최원점이 1700 을 넘는다.
        #  세 존의 팔레트가 이 라인 위에 800 씩 붙어 2400 (= 트럭 바닥, 5-2) 이 된다.
        cen = pallet_to_world(pw / 2.0, pd / 2.0)
        span = np.abs(CR @ np.array([pw, pd]))          # 회전 후 월드축 투영 (mm)
        #  ★ 롤러 상면 = **팔레트 밑면** = 트럭 적재함 바닥 = 월드 z 0 (도크 규약).
        #    팔레트를 들지 않고 그대로 밀어 넣으므로 세 면이 같은 높이다.
        #    ⚠ truck.bed_height_mm(850) 을 여기 쓰면 안 된다 — 그건 월드 z 가 아니라
        #      **야드 지면 대비** 도크 높이다. 쓰면 슬래브가 팔레트 위에 떠서 상판처럼 보인다.
        bed = origin[2] - cfg["pallet"]["thickness_mm"]      # = 0
        VisualCuboid(prim_path="/World/vis/roller", name="vis_roller",
                     position=np.array([cen[0], 0.0, (bed - 75.0) / 1000.0]),
                     scale=np.array([(span[0] + 400.0) / 1000.0, 6.0, 0.15]),
                     color=np.array([0.30, 0.30, 0.34]))

        # ── 팔레트 (이 존) ──────────────────────────────────────────
        VisualCuboid(prim_path="/World/vis/pallet", name="vis_pallet",
                     position=np.array([cen[0], cen[1], (origin[2] - th / 2.0) / 1000.0]),
                     scale=np.array([pw, pd, th]) / 1000.0, orientation=qz,
                     color=np.array([0.45, 0.32, 0.18]))

        # ── 옆 존 팔레트 (트럭에 3개가 어떻게 붙는지 보이라고) ────────
        for z2 in cfg["zones"]["list"]:
            if z2 == a.zone:
                continue
            b2 = np.array(cfg["zones"]["robot_base_world_mm"][z2], float)[:2]
            o2 = np.array(cfg["zones"]["pallet_origin_world_mm"][z2], float)
            c2 = (b2 + CR @ (o2[:2] - b2) + CR @ np.array([pw / 2.0, pd / 2.0])) / 1000.0
            VisualCuboid(prim_path="/World/vis/pallet_%s" % z2, name="vis_pallet_%s" % z2,
                         position=np.array([c2[0], c2[1], (o2[2] - th / 2.0) / 1000.0]),
                         scale=np.array([pw, pd, th]) / 1000.0, orientation=qz,
                         color=np.array([0.30, 0.24, 0.16]))
        x0, x1 = cfg["conveyor"]["pickup_range_mm"]
        VisualCuboid(prim_path="/World/vis/belt", name="vis_belt",
                     position=np.array([(x0 + x1) / 2000.0, 0.0, (belt - 50.0) / 1000.0]),
                     scale=np.array([(x1 - x0) / 1000.0, 0.7, 0.1]),
                     color=np.array([0.24, 0.27, 0.31]))
        col = {"3호": np.array([0.86, 0.68, 0.40]),
               "4호": np.array([0.78, 0.54, 0.26]),
               "5호": np.array([0.66, 0.42, 0.20])}
        out = []
        for k, pl2 in enumerate(plan):
            b = types[pl2.get("box_type", "3호")]
            out.append(VisualCuboid(
                prim_path="/World/vis/box_%02d" % k, name="vis_box_%02d" % k,
                position=np.array([0.0, 0.0, -5.0]),      # 차례가 올 때까지 바닥 아래
                scale=np.array([b["w_mm"], b["d_mm"], b["h_mm"]]) / 1000.0,
                color=col.get(b["name"], np.array([0.8, 0.6, 0.3]))))
        print("VISUAL 팔레트 · 픽업벨트 · 박스 %d 개 (시각 전용, 물리 없음)" % len(out))
        return out

    vis_boxes = make_visuals() if ((a.gui or a.visual) and not a.no_visual) else None

    B1.teleport(S, S.ready, steps=int(0.5 * S.hz))
    yaw_cur = [0.0]                      # 마지막으로 지령한 툴 yaw (rad). 박스 간 이어진다
    j5_hist = [float(np.rad2deg(S.ready[4]))]
    rows, fails = [], []
    obstacle = 0.0                                   # 적재 최상단 (팔레트 좌표 mm)

    print("%3s %-5s %10s %8s %8s %9s %10s %13s %10s  %s"
          % ("#", "type", "배치 mm", "적재top", "사이클s", "j5 범위", "이송여유",
             "기울기 지령/실제", "박스-팔", "판정"))
    print("    (이송여유 '-' = 적재가 비어 팔레트 위 통로 밖에 팔이 없었다는 뜻)")

    for i, pl in enumerate(plan, 1):
        box = types[pl.get("box_type", "3호")]
        h = box["h_mm"]
        cur_box[:] = [box["w_mm"], box["d_mm"], h]
        top = pl["z_mm"] + h                          # 이 박스의 상단 (팔레트 좌표)
        # 이송 높이 — 흡착면 기준. 박스는 이 아래로 h 만큼 매달린다.
        #   ① 팔레트 쪽: 적재 최상단 + 접근여유 + 박스높이   (check_place_plan.py 검사 5)
        #   ② 컨베이어 쪽: 벨트 상면 + 접근여유 + 박스높이
        # ★ ②를 빼면 안 된다. 5호(h=340) 를 빈 팔레트에 놓을 때 ①만 쓰면 1030 mm 라
        #   박스 밑면이 690 mm — **벨트 상면(800) 아래 110 mm** 다. 집자마자 팔이
        #   박스를 벨트에 내리누른다. (영상으로 잡혔다. 숫자만 보면 안 보인다.)
        carry_face = max(origin[2] + max(obstacle, top), belt) + app_mm + h
        carry_face /= 1000.0
        stack_top_m = (origin[2] + obstacle) / 1000.0

        pick_xy = np.array([stop - box["w_mm"] / 2.0, 0.0])
        if vis_boxes:                      # 이번 박스를 벨트 위 픽업 자리에 올린다
            vis_now[0] = vis_boxes[i - 1]
            vis_now[0].set_world_pose(
                position=np.array([pick_xy[0] / 1000.0, pick_xy[1] / 1000.0,
                                   (belt + h / 2.0) / 1000.0]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]))
        pick_face = (belt + h) / 1000.0
        place_xy = pallet_to_world(pl["x_mm"], pl["y_mm"])
        place_face = (origin[2] + top) / 1000.0

        face_prev = face_of(S.art.get_joint_positions()[:6])

        # ★ yaw 계획 — 방향을 **손목이 아니라 j1 이 만들게 한다.**
        #   이송 중 박스가 어느 쪽을 보든 상관없고, 필요한 건 놓을 때의 yaw 뿐이다.
        #   호가 끝나는 방향에서 가장 가까운 대표각(직사각이라 180° 주기)을 고르면
        #   손목이 돌아야 하는 양이 최소가 된다. yaw 를 월드에 고정하던 옛 방식은
        #   j1 이 도는 만큼(여기서는 180°) 손목에 그대로 청구서를 넘겼다.
        yaw_pick = _near(0.0, math.degrees(yaw_cur[0])
                         + math.degrees(dtheta(face_prev[:2], pick_xy / 1000.0)))
        yaw_place = _near(float(pl.get("yaw_deg", 0) or 0) + a.cell_yaw,
                          yaw_pick + math.degrees(dtheta(pick_xy / 1000.0, place_xy)))
        yp, yq = math.radians(yaw_pick), math.radians(yaw_place)

        wps = [
            ("pick_app",   np.r_[pick_xy / 1000.0, pick_face + app_mm / 1000.0], yp),
            ("pick",       np.r_[pick_xy / 1000.0, pick_face],                   yp),
            ("carry_up",   np.r_[pick_xy / 1000.0, carry_face],                  yp),
            ("carry_over", np.r_[place_xy, carry_face],                          yq),
            ("place_app",  np.r_[place_xy, place_face + app_mm / 1000.0],        yq),
            ("place",      np.r_[place_xy, place_face],                          yq),
            ("retreat",    np.r_[place_xy, carry_face],                          yq),
        ]

        # 지금 목표점의 통로 기둥 — 명세서 7.5 와 같은 정의 (박스 단면 + 여유 사방).
        # 픽업 구간에서는 기둥이 컨베이어 위에 있어 팔레트 필터가 알아서 걸러낸다.
        clr_mm = cfg["heightmap"]["gripper_clearance_mm"]
        # 기둥 반폭도 셀 회전을 탄다 — 90도면 w 와 d 가 서로 바뀐다
        _h = np.abs(CR @ np.array([box["w_mm"] / 2.0 + clr_mm,
                                   box["d_mm"] / 2.0 + clr_mm])) / 1000.0
        corr = (place_xy[0], place_xy[1], _h[0], _h[1])

        sample.update(low=1e9, link="", at=stack_top_m, tilt=0.0, tilt_carry=0.0,
                      tilt_cmd=0.0, tilt_cmd_carry=0.0, at_wp="",
                      boxgap=1e9, boxlink="", boxstack=1e9, boxbelt=1e9)
        t, bad, miss, over = 0.0, None, 0, 0
        yaw_prev = yaw_cur[0]
        for name, face, yaw_t in wps:
            prev_q = np.asarray(S.art.get_joint_positions()[:6], float)
            q, ok = solve(face, seed_for(prev_q), yaw_t)
            if not ok:
                bad = name
                fails.append("#%d %s: IK 수렴 실패 (흡착면 %s mm)"
                             % (i, name, np.round(face * 1000)))
                break
            dt, m, ov, q_cmd = move(face_prev, face, q, stack_top_m, corr,
                                    name in CARRY_WPS, name,
                                    yaw_from=yaw_prev, yaw_to=yaw_t,
                                    arc=(name in ARC_WPS and a.transit == "arc"))
            t += dt
            miss += m
            over += ov
            # ★ `face_prev = face` 로 두면 안 된다. 경로 중간 IK 가 실패하면 팔이
            #   웨이포인트에 **도달하지 못한 채** 구간이 끝나는데, 다음 구간을 도달했다고
            #   가정한 좌표에서 시작하면 첫 스텝이 큰 점프가 되어 팔이 휙 돈다.
            #   (실측: 그 때문에 #3 의 흡착 중 툴 기울기가 179.5° 로 찍혔다.)
            #   **실제로 지령된 마지막 자세**에서 다시 잡는다.
            face_prev = face_of(q_cmd)
            yaw_prev = yaw_t
            j5_hist.append(float(np.rad2deg(np.asarray(S.art.get_joint_positions()[:6])[4])))
        yaw_cur[0] = yaw_prev
        # 박스를 놓은 뒤 빈 그리퍼로 손목을 등가각으로 되돌린다 (감김 누적 방지)
        t += unwrap_wrist()
        if over:
            fails.append("#%d: 속도한계를 넘는 지령 %d 스텝 — 이등분으로도 못 맞춘 구간이다"
                         " (손목 특이점 근처로 보라)" % (i, over))
        if miss:
            gap = float(np.linalg.norm(face_of(S.art.get_joint_positions()[:6]) - wps[-1][1]))
            fails.append("#%d: 경로 중간 IK 실패 %d 스텝 — 직선 경로가 도달범위를 벗어난다"
                         " (최종 흡착면이 목표에서 %.0f mm)" % (i, miss, gap * 1000))

        clr = (sample["low"] - stack_top_m) * 1000 if sample["low"] < 1e8 else float("nan")
        seg = j5_hist[-len(wps):] if bad is None else j5_hist[-1:]
        tc = sample["tilt_carry"]
        tcmd = sample["tilt_cmd_carry"]
        bg = sample["boxgap"] * 1000 if sample["boxgap"] < 1e8 else float("nan")
        if bad:
            verdict = "✗ %s" % bad
        elif tc > TILT_TOL_DEG:
            verdict = "✗ 기울기 %.1f°" % tc          # 박스를 쏟는다 — 제일 먼저 잡을 것
        elif not math.isnan(bg) and bg <= 0:
            verdict = "✗ 박스가 %s 파고듦 %.0fmm" % (sample["boxlink"], -bg)
        elif t > target_s:
            verdict = "⚠ %.1fs > %.1f" % (t, target_s)
        else:
            verdict = "✓"
        print("%3d %-5s (%4d,%4d) %8.0f %8.2f %9s %10s %6.1f/%-6.1f %10s  %s"
              % (i, box["name"], pl["x_mm"], pl["y_mm"], origin[2] + obstacle, t,
                 "%.0f~%.0f" % (min(seg), max(seg)),
                 "-" if math.isnan(clr) else "%.0fmm" % clr, tcmd, tc,
                 ("%s %.0fmm" % ("" if math.isnan(bg) or bg > 0 else "✗", bg)
                  if not math.isnan(bg) else "-"), verdict))
        rows.append((i, t, clr, sample["link"], min(seg), max(seg), tc, sample["tilt"],
                     tcmd, sample["at_wp"], bg, sample["boxlink"],
                     sample["boxstack"] * 1000, sample["boxbelt"] * 1000))
        obstacle = max(obstacle, top)

    # ---- 요약
    ts = [r[1] for r in rows]
    clrs = [r[2] for r in rows if not math.isnan(r[2])]
    j5 = np.array(j5_hist)
    print("\n사이클 타임   평균 %.2f s   최대 %.2f s   목표 %.1f s   (속도 %.0f%%)"
          % (np.mean(ts), np.max(ts), target_s, a.vel * 100))
    print("             속도 100%% 환산 평균 %.2f s / 최대 %.2f s — 관절속도가 %.0f%% 로 "
          "선형이라 대략치다" % (np.mean(ts) * a.vel, np.max(ts) * a.vel, a.vel * 100))
    print("             목표까지 필요한 최소 속도배율 %.2f" % (a.vel * np.max(ts) / target_s))
    print("j5           범위 %.1f ~ %.1f deg   총 이동량 %.0f deg   한계 ±360"
          % (j5.min(), j5.max(), np.abs(np.diff(j5)).sum()))
    if abs(j5).max() > 300:
        print("             ⚠ 감김이 한계(±360)의 %.0f%% 까지 갔다" % (abs(j5).max() / 3.6))
    if clrs:
        worst = min(rows, key=lambda r: r[2] if not math.isnan(r[2]) else 1e9)
        print("이송 여유     최소 %.0f mm (#%d, %s)   평균 %.0f mm"
              % (min(clrs), worst[0], worst[3], np.mean(clrs)))
        print("             기준: check_place_plan.py 의 TRAVERSE_MIN_MM = %.0f mm"
              % CP.TRAVERSE_MIN_MM)
        if min(clrs) < CP.TRAVERSE_MIN_MM:
            print("             ✗ 팔이 적재물 위 %.0f mm 까지 내려온다 — 기준 미달"
                  % min(clrs))
    tcs = [r[6] for r in rows]
    if tcs:
        worst_t = max(rows, key=lambda r: r[6])
        print("툴 기울기     흡착 중 최대 %.1f°  (#%d)   전 구간 최대 %.1f°   허용 %.1f° (가정)"
              % (max(tcs), worst_t[0], max(r[7] for r in rows), TILT_TOL_DEG))
        cmds = [r[8] for r in rows]
        print("             지령 기준 최대 %.1f°  (%s 구간)   보간 방식 = %s"
              % (max(cmds), worst_t[9] or "-", a.interp))
        if max(tcs) > TILT_TOL_DEG and max(cmds) <= TILT_TOL_DEG:
            print("             → 지령은 수평인데 실제만 기운다. **팔이 지령을 못 따라간 것**이다"
                  " — 관절 속도한계·드라이브 게인 쪽이지 IK 문제가 아니다")
        elif max(cmds) > TILT_TOL_DEG:
            print("             → 지령 자체가 기운다. IK/보간이 자세를 안 지키고 있다")
    bgs = [r[10] for r in rows if not math.isnan(r[10])]
    if bgs:
        wb = min((r for r in rows if not math.isnan(r[10])), key=lambda r: r[10])
        print("박스-팔 간격  최소 %.0f mm (#%d, %s)   평균 %.0f mm"
              % (min(bgs), wb[0], wb[11], np.mean(bgs)))
        print("             들고 있는 박스가 팔 자신에 걸리는지. self_collision=False 라"
              " **Isaac 물리는 이걸 안 잡는다**")
        if min(bgs) <= 0:
            print("             ✗ 박스가 팔을 파고든다 — 실물이면 걸려서 떨어진다")
    bs = [r[12] for r in rows if r[12] < 1e8]
    bb = [r[13] for r in rows if r[13] < 1e8]
    if bs or bb:
        print("박스 밑면 여유  (이송 구간 carry_over 만) 적재물 위 최소 %s   벨트 위 최소 %s"
              % ("%.0f mm" % min(bs) if bs else "-", "%.0f mm" % min(bb) if bb else "-"))
        print("             부딪히는 건 흡착패드가 아니라 **매달린 박스 밑면**이다"
              " (명세서 7.5 / check_place_plan.py 검사 5)")
        if (bs and min(bs) < 0) or (bb and min(bb) < 0):
            print("             ✗ 박스가 적재물 또는 벨트를 파고든다")
    for f in fails:
        print("FAIL: %s" % f)
    ok = (not fails and (not clrs or min(clrs) >= CP.TRAVERSE_MIN_MM)
          and np.max(ts) <= target_s and (not tcs or max(tcs) <= TILT_TOL_DEG)
          and (not bgs or min(bgs) > 0))
    print("\nRESULT: %s" % ("PASS" if ok else "확인 필요 — 위 항목"))
    print("※ 박스는 **시각 전용**이다 (물리·흡착 없음). 실제 접촉은 B3 이후 확인한다."
          if vis_boxes else
          "※ 박스를 띄우지 않았다. --gui 또는 --visual 로 띄운다. 실제 접촉은 B3 이후.")
    sys.stdout.flush()
    if a.hold:
        B1.hold_open(S)
    print("\n결과: %s" % tee.path)
    tee.close()
    shutil.rmtree(S.tmpdir, ignore_errors=True)
    kit.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
