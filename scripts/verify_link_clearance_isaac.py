#!/usr/bin/env python3
"""`link_2` 대 적재물 간섭 — **Isaac / Lula 판**. 책상판(issues/25)의 확인.

    ~/isaac-sim-5.1.0/python.sh scripts/verify_link_clearance_isaac.py
    ~/isaac-sim-5.1.0/python.sh scripts/verify_link_clearance_isaac.py --gui --spawn --hold
    ~/isaac-sim-5.1.0/python.sh scripts/verify_link_clearance_isaac.py --move

`verify_link_clearance.py --probe` 의 **책상 계산(scipy)** 자리에 실제 제어에 쓸
**Lula** 를 넣은 것이다. 기하(근거리 기둥 · 목표점)는 같은 함수를 import 해서
쓴다 — 두 스크립트가 서로 다른 기둥을 보고 다른 판정을 내는 일을 원천 차단한다.

## 책상판이 남긴 두 가지 가정을 여기서 깬다

| | 책상판 | 이 스크립트 |
|---|---|---|
| IK | scipy, 재시작 40회 중 **여유 최대를 고름** | Lula 가 준 **한 해** 그대로 |
| 링크 위치 | `fk.py` (URDF 원점 계산) | **씬의 링크 프림 월드 포즈** |

1. **자세 선택권.** 6축 팔은 한 점을 여러 팔꿈치 가지로 잡는다. 책상판은 그중
   적재물을 가장 잘 피하는 가지를 골라 "여유 +112 mm" 라 했다. 런타임에는 그런
   선택권이 없다 — Lula 가 주는 해 하나로 간다. `demo_stack15.py:248` 이 같은
   함정을 이미 기록했다 ("책상 +70 도 실기에선 닿는다").
2. **FK 모델.** 책상판의 여유는 `fk.py` 가 계산한 링크 위치 위에 얹혀 있다.
   여기서는 씬에서 링크 프림의 월드 포즈를 **직접 읽어** 같은 계산을 다시 한다.
   두 값이 갈리면 충돌구체 좌표계 전제(descriptor 의 center 가 URDF 링크 프레임)가
   깨진 것이다.

그래서 표에 여유가 **세 개** 나온다: `desk*`(책상 최적) · `lula`(Lula 해 + fk.py) ·
`scene`(Lula 해 + 씬 프림). 읽는 법:

  · `lula` 가 `desk*` 보다 많이 낮다 -> 가지 선택이 문제다. 책상 숫자를 못 믿는다
  · `scene` 이 `lula` 와 다르다 -> FK 모델/충돌구체 좌표계가 어긋났다
  · 셋이 비슷하다 -> issues/25 의 표를 그대로 써도 된다

## 기둥은 시각용이다 (`--spawn`)

`--spawn` 은 근거리 기둥을 큐브로 그린다. 숫자는 프림이 있든 없든 같은 AABB 로
계산한다 — 눈으로 확인하려고 그리는 것이다. `--collide` 를 주면 충돌까지 켜서
`--move` 때 실제로 밀리는지 본다 (기본 꺼짐 — 팔이 기둥을 치면 물리가 튄다).
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                            # noqa: E402
import fk as F                                           # noqa: E402
import verify_reach as VR                                # 충돌구체·페데스탈 판정  # noqa: E402
import verify_lula_ik as B1                              # Isaac 배선 재사용  # noqa: E402
import verify_link_clearance as LC                       # 기하·간섭 계산 재사용  # noqa: E402


def link_prim_paths(prim_root):
    """씬에서 링크 프림 경로를 찾는다. 이름이 URDF 링크명과 같은 프림을 쓴다.

    임포터가 계층을 어떻게 만들든(중간에 그룹이 끼든) 이름으로 찾으면 맞는다.
    못 찾은 링크는 dict 에서 빠진다 — 호출부가 그 링크를 건너뛴다.
    """
    import omni.usd
    from pxr import Usd                                   # noqa: F401
    stage = omni.usd.get_context().get_stage()
    want = set(LC.LINKS)
    out = {}
    for p in stage.Traverse():
        n = p.GetName()
        if n in want and str(p.GetPath()).startswith(prim_root):
            out.setdefault(n, str(p.GetPath()))
    return out


def scene_link_poses(paths):
    """{링크: (R 3x3, p 3)} — 씬 프림의 **월드** 포즈. 물리가 돈 뒤 값이다."""
    from isaacsim.core.utils.rotations import quat_to_rot_matrix
    from isaacsim.core.prims import SingleXFormPrim
    out = {}
    for link, path in paths.items():
        pos, quat = SingleXFormPrim(path).get_world_pose()
        out[link] = (np.asarray(quat_to_rot_matrix(np.asarray(quat))), np.asarray(pos))
    return out


def clearance_from_poses(poses, spheres, obstacles):
    """링크 월드 포즈로부터 적재물 최소여유 [m]. (전체최소, 링크, link_2 값)."""
    worst, where, l2 = None, "", float("inf")
    for link, (R, p) in poses.items():
        for c, r in spheres.get(link, []):
            w = p + R @ c
            for lo, hi in obstacles:
                d = LC.sphere_aabb_clearance(w, r, lo, hi)
                if link == "link_2" and d < l2:
                    l2 = d
                if worst is None or d < worst:
                    worst, where = d, link
    return worst, where, l2


def desk_poses(q, joints, base_m):
    """`fk.py` 로 계산한 링크 월드 포즈. 책상판과 같은 경로다."""
    T = F.fk(list(q), joints)
    return {ln: (np.asarray(T[ln][0]), base_m + np.asarray(T[ln][1])) for ln in LC.LINKS}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--probe-h", default="600,800,900,1000,1100,1200,1300",
                    help="쓸어볼 근거리 기둥 높이 [mm]")
    ap.add_argument("--probe-z", default="0",
                    help="목표의 밑면 팔레트 z [mm]. 쉼표로 여러 개")
    ap.add_argument("--probe-box", default="3호", help="놓을 박스 종류")
    ap.add_argument("--base-dy", type=float, default=0.0,
                    help="로봇 베이스를 월드 Y 로 미는 양 [mm]. ★ 씬의 베이스는 "
                         "안 옮긴다 — 감도만 볼 때는 책상판을 쓸 것")
    ap.add_argument("--desk", action="store_true",
                    help="책상 최적값(desk*)도 같이 낸다. scipy 로 40회 재시작해 느리다")
    ap.add_argument("--move", action="store_true",
                    help="점마다 실제로 지령하고 물리로 수렴시킨 뒤 잰다 (처짐 포함)")
    ap.add_argument("--ready-each", action="store_true",
                    help="점마다 READY 로 되돌린 뒤 푼다. 웜스타트 영향을 뺀다")
    ap.add_argument("--spawn", action="store_true", help="근거리 기둥을 큐브로 그린다")
    ap.add_argument("--collide", action="store_true",
                    help="--spawn 기둥에 충돌을 켠다. --move 와 같이 쓸 때만 의미 있다")
    ap.add_argument("--settle", type=float, default=6.0)
    ap.add_argument("--gui", action="store_true", help="창을 띄운다")
    ap.add_argument("--hold", action="store_true", help="끝난 뒤 창을 열어 둔다")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    _paths.check()
    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})
    # ★ Tee 는 SimulationApp 뒤에 건다 (verify_lula_ik.py 의 같은 주석 참조).
    tee = B1.start_log(a.log or _paths.ROOT / "results" /
                       ("link2_isaac_%s.txt" % a.zone))

    # 그리퍼가 붙은 판이 있으면 그걸 쓴다 — 런타임과 같은 형상이어야 한다.
    asset = B1.ASSET_SUCTION if B1.ASSET_SUCTION.exists() else None
    S = B1.bring_up(zone=a.zone, render=a.gui, asset=asset)
    cfg = S.cfg
    ped_mm = cfg["robot"]["pedestal_height_mm"]
    joints = F.load_joints()
    spheres = VR.load_spheres()
    hs = [float(v) for v in a.probe_h.split(",")]
    tzs = [float(v) for v in a.probe_z.split(",")]

    origin_mm, base_m = LC.geometry(cfg, a.zone, a.base_dy, ped_mm)
    paths = link_prim_paths(B1.PRIM)
    missing = [ln for ln in LC.LINKS if ln not in paths]

    print("ASSET   %s" % (asset or B1.ASSET))
    print("ZONE    %s   base world %s mm   pedestal %s mm   gripper %.0f mm"
          % (a.zone, list(np.round(S.base * 1000).astype(int)), ped_mm, S.grip * 1000))
    print("링크프림 %s" % ({k: paths[k] for k in sorted(paths)} or "못 찾음"))
    if missing:
        print("  ⚠ 씬에서 못 찾은 링크 %s — scene 열에서 빠진다" % missing)
    if a.base_dy:
        print("  ⚠ --base-dy %+.0f 은 **계산상 베이스**만 옮긴다. 씬의 로봇은 그대로다."
              "\n     Isaac 에서 이격 감도를 보려면 씬을 옮겨야 한다 — 여기서는 무의미하다."
              % a.base_dy)
    print("WARM    %s" % ("점마다 READY 리셋" if a.ready_each
                          else "직전 해에서 이어 푼다 (런타임과 같은 조건)"))
    print("측정    desk* = 책상 최적(가지 선택) · lula = Lula 해 + fk.py "
          "· scene = Lula 해 + 씬 프림\n")

    hdr = ("%6s %6s | %8s %8s %8s | %9s %7s %6s  판정"
           % ("기둥H", "tgt_z", "desk*", "lula", "scene", "전체최소", "링크", "IKms"))
    print(hdr)
    print("-" * len(hdr))

    B1.teleport(S, S.ready, steps=int(0.5 * S.hz))
    agree = True
    rows = []
    for tz in tzs:
        for hv in hs:
            near = LC.near_column_aabb(cfg, a.zone, origin_mm, hv, a.probe_box)
            sx, sy, sz = LC.far_target_suction(cfg, a.zone, origin_mm, tz, a.probe_box)
            if a.spawn:
                spawn_column(near, hv, tz, a.collide)
            if a.ready_each:
                B1.teleport(S, S.ready)

            suction = np.array([sx, sy, sz])
            target = suction + np.array([0.0, 0.0, S.grip])   # tool0 = 흡착면 + 그리퍼
            ok, q, err, down, ms, _ms_max = B1.solve(S, target)
            if not ok:
                print("%6.0f %6.0f | %8s %8s %8s | %9s %7s %6.1f  ✗ IK 미수렴"
                      % (hv, tz, "-", "-", "-", "-", "", ms))
                rows.append((hv, tz, None))
                continue

            # Lula 해 + fk.py
            l_worst, l_where, l_l2 = clearance_from_poses(
                desk_poses(q, joints, base_m), spheres, [near])
            # Lula 해 + 씬 프림 — 관절을 실제로 옮겨야 프림이 움직인다
            if a.move:
                B1.goto(S, q, secs=a.settle)
            else:
                B1.teleport(S, q)
            s_worst, s_where, s_l2 = clearance_from_poses(
                scene_link_poses(paths), spheres, [near]) if paths else (
                    float("nan"), "", float("nan"))

            d_l2 = float("nan")
            if a.desk:
                v, _ = LC.probe(cfg, a.zone, a.base_dy, ped_mm, S.grip, hv, 40,
                                a.probe_box, target_z_mm=tz)
                d_l2 = v if v is not None else float("nan")

            # 판정 — 실기 기준은 scene 이다. 없으면 lula.
            ref = s_l2 if paths else l_l2
            mark = "✓" if ref > 0 else "!! 닿는다"
            gap = abs(s_l2 - l_l2) * 1000 if paths else 0.0
            if gap > 20.0:
                mark += "  ⚠fk불일치 %.0fmm" % gap
                agree = False
            print("%6.0f %6.0f | %8s %8.1f %8s | %+8.1f %7s %6.1f  %s"
                  % (hv, tz,
                     "%+.1f" % (d_l2 * 1) if a.desk and d_l2 == d_l2 else "-",
                     l_l2 * 1000,
                     "%+.1f" % (s_l2 * 1000) if paths else "-",
                     (s_worst if paths else l_worst) * 1000,
                     s_where if paths else l_where, ms, mark))
            rows.append((hv, tz, ref))
            if not a.ready_each:
                B1.teleport(S, q)      # 다음 점이 이 해를 웜스타트로 물게 한다

    # 한계 — 여유가 양수인 가장 높은 기둥
    print()
    for tz in tzs:
        lim = None
        for hv, t, ref in rows:
            if t == tz and ref is not None and ref > 0:
                lim = hv
        print("tgt_z %4.0f  ->  Isaac 한계 %s mm   (책상 어림 lim = ped-150 = %.0f)"
              % (tz, "%.0f" % lim if lim is not None else "없음", ped_mm - 150))
    if not agree:
        print("\n⚠ fk.py 와 씬 프림이 20mm 넘게 갈린 점이 있다 — 충돌구체 좌표계 전제를"
              "\n  다시 봐야 한다. 그 전에는 책상판 숫자(issues/25)를 못 쓴다.")
    print("\n※ 절대값은 이 표가 정본이다 (책상판은 가지를 골라서 낙관적이다).")
    if a.hold:
        B1.hold_open(S)
    tee.close()          # ★ kit.close() 전에. 안 닫으면 로그 파일이 빈다
    kit.close()
    return 0


def spawn_column(near, h_mm, tz_mm, collide):
    """근거리 기둥을 큐브로 그린다. **숫자에는 영향이 없다** — 눈으로 볼 용도다."""
    from isaacsim.core.api.objects import FixedCuboid, VisualCuboid
    lo, hi = near
    size = hi - lo
    center = (lo + hi) / 2.0
    path = "/World/NearColumn_%d_%d" % (int(h_mm), int(tz_mm))
    import omni.usd
    if omni.usd.get_context().get_stage().GetPrimAtPath(path):
        return
    cls = FixedCuboid if collide else VisualCuboid
    cls(prim_path=path, name=path.split("/")[-1], position=center,
        scale=size, color=np.array([0.85, 0.35, 0.15]))


if __name__ == "__main__":
    sys.exit(main())
