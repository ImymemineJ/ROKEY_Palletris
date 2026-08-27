#!/usr/bin/env python3
"""test_3 기하를 **test1-1 에서 실측 검증된 관계**로 맞춘다 (2026-08-26 김병직).

## 왜

`test_3.yaml` 은 `max_stack = 페데스탈 - 팔레트상면` 이라는 **스윕 규칙**으로 기하를
정했다 (1950 - 950 = 1000). 그런데 그 규칙은 test1-1 에서 **이미 깨졌다**:

    test1-1: 페데스탈 900 · 팔레트상면 150 -> 규칙상 750
             실제로 세운 것 **1300 (9단)**  — results/relief_s42.txt
             협곡 낙하(heightmap.drop_relief_mm)가 뚫었다. 팔이 협곡으로
             안 내려가도 되니 `_corridor_ok` 의 전제가 사라진다.

규칙보다 **실측이 우선한다** (CLAUDE.md §10-1 — 계획 파라미터는 Isaac 으로 판정한다).

## 옮기는 관계

배치 기하는 두 씬이 이미 같다 — 이격축 팔레트폭 800 · 측면 600 · 로봇 중앙.
**다른 것은 팔레트 높이 하나뿐**이다 (바닥 150 vs 셀 컨베이어 950).
그래서 로봇도 그만큼 올리면 같은 조건이 된다:

    베이스 - 팔레트상면(dz)   test1-1  900 - 150 = 750
                              test_3   ??? - 950 = 750  ->  페데스탈 1700

    빈 팔레트 최원점 (도달 1700)
      test1-1 이격 500 · dz 750   1616.3  여유 83.7
      test_3  이격 500 · dz 750   1616.3  **완전히 동일**
      test_3  이격 500 · dz 1000  1746.4  초과 46.4   <- 페데스탈 1950 이 문제였다

## ★ 이격은 X 축이다 (test1-1 은 Y)

test_3 는 팔레트가 로봇의 **-X 쪽**이라 이격이 X 방향이다.
그리고 **로봇을 옮기면 안 된다** — 지선(픽업)이 로봇 X 에 정렬돼 있다
(setup_test3_scene 검증: "지선 중심 = 로봇 X  OK -19596.0 vs -19596.0").
-> **팔레트를 -X 로** 옮겨 이격을 벌린다.

## 쓰는 법

    PYTHONPATH=/tmp/usdlib ~/isaac-sim-5.1.0/python.sh \
        scripts/set_test3_geometry.py --pedestal 1700 --standoff 500 --max-stack 1300

    # 되돌리기 (씬 기본값)
    ... --pedestal 1950 --standoff 400 --max-stack 1000

    --dry-run 이면 바꾸지 않고 계산만 보여준다.

pxr 설치는 `scripts/fix_usd_abs_paths.py` docstring 참조.

⚠ 팔레트를 옮기면 **가이드 벽이 어긋난다** ([10]·[12] 가 팔레트 기준으로 세운다).
  이 스크립트 뒤에 `setup_test3_scene.py` 를 한 번 더 돌려 다시 세울 것.
⚠ `sync_scene.sh` 배포 뒤에도 다시 돌려야 한다 (rsync --delete 라 덮인다).
"""
import argparse
import math
import os
import re
import shutil
import sys

import yaml
from pxr import Gf, Usd, UsdGeom

HOME = os.path.expanduser("~")
ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
YAML = os.path.join(ROOT, "src/packing_bringup/config/test_3.yaml")
SCENE = os.path.join(HOME, "isaac_scenes/Collected_test_3/test_3.usd")
BACKUP = os.path.join(os.path.dirname(SCENE), "backup")
ZONES = ("A", "B", "C")
CYL_H = 1.2                 # 기둥 원통의 자체 높이 [m]. 실제 높이 = CYL_H * scale.z
REACH_MM = 1700.0           # H2017 도달거리


def _op(prim, kind):
    for o in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if o.GetOpType() == kind:
            return o
    return None


def _block_sub(lines, key, col, value):
    """`key:` 블록 안의 `A/B/C: [x, y, z]` 에서 col 번째 숫자만 바꾼다.

    ★ 블록 한정이 필수다. `A: [-8796, 4476, 950]` 꼴이 `pallet_origin_world_mm` 과
      `robot_base_world_mm` 양쪽에 있어서, 파일 전체에 정규식을 걸면 엉뚱한 쪽을
      덮어쓴다 (2026-08-26 --dry-run 으로 잡았다).
    """
    n, inblk = 0, False
    pat = re.compile(r"^(\s+[ABC]:\s*\[\s*)(-?\d+)(,\s*)(-?\d+)(,\s*)(-?\d+)(\s*\])")
    for i, ln in enumerate(lines):
        if re.match(r"^\s*%s:\s*$" % re.escape(key), ln):
            inblk = True
            continue
        if not inblk:
            continue
        m = pat.match(ln)
        if m:
            g = list(m.groups())
            g[col * 2 + 1] = str(int(value(int(g[col * 2 + 1]))))
            lines[i] = "".join(g) + "\n"
            n += 1
            continue
        if ln.strip() and not re.match(r"^\s*#", ln):
            inblk = False          # 블록 안의 다음 키를 만나면 끝
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pedestal", type=int, required=True, help="페데스탈 높이 [mm]")
    ap.add_argument("--standoff", type=int, required=True,
                    help="팔레트 로봇쪽 모서리 ~ 로봇 X 이격 [mm]")
    ap.add_argument("--max-stack", type=int, default=None,
                    help="안 주면 그대로 둔다")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    cfg = yaml.safe_load(open(YAML, encoding="utf-8"))
    pal_top = int(cfg["zones"]["pallet_origin_world_mm"]["A"][2])
    pal_w = int(cfg["pallet"]["width_mm"])
    pal_d = int(cfg["pallet"]["depth_mm"])
    cur_ped = int(cfg["robot"]["pedestal_height_mm"])
    cur_ms = int(cfg["pallet"]["max_stack_mm"])
    ms = cur_ms if a.max_stack is None else int(a.max_stack)

    # 현재 이격 (로봇 X - 팔레트 X최대)
    rb = cfg["zones"]["robot_base_world_mm"]["A"]
    po = cfg["zones"]["pallet_origin_world_mm"]["A"]
    cur_gap = rb[0] - (po[0] + pal_w)
    # 팔레트를 X 로 이 만큼 옮긴다 (음수면 -X). 로봇이 팔레트의 **+X 쪽**이므로
    # 이격을 늘리려면 팔레트가 -X 로 가야 한다.
    #   이격 = 로봇X - (팔레트원점X + 폭)   ->   원점X_new = 원점X + (이격_cur - 이격_new)
    # ★ 2026-08-26 --dry-run 으로 부호 오류를 잡았다. 반대로 하면 이격이 줄어든다.
    pal_dx = cur_gap - a.standoff

    dz = a.pedestal - pal_top
    far = math.hypot(math.hypot(a.standoff + pal_w, pal_d / 2.0), dz)

    print("페데스탈   %d -> %d   (베이스 - 팔레트상면 = %d)" % (cur_ped, a.pedestal, dz))
    print("이격(X)    %d -> %d   (팔레트를 X %+d mm)" % (cur_gap, a.standoff, pal_dx))
    print("max_stack  %d -> %d" % (cur_ms, ms))
    print()
    print("빈 팔레트 최원점 %.1f mm / 도달 %.0f  -> %s"
          % (far, REACH_MM,
             "여유 %.1f" % (REACH_MM - far) if far <= REACH_MM
             else "★ 초과 %.1f — 멈춘다" % (far - REACH_MM)))
    if far > REACH_MM and not a.dry_run:
        print("  도달범위를 넘는다. --dry-run 으로 조합을 먼저 확인할 것.")
        return 1
    print("적재 최상단 %d (팔레트상면 %d + %d) · 트럭 내고 여유는 별도 확인"
          % (pal_top + ms, pal_top, ms))
    if pal_top + ms > a.pedestal:
        print("  ⓘ 스윕 규칙(기둥윗면 <= 페데스탈)은 넘는다 — test1-1 이 같은 상태로"
              " 9단을 세웠다(낙하). Isaac 으로 판정할 것.")
    print()
    if a.dry_run:
        print("(dry-run — 아무것도 안 바꿨다)")
        return 0

    # ── yaml ──
    txt = open(YAML, encoding="utf-8").read()
    n_ped = len(re.findall(r"^(\s*pedestal_height_mm:\s*)\d+", txt, re.M))
    txt = re.sub(r"^(\s*pedestal_height_mm:\s*)\d+",
                 r"\g<1>%d" % a.pedestal, txt, flags=re.M)
    n_ms = len(re.findall(r"^(\s*max_stack_mm:\s*)\d+", txt, re.M))
    txt = re.sub(r"^(\s*max_stack_mm:\s*)\d+", r"\g<1>%d" % ms, txt, flags=re.M)

    lines = txt.splitlines(keepends=True)
    n_rb = _block_sub(lines, "robot_base_world_mm", 2, lambda _v: a.pedestal)
    n_po = _block_sub(lines, "pallet_origin_world_mm", 0, lambda v: v + pal_dx)
    if (n_rb, n_po) != (3, 3):
        print("★ yaml 치환이 robot_base %d · pallet_origin %d 곳 — 둘 다 3 이어야 한다."
              % (n_rb, n_po))
        return 1
    open(YAML, "w", encoding="utf-8").write("".join(lines))
    print("yaml  페데스탈 %d곳 · max_stack %d곳 · robot_base z 3곳 · pallet_origin x 3곳"
          % (n_ped, n_ms))

    # ── 씬 ──
    os.makedirs(BACKUP, exist_ok=True)
    bak = os.path.join(BACKUP, "test_3.usd.bak.pre_ped%d_gap%d" % (a.pedestal, a.standoff))
    shutil.copy2(SCENE, bak)
    st = Usd.Stage.Open(SCENE)
    for z in ZONES:
        rb_p = st.GetPrimAtPath("/World/robot_%s" % z)
        pd_p = st.GetPrimAtPath("/World/Pedestal_%s" % z)
        pl_p = st.GetPrimAtPath("/World/Pallet_%s" % z)
        if not (rb_p and pd_p and pl_p):
            print("  !! robot_%s / Pedestal_%s / Pallet_%s 중 없는 것이 있다" % (z, z, z))
            return 1
        o = _op(rb_p, UsdGeom.XformOp.TypeTranslate)
        c = o.Get(); o.Set(Gf.Vec3d(float(c[0]), float(c[1]), a.pedestal / 1000.0))
        # ★ 기둥은 원통이고 **중심이 절반 높이**다. 로봇만 옮기면 허공에 선다.
        o = _op(pd_p, UsdGeom.XformOp.TypeTranslate)
        c = o.Get(); o.Set(Gf.Vec3d(float(c[0]), float(c[1]), a.pedestal / 2000.0))
        o = _op(pd_p, UsdGeom.XformOp.TypeScale)
        c = o.Get(); o.Set(Gf.Vec3f(float(c[0]), float(c[1]),
                                    (a.pedestal / 1000.0) / CYL_H))
        # ★ 팔레트를 -X 로. 로봇은 못 옮긴다 (지선이 로봇 X 에 정렬돼 있다).
        o = _op(pl_p, UsdGeom.XformOp.TypeTranslate)
        c = o.Get(); o.Set(Gf.Vec3d(float(c[0]) + pal_dx / 1000.0, float(c[1]), float(c[2])))
        print("  %s: robot z=%.3f · Pedestal z=%.3f scale.z=%.4f · Pallet x=%.3f"
              % (z, a.pedestal / 1000.0, a.pedestal / 2000.0,
                 (a.pedestal / 1000.0) / CYL_H, float(c[0]) + pal_dx / 1000.0))
    st.GetRootLayer().Save()
    print("씬 저장 · 백업 %s" % bak.replace(HOME, "~"))
    print()
    print("⚠ 팔레트를 옮겼으니 가이드 벽을 다시 세워야 한다:")
    print("   PL_STAGE=$HOME/isaac_scenes/Collected_test_3/test_3.usd \\")
    print("   PL_CONFIG_OVERLAY=$HOME/palletris_ws/src/packing_bringup/config/test_3.yaml \\")
    print("   ~/isaac-sim-5.1.0/isaac-sim.sh --no-window --exec "
          "$HOME/palletris_ws/scripts/setup_test3_scene.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
