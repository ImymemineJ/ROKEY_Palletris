#!/usr/bin/env python3
"""test_3 의 **페데스탈 높이를 바꾼다** — yaml 과 씬 USD 를 한 번에 (2026-08-26 김병직).

## 왜

`test_3.yaml` 의 페데스탈 1950 에서는 **벨트 위 박스를 못 집는다.**
실측 2026-08-26 (관제탑 → Isaac 전 구간, zone B):

    [adapter] ← PICK seq=1 BOX002 (-13596, 4181, 800)
    [robot_motion] 도착 잔차 87.87 mm (허용 2.0)
                   흡착면 z 1.0999 -> 목표 1.012

XY 는 0.3 mm 로 정확한데 **Z 만 88 mm 부족**하다. 도달거리 문제가 아니다
(직선 1298 mm · 한계 1700) — 너무 가까이서 깊게 내려가야 해서 관절이 안 된다.

★ 작업공간은 베이스에 붙어 있다 — **베이스를 d 내리면 도달 z 도 정확히 d 내려간다.**
  따라서 필요한 페데스탈 <= 1950 - 88 = **1862**. 여유를 두면 1830~1850.

낮추면 팔레트 쪽도 같이 좋아진다 (빈 팔레트 최원점 1673 -> 1616, 한계 1700).

## `max_stack` 도 따라온다

`test_3.yaml` 은 `max_stack = 페데스탈 - 팔레트상면` 을 쓴다 (1950-950=1000).
그래서 페데스탈을 내리면 max_stack 도 준다 — 기본값 `auto` 가 그 계산을 한다.

⚠ 그 규칙은 **보수적인 상한**이다. test1-1 은 같은 규칙이면 750(900-150)인데
  실측으로 **1300(9단)** 을 세웠다 (`results/relief_s42.txt`). `_corridor_ok` 가
  "팔이 협곡으로 내려가야 한다"는 전제로 만든 규칙인데, 협곡 낙하
  (`heightmap.drop_relief_mm`)를 켜면 안 내려가도 되기 때문이다.
  -> 낮춘 뒤 낙하를 켜서 max_stack 을 되찾을 수 있는지는 **Isaac 으로 판정한다**
     (CLAUDE.md §10-1).

## 쓰는 법

    PYTHONPATH=/tmp/usdlib ~/isaac-sim-5.1.0/python.sh \
        scripts/set_test3_pedestal.py 1830

    # 되돌리기
    PYTHONPATH=/tmp/usdlib ~/isaac-sim-5.1.0/python.sh \
        scripts/set_test3_pedestal.py 1950

    # max_stack 은 건드리지 않기
    ... scripts/set_test3_pedestal.py 1830 --max-stack keep

pxr 설치는 `scripts/fix_usd_abs_paths.py` docstring 참조.
⚠ `sync_scene.sh` 배포 뒤에는 다시 돌려야 한다 (rsync --delete 라 덮인다).
"""
import argparse
import os
import re
import shutil
import sys

import yaml
from pxr import Gf, Usd, UsdGeom

HOME = os.path.expanduser("~")
ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
YAML = os.path.join(ROOT, "src/packing_bringup/config/test_3.yaml")
SCENE = os.path.join(HOME, "isaac_scenes/project_1/test_3.usd")
BACKUP = os.path.join(os.path.dirname(SCENE), "backup")
ZONES = ("A", "B", "C")
CYL_H = 1.2                 # 원통 자체 높이 [m]. 실제 높이 = CYL_H * scale.z


def _set_translate_z(prim, z_m):
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            cur = op.Get()
            op.Set(Gf.Vec3d(float(cur[0]), float(cur[1]), float(z_m)))
            return True
    return False


def _set_scale_z(prim, sz):
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            cur = op.Get()
            op.Set(Gf.Vec3f(float(cur[0]), float(cur[1]), float(sz)))
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pedestal_mm", type=int, help="새 페데스탈 높이 [mm] (원래 1950)")
    ap.add_argument("--max-stack", default="auto",
                    help="auto = 페데스탈-팔레트상면 · keep = 그대로 · 숫자 = 그 값")
    ap.add_argument("--dry-run", action="store_true", help="바꾸지 않고 보여만 준다")
    a = ap.parse_args()

    P = int(a.pedestal_mm)
    cfg = yaml.safe_load(open(YAML, encoding="utf-8"))
    pal_top = int(cfg["zones"]["pallet_origin_world_mm"]["A"][2])
    cur_P = int(cfg["robot"]["pedestal_height_mm"])
    cur_ms = int(cfg["pallet"]["max_stack_mm"])

    if a.max_stack == "auto":
        ms = P - pal_top
    elif a.max_stack == "keep":
        ms = cur_ms
    else:
        ms = int(a.max_stack)

    print("페데스탈  %d -> %d mm" % (cur_P, P))
    print("팔레트 상면 %d · max_stack %d -> %d  (%s)"
          % (pal_top, cur_ms, ms, a.max_stack))
    if ms + pal_top > P:
        print("  ⚠ 적재 최상단 %d 가 페데스탈 %d 를 넘는다 — 스윕 규칙 위반."
              % (ms + pal_top, P))
        print("    test1-1 은 이 상태로 1300 을 세웠지만(낙하 켬) test_3 는 미검증이다.")
    print()
    if a.dry_run:
        print("(dry-run — 아무것도 안 바꿨다)")
        return 0

    # ── ① yaml ──
    txt = open(YAML, encoding="utf-8").read()
    n1 = len(re.findall(r"^(\s*pedestal_height_mm:\s*)\d+", txt, re.M))
    txt = re.sub(r"^(\s*pedestal_height_mm:\s*)\d+", r"\g<1>%d" % P, txt, flags=re.M)
    n2 = len(re.findall(r"^(\s*max_stack_mm:\s*)\d+", txt, re.M))
    txt = re.sub(r"^(\s*max_stack_mm:\s*)\d+", r"\g<1>%d" % ms, txt, flags=re.M)
    # ── robot_base_world_mm 의 z 세 줄만 ──
    # ★★ **블록 안으로 한정한다.** `A: [-8796, 4476, 950]` 꼴이 `pallet_origin_world_mm`
    #   에도 똑같이 있어서, 파일 전체에 정규식을 걸면 **팔레트 원점의 z(950)까지
    #   페데스탈 값으로 덮어쓴다.** 2026-08-26 --dry-run 으로 잡았다.
    lines = txt.splitlines(keepends=True)
    n3, inblk = 0, False
    for i, ln in enumerate(lines):
        if re.match(r"^\s*robot_base_world_mm:\s*$", ln):
            inblk = True
            continue
        if inblk:
            m = re.match(r"^(\s+[ABC]:\s*\[\s*-?\d+,\s*-?\d+,\s*)(\d+)(\])", ln)
            if m:
                lines[i] = "%s%d%s\n" % (m.group(1), P, m.group(3))
                n3 += 1
                continue
            if ln.strip() and not ln.startswith((" ", "\t")):
                inblk = False          # 들여쓰기가 끝나면 블록 밖이다
            elif ln.strip() and not re.match(r"^\s*#", ln) and n3:
                inblk = False          # 다음 키를 만나면 끝
    txt = "".join(lines)
    open(YAML, "w", encoding="utf-8").write(txt)
    print("yaml  pedestal %d곳 · max_stack %d곳 · robot_base z %d곳" % (n1, n2, n3))
    if n3 != 3:
        print("  ★ robot_base z 를 %d 곳 바꿨다 — 3 이어야 한다. 되돌리고 확인할 것!" % n3)
        return 1

    # ── ② 씬 ──
    os.makedirs(BACKUP, exist_ok=True)
    bak = os.path.join(BACKUP, "test_3.usd.bak.pre_ped%d" % P)
    shutil.copy2(SCENE, bak)
    st = Usd.Stage.Open(SCENE)
    for z in ZONES:
        rb = st.GetPrimAtPath("/World/robot_%s" % z)
        pd = st.GetPrimAtPath("/World/Pedestal_%s" % z)
        if not rb or not pd:
            print("  !! /World/robot_%s 또는 Pedestal_%s 가 없다" % (z, z))
            return 1
        _set_translate_z(rb, P / 1000.0)
        # ★ 원통은 **중심**이 절반 높이에 있고 실제 높이는 height*scale.z 다.
        _set_translate_z(pd, P / 2000.0)
        _set_scale_z(pd, (P / 1000.0) / CYL_H)
        print("  robot_%s z=%.3f · Pedestal_%s z=%.3f scale.z=%.4f"
              % (z, P / 1000.0, z, P / 2000.0, (P / 1000.0) / CYL_H))
    st.GetRootLayer().Save()
    print("씬 저장 · 백업 %s" % bak.replace(HOME, "~"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
