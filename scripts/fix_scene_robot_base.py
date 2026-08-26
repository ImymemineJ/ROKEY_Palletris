#!/usr/bin/env python3
"""씬의 로봇 베이스를 `system.yaml` 정본에 맞춘다 (2026-08-26 김병직).

## 왜 필요한가

`test1-1.usd` 의 로봇이 y = 0 (이격 300)에 있는데, 정본은 이격 **500** 이다
(`system.yaml` zones.robot_base_world_mm). 어제 채택한 실행이 전부
`--standoff 500` 이었고 발표 숫자(28/29 · 80.3%)가 그 조건에서 나왔다.

`demo_stack15.py` 는 `--standoff` 로 **런타임에 씬을 덮어써서** 이 불일치를
못 봤다. 통합(`sim_main.py`)은 씬 값을 그대로 쓰므로 어제와 다른 조건으로 돈다.
`sim_main` 이 "!! 로봇 베이스가 사양과 다르다" 를 찍지만 경고일 뿐 고쳐 주지는 않는다.

## ★ 기둥도 같이 옮긴다

씬에는 `/World/Pedestal`(Cylinder)이 **따로** 있다. 로봇만 옮기면 로봇이
기둥 위에서 200 mm 벗어나 허공에 선다. 둘은 항상 같은 y 다.

## 손대지 않는 것 — `test_3.usd` (3구역)

좌표계가 아예 다르다. 로봇 x = -7.6/-13.6/-19.6 · z = 1.95 이고, 로봇과
팔레트의 **y 가 같다** — 이격이 y 가 아니라 x 방향 800 mm 다.
`system.yaml` 의 프레임과 대응이 1:1 이 아니므로 여기서 건드리지 않는다.
3구역으로 갈 때 학민이와 좌표 규약부터 맞춰야 한다.

## 쓰는 법

    PYTHONPATH=/tmp/usdlib ~/isaac-sim-5.1.0/python.sh scripts/fix_scene_robot_base.py

pxr 설치는 `scripts/fix_usd_abs_paths.py` docstring 참조.
`sync_scene.sh` 배포 뒤에는 다시 돌려야 한다 (rsync --delete 라 덮인다).
"""
import os
import shutil
import sys

import yaml
from pxr import Usd, UsdGeom, Gf

HOME = os.path.expanduser("~")
ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SCENE = os.path.join(HOME, "isaac_scenes/project_1/test1-1.usd")
BACKUP = os.path.join(os.path.dirname(SCENE), "backup")
ZONE = "B"                       # test1-1 은 B 구역 전용 씬이다
# 로봇과 항상 같은 y 를 가져야 하는 프림들
TOGETHER = ("/World/robot", "/World/Pedestal")


def main():
    cfg = yaml.safe_load(open(os.path.join(
        ROOT, "src/packing_bringup/config/system.yaml")))
    base_mm = cfg["zones"]["robot_base_world_mm"][ZONE]
    pal_mm = cfg["zones"]["pallet_origin_world_mm"][ZONE]
    want_y = base_mm[1] / 1000.0
    print("yaml 정본: 로봇 베이스 %s mm · 팔레트 원점 %s mm → 이격 %d mm"
          % (base_mm, pal_mm, pal_mm[1] - base_mm[1]))

    st = Usd.Stage.Open(SCENE)
    todo = []
    for path in TOGETHER:
        prim = st.GetPrimAtPath(path)
        if not prim:
            print("!! 프림 없음: %s" % path)
            return 1
        op = None
        for o in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if o.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op = o
        if op is None:
            print("!! translate op 없음: %s" % path)
            return 1
        cur = op.Get()
        todo.append((path, op, cur))
        print("  %-18s 지금 y=%+.3f → %+.3f" % (path, cur[1], want_y))

    if all(abs(cur[1] - want_y) < 1e-6 for _, _, cur in todo):
        print("이미 맞다 — 바꿀 것 없음")
        return 0

    os.makedirs(BACKUP, exist_ok=True)
    stamp = sys.argv[1] if len(sys.argv) > 1 else "robotbase"
    bak = os.path.join(BACKUP, "test1-1.usd.bak.pre_%s" % stamp)
    shutil.copy2(SCENE, bak)
    print("백업 %s" % bak.replace(HOME, "~"))

    for path, op, cur in todo:
        op.Set(Gf.Vec3d(float(cur[0]), float(want_y), float(cur[2])))
    st.GetRootLayer().Save()

    # 검증 — 다시 열어서 읽는다
    st2 = Usd.Stage.Open(SCENE)
    ok = True
    for path in TOGETHER:
        for o in UsdGeom.Xformable(st2.GetPrimAtPath(path)).GetOrderedXformOps():
            if o.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                got = o.Get()
                good = abs(got[1] - want_y) < 1e-6
                ok &= good
                print("  %-18s → %s  %s" % (path, got, "OK" if good else "★ 실패"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
