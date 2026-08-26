#!/usr/bin/env python3
"""씬 USD 의 **절대경로 참조를 상대경로로** 바꾼다 (2026-08-26 김병직).

## 왜 필요한가

씬 3개가 로봇을 `/home/rokey/isaac_assets/h2017_suction.usd` 로 가리켰다.
**사용자명이 다른 장비에서 그대로 깨진다** — 씬은 열리는데 로봇만 없고,
`Could not open asset` 경고 한 줄이 수백 줄 로그에 묻힌다.
`scripts/sync_scene.sh` 가 스스로 경고하는 바로 그 실수다:

    /home/rokey/... 처럼 절대경로로 박으면 사용자명이 다른 장비에서 깨진다

작업 PC 와 발표용 GUI 노트북이 **다른 장비**라 반드시 고쳐야 한다.

## 전제

`~/isaac_scenes/` 와 `~/isaac_assets/` 가 **$HOME 아래 형제 디렉토리**다.
그래서 `~/isaac_scenes/project_1/x.usd` -> `../../isaac_assets/...` 로 닿는다.
아니면 이 스크립트가 마지막에 "★ 없음" 을 찍는다.

## ⚠ 다시 돌려야 하는 때

`sync_scene.sh` 는 `rsync --delete` 로 배포한다 — **학민이가 배포하면 이 수정이
통째로 덮인다.** 배포받은 뒤 한 번 더 돌리면 된다.
근본 해결은 학민이 원본에서 같은 수정을 하는 것이다.

## 쓰는 법

    ~/isaac-sim-5.1.0/python.sh scripts/fix_usd_abs_paths.py

pxr 가 없다는 오류가 나면 (Isaac 5.1 에는 standalone pxr 이 없다):

    ~/isaac-sim-5.1.0/python.sh -m pip install --target=/tmp/usdlib usd-core
    PYTHONPATH=/tmp/usdlib ~/isaac-sim-5.1.0/python.sh scripts/fix_usd_abs_paths.py

★ `--target` 으로 따로 깔아 `PYTHONPATH` 로만 쓴다. Isaac 런타임의 USD 를
  건드리면 안 된다.

## 실측 (2026-08-26)

  절대경로 3건 (test1-1 · test_3 · test_3_v1, 전부 같은 로봇 USD) -> 전부 교체
  고친 뒤 Isaac 로드: `Could not open asset` **0건**, 로봇 정상 (READY 수렴 확인)
  pxr 26.8 로 저장한 usdc 를 Isaac 5.1 이 문제없이 읽는다
"""
import os
import shutil
import sys
from pxr import Sdf

HOME = os.path.expanduser("~")
SCENES = os.path.join(HOME, "isaac_scenes/project_1")
BACKUP = os.path.join(SCENES, "backup")
OLD = "/home/rokey/isaac_assets/h2017_suction.usd"
NEW = "../../isaac_assets/h2017_suction.usd"
STAMP = sys.argv[1] if len(sys.argv) > 1 else "relpath"

os.makedirs(BACKUP, exist_ok=True)
changed = []
for name in ("test1-1.usd", "test_3.usd", "test_3_v1.usd"):
    path = os.path.join(SCENES, name)
    if not os.path.exists(path):
        print("건너뜀 (없음):", name); continue
    lyr = Sdf.Layer.FindOrOpen(path)
    deps = list(lyr.GetCompositionAssetDependencies())
    if OLD not in deps:
        print("건너뜀 (해당 참조 없음):", name, deps); continue

    bak = os.path.join(BACKUP, "%s.bak.pre_%s" % (name, STAMP))
    shutil.copy2(path, bak)

    lyr.UpdateCompositionAssetDependency(OLD, NEW)
    lyr.Save()
    after = list(Sdf.Layer.FindOrOpen(path).GetCompositionAssetDependencies())
    ok = NEW in after and OLD not in after
    print("%-14s %s   %s -> %s" % (name, "OK " if ok else "실패", OLD, NEW))
    print("               백업 %s" % bak.replace(HOME, "~"))
    changed.append(name)

print()
print("바꾼 파일 %d 개" % len(changed))
# 해석되는지 확인
target = os.path.normpath(os.path.join(SCENES, NEW))
print("해석 결과 %s  %s" % (target.replace(HOME, "~"),
                          "존재함 ✅" if os.path.exists(target) else "★ 없음"))
