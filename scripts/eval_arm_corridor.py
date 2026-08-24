#!/usr/bin/env python3
"""`_corridor_ok` 의 팔 통로 판정을 **Isaac 없이** 재본다 (트랙 C).

    PYTHONPATH=src/packing_algo:src/packing_common python3 scripts/eval_arm_corridor.py

## 왜 필요한가

2026-08-24 영상(`260824error1.mp4`)에서 팔이 이미 쌓은 박스를 밀어내고 박스가
팔레트 밖으로 떨어졌다. 로그상 목표는 팔레트 로컬 (Px 565, Py 630, 바닥) —
**사방이 자기보다 높은 기둥으로 둘러싸인 협곡 바닥**이었다.

원인은 구 `_corridor_ok` 가 주변 링을 박스 **밑면**과 비교한 것이다. 그래서
  · 무해한 배치(높은 이웃 옆에 미끄러져 들어가기)를 금지해 적재율만 깎고
  · 정작 닿는 높이대(박스 윗면 위 = 팔뚝)를 못 봤다
`gripper_clearance_mm` 을 30→60 으로 올려도 협곡이 안 줄던 이유가 이것이다.

## 지표

「협곡」 = 놓은 박스 주변 120mm 안에서, **팔뚝 밑면(박스 윗면 + 그리퍼 길이)보다
높은 벽**이 4변 중 2변 이상인 배치. `_corridor_ok` 와 다른 식으로 재야 순환
논증이 안 된다 — 그래서 판정 반경(120)을 규칙의 측면 여유(60)와 일부러 다르게 뒀다.

## 한계

**이건 기하 모델이지 충돌 시뮬이 아니다.** 여기서 줄었다고 실물이 안 닿는다는
보장은 없다. 확정은 Isaac 스윕으로만 한다 — 여기서는 **빠른 기각**에만 쓴다.
"""
import copy
import random
import sys
from pathlib import Path

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parent.parent / "src" / p)
                for p in ("packing_algo", "packing_common")]

from packing_algo.heightmap_core import HeightmapPacker      # noqa: E402
from packing_common.config import load                        # noqa: E402

CFG = load()
TYPES = [(t["w_mm"], t["d_mm"], t["h_mm"]) for t in CFG["boxes"]["types"]]
PROB = np.array(CFG["boxes"]["size_distribution"], float)
# ★ robot 섹션을 뺀 cfg = arm_aware False = 구 동작. 코드를 되돌리지 않고 비교할 수 있다.
LEGACY = copy.deepcopy(CFG); LEGACY.pop("robot")
# ★ 판정 기준은 **모든 행에서 같아야** 한다. pk.arm_gap 을 쓰면 LEGACY 행은 robot
#   섹션이 없어 0 이 되어, 구 동작만 더 엄한 잣대로 재는 불공정 비교가 된다.
ARM_GAP = int(CFG["robot"]["gripper_length_mm"])


def canyon_walls(hm, i, j, wc, dc, top, r=12):
    """박스 주변 4변 중 `top` 보다 높은 벽의 개수. r=12셀=120mm."""
    W, D = hm.shape
    bands = [hm[max(0, i - r):i + wc + r, max(0, j - r):j],
             hm[max(0, i - r):i + wc + r, j + dc:min(D, j + dc + r)],
             hm[max(0, i - r):i, max(0, j - r):j + dc + r],
             hm[i + wc:min(W, i + wc + r), max(0, j - r):j + dc + r]]
    return sum(1 for b in bands if b.size and int(b.max()) > top)


def stack_depth(pk):
    cnt = np.zeros(pk.hm.shape, int)
    for pose, w, d, _h in pk.placed:
        ww, dd = (w, d) if round(pose["yaw_deg"] / 90) % 2 == 0 else (d, w)
        i = int(round((pose["x_mm"] - ww / 2) / pk.cell))
        j = int(round((pose["y_mm"] - dd / 2) / pk.cell))
        cnt[max(i, 0):min(i + ww // pk.cell, pk.W),
            max(j, 0):min(j + dd // pk.cell, pk.D)] += 1
    return int(cnt.max())


def run(cfg, n, seed, noise_mm=3, **attrs):
    pk = HeightmapPacker(cfg)
    for k, v in attrs.items():
        setattr(pk, k, v)
    rng = np.random.default_rng(seed)
    seq = [TYPES[int(i)] for i in rng.choice(len(TYPES), n, p=PROB / PROB.sum())]
    rn = random.Random(7)                       # ★ 노이즈 전용 스트림 (TS-005)
    canyon = 0
    for k, (w, d, h) in enumerate(seq):
        ok, pose, _, _ = pk.plan(f"B{k}", w, d, h)
        if not ok:
            pk.cancel(f"B{k}")
            continue
        ww, dd = (w, d) if pose["yaw_deg"] == 0 else (d, w)
        ci, cj = (pose["x_mm"] - ww // 2) // pk.cell, (pose["y_mm"] - dd // 2) // pk.cell
        if canyon_walls(pk.hm, ci, cj, ww // pk.cell, dd // pk.cell,
                        pose["z_mm"] + h + ARM_GAP) >= 2:
            canyon += 1
        act = dict(pose)
        if noise_mm:
            for ax in ("x_mm", "y_mm", "z_mm"):
                act[ax] += rn.randint(-noise_mm, noise_mm)
        pk.update(f"B{k}", w, d, h, act)
    return len(pk.placed), stack_depth(pk), pk.capacity_rate() * 100, canyon


def main():
    cases = [("구 동작 (밑면 기준 · 30mm)", LEGACY, {}),
             ("새 규칙 (윗면 기준 · 60mm)", CFG, {}),
             ("  참고: 측면 30mm", CFG, dict(arm_side=3)),
             ("  참고: 측면 120mm", CFG, dict(arm_side=12))]
    for n in (20, 45):
        print("=" * 72)
        print("%d 박스 · seed 0~19 평균 · 실측오차 ±3mm" % n)
        print("%-28s %7s %5s %9s %9s" % ("", "놓음", "층", "용량적재율", "협곡배치"))
        for tag, cfg, attrs in cases:
            a = np.array([run(cfg, n, sd, **attrs) for sd in range(20)], float).mean(axis=0)
            print("%-28s %7.1f %5.1f %8.1f%% %9.2f" % (tag, a[0], a[1], a[2], a[3]))
    print()
    print("※ 협곡 = 주변 120mm 안에서 팔뚝 밑면보다 높은 벽이 2변 이상. 기하 모델이지")
    print("  충돌 시뮬이 아니다 — 확정은 Isaac 스윕으로 한다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
