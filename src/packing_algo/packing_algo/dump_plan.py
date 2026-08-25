"""완화된 안전값으로 계획을 뽑아 **Isaac 검증기에 먹일 JSON** 으로 저장한다.

## 왜 필요한가

적재율을 더 올리는 가장 큰 레버 두 개가 **알고리즘이 보수적으로 잡아 둔 안전값**이다:

    min_support_ratio  0.7    박스 바닥의 몇 %가 받쳐져야 하나
    통로 여유          등방 12셀  사방 120mm 에 팔이 지나갈 여유

둘 다 데스크 근거뿐이고 **실기에서 확인된 값이 아니다.** 풀면 적재율이 최대 +9.2%p
오르지만, 넘어지거나 부딪히면 못 쓴다. 판정은 Isaac 만 할 수 있다 (TS-027 의 교훈).

그래서 이 도구는 **"완화한 설정으로 뽑은 계획"** 을 파일로 내보낸다.
트랙 B 가 그대로 `verify_transit_sweep_isaac.py` 에 먹여 궤적 간섭을 보고,
Isaac 에서 실제로 쌓아 넘어지는지 보면 된다.

    ros2 run packing_algo dump_plan --support 0.65 --side 6 -o /tmp/plan_relaxed.json
    ros2 run packing_algo dump_plan -o /tmp/plan_current.json          # 현행 (대조군)

    # 트랙 B 쪽에서
    python3 scripts/verify_transit_sweep_isaac.py /tmp/plan_relaxed.json
"""
import argparse
import copy
import json
import random

import numpy as np

from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load


def make_packer(cfg, near, far, side):
    """통로 여유를 방향별로 준 패커. 기본(12/12/12)이면 현행과 같다."""
    class Directional(HeightmapPacker):
        def _corridor_ok(self, wc, dc, bz, h_mm):
            pad = np.pad(self.hm, ((side, side), (near, far)),
                         mode="constant", constant_values=0)
            big = self._sliding_max(self._sliding_max(pad, wc + 2 * side, 0),
                                    dc + near + far, 1)
            return big <= bz + h_mm + self.arm_gap + self.tol
    return Directional(cfg)


def build(cfg, near, far, side, seed, cap=200, dry=20):
    """포화까지 쌓고 계획 목록을 돌려준다."""
    types = {(b["w_mm"], b["d_mm"], b["h_mm"]): b["name"] for b in cfg["boxes"]["types"]}
    boxes = list(types)
    pk = make_packer(cfg, near, far, side)
    rb = random.Random(seed)
    plan, d0 = [], 0
    for k in range(cap):
        w, d, h = rb.choice(boxes)
        bid = f"B{k:03d}"
        ok, pose, _, msg = pk.plan(bid, w, d, h)
        if not ok:
            pk.cancel(bid)
            d0 += 1
            if msg == "PALLET_FULL" or d0 >= dry:
                break
            continue
        d0 = 0
        pk.update(bid, w, d, h, dict(pose))
        # ★ verify_transit_sweep_isaac.py 가 읽는 형식 — box_type 은 이름이어야 한다
        plan.append(dict(box_id=bid, box_type=types[(w, d, h)], **pose))
    return pk, plan


def main():
    ap = argparse.ArgumentParser(description="완화 설정 계획을 Isaac 검증용 JSON 으로")
    ap.add_argument("--support", type=float, default=None,
                    help="min_support_ratio (기본: system.yaml 값 0.7)")
    ap.add_argument("--near", type=int, default=12, help="통로 여유 — 로봇 쪽 (셀)")
    ap.add_argument("--far", type=int, default=12, help="통로 여유 — 먼 쪽 (셀)")
    ap.add_argument("--side", type=int, default=12, help="통로 여유 — 양옆 (셀)")
    ap.add_argument("--seed", type=int, default=3, help="박스 순서 시드")
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()

    cfg = copy.deepcopy(load())
    if args.support is not None:
        cfg["heightmap"]["min_support_ratio"] = args.support
    pk, plan = build(cfg, args.near, args.far, args.side, args.seed)

    with open(args.out, "w") as f:
        json.dump(plan, f, ensure_ascii=False, indent=1)

    sup = cfg["heightmap"]["min_support_ratio"]
    print(f"설정: min_support {sup} · 통로 {args.near}/{args.far}/{args.side} 셀 · 시드 {args.seed}")
    print(f"결과: {len(plan)}개 · 적재율 {pk.fill_rate()*100:.1f}% · 최고 {int(pk.hm.max())}mm")
    print(f"저장: {args.out}")
    print()
    print("트랙 B 쪽에서 확인:")
    print(f"  python3 scripts/verify_transit_sweep_isaac.py {args.out}")


if __name__ == "__main__":
    main()
