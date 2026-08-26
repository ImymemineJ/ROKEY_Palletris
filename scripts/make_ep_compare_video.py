#!/usr/bin/env python3
"""제안 알고리즘 vs Extreme Point (EP) 3D 비교 비디오 생성 스크립트.

실행:
    python3 scripts/make_ep_compare_video.py
    python3 scripts/make_ep_compare_video.py -o docs/발표/05_compare_ep.mp4 --fps 2
"""
import argparse
import copy
from pathlib import Path

import _paths
_paths.check()

from packing_common.config import load
from packing_algo import viz
from packing_algo.extreme_point import ExtremePointPacker
from packing_algo.heightmap_core import HeightmapPacker


def main():
    ap = argparse.ArgumentParser(description="제안 알고리즘 vs EP 3D 비교 비디오 생성")
    ap.add_argument("-o", "--out", default="docs/발표/05_compare_ep.mp4", help="저장할 mp4 경로")
    ap.add_argument("--fps", type=int, default=2, help="영상 FPS (기본 2)")
    ap.add_argument("--hold", type=int, default=10, help="완료 후 최종 적재율 화면 유지 프레임 수 (기본 10)")
    ap.add_argument("--boxes", type=int, default=35, help="투입할 박스 개수 (기본 35)")
    ap.add_argument("--seed", type=int, default=42, help="박스 무작위 시퀀스 난수 시드 (기본 42)")
    args = ap.parse_args()

    cfg = load()

    # ★ 제안 알고리즘 [최종 튜닝 버전 파라미터]
    cfg_final = copy.deepcopy(cfg)
    cfg_final["heightmap"]["w_level"] = 0.05
    cfg_final["heightmap"]["min_support_ratio"] = 0.65
    cfg_final["heightmap"]["plan_clearance_mm"] = 10
    cfg_final["heightmap"]["drop_relief_mm"] = 200
    cfg_final["heightmap"]["drop_max_kg"] = 3.5
    cfg_final["heightmap"]["lookahead"] = {"k": 4, "rollouts": 3, "depth": 8}

    print(f"=== 1. 시뮬레이션 데이터 계산 중 (박스 수: {args.boxes}, 시드: {args.seed}) ===")
    pk_hm, steps_hm = viz.simulate(cfg_final, n=args.boxes, seed_box=args.seed, packer_cls=HeightmapPacker)
    pk_ep, steps_ep = viz.simulate(cfg, n=args.boxes, seed_box=args.seed, packer_cls=ExtremePointPacker)

    print(f"  - 제안 알고리즘: {steps_hm[-1]['n_ok']}개 적재 / 적재율 {steps_hm[-1]['fill']*100:.1f}%")
    print(f"  - EP 알고리즘:   {steps_ep[-1]['n_ok']}개 적재 / 적재율 {steps_ep[-1]['fill']*100:.1f}%")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"=== 2. MP4 애니메이션 비디오 렌더링 중... ({out_path}) ===")
    viz.render_compare_video(
        cfg, steps_hm, steps_ep, str(out_path),
        name_a="제안 (최종 튜닝 Heightmap)",
        name_b="Extreme Point (EP 3D 극점)",
        fps=args.fps, hold=args.hold
    )
    print(f"🎉 성공적으로 비디오 생성 완료 -> {out_path}")


if __name__ == "__main__":
    main()
