#!/usr/bin/env python3
"""spawn_feeder.py 의 박스 설정이 system.yaml 과 같은지 대조한다.

왜 필요한가
    spawn_feeder.py 는 Isaac GUI 안(kit python)에서 도는데 거기엔 yaml 이 없다.
    그래서 system.yaml 을 직접 못 읽고 값을 박아 둘 수밖에 없다. 박아 둔 값은
    사양이 바뀌면 조용히 어긋난다 (실제로 5호가 480x360 오타값으로 있었다).
    이 스크립트로 어긋남을 잡는다.

    python3 check_spawn_vs_spec.py       # 시스템 python3 로 (yaml 필요)
"""

import ast
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
FEEDER = os.path.join(HERE, "spawn_feeder.py")
SPEC = os.path.expanduser(
    "~/palletris_ws/src/packing_bringup/config/system.yaml")


def literals(path, names):
    """모듈을 실행하지 않고 최상위 대입문의 리터럴만 꺼낸다."""
    tree = ast.parse(open(path).read())
    out = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for t in node.targets:
            if isinstance(t, ast.Name) and t.id in names:
                try:
                    out[t.id] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    return out


cfg = yaml.safe_load(open(SPEC))
f = literals(FEEDER, {"BOX_SPECS", "BOX_MASS", "BOX_RATIO"})
dist = cfg["boxes"]["size_distribution"]

print(f"{'':5} {'system.yaml':34} {'spawn_feeder.py':34} ")
bad = 0
for i, t in enumerate(cfg["boxes"]["types"]):
    n = t["name"]
    want = (t["w_mm"] / 1000, t["d_mm"] / 1000, t["h_mm"] / 1000,
            t["mass_g"] / 1000, dist[i])
    got = tuple(f["BOX_SPECS"][n]) + (f["BOX_MASS"][n], f["BOX_RATIO"][n])
    ok = all(abs(a - b) < 1e-9 for a, b in zip(want, got))
    bad += not ok
    print(f"{n:5} {str(want):34} {str(got):34} {'OK' if ok else '<-- 불일치'}")

print(f"\n벨트 속도 {cfg['conveyor']['belt_velocity_mps']} m/s   "
      f"벨트 상면 {cfg['conveyor']['belt_top_z_mm']} mm   "
      f"페데스탈 {cfg['robot']['pedestal_height_mm']} mm")
print(f"투입 간격 stage1 {cfg['boxes']['stage1']['spawn_interval_s']}s "
      f"x {cfg['boxes']['stage1']['total_count']}개   "
      f"stage2 {cfg['boxes']['stage2']['spawn_interval_s']}s "
      f"x {cfg['boxes']['stage2']['total_count']}개")
sys.exit(1 if bad else 0)
