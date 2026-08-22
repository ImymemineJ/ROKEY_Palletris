#!/bin/bash
# test1-1.usd 를 system.yaml 좌표계로 재배치한다.
#   ./run_layout_to_spec.sh
#   PL_ZONE=A ./run_layout_to_spec.sh
#
# 로봇 베이스 y(-800) 와 팔레트 y(-1400) 는 이제 system.yaml 에 들어가 있다.
# 여기서 덮어쓰지 않는다 — 씬과 사양이 어긋나면 안 된다 (철칙 3).
# 임시로 다른 값을 보려면 PL_ROBOT_Y / PL_PALLET_Y 를 붙여서 돌린다.
#
# Isaac 의 kit python 에는 yaml 이 없고, yaml 이 있는 시스템 python3 에는 pxr 이
# 없다. 그래서 여기서 값을 뽑아 PL_SPEC(JSON) 으로 넘긴다.
set -e
HERE="$(dirname "$(readlink -f "$0")")"
SPEC_YAML="$HOME/palletris_ws/src/packing_bringup/config/system.yaml"

export PL_SPEC="$(python3 -c "
import yaml, json, sys
c = yaml.safe_load(open('$SPEC_YAML'))
# truck 은 없을 수도 있다 (팔레트를 바닥에 두는 배치). 있으면 넘긴다.
json.dump({k: c[k] for k in ('zones', 'conveyor', 'pallet', 'robot', 'truck') if k in c}, sys.stdout)
")"

USDLIB="$HOME/isaacsim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311"
export PYTHONPATH="$USDLIB"
export LD_LIBRARY_PATH="$USDLIB/bin:$LD_LIBRARY_PATH"
exec "$HOME/isaacsim/kit/python/bin/python3" "$HERE/layout_to_spec_test1_1.py" "$@"
