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

# ★ `export VAR="$(...)"` 로 쓰면 export 의 종료코드가 0 이라 set -e 가 yaml 실패를
#   못 잡는다. system.yaml 이 깨져도 그대로 진행해 엉뚱한 JSONDecodeError 로 끝난다
#   (2026-08-22 충돌 마커가 커밋됐을 때 실제로 그랬다). 그래서 대입과 export 를 나눈다.
PL_SPEC="$(python3 -c "
import yaml, json, sys
c = yaml.safe_load(open('$SPEC_YAML'))
# truck 은 씬에 야드 지면을 그릴 때만 쓴다. 높이 계산에는 안 들어간다 (작업명세서 §3.1).
json.dump({k: c[k] for k in ('zones', 'conveyor', 'pallet', 'robot', 'truck', 'boxes', 'physics', 'recognition') if k in c}, sys.stdout)
")"
export PL_SPEC

USDLIB="$HOME/isaacsim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311"
export PYTHONPATH="$USDLIB"
export LD_LIBRARY_PATH="$USDLIB/bin:$LD_LIBRARY_PATH"
exec "$HOME/isaacsim/kit/python/bin/python3" "$HERE/layout_to_spec_test1_1.py" "$@"
