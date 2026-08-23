#!/bin/bash
# 시연 한 방 — 창을 띄우고, 15박스를 라인으로 흘리고, 끝나도 창을 닫지 않는다.
#
#   ./scripts/run_demo.sh              도착 순서 (실제 라인 조건. 팔레트 2판)
#   ./scripts/run_demo.sh volume       부피 내림차순 (B5 데모와 같은 조건)
#   ./scripts/run_demo.sh volume 8     박스 8개만
#
# 게이트(헤드리스·창 없음·끝나면 종료)는 이게 아니라 아래를 쓴다:
#   cd ~/isaacsim && ./isaac-sim.sh --no-window \
#       --exec ~/palletris_ws/scripts/demo_stack15_test1_1.py
#
# ★ `isaac-sim.sh` 여야 한다. 벨트가 `IsaacConveyor` OmniGraph 노드로 도는데
#   그 노드는 렌더 틱에 물려 있어서 `python.sh` standalone 은 속도를 틀리게 돌린다
#   (근거: src/packing_sim/isaac/conveyor_ctl.py docstring).
set -e
HERE="$(dirname "$(readlink -f "$0")")"
ISAAC="${ISAAC_DIR:-$HOME/isaacsim}"

export PL_HOLD=1                         # 다 놓은 뒤 창을 닫지 않는다
export PL_ORDER="${1:-arrival}"
[ -n "$2" ] && export PL_N="$2"

echo "시연 시작 — 순서 $PL_ORDER · 박스 ${PL_N:-사양값(15)}"
echo "   씬을 다시 깔려면 먼저: $HERE/../src/packing_sim/run_layout_to_spec.sh"
echo
cd "$ISAAC"
exec ./isaac-sim.sh --exec "$HERE/demo_stack15_test1_1.py"
