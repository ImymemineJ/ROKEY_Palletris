#!/bin/bash
# 시연 실행의 **단축키**. 진짜 코드는 scripts/demo_stack15_test1_1.py 하나다.
#
# 이것과 아래 둘은 완전히 같다:
#   ./scripts/run_demo.sh
#   cd ~/isaacsim && ./isaac-sim.sh --exec ~/palletris_ws/scripts/demo_stack15_test1_1.py
#
# 설정은 이제 스크립트 안의 PL_MODE 가 들고 있다 (기본 demo). 여기서는
# Isaac 디렉터리로 옮겨 주는 일만 한다 — isaac-sim.sh 는 자기 폴더에서 실행해야 한다.
#
#   ./scripts/run_demo.sh              시연 (창 · 11개 · 속도 0.7 · 창유지)
#   PL_N=15 ./scripts/run_demo.sh      15개로 (팔레트가 한 번 갈린다)
#   PL_CAM=0 ./scripts/run_demo.sh     카메라 끄기
#
# 게이트(수치 측정)는 이 껍데기를 안 쓴다 — 전부 사양값이어야 한다:
#   cd ~/isaacsim && PL_MODE=gate ./isaac-sim.sh --no-window \
#       --exec ~/palletris_ws/scripts/demo_stack15_test1_1.py
set -e
HERE="$(dirname "$(readlink -f "$0")")"
cd "${ISAAC_DIR:-$HOME/isaacsim}"
exec ./isaac-sim.sh --exec "$HERE/demo_stack15_test1_1.py" "$@"
