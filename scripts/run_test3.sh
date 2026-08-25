#!/bin/bash
# test_3.usd 데모 — 창을 띄우고 직접 Play 를 눌러 본다.
#
#   ./scripts/run_test3.sh            존 A · 박스 6 개 · 라우팅 켬
#   PL_N=33 ./scripts/run_test3.sh        박스 수 바꾸기
#   PL_PER_PALLET=0 ./scripts/run_test3.sh 반출 없이 계속 쌓기 (알고리즘 붙인 뒤)
#   PL_ROUTE=0 ./scripts/run_test3.sh      라우팅 끄고 존 A 지선만
#   PL_AUTOPLAY=1 ./scripts/run_test3.sh   Play 를 자동으로 누르기
#   PL_ZONE=B ./scripts/run_test3.sh       다른 로봇으로
#
# ★ 팔레트 좌표·픽업 축 같은 test_3 전용 값은 config/test_3.yaml 이 얹는다.
#   system.yaml 은 그대로 두므로 test1-1 기준선은 영향받지 않는다.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
export PL_STAGE="${PL_STAGE:-/home/rokey/isaac_scenes/project_1/test_3.usd}"
export PL_CONFIG_OVERLAY="${PL_CONFIG_OVERLAY:-src/packing_bringup/config/test_3.yaml}"
export PL_ZONE="${PL_ZONE:-A}"
export PL_ROBOT_PRIM="${PL_ROBOT_PRIM:-/World/robot_$PL_ZONE}"
export PL_STOPPER_PRIM="${PL_STOPPER_PRIM:-/World/Stopper_$PL_ZONE}"
export PL_MODE="${PL_MODE:-demo}"
export PL_N="${PL_N:-9}"        # 배송지 3 곳이라 한 존에 약 3 개가 간다
# ★ 팔레트에 이만큼 놓으면 **트럭으로 내보내고 끝낸다.**
#   패킹 알고리즘을 아직 안 받았으므로 지금은 1 개만 놓고 반출까지 보는 것이 목표다.
#   알고리즘이 붙으면 0(반출 안 함) 또는 원하는 수로 바꾼다.
export PL_PER_PALLET="${PL_PER_PALLET:-1}"
export PL_ROUTE="${PL_ROUTE:-1}"
export PL_CAM="${PL_CAM:-1}"          # 픽업 카메라 인식 — test_3 에서 5/5 · 오차 1.5 mm 확인
export PL_CAMVIEW="${PL_CAMVIEW:-0}"
export PL_WRISTROS="${PL_WRISTROS:-0}"
exec "$HERE/run_demo.sh" "$@"
