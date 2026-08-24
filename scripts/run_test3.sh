#!/bin/bash
# test_3.usd 데모 — 창을 띄우고 직접 Play 를 눌러 본다.
#
#   ./scripts/run_test3.sh            존 A · 박스 6 개 · 라우팅 켬
#   PL_N=12 ./scripts/run_test3.sh    박스 수 바꾸기
#   PL_ROUTE=0 ./scripts/run_test3.sh 라우팅 끄고 존 A 지선만
#   PL_AUTOPLAY=1 ./scripts/run_test3.sh  Play 를 자동으로 누르기
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
export PL_N="${PL_N:-6}"
export PL_ROUTE="${PL_ROUTE:-1}"
export PL_CAM="${PL_CAM:-0}"          # 인식은 아직 test_3 좌표에 안 맞춰 놨다
export PL_CAMVIEW="${PL_CAMVIEW:-0}"
export PL_WRISTROS="${PL_WRISTROS:-0}"
exec "$HERE/run_demo.sh" "$@"
