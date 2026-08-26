#!/bin/bash
# test_3.usd 데모 — 창을 띄우고 직접 Play 를 눌러 본다.
#
#   ./scripts/run_test3.sh            존 A · 박스 6 개 · 라우팅 켬
#   PL_N=33 ./scripts/run_test3.sh        박스 수 바꾸기
#   PL_PER_PALLET=1 ./scripts/run_test3.sh 한 개만 놓고 반출까지만 보기
#   PL_ROUTE=0 ./scripts/run_test3.sh      라우팅 끄고 존 A 지선만
#   PL_AUTOPLAY=1 ./scripts/run_test3.sh   Play 를 자동으로 누르기
#   PL_ZONES=A ./scripts/run_test3.sh      한 대만 (기본은 A,B,C 세 대 동시)
#   PL_ZONE=B ./scripts/run_test3.sh       한 대만 돌릴 때 어느 로봇인지
#   PL_TURN_S=8 ./scripts/run_test3.sh     코너 회전을 더 느리게 (기본 4 s)
#   PL_SHIP_MPS=1.0 ./scripts/run_test3.sh 팔레트 반출 속도 (기본 0.5 m/s)
#   PL_TURN_S=8 ./scripts/run_test3.sh     커브를 더 느리게 돌기 (기본 4 s)
#   PL_METER=2 ./scripts/run_test3.sh      한 존 지선에 허용할 박스 수 (기본 1)
#
# ★ 코너에서 팔레트를 **제자리로** 90° 돌린다. 1톤 트럭 적재함 2800 에 팔레트
#   3장을 넣으려면 가로지르는 치수가 1200 이 아니라 800 이어야 한다
#   (3x800=2400, 여유 400).
# ★ 본선은 이제 **안 멈춘다.** 존이 차면 그 박스가 순환 루프를 한 바퀴 돈다.
#
# ★ 팔레트 좌표·픽업 축 같은 test_3 전용 값은 config/test_3.yaml 이 얹는다.
#   system.yaml 은 그대로 두므로 test1-1 기준선은 영향받지 않는다.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
export PL_STAGE="${PL_STAGE:-/home/rokey/isaac_scenes/project_1/test_3.usd}"
export PL_CONFIG_OVERLAY="${PL_CONFIG_OVERLAY:-src/packing_bringup/config/test_3.yaml}"
export PL_ZONE="${PL_ZONE:-A}"
# ★ 로봇 **3 대를 동시에** 돌린다. 존마다 워커·팔·팔레트가 따로다.
#   한 대만 보려면  PL_ZONES=A ./scripts/run_test3.sh
export PL_ZONES="${PL_ZONES:-A,B,C}"
export PL_ROBOT_PRIM="${PL_ROBOT_PRIM:-/World/robot_$PL_ZONE}"
export PL_STOPPER_PRIM="${PL_STOPPER_PRIM:-/World/Stopper_$PL_ZONE}"
export PL_MODE="${PL_MODE:-demo}"
# ★ 2026-08-26 12 -> 33. 반출을 끄면(PL_PER_PALLET=0) 팔레트를 채워야 적재
#   알고리즘이 보인다. 배송지 3 곳이라 한 존에 약 11 개가 간다.
export PL_N="${PL_N:-33}"
# ★ 팔레트에 이만큼 놓으면 **트럭으로 내보내고 끝낸다.** 0 = 반출 안 하고 계속 쌓기.
#   ★ 2026-08-26 기본값 1 -> **0.** 1 은 "패킹 알고리즘을 아직 안 받았으니 한 개만
#     놓고 반출까지 보자" 던 시절의 값이다. 알고리즘(HeightmapPacker)은 이미
#     붙어 있어서(ArmAwarePacker 가 상속한다) 1 로 두면 최적 적재가 한 개 놓고
#     끝나 **아예 안 보인다.** 반출만 확인할 때는 PL_PER_PALLET=1 로 되돌린다.
export PL_PER_PALLET="${PL_PER_PALLET:-0}"
export PL_ROUTE="${PL_ROUTE:-1}"
export PL_CAM="${PL_CAM:-1}"          # 픽업 카메라 인식 — test_3 에서 5/5 · 오차 1.5 mm 확인
export PL_CAMVIEW="${PL_CAMVIEW:-0}"
export PL_WRISTROS="${PL_WRISTROS:-0}"
exec "$HERE/run_demo.sh" "$@"
