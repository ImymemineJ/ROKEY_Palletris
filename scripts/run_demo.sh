#!/bin/bash
# 시연 실행의 **단축키**. 진짜 코드는 scripts/demo_stack15_test1_1.py 하나다.
#
# 이것과 아래는 같다 (ROS 환경 두 줄만 여기서 대신 잡아 준다):
#   ./scripts/run_demo.sh
#   isaac_ros && cd ~/isaacsim && \
#       ./isaac-sim.sh --exec ~/palletris_ws/scripts/demo_stack15_test1_1.py
#
# 설정은 이제 스크립트 안의 PL_MODE 가 들고 있다 (기본 demo). 여기서는
# Isaac 디렉터리로 옮겨 주는 일과, 손목 카메라를 ROS 2 로 쏘는 데 필요한
# 브릿지 .so 경로를 잡는 일만 한다 — 둘 다 프로세스가 뜨기 **전에** 해야 한다.
#
#   ./scripts/run_demo.sh              시연 (창 · 11개 · 속도 0.7 · 창유지)
#     -> 씬이 뜨면 멈춰 선다. **Play 를 눌러야 시작한다.**
#        (창 없는 실행은 자동으로 시작한다. PL_AUTOPLAY=1 로 강제 가능)
#        일시정지·Stop 은 아직 못 이어간다 — 누르면 그 실행은 거기서 끝이다.
#   PL_N=15 ./scripts/run_demo.sh      15개로 (팔레트가 한 번 갈린다)
#   PL_CAM=0 ./scripts/run_demo.sh     픽업 좌표 인식 끄기
#   PL_WRISTROS=0 ./scripts/run_demo.sh   손목 카메라 ROS 발행 끄기
#
# 화면 둘:
#   view2 (Isaac 창 안)  = 벨트 위 하향 카메라  — 자동으로 뜬다
#   손목 카메라          = 다른 터미널에서
#       ros2 run rqt_image_view rqt_image_view /wrist_cam/rgb
#
# 게이트(수치 측정)는 이 껍데기를 안 쓴다 — 전부 사양값이어야 한다:
#   cd ~/isaacsim && PL_MODE=gate ./isaac-sim.sh --no-window \
#       --exec ~/palletris_ws/scripts/demo_stack15_test1_1.py
set -e
HERE="$(dirname "$(readlink -f "$0")")"

# ★ ROS 2 브릿지 라이브러리 경로. `.bashrc` 의 isaac_ros 함수와 같은 일이다.
#   프로세스가 뜬 뒤에 os.environ 을 고쳐도 소용없어서(glibc 가 시작 때 읽는다)
#   여기, exec 전에 잡는다.
ISAAC_DIR="${ISAAC_DIR:-$HOME/isaacsim}"
BRIDGE_LIB="$ISAAC_DIR/exts/isaacsim.ros2.bridge/${ROS_DISTRO:-jazzy}/lib"
if [ -d "$BRIDGE_LIB" ] && [[ ":$LD_LIBRARY_PATH:" != *":$BRIDGE_LIB:"* ]]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_LIB"
fi

# ★ 낙하 배치 — 2026-08-25 채택 실행값 (적재율 75.5%).
#   ⚠ 이 둘만 **아직 환경변수다.** yaml 로 못 옮긴다:
#     PL_DROP_KG      packing_robot/cycle.py     모듈 import 시점에 읽는다
#     PL_CONTACT_STOP packing_sim/isaac/robot_motion.py
#   둘 다 **Isaac 프로세스 안에서** 읽히므로 여기(exec 전)에서 걸어야 한다.
#   ROS launch 에 넣어도 Isaac 에는 안 닿는다 — 다른 프로세스다.
#   ★ drop_max_kg(system.yaml) 와 PL_DROP_KG 가 **같아야 한다.** 어긋나면
#     계획이 연 자리를 로봇이 거부한다.
#   ⚠ 헤드리스 게이트처럼 이 껍데기를 안 쓰는 실행은 직접 export 해야 한다.
: "${PL_DROP_KG:=3.5}"
: "${PL_CONTACT_STOP:=1}"
export PL_DROP_KG PL_CONTACT_STOP

cd "$ISAAC_DIR"
exec ./isaac-sim.sh --exec "$HERE/demo_stack15_test1_1.py" "$@"
