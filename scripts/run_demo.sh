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

# ★ PL_NOWINDOW=1 이면 창 없이 돈다. 시뮬이 실시간의 약 7 배로 가서 지표를
#   빨리 받을 수 있다. 창이 없으면 Play 를 눌러 줄 사람이 없으므로
#   PL_AUTOPLAY 를 같이 켠다 (안 켜면 스크립트가 경고하고 멈춘다).
#   ⚠ 눈으로 봐야 하는 판정에는 쓰지 말 것 — CLAUDE.md §10-3.
#     계측은 "무엇이 얼마나" 를, 화면은 "무엇이 빠졌는지" 를 답한다.
#   ⚠ 헤드리스의 사이클 시간에는 폴링 인공물이 섞인다 (§10-2).
WIN=()
if [ "${PL_NOWINDOW:-0}" = "1" ]; then
    WIN=(--no-window)
    export PL_AUTOPLAY="${PL_AUTOPLAY:-1}"
fi

# ★ 실행 로그를 파일로도 남긴다. 터미널에만 찍히면 끝난 뒤에 원인을 못 찾는다 —
#   스크롤이 잘리고, 다른 사람이 볼 수도 없다. `PL_LOGDIR` 로 자리를 바꾼다.
# ★ 배속. `PL_RATE=120` 이면 약 2 배 (기본 60).
#
#   Kit 메인 루프가 `app/runLoops/main/rateLimitFrequency` 로 초당 틱 수에 묶여
#   있다. 물리 dt 는 `World` 가 따로 잡으므로(사양 60 Hz), 이 값을 올리면
#   **같은 시뮬 시간을 더 짧은 벽시계에** 돈다. Isaac 자체 테스트도 같은 키로
#   물리 레이트를 맞춘다 (exts/isaacsim.test.collection/.../robot_helpers.py:79).
#
#   ⚠ 기계가 못 따라오면 배수가 안 나온다. 화면이 빨라 보이는 것과 실제 배수는
#     다르다 — 종료 지표의 `벽시계` 줄로 확인할 것.
#   ⚠ **사이클 시간을 재는 실행에는 쓰지 마라** (CLAUDE.md §10-2).
#     `cycle.MOTION_POLL_S` 가 벽시계 기준이라, 배속을 올리면 폴 한 번이 더 긴
#     시뮬 시간이 되어 사이클에 인공물이 늘어난다. 지표용은 기본값 그대로.
KIT=()
if [ -n "${PL_RATE:-}" ]; then
    KIT+=(--/app/runLoops/main/rateLimitFrequency="$PL_RATE")
    echo "배속: rateLimitFrequency=$PL_RATE (기본 60 -> 약 $((PL_RATE / 60)) 배)" >&2
fi
# 그 밖의 Kit 설정을 직접 줄 때. 예: PL_KIT_ARGS="--/app/window/width=1920"
if [ -n "${PL_KIT_ARGS:-}" ]; then
    # shellcheck disable=SC2206
    KIT+=($PL_KIT_ARGS)
fi

LOGDIR="${PL_LOGDIR:-$HOME/palletris_ws/results/runs}"
mkdir -p "$LOGDIR" 2>/dev/null || LOGDIR=""
cd "$ISAAC_DIR"
if [ -n "$LOGDIR" ]; then
    LOGF="$LOGDIR/run_$(date +%Y%m%d_%H%M%S).log"
    echo "로그: $LOGF" >&2
    # stdbuf: 파이프로 넘기면 블록 버퍼가 되어 진행이 안 보인다.
    exec stdbuf -oL -eL ./isaac-sim.sh "${WIN[@]}" "${KIT[@]}" \
         --exec "$HERE/demo_stack15_test1_1.py" "$@" 2>&1 | tee "$LOGF"
fi
exec ./isaac-sim.sh "${WIN[@]}" "${KIT[@]}" --exec "$HERE/demo_stack15_test1_1.py" "$@"
