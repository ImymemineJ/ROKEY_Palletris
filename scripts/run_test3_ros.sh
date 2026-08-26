#!/bin/bash
# test_3 + ROS 2 — 관제탑이 로봇 3 대를 움직이는 쪽 (트랙 B).
#
#   ./scripts/run_test3_ros.sh              A,B,C · 창 · 자동 Play
#   PL_ZONES=B ./scripts/run_test3_ros.sh   한 대만
#   PL_GUI=0   ./scripts/run_test3_ros.sh   창 없이
#
# ⚠ `run_test3.sh` 와 **다른 실행이다.** 저쪽은 데모 스크립트
#   (`demo_stack15_test1_1.py`) 로 Isaac 혼자 다 하고, 이쪽은 `sim_main.py --ros`
#   로 **ROS 2 만 열고 판단은 관제탑에 맡긴다.** 둘을 동시에 띄우면 안 된다.
#
# ★ 이 껍데기가 하는 일은 세 가지뿐이다 — 셋 다 프로세스가 뜨기 **전에** 해야 한다.
#   환경변수는 glibc·kit 이 **시작 때만** 읽으므로 sim_main 안에서 고쳐도 늦는다:
#     1) ROS 2 환경  (아래 ①②③ — 하나만 빠져도 증상이 전부 다르다)
#     2) ROS_DOMAIN_ID=115  (팀 전원 동일. 틀리면 노드는 다 살아 있는데 서로 안 보인다)
#     3) PL_CONFIG_OVERLAY  (없으면 test1-1 좌표로 움직인다. 경고도 안 뜬다)
set -e
HERE="$(dirname "$(readlink -f "$0")")"
WS="$(dirname "$HERE")"

ISAAC_DIR="${ISAAC_DIR:-$HOME/isaac-sim-5.1.0}"
[ -d "$ISAAC_DIR" ] || ISAAC_DIR="$HOME/isaacsim"

# ── ROS 2 환경 ──────────────────────────────────────────────────────────────
# ★★ **시스템 ROS 2 를 쓴다.** Isaac 에는 내장 jazzy 가 딸려 오지만, 우리가
#   colcon 으로 빌드한 `packing_msgs` 는 `/opt/ros/jazzy` 것에 링크돼 있다.
#   내장 쪽에 물리면 **fastcdr ABI 가 달라서** 이렇게 죽는다 (2026-08-26 실측):
#
#     symbol lookup error: libpacking_msgs__rosidl_typesupport_fastrtps_c.so:
#       undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEPc
#
#   그래서 내장 브릿지 lib 경로는 **시스템 ROS 가 없을 때만** 넣는다. 넣어 두면
#   시스템 것을 가리는 수가 있다.
#
# ⚠ 여기서 세 가지가 **동시에** 맞아야 한다. 하나만 빠져도 증상이 다 다르다:
#     ① AMENT_PREFIX_PATH  — 우리 파이썬이 `.srv` **원문**을 읽어 노드 필드를 만든다
#          없으면: FileNotFoundError … SetVelocity.srv 를 못 찾았다
#          (⚠ traceback 을 `kit.close()` 가 먹어서 종료코드 0 으로 조용히 죽는다)
#     ② LD_LIBRARY_PATH    — 브릿지 C++ 이 `libpacking_msgs__rosidl_*.so` 를 dlopen
#          없으면: "packing_msgs/srv/SwapPallet does not exist or is not available
#          in the ROS 2 environment" 가 지나가고 **몇 프레임 뒤 SEGFAULT**
#     ③ 그 .so 와 같은 ROS 배포판  — 위 fastcdr ABI
#          어긋나면: symbol lookup error
#   셋 다 `source install/setup.bash` 한 줄이 해 준다 (install/setup 이 /opt/ros 를
#   같이 물고 온다). 직접 손으로 맞추려 들지 말 것.
# ★★ 2026-08-26 수정 — **둘 다** 소스한다. 예전에는 `install/setup.bash` 하나만
#   걸고 "그게 /opt/ros 를 같이 물고 온다" 고 봤는데 **그 전제가 깨질 수 있다.**
#   colcon 을 ROS 안 잡힌 셸에서 돌리면 `install/setup.bash` 가 **언더레이 없이**
#   다시 쓰인다. 그러면 이 스크립트는 멀쩡히 돌고 Isaac 만 조용히 이렇게 된다
#   (실측 2026-08-26):
#       Using backup internal ROS2 jazzy distro
#       Could not load ... librmw_implementation.so: libament_index_cpp.so 없음
#       ROS2 Bridge startup failed
#       -> OmniGraphError: unrecognized type 'isaacsim.ros2.bridge.ROS2Context'
#   증상이 브리지가 아니라 **그래프 노드 타입**으로 나와서 원인이 안 보인다.
set +e                       # ROS setup 스크립트는 비0 종료를 흘린다
if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
    . "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi
if [ -f "$WS/install/setup.bash" ]; then
    . "$WS/install/setup.bash"
fi
set -e

# ★ 여기서 **멈춘다.** 이대로 두면 Isaac 이 11 초 뒤 엉뚱한 에러로 죽는다.
if [ -z "${ROS_DISTRO:-}" ] || [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    echo "✖ 시스템 ROS 2 가 안 잡혔다 (ROS_DISTRO='${ROS_DISTRO:-}')." >&2
    echo "  이대로 띄우면 Isaac 이 내장 배포판으로 가서 ROS2 브리지가 죽는다." >&2
    echo "  고치는 법:  source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" >&2
    exit 1
fi

if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    # 시스템 ROS 가 아예 없다 — 내장으로 간다. packing_msgs 는 못 쓴다.
    BRIDGE_LIB="$ISAAC_DIR/exts/isaacsim.ros2.bridge/${ROS_DISTRO:-jazzy}/lib"
    [ -d "$BRIDGE_LIB" ] && export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$BRIDGE_LIB"
    echo "⚠ 시스템 ROS 2 를 못 찾았다 — 내장 배포판으로 간다." >&2
    echo "  packing_msgs 타입이 안 잡혀 세그폴트가 날 수 있다." >&2
fi

_need="$WS/install/packing_msgs"
for _f in "share/packing_msgs/srv/SetVelocity.srv" \
          "lib/libpacking_msgs__rosidl_typesupport_c.so"; do
    [ -f "$_need/$_f" ] || {
        echo "⚠ packing_msgs/$_f 가 없다 — colcon build 를 먼저 돌려라." >&2
        echo "  (이대로 두면 Isaac 이 세그폴트로 죽는다)" >&2
    }
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-115}"
export PL_STAGE="${PL_STAGE:-$HOME/isaac_scenes/Collected_test_3/test_3.usd}"
export PL_CONFIG_OVERLAY="${PL_CONFIG_OVERLAY:-$WS/src/packing_bringup/config/test_3.yaml}"
export PL_ZONES="${PL_ZONES:-A,B,C}"
# ★ 픽업 카메라를 **우리가** ROS 로 쏠지. 씬(Action Graph)에서 직접 배선하기로
#   했으면 0 으로 끈다 — 발행자가 둘이면 vision 이 두 이미지를 번갈아 받는다.
export PL_CAMROS="${PL_CAMROS:-1}"

# ★ 급송 — `--ros` 경로에는 박스를 넣는 주체가 없다 (씬이 아직 안 낸다).
#   PL_SPAWN=0 으로 끌 수 있다.
export PL_SPAWN="${PL_SPAWN:-8}"       # 초 간격. 0 = 안 넣는다
export PL_SPAWN_N="${PL_SPAWN_N:-0}"   # 총 개수. 0 = 무제한
# ★ 협곡 낙하 (2026-08-27). system.yaml 의 drop_max_kg 3.5 는 로봇의 PL_DROP_KG 와
#   같아야 한다 (yaml 주석의 경고). 이걸 안 켜고 돌리면 계획은 협곡 안에 자리를
#   주는데 로봇은 낙하 없이 내려가다 옆 박스를 쓸어 친다 (B섹터 이상현상.mp4 실측).
export PL_DROP_KG="${PL_DROP_KG:-3.5}"
# ★ /clock·/tf 강제 발행 (2026-08-27). 씬의 ROS2Bridge/Clock 노드가 발행자 0 실측 —
#   PL_CAMROS=2 와 같은 사연. 씬이 진짜로 내는 게 확인되면 1(자동)로 되돌릴 것.
export PL_CLOCKTF="${PL_CLOCKTF:-2}"

ARGS=(--stage "$PL_STAGE" --zones "$PL_ZONES" --ros --autorun
      --spawn-period "$PL_SPAWN" --spawn-count "$PL_SPAWN_N")
[ "${PL_GUI:-1}" != "0" ] && ARGS+=(--gui)

echo "[run_test3_ros] 씬     $PL_STAGE"
echo "[run_test3_ros] 오버레이 $PL_CONFIG_OVERLAY"
echo "[run_test3_ros] 구역   $PL_ZONES · DOMAIN $ROS_DOMAIN_ID"
cd "$WS"
# ★ 로그 자동 저장 (2026-08-26 밤, 김병직 요청) — 터미널 스크롤이 잘려서 전체
#   로그를 못 보는 문제. 저장소 **밖**(~/palletris_logs)에 두므로 git 에 안 걸린다.
#   bringup 쪽은 ROS 가 이미 ~/.ros/log/ 에 자동 저장한다 — 여기만 채우면 된다.
LOG_DIR="$HOME/palletris_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/isaac_$(date +%Y%m%d_%H%M%S).log"
echo "[run_test3_ros] 로그   $LOG"
# ★ 파이프에 물리면 파이썬 stdout 이 4 KB 블록 버퍼가 되어 로그가 한참 뒤처진다
#   (첫 시도 실측: 실행 몇 분째인데 파일은 BOX001 스폰에서 끊겨 있었다).
#   demo_stack15 의 Tee 가 당한 것과 같은 함정 — 줄 단위로 강제한다.
export PYTHONUNBUFFERED=1
stdbuf -oL -eL "$ISAAC_DIR/python.sh" src/packing_sim/isaac/sim_main.py "${ARGS[@]}" "$@" 2>&1 | tee "$LOG"
