#!/bin/bash
# ★ PC 한 대에서 전부 돌린다 (2026-08-27).
#
#   ./scripts/run_all_local.sh
#
# 3 대로 나눠 돌리던 것(.2 Isaac · .3 인식 · .4 관제탑)을 한 대에 합친 것이다.
# 역할 분담이 없어졌을 뿐 **띄우는 노드와 인자는 그때와 같다**:
#     .2  run_test3_ros.sh                       -> 아래 [1]
#     .3  bringup vision:=true depth:=false ...  -\
#     .4  bringup vision:=false                  -/ 합쳐서 아래 [2] 한 번
#   (.3 은 인식만 · .4 는 나머지만 켰다. 한 대면 나눌 이유가 없으므로
#    bringup 을 한 번만 띄우고 전부 켠다. 결과 노드 집합은 똑같다.)
#
# ── 한 대라서 달라지는 것 두 가지 ──────────────────────────────────────────
#  ① DDS 는 **그대로 열어 둔다** (기본 PL_NET=subnet). 노드를 한 대에 모으는
#     것과 망을 닫는 것은 별개다. fastdds_whitelist.xml 은
#     `useBuiltinTransports=false` 라 공유메모리가 꺼지는 손해가 있지만, 그건
#     3 대로 돌리던 내내 이미 감수하던 조건이고, 닫으면 팀원 PC 에서 토픽이
#     통째로 안 보인다.
#     ⚠ 그래서 기본은 **subnet** 이다 (3 대 때와 같은 설정). 노드를 한 대에
#       올리는 것과 망을 닫는 것은 다른 얘기다 — 평가 항목 §1 "다중 PC ROS 2
#       통신" 과 팀원의 live_view·rqt 가 여기에 걸린다.
#       진짜 혼자 돌릴 때만:  PL_NET=local ./scripts/run_all_local.sh
#  ② 순서를 스크립트가 지킨다. bringup 은 use_sim_time:=true 라 **/clock 발행자가
#     없으면 모든 타이머가 멎는다** (CLAUDE.md §10). 에러 한 줄 안 나고 관제탑의
#     20 초 타임아웃까지 죽는다. 그래서 Isaac 을 먼저 띄우고 /clock 을 **확인한 뒤**
#     bringup 을 올린다.
#
# 끄기: 이 터미널에서 Ctrl-C 한 번. Isaac 까지 같이 내린다.
#
#   PL_ZONES=A     ./scripts/run_all_local.sh   한 구역만 (첫 박스가 훨씬 빨리 온다)
#   PL_SPAWN=4     ./scripts/run_all_local.sh   급송 간격 (기본 8초)
#   PL_GUI=0       ./scripts/run_all_local.sh   Isaac 창 없이
#   PL_NET=local   ./scripts/run_all_local.sh   망까지 닫고 혼자 (다른 PC 안 보임)
set -e
HERE="$(dirname "$(readlink -f "$0")")"
WS="$(dirname "$HERE")"
cd "$WS"

LOG_DIR="$HOME/palletris_logs"; mkdir -p "$LOG_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
ISAAC_LOG="$LOG_DIR/isaac_$TS.log"
CTRL_LOG="$LOG_DIR/ctrl_$TS.log"

# ── ROS 환경 ────────────────────────────────────────────────────────────────
set +e
. /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
. "$WS/install/setup.bash"
set -e
[ -n "${AMENT_PREFIX_PATH:-}" ] || {
    echo "✖ ROS 2 가 안 잡혔다. colcon build --symlink-install 부터." >&2; exit 1; }

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-115}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PL_CONFIG_OVERLAY="${PL_CONFIG_OVERLAY:-$WS/src/packing_bringup/config/test_3.yaml}"

# ★ 씬 선택 (2026-08-27). 배경(full_warehouse·카트) 들어간 새 씬이
#   `Collected_test_3_share/` 로 왔다 — 팀 정본 경로(`Collected_test_3/`)가
#   아니라서 run_test3_ros.sh 기본값이 **옛 씬을 집는다.** 새 것이 있으면
#   그쪽을 쓴다. 정본 폴더가 새 씬으로 갱신되면 이 블록은 지워도 된다.
#   (검증 2026-08-27: share 쪽 83 프림 = 옛 80 + full_warehouse + 카트 2,
#    로봇 3·카메라 5·ROS2Bridge 전부 유지)
if [ -z "${PL_STAGE:-}" ] && [ -f "$HOME/isaac_scenes/Collected_test_3_share/test_3.usd" ]; then
    export PL_STAGE="$HOME/isaac_scenes/Collected_test_3_share/test_3.usd"
fi

# ★★ 기본은 **subnet** 이다 — 노드를 한 대에 다 올려도 망까지 닫으면 안 된다.
#   평가 항목 §1 이 "다중 PC ROS 2 통신" 이고, 팀원이 live_view·rqt·ros2 topic 으로
#   붙어 보는 것도 여기서 끊긴다. LOCALHOST 로 가두면 **이 PC 밖에서는 토픽이
#   아예 안 보인다** — 에러가 아니라 그냥 빈 목록으로 나와서 원인 찾기가 어렵다.
#   대가는 공유메모리 전송이 꺼지는 것(whitelist 의 useBuiltinTransports=false)뿐이고,
#   그건 3 대로 돌리던 내내 이미 감수하던 조건이다.
if [ "${PL_NET:-subnet}" = "local" ]; then
    # 혼자 돌릴 때만. 공유메모리가 살아나 이미지가 빨라지고 팀원 PC 와도 안 섞인다.
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    unset FASTRTPS_DEFAULT_PROFILES_FILE
else
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
    # ★ "없으면 복사" 로는 모자란다 — 프로파일 내용이 바뀐다. main d5c0734 가
    #   16MB 송수신 버퍼를 넣었는데, 예전에 복사해 둔 ~/.ros 것을 그냥 쓰면
    #   이미지 UDP 단편 손실이 그대로다 (§15: 깊이 커버리지 4/8 vs 6/8).
    #   그래서 **다르면 갱신**한다.
    mkdir -p "$HOME/.ros"
    cmp -s "$WS/scripts/fastdds_whitelist.xml" "$FASTRTPS_DEFAULT_PROFILES_FILE" || {
        cp "$WS/scripts/fastdds_whitelist.xml" "$FASTRTPS_DEFAULT_PROFILES_FILE"
        echo "[run_all_local] fastdds 프로파일 갱신 — ~/.ros/fastdds_whitelist.xml"; }
    # ★ 이 PC 의 IP 가 whitelist 에 없으면 **루프백으로만 돈다.** 다른 PC 에서
    #   안 보이는데 여기서는 멀쩡해서, 통신이 되는 줄 알고 넘어가기 딱 좋다.
    _ips="$(ip -4 -o addr show 2>/dev/null | awk '{split($4,a,"/"); print a[1]}')"
    _hit=0
    for _ip in $_ips; do
        [ "$_ip" = "127.0.0.1" ] && continue
        grep -q "<address>$_ip</address>" "$FASTRTPS_DEFAULT_PROFILES_FILE" && _hit=1
    done
    [ "$_hit" = 1 ] || {
        echo "⚠ 이 PC 의 IP 가 fastdds_whitelist.xml 에 없다 ($_ips)." >&2
        echo "  이대로면 루프백으로만 돌아 **다른 PC 에서 토픽이 안 보인다.**" >&2
        echo "  whitelist 에 IP 를 넣든지, 혼자 돌리려면 PL_NET=local 로." >&2; }
fi

# ── 튜닝값 — docs/2026-08-27_시연_다중PC_배치.md §2 기준 ────────────────────
# ★★ PL_GTPICK 과 vision_zone 은 **짝이다.** 어긋나면 `/zone_*/box_at_pickup` 을
#   아무도 안 내고 관제탑이 영원히 IDLE 이 된다 — 12 초 뒤 "인식이 침묵한다"로
#   박스를 치우기만 한다 (2026-08-27 실측으로 밟았다). 에러는 안 난다.
#
#     PL_GTPICK=1 (기본)  씬이 **좌표+신원** 을 box_at_pickup 으로 낸다
#                         -> vision_zone 은 반드시 false (안 그러면 발행자 둘)
#     PL_GTPICK=2         씬은 **신원만** box_ident 로 낸다 (신원 대역 모드)
#                         -> vision_zone 은 반드시 true (좌표를 존 카메라가 낸다)
#
#   그래서 아래에서 vision_zone 을 **PL_GTPICK 에서 파생**시킨다. 손으로 둘을
#   맞추게 하면 반드시 어긋난다.
#   기본 1 은 배치안 §2 PC1 과 같다 — 존 카메라 없이도 도는 쪽이라 시연에 안전하다.
export PL_GTPICK="${PL_GTPICK:-1}"
export PL_CAMROS="${PL_CAMROS:-2}"          # 카메라를 우리가 ROS 로 쏜다 (씬 발행자 0)
export PL_SPUR_STOP="${PL_SPUR_STOP:-2}"
export PL_METER="${PL_METER:-2}"
export PL_SPAWN="${PL_SPAWN:-8}"
# ★ 3 이지 10 이 아니다. 시연은 화면이 주인공이라 **배속(10)보다 부드러움(3)** 을
#   택한다 — 3 이면 약 0.75 배속·20fps 로 로봇 동작이 자연스럽다 (배치안 §2 PC1).
#   적재 후반을 빨리 감고 싶으면 그때만 PL_RENDER_EVERY=10 으로 다시 띄운다.
export PL_RENDER_EVERY="${PL_RENDER_EVERY:-3}"
# ★ 만재 팔레트 반출 연출 (PL_OUTFEED=1, main e33c03f~e01ccd3). 연출을 안 볼
#   거면 PL_OUTFEED=0.
export PL_OUTFEED="${PL_OUTFEED:-1}"
# ★ 깊이 교차검증 (2026-08-27). RGB 만으로는 종류를 못 가른다 — 실측 정확도 46 %.
#   틀리면 로봇이 명령받은 z 로 내려가다 진짜 박스 윗면에 얹혀 IK_FAIL 이 난다
#   (4호·5호가 한 개도 안 집혔다). 깊이는 윗면 z 를 직접 잰다.
#     crosscheck  깊이는 **채점만** — RGB 가 정한다 (기본. 어긋남을 눈으로 본다)
#     prefer      깊이가 있으면 **깊이가 정한다** — 크기 오분류를 실제로 고친다
#   ⚠ prefer 는 재현성이 깨질 수 있다 (깊이 도착률이 100 % 가 아니면 같은 시드에
#     박스마다 경로가 달라진다 — CLAUDE.md §15). 먼저 crosscheck 로 깊이가
#     제정신인지 보고 올릴 것.
#   ★ 2026-08-27 실측으로 **prefer 로 올렸다.** 깊이가 나오기 시작한 첫 판에서
#     불일치 26 건이 **전부 깊이 쪽이 한 등급 크고**, 그게 spawner 실물과 맞았다:
#       RGB 포기 -> 깊이 3호 x17 · RGB 3호 -> 깊이 4호 x3
#       RGB 포기 -> 깊이 5호 x3  · RGB 4호 -> 깊이 5호 x3
#     같은 판의 RGB 등록 정확도는 5/11 이었다. CLAUDE.md §15 가 prefer 를 막아 둔
#     이유는 "진짜 Isaac 깊이로 잰 적이 없다" 였는데, 이제 쟀다.
#   ⚠ 남는 위험은 재현성뿐이다 (깊이 도착률 < 100 % 면 같은 시드에 경로가 달라진다).
#     되돌리려면 PL_DEPTH_MODE=crosscheck.
export PL_DEPTH_MODE="${PL_DEPTH_MODE:-prefer}"

# ── 남은 프로세스 정리 ──────────────────────────────────────────────────────
# ★ 안 지우면 관제탑이 둘이 되어 같은 구역에 명령을 겹쳐 쏜다. 에러는 안 난다.
python3 scripts/kill_ros_nodes.py || true
# ★ kill_ros_nodes 는 lib/ 경로만 본다 — `python3 -m packing_algo.algo_service_node`
#   같은 **수동 실행 잔재**는 못 잡는다. 실측(2026-08-27): 이틀 전 것 2개가 살아남아
#   /algo/plan_placement 서버가 3개가 될 뻔했다 (어느 것이 답할지 미정).
for pat in 'sim_main\.py' 'packing_algo\.algo_service_node' 'packing_algo/live_view'; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do
        [ "$p" = "$$" ] || kill "$p" 2>/dev/null || true
    done
done
sleep 1

# ── git 에 안 오는 생성물 3종 (docs/2026-08-27_시연_다중PC_배치.md §4) ────────
# ★ 셋 다 강의실에서 실제로 밟은 함정이다. 없으면 증상이 전부 다른 데서 나온다.
_MISS=0
[ -d "$HOME/isaac_scenes/Collected_test_3" ] || {
    echo "✖ 씬이 없다: ~/isaac_scenes/Collected_test_3/" >&2
    echo "  증상: \"Selected Prim is not a Rigid Body\" 전면 + 로봇 아티큘레이션 없음" >&2
    echo "  rsync -av --delete --exclude='_archive/' byeongjik@10.10.0.3:isaac_scenes/ ~/isaac_scenes/" >&2
    _MISS=1; }
[ -d "$WS/qr_labels" ] && [ "$(ls -1 "$WS/qr_labels" 2>/dev/null | wc -l)" -gt 0 ] || {
    echo "✖ qr_labels/ 가 비었다 — 박스에 QR 이 안 붙어 인식이 전멸한다." >&2
    echo "  rsync -av byeongjik@10.10.0.3:palletris_ws/qr_labels/ ~/palletris_ws/qr_labels/" >&2
    _MISS=1; }
[ "$_MISS" = 0 ] || exit 1

# ── DB 마스터 확인 ──────────────────────────────────────────────────────────
# ★ boxes_master 가 비어 있으면 db_service_node 가 **기동 즉시 죽는다.**
#   그러면 관제탑이 /db/get_box_info 를 못 찾아 박스를 12 초 뒤 치운다 —
#   "로봇이 아무것도 안 한다" 로 보이는 증상의 실제 원인이었다 (2026-08-27).
DB="$WS/palletris.db"
if ! python3 - "$DB" <<'PY'
import sqlite3, sys
try:
    n = sqlite3.connect(f"file:{sys.argv[1]}?mode=ro", uri=True) \
               .execute("select count(*) from boxes_master").fetchone()[0]
except Exception:
    n = 0
sys.exit(0 if n else 1)
PY
then
    echo "[run_all_local] boxes_master 가 비었다 — gen_master_db 를 돌린다"
    ros2 run packing_db gen_master_db >/dev/null
fi

# ── 종료 처리 ───────────────────────────────────────────────────────────────
ISAAC_PID=""
cleanup() {
    trap - INT TERM EXIT
    echo ""; echo "[run_all_local] 내리는 중..."
    python3 "$WS/scripts/kill_ros_nodes.py" >/dev/null 2>&1 || true
    [ -n "$ISAAC_PID" ] && kill -- "-$ISAAC_PID" 2>/dev/null || true
    sleep 1
    for p in $(pgrep -f 'sim_main\.py' 2>/dev/null); do kill "$p" 2>/dev/null || true; done
    echo "[run_all_local] 로그  $ISAAC_LOG"
    echo "[run_all_local]       $CTRL_LOG"
}
trap cleanup INT TERM EXIT

# ── [1] Isaac ───────────────────────────────────────────────────────────────
echo "[run_all_local] DOMAIN $ROS_DOMAIN_ID · 구역 ${PL_ZONES:-A,B,C} · 급송 ${PL_SPAWN}초 · 망 ${PL_NET:-subnet} · gtpick $PL_GTPICK · depth $PL_DEPTH_MODE"
echo "[run_all_local] 씬     ${PL_STAGE:-Collected_test_3 (기본)}"
echo "[run_all_local] [1/2] Isaac 기동 — 로그 $ISAAC_LOG"
# ★ setsid: Isaac 은 자식(kit)을 여럿 낳는다. 프로세스 그룹째 잡아야 Ctrl-C 로
#   전부 내려간다. 안 그러면 kit 이 살아남아 다음 실행이 8765 포트에서 막힌다.
setsid bash "$WS/scripts/run_test3_ros.sh" >"$ISAAC_LOG" 2>&1 &
ISAAC_PID=$!

# ── /clock 을 기다린다 ──────────────────────────────────────────────────────
echo "[run_all_local]       /clock 대기 (기동에 40~60초 걸린다)"
OK=0
for i in $(seq 1 90); do
    if ! kill -0 "$ISAAC_PID" 2>/dev/null; then
        echo "✖ Isaac 이 죽었다. 로그를 볼 것: $ISAAC_LOG" >&2
        tail -30 "$ISAAC_LOG" >&2; exit 1
    fi
    if ros2 topic info /clock 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
        OK=1; break
    fi
    sleep 2
done
[ "$OK" = 1 ] || {
    echo "✖ /clock 발행자가 180초 안에 안 떴다. 이대로 bringup 을 올리면" >&2
    echo "  ROS 시계가 0 에 멈춰 **모든 타이머가 멎는다** — 에러도 안 난다." >&2
    echo "  로그: $ISAAC_LOG" >&2; exit 1; }
echo "[run_all_local]       /clock OK"

# ── [2] ROS 노드 전부 ───────────────────────────────────────────────────────
# .3(인식) 과 .4(관제탑·algo·db) 를 합친 것.
#
# ★ vision_zone 은 위 PL_GTPICK 에서 파생된다 (거기 주석 참조).
#   어느 쪽이든 등록·분류·검증(cam1·cam2)은 **진짜 비전** 그대로다.
if [ "$PL_GTPICK" = "2" ]; then
    VZ=true    # 씬은 신원만 — 좌표는 존 카메라가 낸다
else
    VZ=false   # 씬이 좌표까지 낸다 — 존 카메라를 켜면 발행자가 둘이 된다
fi
# ★ 실수 방지 (2026-08-27 — 같은 사고 2회). 2-PC 용 orchestrator:=false 를
#   한 대 실행에 그대로 붙이면 **로봇에 명령 내릴 노드가 아무 데도 없다.**
#   인식·알고리즘·DB 는 다 도는데 적재만 안 되고, 에러는 한 줄도 안 난다.
case " $* " in *" orchestrator:=false "*)
    echo ""
    echo "⚠⚠ orchestrator:=false — 이 PC 에는 관제탑이 안 뜬다!" >&2
    echo "   다른 PC(.4)에서 관제탑을 띄우는 2-PC 구성일 때만 쓸 것." >&2
    echo "   한 대로 다 돌리려면 이 인자를 빼고 다시 실행:" >&2
    echo "     PL_GTPICK=2 PL_RENDER_EVERY=10 PL_SPAWN=4 ./scripts/run_all_local.sh" >&2
    echo "   (10초 기다렸다 계속한다 — 2-PC 가 맞으면 그대로 두면 된다)" >&2
    sleep 10 ;;
esac
echo "[run_all_local] [2/2] bringup — 로그 $CTRL_LOG"
echo "[run_all_local]       Isaac 로그는 다른 터미널에서:  tail -f $ISAAC_LOG"
echo ""
ros2 launch packing_bringup bringup.launch.py \
    vision:=true vision_zone:=$VZ depth:=true depth_mode:="$PL_DEPTH_MODE" \
    config_overlay:="$PL_CONFIG_OVERLAY" \
    "$@" 2>&1 | tee "$CTRL_LOG"
