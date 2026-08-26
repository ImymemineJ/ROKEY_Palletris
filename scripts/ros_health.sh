#!/bin/bash
# ROS 2 연결 상태 한 장 — "누가 붙었고 누가 안 붙었나".
#
#   ./scripts/ros_health.sh              A,B,C 세 구역
#   PL_ZONES=b ./scripts/ros_health.sh   한 구역만
#   watch -n2 ./scripts/ros_health.sh    실시간
#
# ★ rqt_graph 는 노드 3구역이 되면 선이 엉켜 못 읽는다. 이건 **계약(§3-2·§3-3)에
#   적힌 것만** 골라 본다. 계약에 없는 토픽은 여기 안 나온다.
#
# 판정 규칙 — 토픽은 **양쪽이 다 있어야** 이어진 것이다.
#   pub 0 = 발행자 없음 (내는 쪽이 안 떴다)
#   sub 0 = 구독자 없음 (받는 쪽이 안 떴다) — 데이터는 흐르지만 아무도 안 듣는다
# ⚠ `set -u` 는 **소스 뒤에** 건다. ROS 의 setup.bash 가 미정의 변수를 만지므로
#   앞에 걸면 첫 줄도 못 찍고 조용히 죽는다 (2026-08-26 실측).
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[ -f install/setup.bash ] && source install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-115}"
set -u

G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; D=$'\e[2m'; N=$'\e[0m'
ZONES="${PL_ZONES:-a,b,c}"

TOPICS="$(ros2 topic list 2>/dev/null)"
SERVICES="$(ros2 service list 2>/dev/null)"
NODES="$(ros2 node list --all 2>/dev/null | grep -v _ros2cli_daemon)"   # --all: Isaac 브릿지 노드는 이름이 _ 로 시작하는 **숨김 노드**라 기본 목록에 안 나온다

# $1 토픽  $2 기대(pub 낼 쪽)  $3 기대(sub 받을 쪽)
row_topic() {
    local t="$1" who_p="$2" who_s="$3" info p s mark
    if ! grep -qx -- "$t" <<< "$TOPICS"; then
        printf "  %s%-34s  %-9s %s%s\n" "$R" "$t" "없음" "($who_p -> $who_s)" "$N"; return
    fi
    info="$(ros2 topic info "$t" 2>/dev/null)"
    p="$(sed -n 's/.*Publisher count: *//p'    <<< "$info")"
    s="$(sed -n 's/.*Subscription count: *//p' <<< "$info")"
    if [ "${p:-0}" -gt 0 ] && [ "${s:-0}" -gt 0 ]; then mark="$G OK "
    elif [ "${p:-0}" -eq 0 ];                     then mark="$R pub0"
    else                                               mark="$Y sub0"; fi
    printf "  %s%-34s  pub %-2s sub %-2s%s  %s%s -> %s%s\n" \
           "$mark" "$t" "${p:-?}" "${s:-?}" "$N" "$D" "$who_p" "$who_s" "$N"
}

row_srv() {
    local s="$1" who="$2"
    if grep -qx -- "$s" <<< "$SERVICES"; then
        printf "  %s OK %s%-34s%s%s%s%s\n" "$G" "$N" "$s" "$D" "  " "$who" "$N"
    else
        printf "  %s없음%s %-34s%s%s%s\n" "$R" "$N" "$s" "$D" "  $who" "$N"
    fi
}

echo
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID   노드 $(grep -c . <<< "$NODES") 개"
echo "  $D$(tr '\n' ' ' <<< "$NODES")$N"

echo
echo "── A. 씬 기반 (Isaac Action Graph) ─────────────────────"
row_topic /clock       "Isaac"  "전원"
row_topic /tf          "Isaac"  "vision"
row_topic /tf_static   "Isaac"  "vision"

echo
echo "── 카메라 (B 레인) ────────────────────────────────────"
for z in ${ZONES//,/ }; do
    row_topic "/pick_cam_$z/rgb"         "Isaac" "vision_$z"
    row_topic "/pick_cam_$z/camera_info" "Isaac" "vision_$z"
done

echo
echo "── 로봇 왕복 (관제탑 <-> Isaac ros_side) ───────────────"
for z in ${ZONES//,/ }; do
    row_topic "/zone_$z/robot/cmd"      "관제탑" "Isaac"
    row_topic "/zone_$z/robot/event"    "Isaac"  "관제탑"
done

echo
echo "── 인식 · 상태 ────────────────────────────────────────"
for z in ${ZONES//,/ }; do
    row_topic "/zone_$z/box_at_pickup" "vision" "관제탑"
    row_topic "/zone_$z/status"        "관제탑" "중재자·모니터"
done

echo
echo "── 서비스 ─────────────────────────────────────────────"
row_srv /conveyor/set_velocity   "Isaac (--ros)"
row_srv /sim/despawn_box         "Isaac (--ros)"
row_srv /sim/set_spawn_enabled   "Isaac (--ros)"
row_srv /sim/swap_pallet         "Isaac (--ros)"
row_srv /algo/plan_placement     "algo"
row_srv /algo/update_heightmap   "algo"
row_srv /algo/cancel_plan        "algo"
row_srv /algo/reset_heightmap    "algo"
row_srv /db/register_box         "db"
row_srv /db/get_box_info         "db"
row_srv /db/mark_placed          "db"
row_srv /diverter/request        "db (중재자)"
echo

# ── 중복 노드 검사 ─────────────────────────────────────
# ★ `ros2 launch` 를 Ctrl-C 나 kill 로 끊으면 **자식 파이썬 노드가 살아남는다.**
#   그러면 관제탑이 둘·셋이 되어 같은 구역에 명령을 겹쳐 쏜다 — 에러는 안 난다.
#   토픽에 `pub 2` 처럼 수가 하나 더 보이면 여기를 볼 것 (2026-08-26 실측: 3 개).
# ⚠ 죽일 때 `pkill -f orchestrator_node` 를 쓰지 마라 — **자기 셸의 명령줄에도**
#   그 문자열이 들어 있어 자기를 죽인다. 설치 경로로 고르는 아래 방식을 쓸 것.
_dup=""
for _n in palletris_orchestrator packing_vision packing_algo packing_db; do
    _c=$(ps -eo args | grep -c "lib/$_n/" 2>/dev/null)
    _c=$((_c - 1))                     # grep 자기 자신 제외
    case "$_n" in packing_vision) _lim=4 ;; *) _lim=1 ;; esac
    [ "$_c" -gt "$_lim" ] && _dup="$_dup $_n($_c>$_lim)"
done
if [ -n "$_dup" ]; then
    printf "  %s★ 중복 노드:%s%s\n" "$R" "$_dup" "$N"
    printf "  %s   정리:  python3 scripts/kill_ros_nodes.py%s\n" "$D" "$N"
    echo
fi
