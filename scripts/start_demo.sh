#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════
#  시연 시작 — DB 를 비우고 Isaac 을 띄운다 (2026-08-27)
#
#      ./scripts/start_demo.sh                 비우고 실행
#      ./scripts/start_demo.sh --keep          안 비우고 실행 (이어서 볼 때)
#      PL_ZONES=B ./scripts/start_demo.sh      run_test3_ros.sh 의 환경변수 그대로
#
#  ★ 왜 필요한가 (실측): Isaac 은 껐다 켜면 씬이 비는데 `palletris.db` 는 안
#    비워진다. 그래서 대시보드가 **어제 실행을 오늘 것처럼** 보여준다.
#    지표 코드가 "시간이 뒤로 간 자리" 로 실행을 가르기는 하는데, 그 뒤에
#    기록이 30 건은 쌓여야 인정한다 (`metrics.RUN_MIN_EVENTS`) — 그래서
#    **시작 직후 한동안은 반드시 어긋난다.** 시작할 때 비우면 그 구간이 없다.
#
#  ★ 지우기 전에 **반드시 백업한다.** 지우는 건 되돌릴 수 없고, 발표 직후
#    "방금 그 실행" 을 다시 봐야 할 일이 실제로 생긴다. 백업은 손해가 없다.
#
#  ★ 노드가 떠 있는 채로 비우면 `database is locked` 가 난다 —
#    `db_service_node` 가 유일 writer 라는 규약 때문이다 (CLAUDE.md §10).
#    그래서 남은 노드를 먼저 정리한다. 이건 시연 절차 문서의 PC1 첫 줄과 같다.
#
#  ⚠ 이 스크립트는 **껍데기다.** 실행 자체는 `run_test3_ros.sh`(트랙 B) 가 한다.
#    그 파일은 손대지 않았다 — 인자·환경변수는 전부 그대로 넘어간다.
# ══════════════════════════════════════════════════════════════════════════
set -e
HERE="$(dirname "$(readlink -f "$0")")"
WS="$(dirname "$HERE")"

KEEP=0
ARGS=()
for a in "$@"; do
  case "$a" in
    --keep) KEEP=1 ;;
    *) ARGS+=("$a") ;;
  esac
done

DB="${PALLETRIS_DB:-$WS/palletris.db}"
BACKUP_DIR="${PL_BACKUP_DIR:-$HOME/palletris_runs}"

if [ "$KEEP" = "1" ]; then
  echo "[start_demo] --keep : DB 를 그대로 둔다 ($DB)"
else
  source /opt/ros/jazzy/setup.bash
  source "$WS/install/setup.bash"

  # ① 남은 노드 정리 — db_service_node 가 DB 를 잡고 있으면 못 비운다
  echo "[start_demo] 1/3 남은 ROS 노드 정리"
  python3 "$WS/scripts/kill_ros_nodes.py" >/dev/null 2>&1 || true

  # ② 백업 — 지우기 전에 항상
  if [ -f "$DB" ]; then
    mkdir -p "$BACKUP_DIR"
    BK="$BACKUP_DIR/palletris_$(date +%m%d_%H%M%S).db"
    cp "$DB" "$BK"
    echo "[start_demo] 2/3 백업 → $BK"
  else
    echo "[start_demo] 2/3 백업 건너뜀 (DB 가 없다 — 새로 만든다)"
  fi

  # ③ 비우고 정답지 다시 깔기
  echo "[start_demo] 3/3 DB 초기화"
  ros2 run packing_db gen_master_db --reset
fi

echo "[start_demo] Isaac 시작 → run_test3_ros.sh ${ARGS[*]}"
exec "$HERE/run_test3_ros.sh" "${ARGS[@]}"
