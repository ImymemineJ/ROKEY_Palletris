#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════
#  실행 기록 — rosbag + DB 사본 + 로그를 **한 폴더에** 모은다
#
#      bash scripts/record_run.sh                 # 기본 이름
#      bash scripts/record_run.sh run1            # 이름 지정
#      RECORD_HEIGHTMAP=1 bash scripts/record_run.sh   # heightmap 도 같이
#
#  Ctrl+C 로 멈추면 DB 사본까지 자동으로 뜬다.
#
# ── 왜 이 셋을 같이 모으나 ────────────────────────────────────────────
#   bag  = 토픽만 담긴다. "무슨 일이 있었나"
#   DB   = 누적 상태. "결과가 어떻게 됐나"
#   로그 = **서비스 호출 결과**. bag 에 안 담기는 유일한 것 —
#          중재자가 왜 그 구역을 골랐는지는 여기에만 남는다.
#   셋이 같은 실행이어야 서로 대조할 수 있다.
#
# ── ★ 카메라 이미지는 일부러 뺐다 ────────────────────────────────────
#   1280x720 x 5 대면 이론상 ~100 MB/s 다 (docs/2026-08-27_시연_다중PC_배치.md).
#   몇 분만 돌려도 수십 GB 가 되고, 이미 단편 손실이 나는 링크에 소비자를
#   하나 더 얹는 셈이다. 이미지가 필요하면 그때 따로 뜬다.
# ══════════════════════════════════════════════════════════════════════
# ★ set -u 를 안 쓴다. ROS 의 setup.bash 가 미정의 변수를 참조해서
#   (AMENT_TRACE_SETUP_FILES) source 하는 순간 죽는다 — 실측으로 걸렸다.
set -o pipefail

NAME="${1:-run_$(date +%Y%m%d_%H%M%S)}"
OUT="$HOME/palletris_records/$NAME"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ── 환경 (docs/2026-08-27_시연_다중PC_배치.md 와 같아야 한다) ──────────
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash" 2>/dev/null || {
  echo "!! install/setup.bash 가 없다 — colcon build 부터 할 것"; exit 1; }

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-115}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# ★ SUBNET 이어야 다른 PC 가 보인다. LOCALHOST 면 토픽이 0 개인데 에러도 안 난다
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

mkdir -p "$OUT"

# ── 기록할 토픽 ───────────────────────────────────────────────────────
TOPICS=(
  # 관제탑 → 모니터링 (2 Hz). 대시보드 카드가 이걸 쓴다
  /zone_a/status /zone_b/status /zone_c/status
  # 관제탑 → 로봇 / 로봇 → 관제탑. ★ 구역별이다 (전역 /robot/event 는 없다)
  /zone_a/robot/cmd   /zone_b/robot/cmd   /zone_c/robot/cmd
  /zone_a/robot/event /zone_b/robot/event /zone_c/robot/event
  # 인식 → 관제탑. "박스가 결과적으로 어느 구역에 도착했나" 가 여기 나온다
  /zone_a/box_at_pickup /zone_b/box_at_pickup /zone_c/box_at_pickup
  /zone_a/box_ident     /zone_b/box_ident     /zone_c/box_ident
  # 깊이 교차검증 진단
  /zone_a/vision_check /zone_b/vision_check /zone_c/vision_check
  /line/vision_check
  # DB → 모니터링 (1 Hz)
  /metrics/summary
  # 시간·좌표계
  /clock /tf /tf_static
)

# heightmap 은 algo 를 `-p heightmap_topic:=/algo/heightmap` 로 띄웠을 때만 나온다
if [ "${RECORD_HEIGHTMAP:-0}" = "1" ]; then
  TOPICS+=(/algo/heightmap/a /algo/heightmap/b /algo/heightmap/c)
fi

echo "═══════════════════════════════════════════════"
echo "  기록 폴더 : $OUT"
echo "  DOMAIN_ID : $ROS_DOMAIN_ID   탐색범위: $ROS_AUTOMATIC_DISCOVERY_RANGE"
echo "  토픽      : ${#TOPICS[@]} 개 (카메라 이미지 제외)"
echo "═══════════════════════════════════════════════"

# ── ★ 있는 토픽만 거른다 ──────────────────────────────────────────────
#   `ros2 bag record` 에 없는 토픽을 주면 조용히 기다리기만 한다.
#   여기서 걸러 두면 **무엇이 안 잡혔는지 지금 눈에 보인다.**
echo "[1/3] 토픽 확인 중 (3초)..."
LIVE=$(timeout 5 ros2 topic list 2>/dev/null)
REC=(); MISS=()
for t in "${TOPICS[@]}"; do
  if grep -qx -- "$t" <<<"$LIVE"; then REC+=("$t"); else MISS+=("$t"); fi
done

if [ ${#MISS[@]} -gt 0 ]; then
  echo "  ⚠ 아직 없는 토픽 ${#MISS[@]} 개 — 기록에서 빠진다:"
  printf '      %s\n' "${MISS[@]}"
  echo "    (노드가 다 떠 있는지 확인할 것. 특히 Isaac 이 먼저 떠야 /clock 이 나온다)"
fi
if [ ${#REC[@]} -eq 0 ]; then
  echo "!! 기록할 토픽이 하나도 없다. 노드를 먼저 띄울 것"; exit 1
fi
echo "  → ${#REC[@]} 개 기록한다"

# ── 종료 시 DB 사본 + 요약 ────────────────────────────────────────────
finish() {
  echo
  echo "[3/3] 마무리..."
  if [ -f "$WS/palletris.db" ]; then
    # ★ sqlite3 .backup 은 쓰기 중에도 안전하다. cp 는 WAL 때문에 깨질 수 있다
    if command -v sqlite3 >/dev/null; then
      sqlite3 "$WS/palletris.db" ".backup '$OUT/palletris.db'" 2>/dev/null \
        && echo "  DB 사본  : $OUT/palletris.db"
    else
      cp "$WS/palletris.db" "$OUT/palletris.db" && echo "  DB 사본  : $OUT/palletris.db (cp)"
    fi
  else
    echo "  ⚠ palletris.db 가 없다"
  fi
  { echo "name=$NAME"; echo "date=$(date -Is)"; echo "host=$(hostname)";
    echo "domain=$ROS_DOMAIN_ID"; echo "overlay=${PL_CONFIG_OVERLAY:-}";
    echo "git=$(git -C "$WS" rev-parse --short HEAD 2>/dev/null)";
    echo "topics=${#REC[@]}"; printf 'topic=%s\n' "${REC[@]}"; } > "$OUT/meta.txt"
  echo "  메타     : $OUT/meta.txt"
  echo
  echo "  크기: $(du -sh "$OUT" 2>/dev/null | cut -f1)"
  echo
  echo "  ── 옮기기 ──"
  echo "  tar czf ~/$NAME.tar.gz -C ~/palletris_records $NAME"
  echo "  scp ~/$NAME.tar.gz <받을PC>:~/"
  echo "═══════════════════════════════════════════════"
}
trap finish EXIT INT TERM

echo "[2/3] 기록 시작 — 끝나면 Ctrl+C"
# ★ Jazzy 는 위치인자를 폐기 예정으로 경고한다 — --topics 를 쓴다
ros2 bag record -o "$OUT/bag" --topics "${REC[@]}"
