#!/usr/bin/env bash
# ══════════════════════════════════════════════════════
#  PALLETRIS Dashboard 시작 스크립트
#  사용법: bash scripts/dashboard/start_dashboard.sh
#
#  필요 조건:
#    ros-jazzy-rosbridge-suite 설치됨
#    sudo apt install ros-jazzy-rosbridge-suite
# ══════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

PORT=8765         # HTTP 서버 포트
RB_PORT=9090      # rosbridge WebSocket 포트

echo "=== PALLETRIS Dashboard ==="
echo "WS root : $WS_ROOT"
echo "HTTP    : http://localhost:$PORT"
echo "rosbridge: ws://localhost:$RB_PORT"
echo ""

# ── ROS 2 환경 소스 ────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash" 2>/dev/null || true

# ── rosbridge 서버 ─────────────────────────────────────
if ! ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
  echo "[ERROR] rosbridge_server 패키지를 찾을 수 없습니다."
  echo "        다음 명령으로 설치하세요:"
  echo "        sudo apt install ros-jazzy-rosbridge-suite"
  exit 1
fi

echo "[1/2] rosbridge 서버 시작 (포트 $RB_PORT)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    port:=$RB_PORT \
    &
ROSBRIDGE_PID=$!

sleep 1

# ── Python HTTP 서버 ───────────────────────────────────
echo "[2/2] HTTP 서버 시작 (포트 $PORT)..."
cd "$SCRIPT_DIR"
python3 -m http.server $PORT &
HTTP_PID=$!

sleep 0.5

# ── 브라우저 열기 ──────────────────────────────────────
URL="http://localhost:$PORT/index.html"
echo ""
echo "✔ 대시보드가 시작되었습니다!"
echo "  → $URL"
echo ""
echo "  데모 모드 (rosbridge 없이 UI만 테스트):"
echo "  → http://localhost:$PORT/index.html?demo=1"
echo ""
echo "  Ctrl+C 로 종료합니다."

# 브라우저 자동 열기 시도
if command -v xdg-open &>/dev/null; then
  xdg-open "$URL" 2>/dev/null &
elif command -v google-chrome &>/dev/null; then
  google-chrome "$URL" &
fi

# ── 종료 처리 ──────────────────────────────────────────
trap 'echo ""; echo "종료 중..."; kill $ROSBRIDGE_PID $HTTP_PID 2>/dev/null; exit 0' INT TERM

wait
