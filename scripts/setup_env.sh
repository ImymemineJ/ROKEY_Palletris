#!/usr/bin/env bash
# clone 직후 각자 1회 실행. 두 번 실행해도 안전하다.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

mkdir -p "$HOME/.ros"
cp "$REPO/scripts/fastdds_whitelist.xml" "$HOME/.ros/"
echo ">> ~/.ros/fastdds_whitelist.xml 배치 완료"

MARK="# >>> pjt_ws env >>>"
if grep -q "$MARK" "$HOME/.bashrc" 2>/dev/null; then
  echo ">> ~/.bashrc 에 이미 설정이 있습니다. 건너뜁니다."
else
  cat >> "$HOME/.bashrc" <<BASHRC

$MARK
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/.ros/fastdds_whitelist.xml
[ -f "$REPO/install/setup.bash" ] && source "$REPO/install/setup.bash"
# <<< pjt_ws env <<<
BASHRC
  echo ">> ~/.bashrc 설정 추가 완료. 새 터미널을 열거나 'source ~/.bashrc' 하세요."
fi

echo
echo ">> 확인:"
echo "   echo \$ROS_DOMAIN_ID     → 115 가 나와야 합니다 (4명 전원 동일)"
