#!/usr/bin/env bash
# 씬 USD 배포. **김학민 전용.**
# USD 는 git merge 가 불가능하므로 git 이 아니라 rsync 로 배포한다.
# --delete 가 있으니 경로를 반드시 확인하고 실행할 것.
set -euo pipefail
SRC="$HOME/isaac_scenes/"

if [ ! -d "$SRC" ]; then
  echo "!! $SRC 가 없습니다."; exit 1
fi

read -rp "정말 $SRC 를 10.10.0.2~4 로 덮어씁니까? (yes 입력) " ans
[ "$ans" = "yes" ] || { echo "취소"; exit 1; }

for ip in 10.10.0.2 10.10.0.3 10.10.0.4; do
  echo ">> $ip"
  rsync -av --delete "$SRC" "rokey@${ip}:/home/rokey/isaac_scenes/"
done

echo
echo ">> 완료. 팀 채팅에 '씬 배포함' 공지할 것."
