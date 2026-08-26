#!/usr/bin/env bash
# 씬 USD 배포. **김학민 전용.**
# USD 는 git merge 가 불가능하므로 git 이 아니라 rsync 로 배포한다 (설계결정서 §10.3).
# --delete 가 있으니 경로를 반드시 확인하고 실행할 것.
set -euo pipefail
SRC="$HOME/isaac_scenes/"

# ── 배포 대상 ────────────────────────────────────────────────────────────
# GUI 데스크톱 2대는 사용자명이 rokey 로 같지만, **개인 노트북은 각자 다르다.**
# 그래서 IP 만 적지 말고 반드시 `사용자명@IP` 로 적는다.
#
# 노트북까지 한 번에 뿌리려면 목록을 넘긴다:
#   SCENE_HOSTS="rokey@10.10.0.2 rokey@10.10.0.3 byeongjik@10.10.0.9" ./scripts/sync_scene.sh
HOSTS="${SCENE_HOSTS:-rokey@10.10.0.2 rokey@10.10.0.3}"

if [ ! -d "$SRC" ]; then
  echo "!! $SRC 가 없습니다."; exit 1
fi

echo "배포 대상: $HOSTS"
read -rp "정말 위 장비들의 ~/isaac_scenes/ 를 덮어씁니까? (yes 입력) " ans
[ "$ans" = "yes" ] || { echo "취소"; exit 1; }

# ★★ --delete 안전장치 (2026-08-27 김병직 — 사고 실측).
#   보내는 쪽에 없는 파일을 받는 쪽에서 지우는 게 --delete 다. 2026-08-26 밤,
#   보내는 쪽 폴더에 에셋(_archive 등)이 빠진 채 배포해서 **받는 쪽 에셋 1.1GB
#   가 통째로 지워졌다** — 씬의 모든 Reference 가 "Selected Prim is not a Rigid
#   Body" 로 죽었고 tar 백업에서 복원했다. 두 겹으로 막는다:
#   ① 지워질 목록을 먼저 보여주고 다시 묻는다 (--dry-run)
#   ② _archive/ 는 배포 대상에서 빼고, 받는 쪽 것도 안 지운다 (--exclude 는
#      --delete 와 결합하면 "제외 = 삭제도 안 함" 이다)
for host in $HOSTS; do
  echo ">> $host — 삭제될 파일 미리보기"
  DEL=$(rsync -avn --delete --exclude='_archive/' "$SRC" "$host:isaac_scenes/" \
        | grep '^deleting ' || true)
  if [ -n "$DEL" ]; then
    echo "$DEL"
    read -rp "위 파일들이 $host 에서 **지워집니다**. 계속? (yes 입력) " ans2
    [ "$ans2" = "yes" ] || { echo "취소 ($host 건너뜀)"; continue; }
  fi
  # ★ 원격 경로를 **상대경로**로 준다 = 원격 $HOME 기준.
  #   /home/rokey/... 처럼 절대경로로 박으면 사용자명이 다른 장비에서 깨진다.
  rsync -av --delete --exclude='_archive/' "$SRC" "$host:isaac_scenes/"
done

echo
echo ">> 완료. 팀 채팅에 '씬 배포함' 공지할 것."
