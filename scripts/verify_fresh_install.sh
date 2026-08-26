#!/usr/bin/env bash
# 깨끗한 설치 검사 — "설치본에서 파일을 못 찾는다" 를 잡는다.
#
#     scripts/verify_fresh_install.sh
#
# ★ 왜 필요한가 (2026-08-26 실측)
#   팀 전원이 `--symlink-install` 을 쓴다. 그건 복사가 아니라 **바로가기**를 만드는
#   방식이라 `realpath(__file__)` 이 소스 트리로 되돌아간다. 그래서 자산 경로가
#   틀려도 우연히 돌아간다. 옵션을 빼고 빌드하면 그 자리에서 죽는다:
#
#       gen_master_db    FileNotFoundError: .../site-packages/packing_db/schema.sql
#       db_service_node  SystemExit: schema.sql 을 찾지 못했다
#
#   CI 도 packing_db 는 colcon test 를 안 돌려 못 잡았다. 이 스크립트가 그 구멍이다.
#
# ★ 일부러 하는 것 둘
#   1) `cd "$TMP"` — 워크스페이스 **밖**에서 실행한다. cwd 를 훑어 찾는 경로가
#      섞이면 검사가 무의미해진다.
#   2) `unset PALLETRIS_CONFIG` — 환경변수로 떠먹여 주지 않는다. share 폴백이
#      진짜로 도는지 봐야 한다.
set -euo pipefail

WS=$(cd "$(dirname "$0")/.." && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "════ --symlink-install 없이 빌드 ════"
cd "$WS"
# ★ 전체를 빌드한다. packing_bringup 이 나머지 패키지 전부에 exec_depend 를 걸어
#   부분 빌드가 안 되고, 애초에 이 검사의 목적이 "새 PC 에서 clone 하면" 이라
#   골라 담으면 검사가 헐거워진다. 1~2 분 걸린다.
colcon build --build-base "$TMP/build" --install-base "$TMP/install" \
  --event-handlers console_direct- 2>&1 | tail -3

# shellcheck disable=SC1091
set +u; . "$TMP/install/setup.bash"; set -u

cd "$TMP"                              # ★ 워크스페이스 밖
unset PALLETRIS_CONFIG || true         # ★ 떠먹여 주지 않는다
unset PALLETRIS_DB || true

echo "════ gen_master_db ════"
ros2 run packing_db gen_master_db --db-path "$TMP/palletris.db" 2>&1 | head -3

echo "════ db_service_node ════"
ros2 run packing_db db_service_node --db-path "$TMP/palletris.db" \
  > "$TMP/node.log" 2>&1 &
NODE=$!
for _ in $(seq 40); do
  grep -q "db_service_node 준비" "$TMP/node.log" 2>/dev/null && break
  kill -0 "$NODE" 2>/dev/null || break          # 이미 죽었으면 기다릴 것 없다
  sleep 0.5
done
kill "$NODE" 2>/dev/null || true
wait "$NODE" 2>/dev/null || true

if ! grep -q "db_service_node 준비" "$TMP/node.log"; then
  echo "★ 실패 — db_service_node 가 안 떴다"
  cat "$TMP/node.log"
  exit 1
fi

echo
echo "✅ 설치본에서 gen_master_db · db_service_node 둘 다 뜬다"
