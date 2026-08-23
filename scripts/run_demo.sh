#!/bin/bash
# 시연 한 방 — 창을 띄우고, 15박스를 라인으로 흘리고, 끝나도 창을 닫지 않는다.
#
#   ./scripts/run_demo.sh              도착 순서 · 11개 — **시연 기본값**
#   ./scripts/run_demo.sh arrival 15   사양값 15개 (팔레트가 한 번 갈린다)
#   ./scripts/run_demo.sh volume 8     비교용
#
# ★ 왜 11개인가 — 벨트 1 · 로봇 1 · **팔레트 1** 구성에 맞춘 값이다.
#   seed 42 로 도착 순서면 12번째 5호에서 팔레트가 찬다 (실측:
#   results/stack15_pass2_20260823.log — "간섭 없이 갈 수 있는 자리가 없다").
#   막는 것은 높이가 아니라 바닥이다. 최상단이 840 mm 로 한계 1300 에 한참
#   못 미치는데도 못 쌓는다 — heightmap.enforce_load_order 때문에 9 kg 5호는
#   바닥이나 다른 5호 위에만 갈 수 있고, seed 42 는 5호가 4개라(분포 기대값 3)
#   그것들이 바닥을 다 먹는다.
#   그래서 11개면 팔레트를 한 판으로 끝낼 수 있다. 15개로 돌리면 12번째에서
#   swap_pallet 이 돈다 — 결함이 아니라 사양 경로다.
#
# 게이트(헤드리스·창 없음·끝나면 종료)는 이게 아니라 아래를 쓴다:
#   cd ~/isaacsim && ./isaac-sim.sh --no-window \
#       --exec ~/palletris_ws/scripts/demo_stack15_test1_1.py
#
# ★ `isaac-sim.sh` 여야 한다. 벨트가 `IsaacConveyor` OmniGraph 노드로 도는데
#   그 노드는 렌더 틱에 물려 있어서 `python.sh` standalone 은 속도를 틀리게 돌린다
#   (근거: src/packing_sim/isaac/conveyor_ctl.py docstring).
set -e
HERE="$(dirname "$(readlink -f "$0")")"
ISAAC="${ISAAC_DIR:-$HOME/isaacsim}"

export PL_HOLD=1                         # 다 놓은 뒤 창을 닫지 않는다
# ★ 팔 속도. 사양값은 0.5 인데 시연은 0.7 로 돌린다. **이 실행에서만** 덮어쓰고
#   system.yaml 도 B 코드도 안 건드린다 (게이트는 사양값 그대로 돈다).
#   근거 — 첫 5호가 이송 177도 스윙 뒤 착지에서 미끄러지는 일이 있었다. 박스가
#   흡착 패드에 강성 100 · 감쇠 0 으로 매달려 진자가 되는데, 감쇠가 0이라 하강
#   2.3 s 동안 안 잦아들고 놓는 위상에 따라 착지가 갈린다 (gripper.py 주석의
#   "22~76 mm", 실측 -99.8 mm). 속도를 바꿔 실측한 결과:
#     0.3  3번 -82.8 mm · knock 601 mm · 속도초과 9 · 잔차초과 3   실패 (더 나쁨)
#     0.5  3번 5회 중 1회 미끄러짐 (-99.8 mm)                      간헐 실패
#     0.7  3번 +1.5 mm 2회 연속 · knock 0.9~1.2 mm · 속도초과 2    통과
#   덤으로 사이클이 14.4 -> 12.0 s 로 떨어져 스폰 간격(12 s)과 같아진다.
#   그래서 벨트 위에 줄이 더 안 길어진다 (라인 위 5개 -> 4개).
#   ⚠ 사양값을 0.7 로 바꾸자는 제안은 팀 안건이다. 여기서 정하지 않는다.
export PL_SPEED="${PL_SPEED:-0.7}"
export PL_ORDER="${1:-arrival}"
export PL_N="${2:-11}"                   # 팔레트 한 판으로 끝나는 수 (위 주석)

echo "시연 시작 — 순서 $PL_ORDER · 박스 $PL_N 개 · 팔레트 1판 · 팔속도 $PL_SPEED"
echo "   씬을 다시 깔려면 먼저: $HERE/../src/packing_sim/run_layout_to_spec.sh"
echo
cd "$ISAAC"
exec ./isaac-sim.sh --exec "$HERE/demo_stack15_test1_1.py"
