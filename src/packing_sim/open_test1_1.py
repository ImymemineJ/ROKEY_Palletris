# ⚠⚠ **구버전이다. 지금 씬과 안 맞는다.** (2026-08-23)
#
# 이 경로(`open_test1_1.py` + `spawn_feeder` + `belt_cycle` + `robot_h2017`)는
# **스토퍼 벽이 생기기 전** 설계다. 지금 씬에는 x 10000~10100 에 실물 벽이 서 있어
# "벨트를 브레이크로 쓴다" 는 전제가 성립하지 않는다.
# 통합 경로는 `scripts/demo_stack15_test1_1.py` 하나다 (cycle.py 상태머신 +
# robot_motion + 소켓 브릿지 + HeightmapPacker). 게이트 수치는 전부 그쪽 것이다.
#
# 트랙B 리뷰(2026-08-23)가 이 경로에서 다섯 가지를 지적했고 **전부 확인됐다.**
# 지금은 아무도 안 부르므로 잠복이지만, 되살리려면 먼저 고쳐야 한다:
#   1. `robot_h2017.ROBOT_PRIM = "/World/robot/h2017"` 가 씬에 없다. 씬 빌더는
#      defaultPrim 을 `/World/robot` 에 직접 합성한다 (EE 도 `/World/robot/root_joint/tool0`).
#   2. 이송 고도에 매달린 박스 높이(+h)가 빠졌다 — 여유가 approach(200) − h 라
#      3호 −10 / 4호 −80 / 5호 −140 mm 로 전부 음수다. 같은 층을 관통한다.
#   3. IK/FK 실패 시 웨이포인트를 건너뛰고 다음 액션을 그대로 실행한다 (공중 흡착).
#   4. HOME 복귀가 보간 없는 계단 입력이다.
#   5. `belt_cycle.pallet_slot` 이 층 높이를 현재 박스 높이로 전 층에 적용한다.
#
# isaac-sim.sh --exec 로 실행되어 test1-1.usd (벨트 1개 라인) 를 열고
# 피더 + 벨트 간헐 구동까지 붙인다.
#   cd ~/isaacsim && ./isaac-sim.sh --exec /home/rokey/palletris_ws/src/packing_sim/open_test1_1.py
#
# 붙는 것
#   spawn_feeder  x=500mm 에 박스를 하나씩 떨어뜨린다.
#   belt_cycle    선두 박스가 픽 지점에 오면 벨트를 세우고, 로봇 사이클이
#                 끝나면 다시 돌린다. 스토퍼는 없다 — 벨트가 브레이크다.
#
# 끄는 법
#   PL_NO_FEEDER=1   피더 없이 스테이지만
#   PL_NO_CYCLE=1    벨트를 계속 돌린다 (예전 동작). 박스는 끝에서 떨어진다.
#   PL_NO_ROBOT=1    H2017 대신 스텁이 집는다 (박스를 순간이동시킨다).
#
# 자주 쓰는 값
#   PL_N, PL_INTERVAL   투입 개수/간격 (기본 10개, 12초 = 사양 stage1)
#   PL_PICK_X           픽 지점 [mm] (기본 9760 = 사양 도달검증과 동일)
#   PL_CYCLE_S          스텁 로봇 1 사이클 [초] (기본 10.0)
#   PL_JITTER=1         박스 가로 위치·요를 다시 흔든다 (기본은 끔)
#
# test1-1.usd 에는 소터가 없어서 PL_SORT 분기는 여기 없다.
import os
import sys

import omni.usd

STAGE = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")
# spawn_feeder 는 import 시점에 PL_STAGE 를 읽어 STAGE_PATH 를 정한다.
# 여기서 먼저 맞춰 두지 않으면 피더가 test1.usd 를 다시 열어버린다.
os.environ["PL_STAGE"] = STAGE
# 로봇이 고정된 픽 자세 하나로 집게 하는 게 목적이라 흔들림을 끈다.
# 박스가 벨트 한가운데에 반듯하게 떨어지고, belt_cycle 이 x 를 세워 준다.
# 인식/파지 쪽을 시험하려고 흐트러뜨리고 싶으면 PL_JITTER=1 로 돌린다.
os.environ.setdefault("PL_JITTER", "0")

omni.usd.get_context().open_stage(STAGE)
print(f"[open_test1_1] 스테이지 열기: {STAGE}")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

if os.environ.get("PL_NO_FEEDER"):
    print("[open_test1_1] PL_NO_FEEDER — 피더를 붙이지 않는다.")
else:
    import spawn_feeder
    spawn_feeder.start()
    print("[open_test1_1] 피더 부착 완료.")

if os.environ.get("PL_NO_ROBOT"):
    print("[open_test1_1] PL_NO_ROBOT — 스텁이 집는다 (H2017 안 씀).")
else:
    import robot_h2017
    robot_h2017.attach()

if os.environ.get("PL_NO_CYCLE"):
    print("[open_test1_1] PL_NO_CYCLE — 벨트를 계속 돌린다.")
else:
    import belt_cycle
    belt_cycle.start()

print("[open_test1_1] Play 를 눌러라.")
