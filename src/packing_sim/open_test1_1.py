# isaac-sim.sh --exec 로 실행되어 test1-1.usd (벨트 1개 씬) 를 열고 피더까지 붙인다.
#   cd ~/isaacsim && ./isaac-sim.sh --exec /home/rokey/palletris_ws/src/packing_sim/open_test1_1.py
#
# open_test1.py 와 하는 일은 같고 스테이지만 test1-1.usd 다.
# 피더 없이 스테이지만 보려면 PL_NO_FEEDER=1.
# 투입 개수/간격은 PL_N, PL_INTERVAL 로 바꾼다 (기본 10개, 3초).
#
# test1-1.usd 에는 소터가 없다 — 벨트를 /World/ConveyorTrack 하나만 남긴 씬이라
# 분류 켜고 끌 것도 없다. 그래서 PL_SORT 분기는 여기 없다.
# 벨트가 하나뿐이라 박스는 벨트 끝(x 약 -8.9)에서 바닥으로 떨어진다.
import os
import sys

import omni.usd

STAGE = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")
# spawn_feeder 는 import 시점에 PL_STAGE 를 읽어 STAGE_PATH 를 정한다.
# 여기서 먼저 맞춰 두지 않으면 피더가 test1.usd 를 다시 열어버린다.
os.environ["PL_STAGE"] = STAGE

omni.usd.get_context().open_stage(STAGE)
print(f"[open_test1_1] 스테이지 열기: {STAGE}")

if os.environ.get("PL_NO_FEEDER"):
    print("[open_test1_1] PL_NO_FEEDER — 피더를 붙이지 않는다.")
else:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import spawn_feeder
    spawn_feeder.start()
    print("[open_test1_1] 피더 부착 완료. Play 를 누르면 박스가 투입된다.")
