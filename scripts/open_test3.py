"""test_3.usd 를 **그냥 연다.** 카메라를 놓고 저장하기 위한 빈 세션.

    cd ~/isaacsim && ./isaac-sim.sh --exec ~/palletris_ws/scripts/open_test3.py

★ 왜 이게 따로 필요한가. `demo_stack15_test1_1.py` 로 띄운 세션에서 저장하면
  드라이버가 **런타임에 만든 것들이 씬에 구워진다** — `/World/boxes/BOX000xx`,
  `/World/PalletShipped_*`, `/World/ROS2WristCam`, 그리고 로봇 관절이 집던
  자세 그대로. 그래서 카메라를 놓을 때는 아무것도 안 붙은 세션을 쓴다.

★ 여기서는 스폰도 로봇도 안 붙인다. 스테이지만 연다.
"""
import os

import omni.usd

STAGE = os.environ.get(
    "PL_STAGE", "/home/rokey/isaac_scenes/Collected_test_3/test_3.usd")
os.environ["PL_STAGE"] = STAGE

omni.usd.get_context().open_stage(STAGE)

print("")
print("=" * 72)
print("[open_test3] 스테이지: %s" % STAGE)
print("")
print("  카메라를 놓고 **Ctrl+S** 로 저장하면 된다. 이 세션은 아무것도 안 붙였다.")
print("")
print("  ★ TF 프레임 이름은 **prim 이름**이 된다 (ROS2PublishTransformTree 규칙).")
print("    카메라 본체는 다섯 다 이름이 `Camera` 라 프레임이 충돌하므로,")
print("    TF 에는 **부모 Xform** 을 넣는다. 그래서 이 구조가 좋다:")
print("")
print("      /World/CamPick_A/Camera     <- 존 A 픽업")
print("      /World/CamPick_B/Camera     <- 존 B")
print("      /World/CamPick_C/Camera     <- 존 C")
print("      /World/CamSpawn/Camera      <- 투입부 (cam1)")
print("      /World/CamLine2/Camera      <- 검증 (cam2) · 이름은 자유")
print("")
print("    이름이 달라도 된다 — 저장만 하면 Camera prim 을 전수 찾아 부모를")
print("    TF 대상으로 등록한다.")
print("=" * 72)
print("")
