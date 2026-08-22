"""test2.usd 에 ROS 2 발행 그래프를 넣고 저장한다.

왜
    test2.usd 를 열고 Play 를 눌러도 ros2 topic list 에 아무것도 안 나왔다.
    씬에 ROS 2 노드가 하나도 없어서 발행할 게 없었다. open_test2_ros2.py 가
    런타임에 그래프를 만들어 주긴 했지만 스테이지에 저장하지 않아서, 그
    스크립트로 열 때만 토픽이 나왔다. 씬을 어떻게 열든 나오게 박아 넣는다.

만드는 것  /World/ROS2Bridge (ActionGraph)
    OnPlaybackTick ─┬─> ROS2PublishClock          -> /clock
                    └─> ROS2PublishTransformTree  -> /tf   (박스 큐브 2개)
    ROS2Context 는 useDomainIDEnvVar=True 라 ROS_DOMAIN_ID 환경변수를 따른다
    (팀 값 116). 씬에 도메인을 박아 두면 팀이 값을 바꿀 때 씬도 고쳐야 한다.

    Play 중에만 돈다 (OnPlaybackTick). Stop 상태에서는 토픽이 안 보인다.

확인
    ros2 topic list          # /clock, /tf
    ros2 topic echo /clock

    ~/isaacsim/python.sh add_ros2_graph.py
"""

import os

from isaacsim import SimulationApp

HEADLESS = bool(os.environ.get("PL_HEADLESS", "1") != "0")
simulation_app = SimulationApp({"headless": HEADLESS})

import shutil  # noqa: E402
import time  # noqa: E402

import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

STAGE = os.environ.get(
    "PL_STAGE", "/home/rokey/isaac_scenes/project_1/test2.usd")
GRAPH_PATH = "/World/ROS2Bridge"

# /tf 로 내보낼 대상. test2.usd 에서 움직이는 것은 이 큐브 둘뿐이다.
TF_TARGETS = ["/World/Cube", "/World/Cube_01"]

enable_extension("isaacsim.core.nodes")
enable_extension("isaacsim.ros2.bridge")
for _ in range(30):
    simulation_app.update()

import omni.graph.core as og  # noqa: E402  (확장을 켠 뒤에 import)

backup = f"{STAGE}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(STAGE, backup)
print(f"백업: {backup}")

ctx = omni.usd.get_context()
ctx.open_stage(STAGE)
for _ in range(60):
    simulation_app.update()
stage = ctx.get_stage()

# 이미 있으면 지우고 다시 만든다 (멱등).
if stage.GetPrimAtPath(GRAPH_PATH):
    stage.RemovePrim(GRAPH_PATH)
    print(f"기존 {GRAPH_PATH} 제거 후 재생성")

targets = [p for p in TF_TARGETS if stage.GetPrimAtPath(p).IsValid()]
if len(targets) != len(TF_TARGETS):
    missing = set(TF_TARGETS) - set(targets)
    print(f"경고: {missing} 가 씬에 없다 — /tf 대상에서 뺀다")

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        keys.SET_VALUES: [
            # 도메인은 씬에 박지 않고 ROS_DOMAIN_ID 환경변수를 따른다.
            ("Context.inputs:useDomainIDEnvVar", True),
            ("PublishClock.inputs:topicName", "clock"),
            ("PublishTF.inputs:topicName", "tf"),
            ("PublishTF.inputs:parentPrim", [stage.GetPrimAtPath("/World").GetPath()]),
            ("PublishTF.inputs:targetPrims",
             [stage.GetPrimAtPath(p).GetPath() for p in targets]),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("Context.outputs:context", "PublishTF.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
        ],
    },
)
for _ in range(10):
    simulation_app.update()

print(f"생성: {GRAPH_PATH}")
for c in stage.GetPrimAtPath(GRAPH_PATH).GetChildren():
    print(f"   {c.GetName():<16s} {c.GetTypeName()}")
print(f"   /tf 대상: {targets}")

ctx.save_stage()
for _ in range(20):
    simulation_app.update()
print(f"저장 완료: {STAGE}")
print("\n확인:  Play 후 다른 터미널에서")
print("   ros2 topic list        # /clock, /tf")
print("   ros2 topic echo /clock")

simulation_app.close()
