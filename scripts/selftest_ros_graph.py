#!/usr/bin/env python3
"""`ros_graph.py` 를 **씬 없이** 확인한다 — 빈 스테이지에 그래프만 세운다.

씬(test1-1.usd)이 없는 PC 에서도 통신 계층을 끝까지 볼 수 있다.
바깥 셸에서:

    ros2 service call /conveyor/set_velocity packing_msgs/srv/SetVelocity \
        "{seg: main, velocity_mps: 0.5}"          -> ok=true, actual_mps=0.5
    ros2 topic echo /robot/event --once            -> seq=99 짜리 이벤트
    ros2 topic pub --once /robot/cmd packing_msgs/msg/RobotCmd "{seq: 7, ...}"

실행 (환경변수가 필요하다 — TS-022):
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib
    source install/setup.bash
    ~/isaacsim/python.sh scripts/selftest_ros_graph.py
"""
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RESULT = ROOT / "ros_graph_selftest.json"
HOLD_S = float(os.environ.get("PL_SELFTEST_HOLD_S", "150"))

# ★ SimulationApp 이 가장 먼저다. 그 전에 omni.* 를 import 하면 없다.
from isaacsim import SimulationApp                                  # noqa: E402
app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension         # noqa: E402
for _e in ("isaacsim.core.nodes", "isaacsim.ros2.bridge", "omni.graph.scriptnode"):
    enable_extension(_e)
app.update()

sys.path.insert(0, str(ROOT / "src/packing_sim/isaac"))
import ros_graph                                                    # noqa: E402
import omni.timeline                                                # noqa: E402
from isaacsim.core.utils.stage import create_new_stage              # noqa: E402

OUT = {}
def save():
    RESULT.write_text(json.dumps(OUT, ensure_ascii=False, indent=1))

# 가짜 핸들러 — 진짜는 sim_main 이 spawner/conveyor_ctl/robot_motion 을 꽂는다.
# 여기서는 **통로만** 본다. 입력이 그대로 오고 응답이 그대로 돌아가는지.
def _sv(**kw):
    print(f"[SELFTEST] set_velocity 수신 = {kw}  타입={{k: type(v).__name__ for k, v in kw.items()}}")
    return {"ok": bool(kw.get("seg")), "actual_mps": float(kw.get("velocity_mps", 0.0))}


ros_graph.set_handlers(
    set_velocity=_sv,
    despawn_box=lambda box_id="": {"ok": str(box_id).startswith("BOX")},
    set_spawn_enabled=lambda enabled=False: {"ok": True},
    swap_pallet=lambda zone="": {"ok": bool(zone)},
)

create_new_stage()
app.update()
ros_graph.build(zone="B")
OUT["build"] = "OK"; save()

omni.timeline.get_timeline_interface().play()
for _ in range(150):                       # ★ 동적 필드가 생길 때까지 돌린다
    app.update()

try:
    ros_graph.wire()
    OUT["wire"] = "OK"
except Exception as e:                                              # noqa: BLE001
    OUT["wire"] = f"FAIL {type(e).__name__}: {e}"
save()

# 발행 경로 확인용 이벤트 하나 (EventGate 가 다음 틱에 내보낸다)
ros_graph.send_event(seq=99, action=1, box_id="BOX099", ok=True, reason="",
                     has_actual=True, actual_x_mm=170, actual_y_mm=675,
                     actual_z_mm=0, actual_yaw_deg=0.0, duration_s=3.5)

t0 = time.time()
while time.time() - t0 < HOLD_S:
    app.update()
    cmd = ros_graph.take_cmd()
    if cmd:
        OUT.setdefault("received_cmds", []).append({k: str(v) for k, v in cmd.items()})
        save()
        # 받은 명령에 그대로 응답해 본다 — seq 되돌리기 규약 (§3-4 2번)
        ros_graph.send_event(seq=int(cmd.get("seq", 0)), action=int(cmd.get("action", 0)),
                             box_id=str(cmd.get("box_id", "")), ok=True, reason="",
                             has_actual=True,
                             actual_x_mm=int(cmd.get("x_mm", 0)),
                             actual_y_mm=int(cmd.get("y_mm", 0)),
                             actual_z_mm=int(cmd.get("z_mm", 0)),
                             actual_yaw_deg=float(cmd.get("yaw_deg", 0.0)),
                             duration_s=1.0)
OUT["done"] = True; save()
app.close()
