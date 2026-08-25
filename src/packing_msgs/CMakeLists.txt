cmake_minimum_required(VERSION 3.8)
project(packing_msgs)

# ─────────────────────────────────────────────────────────────────────────────
#  Project Palletris 공용 인터페이스
#  메시지 7 + 서비스 15 = 22개.
#  ★ 아래 목록에서 하나라도 빠지면 그 타입만 조용히 생성되지 않고,
#    다른 사람이 import 할 때가 되어서야 에러가 난다. 반드시 전부 나열할 것.
# ─────────────────────────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)      # BoxAtPickup.msg 의 Pose
find_package(builtin_interfaces REQUIRED) # ZoneStatus.msg 의 Time

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── 메시지 7 ──
  "msg/PlacePose.msg"
  "msg/BoxInfo.msg"
  "msg/BoxAtPickup.msg"
  "msg/RobotCmd.msg"
  "msg/RobotEvent.msg"
  "msg/ZoneStatus.msg"
  "msg/MetricsSummary.msg"

  # ── /db/* 6 + /diverter/* 1 = 7 ──
  "srv/RegisterBox.srv"
  "srv/GetBoxInfo.srv"
  "srv/VerifyBox.srv"
  "srv/SetStatus.srv"
  "srv/MarkPlaced.srv"
  "srv/ZoneQueue.srv"
  "srv/DiverterRequest.srv"

  # ── /algo/* 4 ──
  "srv/PlanPlacement.srv"
  "srv/UpdateHeightmap.srv"
  "srv/CancelPlan.srv"
  "srv/ResetHeightmap.srv"

  # ── /conveyor/* 1 + /sim/* 3 = 4 ──
  "srv/SetVelocity.srv"
  "srv/DespawnBox.srv"
  "srv/SwapPallet.srv"
  "srv/SetSpawnEnabled.srv"

  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_package()
