cmake_minimum_required(VERSION 3.8)
project(packing_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# ZoneStatus.stamp 가 builtin_interfaces/Time 이다.
# geometry_msgs 는 더 쓰지 않는다 — BoxAtPickup 이 Pose 를 버리고 mm 정수로 갔다 (§3-2).
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── 메시지 (CLAUDE.md §3-1 · §3-2) ──
  "msg/PlacePose.msg" "msg/BoxInfo.msg" "msg/BoxAtPickup.msg"
  "msg/RobotCmd.msg" "msg/RobotEvent.msg" "msg/ZoneStatus.msg"
  # SpawnEvent 는 §3 표에 없다. Isaac Action Graph 쪽에서 쓸 수 있어 남겨 둔다 (미결).
  "msg/SpawnEvent.msg"
  # ── 서비스 15 (CLAUDE.md §3-3) ──
  "srv/RegisterBox.srv" "srv/VerifyBox.srv" "srv/GetBoxInfo.srv" "srv/SetStatus.srv"
  "srv/MarkPlaced.srv" "srv/ZoneQueue.srv" "srv/DiverterRequest.srv"
  "srv/PlanPlacement.srv" "srv/UpdateHeightmap.srv" "srv/CancelPlan.srv"
  "srv/ResetHeightmap.srv"
  "srv/SetVelocity.srv" "srv/SwapPallet.srv" "srv/SetSpawnEnabled.srv" "srv/DespawnBox.srv"
  DEPENDENCIES builtin_interfaces)

ament_package()
