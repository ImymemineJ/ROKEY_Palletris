cmake_minimum_required(VERSION 3.8)
project(packing_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── 메시지 5 (작업명세서 §2.1) ──
  "msg/PlacePose.msg" "msg/BoxInfo.msg" "msg/SpawnEvent.msg"
  "msg/ZoneStatus.msg" "msg/BoxAtPickup.msg"
  # ── 서비스 15 (작업명세서 §2.2) ──
  "srv/RegisterBox.srv" "srv/VerifyBox.srv" "srv/GetBoxInfo.srv" "srv/SetStatus.srv"
  "srv/MarkPlaced.srv" "srv/ZoneQueue.srv" "srv/DiverterRequest.srv"
  "srv/PlanPlacement.srv" "srv/UpdateHeightmap.srv" "srv/CancelPlan.srv" "srv/ResetZone.srv"
  "srv/SetVelocity.srv" "srv/SwapPallet.srv" "srv/SetSpawnEnabled.srv" "srv/DespawnBox.srv"
  DEPENDENCIES geometry_msgs)

ament_package()
