cmake_minimum_required(VERSION 3.8)
project(packing_msgs)

# ─────────────────────────────────────────────────────────────────────────────
#  Project Palletris 공용 인터페이스
#  메시지 8 + 서비스 17 = 25개.
#  ★ 아래 목록에서 하나라도 빠지면 그 타입만 조용히 생성되지 않고,
#    다른 사람이 import 할 때가 되어서야 에러가 난다. 반드시 전부 나열할 것.
# ─────────────────────────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# geometry_msgs 는 더 쓰지 않는다 — BoxAtPickup 이 Pose 를 버리고 mm 정수로 갔다 (§3-2).
find_package(builtin_interfaces REQUIRED) # ZoneStatus.msg 의 Time

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── 메시지 8 ──
  "msg/PlacePose.msg"
  "msg/BoxInfo.msg"
  "msg/BoxAtPickup.msg"
  "msg/RobotCmd.msg"
  "msg/RobotEvent.msg"
  "msg/ZoneStatus.msg"
  "msg/MetricsSummary.msg"
  # ★ main 에서 옴 — §3 표에는 없지만 Isaac Action Graph 쪽에서 쓸 수 있어
  #   남겨 둔다 (트랙 A 미결). 지우려면 트랙 A 확인 후.
  "msg/SpawnEvent.msg"

  # ── /db/* 7 + /diverter/* 2 = 9 ──
  "srv/RegisterBox.srv"
  "srv/GetBoxInfo.srv"
  "srv/VerifyBox.srv"
  "srv/SetStatus.srv"
  "srv/MarkPlaced.srv"
  "srv/ZoneQueue.srv"
  # ★ 2026-08-25 추가 — 인식률의 분모를 바로잡기 위한 것이다 (파일 주석 참조).
  "srv/ReportMiss.srv"
  "srv/DiverterRequest.srv"
  # ★ 2026-08-25 — 분기기 3대를 게이트 단위로 운전하기 위한 것 (파일 주석 참조)
  "srv/DiverterDecide.srv"

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

  DEPENDENCIES builtin_interfaces
)

ament_package()
