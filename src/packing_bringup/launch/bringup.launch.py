#!/usr/bin/env python3
"""Palletris ROS 2 그래프 기동.

    ros2 launch packing_bringup bringup.launch.py
    ros2 launch packing_bringup bringup.launch.py zone:=b use_sim_time:=false
    ros2 launch packing_bringup bringup.launch.py vision:=false      # 인식 끄고

띄우는 것 (Isaac 밖의 rclpy 노드 전부)
    algo_service_node    /algo/*        4개   (트랙 C)
    db_service_node      /db/* + /diverter/request  7개 (트랙 D)
    vision_node          cam1·cam2·pick_cam        (트랙 D)
    orchestrator_node    관제탑 — /robot/cmd 의 유일한 발신자 (트랙 B)

Isaac(Action Graph)이 내는 것은 여기서 안 띄운다 — /clock · /tf · 카메라 영상 ·
씬 서비스(/conveyor/set_velocity · /sim/*).

★ use_sim_time 을 launch 가 강제한다 (CLAUDE.md §3-4 규약 1)
  빠뜨리면 tf 조회가 전부 "lookup would require extrapolation" 으로 실패하는데,
  rclpy 가 콜백 예외를 삼켜서 **에러 한 줄 없이** 박스가 영영 안 온다.
  노드를 손으로 띄우면 매번 붙여야 하니 launch 한 곳에서 건다.

⚠ use_sim_time:=true 인데 /clock 발행자가 없으면 ROS 시계가 0 에 멈춘다.
  Isaac 없이 ROS 그래프만 돌려볼 때는 use_sim_time:=false 로 두거나,
  scripts/verify_ros_graph.py 처럼 /clock 을 내주는 대역을 같이 띄운다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    zone = LaunchConfiguration("zone")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    args = [
        DeclareLaunchArgument("zone", default_value="b",
                              description="관제탑이 맡을 구역. stage 1 은 b 하나다"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="규약 1. Isaac 없이 돌릴 때만 false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("algo", default_value="true"),
        DeclareLaunchArgument("db", default_value="true"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("orchestrator", default_value="true"),
    ]

    # 모든 노드가 같은 파라미터를 받게 한 곳에 모은다.
    common = [{"use_sim_time": use_sim_time}]
    cli = ["--ros-args", "--log-level", log_level]

    def node(pkg, exe, cond, extra=None):
        # ★ name= 을 주지 않는다.
        #   vision_node 는 한 프로세스에 노드 둘(line_detector · detector_zone_b)을
        #   띄운다. launch 가 name 을 주면 **둘 다** 그 이름으로 덮어써서 그래프에
        #   같은 이름이 두 개 생긴다 ("nodes that share an exact name" 경고).
        #   그러면 파라미터·서비스가 어느 쪽 것인지 알 수 없어진다.
        #   각 노드가 스스로 붙인 이름을 그대로 쓰게 둔다.
        return Node(package=pkg, executable=exe, output="screen",
                    parameters=common + (extra or []),
                    arguments=cli,
                    condition=IfCondition(LaunchConfiguration(cond)))

    return LaunchDescription(args + [GroupAction([
        # 알고리즘 — heightmap 3장을 소유한다. 로봇 명령은 절대 안 낸다.
        node("packing_algo", "algo_service_node", "algo"),

        # DB + 중재자 — palletris.db 의 유일한 writer.
        # 두 프로세스가 같은 .db 를 열면 `database is locked` 가 난다.
        node("packing_db", "db_service_node", "db"),

        # 인식 — cam1 등록 · cam2 검증 · pick_cam 픽업 좌표.
        node("packing_vision", "vision_node", "vision",
             [{"zone": zone}]),

        # 관제탑 — ★ /robot/cmd 를 내는 유일한 노드다 (CLAUDE.md §8).
        #   구역이 3곳이어도 이 프로세스를 늘리지 않는다.
        node("palletris_orchestrator", "orchestrator_node", "orchestrator",
             [{"zone": zone}]),
    ])])
