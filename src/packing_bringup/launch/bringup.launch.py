#!/usr/bin/env python3
"""Palletris ROS 2 그래프 기동.

    ros2 launch packing_bringup bringup.launch.py
    ros2 launch packing_bringup bringup.launch.py use_sim_time:=false
    ros2 launch packing_bringup bringup.launch.py vision:=false      # 인식 끄고

    # ★ 기본값은 stage 2 (3구역, system.yaml stage:2 와 짝). 예전 1구역(B만) 으로
    #   돌리려면 zones · pick_cam_ns 를 같이 좁혀야 한다 — 하나만 바꾸면
    #   관제탑과 인식 노드가 서로 다른 /pick_cam_* 토픽을 봐서 아무것도 안 온다.
    ros2 launch packing_bringup bringup.launch.py \
        zone:=b zones:="['b']" pick_cam_ns:="['/pick_cam']"

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

⚠⚠ use_sim_time:=true 인데 /clock 발행자가 없으면 **모든 타이머가 멎는다.**
  타임스탬프만 0 이 되는 게 아니다 — ROS 시계가 0 에 멈춰 create_timer 가
  영원히 안 뜬다. 실측으로 확인했다:
    · ZoneStatus 2 Hz 발행 안 됨
    · /metrics/summary 1 Hz 발행 안 됨
    · ★ 관제탑의 로봇 20초 타임아웃 감시도 안 돎 — 로봇이 응답을 안 해도
      영원히 기다린다. "예외 처리(크래시 없음)" 가 통째로 무력해진다.
  노드는 멀쩡히 살아 있고 에러도 안 난다. 그래서 원인 찾기가 제일 어렵다.

  → Isaac(Action Graph)을 **먼저** 띄우고 이 launch 를 올린다.
  → Isaac 없이 ROS 그래프만 볼 때는 use_sim_time:=false.
  → scripts/verify_ros_graph.py 는 /clock 을 스스로 내주므로 true 로 둬도 된다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_list(v):
    """launch 인자 문자열 "['a','b']" -> ["a","b"]. 한 개짜리 "b" 도 받는다."""
    import ast
    v = str(v).strip()
    if v.startswith("["):
        return [str(x) for x in ast.literal_eval(v)]
    return [x for x in v.replace(",", " ").split() if x]


def generate_launch_description():
    zone = LaunchConfiguration("zone")
    zones = LaunchConfiguration("zones")
    pick_cam_ns = LaunchConfiguration("pick_cam_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    args = [
        # ★ zone(단수)은 zones 가 비었을 때만 쓰는 폴백이다 — 관제탑이 zones 를
        #   우선한다 (orchestrator_node._init_cycles). 기본은 zones 가 채워져
        #   있으니 실제로는 안 쓰인다.
        DeclareLaunchArgument("zone", default_value="b",
                              description="zones 가 비었을 때만 쓰는 폴백 구역"),
        # ★ 구역 인식은 **구역마다 프로세스를 따로** 띄운다 (CLAUDE.md §8 "3").
        #   관제탑과 다르다 — 관제탑은 상태머신이라 하나여야 명령이 안 겹치지만,
        #   인식은 이미지 추론이라 한 프로세스에 몰면 파이썬 GIL 때문에 세 구역이
        #   줄을 서서 돈다. 프로세스를 나눠야 코어를 실제로 나눠 쓴다.
        #   1구역(stage 1)으로 좁히려면:
        #     zone:=b zones:="['b']" pick_cam_ns:="['/pick_cam']"
        #   ⚠ 두 목록의 **순서와 개수가 짝**이어야 한다. 어긋나면 다른 구역
        #     카메라로 좌표를 낸다 — 아래 _zone_detectors 가 시작할 때 막는다.
        #   ⚠ system.yaml 의 stage 값과 반드시 맞춰야 한다 — 하네스
        #     (verify_ros_graph.py)는 stage 를 보고 카메라 토픽을 정하는데
        #     여기 기본값과 어긋나면 관제탑이 다른 이름의 토픽을 구독해
        #     이미지가 하나도 안 온다 (2026-08-25 실측 발견).
        DeclareLaunchArgument(
            "zones", default_value="['a','b','c']",
            description="인식 노드가 볼 구역 목록. system.yaml stage:2 와 짝"),
        DeclareLaunchArgument(
            "pick_cam_ns", default_value="['/pick_cam_a','/pick_cam_b','/pick_cam_c']",
            description="zones 와 같은 순서의 픽업 카메라 네임스페이스"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="규약 1. Isaac 없이 돌릴 때만 false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("algo", default_value="true"),
        DeclareLaunchArgument("db", default_value="true"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("orchestrator", default_value="true"),
        # ★★ 3 구역 씬(test_3)은 좌표계가 통째로 다르다 — 팔레트가 셀 컨베이어
        #   위(z 950)고 페데스탈이 1950 이며 최대 적재가 1000 이다. 그 값들이
        #   `config/test_3.yaml` 에 있고 `system.yaml` **위에 겹쳐서** 쓴다
        #   (오버레이라 단독으로는 불완전하다 — `packing_common/config.py` 참조).
        #
        #   ⚠ 이걸 안 주면 **에러 없이 어긋난다.** 관제탑·algo 는 test1-1 좌표로
        #     계획하고 Isaac 은 test_3 좌표로 움직인다 — 엉뚱한 자리에 놓는데
        #     아무 경고도 안 뜬다. 그래서 상속에만 맡기지 않고 인자로 받는다.
        #
        #   ros2 launch packing_bringup bringup.launch.py \
        #       config_overlay:=$PWD/src/packing_bringup/config/test_3.yaml
        #   기본값은 **상속된 환경변수**다 — 터미널에서 export 해 둔 경우 그게 살고,
        #   인자를 주면 그쪽이 이긴다. 빈 문자열을 그냥 넘기면 export 한 값을
        #   덮어써서 꺼 버린다.
        DeclareLaunchArgument(
            "config_overlay",
            default_value=os.environ.get("PL_CONFIG_OVERLAY", ""),
            description="system.yaml 위에 겹칠 오버레이 yaml (3구역은 test_3.yaml)"),
    ]

    # 모든 노드가 같은 파라미터를 받게 한 곳에 모은다.
    common = [{"use_sim_time": use_sim_time}]
    cli = ["--ros-args", "--log-level", log_level]

    def _overlay_env():
        """`config_overlay` 를 노드 환경변수로 넘긴다.

        ★ 인자의 기본값이 상속된 `PL_CONFIG_OVERLAY` 라, 안 주면 터미널에서
          export 한 값이 그대로 간다. 값이 없으면 빈 문자열이 가는데
          `config.load()` 가 falsy 로 보고 오버레이를 안 건다.
        """
        return {"PL_CONFIG_OVERLAY": LaunchConfiguration("config_overlay")}

    def node(pkg, exe, cond, extra=None):
        # ★ name= 을 주지 않는다.
        #   vision_node 는 한 프로세스에 노드 둘(line_detector · zone_detector)을
        #   띄운다. launch 가 name 을 주면 **둘 다** 그 이름으로 덮어써서 그래프에
        #   같은 이름이 두 개 생긴다 ("nodes that share an exact name" 경고).
        #   그러면 파라미터·서비스가 어느 쪽 것인지 알 수 없어진다.
        #   각 노드가 스스로 붙인 이름을 그대로 쓰게 둔다.
        return Node(package=pkg, executable=exe, output="screen",
                    parameters=common + (extra or []),
                    arguments=cli,
                    additional_env=_overlay_env(),
                    condition=IfCondition(LaunchConfiguration(cond)))

    def _zone_detectors(context, *_a, **_k):
        """zones / pick_cam_ns 를 실행 시점에 펼쳐 구역마다 프로세스를 만든다.

        launch 인자는 문자열이라 선언 시점에는 개수를 모른다. OpaqueFunction 으로
        값을 받은 뒤에야 노드를 몇 개 만들지 정할 수 있다.
        """
        zs = _as_list(zones.perform(context))
        cs = _as_list(pick_cam_ns.perform(context))
        if len(zs) != len(cs):
            raise RuntimeError(
                f"zones {zs} 와 pick_cam_ns {cs} 의 개수가 다르다. "
                "짝이 어긋나면 다른 구역 카메라로 좌표를 낸다")
        return [Node(package="packing_vision", executable="zone_detector",
                     name=f"detector_zone_{z}", output="screen",
                     parameters=common + [{"zone": z, "pick_cam_ns": c}],
                     arguments=cli,
                     additional_env=_overlay_env(),
                     condition=IfCondition(LaunchConfiguration("vision")))
                for z, c in zip(zs, cs)]

    return LaunchDescription(args + [GroupAction([
        # 알고리즘 — heightmap 3장을 소유한다. 로봇 명령은 절대 안 낸다.
        node("packing_algo", "algo_service_node", "algo"),

        # DB + 중재자 — palletris.db 의 유일한 writer.
        # 두 프로세스가 같은 .db 를 열면 `database is locked` 가 난다.
        node("packing_db", "db_service_node", "db"),

        # 투입부 인식 — cam1 등록 · cam2 검증. 라인이 하나라 이것도 하나다.
        node("packing_vision", "vision_node", "vision"),

        # 관제탑 — ★ 로봇 명령을 내는 유일한 노드다 (CLAUDE.md §8).
        #   구역이 3곳이어도 **프로세스도 노드도 하나**다. 구역별 상태만 늘어난다.
        #   인식 노드와 반대다 — 인식은 이미지 추론이라 GIL 때문에 나눠야 하지만,
        #   관제탑은 상태머신이라 하나여야 명령이 겹치지 않는다.
        node("palletris_orchestrator", "orchestrator_node", "orchestrator",
             [{"zone": zone, "zones": ParameterValue(zones, value_type=None)}]),

        # 구역 인식 — 구역 수만큼 프로세스가 생긴다.
        OpaqueFunction(function=_zone_detectors),
    ])])
