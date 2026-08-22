"""H2017 RMPflow controller. **1차 구현에서는 쓰지 않는다 — 보관용이다.**

작업명세서 6.2 가 정한 제어 스택은 **Lula IK + 웨이포인트 보간**이다.
RMPflow(반응형 모션 정책)는 그것과 다른 물건이고, 구 저장소에서 실측한 성질 때문에
1차에는 맞지 않는다 — **적분항이 없어 정상상태 오차가 15 mm 남는다.**
위치 오차로 상태를 전환하면 영원히 안 넘어간다 (`docs/픽앤플레이스_통합_기록.md`).

그래도 지우지 않는 이유: 장애물 회피가 필요해지면(stage 2 에서 팔이 적재물 옆을
지나야 할 때) Lula IK 에는 그 기능이 없고 RMPflow 에는 있다. 설정
(`config/h2017_rmpflow_common.yaml`)은 게인·`body_cylinders`까지 H2017 로 맞춰 둔
상태라, 필요해지면 그날 처음부터 만들지 않아도 된다.

---

원본 주석:

M0609 판(`M0609/jik/rmpflow/m0609_rmpflow_controller.py`)과 구조가 같다.
바뀐 곳은 세 군데다.

  1. 설정 파일이 H2017 것을 가리킨다.
  2. **URDF 는 반드시 h2017.urdf 를 물린다.** m0609.urdf 를 물린 채 yaml 만 바꾸면
     joint_2 한계(+-2.1817)가 M0609 의 +-6.28 로 들어가 IK 가 도달 불가능한 해를 낸다
     (사양서 6-3). 한계는 descriptor 가 아니라 URDF 에서 온다.
  3. EE 프레임 기본값이 `tool0` 이다. 흡착은 툴 z축이 접근 방향이고,
     tool0 의 z축이 그 방향으로 정의돼 있다 (m0609 판은 link_6 을 썼다).

이 모듈은 Isaac 쪽에서만 돈다. **rclpy 를 import 하지 않는다** — isaac_python(3.11)과
ROS 2 Jazzy(3.12)는 파이썬이 달라 같은 프로세스에서 못 섞는다.
관제탑과의 통신은 OmniGraph ROS2 노드나 별도 브리지 프로세스로 붙인다.
"""

from pathlib import Path

import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation

# packing_robot/packing_robot/ -> packing_robot/{urdf,config}/
BASE_DIR = Path(__file__).resolve().parent
DEFAULT_URDF = BASE_DIR / "../urdf/h2017.urdf"
DEFAULT_DESCRIPTION = BASE_DIR / "../config/h2017_descriptor.yaml"
DEFAULT_RMPFLOW = BASE_DIR / "../config/h2017_rmpflow_common.yaml"


class RMPFlowController(mg.MotionPolicyController):
    """H2017용 RMPflow controller."""

    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        physics_dt: float = 1.0 / 60.0,
        urdf_path: str | None = None,
        robot_description_path: str | None = None,
        rmpflow_config_path: str | None = None,
        end_effector_frame_name: str = "tool0",
        maximum_substep_size: float = 0.00334,
    ) -> None:
        urdf_path = str(Path(urdf_path) if urdf_path else DEFAULT_URDF.resolve())
        robot_description_path = str(
            Path(robot_description_path)
            if robot_description_path
            else DEFAULT_DESCRIPTION.resolve()
        )
        rmpflow_config_path = str(
            Path(rmpflow_config_path) if rmpflow_config_path else DEFAULT_RMPFLOW.resolve()
        )

        for p in (urdf_path, robot_description_path, rmpflow_config_path):
            if not Path(p).exists():
                raise FileNotFoundError(f"설정 파일이 없다: {p}")
        if "m0609" in Path(urdf_path).name.lower():
            raise ValueError(
                f"H2017 컨트롤러에 M0609 URDF 가 물렸다: {urdf_path}\n"
                "joint_2 한계가 틀리게 들어가 IK 가 도달 불가능한 해를 낸다 (사양서 6-3)."
            )

        self.rmp_flow = mg.lula.motion_policies.RmpFlow(
            robot_description_path=robot_description_path,
            rmpflow_config_path=rmpflow_config_path,
            urdf_path=urdf_path,
            end_effector_frame_name=end_effector_frame_name,
            maximum_substep_size=maximum_substep_size,
        )

        self.articulation_rmp = mg.ArticulationMotionPolicy(
            robot_articulation, self.rmp_flow, physics_dt
        )
        super().__init__(name=name, articulation_motion_policy=self.articulation_rmp)

        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )

    def reset(self):
        super().reset()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )
