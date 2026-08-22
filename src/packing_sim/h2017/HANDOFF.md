# H2017 을 test1-1.usd 에 물리기 — 인수인계

작성 2026-08-22. 목표: **다른 사람이 만든 H2017 이 오면 그대로 물려서 도는
상태로 `test1-1.usd` 를 준비해 두는 것.**

## 결론부터

메시는 **더 안 받아도 된다.** URDF 가 참조하는
`package://dsr_description2/meshes/h2017_{white,collision}/*.dae` 가 이 PC 에
없지만, 그건 URDF→USD 를 **새로 임포트할 때만** 필요하다. 이미 임포트된
USD 가 있고 관절 한계가 URDF 와 정확히 일치하는 것을 확인했다.

## 지금까지 한 것

### 1. 받은 파일 설치  `src/packing_sim/h2017/`

```
urdf/h2017.urdf                  joint_2 effort=1000 (상류 372 는 자리표시자)
config/h2017_descriptor.yaml     Lula descriptor. default_q [0,0,-1.5708,0,-1.5708,0]
config/h2017_rmpflow_common.yaml RMPflow 설정 (1차에서는 안 씀 — 보관용)
rmpflow_controller.py            위와 같음. 보관용
tools/fk.py                      URDF 원점만으로 도는 정기구학 (numpy 불필요)
tools/import_h2017_usd.py        URDF→USD 임포터. **메시가 없어 못 돌린다**
make_h2017_usd.py                ← 새로 만든 것. 아래 참조
```

### 2. `~/isaac_scenes/robot/h2017.usd` 생성 (완료 · 검증됨)

`tools/import_h2017_usd.py` 를 못 돌리므로, 기존
`project_1/SubUSDs/h2017.usd` 를 URDF·system.yaml 값으로 교정했다.

```
revolute 6개 · tool0 있음 (/Root/h2017/tool0) · articulation root 정상
joint      lower    upper   maxForce  stiffness  damping
joint_1  -360.00   360.00      372.0     1000.0    100.0
joint_2  -125.00   125.00     1000.0     1000.0    100.0   ← URDF effort
joint_3  -160.00   160.00      372.0     1000.0    100.0
joint_4  -360.00   360.00      163.0     1000.0    100.0
joint_5  -360.00   360.00       96.0     1000.0    100.0
joint_6  -360.00   360.00       50.0     1000.0    100.0
```

관절 한계는 **원래부터 URDF 와 일치**했다 (joint_2 ±125.00° = ±2.1817 rad,
joint_3 ±160.00° = ±2.7925). 사양서 6-3 이 경고한 "m0609 한계가 들어가는"
문제는 없다. 바꾼 건 maxForce 와 드라이브 게인, articulation solver 반복수
(system.yaml `physics.articulation` 32/4) 뿐이다.

## 남은 일

**로봇을 `test1-1.usd` 에 실제로 넣는 것.** 아직 안 했다.

필요한 것:
1. `~/isaac_scenes/robot/h2017.usd` 를 `/World/robot` 으로 reference
2. 페데스탈 (원기둥 D400 × 높이 H) 추가
3. 배치 좌표 결정 ← **여기가 막힌 지점, 아래 참조**
4. 검증: articulation 로드 · Lula IK 가 벨트 위 픽 지점을 푸는지

## ★ 먼저 정해야 하는 것 — 좌표계가 두 개다

`test1-1.usd` 의 실측값과 `system.yaml` 의 값이 **다른 세계**다.

|  | test1-1.usd 실측 | system.yaml |
|---|---|---|
| 벨트 상면 | **1780.5 mm** | `conveyor.belt_top_z_mm: 800` |
| 라인 위치 | x ≈ −9.5 m, y 0→17.2 m | x 0→11.2 m |
| 로봇 베이스 | (없음) | `[10900, 0, 500]` mm |
| 페데스탈 | (없음) | `robot.pedestal_height_mm: 500` |

이 상태로는 로봇을 "명세대로" 놓을 수가 없다. 벨트가 1780 인데 페데스탈이
500 이면 베이스에서 벨트까지 1.28 m 라 H2017(리치 1.7 m)이 매우 어색하게 뻗는다.

**둘 중 하나를 정해야 한다.**
- (A) 씬을 명세에 맞춘다 — 벨트를 800 mm 로. 예전에 900 mm 로 낮추려다 되돌린
  적이 있다 (A06/A03 은 벨트면이 원래 1780 이라 조각을 내리면 프레임이 지하에
  묻힌다. 낮은 계열 A05/A02 로 교체 + z 스케일이 필요하다 →
  `set_belt_900.py` 에 그 방법이 다 들어 있다).
- (B) 씬 좌표를 그대로 두고 로봇만 실측 벨트에 맞춰 놓는다. 그러면
  `system.yaml` 값과 어긋나므로 철칙 3 위반이고, 나중에 다른 트랙과 좌표가
  안 맞는다.

**김학민이 결정할 사항이고, 팀(특히 로봇 트랙)과 맞춰야 한다.**

## 실행할 때 걸린 것들 (다시 밟지 말 것)

- **`make_h2017_usd.py` 는 `~/isaacsim/python.sh` 로 돌린다.** `pxr` 과 `yaml`
  을 한 인터프리터에서 같이 써야 하는데, `kit/python` 에는 yaml 이 없고
  `python.sh` 는 `SimulationApp()` 을 띄우기 전에는 pxr 이 없다. 그래서
  스크립트가 SimulationApp 을 먼저 띄운다.
- **그 스크립트가 마지막에 안 끝나고 매달린다.** USD 는 이미 저장된 뒤라
  결과물은 정상이다. `_app.close()` 부근에서 멈추는 것으로 보인다. 급하면
  파일 생성 확인 후 죽여도 된다.
- ROS 2 브리지를 쓸 때 **`/opt/ros/jazzy` 를 source 하면 안 된다** (python 3.12
  rclpy 가 잡혀 실패). `export ROS_DISTRO=jazzy` +
  `LD_LIBRARY_PATH=...:$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib` 를 쓴다.
- **GUI 에서 Ctrl+S 하면 씬이 깨질 수 있다.** Conveyor 확장이 중복 그래프와
  `PhysicsRigidBodyAPI` 를 박아 넣어 컨베이어와 바닥을 동적 강체로 만든 적이
  있다. 저장 후 반드시:
  `cd ~/palletris_ws/src/packing_sim && ./run_fix_dup_conveyor.sh`

## 파일 위치

```
씬        ~/isaac_scenes/project_1/test1-1.usd     (test1.usd 사본, 소터 없음)
로봇      ~/isaac_scenes/robot/h2017.usd           ← 새로 만든 것
코드      ~/palletris_ws/src/packing_sim/
씬 배포   ~/palletris_ws/scripts/sync_scene.sh     (USD 는 git 에 안 들어간다)
```

여는 법:
```bash
cd ~/isaacsim && ./isaac-sim.sh --exec ~/palletris_ws/src/packing_sim/open_test1_1.py
```
