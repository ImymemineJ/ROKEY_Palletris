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
rmpflow_controller.py            RMPflow 래퍼. 1차에서는 안 쓴다 — 보관용
tools/fk.py                      URDF 원점만으로 도는 정기구학 (numpy 불필요)
tools/import_h2017_usd.py        URDF→USD 임포터. **메시가 없어 못 돌린다**
make_h2017_usd.py                ← 새로 만든 것. 아래 참조
```

★ 로봇 자산(urdf · descriptor · rmpflow)의 **정본은 `src/packing_robot/` 하나뿐**이다.
  여기 있던 사본 3개는 지웠다 — 둘이 갈라져서 rmpflow 설정의 페데스탈 원통이
  한쪽은 -0.500, 정본은 -0.900 이었다 (400 mm 차). `rmpflow_controller.py` 도
  정본을 가리키게 고쳤다. 사본을 다시 만들지 마라.

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
2. ~~페데스탈 (원기둥 D400 × 높이 H) 추가~~ ← 끝남 (`/World/Pedestal`)
3. ~~배치 좌표 결정~~ ← **끝남. (10.9, 0, 0.9). 아래 참조**
4. 검증: articulation 로드 · Lula IK 가 벨트 위 픽 지점을 푸는지

## ★ 좌표계 — 정해졌다 (2026-08-22, 김학민)

**(A) 로 결정. 씬을 `system.yaml` 에 맞췄다.** 아래 "두 개다" 절은 결정 전
기록이라 참고용으로만 남긴다.

**현재 씬 수치를 여기에 적지 않는다.** 정본은 `src/packing_bringup/config/system.yaml`
하나이고, 씬은 `src/packing_sim/run_layout_to_spec.sh` 가 그걸 읽어 매번 다시 만든다.
여기에 사본을 적어 두면 반드시 갈라진다 — 실제로 갈라져서, 이 자리에 2026-08-22 까지
`페데스탈 H700 · 상면 z=700`, `스토퍼 없다`, `팔레트 x 11200~12000`, `픽 지점 x=9700`
이 "현재 실측값" 이라고 적혀 있었다. 넷 다 지금 씬과 다르다 (페데스탈은 900 이고,
스토퍼 벽은 실물로 x 10000~10100 에 서 있으며, 팔레트는 x 10300~11500 이고,
박스 앞면은 x=10000 에 선다). 로봇 베이스 z 를 0.7 로 읽으면 IK 와 도달 판정이
통째로 200 mm 어긋난다.

지금 값을 보려면:

```bash
python3 -c "import yaml;c=yaml.safe_load(open('src/packing_bringup/config/system.yaml'));print(c['zones']['robot_base_world_mm'],c['robot']['pedestal_height_mm'],c['conveyor'])"
```

씬을 다시 깔려면 `./src/packing_sim/run_layout_to_spec.sh` (백업을 뜨고 재현한다).
실측 요약은 `src/packing_sim/STATUS.md` 가 매 작업마다 갱신한다.

## 벨트 간헐 구동 — 스토퍼를 없앴다 (2026-08-22)

박스를 벽으로 막지 않는다. **선두 박스가 픽 지점에 오면 벨트를 세우고,
로봇이 적재를 끝내고 원위치하면 다시 돌린다.** 픽 지점이 항상 같으므로
로봇은 고정된 자세 하나로 집으면 된다. `belt_cycle.py` 가 그 상태 기계다.

    RUN -> (선두 박스 x >= 픽지점 - 40mm) -> 벨트 정지
        -> SETTLE (완전히 멎을 때까지) -> PICK (로봇 사이클)
        -> 로봇 원위치 -> RUN

**로봇 트랙이 할 일은 두 함수를 주는 것뿐이다.**

```python
import belt_cycle
belt_cycle.set_robot(on_pick=start_cycle, is_idle=is_home)
#   on_pick(box_path, (x, y, z))  픽 지점에 박스가 섰을 때 한 번 불린다
#   is_idle() -> bool             적재 끝내고 원위치했으면 True
belt_cycle.start()
```

안 주면 **스텁**이 돈다 (박스를 키네마틱으로 바꿔 팔레트로 옮긴다,
기본 10초/사이클). 벨트 타이밍만 눈으로 볼 때 쓴다.

주의할 점
- **정지 선행 40mm** (`PL_STOP_LEAD`). 벨트를 세워도 박스가 관성으로 그만큼
  더 간다. 0.5 m/s 에서 잰 값이라 **속도를 바꾸면 다시 재야 한다.**
- **막는 벽이 없다.** 사이클이 멈추면 박스가 x=10000 끝에서 떨어진다.
  `belt_cycle` 이 안 돌고 있으면(`PL_NO_CYCLE=1`) 그렇게 된다.
- `system.yaml` 의 `stopper_x_mm: 10000` 은 이제 **라인 끝 좌표**로만 쓴다.
  실물 벽은 없다. 다른 트랙과 이 해석을 맞춰야 한다.
- 피더 흔들림은 `open_test1_1.py` 가 기본으로 끈다 (`PL_JITTER=0`) — 박스가
  벨트 한가운데에 반듯하게 온다. 인식/파지를 시험하려면 `PL_JITTER=1`.

## 컨베이어 조각을 만질 때 (두 번 틀렸던 것)

- **`inputs:direction` 은 로컬 축 기준이다.** 조각의 `xformOp:orient` 가
  180° 라 로컬 +X 가 월드 −X 다. **월드 +X 로 흘리려면 `direction.x = -1`.**
  +1 로 두면 박스가 뒤로 끌려가 입구 끝(x=0)에서 떨어진다. 이걸 한동안
  "벨트가 좁아 박스가 튕겨 나간다" 로 오진했다. 폭 문제가 아니었다.
- **`inputs:animateDirection` 은 그림만 돌린다.** 물리 방향과 부호를 반드시
  맞춰라. 어긋나면 눈에는 반대로 도는 것처럼 보인다.
- **월드 속도 = `graph:variable:Velocity` × `scale.x`.** 사양 0.5 m/s 를
  내려면 Velocity = 0.5 / scale.x (지금 scale.x 2.5161 → 0.198724).
  예전 메모의 "A05 는 이득 1.0" 은 오측이었다.
- **GUI 에서 Ctrl+S 하면 스크립트로 넣은 것이 통째로 날아간다.** 2026-08-22
  에 페데스탈·팔레트·재배치가 전부 되돌아갔다. GUI 로 조각을 손봤으면
  저장 뒤 `./run_layout_to_spec.sh` 를 다시 돌려라 (GUI 에서 정한 폭
  `scale.y` 와 조각 개수는 스크립트가 그대로 살린다).

## (참고, 결정 전 기록) 좌표계가 두 개였다

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
