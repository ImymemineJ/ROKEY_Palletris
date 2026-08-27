# Project Palletris

창고 자동 팔레타이징 시뮬레이션. 컨베이어로 오는 박스를 카메라로 인식하고,
로봇 팔 3대가 3D bin-packing 알고리즘이 계산한 자리에 쌓은 뒤, 가득 찬 팔레트를
컨베이어로 트럭까지 밀어 넣는다.

Isaac Sim 5.1 + ROS 2 Jazzy · 두산 H2017 3대 · 다중 PC ROS 2 통신.

> 설계 근거와 규약은 [`CLAUDE.md`](CLAUDE.md), 실측 기록은 [`docs/`](docs/) 에 있다.
> **숫자의 정본은 `src/packing_bringup/config/system.yaml`** 이고, 시연 조건은
> `test_3.yaml` 오버레이가 덮어쓴다.

---

## 1. 요구 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04 |
| 미들웨어 | ROS 2 **Jazzy** (시스템 설치 — `/opt/ros/jazzy`) |
| 시뮬레이터 | Isaac Sim **5.1** (`~/isaac-sim-5.1.0` 또는 `~/isaacsim`) |
| 파이썬 패키지 | `numpy` · `opencv-python`(cv2) · `scipy` · `pyyaml` · `cv_bridge` |

`ros-jazzy-desktop` 을 설치했다면 대부분 들어 있다. 빠진 것만:

```bash
sudo apt install -y python3-opencv python3-scipy ros-jazzy-cv-bridge
```

> ⚠ **Isaac 내장 ROS 2 를 쓰면 안 된다.** `packing_msgs` 는 `/opt/ros/jazzy` 에
> 링크돼 있어서, 내장 쪽에 물리면 fastcdr ABI 불일치로 죽는다
> (`undefined symbol: ...eprosima7fastcdr...`). 실행 스크립트가 알아서 시스템 ROS 를
> 고르므로 스크립트를 통해 실행할 것.

---

## 2. 설치 — 네 단계

### ① 클론 · 빌드

```bash
git clone <저장소 URL> ~/palletris_ws
cd ~/palletris_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
mkdir -p ~/palletris_logs        # 실행 로그는 저장소 밖에 쌓인다
```

빌드가 `symbolic link ... Is a directory` 로 실패하면 예전 빌드 산출물이 남은 것이다:

```bash
rm -rf build install log && colcon build --symlink-install
```

### ② 생성물 2종 만들기 (★ 빠뜨리면 조용히 안 돈다)

`palletris.db` 와 `qr_labels/` 는 **생성물이라 git 에 없다.** 둘 다 저장소 안의
정답지 CSV 에서 만들어진다:

```bash
cd ~/palletris_ws
python3 scripts/gen_master_db.py --reset     # palletris.db (박스 150개 · 주소 · 구역표)
python3 scripts/gen_qr_labels.py             # qr_labels/BOX001.png ... 150장
```

- 없으면 **에러 없이** 이렇게 된다: 박스에 QR 이 안 붙어 인식이 전멸하고,
  DB 조회가 실패해 관제탑이 전 박스를 폐기한다.
- `palletris.db` 는 **DB 노드를 띄우는 PC**, `qr_labels/` 는 **Isaac 을 띄우는 PC**
  에 있어야 한다.

### ③ Isaac 씬 배치

USD 는 바이너리라 git 에 넣지 않는다(`.gitignore`). 씬 묶음
`Collected_test_3/`(약 1.2 GB, 에셋 포함)을 받아서 이렇게 둔다:

```
~/isaac_scenes/Collected_test_3/test_3.usd
~/isaac_scenes/Collected_test_3/isaac_assets/...
~/isaac_scenes/Collected_test_3/omniverse-content-production.../...
```

다른 경로에 두려면 `PL_STAGE` 로 지정한다:

```bash
export PL_STAGE=/원하는/경로/test_3.usd
```

> ⚠ **에셋 폴더를 통째로 가져와야 한다.** `test_3.usd` 만 복사하면 참조가 안 풀려
> 모든 프림이 `Selected Prim is not a Rigid Body` 로 죽고 로봇 아티큘레이션이
> 사라진다. 팀 내부 배포는 `scripts/sync_scene.sh` (rsync) 를 쓴다.

### ④ 네트워크 (다중 PC 로 돌릴 때만)

```bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# 이미지가 커서 UDP 단편 손실이 난다 — 송수신 버퍼를 키운다
mkdir -p ~/.ros && cp scripts/fastdds_whitelist.xml ~/.ros/
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216
```

> ⚠ `fastdds_whitelist.xml` 은 인터페이스를 `10.10.0.1~4` 로 못 박아 뒀다.
> **다른 대역이면 그 파일의 `<address>` 를 자기 IP 로 고칠 것** — 안 고치면
> 에러 없이 토픽이 하나도 안 보인다. 한 대에서만 돌릴 거면 이 변수를 아예 빼고
> `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 를 쓴다.

---

## 3. 실행

### 한 대에서 전부 (개발·검증용)

터미널 2개가 필요하다. **Isaac 을 먼저 띄운다** — 모든 노드가 `use_sim_time` 이라
`/clock` 발행자가 없으면 타이머가 하나도 안 돈다.

```bash
# 터미널 1 — Isaac
cd ~/palletris_ws && python3 scripts/kill_ros_nodes.py \
  && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  && PL_GTPICK=2 PL_CAMROS=2 PL_SPUR_STOP=2 PL_RENDER_EVERY=10 PL_METER=2 PL_SPAWN=8 \
     ./scripts/run_test3_ros.sh
```

```bash
# 터미널 2 — 인식 · 관제탑 · 알고리즘 · DB
cd ~/palletris_ws && source install/setup.bash \
  && export ROS_DOMAIN_ID=115 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
            RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            PL_CONFIG_OVERLAY=src/packing_bringup/config/test_3.yaml \
  && ros2 launch packing_bringup bringup.launch.py vision:=true depth:=false \
     2>&1 | tee ~/palletris_logs/bringup_$(date +%Y%m%d_%H%M%S).log
```

### 두 대로 나눠서 (시연 구성)

`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` + §2-④ 를 양쪽에 적용한 뒤:

- **PC1 (Isaac)** — 위 터미널 1 과 같되 `LOCALHOST` 를 `SUBNET` 으로
- **PC2 (인식·관제탑·알고리즘·DB)** — 위 터미널 2 와 같되 `LOCALHOST` 를 `SUBNET` 으로

세 대로 나누려면 PC2 는 `vision:=true algo:=false db:=false orchestrator:=false`,
PC3 는 `vision:=false` 를 준다. 배치 근거는
[`docs/2026-08-27_시연_다중PC_배치.md`](docs/2026-08-27_시연_다중PC_배치.md).

### 자주 쓰는 스위치

| 변수 | 뜻 |
|---|---|
| `PL_RENDER_EVERY` | 렌더 건너뛰기. **10 = 약 2.2배속**(검증용) · **3 = 부드러움**(시연용) |
| `PL_OUTFEED=1` | 만재 팔레트를 컨베이어로 **트럭까지** 보내는 연출 (기본 꺼짐) |
| `PL_BG=0` | 장식 배경 끄기. 8 GB GPU 에서 씬이 OOM 으로 죽을 때 |
| `PL_ZONES=B` | 로봇 한 대만 |
| `PL_GUI=0` | 창 없이 (헤드리스) |
| `PL_SPAWN=8` | 박스 투입 간격 [초]. `0` 이면 투입 안 함 |

`bringup.launch.py` 인자: `vision` · `vision_zone` · `depth` · `depth_mode` ·
`algo` · `db` · `orchestrator` · `zone` · `log_level` · `use_sim_time`.

> ⚠ **`PL_GTPICK=2` 와 `vision_zone:=false` 를 같이 쓰면 안 된다.** 픽업 좌표를
> 낼 발행자가 사라져 전 구역이 굶는다. 둘 중 하나만 쓸 것.

---

## 4. 잘 도는지 확인

```bash
source install/setup.bash && export ROS_DOMAIN_ID=115
ros2 topic info /clock                    # Publisher 1 (Isaac 이 살아 있다)
ros2 topic hz /pick_cam_a/rgb             # 이미지가 온다
ros2 topic echo /zone_a/status --once     # 관제탑 상태·적재 개수
```

정상이면 터미널 2 로그에 이런 줄이 이어진다:

```
[orchestrator.a] [①→②] 박스 도착: BOX008 (-7555, 4024) yaw 0.0°
[orchestrator.a] [④→⑤] 집기 성공 (1.5초)
[orchestrator.a] [⑥→①] 적재 3개 · 충전율 35.1% · 최고 280mm
[db_service_node] [지표] 적재 74 · 성공률 100.0% · 사이클 7.1s · 충전율 68.9%
```

## 5. 안 될 때

| 증상 | 원인 |
|---|---|
| 노드는 사는데 타이머가 안 돈다 | `/clock` 발행자 0 — Isaac 을 먼저 띄울 것 |
| 토픽이 안 보이는데 에러도 없다 | `ROS_DOMAIN_ID` 불일치 · 디스커버리 범위 · 화이트리스트 IP |
| 이미지가 하나도 안 온다 | 구독 QoS 가 RELIABLE (센서는 BEST_EFFORT 여야 한다) |
| 카메라 이미지가 캄캄하다 | 돔 라이트 없음 (오프스크린 렌더는 GUI 조명을 못 받는다) |
| 박스에 QR 이 없다 · 전 박스 폐기 | `qr_labels/` 또는 `palletris.db` 미생성 (§2-②) |
| 로드 중 `Killed` | GPU 메모리 부족 — `PL_BG=0` |
| `Selected Prim is not a Rigid Body` 다발 | 씬 에셋 폴더 누락 (§2-③) |
| `database is locked` | `db_service_node` 외의 프로세스가 `.db` 를 열었다 |

더 많은 증상·원인은 [`CLAUDE.md` §10](CLAUDE.md) 에 있다.

---

## 6. 저장소 구조

| 경로 | 트랙 | 무엇 |
|---|---|---|
| `src/packing_common/` | 공용 | `config.py` · `frames.py` · `units.py` (복사 금지, import) |
| `src/packing_msgs/` | 공용 | ROS 인터페이스. 수정하면 팀 전원 재빌드 |
| `src/packing_sim/isaac/` | A | `sim_main.py` · `spawner.py` · `ros_graph.py` · `router.py` |
| `src/packing_robot/` | B | `rmpflow_controller.py` · `cycle.py` · `gripper.py` |
| `src/palletris_orchestrator/` | B | 관제탑 (상태머신 7개, 로봇 명령의 유일한 발신자) |
| `src/packing_algo/` | C | `heightmap_core.py` · `algo_service_node.py` |
| `src/packing_vision/` | D | `vision_node.py` · `detect.py` · `geometry.py` · `depth.py` |
| `src/packing_db/` | D | `db_service_node.py` (`palletris.db` 유일 writer) · 중재자 |
| `src/packing_bringup/` | 공용 | launch · `config/system.yaml` · `config/test_3.yaml` |
| `scripts/` | — | 실행 · 생성 · 검증 스크립트 |
| `docs/` | — | 설계 결정과 실측 기록 |
