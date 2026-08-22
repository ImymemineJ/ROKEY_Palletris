# Project Palletris — 스마트 물류창고 로봇팔 3D 패킹 시스템

컨베이어 + 로봇팔로 박스를 구역 분류하고, heightmap 기반 3D 최적화 패킹으로
팔레트에 적재한 뒤 트럭에 상차하는 디지털 트윈 시스템.

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04 LTS |
| 미들웨어 | ROS 2 Jazzy |
| 시뮬레이터 | NVIDIA Isaac Sim 5.1 |
| 로봇 | 두산 H2017 (페데스탈 700mm) |
| 워크스페이스 | `~/palletris_ws` |

## 문서

- `docs/작업명세서.md` — **인터페이스·수치의 정본**
- `docs/설계결정서.md` — 왜 이렇게 정했는지의 근거
- `docs/가이드/공통가이드_전원필독.md` · `docs/가이드/트랙A~D_*.md`

---

## 개발 환경 세팅

### 1. 저장소 받기

```bash
git clone git@github.com:ImymemineJ/Project_Palletris.git ~/palletris_ws
cd ~/palletris_ws
```

### 2. FastDDS 화이트리스트 배치

유선망(10.10.0.1~4)으로만 통신하도록 제한하는 설정이다.
없으면 DDS가 wifi·docker0 등 모든 인터페이스를 뒤져 노드 발견이 불안정해진다.

```bash
mkdir -p ~/.ros
cp scripts/fastdds_whitelist.xml ~/.ros/
```

> 실습실 유선망에 연결하기 전(개인 노트북 단독 작업 등)에는 이 설정이 오히려
> 통신을 막을 수 있다. 그때는 아래 `FASTRTPS_DEFAULT_PROFILES_FILE` 줄만 주석 처리한다.

### 3. `~/.bashrc` 맨 아래에 추가 (작업명세서 §1.1)

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
source $HOME/palletris_ws/install/setup.bash
```

`nano ~/.bashrc` → 맨 아래에 붙여넣기 → **Ctrl+O** → **Enter** → **Ctrl+X**

> ⚠️ **`ROS_DOMAIN_ID` 는 4명 전원 `115` 여야 한다.**
> 값이 다르면 ROS2 통신이 **에러 없이 그냥 안 된다.** 수업 자료에 50과 103으로
> 어긋나게 적혀 있으니 그대로 따르지 말 것. (설계결정서 §10.1)
>
> `install/setup.bash` 는 첫 빌드 전에는 없어서 `No such file` 경고가 뜬다. 빌드하면 사라진다.

### 4. 적용 및 확인

```bash
source ~/.bashrc
echo $ROS_DOMAIN_ID        # → 115
ls ~/.ros/                 # → fastdds_whitelist.xml
```

### 5. 빌드

```bash
cd ~/palletris_ws
colcon build --symlink-install
source install/setup.bash
```

### 6. 자기 브랜치로 이동

```bash
git switch track/sim       # 김학민 / 병직 track/robot / 정인 track/algo / 범진 track/db
```

---

## 트랙 담당

| 트랙 | 담당 | 소유 디렉터리 | 브랜치 |
|---|---|---|---|
| A 씬/에셋 | 김학민 | `src/packing_sim`, `src/packing_common`, `scripts/` | `track/sim` |
| B 로봇제어 | 김병직 | `src/packing_robot` | `track/robot` |
| C 패킹알고리즘 | 정정인 | `src/packing_algo` | `track/algo` |
| D 인터페이스/DB | 민범진 | `src/packing_msgs`, `src/packing_db` | `track/db` |
| 공용 | 합의 필요 | `src/packing_bringup`, `docs/`, `.github/` | — |

## 철칙 4개

1. §2 인터페이스는 Day 1 오전에 4인이 함께 확정한다.
2. 씬 USD 는 김학민만 수정한다. git 이 아니라 `scripts/sync_scene.sh` 로 배포.
3. 모든 숫자는 `src/packing_bringup/config/system.yaml` 에서 읽는다.
4. 어떤 콜백 안에서도 `spin_until_future_complete` 를 쓰지 않는다.
