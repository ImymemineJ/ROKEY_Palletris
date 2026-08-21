# 스마트 물류창고 로봇팔 3D 패킹 시스템

컨베이어 + 로봇팔로 박스를 구역 분류하고, heightmap 기반 3D 최적화 패킹으로
팔레트에 적재한 뒤 트럭에 상차하는 디지털 트윈 시스템.

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04 LTS |
| 미들웨어 | ROS 2 Jazzy |
| 시뮬레이터 | NVIDIA Isaac Sim 5.1 |
| 로봇 | 두산 H2017 (페데스탈 500mm) |
| `ROS_DOMAIN_ID` | **115** (전원 동일) |

## 문서

- `docs/작업명세서.md` — **인터페이스·수치의 정본**
- `docs/설계결정서.md` — 왜 이렇게 정했는지의 근거
- `docs/GitHub_협업_가이드.md` — 브랜치·PR·충돌 규칙

## 처음 받는 사람 (3분)

```bash
git clone <이 저장소 URL> ~/palletris_ws
cd ~/palletris_ws
bash scripts/setup_env.sh          # ~/.bashrc 설정 + fastdds 파일 배치
source ~/.bashrc
colcon build --symlink-install
source install/setup.bash
```

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
