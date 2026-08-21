# 스마트 물류창고 로봇팔 3D 패킹 시스템

디지털 트윈 환경(Isaac Sim + ROS 2 Jazzy)에서 컨베이어와 로봇팔이 협업해 박스를 구역별로
분류하고, heightmap 알고리즘으로 3D 최적화 패킹을 한 뒤 팔레트를 트럭에 상차하는 프로젝트다.
1차 구현 범위는 **B구역(중앙 라인) 단독 — 랜덤 박스 스폰부터 팔레트 1개 적재까지.**

## 정본 문서 — 아래는 자동으로 함께 로드된다

인터페이스·수치·설계 근거의 정본은 아래 두 문서다. 이 CLAUDE.md는 그 요약/안내판일 뿐이며,
내용이 어긋나면 **반드시 정본을 따른다.** 특히 `작업명세서.md`가 `설계결정서.md`보다 우선한다.

@docs/작업명세서.md
@docs/설계결정서.md
@docs/가이드/공통가이드_전원필독.md
@응답지침.md

> 위 세 파일이 저장소에 없으면 이 import는 무시된다(에러 없이 조용히 스킵되므로 주기적으로
> 확인할 것). 현재 구조: `docs/작업명세서.md`, `docs/설계결정서.md`, `docs/가이드/` 폴더
> (하위 `img/` 포함, 트랙별 가이드가 여기 있다). 경로는 269075d 커밋(문서·스크립트를 루트로,
> `src/`는 ROS 패키지만) 기준이며, 이후 또 옮겨지면 이 import 3줄을 함께 갱신할 것.

## 트랙 담당 — 내 코드가 어디 있는지

| 트랙 | 담당 | 디렉터리 | 브랜치 |
|---|---|---|---|
| A 씬/에셋 | 김학민 | `src/packing_sim`, `src/packing_common`, `scripts/` | `track/sim` |
| B 로봇제어 | 김병직 | `src/packing_robot` | `track/robot` |
| C 패킹알고리즘 | 정정인 | `src/packing_algo` | `track/algo` |
| D 인터페이스/DB | 민범진 | `src/packing_msgs`, `src/packing_db` | `track/db` |
| 공용 | 합의 필요 | `src/packing_bringup`, `docs/`, `.github/` | — |

각 트랙 디렉터리 아래에 별도 `CLAUDE.md`를 둘 수 있다 — 그 폴더에서 작업할 때 이 루트 파일과
함께 자동으로 로드된다 (현재 `src/packing_algo/CLAUDE.md`가 있음, 트랙 C).

## 철칙 4개 — 절대 어기지 말 것

1. **인터페이스 고정.** `src/packing_msgs`의 메시지/서비스/토픽 정의는 Day 1 오전에 4인이 합의한
   것으로 고정한다. 바꾸려면 `main`에 먼저 머지 후 전원이 pull한다.
2. **씬 USD는 김학민 전용.** `*.usd`/`*.usda`/`*.usdc`는 김학민만 수정하고, git이 아니라
   `scripts/sync_scene.sh`(rsync)로 배포한다. 커밋 금지(`.gitignore`에 등록됨, CI가 검사한다).
3. **숫자는 전부 config에서.** 모든 수치는 `src/packing_bringup/config/system.yaml`에서 읽는다.
   코드에 상수를 박지 않는다.
4. **콜백 안에서 블로킹 금지.** 어떤 콜백(타이머 포함) 안에서도 `spin_until_future_complete`를
   쓰지 않는다 — 에러 없이 조용히 데드락난다. `ReentrantCallbackGroup` + `MultiThreadedExecutor`,
   또는 `call_async` + 논블로킹 폴링/`done_callback`만 쓴다.

## 핵심 아키텍처 — 소켓 브릿지 (PC1 내부 구현)

Isaac Sim 5.1 내장 Python은 3.11, Ubuntu 24.04의 ROS 2 Jazzy는 3.12다. Isaac 프로세스 안에서
`import rclpy`를 하면 ABI 불일치로 깨진다. 그래서 PC1 안에서 Isaac 프로세스("몸" —
`sim_server.py`, `robot_motion.py`)와 ROS 프로세스("머리" — `sim_bridge_node`, `orchestrator`)를
분리하고, localhost:8765 개행구분 JSON 소켓(요청-응답만, push 없음)으로 통신한다. 다른 PC에서
보이는 것은 ROS 노드 2개뿐이다. Isaac 쪽 코드에 `import rclpy`가, ROS 쪽 코드에 `import omni`가
한 줄도 있으면 안 된다. 프로토콜 표와 규칙 6개는 `가이드/공통가이드_전원필독.md` §3 참고.

## 단위·좌표 규약 — 어기면 통합이 조용히 깨진다

- 계층별 단위: 사람/문서/**DB**/**config** = **mm 정수**, 코드/Isaac/ROS2 메시지 = **m(float)**,
  heightmap 내부 = **셀(10mm)**. 변환은 `packing_common/units.py` 함수만 쓴다 — 직접 나눗셈 금지
  (부동소수 오차로 셀 인덱스가 1칸 어긋날 수 있다).
- `PlacePose`: `x_mm, y_mm`은 박스 **중심**, `z_mm`은 박스 **밑면**. 변환은
  `packing_common/frames.py`의 `world_to_pallet_place` / `pallet_place_to_world` 짝 함수만 쓴다
  — 트랙 B·C가 반드시 같은 파일을 import해야 좌표가 맞는다.
- 좌표계는 Z-up, 컨베이어 진행 = +X. B구역 팔레트 원점(월드) = `(11200, -600, 150)`,
  로봇 베이스 = `(10900, 0, 500)`.
- `box_id` 규칙: `BOX00000` ~ `BOX00014`(stage 1, 15개).

## 개발 환경

Ubuntu 24.04 · ROS 2 Jazzy · Isaac Sim 5.1 · 두산 H2017(페데스탈 500mm) ·
`ROS_DOMAIN_ID=115`(전원 동일, `~/.ros/fastdds_whitelist.xml` 필요).
빌드: `colcon build --symlink-install`. 패키지 7개:
`packing_msgs packing_common packing_sim packing_robot packing_algo packing_db packing_bringup`.
`packing_msgs`와 `packing_common`이 먼저 빌드돼야 나머지가 시작 가능하다.

## 커밋 금지 목록

`*.usd*`, `*.db*`, `build/ install/ log/`, 5MB 넘는 파일 — CI의 `guard` job이 이걸 막는다.
PR 올리기 전에 `.github/pull_request_template.md`의 체크리스트를 확인할 것.

## 막혔을 때

증상별 원인 후보는 `가이드/공통가이드_전원필독.md` §7과 `작업명세서.md` §14(자주 터질 함정
표)를 먼저 본다. 같은 문제로 30분 이상 막히면 혼자 붙잡지 말고 팀에 공유한다.
