# CLAUDE.md — Project Palletris

> 워크스페이스 루트(`~/palletris_ws/`)에 둔다.
> Claude Code 가 매 세션 시작 시 자동으로 읽는 프로젝트 컨텍스트다.

---

## 0. 이 프로젝트가 뭔가

창고 자동 팔레타이징 시뮬레이션. 컨베이어로 오는 박스를 로봇 팔이 집어
3D bin-packing 알고리즘이 계산한 자리에 쌓는다. 로보틱스 캡스톤, 6일 일정.

**우선순위: 완벽한 코드보다 "돌아가는 데모".**

---

## 1. 환경 — 고정값. 지어내지 말 것

| 항목 | 값 |
|---|---|
| OS / 미들웨어 | Ubuntu 24.04 · ROS 2 **Jazzy** (Python 3.12) |
| 시뮬레이터 | Isaac Sim **5.1** (내장 Python **3.11** — 버전이 다르다) |
| 로봇 | 두산 **H2017** (도달거리 1700mm) + 페데스탈 **900mm** |
| `ROS_DOMAIN_ID` | **115** (팀 전원 동일) |
| 워크스페이스 | `~/palletris_ws` |
| DB | SQLite `palletris.db` |

### 좌표 · 치수 (Z-up, 컨베이어 진행 = +X)

| 항목 | 값 |
|---|---|
| 벨트 상면 / 스토퍼 | z = 0.8 m / x = 10.0 m |
| 로봇 베이스 | (10.9, **−0.1**, **0.9**) m |
| 팔레트 원점(상면 X최소·Y최소 모서리) | **(10.3, +0.3, 0.15)** m |
| 팔레트 | **1200 × 800** mm (Px=월드+X **1200** · Py=월드+Y **800**), 최대 적재 **1300** mm |
| heightmap 격자 | 10 mm → **120 × 80** 셀 (`hm[px][py]`) |
| 박스 3종 (mm / kg) | 340×250×210 / **3** · 410×310×280 / **5** · 480×380×340 / **9** |

> **위 값의 출처는 `src/packing_bringup/config/system.yaml` 이다.** 어긋나면 config 가 맞다.
>
> **팔레트가 왜 1200 × 800 인가 (2026-08-22 90° 회전):** 존 방향(Y)이 800 이어야
> 3장을 붙여 2400 mm 가 되어 트럭 바닥 2800 에 들어간다. 1200 × 3 = 3600 은 초과라
> 상차가 성립하지 않았다 (작업명세서 §3.2). 팔레트만 돌리면 최원점이 도달범위를
> 넘으므로, 로봇이 **긴 변(1200) 바깥** 에 선다.
>
> **이격 300 → 400 (2026-08-24):** joint_1 이 픽에서 배치로 90° 넘게 도는 동안 팔뚝이
> 수평 원호를 그리며 근거리 기둥을 쓸어 밀었다. **궤적 전체 검사로만 잡힌다** — 정지
> 자세 검사는 같은 계획을 통과로 본다(280 mm 차이). ⚠ 더 빼면 나빠진다(450 에서 −203).
> `scripts/verify_transit_sweep_isaac.py` 로 재현. G1 재검증 시 B존 여유 +90 → **+13 mm**.
>
> **`max_stack 1300` 근거:** 트럭 내高 1600 − 팔레트 150 − 천장여유 150 (§3.4).
> ⚠ 실측은 **7단 1050mm 까지만** 확인됐다 (`scripts/demo_stack15.py`, knock 0.2mm).
> 1300 구간은 미검증 — 김병직님 확인 필요.

### 단위 규칙

- 설정 파일 · DB = **mm 정수** / Isaac · ROS 코드 = **m float**
- 변환은 **반드시 `packing_common.units` 함수만** 사용. 직접 `/1000` 금지.
- `PlacePose`: `x_mm`·`y_mm` = 박스 **중심**, `z_mm` = 박스 **밑면**
  → z 를 중심으로 쓰면 팔레트에 절반 파묻힌다.

---

## 2. 왜 Isaac 안과 밖이 나뉘어 있나 — 핵심 제약

**Isaac Sim 은 Python 3.11, ROS 2 Jazzy 는 Python 3.12.**
따라서 Isaac 내부 Python 에서 `packing_msgs` 를 import 할 수 없다.

| | 무엇이 있나 |
|---|---|
| **Isaac 안** (Action Graph) | 카메라 발행, 씬 제어, 로봇 제어. C/rcl 백엔드라 버전 무관 |
| **Isaac 밖** (rclpy 노드) | detector, orchestrator, algo, db_service_node. 실제 로직 전부 |

**결론: 로직은 Isaac 밖에 짠다.** Isaac 안에 rclpy 커스텀 노드를 만들지 마라.

---

## 3. 노드 구조

```
[Isaac 안 — Action Graph]           [Isaac 밖 — rclpy]

카메라 그래프  ──/line/rgb1,depth1──→ detector_line1 ──(srv)──→ db_service_node
              ──/zone_b/rgb───────→ detector_zone_b
                                          │(topic)
                                          ↓
sim_control_graph   ←──(srv)────────  orchestrator ──(srv)──→ algo
robot_control_graph ←──(topic)──────      │        ──(srv)──→ db_service_node
                    ──/robot/event──→     ┘
```

### 각 노드의 책임

| 노드 | 트랙 | 하는 일 | 하지 않는 일 |
|---|---|---|---|
| `detector_line1` | D | 카메라1(RGB-D)로 크기·ID 판정 → DB 등록 | 로봇 명령 |
| `detector_zone_b` | D | 천장 RGB로 도착 감지 + **pick pose(x/y/yaw)** 발행 | 로봇 명령 |
| `orchestrator` | B | **로봇 명령의 유일한 발신자.** 7상태 상태머신 | 자리 계산 |
| `algo` | C | heightmap 소유. 놓을 자리 계산 | **로봇 명령 금지** |
| `db_service_node` | D | **`palletris.db` 를 여는 유일한 프로세스** | |

★ **`algo` 나 `detector` 가 `/robot/cmd` 를 발행하면 설계 위반이다.**
   벨트 정지·재시도·종료 판단이 여러 노드로 흩어지면 디버깅이 불가능해진다.

---

## 4. 확정된 설계 결정 (2026-08 재검토 결과)

### D1. 천장 카메라 토픽은 구역별 네임스페이스
`/zone_a/rgb`, `/zone_b/rgb`, `/zone_c/rgb`.
같은 토픽명을 3대가 공유하면 어느 구역 영상인지 구분 불가.

### D2. `orchestrator` 가 로봇 명령의 유일한 발신자
`algo → robot` 직접 호출 금지. `algo` 는 순수 계산기다.

### D3. heightmap 은 `algo` 가 소유
천장 depth 로 heightmap 을 재구성하지 않는다.
`/algo/update_heightmap` 으로 **실측 pose** 를 받아 갱신한다.

### D4. 천장 카메라는 RGB 만 (depth 없음)
팔레트 위 z 는 `algo` 의 heightmap 이 이미 안다.
RGB-D 는 메인 라인 카메라1 한 대뿐.

| 값 | 출처 |
|---|---|
| x, y, yaw | 천장 RGB (실측) |
| z (팔레트 위) | `algo` heightmap (계산) |
| z (픽업 지점) | 벨트 상면 800mm + 박스 높이(DB) |

### D5. `detector_zone_*` 이 pick pose 까지 발행
```json
{ "box_id": "BOX00007", "x_mm": 10000, "y_mm": -12, "yaw_deg": 3.1 }
```
y 와 yaw 를 "0 이라고 가정" 하면 그리퍼가 박스 모서리를 문다.

### D6. DB 조회는 **서비스**. 토픽 캐싱 금지
`orchestrator` 는 집기 직전에 `/db/get_box_info` 로 매번 조회한다.
메모리 캐시를 쓰면 노드 재시작 시 날아가고, 2차 재순환에서 순서가 깨진다.

> **철칙 4번("콜백 안 블로킹 금지") 정정:**
> 싱글스레드 executor 에서만 데드락이다.
> `ReentrantCallbackGroup` + `MultiThreadedExecutor` 면 합법적으로 해결된다.
> 단 **이 프로젝트에서는 `call_async` + `done_callback` 만 쓴다.**

---

## 5. ★ `NO_FEASIBLE_POSE` ≠ `PALLET_FULL` — 가장 자주 틀리는 곳

| 신호 | 의미 | 처리 |
|---|---|---|
| `NO_FEASIBLE_POSE` | **이 박스만** 자리가 없다. 다음 박스는 들어갈 수 있다 | 그 박스만 despawn 후 **계속** |
| `PALLET_FULL` | 가장 작은 박스로도 후보가 없다 | **시나리오 종료** |

실측 (투입 30개 기준): `NO_FEASIBLE_POSE` 평균 4.3회, `PALLET_FULL` 9.2회.

> ⚠ **이 수치는 확인이 필요하다.** 트랙 C 가 시드 30개로 재측정한 값은
> `NO_FEASIBLE_POSE` **8.5회**, 첫 실패에 종료 시 충전율 **49.6%** → 버리고 계속 **60.2%**,
> 팔레트 포화까지 가면 **65.6% ± 4.3** (적재 27.3개) 다. 조건이 다른 것으로 보인다.
> 특히 `PALLET_FULL` 은 시나리오를 끝내는 신호라 **평균 9.2회가 나올 수 없다.**

**첫 `NO_FEASIBLE_POSE` 에서 종료하면 충전율 56% 에서 데모가 멈춘다.**

1차에는 Return line 이 없으므로 `/sim/despawn_box` 로 박스를 지운다.
스토퍼에 두면 뒤 박스가 밀려 라인이 막힌다.

```
NO_FEASIBLE_POSE 수신
  → /algo/cancel_plan(zone, box_id)
  → /db/set_status(box_id, 'REJECTED')
  → /sim/despawn_box(box_id)
  → 벨트 재가동 → 다음 박스
```

---

## 6. 시나리오 범위

| | 1차 (stage 1) | 2차 (stage 2) |
|---|---|---|
| 구역 | **B 만** | A · B · C |
| 박스 ID | **색상** (HSV 판정) | QR |
| 목적지 `dest` | 없음 (필드는 미리 만들어 둠, 값은 `""`) | QR 에 포함 |
| 중재자 | 불필요 | 필요 |
| Return line | **없음** → despawn | 있음 → 반송 |
| 카메라 | 2대 | 5대 |
| 종료 | **팔레트 1장 만재 → 끝** | |

**1차는 팔레트 교체·트럭 상차를 하지 않는다.**
따라서 `/sim/swap_pallet`, `/algo/reset` 은 1차에서 호출하지 않는다.

---

## 7. orchestrator 상태 7개

| 상태 | 하는 일 | 다음 |
|---|---|---|
| `IDLE` | 벨트 가동, 도착 대기 | 도착 → `FETCHING` |
| `FETCHING` | 벨트 정지 + `/db/get_box_info` | 응답 → `PLANNING` |
| `PLANNING` | `/algo/plan_placement` | ok → `PICKING` |
| `PICKING` | `/robot/cmd` 집기 | 성공 → `PLACING` |
| `PLACING` | `/robot/cmd` 놓기 | 성공 → `UPDATING` |
| `UPDATING` | `/algo/update_heightmap` (**실측 pose**) | → `IDLE` |
| `DONE` | 최종 지표 출력 후 정지 | (끝) |

**이탈 3경로:**

| 조건 | 처리 |
|---|---|
| `NO_FEASIBLE_POSE` | `cancel_plan` → `despawn_box` → `IDLE` |
| `PALLET_FULL` | → `DONE` |
| 픽/플레이스 실패 | `cancel_plan` → `set_status(PICK_FAILED)` → `IDLE` |

---

## 8. 코딩 규칙

- **모든 서비스 호출은 `call_async` + `future.add_done_callback`.**
  `spin_until_future_complete` 금지 — 콜백 안에서 부르면
  executor 가 자기 자신을 기다려 **에러 없이 조용히 멈춘다.**
- `MultiThreadedExecutor` + `ReentrantCallbackGroup` 필수.
- 서비스 타임아웃 2초. 초과 시 `cancel_plan` 후 복귀. **노드가 죽으면 안 된다.**
- 카메라 토픽 QoS 는 `qos_profile_sensor_data` (BEST_EFFORT).
  기본 RELIABLE 로 구독하면 이미지가 하나도 안 온다.
- **모든 이탈 경로에서 `/algo/cancel_plan` 호출.**
  빠뜨리면 그 자리가 영원히 예약된 채 남는다.
- **주석은 한국어.** 특히 "왜 이렇게 했는지".
- `packing_common/frames.py` 를 **import** 해서 쓴다. 복사 금지.

---

## 9. 자주 나오는 증상 → 원인

| 증상 | 원인 후보 |
|---|---|
| 토픽이 안 보이는데 에러도 없다 | `ROS_DOMAIN_ID` 불일치 |
| 이미지가 하나도 안 온다 | QoS 가 RELIABLE (BEST_EFFORT 여야 함) |
| 서비스 응답이 영원히 안 온다 | 콜백 안에서 동기 호출 (`spin_until_future_complete`) |
| 박스가 벨트를 뚫고 떨어진다 | 스폰 z 에 `h/2` 안 더함 |
| 쌓은 박스가 저절로 무너진다 | solver iteration 을 씬에 설정함 |
| `database is locked` | `db_service_node` 외 프로세스가 `.db` 를 열었다 |
| 2층부터 `NO_FEASIBLE_POSE` 만 | `support_tol_mm` < 양자화 간격 (10 이어야 함) |
| 박스가 팔레트 밖에 놓임 | `frames.py` 를 복사해 씀 / 팔레트 원점 오류 |
| 충전율이 이상하게 낮다 | 분모를 `max_stack`(1300) 고정으로 씀 (**실제 최고높이**여야 함) |

---

## 10. 하지 말 것

- ❌ 공용 파일(`packing_msgs/`, `config/system.yaml`, 씬 USD)을 **승인 없이** 수정
- ❌ `algo` 나 `detector` 에서 `/robot/cmd` 발행
- ❌ `spin_until_future_complete` 사용
- ❌ `db_service_node` 외 프로세스에서 `sqlite3.connect("palletris.db")`
- ❌ 아직 없는 노드의 mock 을 임의로 만들어 붙이기
- ❌ 문서에 없는 숫자·API 추측해서 쓰기 (모르면 "확인 필요" 라고 남길 것)
- ❌ 아직 필요 없는 최적화·리팩터링

---

## 11. 팀 · 트랙

| 트랙 | 담당 | 범위 |
|---|---|---|
| A | 김학민 | Isaac 씬, 카메라·컨베이어, USD |
| B | 김병직 | 로봇 팔 제어, orchestrator |
| C | 정정인 | 패킹 알고리즘 (`algo_service_node`) |
| D | 민범진 | 비전/인식, DB 서비스, 인터페이스 설계 |

**공용 파일 수정 = 팀 4명 전원 재빌드.** 바꿨으면 반드시 보고할 것.

---

## 12. 미결 (확인 필요 — 추측 금지)

| 항목 | 담당 | 왜 |
|---|---|---|
| `stage1.total_count` 15 → **30** | 팀 | 15 로는 팔레트가 안 참 (56.7% vs 65.5%) |
| ~~`gripper_clearance_mm` 30 → 10~~ **기각** | 김병직 | ★ **방향이 반대였다.** "흡착 툴 단면 실측" 의 답은 **122.5 mm**(그리퍼 몸통 Ø245 반경)다. 다만 값을 올리는 대신 `_corridor_ok` 의 **높이 기준**을 고쳤다 (2026-08-24) — 링 반경은 122, 비교 기준은 밑면이 아니라 **윗면+그리퍼길이 150**. `clr 60` 안은 협곡을 안 줄이고 적재율만 깎아 폐기 |
| `/sim/despawn_box` Day 4 → **Day 2** | 김학민 | §5 때문에 통합 전 필요 |
| 천장 카메라 z / FOV / 해상도 | 김학민 | D5 픽셀→mm 변환 |
| 색 팔레트 공유 | 김학민 | 스폰 색상과 인식 색상 일치 필요 |
| ~~C1 수용기준 5층 → 4층~~ **재개** | 정정인/팀 | 5층 근거(`210×5=1050 ≤ 1300`)는 **팔이 도달한다는 전제**였다. 2026-08-24 `_corridor_ok` 수정 후 4층 — 실측이 그 전제를 깼다 |
| 트랙 C 수용기준이 **기하 간섭을 안 본다** | 정정인 | `r8·g250` 은 테스트 3개를 다 통과하면서 Isaac 에서 닿는다. 느슨할수록 점수가 오르는 구조. `verify_transit_sweep_isaac.py` 를 게이트로 |