# CLAUDE.md — Project Palletris

> 워크스페이스 루트(`~/palletris_ws/`)에 둔다.
> Claude Code 가 매 세션 시작 시 자동으로 읽는 프로젝트 컨텍스트다.
>
> ★ **숫자의 정본은 `src/packing_bringup/config/system.yaml` 이다.** 이 문서와 다르면 yaml 을 따르고,
>   다른 부분을 발견하면 지어내지 말고 보고할 것.

---

## 0. 이 프로젝트가 뭔가

창고 자동 팔레타이징 시뮬레이션. 컨베이어로 오는 박스를 로봇 팔이 집어
3D bin-packing 알고리즘이 계산한 자리에 쌓고, 팔레트째 트럭에 넣는다.
로보틱스 캡스톤, 6일 일정.

**우선순위: 완벽한 코드보다 "돌아가는 데모".**

---

## 1. 환경 — 고정값

| 항목 | 값 |
|---|---|
| OS / 미들웨어 | Ubuntu 24.04 · ROS 2 **Jazzy** (Python 3.12) |
| 시뮬레이터 | Isaac Sim **5.1** (내장 Python **3.11** — 버전이 다르다) |
| 로봇 | 두산 **H2017** (도달거리 1700mm) |
| `ROS_DOMAIN_ID` | **115** (팀 전원 동일) |
| 워크스페이스 | `~/palletris_ws` |
| 실행 형태 | **다중 PC** ROS 2 통신 |

---

## 2. ★★ 통신 구조 — 여기가 가장 크게 바뀌었다

### 지금 구조: ROS 2 하나뿐

```
[Isaac Sim 안]                        [Isaac 밖 — rclpy 노드]
Action Graph  ══ ROS 2 토픽/서비스 ══  관제탑 · 인식 · 알고리즘 · DB
```

Isaac 안팎을 잇는 통로는 **ROS 2 하나뿐이다.**
Action Graph 의 C/rcl 백엔드가 Python 버전 차이(3.11 vs 3.12)를 우회하므로
소켓이 필요 없다.

### ❌ 폐기된 방식 — 절대 다시 도입하지 말 것

| 폐기 | 대체 |
|---|---|
| **JSON 소켓 브리지 (localhost:8765)** | **Action Graph 가 직접 ROS 2 발행** |
| `std_msgs/String` + JSON 문자열 | **`packing_msgs` 전용 타입** |
| `ros_svc.py` 어댑터 | 노드가 직접 `create_client` |
| `json.dumps()` / `json.loads()` 로 메시지 조립 | 필드에 직접 대입 |
| 브리지 폴링 (`server.poll()`) | 구독 콜백 |

**Claude Code 가 "Isaac 안에서 rclpy 를 못 쓰니 소켓이 필요하다"고 판단하면
그건 틀린 것이다** — 필요한 건 **Action Graph 노드**이지 소켓이 아니다.

> ⚠️ `src/packing_sim/isaac/sim_server.py` 가 아직 저장소에 남아 있다.
> **김학민 소유 파일이므로 임의로 지우지 말 것.** 새 코드에서 import 하지도 말 것.

> ⚠️ 옛 코드나 옛 대화 요약에 `localhost:8765`, `SimServer`, `handlers()`,
> `std_msgs/String` 이 보이면 그건 **폐기된 버전**이다. 참고하지 말 것.

---

## 3. ★★ 인터페이스 전체 정의

**토픽에 JSON 문자열을 쓰지 않는다.** 전부 `packing_msgs` 전용 타입이다.

### 3-1. 공용 msg 2개

```
# msg/PlacePose.msg
# ★ x_mm / y_mm 는 박스 "중심", z_mm 은 박스 "밑면"
#   z 를 중심으로 쓰면 팔레트에 절반 파묻힌다.
int32   x_mm
int32   y_mm
int32   z_mm
float32 yaw_deg      # 0 또는 90
```

```
# msg/BoxInfo.msg
string  box_id
int32   size_w_mm
int32   size_d_mm
int32   size_h_mm
int32   weight_g
string  color
string  dest           # QR 목적지 "A"|"B"|"C"
string  qr_payload
int32   retry_count    # Return line 재순환 횟수
string  status         # SPAWNED|SCANNED|VERIFIED|DIVERTED|PICKED|PLACED|PICK_FAILED|REJECTED
```

### 3-2. 토픽 4종

| 토픽 | 타입 | 방향 |
|---|---|---|
| 카메라 영상 | `sensor_msgs/Image` | Action Graph → 인식 노드 |
| `/tf` · `/tf_static` | Isaac `ROS2 Publish Transform Tree` | Action Graph → 전원 |
| `/clock` | Isaac `ROS2 Publish Clock` | Action Graph → 전원 |
| `/zone_*/box_at_pickup` | `packing_msgs/msg/BoxAtPickup` | 인식 → 관제탑 |
| `/robot/cmd` | `packing_msgs/msg/RobotCmd` | 관제탑 → 로봇 |
| `/robot/event` | `packing_msgs/msg/RobotEvent` | 로봇 → 관제탑 |
| `/zone_*/status` | `packing_msgs/msg/ZoneStatus` | 관제탑 → 중재자·모니터링 |

```
# msg/BoxAtPickup.msg   — 인식 노드가 픽업 좌표까지 실어 보낸다
# ★ 좌표는 mm 정수가 아니라 geometry_msgs/Pose 다 (2026-08-24 확정).
#   여기만 단위가 m 인 이유: pick_cam 이 내놓는 것이 원래 Pose 라
#   중간에 mm 로 꺾었다가 로봇 앞에서 다시 m 로 펴면 왕복 오차가 생긴다.
bool                present   # false = 박스 없음. 무시하고 사이클을 시작하지 말 것
string              box_id
geometry_msgs/Pose  pose      # 월드 좌표, 단위 m. ★ 박스 "중심"
```

```
# msg/RobotCmd.msg   — 관제탑 → 로봇
# 왜 서비스가 아니라 토픽인가: 모션이 5~10초 걸린다. 서비스로 묶으면
# 그동안 응답을 붙들고 있어야 하고 타임아웃 관리가 복잡해진다.
uint8 PICK  = 0
uint8 PLACE = 1
uint8 HOME  = 2
uint8 STOP  = 3
uint8   action

string  box_id
uint32  seq          # ★ 명령 일련번호. event 가 이 값을 되돌려줘야
                     #   "지금 온 결과가 방금 그 명령의 것"임을 보장한다.

int32   x_mm         # 박스 중심
int32   y_mm
int32   z_mm         # ★ 박스 밑면
float32 yaw_deg

int32   size_w_mm
int32   size_d_mm
int32   size_h_mm

int32   approach_z_mm   # 수직 하강 가능 높이. 0 이면 로봇 재량
```

```
# msg/RobotEvent.msg   — 로봇 → 관제탑
uint32  seq          # RobotCmd.seq 를 그대로 복사
uint8   action
string  box_id

bool    ok
string  reason       # "GRASP_FAILED"|"IK_UNSOLVABLE"|"COLLISION"|
                     # "DROPPED"|"TIMEOUT"|"ABORTED" / 성공이면 ""

# ── 실측 결과 (PLACE 성공 시에만 유효) ──
bool    has_actual
int32   actual_x_mm
int32   actual_y_mm
int32   actual_z_mm      # 밑면 기준
float32 actual_yaw_deg

float32 duration_s
```

```
# msg/ZoneStatus.msg   — 관제탑 → 중재자·모니터링. 2 Hz
string  zone           # "a"|"b"|"c"
string  state          # IDLE|FETCHING|PLANNING|PICKING|PLACING|UPDATING|DONE
int32   queue_len
int32   placed_count
int32   rejected_count
float32 fill_rate      # 분모는 max_stack 이 아니라 실제 최고높이
int32   max_height_mm
bool    accepting      # ★ false = 더 보내지 마라. 중재자는 이 칸만 본다
builtin_interfaces/Time stamp
```

### 3-3. 서비스 15종 — 전부 `packing_msgs/srv`

| 계열 | 이름 | 서버 | 요청 → 응답 |
|---|---|---|---|
| 알고리즘 | `PlanPlacement` | algo | `zone, box_id, w, d, h` → `ok, PlacePose, score, message` |
| 알고리즘 | `UpdateHeightmap` | algo | `zone, box_id, w, d, h, actual_pose` → `ok, fill_rate, placed_count, max_height_mm` |
| 알고리즘 | `CancelPlan` | algo | `zone, box_id` → `ok` |
| 알고리즘 | `ResetHeightmap` | algo | `zone` → `ok` |
| 씬 | `SetVelocity` | 씬 | `seg, velocity_mps` → `ok, actual_mps` |
| 씬 | `DespawnBox` | 씬 | `box_id` → `ok` |
| 씬 | `SetSpawnEnabled` | 씬 | `enabled` → `ok` |
| 씬 | `SwapPallet` | 씬 | `zone` → `ok` |
| DB | `RegisterBox` | db | `box_id, w, d, h, color, dest, qr_payload` → `ok, message` |
| DB | `GetBoxInfo` | db | `box_id` → `found, BoxInfo` |
| DB | `VerifyBox` | db | `box_id, qr_payload` → `match, registered_id` |
| DB | `SetStatus` | db | `box_id, status, reason` → `ok` |
| DB | `MarkPlaced` | db | `box_id, zone, actual_pose` → `ok` |
| DB | `ZoneQueue` | db | `zone` → `count, placed_count` |
| 중재자 | `DiverterRequest` | 중재자 | `box_id` → `target, reason` |

**`PlanPlacement.message` 값 3가지:** `"OK"` · `"NO_FEASIBLE_POSE"` · `"PALLET_FULL"`
**`DiverterRequest.target` 값 4가지:** `"A"` · `"B"` · `"C"` · `"RETURN"`

### 3-4. 반드시 지킬 규약 5개

1. **모든 노드에 `use_sim_time:=true`.**
   없으면 tf 조회가 전부 실패하고 "lookup would require extrapolation" 이 뜬다.
2. **`RobotCmd.seq` 를 `RobotEvent.seq` 가 그대로 되돌려준다.**
   관제탑은 `ev.seq != self.seq` 면 무시한다. 없으면 타임아웃 후 늦게 온
   이전 응답을 새 박스의 결과로 오인한다 — 에러 없이 엉뚱하게 도는 버그다.
3. **`has_actual == true` 일 때만 `actual_*` 를 `UpdateHeightmap` 에 넣는다.**
   계획값을 되보내면 오차가 누적돼 알고리즘은 1300 안이라 믿는데 실제 더미가 넘는다.
4. **`DiverterRequest` 는 응답 제한 0.6초.** 초과 시 씬이 `NORMAL` 로 통과시킨다.
   일반 서비스 타임아웃 2.0초를 쓰면 답이 올 때 박스가 이미 분기점을 지나 있다.
5. **카메라 토픽 QoS 는 `qos_profile_sensor_data` (BEST_EFFORT).**
   기본 RELIABLE 로 구독하면 이미지가 하나도 안 온다.

---

## 4. 좌표 · 치수

**z = 0 은 도크 바닥 = 트럭 적재함 바닥이다. 야드 지면이 아니다.**
야드 지면은 z = −850. 이 규약 덕분에 "팔레트를 들지 않고 밀어서 트럭에 넣는다"가
높이차 0 으로 이미 만족된다.

### 컨베이어 (진행 = +X)
| 항목 | 값 |
|---|---|
| 벨트 상면 (롤러 면) | z = **800** mm |
| 스폰 지점 | (**500**, 0, 810) mm — 박스 밑면 중심 |
| 스토퍼 | x = **10000** mm |
| 픽업 구간 | x = 7500 ~ 10000 mm |
| 벨트 속도 | 0.5 m/s |

### ★★ bbox 를 그대로 쓰지 마라

| | bbox (프레임·난간 포함) | **써야 하는 값 (롤러 면)** |
|---|---|---|
| 폭 | 1970 mm | **1546 mm** |
| Z 상면 | 1213 mm | **800 mm** |
| X (main) | −33 ~ 7515 | **0 ~ 7500** |
| X (b_pickup) | 7489 ~ 10005 | **7500 ~ 10000** |

- **물리는 롤러에만 걸려 있다** (`bind_phys(_t + "/Rollers")`).
- bbox 1213 은 **난간 꼭대기**다. 벨트 높이로 쓰면 로봇이 413 mm 위에서 놓는다.
- 벨트 폭 1546 은 **의도된 값**이다 — 팔레트(1200×800) 반출 컨베이어 겸용.
- 이음매 7489~7515 로 26 mm 겹침. 여기서 박스가 튀면 `pick_cam` 이 잡아준다.

### 주요 프림
| 프림 | X 범위 | Z 상면 |
|---|---|---|
| Stopper | 10000 ~ 10100 | 900 |
| Pedestal | 10700 ~ 11100 | 900 |
| Pallet | 10300 ~ 11500 | 150 |
| PalletOutfeed | 10250 ~ 12500 | **0** |

### 구역 · 로봇 · 팔레트
| 구역 | 팔레트 원점 (mm) | 로봇 베이스 (mm) |
|---|---|---|
| A | (10300, **1800**, 150) | (10900, **1500**, **900**) |
| B | (10300, **300**, 150) | (10900, **0**, **900**) |
| C | (10300, **−1200**, 150) | (10900, **−1500**, **900**) |

- **페데스탈 900 mm** (구 문서의 500·700·1700 은 전부 폐기값)
- 팔레트 **1200(X) × 800(Y)** × 두께 150 → Y 로 3장 = 2400, 트럭 바닥 2800
- **최대 적재 1300 mm** (팔레트 **상면** 기준). 850 이 아니다.
- heightmap 격자 10 mm → **120 × 80** 셀

### 박스 3종
| 이름 | w × d × h (mm) | 질량 | 혼입비 |
|---|---|---|---|
| 3호 | 340 × 250 × 210 | **3 kg** | 0.5 |
| 4호 | 410 × 310 × 280 | **5 kg** | 0.3 |
| 5호 | 480 × 380 × 340 | **9 kg** | 0.2 |

### 랜덤 스폰 (평가 항목 "동적 대응")
```yaml
spawn_y_jitter_mm: 150      # y 를 −150 ~ +150 에서 균등 추출
spawn_yaw_jitter_deg: 15    # yaw 를 −15 ~ +15 에서 균등 추출
random_seed: 42             # 고정 — 절대 바꾸지 말 것
```
제약은 벨트 폭이 아니라 **로봇 도달거리 1700**이다.
시드 고정이라 **"매번 다른 위치"와 "Play/Stop 재현성"을 동시에** 만족한다.

### 단위 규칙
- 설정 파일 · DB = **mm 정수** / Isaac · ROS 코드 = **m float**
- 변환은 **반드시 `packing_common.units` 함수만**. 직접 `/1000` 금지.
- **코드에 숫자를 박지 않는다.** 전부 `system.yaml` 에서 읽는다.

---

## 5. 카메라 4종

| 카메라 | 위치 | 역할 |
|---|---|---|
| `cam1` | x = 600 | 투입부 검사 · 크기/ID 판정 |
| `cam2` | x = 1400 | 투입부 검증 |
| **`pick_cam`** | 스토퍼 위 (y 0, z **2300**) 하향 | **픽업 좌표 실측** |
| **`wrist_cam`** | 그리퍼 몸통 · offset (170, 0, 30) · tilt 18° | hand-eye |

- 해상도 1280×720 · HFOV 60° · 30 Hz
- ⚠️ **오프스크린 렌더는 GUI 조명 리그를 못 받는다.** 돔 라이트(`intensity: 1000`)가
  꺼져 있으면 눈으로는 멀쩡한데 카메라 이미지가 캄캄하다.
- ⚠️ 화면이 전부 0 이면 조명보다 **카메라가 형상 안에 파묻혔는지** 먼저 의심할 것.
- **처리 효율**: 천장 카메라는 도착 신호가 왔을 때만 추론한다.

---

## 6. flow — 박스 한 개 사이클

```
1. 박스 스폰            씬 (랜덤 y ±150 · 랜덤 yaw ±15°)
2. 투입부 인식           cam1 → RegisterBox → cam2 → VerifyBox
3. 목적지 판단 · 분기     중재자 DiverterRequest (x=2000, 0.6초 제한)
4. 픽업 위치 도착        pick_cam → BoxAtPickup (Pose, 월드 m)
5. 벨트 정지            관제탑 SetVelocity(0)
6. 놓을 자리 계산        PlanPlacement
     ├─ NO_FEASIBLE_POSE → CancelPlan → 되돌림 (종료 아님)
     └─ PALLET_FULL      → 트럭 상차
7. 집기 → 놓기           RobotCmd → RobotEvent (실측 pose 회신)
8. UpdateHeightmap → SetVelocity(0.5) → 1번으로 반복
```

---

## 7. ★ `NO_FEASIBLE_POSE` ≠ `PALLET_FULL`

| 신호 | 의미 | 처리 |
|---|---|---|
| `NO_FEASIBLE_POSE` | **이 박스만** 자리가 없다 | 그 박스만 되돌림 후 **계속** |
| `PALLET_FULL` | 가장 작은 박스로도 후보가 없다 | 트럭 상차 · 종료 |

**첫 `NO_FEASIBLE_POSE` 에서 종료하면 팔레트가 반만 찬 채 데모가 멈춘다.**
스토퍼에 박스를 두면 뒤 박스가 밀려 라인이 막히므로 반드시 치운다.

---

## 8. 노드 구조

| 노드 | 개수 | 트랙 | 하는 일 | 하지 않는 일 |
|---|---|---|---|---|
| **관제탑** | **1** | B | **로봇 명령의 유일한 발신자.** 상태머신 | 자리 계산 |
| 구역 인식 노드 | 3 | D | 도착 감지 + 픽업 좌표 발행 | 로봇 명령 |
| 라인 인식 노드 | 2 | D | cam1 판독 · cam2 검증 | 로봇 명령 |
| 중재자 | 1 | D | 목적지 배분 · 되돌림 판단 | |
| 적재 알고리즘 | 1 | C | heightmap 3장 소유 | **로봇 명령 금지** |
| db_service_node | 1 | D | `palletris.db` 유일 writer | |

★ **관제탑은 하나다.** 구역이 3곳이어도 프로세스를 늘리지 않는다.
★ 알고리즘이나 인식 노드가 `/robot/cmd` 를 발행하면 설계 위반이다.

### 관제탑 상태 7개
`IDLE` → `FETCHING` → `PLANNING` → `PICKING` → `PLACING` → `UPDATING` → `IDLE` / `DONE`

**이탈 3경로:** `NO_FEASIBLE_POSE` · `PALLET_FULL` · 실패/타임아웃(**20초**)
→ 모두 `CancelPlan` 을 호출한 뒤 복귀한다.

---

## 9. 코딩 규칙

- **모든 서비스 호출은 `call_async` + `future.add_done_callback`.**
  `spin_until_future_complete` 금지 — 콜백 안에서 부르면
  executor 가 자기 자신을 기다려 **에러 없이 조용히 멈춘다.**
- `MultiThreadedExecutor` + `ReentrantCallbackGroup` 필수.
- 로봇 모션 타임아웃 **20초**. 초과 시 `CancelPlan` 후 복귀. **노드가 죽으면 안 된다.**
- 구독 콜백은 **입력을 검증한다.** rclpy 는 콜백 예외를 삼키므로,
  잘못된 메시지 하나에 박스가 영원히 안 오는데 에러는 안 뜬다.
- ⚠️ **Isaac 메인(물리) 스레드에서 blocking wait 을 하지 마라.**
  판정을 `rm.step()` 이 내리는데 그 step 도 메인 스레드라 데드락이 된다.
- **주석은 한국어.** 특히 "왜 이렇게 했는지".
- `packing_common/frames.py` 를 **import** 해서 쓴다. 복사 금지.

---

## 10. 자주 나오는 증상 → 원인

| 증상 | 원인 후보 |
|---|---|
| 토픽이 안 보이는데 에러도 없다 | `ROS_DOMAIN_ID` 불일치 / 콜백 예외가 삼켜짐 |
| 이미지가 하나도 안 온다 | QoS 가 RELIABLE |
| 카메라 이미지가 캄캄하다 | 돔 라이트 꺼짐 / **카메라가 형상 안에 파묻힘** |
| tf lookup 이 계속 실패한다 | `use_sim_time` 미설정 / `/clock` 미발행 |
| **노드는 살아 있는데 타이머가 하나도 안 돈다** | **`use_sim_time:=true` 인데 `/clock` 발행자가 없다.** ROS 시계가 0 에 멈춰 타이머가 영원히 안 뜬다 — 상태 발행·지표·**로봇 20초 타임아웃까지 전부 죽는다.** Isaac 을 먼저 띄우거나 `use_sim_time:=false` 로 돌릴 것 |
| **로봇이 박스를 400 mm 위에서 놓는다** | **벨트 높이에 bbox 1213 을 씀 (800 이어야 함)** |
| 서비스 응답이 영원히 안 온다 | 콜백 안에서 동기 호출 |
| Isaac 이 통째로 멈춘다 | 메인 스레드에서 `Event.wait()` |
| 로봇 응답이 엉뚱한 박스 것으로 처리된다 | `seq` 검사 누락 |
| 박스가 벨트를 뚫고 떨어진다 | 스폰 z 에 `h/2` 안 더함 |
| 흡착했는데 박스가 40 mm 떠서 붙는다 | 패드 D6 조인트 6축을 다 잠금 |
| 팔이 최대 토크로 잡아채며 박스를 친다 | 웨이포인트 없이 계단 입력 |
| 다층에서 팔이 기둥을 친다 | 페데스탈이 900 이 아님 |
| 2층부터 `NO_FEASIBLE_POSE` 만 | `support_tol_mm` < `cell_mm` (둘 다 10) |
| 충전율이 이상하게 낮다 | 분모를 `max_stack_mm` 고정으로 씀 |
| 실제 더미가 계산보다 높다 | `UpdateHeightmap` 에 계획 pose 를 넣음 |
| `database is locked` | `db_service_node` 외 프로세스가 `.db` 를 열었다 |

---

## 11. 하지 말 것

- ❌ **JSON 소켓 브리지 재도입** (§2)
- ❌ **토픽에 `std_msgs/String` + JSON 사용** — 전용 타입이 있다 (§3)
- ❌ 공용 파일(`packing_msgs/`, `src/packing_bringup/config/system.yaml`, 씬 USD)을 **승인 없이** 수정
- ❌ **코드에 숫자 상수 박기** — 전부 `system.yaml` 에서 읽는다
- ❌ **컨베이어 bbox 값(1970 · 1213)을 사양으로 쓰기**
- ❌ **관제탑을 구역마다 띄우기** — 하나다
- ❌ 알고리즘·인식 노드에서 `/robot/cmd` 발행
- ❌ `spin_until_future_complete` 사용
- ❌ `random_seed: 42` 변경
- ❌ 흡착 패드 D6 조인트 값 임의 변경 (`h2017_cell.usd` 실측값이다)
- ❌ 아직 없는 노드의 mock 을 임의로 만들어 붙이기
- ❌ 문서에 없는 숫자·API 추측해서 쓰기 (모르면 "확인 필요" 라고 남길 것)

---

## 12. 저장소 구조

| 경로 | 무엇 |
|---|---|
| `src/packing_common/` | 공용 — `config.py` · `frames.py` · `units.py`. **복사 금지, import** |
| `src/packing_msgs/` | 공용 인터페이스. 수정 = 팀 전원 재빌드 |
| `src/packing_db/` | 트랙 D — `db_service_node.py` (palletris.db 유일 writer + 중재자) · `schema.sql` · `gen_master_db.py` |
| `src/packing_algo/` | 트랙 C — `heightmap_core.py` · `algo_service_node.py` |
| `src/packing_robot/` | 트랙 B — `rmpflow_controller.py` · `fk.py` · `cycle.py` |
| `src/palletris_orchestrator/` | 트랙 B — `orchestrator_node.py` (관제탑. 상태머신 7개) |
| `src/packing_vision/` | 트랙 D — `vision_node.py` (cam1 등록 · cam2 검증 · pick_cam 좌표). 코어 `detect.py`·`geometry.py` 는 ROS 의존성 0 |
| `src/packing_sim/isaac/` | 트랙 A — `spawner.py` · `camera_pick.py` · `conveyor_ctl.py` · `robot_motion.py` · `sim_main.py` |
| `scripts/` | 검증 — `verify_reach.py` · `check_place_plan.py` 등 |

★ **`spawner.py` 가 박스 스폰 담당이다.** 랜덤 지터는 여기서 적용한다.
★ **`.claude/worktrees/` 는 작업 브랜치 사본이다. 여기를 고치지 말 것.**

---

## 13. 평가 항목과의 대응

| 평가 항목 | 우리 대응 |
|---|---|
| ROS2 연동 (clock · 이미지 · tf) | Action Graph 3종 발행 |
| 동적 대응 (랜덤 스폰) | y ±150 · yaw ±15° · `pick_cam` 실측 |
| 다중 로봇 조율 | 로봇 3대 + 관제탑 + 중재자 |
| 예외 처리 (크래시 없음) | 입력 검증 · 타임아웃 20초 · `CancelPlan` |
| 처리 효율 (추론 주기 제한) | 천장 카메라는 도착 신호 시에만 추론 |
| 검증 및 분석 (정량) | 종료 지표: 적재수 · 충전율 · 최고높이 · 소요시간 |
| 재현성 (Play/Stop) | `random_seed: 42` · 리셋 처리 |
| AI 비전 활용 | `pick_cam` 좌표 → `RobotCmd` · `wrist_cam` hand-eye |
| 챌린지 | heightmap vs naive · `w_far` · `w_wall` 개선 실측 |

---

## 14. 팀 · 트랙

| 트랙 | 담당 | 범위 |
|---|---|---|
| A | 김학민 | Isaac 씬, 카메라·컨베이어·tf, USD |
| B | 김병직 | 로봇 팔 제어, 관제탑 |
| C | 정정인 | 패킹 알고리즘 |
| D | 민범진 | 비전/인식, DB, 인터페이스 설계 |

**공용 파일 수정 = 팀 4명 전원 재빌드.** 바꿨으면 반드시 보고할 것.

---

## 15. 미결 (확인 필요 — 추측 금지)

| 항목 | 담당 | 왜 |
|---|---|---|
| `spawn_y_jitter_mm` · `spawn_yaw_jitter_deg` **yaml + `spawner.py` 적용** | 김학민 | 키만 넣으면 아무 일도 안 일어난다 |
| `sim_server.py` 존치 여부 | 김학민 | 소켓 폐기 후 남아 있다 |
| `recognition.enabled: false` → `true` | 팀 | 최종 구현은 인식 포함 |
| `stage: 1` → `2` 전환 시점 | 팀 | 최종은 3구역 |
| tf 프레임 이름 규약 | 김학민 | 씬 prim 이름과 일치해야 함 |
| `gripper_clearance_mm` 30 → **10** | 김병직 | 충전율 **+7.2%p** |
| 조각 이음매(7489~7515)에서 박스가 튀는지 | 김학민 | 3호(250)가 걸리면 라인이 막힌다 |
