# CLAUDE.md — Project Palletris

> 워크스페이스 루트(`~/palletris_ws/`)에 둔다.
> Claude Code 가 매 세션 시작 시 자동으로 읽는 프로젝트 컨텍스트다.
>
> ★ **숫자의 정본은 `config/system.yaml` 이다.** 이 문서와 다르면 yaml 을 따르고,
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
string  box_id
bool    present      # false = 박스 없음. 무시하고 사이클을 시작하지 말 것
int32   x_mm         # 박스 중심 (pick_cam 실측)
int32   y_mm
float32 yaw_deg
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
| A | (10300, **1800**, 150) | (10900, **1400**, **900**) |
| B | (10300, **300**, 150) | (10900, **−100**, **900**) |
| C | (10300, **−1200**, 150) | (10900, **−1600**, **900**) |

> **이격 300 → 400 (2026-08-24, 김병직 실측).** joint_1 이 픽(−X)에서 배치(+Y)로 90°
> 넘게 도는 동안 팔뚝이 수평 원호를 그리며 근거리 기둥을 쓸어 밀었다.
> **궤적 전체 검사로만 잡힌다** — 정지 자세 검사는 같은 계획을 통과로 본다(280mm 차이).
> ⚠ **더 빼면 나빠진다** (450 에서 −203). `scripts/verify_transit_sweep_isaac.py` 로 재현.

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

> **`max_stack 1300` 검증 완료 (2026-08-25, 김병직 실측).** 이전 기록은 7단 1050mm 였다.
> `demo_stack15.py --plan-mode algo --standoff 400 --algo-clearance 10
> --hm corridor_far_ratio=0.0 --hm corridor_near_gap_mm=0` 로 **9단 1300mm** 를 세웠다 —
> 20/20 적재, 최대 knock **1.5mm**, 팔레트 밖 0.0mm (`results/dir0.txt`).
> 같은 실행에서 joint_5 는 **−107.1 ~ 25.2도**(한계 ±360)라 되감기도 필요 없다.

> **통로 검사에는 방향이 있다 (2026-08-25, 김병직 실측):** 팔은 **로봇 쪽(Py 작은 쪽)에서**
> 들어온다. 먼 쪽 기둥은 툴이 이미 지나온 뒤라 안 걸리고, 로봇 쪽 기둥은 팔뚝이 그 위를
> 넘어와야 한다. `_corridor_ok` 는 원래 사방을 똑같이 봐서 **한쪽은 괜히 막고 한쪽은
> 통과시켰다.** `corridor_far_ratio`(먼 쪽 여유폭 배율) · `corridor_near_gap_mm`(로봇 쪽
> 높이 여유)로 가른다. **둘 다 기본값이 기존 동작**이라 켜야만 바뀐다.
> Isaac 20박스 A/B: 끄면 적재 17·버림 1·실패 2 · knock 63.2mm, 켜면 **20/0/0 · 1.5mm**.

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
4. 픽업 위치 도착        pick_cam → BoxAtPickup (x·y·yaw)
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
| 쌓은 박스가 저절로 무너진다 | solver iteration 을 씬에 설정함 |
| 놓고 나서 박스가 혼자 조금씩 미끄러진다 | 박스 프림에 solver iteration·마찰이 안 걸렸다 (`spawner.py` 는 걸고, `verify_suction.make_box` 는 안 건다) |
| 박스가 슬롯에 안 들어가고 `IK_FAIL` | 계획 간격이 0 이라 슬롯이 박스와 정확히 같은 크기다 (`--algo-clearance`) |
| 깊은 구덩이 자리에서만 `IK_FAIL` | `_corridor_ok` 가 로봇 쪽 기둥을 안 본다 (`corridor_near_gap_mm`) |
| `database is locked` | `db_service_node` 외 프로세스가 `.db` 를 열었다 |

---

## 10-1. ★ 계획 파라미터는 **Isaac 으로 판정한다**

데스크 지표(수용 테스트 · 시드 스윕)와 Isaac 이 **부호까지 어긋난다.** 2026-08-25 실측:

| | 데스크 예측 | Isaac 실측 |
|---|---|---|
| 방향성 통로의 값 | 적재율 **−0.8%p** (손해) | **+3개** (이득) — 부호 반대 |
| `near_gap` 50 vs 0 | 50 이 낫다 (56.1% vs 34.9%) | **0 이 낫다** (20 vs 17) — 순위 반대 |
| `near_gap` 0 성능 | 20개 · 1300mm | 20개 · 1300mm — 일치 |

**원인: 데스크에서는 실패가 공짜다.** 배치 실패 1건을 박스 1개 손해로 센다.
실측에서는 **실패 2건이 적재 3개를 깎았다** — 팔이 헤집어 스택을 63mm 밀고,
무너진 지형이 뒤 박스 자리까지 죽였다. 기하 프록시는 그 연쇄를 원리적으로 못 본다.

**규칙**

- 데스크 스윕은 **후보를 좁히는 용도**다. 채택 판정으로 쓰지 않는다.
- 계획 파라미터(가중치 · 통로 · 간격)를 바꾸면 **`demo_stack15.py` 로 끝까지 한 번 돌린다.**
  볼 것은 `적재 / 버림 / 실패` · `최대 knock` · `팔레트 밖` 네 줄이다.
- 데스크로 재려면 **조건을 실측과 같게** 맞춘다 (같은 박스 순서 · 같은 간격).
  조건이 다르면 순위까지 뒤집힌다.
- 수용 테스트(`test_core.py`)는 **회귀 방지**이지 성능 게이트가 아니다.

---

## 10-2. 사이클 시간 — 어디서 쓰는지 먼저 재라 (2026-08-25)

**18.3 s → 10.6 s.** 같은 조건(20박스 · 도착순 · 간격10 · 이격400), 둘 다 20/20 적재.

계측(`--diag-time`)이 없었으면 엉뚱한 데를 깎았을 것이다. 한 사이클 19.5 s 중
**실제 관절 이동은 4.8 s** 뿐이었고 나머지가 어디로 가는지 아무도 몰랐다.

| 고친 것 | 사이클 | 근거 |
|---|---|---|
| (시작) | 18.3 s | |
| 도착 판정: 절대오차 → **수렴 정체** | 12.6 s | 정착 잔차 실측 1.03~2.05 mm — **임계값 1 mm 는 최소값보다 작았다** |
| `REACH_TOL_M` 1 → **2 mm** | ↑ | 같은 실측 |
| 구간별 허용: 이송 **8 mm** | 11.7 s | 웨이포인트 8개 중 6개가 이송인데 하강과 같은 정밀도를 요구했다 |
| `joint_speed_scale` 0.8 → **1.0** | **10.6 s** | 정격 상한. 배출이 속도와 무관하므로 빠를수록 이득 |

**부작용은 오히려 줄었다** — 최대 knock 3.3 → **2.7 mm**, 실패 0, 9단 1300 mm 유지.

### 재면서 틀린 것 두 개 (같은 실수 반복 금지)

- **"폴링이 4초를 먹는다"** → 실제로는 구간당 100 ms, 사이클 0.6 s 였다
- **"속도를 낮추면 뒤처짐이 줄어 총 시간이 준다"(U자 곡선)** → 배출은 속도와
  **무관하게 ~590 ms 로 일정**하다. 제어기의 고정 응답시간이지 추종 지연이 아니다.
  느리게 하면 이동만 길어져 손해다 (0.5 → 15.9 s)

### ⚠ 발표에서 구분할 것

헤드리스는 시뮬이 실시간의 **7배**로 돈다. `cycle.MOTION_POLL_S` 는 **벽시계** 20 ms 라
폴 한 번이 **138 ms 시뮬**이 된다. 사이클에 든 약 1~2 s 는 **실기에 없는 계측 인공물**이다.
`--poll-ms` 로 걷어낼 수 있다.

### 남은 덩어리

| | 사이클 10.6 s 중 | |
|---|---|---|
| 실제 관절 이동 | ~3.4 s | 정격이 상한 |
| 지연 배출 | ~4.8 s | 절반이 폴링 인공물 |
| `release` | 1.5 s | → **1.1 s 가능** (아래) |
| 급송·서비스 | ~1.0 s | |

**`release` 실측** (`--diag-release`, 놓은 뒤 박스 이동 100 ms 구간):
0~0.5 s 는 **0.00~0.03 mm** — 박스가 아예 안 움직인다. 0.5 s 에 흡착 해제,
**0.7 s 에 자리잡고**(0.39~2.26 mm) 0.8 s 부터 조용하다 (최악 1.0 s).
-> `RELEASE_MIN_HOLD_S` 0.5 → 0.3 · `settle_wait_s` 0.5 → 0.3 · damping 은 유지.
**공용 파일이라 팀 합의 필요.**

---

## 10-3. ★ 검증에 **사람 눈**을 한 번은 넣는다 (2026-08-25)

돌아갈 때(배치→픽) 이송 고도가 **픽 높이만** 기준이라 적재 최상단을 안 넘었다.
갈 때는 `max(pick_top, put_top, stack_top)` 인데 올 때만 `pick_top` 이었다 —
실측에서 **300 mm 낮은 길**로 스택 위를 지나가며 팔이 박스를 쳤다.

**이걸 잡은 것은 계측이 아니라 화면이었다.** 김병직님이 "배치하고 돌아갈 때
위로 올라가는 걸 설정했는데 적용이 안 되어 있다" 고 지적해서 찾았다.

숫자만 봤을 때 이 버그는 다른 결론으로 위장돼 있었다:

| 관측 | 당시 결론 | 진짜 원인 |
|---|---|---|
| 높은 자리 박스가 낙하 (3회) | *"`min_support_ratio` 0.7 이 부족하다"* | 돌아갈 때 고도 |
| 궤적 스윕이 대조군까지 −6.5 mm | *"게이트가 너무 빡빡하다"* | 같음 |
| 배치 오차 +10.5 mm · `update_heightmap` 거부 | *"물리 잡음"* | 같음 |

고친 뒤 knock **700.8 → 0.8 mm**, `update_heightmap` 거부 1 → 0건.

**규칙: 판정을 확정하기 전에 `--gui` 로 최소 한 번은 눈으로 본다.**
계측은 "무엇이 얼마나" 를 답하고, 화면은 "무엇이 빠졌는지" 를 답한다.
빠진 것은 로그에 안 찍힌다 — 없는 동작은 없는 줄로 나타나지 않는다.

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
| ★ **씬에 ROS2 브리지 노드가 0 개다** | 김학민 | 2026-08-26 실측. 씬의 OmniGraph 26 개가 **전부 컨베이어**고 `/clock`·`/tf`·카메라를 아무도 안 낸다 — `ros2 topic info /tf` 가 **Publisher 0 · Subscription 4**. 평가 항목 §13 "ROS2 연동" 이 미구현이다. `setup_test3_scene.py` [11] 에 /clock·/tf 를 붙였으나 **아직 씬에 적용 안 함** |
| tf 프레임 이름 규약 | 김학민/민범진 | 씬 prim 이름과 일치해야 함. ⚠ **양쪽이 다르다** — 씬은 `CamPick_A`·`CamSpawn`, 트랙 D 하네스(`verify_ros_graph.py`)는 `pick_cam_a`·`cam1` 을 쓴다. `ROS2PublishTransformTree` 는 프레임 이름을 prim 이름으로 정하므로 **prim 을 고치든 하네스를 고치든 한쪽이 바뀌어야 한다** |
| `gripper_clearance_mm` 30 → **10** | 김병직 | 충전율 **+7.2%p** |
| 조각 이음매(7489~7515)에서 박스가 튀는지 | 김학민 | 3호(250)가 걸리면 라인이 막힌다 |
| **서비스 응답의 float 이 안 실린다** | 김학민 | Isaac ROS2 브리지가 `Unable to write float value (nil)`. `SetVelocity.actual_mps` 만 해당. string·bool·**토픽 float32** 는 정상 (TS-023) |
| `/sim/*` · `/conveyor/*` 서비스 경로 확정 | 팀 | §3-3 이 srv 타입만 정했다. `ros_graph.SERVICES` 의 잠정값을 쓰는 중 |
| `msg/SpawnEvent.msg` 존치 여부 | 민범진 | §3 표에 없는데 저장소에 있다. Isaac Action Graph 쪽에서 쓸 수 있어 남겨 뒀다 |
| `BoxInfo.weight_g` vs yaml `mass_g` | 민범진 | §3-1 은 `weight_g`, `system.yaml` 정본은 `mass_g: 3000`. 이름이 다르다 |
| `/algo/reset_heightmap` 경로 확정 | 팀 | §3-3 이 srv 타입(`ResetHeightmap`)만 정하고 **서비스 경로를 안 적었다.** srv 이름에 맞춰 잠정 확정 |
| §12 저장소 구조 누락 2건 | 팀 | `packing_algo` 에 `baseline.py`(§13 챌린지 근거), `packing_robot` 에 **관제탑이 어디 사는지**가 없다 |
| `stage1.total_count` 15 → **30** | 팀 | 15 로는 팔레트가 안 참 (56.7% vs 65.5%) |
| ~~`gripper_clearance_mm` 30 → 10~~ **기각** | 김병직 | ★ **방향이 반대였다.** "흡착 툴 단면 실측" 의 답은 **122.5 mm**(그리퍼 몸통 Ø245 반경)다. 다만 값을 올리는 대신 `_corridor_ok` 의 **높이 기준**을 고쳤다 (2026-08-24) — 링 반경은 122, 비교 기준은 밑면이 아니라 **윗면+그리퍼길이 150**. `clr 60` 안은 협곡을 안 줄이고 적재율만 깎아 폐기 |
| `/sim/despawn_box` Day 4 → **Day 2** | 김학민 | §5 때문에 통합 전 필요 |
| 천장 카메라 z / FOV / 해상도 | 김학민 | D5 픽셀→mm 변환 |
| 색 팔레트 공유 | 김학민 | 스폰 색상과 인식 색상 일치 필요. 2026-08-25: `boxes.colors`(red/green/blue, 채도 최대)를 **`test_3.yaml` 오버레이**에 넣었다 — `system.yaml` 이 공용 파일이라 합의 전까지 그쪽에 둔다. 배송지→존→색을 `by_dest`+`by_color` 로 유도하므로 표는 늘지 않는다 |
| ~~C1 수용기준 5층 → 4층~~ **재개** | 정정인/팀 | 5층 근거(`210×5=1050 ≤ 1300`)는 **팔이 도달한다는 전제**였다. 2026-08-24 `_corridor_ok` 수정 후 4층 — 실측이 그 전제를 깼다. **2026-08-25 추가:** 방향성 통로 + 간격 10 으로 Isaac 에서 **9단 1300mm** 가 섰다. 수용 테스트는 코어 기본값(간격 0·대칭 통로)이라 그 조건을 안 재고 있다 — 기준을 어느 조건에 맞출지가 결정할 것 |
| 트랙 C 수용기준이 **기하 간섭을 안 본다** | 정정인 | `r8·g250` 은 테스트 3개를 다 통과하면서 Isaac 에서 닿는다. 느슨할수록 점수가 오르는 구조. `verify_transit_sweep_isaac.py` 를 게이트로 |
| ~~`w_far` 0.1 유지 여부~~ **유지** | 김병직 | 2026-08-25 Isaac 20박스에서 `w_far` 0.1 유지 + 방향성 통로로 **20/20 · 최대 knock 1.5mm** (`results/dir0.txt`). 문서 18 의 knock 94mm 는 통로가 방향을 안 볼 때의 값이다 (TS-016) |
| ~~방향성 통로 채택 여부~~ **불필요** | 김병직 | 2026-08-25 종결. 정정인님 `w_level` 0.05 + 계획 간격 10 만으로 Isaac 20/20 · **10단 1300mm** · knock 11.5mm · 통과. 방향성 통로 없이 된다. 코드(`corridor_far_ratio`·`corridor_near_gap_mm`)는 **기본 꺼짐**으로 남겨 둔다 — 켜면 knock 이 11.5 → 2.7mm 로 떨어지지만 단수가 10 → 9 로 준다. 지금은 켤 이유가 없다 |
| ★ **계획 간격 0 → 10** (필수) | 정정인 | 2026-08-25 실측으로 **없으면 깨진다.** 간격 0 은 슬롯이 박스와 정확히 같은 크기라, 물리 속성을 규정대로 걸자 박스가 안 들어가고 `IK_FAIL` 이 났다 — 9kg 5호가 걸리며 이웃을 **1046mm 밀어 팔레트 밖으로 떨어뜨렸다** (`results/wlevel.txt`, RESULT 실패). 간격 10 을 넣자 20/20 통과. ⚠ **config 한 줄이 아니다** — `algo_service_node.py:80` 이 `pk.plan(box_id, w, d, h)` 를 간격 없이 부른다. 파이프라인에 그 개념이 아예 없다 |
| 하중 순서 로직 없음 | 정정인/김병직 | 2026-08-25 오전 "안 한다", 낮 "재개", 저녁 **다시 보류**. 재개 근거였던 붕괴 3건이 지지율이 아니라 **돌아갈 때 상승 버그**였다 (§10-3). 고친 뒤 9kg 5호가 z 960 에 올라가도 안 무너진다. **다만 셋 다 한 계획씩만 봤다** — 하중 순서가 필요 없다는 증명은 아니다 |
| ~~사이클 18.3 s~~ **10.6 s** | 김병직 | 2026-08-25 해소. 아래 §10-2 참고. ⚠ 남은 결정 2건: `joint_speed_scale` 0.5 → **1.0** 정본화(공용 파일) · `release` 3상수 1.5 → 1.1 s |
| `min_support_ratio` 0.7 유지 여부 | 김병직 | ⚠ **2026-08-25 낮 결론("0.7 은 안전하지 않다")은 틀렸다.** 붕괴 3건이 전부 **돌아갈 때 상승 버그**(§10-3) 가 있던 상태에서 측정됐다 — 지지율이 아니라 팔이 낮게 지나가며 친 것이었다. 버그를 고친 뒤 **0.7 이 3/3 통과**한다. 다만 같은 시드에서 0.85 보다 느슨하다: knock 6.6 vs 0.8 · 팔레트 밖 10.0 vs 2.6 · 배치오차 10.9 vs 5.4 (적재 개수는 23 로 동일). 완화(0.65·0.6)는 여전히 미검증 |
| ~~팔레트 90° 회전으로 적재율 개선~~ **기각** | 정정인 | 실측 −3.1%p 이고, 애초에 트럭 바닥 2800 에 1200×3=3600 이 안 들어간다. **현행이 이미 돌린 결과**다 (TS-029) |
| 적재율 표기: 실제높이 기준 + **용량 기준 병기** | 팀 | §7.8 정본은 그대로 두고 `capacity_rate` 를 보조로 추가했다. 포화 시 66.9% vs 66.3% 로 사실상 같다 — 발표에 어느 곡선을 쓸지만 정하면 된다 (TS-017) |
| ~~`/robot/cmd` 네임스페이스~~ **해소** | 김병직/김학민 | 2026-08-26. 관제탑은 `/zone_{a|b|c}/robot/cmd` 를 쓰는데 `ros_graph.py` 가 `/robot/cmd` 단수라 **서로 못 만났다** (에러도 안 난다). `ros_graph` 를 구역별로 맞췄다. ⚠ §3-2 표는 아직 단수형이다 — 문서를 고칠지 결정할 것 |
| ~~`db_service_node` 가 죽는다~~ **해소** | 민범진 | 2026-08-26. `schema.sql` 은 `share/packing_db/` 에 깔리는데 노드가 `../share/...` 로 한 단계만 올라가 못 찾았다 (`gen_master_db.py` 도 같은 버그). `ament_index` 로 바꿨다. **트랙 D 파일을 고쳤으니 확인 바람** |
