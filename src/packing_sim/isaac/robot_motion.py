"""B 소유 소켓 명령 핸들러 — Isaac 프로세스 안에서 돈다 (공통가이드 §3, 작업명세서 6.4).

    move_ee · motion_status · grasp · release · is_holding · tcp_speed

`sim_main.py`(김학민)가 이 모듈을 import 해서 `RobotMotion` 을 만들고,
`handlers()` 를 `sim_server` 의 cmd 표에 등록한 뒤, 메인 루프에서 매 물리 스텝 `step()` 을
부른다. **이 파일에는 `import rclpy` 가 한 줄도 없다** — Isaac 3.11 과 Jazzy 3.12 는
ABI 가 안 맞는다.

    # sim_main.py 쪽 (김학민)
    from packing_sim.isaac import robot_motion
    rm = robot_motion.RobotMotion(art, lula, cfg)
    server.register(rm.handlers())
    while running:
        rm.step()
        world.step(render=True)

## ★ 스레드 규약 — 여기가 이 파일의 핵심이다

소켓 핸들러는 **소켓 스레드**에서, `step()` 은 **메인(물리) 스레드**에서 돈다.
규칙 1("핸들러는 절대 블로킹하지 않는다")이 막는 것은 **물리 루프가 서는 것**이지
소켓 스레드가 잠깐 기다리는 것이 아니다. 그래서:

  · `move_ee` 는 목표만 걸고 즉시 `{"accepted":true}`. IK 는 `step()` 이 푼다
    (실패는 `motion_status` 가 `IK_FAIL` 로 알린다). 프로토콜 표 그대로다.
  · `grasp` / `release` 는 표가 `{"ok":bool}` 을 요구한다. 판정은 물리 스텝이
    지나야 나오므로 **소켓 스레드만** Event 로 기다리고, 진행은 `step()` 이 시킨다.
    물리는 그동안 정상으로 돈다.
  · 나머지는 `step()` 이 갱신해 둔 값을 읽기만 한다.

`Gripper` 를 `begin_*` / `poll_*` 로 쓴다. 차단형 `grasp()` 를 부르면 그 안에서 물리를
직접 돌려버려 규칙 1 을 어긴다 — **여기서는 절대 쓰지 말 것.**

## ★ `move_ee` 의 target_m 은 **흡착면**이다

tool0 가 아니다. 수직 하강 자세에서 흡착면은 tool0 보다 `gripper_length_mm` 만큼
아래이므로, 여기서 IK 목표를 만들 때 +Z 로 그 길이를 더한다. 호출자(orchestrator)가
그리퍼 길이를 몰라도 되게 하려는 것이고, 구 저장소에서 이 오프셋을 빼먹어 150 mm
어긋난 적이 있다 (`verify_lula_ik.py` docstring 4번).

## ★ 이송은 **베이스 둘레 호**다 (직선 금지)

픽업(θ≈180°)과 팔레트(θ≈0°)를 직선으로 이으면 중점이 **베이스 바로 위**를 지난다.
반경 0 이라 도달범위 밖이고, 팔이 자기 밑으로 파고들려다 막힌다.
반경과 높이는 선형으로, **각도만 짧은 쪽으로** 돌리면 반경이 절대 0 이 안 된다.

yaw 도 호를 따라 같이 돈다. 월드에 고정하면 j1 이 도는 만큼 손목이 반대로 되감겨
j5 가 한계의 99% 까지 감긴다. 이송 중 박스 방향은 아무래도 좋다.

실측 근거는 `scripts/verify_cycle.py` 다 (15박스 기준):

    평균 17.08 -> 11.89 s · j5 −355.8~+205.1 -> −90~+21° · 툴 기울기 178.5 -> 0.2°
    경로 중간 IK 실패 73 -> 0 스텝

## 이동은 웨이포인트 보간이다

드라이브 타깃을 한 번에 써넣으면(계단 입력) 팔이 최대 토크로 잡아채고 도착하며
오버슈트한다 — 구 저장소에서 박스를 쳐서 날려먹은 원인이다
(`흡착_그리퍼_트러블슈팅` 문제①). 관절 속도한계 × `joint_speed_scale` 로 스텝 수를
정해 나눠 보낸다.
"""
import math
import os
import threading

import numpy as np

STATES = ("IDLE", "MOVING", "REACHED", "IK_FAIL")
BOX_PRIM_FMT = "/World/boxes/%s"          # 공통가이드 §3 규칙 5
# ★ 도착 판정은 **작업공간**에서 한다. 관절 0.5° 는 1 m 리치에서 8.7 mm 라,
#   흡착 접근 간극 2 mm 를 판정할 수 없다 (실측으로 2.14 mm 떠서 붙어 실패했다).
IK_TOL_M = 0.005                          # Lula 해를 FK 로 되돌렸을 때 허용 오차
# ★ 1 mm 였다. **이 제어기의 정상상태 오차보다 작아서 애초에 못 들어가는 값**이었다.
#   2026-08-25 실측 (`results/time2.txt`, 구간 26개): 최소 1.03 · 중앙 1.23 ·
#   90% 1.51 · 최대 2.05 mm. 최소값조차 1 mm 를 넘는다.
#   -> 2 mm 로 푼다. 중앙값이 SETTLE_HOLD(3틱) 만에 통과하고, 꼬리(2.05)는 아래
#      수렴 정체 판정이 받는다.
#   ⚠ 흡착에는 영향이 없다 — `cycle._descend_to_gap` 이 **실측으로 한 번 보정하는
#     닫힌 루프**라 IK 잔차에 안 걸린다. GRIP_TOL_MM 은 3.0 이고 실측 간극은
#     1.28~1.97 mm 였다.
REACH_TOL_M = 0.002                       # 흡착면이 목표에서 이 안이면 REACHED
# ★ 도착 판정에 **자세**도 넣는다 (2026-08-24, TS-015).
#   위치만 보면 손목이 도는 중에 REACHED 가 나가고, 그 상태로 흡착을 놓아
#   박스가 삐딱하게 떨어져 넘어졌다. 실측에서 다음 구간이 yaw 112.9 에서 시작했다.
YAW_TOL_DEG = 2.0                         # 툴 yaw 가 이 안이어야 REACHED
# ★ 1.0 → 2.5 로 늘렸다. 정상 이동은 SETTLE_HOLD 몇 틱이면 빠져나가므로 이 값이
#   길어도 손해가 없다. 이 한도가 실제로 쓰이는 건 "뭔가 안 맞을 때" 뿐이고,
#   바로 그때 더 기다려야 한다. 단 **무한대는 안 된다** — 정상상태 오차가 남는
#   제어기에서는 임계값에 영영 못 들어간다 (구 저장소 RMPflow 교훈).
SETTLE_MAX_S = 2.5                        # 궤적을 다 보낸 뒤 수렴을 기다리는 한도
SETTLE_HOLD = 3                           # 연속 이만큼 만족해야 REACHED
# ★★ 정상상태 오차가 남는 제어기에서는 REACH_TOL_M(1mm)에 **영영 못 들어간다.**
#   2026-08-25 실측(`results/time1.txt`): 웨이포인트 8개가 **전부** SETTLE_MAX_S 를
#   꽉 채웠고, `motion_status` 대기가 사이클 시간의 **80~88%** 였다 (사이클 20초 중
#   16~20초). 그런데 `잔차` 경고는 **0건** — LOG_RESIDUAL_M(5mm) 미만이라는 뜻이다.
#   즉 팔은 이미 멈춰 있었고 1mm 에 못 들어왔을 뿐이다. 주석이 스스로 "1~2mm 는
#   정상 처짐" 이라고 적어 놓고 임계값을 1mm 로 둔 것이 원인이다.
#   -> **절대 임계값이 아니라 "더 줄어드는가"로 끝낸다.** 이게 옛 저장소 RMPflow
#      교훈("정상상태 오차는 성질이다 — 위치 오차로 단계 전환 금지")의 적용이다.
#   ⚠ yaw 는 그대로 YAW_TOL_DEG 를 요구한다. 느슨하게 하면 박스가 삐딱하게 앉아
#     다음 층이 그 위에서 무너진다 (TS-015 재발).
# ★ 정착 구간을 한 줄씩 찍는다 (데모가 --diag-time 에서 켠다). 구간당 430 ms 가
#   어디서 나오는지 — **팔이 움직이는 중인지 이미 멈췄는지** — 이걸로만 갈린다.
SETTLE_LOG = False
SETTLE_PLATEAU_TICKS = 8                  # 이만큼 연속으로
SETTLE_PLATEAU_M = 0.0002                 # 0.2 mm 이상 안 줄면 수렴 끝으로 본다
ARC_MIN_R_M = 0.15                        # 반경이 이보다 작으면 각도가 무의미 — 직선
BISECT = 10                               # 속도한계를 넘으면 u 증분을 이만큼까지 반절
MOVE_LIMIT_S = 20.0                       # 한 구간 상한. 무한루프 방지
LOG_RESIDUAL_M = 0.005                    # 이보다 큰 잔차만 로그. 1~2 mm 는 정상 처짐이다
# ★ 잔차가 이보다 크면 **REACHED 로 보고하지 않는다.** 정상 처짐(1~2 mm)이나
#   RMP 계열 정상상태 오차(check_place_plan 의 RMP_SAG_MM 35)보다 훨씬 큰 값이라
#   여기 걸리면 '못 갔다'가 맞다. 이게 없어서 343 mm 를 못 간 채 REACHED 를 주고,
#   cycle 이 그 자리에서 박스를 놓아 팔레트 밖으로 떨어뜨렸다 (0824error1.mp4).
REACH_ABORT_M = 0.05                      # 50 mm
# ★ yaw 도 같은 이유로 한도를 둔다. main 의 YAW_TOL_DEG(2.0)는 **REACHED 판정**이고
#   이건 **포기 판정**이라 훨씬 느슨하다 — 정상 오차로 실패를 내면 안 된다.
#   90도를 넘으면 박스가 사실상 돌아간 것이라 그대로 놓으면 다음 층이 무너진다.
YAW_ABORT_DEG = 30.0

# ★ IK 가지(branch) 선택 — 팔이 "뻣뻣한" 진짜 이유였다.
#
#   6축 팔은 한 점을 최대 8가지 자세(어깨 좌/우 · 팔꿈치 상/하 · 손목 정/역)로 잡는다.
#   그런데 여기서는 **직전 해를 warm_start 로 물고** 가므로 가지 하나에 갇힌다.
#   그 가지가 나쁘면 이송 내내 못 빠져나온다.
#
#   ⚠ 관절 한계나 자세 공차를 푼다고 해결되지 않는다 — 실측으로 전부 확인했다:
#     · 관절 한계는 j1/j4/j5/j6 ±360°, j2 ±125°, j3 ±160° 로 이미 넉넉하다
#     · descriptor 에 cspace_target 같은 인위적 당김이 없다
#     · orientation_tolerance 를 0.01 -> 3.2 로 열어도 **해가 완전히 동일**하다
#     · Lula 는 supports_collision_avoidance() = False 다
#     · 손목 yaw 를 -180~180 돌려도 j1/j2/j3 가 소수점까지 같고 j6 만 돈다
#       (손목은 팔 끝에 달려 있어 link_2/link_3 위치를 못 바꾼다)
#   즉 자유도가 묶인 게 아니라 **선택지를 안 쓰고 있었다.**
#
#   실측 (scripts/verify_transit_sweep_isaac.py --seeds multi, 이격 400 · 5호 3단):
#     전체 최악 여유  +32.9 mm -> **+70.2 mm** (2.1배)
#     #11 이송고도상승 +32.9 -> +365.1 · #11 가로이송 +32.9 -> +115.0
#   이긴 시드가 박스마다 다르다 (keep/ready/j1flip/wflip 이 골고루) — 하나를 고정할 수 없다.
#
#   ★ 가지는 **구간 시작에서 한 번 고르고 끝까지 유지**한다. 중간에 바꾸면 관절이
#     180도 튄다 (초기 로그에 |dq| 180.0 deg 가 실제로 있었다).
#
#   ★★ 2026-08-24 에는 못 쓰는 구현이었다 (0824error1.mp4). 무엇이 틀렸었나:
#     `_advance_move` 가 매 스텝 IK 를 **직전 해**(p["prev"], q0 에서 출발)로 체인하는데,
#     고른 가지는 `q_end` 에만 들어가고 **경로에는 안 쓰였다** — steps 계산
#     (dq = |q_end - q0|)에만 반영됐다. 즉 팔은 원래 가지로 가고 페이싱만 어긋났다.
#     'wflip' 을 고른 구간에서 도착 잔차 **343 mm**, 그 자리에서 놓아 박스가 떨어졌다.
#
#     측정 도구(verify_transit_sweep_isaac.py --seeds multi)는 **시드에서 체인을
#     시작**하므로 +32.9 -> +70.2 mm 가 나왔다. 컨트롤러는 q0 에서 시작했다. 이 차이다.
#
#   ★★ 2026-08-25 해결. 아래 RECFG_* 참조 — 호를 시작하기 전에 **관절공간으로 팔을
#     그 가지로 옮긴다.** 그러면 컨트롤러도 측정 도구와 같은 자세에서 체인을 시작한다.
#     기본값은 여전히 keep(꺼짐)이다. 켜려면 `PL_IK_SEEDS=multi`.
IK_SEEDS = os.environ.get("PL_IK_SEEDS", "keep")

# ★★ 이송 중 툴 기울기 (2026-08-25). 가지 재구성의 **대안**이다.
#
#   왜: 6축은 툴 위치3+자세3 을 다 묶으면 여유자유도가 0 이라 팔꿈치를 못 움직인다.
#   자세 조건 중 **툴 축 방향**만 조금 풀면 여유가 생긴다. 손목 중심(j4/j5 교점)에서
#   흡착면까지가 121+150 = 271 mm 라 15도면 손목이 70 mm 옆으로 가고, 팔뚝이 그만큼
#   비켜난다. 가지 재구성이 준 이득(+32.9 -> +70.2 mm)과 **같은 크기인데 연속적**이라
#   179도 스윙이 없다 (팔뚝 위치 변경error1.mp4 에서 본 그 동작).
#
#   ⚠ 착지 순간에는 반드시 0 이다 — 매달린 박스가 기울어 착지하면 다음 층이 무너진다
#     (8-24 SAT 15축: 착지 상한 0도). 그래서 sin(pi*u) 로 **구간 양끝에서 0** 이 된다.
#     하강 구간은 애초에 수평 이동이 거의 없어 아래 TILT_MIN_XY_M 에서 걸러진다.
#   기본 0 = 꺼짐. 켜려면 `PL_TOOL_TILT=15`.
TOOL_TILT_DEG = float(os.environ.get("PL_TOOL_TILT", "0"))
TILT_MIN_XY_M = 0.20                      # 이 정도는 가로로 가는 구간에서만 기울인다
TILT_DIRS_DEG = (0.0, 90.0, 180.0, 270.0) # 구간 시작에 중간점으로 채점해 하나 고른다
# ★ 기울기는 **필요한 만큼만** 쓴다 (김병직 요청 2026-08-25 "깊은 부분만").
#   후보를 작은 것부터 보고 여유가 이만큼 확보되면 거기서 멈춘다. 얕은 자리는
#   기울기 0 으로 지나가고, 깊은 협곡에서만 최대치까지 올라간다.
#   ⚠ 매달린 9kg 박스를 기울이는 것이므로 작을수록 좋다 — 전단력이 흡착 한계를 먹는다.
TILT_ENOUGH_M = 0.06                      # 이만큼 여유가 나오면 더 안 기울인다 (60 mm)
TILT_FRACTIONS = (0.34, 0.67, 1.0)        # 최대치의 1/3 · 2/3 · 전부
SEED_PROBE_N = 8                          # 시드 점수용 경로 표본 수 (성기게)
# ★ 현행 가지(keep)가 이만큼 여유로우면 **갈아타지 않는다.**
#   가지를 바꾸면 관절 이동량이 커져 사이클이 길어진다 — 실측 6박스에서
#   13~17 s -> 12~32 s 로 거의 2배가 됐다 (IK 계산 비용이 아니라 j1flip 이 팔을
#   반대편으로 돌리기 때문이다). 여유가 이미 충분하면 그 비용을 낼 이유가 없다.
#   값은 배치 오차(실측 7 mm)와 RMP 처짐(35 mm)을 합친 것보다 넉넉하게 잡는다.
SEED_KEEP_OK_M = 0.06                     # 60 mm

# ★★ 가지 재구성 (2026-08-25). 위 "아직 쓰면 안 된다" 의 해결이다.
#
#   고장은 이랬다: `_pick_seed` 가 고른 q_end 는 **페이싱(dq) 계산에만** 쓰였고,
#   `_advance_move` 는 p["prev"] = q0 에서 IK 를 체인했다. 즉 팔은 원래 가지로 가고
#   끝점만 다른 가지의 값이라 도착 잔차가 343 mm 났다.
#
#   고침: 가지를 갈아타기로 했으면 **호를 시작하기 전에 관절공간으로 팔을 그 가지로
#   옮긴다.** 옮기는 목표는 **지금 그 자리에서의** 그 가지 해(q_re)다 — 툴은 제자리에
#   두고 팔꿈치/어깨만 바꾼다. 그래야 호 파라미터(f0 기준)가 그대로 유효하다.
#
#   ⚠ 관절공간 이동은 툴이 제자리여도 **팔꿈치가 공간을 훑는다.** 그래서 재구성 경로
#     자체도 장애물 여유를 표본으로 채점하고, 못 지키면 그 가지를 **후보에서 뺀다.**
#     (끝점과 호만 보고 고르면 옮기는 도중에 치는 가지를 못 거른다 — 8-24 와 같은 실수)
#
#   ⚠⚠ 2026-08-25 실측 (팔뚝 위치 변경error1.mp4). 처음 판은 **하강 구간에서**
#     179도짜리 어깨 뒤집기(j1flip)를 골라 팔을 스택 옆에서 크게 휘둘렀다.
#     6축 팔은 여유자유도가 0 이라 **"툴은 제자리, 팔뚝만 회전"이 존재하지 않는다** —
#     가지 사이에는 연속 경로가 없고 툴이 반드시 크게 움직인다. 그러니 재구성은
#     **툴이 더미 위로 확실히 떠 있을 때만** 허용해야 한다.
#     그래서 재구성 표본마다 **툴 높이**도 같이 본다. 장애물 최고점 + 여유 아래로
#     내려가는 가지는 탈락시킨다 — 하강 구간은 이 조건에서 자동으로 걸러진다.
RECFG_PROBE_N = 16                        # 재구성 관절경로 표본 수 (툴이 크게 휘므로 촘촘히)
RECFG_MIN_M = 0.05                        # 재구성 도중 최소 여유 [m]. 못 지키면 그 가지 탈락
RECFG_CLEAR_Z_M = 0.10                    # 재구성 중 툴이 장애물 최고점보다 이만큼 위여야 한다
RECFG_OVER_XY_M = 0.20                    # "더미 위" 로 볼 수평 여유 [m]

# ★★ 접촉 감지 — 닿으면 **그 자리에서 멈춘다** (2026-08-25, 김병직 제안).
#
#   지금까지는 못 따라가도 지령을 계속 전진시켰다 (`qk = p["prev"]` 로 유지하고
#   u 는 계속 증가). 그리고 `_settle_move` 가 정체를 인정하기까지 8틱을 더 눌렀다.
#   그동안 팔은 이웃 박스를 **계속 밀고 있다** — 실측 knock 16.2 · 31.5 mm 가 거기서 났다.
#
#   신호: 지령은 나가는데 **툴이 안 움직이면** 닿은 것이다. 그러면 지령을 지금
#   **실제 관절각**으로 되돌려 미는 힘을 없애고 구간을 끝낸다. 두산 실기의 토크센서
#   충돌정지와 같은 취지인데, 여기선 토크 대신 **툴 정지**로 감지한다
#   (Lula 는 collision avoidance 를 지원하지 않는다 — supports_collision_avoidance()=False).
#
#   ⚠⚠ 첫 판은 "틱당 절대 이동량 < 0.3 mm 를 u < 0.9 구간에서" 로 봤는데 **한 번도
#     안 불렸다** (2026-08-25 실측, 접촉 감지 0회). `_advance_move` 는 팔이 막혀도
#     u 를 끝까지 밀어붙이므로, 툴이 멈출 때쯤엔 u 가 이미 0.9 를 넘어 있다.
#     -> **지령 대비 실제 이동 비율**로 본다. 구간 끝에서는 지령도 같이 느려지므로
#        비율은 그대로 유효하고, 창을 자를 필요도 없다.
#   기본 0 = 꺼짐. 켜려면 `PL_CONTACT_STOP=1`.
CONTACT_STOP = os.environ.get("PL_CONTACT_STOP", "0") not in ("0", "", "off")
CONTACT_TICKS = 6                         # 이만큼 연속으로 못 따라가면 닿은 것
CONTACT_RATIO = 0.25                      # 실제/지령 이동이 이 밑이면 "못 따라감"
CONTACT_MIN_CMD_M = 0.0005                # 지령이 이만큼은 움직여야 판정한다 (0.5 mm)
# ★ 계측 — 하강 구간의 **매 틱**을 찍는다. 판정 조건을 세 번 잘못 잡았고(창·절대이동·yaw)
#   그때마다 로그가 답을 갖고 있었다. 추측 대신 이걸 먼저 본다 (CLAUDE.md §10-2 교훈).
#   이송 구간은 제외한다 — 길고 관심사가 아니다. `PL_DIAG_CONTACT=1`.
DIAG_CONTACT = os.environ.get("PL_DIAG_CONTACT", "0") not in ("0", "", "off")


def _wrap(rad):
    """각을 (-pi, pi] 로 접는다 — 항상 짧은 쪽으로 돈다."""
    return math.atan2(math.sin(rad), math.cos(rad))


def _near(target_deg, near_deg, period=180.0):
    """직사각 박스는 yaw 와 yaw+180 이 같은 자리다. near 에 가장 가까운 대표각."""
    return target_deg + round((near_deg - target_deg) / period) * period


def _tool_yaw_deg(R):
    """수직 하강 자세의 툴 yaw [deg]. R = Rz(yaw)·Ry(pi) 를 되푼다."""
    R = np.asarray(R, dtype=float)
    return math.degrees(math.atan2(-R[1, 0], -R[0, 0]))


def _quat_down(yaw_deg, tilt_deg=0.0, tilt_dir_deg=0.0):
    """툴 z 축이 월드 −Z 를 보고, 그 축 둘레로 yaw 만큼 돌린 자세. (w, x, y, z)

    `tilt_deg` 를 주면 그 축을 `tilt_dir_deg` 방향으로 그만큼 **기울인다**.
    0 이면 예전과 완전히 같다 (기본값).

    ★ 왜 기울이나 (2026-08-25): 6축 팔은 툴의 위치3+자세3 을 다 묶으면 여유자유도가
      0 이라 팔꿈치를 못 움직인다. 자세 조건 중 **툴 축 방향**을 조금 풀면 여유가
      생긴다. 손목 중심(j4/j5 교점)에서 흡착면까지가 121+150 = **271 mm** 라
      15도면 손목이 **70 mm** 옆으로 간다 — 팔뚝이 그만큼 비켜난다.
      가지 재구성(+32.9 -> +70.2 mm)과 같은 크기인데 **연속적**이라 179도 스윙이 없다.
    ⚠ 착지 순간에는 0 이어야 한다. 매달린 박스가 기울어 착지하면 다음 층이 무너진다
      (8-24 SAT 15축 실측: 착지 상한 0도). 그래서 구간 양끝에서 0 으로 수렴시킨다.
    """
    ry, rz = math.radians(180.0 - tilt_deg), math.radians(yaw_deg)
    rd = math.radians(tilt_dir_deg)
    # R = Rz(yaw) · Rz(dir) · Ry(180-tilt) · Rz(-dir)
    #   Ry(180-tilt) 로 아래를 보되 tilt 만큼 덜 눕히고, Rz(dir) 로 기울일 방위를 정한다.
    def _rz(a):
        c, s_ = math.cos(a), math.sin(a)
        return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    c, s_ = math.cos(ry), math.sin(ry)
    Ry = np.array([[c, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c]])
    R = _rz(rz) @ _rz(rd) @ Ry @ _rz(-rd)
    return _mat_to_quat(R)


def _quat_zaxis(q):
    """(w,x,y,z) -> 그 자세의 툴 z 축 (월드). R[:, 2] 와 같다."""
    w, x, y, z = (float(v) for v in q)
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def _mat_to_quat(R):
    """회전행렬 -> (w, x, y, z). Lula 가 이 순서를 받는다."""
    t = float(np.trace(R))
    if t > 0.0:
        s_ = math.sqrt(t + 1.0) * 2.0
        return np.array([0.25 * s_, (R[2, 1] - R[1, 2]) / s_,
                         (R[0, 2] - R[2, 0]) / s_, (R[1, 0] - R[0, 1]) / s_])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s_ = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2.0
    q = np.zeros(4)
    q[0] = (R[k, j] - R[j, k]) / s_
    q[1 + i], q[1 + j], q[1 + k] = 0.25 * s_, (R[j, i] + R[i, j]) / s_, (R[k, i] + R[i, k]) / s_
    return q


class RobotMotion:
    """B 명령 6개를 처리한다.

    Args:
        articulation: `SingleArticulation` (초기화 완료 상태)
        lula: `LulaKinematicsSolver`. **`set_robot_base_pose()` 까지 끝난 것**
        cfg: `system.yaml` 딕셔너리
        ee: EE 프레임명. 기본 `tool0`
        gripper: `packing_robot.gripper.Gripper`. 안 주면 `gripper_path` 로 만든다
        gripper_path: SurfaceGripper 프림 경로
        gap_fn: () -> 흡착면~박스 윗면 간극 [mm]. 주면 grasp 판정이 실측까지 본다
        log: 로그 함수
    """

    def __init__(self, articulation, lula, cfg, ee="tool0", gripper=None,
                 gripper_path=None, gap_fn=None, log=print, collision_spheres=None,
                 ready_q=None):
        self.art = articulation
        self.lula = lula
        self.cfg = cfg
        self.ee = ee
        self._log = log

        r = cfg["robot"]
        self.grip_len = r["gripper_length_mm"] / 1000.0
        self.hz = float(cfg["physics"]["scene"]["time_steps_per_second"])
        self.dt = 1.0 / self.hz
        self.speed_scale = float(r.get("joint_speed_scale", 0.5))
        self.vlim = np.asarray(self.art.dof_properties["maxVelocity"][:6], dtype=float)
        self.cap = self.vlim * self.speed_scale / self.hz    # 스텝당 허용 관절 변화 [rad]
        self.base_xy = np.asarray(self.art.get_world_pose()[0], dtype=float)[:2]
        # ★ 충돌구체는 **상위에서 받는다.** 안 주면 시드 선택이 꺼지고 예전 동작 그대로다
        #   (sim_main 처럼 안 넘기는 호출부가 그대로 살아 있어야 한다).
        #   descriptor 경로를 여기서 알 방법이 없어서 인자로 받는다 — Lula 는 이미
        #   그 파일로 만들어져 있으므로 호출부는 알고 있다.
        self.spheres = collision_spheres or {}
        self.ready_q = None if ready_q is None else np.asarray(ready_q, dtype=float)
        self._obst = []                       # move_ee 가 실어 주는 장애물 AABB [m]

        from isaacsim.core.utils.types import ArticulationAction
        self._Action = ArticulationAction

        if gripper is None:
            if gripper_path is None:
                raise ValueError("gripper 또는 gripper_path 중 하나는 줘야 한다")
            import gripper as G                       # packing_robot 패키지
            gripper = G.Gripper(gripper_path, articulation=articulation, cfg=cfg,
                                step=None, gap_fn=gap_fn,
                                tool_pose_fn=self.tool_pose, log=log)
        self.grip = gripper

        self._lk = threading.Lock()
        self._state = "IDLE"
        self._req = None            # move_ee 요청 (아직 IK 안 푼 것)
        self._path = None           # 진행 중인 호 경로
        self._goal_face = None      # 이번 이동의 흡착면 목표 (작업공간 판정용)
        self._goal_yaw = 0.0        # 같은 이동의 툴 yaw 목표 (자세 판정용)
        self._hold = 0
        self._settle = 0
        self._best, self._flat = float("inf"), 0
        self._s0 = None
        self._tcp_mps = 0.0
        self._prev_tool = None
        self._g_evt = None          # grasp 결과 대기
        self._g_res = None
        self._g_pend = None         # begin_grasp 예약 (메인 스레드가 집행)
        self._r_evt = None          # release 결과 대기
        self._r_res = None
        self._r_pend = False        # begin_release 예약
        self._open_pend = False     # 타임아웃 강제 개방 예약
        # ★ 소켓 스레드가 읽는 값은 전부 **캐시**다. PhysX 는 물리 스텝과 겹치는
        #   다른 스레드의 읽기/쓰기를 undefined behavior 로 못박는다 — 실제로
        #   `copyInternalStateToCache() not allowed while simulation is running` 이
        #   콘솔을 도배했다. 메인 스레드의 step() 이 매 스텝 갱신하고 핸들러는
        #   이것만 돌려준다 (한 스텝 = 16 ms 지연, 판정에는 무의미).
        self._face_cache = self.suction_pose()
        self._holding_cache = False

    # ── sim_main 이 부르는 것 ────────────────────────────────
    def handlers(self):
        """`sim_server` 의 cmd 표에 등록할 {cmd: fn}."""
        return {"move_ee": self.h_move_ee,
                "motion_status": self.h_motion_status,
                "grasp": self.h_grasp,
                "release": self.h_release,
                "is_holding": self.h_is_holding,
                "tcp_speed": self.h_tcp_speed}

    def step(self):
        """매 물리 스텝 한 번. **메인 스레드 전용.**

        `world.step()` **앞**에서 부른다 — 이번 스텝에 쓸 드라이브 목표를 여기서 건다.
        """
        self._update_tcp_speed()
        with self._lk:
            if self._req is not None:
                self._start_move(self._req)
                self._req = None
            self._advance_move()
            if self._g_pend is not None:            # 소켓 스레드가 예약한 흡착 시작
                self.grip.begin_grasp(self._g_pend[0])
                self._g_pend = None
            if self._r_pend:
                self.grip.begin_release()
                self._r_pend = False
            if self._open_pend:
                self.grip.open()
                self._open_pend = False
            self._advance_gripper()
            self._face_cache = self.suction_pose()
            self._holding_cache = bool(self.grip.is_holding())

    # ── 소켓 핸들러 (소켓 스레드) ────────────────────────────
    def h_move_ee(self, req):
        """목표만 걸고 즉시 응답한다. IK 는 다음 물리 스텝에 푼다."""
        try:
            target = np.asarray(req["target_m"], dtype=float)
            if target.shape != (3,):
                raise ValueError
        except Exception:
            return {"accepted": False, "error": "bad target_m"}
        # ★ 장애물은 **상위(cycle)가 준다.** robot_motion 이 씬을 뒤지지 않는다 —
        #   무엇을 피할지는 상위 판단이고 여기는 실행기다 (CLAUDE.md §3 · D2).
        #   형식: [[[lox,loy,loz],[hix,hiy,hiz]], ...] 월드 m. 없으면 시드 선택이 꺼진다.
        obst = []
        for ab in (req.get("obstacles") or []):
            try:
                lo, hi = np.asarray(ab[0], dtype=float), np.asarray(ab[1], dtype=float)
                if lo.shape == (3,) and hi.shape == (3,):
                    obst.append((lo, hi))
            except Exception:
                pass                          # 잘못된 항목 하나가 이동을 막으면 안 된다
        with self._lk:
            self._obst = obst
            # ★ 구간 허용 오차도 같이 싣는다. 소켓 스레드가 만든 튜플을 물리 스레드가
            #   그대로 받는다 — dict 를 넘기면 나중에 누가 고쳐 경합이 난다.
            self._req = (target, float(req.get("yaw_deg", 0.0)),
                         float(req.get("tol_m") or REACH_TOL_M))
            self._state = "MOVING"
            self._path = None
        return {"accepted": True}

    def h_motion_status(self, req=None):
        """상태 + **지금 흡착면 위치**. face_m 은 프로토콜 표에 없는 추가 필드다 —
        오케스트레이터가 접근을 닫힌 루프로 보정하는 데 쓴다 (B 소유 명령이라 안전)."""
        with self._lk:
            st, face = self._state, self._face_cache
        return {"state": st, "face_m": [float(v) for v in face]}

    def h_grasp(self, req):
        """흡착. 판정이 날 때까지 **이 스레드만** 기다린다 (물리는 계속 돈다)."""
        box_id = (req or {}).get("box_id")
        path = BOX_PRIM_FMT % box_id if box_id else None
        evt = threading.Event()
        with self._lk:
            if self._g_evt is not None:
                return {"ok": False, "error": "grasp already in progress"}
            self._g_evt, self._g_res = evt, None
            self._g_pend = (path,)                               # 메인 스레드가 집행
        # 그리퍼가 닫기를 시도하는 총 시간 + 여유. 안 오면 실패로 본다.
        if not evt.wait(self.grip.retry_s + 2.0):
            with self._lk:
                self._g_evt = None
            self._log("[robot_motion] grasp 응답 타임아웃")
            return {"ok": False, "error": "timeout"}
        with self._lk:
            res = self._g_res
        return {"ok": bool(res), "box_id": box_id}

    def h_release(self, req=None):
        """놓기. 명세서 6.5 순서를 `Gripper` 가 진행하고 여기서는 결과만 기다린다."""
        evt = threading.Event()
        with self._lk:
            if self._r_evt is not None:
                return {"ok": False, "error": "release already in progress"}
            self._r_evt, self._r_res = evt, None
            self._r_pend = True                                  # 메인 스레드가 집행
        budget = self.grip.release_timeout_s + self.grip.release_damping_s \
            + self.grip.settle_s + 2.0
        if not evt.wait(budget):
            with self._lk:
                self._r_evt = None
            self._log("[robot_motion] release 응답 타임아웃 — 강제로 연다")
            with self._lk:
                self._open_pend = True                           # open 도 메인 스레드에서
            return {"ok": False, "error": "timeout"}
        with self._lk:
            res = self._r_res
        return {"ok": bool(res)}

    def h_is_holding(self, req=None):
        with self._lk:
            return {"holding": self._holding_cache}

    def h_tcp_speed(self, req=None):
        with self._lk:
            return {"mps": self._tcp_mps}

    # ── 내부 ─────────────────────────────────────────────────
    def tool_pose(self):
        """지금 **관절값 그대로**의 tool0 월드 위치 (3,) [m]."""
        q = np.asarray(self.art.get_joint_positions()[:6], dtype=float)
        return np.asarray(self.lula.compute_forward_kinematics(self.ee, q)[0], dtype=float)

    def suction_face(self):
        """흡착면 월드 위치 (3,) [m] 와 툴 yaw [deg] 를 **FK 한 번**으로 같이 낸다.

        settle 은 매 틱 돌기 때문에 위치와 yaw 를 따로 구하면 FK 가 두 배가 된다.
        """
        q = np.asarray(self.art.get_joint_positions()[:6], dtype=float)
        p, R = self.lula.compute_forward_kinematics(self.ee, q)
        R = np.asarray(R)
        return np.asarray(p, dtype=float) + R[:, 2] * self.grip_len, _tool_yaw_deg(R)

    def suction_pose(self):
        """흡착면 월드 위치 (3,) [m]. 툴 +Z 로 그리퍼 길이만큼 나간 점."""
        return self.suction_face()[0]

    def _update_tcp_speed(self):
        cur = self.tool_pose()
        with self._lk:
            if self._prev_tool is not None:
                self._tcp_mps = float(np.linalg.norm(cur - self._prev_tool)) / self.dt
            self._prev_tool = cur

    def _ik(self, face, yaw_deg, seed, tilt_deg=0.0, tilt_dir_deg=0.0):
        """흡착면 목표 -> (q, ok). tool0 목표는 흡착면 + 그리퍼 길이다.

        `ArticulationKinematicsSolver` 를 쓰지 않는다 — 그건 현재 관절각을 시드로
        **강제**해서(`articulation_kinematics_solver.py:94`) 시드를 고를 수가 없다.
        """
        quat = _quat_down(yaw_deg, tilt_deg, tilt_dir_deg)
        # ★ 플랜지 목표는 흡착면에서 **툴 축을 따라** 그리퍼 길이만큼 뒤다.
        #   기울기가 0 이면 툴 축이 -Z 라 예전과 같은 face + [0,0,grip_len] 이 된다.
        #   기울였을 때 그 식을 그대로 쓰면 흡착면이 옆으로 어긋난다.
        zax = _quat_zaxis(quat)
        tgt = np.asarray(face, dtype=float) - zax * self.grip_len
        q, ok = self.lula.compute_inverse_kinematics(
            frame_name=self.ee, target_position=tgt,
            target_orientation=quat,
            warm_start=np.asarray(seed, dtype=float))
        return (np.asarray(q, dtype=float) if ok else None), bool(ok)

    # ── IK 가지 선택 ─────────────────────────────────────────
    def _seed_bank(self, q):
        """가지를 갈아탈 후보 시드 `[(이름, q), ...]`.

        실측(results/proxy_test.txt)에서 **실제로 다른 해로 가는 것은 이 넷뿐**이었다.
        시드를 더 넣어도 같은 해로 수렴한다 — 가지 수는 기구학이 정한다.
        """
        q = np.asarray(q, dtype=float)
        out = [("keep", q)]
        if IK_SEEDS == "keep" or not self.spheres or not self._obst:
            return out                        # 점수를 못 매기면 고를 이유가 없다
        if self.ready_q is not None:
            out.append(("ready", self.ready_q))
        f = q.copy()                          # 어깨 반대편 — 같은 점을 반대쪽에서 잡는다
        f[0] = _wrap(q[0] + math.pi); f[1], f[2] = -q[1], -q[2]
        out.append(("j1flip", f))
        w = q.copy()                          # 손목 뒤집기 — 팔은 그대로, 손목만 다른 가지
        w[3] = _wrap(q[3] + math.pi); w[4] = -q[4]; w[5] = _wrap(q[5] + math.pi)
        out.append(("wflip", w))
        return out

    def _arm_clearance(self, q):
        """자세 q 에서 팔 링크와 **실려 온 장애물** 사이 최소여유 [m]. 없으면 +inf.

        링크 포즈는 Lula FK 로 낸다 — `get_all_frame_names()` 에 link_2~tool0 이 다 있어서
        별도 FK 모듈이 필요 없다. 상대는 `move_ee` 가 준 AABB 다 (씬을 직접 안 읽는다 —
        판단 근거는 상위가 준다. CLAUDE.md §3).
        """
        worst = float("inf")
        for link, sph in self.spheres.items():
            if link in ("base_link", "link_1"):
                continue                      # 베이스 직상 — 팔레트에 닿을 일이 없다
            try:
                pos, rot = self.lula.compute_forward_kinematics(link, q)
            except Exception:
                continue                      # Lula 가 모르는 프레임은 건너뛴다
            pos, rot = np.asarray(pos), np.asarray(rot)
            for c, r in sph:
                w = pos + rot @ np.asarray(c, dtype=float)
                for lo, hi in self._obst:
                    over = np.maximum(lo - w, w - hi)
                    d = (float(np.linalg.norm(np.maximum(over, 0.0)))
                         if np.any(over > 0) else float(np.max(over))) - r
                    if d < worst:
                        worst = d
        return worst

    def _recfg_ok(self, q0, q_re):
        """q0 -> q_re 관절공간 이동이 장애물을 안 치는지. -> (통과여부, 최소여유 [m])

        ★ 툴이 제자리여도 **팔꿈치는 공간을 훑는다.** 이 검사가 없으면 "끝점은 여유로운데
          거기 가는 길에 치는" 가지를 고르게 된다.
        """
        # ★ 장애물(=이미 놓인 박스) 봉투. 재구성 중 툴이 **더미 위를 지날 때만**
        #   높이를 따진다. 컨베이어 쪽은 팔레트에서 멀어 크게 휘둘러도 상관없는데,
        #   높이만 보면 거기서도 다 막혀 기능이 아예 안 켜진다.
        top = max((float(hi[2]) for _, hi in self._obst), default=None)
        z_min = None if top is None else top + RECFG_CLEAR_Z_M
        if z_min is not None:
            lo_xy = np.min([lo[:2] for lo, _ in self._obst], axis=0) - RECFG_OVER_XY_M
            hi_xy = np.max([hi[:2] for _, hi in self._obst], axis=0) + RECFG_OVER_XY_M
        worst = float("inf")
        for k in range(0, RECFG_PROBE_N + 1):
            u = k / float(RECFG_PROBE_N)
            q = q0 + (q_re - q0) * u
            if z_min is not None:
                # 툴이 더미 속으로 들어가는 순간이 하나라도 있으면 그 가지는 못 쓴다.
                # (하강 구간에서 고르는 것도 여기서 같이 걸러진다 — 시작점부터 낮다)
                # ⚠ 재구성 중에는 툴이 기울 수 있다. "플랜지 z - 그리퍼 길이" 로
                #   퉁치면 기운 자세에서 틀린다 — 회전을 그대로 써서 흡착면을 구하고,
                #   플랜지와 흡착면 중 **낮은 쪽**을 본다.
                fp, fR = self.lula.compute_forward_kinematics(self.ee, q)
                fp, fR = np.asarray(fp, dtype=float), np.asarray(fR)
                face = fp + fR[:, 2] * self.grip_len
                over = np.all(face[:2] >= lo_xy) and np.all(face[:2] <= hi_xy)
                tz = min(float(fp[2]), float(face[2]))
                if over and tz < z_min:
                    return False, -1.0
            worst = min(worst, self._arm_clearance(q))
        return worst >= RECFG_MIN_M, worst

    def _pick_seed(self, face, yaw_to, q0):
        """끝점·재구성·호를 함께 보고 가장 여유로운 가지를 고른다.

        -> (q_end, ok, 이름, q_re)   q_re 는 호 시작 전에 관절공간으로 옮겨 갈 자세.
           'keep' 이면 q_re 는 None 이고 예전과 완전히 같은 경로다.

        ★ 구간 **시작에서 한 번** 고르고 끝까지 유지한다. 중간에 바꾸면 관절이 180도 튄다.
        경로는 성기게(SEED_PROBE_N) 훑는다 — 끝점만 보면 도중에 파고드는 가지를 못 거른다.
        """
        best = None
        f0 = self.suction_pose()
        bank = self._seed_bank(q0)
        # ★ 현행 가지가 이미 충분히 여유로우면 그대로 간다 (위 SEED_KEEP_OK_M 주석).
        if len(bank) > 1:
            q_keep, ok_keep = self._ik(face, yaw_to, q0)
            if ok_keep and self._arm_clearance(q_keep) >= SEED_KEEP_OK_M:
                fk = np.asarray(self.lula.compute_forward_kinematics(self.ee, q_keep)[0])
                if np.linalg.norm(fk - (np.asarray(face)
                                        + np.array([0.0, 0.0, self.grip_len]))) <= IK_TOL_M:
                    return q_keep, True, "keep", None
        for name, sd in bank:
            q_end, ok = self._ik(face, yaw_to, sd)
            if not ok:
                continue
            fk = np.asarray(self.lula.compute_forward_kinematics(self.ee, q_end)[0])
            if np.linalg.norm(fk - (np.asarray(face)
                                    + np.array([0.0, 0.0, self.grip_len]))) > IK_TOL_M:
                continue
            # ★ 갈아탈 가지면 **지금 자리에서의** 그 가지 해를 먼저 구한다. 호는 여기서
            #   출발한다. 이게 없으면 팔은 원래 가지로 가고 끝점만 달라져 잔차가 남는다.
            q_re = None
            if name != "keep":
                q_re, ok_re = self._ik(f0, self._cur_yaw_deg(), sd)
                if not ok_re:
                    continue
                pass_re, worst_re = self._recfg_ok(q0, q_re)
                if not pass_re:
                    self._log("[robot_motion] 가지 '%s' 기각 — %s"
                              % (name, "재구성 중 툴이 더미 위로 안 뜬다" if worst_re < 0
                                 else "재구성 도중 여유 %+.0f mm" % (worst_re * 1000)))
                    continue
            # 호는 q_re(있으면)에서 체인해 채점한다 — 실제로 갈 경로와 같아야 한다.
            q = q_re if q_re is not None else q_end
            score = self._arm_clearance(q_end)
            if self._obst and self.spheres:
                for k in range(1, SEED_PROBE_N):
                    u = k / float(SEED_PROBE_N)
                    qi, oki = self._ik(f0 + (np.asarray(face) - f0) * u, yaw_to, q)
                    if not oki:
                        continue
                    q = qi
                    score = min(score, self._arm_clearance(qi))
            if best is None or score > best[0]:
                best = (score, q_end, name, q_re)
        if best is None:
            return None, False, "", None
        return best[1], True, best[2], best[3]

    def _cur_yaw_deg(self):
        """현재 툴 yaw [deg]. 재구성 목표는 **툴을 안 움직이는** 해라야 한다."""
        q = np.asarray(self.art.get_joint_positions()[:6], dtype=float)
        return _tool_yaw_deg(np.asarray(self.lula.compute_forward_kinematics(self.ee, q)[1]))

    def _at(self, p, u):
        """호 위의 흡착면 좌표. 반경·높이는 선형, 각도는 짧은 쪽."""
        if not p["arc"]:
            return p["f0"] + (p["f1"] - p["f0"]) * u
        r = p["r0"] + (p["r1"] - p["r0"]) * u
        th = p["th0"] + p["dth"] * u
        return np.array([self.base_xy[0] + r * math.cos(th),
                         self.base_xy[1] + r * math.sin(th),
                         p["f0"][2] + (p["f1"][2] - p["f0"][2]) * u])

    def _start_move(self, req):
        """호 경로를 세운다. 끝점 IK 로 타당성과 페이싱을 먼저 확인한다."""
        face, yaw_req, tol_m = req
        q0 = np.asarray(self.art.get_joint_positions()[:6], dtype=float)
        f0 = self.suction_pose()
        _, R0 = self.lula.compute_forward_kinematics(self.ee, q0)
        yaw_from = _tool_yaw_deg(R0)
        # 직사각 박스는 yaw 와 yaw+180 이 같은 자리다. 손목을 덜 돌리는 쪽으로 잡는다.
        yaw_to = _near(yaw_req, yaw_from, 180.0)

        # ★ 가지를 여기서 한 번 고른다 (구간 내내 유지된다). 장애물이나 충돌구체가
        #   없으면 _seed_bank 가 keep 하나만 주므로 예전과 완전히 같은 경로다.
        q_end, ok, seed_name, q_re = self._pick_seed(face, yaw_to, q0)
        if ok and seed_name and seed_name != "keep":
            self._log("[robot_motion] IK 가지 '%s' 선택 (끝점 여유 %+.0f mm · 재구성 %.1f deg)"
                      % (seed_name, self._arm_clearance(q_end) * 1000,
                         float(np.rad2deg(np.max(np.abs(q_re - q0))))))
        if not ok:
            # ★ ok 플래그만 믿지 않는다. 해를 FK 로 되돌려 목표와 맞는지까지 본다.
            self._state = "IK_FAIL"
            self._path = None
            self._log("[robot_motion] 끝점 IK 실패 target(흡착면) %s yaw %.1f"
                      % (np.round(face, 4), yaw_to))
            return

        v0, v1 = f0[:2] - self.base_xy, np.asarray(face)[:2] - self.base_xy
        r0, r1 = float(np.linalg.norm(v0)), float(np.linalg.norm(v1))
        th0 = math.atan2(v0[1], v0[0])
        # ★ 페이싱은 **호가 실제로 출발하는 자세**에서 잰다. 재구성이 있으면 q_re 다.
        q_arc0 = q0 if q_re is None else q_re
        dq = np.abs(q_end - q_arc0)
        secs = float(np.max(dq / (self.vlim * self.speed_scale))) if np.any(dq) else 0.0
        steps = max(1, int(math.ceil(secs * self.hz)))
        # 재구성 구간도 같은 속도 상한으로 스텝을 나눈다
        re_steps = 0
        if q_re is not None:
            dqr = np.abs(q_re - q0)
            re_secs = float(np.max(dqr / (self.vlim * self.speed_scale))) if np.any(dqr) else 0.0
            re_steps = max(1, int(math.ceil(re_secs * self.hz)))
        self._path = {"f0": f0, "f1": np.asarray(face, dtype=float),
                      "arc": r0 > ARC_MIN_R_M and r1 > ARC_MIN_R_M,
                      "r0": r0, "r1": r1, "th0": th0,
                      "dth": _wrap(math.atan2(v1[1], v1[0]) - th0),
                      "yaw0": yaw_from, "yaw1": yaw_to,
                      "u": 0.0, "du": 1.0 / steps, "prev": q_arc0, "k": 0,
                      "miss": 0, "over": 0,
                      # ★ 재구성 구간 — 호보다 **먼저** 돈다. q0 -> re_q 관절 보간이고
                      #   툴은 제자리다(re_q 가 같은 흡착면 자세의 다른 가지 해라서).
                      #   없으면(keep) re_q 가 None 이라 이 구간을 통째로 건너뛴다.
                      # 접촉 감지 상태 (위 CONTACT_* 주석)
                      "cprev": None, "ccmd": None, "cstill": 0,
                      "re_q": q_re, "re_q0": q0, "re_u": 0.0,
                      "re_du": (1.0 / re_steps) if re_steps else 1.0,
                      # ★ 구간마다 요구 정밀도가 다르다. 이송(상공 이동·후퇴)은 mm 로
                      #   도착할 이유가 없고, 정밀해야 하는 건 집는/놓는 하강뿐이다.
                      #   상위(cycle)가 `tol_m` 으로 준다. 안 주면 기본값 = 예전 동작.
                      "tol": tol_m}
        # ★ 기울기 방향은 **구간 시작에 한 번** 고른다 (중간점 u=0.5 로 채점).
        #   매 스텝 고르면 방향이 튀어 손목이 떤다. 이득이 없으면 0 으로 둔다.
        p_tmp = self._path
        tilt_deg, tilt_dir = 0.0, 0.0
        if (TOOL_TILT_DEG > 0 and self._obst and self.spheres
                and float(np.linalg.norm(np.asarray(face)[:2] - f0[:2])) >= TILT_MIN_XY_M):
            mid = self._at(p_tmp, 0.5)
            ymid = yaw_from + (yaw_to - yaw_from) * 0.5
            q_base, ok_base = self._ik(mid, ymid, q_arc0)
            base = self._arm_clearance(q_base) if ok_base else None
            # ★ 이미 충분히 여유로우면 아예 안 기울인다 — 얕은 자리는 그냥 지나간다.
            if base is not None and base < TILT_ENOUGH_M:
                best = None
                for fr in TILT_FRACTIONS:          # 작은 기울기부터 본다
                    td = TOOL_TILT_DEG * fr
                    for dd in TILT_DIRS_DEG:
                        qm, okm = self._ik(mid, ymid, q_arc0, td, dd)
                        if not okm:
                            continue
                        cl = self._arm_clearance(qm)
                        if best is None or cl > best[0]:
                            best = (cl, td, dd)
                    # 이 크기에서 이미 충분하면 더 크게 안 간다
                    if best is not None and best[0] >= TILT_ENOUGH_M:
                        break
                if best is not None and best[0] > base + 0.005:
                    tilt_deg, tilt_dir = best[1], best[2]
                    self._log("[robot_motion] 이송 기울기 %.1f도 방위 %.0f도 "
                              "(중간점 여유 %+.0f -> %+.0f mm)"
                              % (tilt_deg, tilt_dir, base * 1000, best[0] * 1000))
        self._path["tilt"] = tilt_deg
        self._path["tdir"] = tilt_dir
        self._goal_face = np.asarray(face, dtype=float)
        self._goal_yaw = float(yaw_to)              # ★ 도착 판정에 쓸 목표 자세
        self._hold = 0
        self._settle = int(SETTLE_MAX_S * self.hz)
        self._best, self._flat = float("inf"), 0      # 수렴 정체 판정 (위 SETTLE_PLATEAU_*)
        self._s0 = None                              # 정착 진입 시점 (잔차, 틱) — SETTLE_LOG
        self._state = "MOVING"
        self._log("[robot_motion] move -> %s yaw %.1f -> %.1f  %s  |dq| %.1f deg  n %d%s"
                  % (np.round(face, 4), yaw_from, yaw_to,
                     "호" if self._path["arc"] else "직선",
                     float(np.rad2deg(np.max(dq))), steps,
                     ("  + 재구성 n %d" % re_steps) if re_steps else ""))

    def _advance_move(self):
        """호를 한 스텝 전진시킨다. **IK 는 스텝당 최소 1회.**"""
        p = self._path
        if p is None:
            return
        # ★ 재구성 먼저 — 관절공간 직선. 툴은 제자리이므로 호 파라미터는 그대로 유효하다.
        if p["re_q"] is not None and p["re_u"] < 1.0 - 1e-9:
            p["re_u"] = min(1.0, p["re_u"] + p["re_du"])
            q = p["re_q0"] + (p["re_q"] - p["re_q0"]) * p["re_u"]
            self.art.apply_action(self._Action(joint_positions=q))
            if p["re_u"] >= 1.0 - 1e-9:
                p["prev"] = np.asarray(p["re_q"], dtype=float)   # 호는 여기서 체인한다
            return
        if p["u"] >= 1.0 - 1e-9:
            self._settle_move(p)
            return

        # ★ 후보 해를 먼저 풀어 보고 관절 변화가 한계를 넘으면 u 증분을 반으로 줄인다.
        #   손목 특이점(j5≈0) 근처에서는 같은 거리에도 관절 이동량이 몇 배가 된다.
        #   그대로 지령하면 드라이브가 못 따라가 툴이 뒤처져 돈다 (실측 179.5°).
        #   여기서 경로가 알아서 느려진다 — 실제 컨트롤러가 하는 일과 같다.
        step_u = min(p["du"], 1.0 - p["u"])
        qk, ok = None, False
        for _ in range(BISECT):
            u2 = p["u"] + step_u
            # ★ sin(pi*u) — 구간 양끝에서 정확히 0 이라 착지 자세는 늘 수평이다.
            tl = p["tilt"] * math.sin(math.pi * u2) if p["tilt"] else 0.0
            cand, cok = self._ik(self._at(p, u2),
                                 p["yaw0"] + (p["yaw1"] - p["yaw0"]) * u2, p["prev"],
                                 tl, p["tdir"])
            if cok and np.all(np.abs(cand - p["prev"]) <= self.cap):
                qk, ok = cand, True
                break
            if cok and qk is None:
                qk = cand                      # 한계는 넘지만 풀리긴 한 해 (마지막 수단)
            step_u /= 2.0
        if qk is None:
            p["miss"] += 1
            qk = p["prev"]
        elif not ok:
            p["over"] += 1
        p["prev"] = qk
        p["u"] += step_u
        p["k"] += 1
        self.art.apply_action(self._Action(joint_positions=qk))
        # ★ 닿았나 — **지령은 나갔는데 툴이 안 따라오면** 벽에 걸린 것이다.
        if CONTACT_STOP:
            now = self.suction_pose()
            cmd = self._at(p, p["u"])            # 이번 스텝의 지령 흡착면
            if p["cprev"] is not None and p["ccmd"] is not None:
                moved = float(np.linalg.norm(now - p["cprev"]))
                want = float(np.linalg.norm(cmd - p["ccmd"]))
                if want >= CONTACT_MIN_CMD_M:
                    p["cstill"] = (p["cstill"] + 1 if moved < want * CONTACT_RATIO else 0)
            if DIAG_CONTACT and p.get("tol", REACH_TOL_M) <= REACH_TOL_M + 1e-9:
                self._log("[diag] adv u %.3f  지령 %5.2f  실제 %5.2f mm  정체 %d  "
                          "남은 %6.2f mm"
                          % (p["u"],
                             (float(np.linalg.norm(cmd - p["ccmd"])) * 1000
                              if p["ccmd"] is not None else 0.0),
                             (float(np.linalg.norm(now - p["cprev"])) * 1000
                              if p["cprev"] is not None else 0.0),
                             p["cstill"],
                             float(np.linalg.norm(now - self._goal_face)) * 1000))
            p["cprev"], p["ccmd"] = now, cmd
            if p["cstill"] >= CONTACT_TICKS:
                # ★ 지령을 **지금 실제 관절각**으로 되돌린다 — 미는 힘을 즉시 없앤다.
                q_now = np.asarray(self.art.get_joint_positions()[:6], dtype=float)
                self.art.apply_action(self._Action(joint_positions=q_now))
                err = float(np.linalg.norm(now - self._goal_face))
                self._log("[robot_motion] 접촉 감지 — %d틱 못 따라감, 그 자리에서 멈춘다 "
                          "(남은 거리 %.1f mm · u %.2f)" % (CONTACT_TICKS, err * 1000, p["u"]))
                self._reach_done(err, p)
                return
        if p["k"] > int(MOVE_LIMIT_S * self.hz):
            self._state = "IK_FAIL"
            self._path = None
            self._log("[robot_motion] 구간이 %.0f s 를 넘었다 (u %.2f, IK 실패 %d)"
                      % (MOVE_LIMIT_S, p["u"], p["miss"]))

    def _settle_move(self, p):
        """끝점 지령을 유지하며 흡착면이 실제로 목표 **위치와 자세**에 들어오는지 본다."""
        self.art.apply_action(self._Action(joint_positions=p["prev"]))
        # (아래 CONTACT_STOP 분기에서 정체가 확인되면 지령을 실제 관절각으로 되돌린다)
        face, yaw = self.suction_face()
        err = float(np.linalg.norm(face - self._goal_face))
        # ★ yaw 와 yaw±180 은 물리적으로 같은 자세다 — 180° 주기로 비교한다.
        #   안 그러면 등가 각으로 수렴했는데 178° 틀렸다고 오판해 매번 한도까지 기다린다.
        dyaw = abs(yaw - _near(self._goal_yaw, yaw, 180.0))
        if self._s0 is None:
            self._s0 = (err, dyaw, self._settle)     # 호가 끝난 직후 값
        tol = p.get("tol", REACH_TOL_M)
        if err < tol and dyaw < YAW_TOL_DEG:
            self._hold += 1
            if self._hold >= SETTLE_HOLD:
                self._reach_done(err, p)
            return
        self._hold = 0
        if DIAG_CONTACT and tol <= REACH_TOL_M + 1e-9:
            self._log("[diag] set  잔차 %6.2f mm  yaw %5.1f도  정체 %d  남은틱 %d"
                      % (err * 1000, dyaw, self._flat, self._settle))
        # ── 수렴 정체 = 이 제어기의 정상상태다. 더 기다려도 안 들어온다 ──
        if err < self._best - SETTLE_PLATEAU_M:
            self._best, self._flat = err, 0
        else:
            self._flat += 1
        if (self._flat >= SETTLE_PLATEAU_TICKS
                and dyaw < YAW_TOL_DEG and err < REACH_ABORT_M):
            # ★ 잔차가 허용 밖인 채 정체했다면 **막힌 것**이다. `_reach_done` 은 지령을
            #   안 거두므로 드라이브가 마지막 깊은 지령을 계속 붙들고 민다 —
            #   상위가 되잡는 동안 내내 이웃을 밀어 knock 13.3 mm 가 났다 (실측).
            if CONTACT_STOP and err > p.get("tol", REACH_TOL_M):
                self.art.apply_action(self._Action(
                    joint_positions=np.asarray(self.art.get_joint_positions()[:6], dtype=float)))
            self._reach_done(err, p)
            return
        # ★★ 크게 막혔을 때 **더 오래 미는** 구조였다 (2026-08-25 실측).
        #   위 정체 탈출에 `err < REACH_ABORT_M(50mm)` 이 붙어 있어서, 잔차가 55.6 mm 면
        #   탈출이 안 걸리고 SETTLE_MAX_S(2.5초) **전체를 누르며 기다렸다.**
        #   그 2.5초 동안 팔이 이웃을 밀어 knock 17.2 mm 가 났다.
        #   -> 정체는 정체다. 크기와 무관하게 **즉시 힘을 뺀다.**
        #      지령을 지금 실제 관절각으로 되돌리면 미는 힘이 그 자리에서 사라진다.
        # ⚠ **yaw 를 같이 봐야 한다** (2026-08-25 실측). 손목이 346도 도는 이송 구간에서
        #   위치는 1.3 mm 로 먼저 도착하고 yaw 만 72.7도 남아 있었는데, 위치 잔차가
        #   안 줄어든다고 "닿았다"로 오판해 **공중(z +144 mm)에서 박스를 놓았다.**
        #   자세가 아직 수렴 중인 것은 접촉이 아니다.
        if (CONTACT_STOP and self._flat >= SETTLE_PLATEAU_TICKS
                and dyaw < YAW_TOL_DEG):
            q_now = np.asarray(self.art.get_joint_positions()[:6], dtype=float)
            self.art.apply_action(self._Action(joint_positions=q_now))
            self._log("[robot_motion] 접촉 감지 — 정체 %d틱, 힘을 뺀다 "
                      "(잔차 %.1f mm · yaw %.1f도)" % (SETTLE_PLATEAU_TICKS, err * 1000, dyaw))
            if err > REACH_ABORT_M or dyaw > YAW_ABORT_DEG:
                # 상위(cycle)가 이 자리를 읽어 되잡거나 낙하로 처리한다.
                self._state = "IK_FAIL"
                self._path = None
                return
            self._reach_done(err, p)
            return
        self._settle -= 1
        if self._settle <= 0:
            if err > max(LOG_RESIDUAL_M, tol) or dyaw > YAW_TOL_DEG:
                self._warn_residual(p, err, dyaw)
            # ★ 여기서 REACHED 를 주면 호출부가 **그 자리에서 놓는다.**
            #   실패로 알려 cycle 이 이탈 경로(cancel_plan -> 복귀)를 타게 한다.
            #   위치와 자세 **둘 다** 본다 — main 의 yaw 판정과 같은 이유로, 크게 틀어진
            #   채 놓으면 박스가 삐딱하게 앉아 다음 층이 그 위에서 무너진다.
            if err > REACH_ABORT_M or dyaw > YAW_ABORT_DEG:
                self._log("[robot_motion] 잔차 %.0f mm (한도 %.0f) · yaw %.1f도 (한도 %.0f)"
                          " — 실패로 알린다"
                          % (err * 1000, REACH_ABORT_M * 1000, dyaw, YAW_ABORT_DEG))
                self._state = "IK_FAIL"
                self._path = None
                return
            self._reach_done(err, p)

    def _warn_residual(self, p, err, dyaw=0.0):
        # ⚠ 여기서 영원히 기다리면 안 된다 — 정상상태 오차가 남는 제어기에서는
        #   임계값에 영영 못 들어간다 (구 저장소 RMPflow 교훈). 한도를 두고
        #   잔차를 로그로 남긴 뒤 넘어간다.
        self._log("[robot_motion] 도착 잔차 %.2f mm (허용 %.1f) · yaw %.1f deg (허용 %.1f)"
                  " — 한도 초과, 진행\n"
                  "               흡착면 %s -> 목표 %s  (IK 실패 %d · 속도초과 %d)"
                  % (err * 1000, REACH_TOL_M * 1000, dyaw, YAW_TOL_DEG,
                     np.round(self.suction_pose(), 4),
                     np.round(self._goal_face, 4), p["miss"], p["over"]))

    def _reach_done(self, err, p=None):
        if SETTLE_LOG and self._s0 is not None:
            e0, d0, t0 = self._s0
            n = t0 - self._settle
            # 잔차가 크게 줄었으면 **팔이 움직이는 중**이었다는 뜻이고,
            # 거의 그대로면 **이미 멈춰 있었는데 로직이 붙잡은** 것이다.
            self._log("[robot_motion] 정착 %3d틱 %5.0f ms · 잔차 %6.2f -> %5.2f mm "
                      "(줄어든 %5.2f) · yaw %4.1f -> %4.1f도"
                      % (n, 1000.0 * n / self.hz, e0 * 1000, err * 1000,
                         (e0 - err) * 1000, d0, 0.0))
        self._s0 = None
        # ★ REACH_TOL 을 못 맞추고 끝난 구간은 잔차를 남긴다 — 이 제어기의 정상상태가
        #   얼마인지가 여기서만 보인다. 한 줄로 짧게.
        if p is not None and err >= p.get("tol", REACH_TOL_M):
            self._log("[robot_motion] 정착 %.2f mm (허용 %.1f) — 더 안 줄어 종료"
                      % (err * 1000, p.get("tol", REACH_TOL_M) * 1000))
        self._state = "REACHED"
        self._path = None
        self._last_err_m = err
        if p is not None and (p["miss"] or p["over"]):
            self._log("[robot_motion] 구간 종료 — IK 실패 %d · 속도초과 %d 스텝"
                      % (p["miss"], p["over"]))

    def _advance_gripper(self):
        if self._g_evt is not None:
            res = self.grip.poll_grasp()
            if res is not None:
                self._g_res = res
                self._g_evt.set()
                self._g_evt = None
        if self._r_evt is not None:
            res = self.grip.poll_release()
            if res is not None:
                self._r_res = res
                self._r_evt.set()
                self._r_evt = None
