"""15박스 연속 적재 — **실제 씬 `test1-1.usd` 위에서** (트랙 A 통합 시험).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec /home/rokey/palletris_ws/scripts/demo_stack15_test1_1.py

## 시작은 사람이 누른다 (창이 있는 실행)

씬이 뜨고 카메라 창까지 준비되면 멈춰 서서 **Play 를 기다린다.** 이 드라이버는
물리 스텝 콜백으로만 도니까, Play 가 안 눌리면 부팅 카운터부터 안 돈다.
창이 없는 실행(게이트·검증)은 누를 사람이 없어 자동으로 누른다. `PL_AUTOPLAY` 로
어느 쪽이든 강제할 수 있다.

  ⚠ **일시정지와 Stop 은 아직 못 이어간다.** 둘 다 그 실행을 거기서 끝낸다 —
    Stop 은 물리 뷰를 무효화해 로봇·그리퍼 핸들을 죽이고 쌓아 둔 팔레트도 되감고,
    일시정지는 이 드라이버의 타이머가 전부 **벽시계**라(스폰 간격·도착 180 s·
    사이클 120 s·워치독 900 s) 물리만 멈춘 사이에 시계가 계속 가서 타임아웃이 터진다.
    둘 다 고치려면 타이머를 시뮬 시간으로 옮기고(일시정지) 수명주기를 재초기화
    가능하게(Stop) 만들어야 한다 — 별건이다.

## 화면 둘 (시연, demo 모드)

    view2  Isaac 창 안에 두 번째 뷰포트가 자동으로 뜬다 — **벨트 위 하향 카메라**
           (/World/CamPick). 박스가 스토퍼에 서는 것을 위에서 본다.
    rqt    **그리퍼 손목 카메라**(/World/robot/gripper/CamWrist)는 창을 안 만든다.
           ROS 2 이미지 토픽 `/wrist_cam/rgb` 로 나가고, 다른 프로세스가 그린다.

    창 하나당 렌더 틱이 느려지고 벨트·소켓이 그 틱에 물려 있어 사이클이 밀린다
    (실측 +35 %, 아래 CAM_PICK_PRIM 근처 주석). 그래서 창은 하나만 쓴다.

    터미널 1                      터미널 2
    ---------------------------   ------------------------------------------
    ~/palletris_ws/scripts/run_demo.sh
                                  ros2 run rqt_image_view rqt_image_view \
                                      /wrist_cam/rgb

    run_demo.sh 를 안 쓰고 직접 부를 거면 브릿지 .so 경로를 먼저 잡아야 한다
    (`.bashrc` 의 isaac_ros 함수):

        isaac_ros && cd ~/isaacsim && \
            ./isaac-sim.sh --exec ~/palletris_ws/scripts/demo_stack15_test1_1.py

    끄려면 PL_CAMVIEW=0 (view2) · PL_WRISTROS=0 (손목 발행). gate 모드는 둘 다 끔.

`scripts/demo_stack15.py`(B5) 는 받침대만 있는 합성 씬에서 박스를 픽 지점으로
**급송**해 15개를 쌓는다. 이 스크립트는 같은 15개를 **A 가 만든 진짜 라인**으로
보낸다 — 컨베이어를 타고 와서 스토퍼에 서고, 거기서 집힌다. 새로 만드는 것은
글루뿐이고 판단하는 코드는 전부 실물이다:

    packing_robot/cycle.py                상태머신 (§2.5)
    packing_sim/isaac/robot_motion.py     B 명령 6개
    packing_sim/isaac/{sim_server,spawner,conveyor_ctl,mainq}.py   <- A
    demo_stack15.{manifest,PalletVision,OnlineServices}            <- B5 그대로 import
    src/packing_sim/packing.HeightmapPacker                        배치 계획

## B5 데모와 다른 점 셋 — 전부 "진짜 라인이라서"

  1) **박스가 벨트를 타고 온다.** 급송(`set_world_pose`)이 없다. 스폰점 x=500 에서
     스토퍼 x=10000 까지 9.5 m 를 0.5 m/s 로 온다 (약 19 s).
  2) **도착 순서 그대로 집는다.** B5 는 오프라인 계획이라 부피 내림차순으로 급송해
     15/15 를 넣었다. 라인에는 그 자유가 없다 — 그래서 온라인 패커
     (`OnlineServices`, 매 박스 비전 재계산)를 쓴다. 못 넣으면 PALLET_FULL 이
     그대로 결과에 남는다. 그게 이 시험이 답해야 할 질문이다.
  3) **벨트를 진짜로 세운다.** `OnlineServices.set_velocity` 는 기록만 하는 mock
     이다(B5 씬에 컨베이어가 없다). 여기서는 소켓으로 실제 구간을 세운다
     — 안 세운 채 흡착하면 박스가 끌려간다 (§2.5).

## 스폰은 사양 스케줄로 돈다

`boxes.stage1.spawn_interval_s`(12 s) 마다 하나씩, `total_count`(15) 개.
사이클이 그보다 오래 걸리면 픽업 구간(7500~10000, 2500 mm)에 줄이 선다 —
설계결정서 §4.4 가 말한 "백압 버퍼"가 실제로 생기는지 보는 것이 이 시험의 절반이다.
`zones.buffer_capacity` 는 4 다.

## 왜 `isaac-sim.sh` 인가

벨트가 `IsaacConveyor` OmniGraph 노드로 도는데 그 노드는 렌더 틱에 물려 있다.
`python.sh` standalone 은 속도를 틀리게 돌린다. 근거는 `isaac/conveyor_ctl.py`
docstring. 여기서는 이송 시간이 결과의 일부라 `isaac-sim.sh` 를 쓴다.
"""
import math
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from omni.physx import get_physx_interface

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src/packing_sim/isaac"))
sys.path.insert(0, str(ROOT / "src/packing_robot/packing_robot"))

import _kitexit                                          # noqa: E402
import _paths                                            # noqa: E402
import import_h2017_usd as IMP                           # noqa: E402
import demo_stack15 as D                                 # B5 산출물 그대로  # noqa: E402

STAGE = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")
PORT = int(os.environ.get("PL_PORT", "8797"))            # 8765=sim_main · 8798=B5 · 8799=B4
# ★ 로봇 프림을 환경변수로 받는다 (2026-08-24). test1-1 은 로봇이 한 대라
#   "/World/robot" 하나면 됐지만, 셀이 셋인 씬(test_3)은 robot_A/B/C 다.
#   이 드라이버는 여전히 **한 번에 한 대**만 돌린다 — 어느 대를 돌릴지만 고른다.
ROBOT_PRIM = os.environ.get("PL_ROBOT_PRIM", "/World/robot")
# 스토퍼도 같이 받는다 — 셀이 셋인 씬은 Stopper_A/B/C 로 존마다 따로 있다.
# 존재 확인에만 쓴다 (판정 좌표는 conveyor.stopper_x_mm 이 준다).
STOPPER_PRIM = os.environ.get("PL_STOPPER_PRIM", "/World/Stopper")
GRIPPER_PRIM = ROBOT_PRIM + "/SurfaceGripper"
EE_FRAME = "tool0"
# 존도 같이 받는다 — 팔레트 원점이 존별이라 로봇과 짝이 맞아야 한다.
ZONE = os.environ.get("PL_ZONE", "B")
# ★ 로봇 여러 대를 **동시에** 돌린다 (2026-08-25). `PL_ZONES=A,B,C` 로 켠다.
#   안 주면 `PL_ZONE` 하나뿐이라 기존 동작이 하나도 안 바뀐다 (test1-1 기준선).
#   존마다 로봇·그리퍼 prim 이름은 `<PL_ROBOT_FMT>` 로 만든다.
ZONES = [z.strip().upper() for z in
         os.environ.get("PL_ZONES", ZONE).split(",") if z.strip()]
ROBOT_FMT = os.environ.get("PL_ROBOT_FMT", "/World/robot_%s")
STOPPER_FMT = os.environ.get("PL_STOPPER_FMT", "/World/Stopper_%s")
MULTI = len(ZONES) > 1


def robot_prim(z):
    """존 `z` 의 로봇 prim. 단일 존이면 `PL_ROBOT_PRIM` 을 그대로 쓴다."""
    return (ROBOT_FMT % z) if MULTI else ROBOT_PRIM


def stopper_prim(z):
    return (STOPPER_FMT % z) if MULTI else STOPPER_PRIM
BOX_ID_FMT = "BOX%05d"
BOOT_STEPS = 20
SETTLE_STEPS = 90                                        # READY 수렴 (1.5 s)
# ★ 지금 집으려는 박스. 워커 스레드가 쓰고 **메인 스레드**(gap_fn)가 읽는다.
#   튜플을 통째로 갈아끼운다 — 키 두 개를 따로 쓰면 새 박스 id 에 옛 높이가
#   붙는 순간이 생긴다.
_CUR = {"box": None}                                     # (box_id, 반높이 m) | None
# 다존일 때는 존마다 따로 잡는다 — gap_fn 이 자기 존 것만 봐야 한다.
_CUR_Z = {}
# ── 운전 모드 ──────────────────────────────────────────────
# ★ 이 파일 하나로 두 가지를 돌린다. 예전에는 시연 설정을 환경변수 넷으로 손수
#   붙여야 했는데, 발표 중에 하나만 빠뜨려도 다른 게 돌아간다 — PL_N 을 빼면
#   15개가 돌아 팔레트가 중간에 갈리고, PL_SPEED 를 빼면 첫 5호가 미끄러질
#   확률이 생긴다. 그래서 모드로 묶었다.
#
#   demo (기본) — 사람이 보는 실행. 창을 띄우고 11개를 팔레트 한 판에 쌓고,
#                 끝나도 창을 안 닫는다. 팔 속도는 0.7 (사양 0.5 가 아니다).
#   gate        — 수치를 재는 실행. **전부 사양값**이라 베이스라인이 된다.
#
#   개별 PL_* 를 주면 모드 기본값을 덮어쓴다. 둘 중 뭘로 돌았는지는 시작할 때
#   한 줄로 찍는다 — 나중에 로그만 보고도 알 수 있어야 한다.
MODE = os.environ.get("PL_MODE", "demo").lower()
if MODE not in ("demo", "gate"):
    raise SystemExit("PL_MODE 는 demo | gate: %r" % MODE)
_DEFAULTS = {
    # 이름        demo    gate            뜻
    "PL_N":     ("11",   "0"),        # 0 = 사양값(15)
    "PL_SPEED": ("0.7",  ""),         # "" = 사양값(joint_speed_scale)
    "PL_HOLD":  ("1",    "0"),        # 끝나고 창을 열어 두나
}


def _env(name):
    """PL_* 를 읽되, 없으면 **모드 기본값**으로 떨어진다."""
    v = os.environ.get(name)
    return _DEFAULTS[name][0 if MODE == "demo" else 1] if v is None else v


N_BOXES = int(_env("PL_N") or "0") or None                # 0/미지정이면 사양값
# ★ 상자 조합을 이 실행에서만 바꾼다. `system.yaml` 의 random_seed 42 는
#   "고정 — 베이스라인 비교 필수" 라 파일을 고치지 않는다.
#   seed 42 는 5호가 4개 뽑힌 빡센 패라(사양 분포로는 3개) 도착 순서로는 14/15 가 상한이다.
#   시연용으로 전부 들어가는 조합을 쓰려면 PL_SEED 를 준다 — 어떤 seed 가 되는지는
#   `python3 scripts/eval_pack_online.py --scan-seeds 60` 이 답한다 (60개 중 6개).
#   ⚠ PL_SEED 를 쓴 실행은 **베이스라인이 아니다.** 결과에 그렇게 찍는다.
SEED = os.environ.get("PL_SEED")
# ★ 투입 순서. stage 1 은 **스포너가 곧 투입 순서**라 여기서 정할 수 있다.
#   arrival = 뽑힌 그대로 (기본). 무엇이 올지 못 고르는 실제 라인의 조건이다.
#   volume  = 부피 내림차순. **B5 데모(`demo_stack15.py`)와 같은 조건**이다 —
#             그쪽 docstring: "15개를 도착 순서 그대로 넣으면 패커가 14/15 밖에
#             못 넣는다. 부피 내림차순으로 정렬하면 15/15 가 들어간다."
#   seed 42 로 실측(`scripts/eval_pack_online.py --sweep`): arrival 은 clearance ·
#   halo 를 어떻게 줘도 14/15 가 상한이고, volume 은 어떤 설정에서도 15/15 다.
#   즉 이 차이는 튜닝이 아니라 **순서**다.
ORDER = os.environ.get("PL_ORDER", "arrival")
if ORDER not in ("arrival", "volume"):
    raise SystemExit("PL_ORDER 는 arrival | volume: %r" % ORDER)
ARRIVE_TIMEOUT_S = 180.0                                 # 다음 박스가 스토퍼에 설 때까지
CYCLE_WALL_S = 120.0
SETTLE_S = float(os.environ.get("PL_SETTLE", "0.7"))     # 놓은 뒤 재기 전 정착 (B5 는 0.5)
WATCHDOG_S = float(os.environ.get("PL_WATCHDOG", "900"))
# 픽업 구간에 박스가 있는데 이 시간이 지나도 못 집으면 지운다 (아래 `_stuck_box`).
STUCK_S = float(os.environ.get("PL_STUCK", "75"))
# ★ 팔레트 1 장에 이만큼 놓으면 **트럭으로 내보내고 끝낸다.** 0 이면 안 내보낸다.
#   패킹 알고리즘을 아직 안 받았으므로, 지금은 1 개만 놓고 반출까지 보는 것이 목표다.
PER_PALLET = int(os.environ.get("PL_PER_PALLET", "0"))
SHIP_TIMEOUT_S = float(os.environ.get("PL_SHIP_TIMEOUT", "180"))
# 반출 중 적재물이 팔레트 대비 얼마까지 밀려도 "안착" 으로 볼 것인가.
# 키네마틱으로 얼려 같은 틱에 같은 변위를 주므로 원리상 0 이어야 한다. 5 mm 는
# bbox 반올림과 렌더 틱 사이 오차를 위한 여유다.
SHIP_SLIP_TOL_MM = float(os.environ.get("PL_SHIP_SLIP_TOL", "5"))
# 코너 턴테이블이 90° 를 도는 데 걸리는 시간 [s]. 보여 주기용이라 물리와 무관하다.
# ★ 2.0 -> 4.0 (2026-08-25 사용자 요청 "너무 빠르다"). 22.5 deg/s 라 눈으로 따라간다.
TURN_S = float(os.environ.get("PL_TURN_S", "4.0"))
# ★ **-90 이다.** 셀은 +Y 로 오고 합류는 +X 로 나가므로 커브는 **오른쪽으로** 돈다
#   (시계방향). +90 으로 두면 발자국은 같아도 팔레트·적재물이 180° 뒤집힌 채 실린다.
TURN_DEG = -90.0
# ★ 반출 속도는 라인 속도와 **따로** 둔다. 라인 값은 5호가 스토퍼에 부딪혀 기우는
#   문제로 묶여 있고, 반출 구간에는 스토퍼가 없기 때문이다.
#   ★ 1.5 -> 0.5 (2026-08-25 사용자 요청 "너무 빠르다"). 실물 팔레트 컨베이어가
#     보통 0.2~0.5 m/s 다. 합류 12.6 m 를 25 s 에 간다 (SHIP_TIMEOUT 180 안).
SHIP_MPS = float(os.environ.get("PL_SHIP_MPS", "0.5"))
VERBOSE = os.environ.get("PL_VERBOSE") not in (None, "", "0")   # 서비스 호출까지 전부
# ★ 시연용. 결과를 찍은 뒤 **창을 닫지 않는다** — 쌓인 팔레트를 보여줘야 하는데
#   헤드리스 게이트와 같이 즉시 종료해 버리면 볼 것이 없다. GUI 로 띄울 때만 쓴다.
def _has_window():
    """이 실행에 창이 있나. `--no-window` 면 False."""
    try:
        import carb.settings
        return bool(carb.settings.get_settings().get("/app/window/enabled"))
    except Exception:
        return False


# ★ 창이 없으면 HOLD 는 의미가 없다. 그런데 예전 판은 그래도 붙잡아서, 헤드리스로
#   돌린 검증 실행이 **결과를 다 찍고도 안 죽었다.** 그게 셋씩 쌓여 GPU 를 나눠
#   쓰는 바람에 바로 그 다음 측정이 통째로 틀어졌다 (실측: 사이클 12.3 -> 26.0 s).
#   재현 안 되는 측정을 만드는 종류라 여기서 막는다.
HOLD = _env("PL_HOLD") not in ("", "0", "false", "no") and _has_window()
# ★ 자동 Play. **창이 있으면 안 누른다** — 시연에서는 사람이 직접 누르고 싶다.
#   이 드라이버는 물리 스텝 콜백(`on_step`)으로만 돌아서, Play 가 안 눌리면
#   부팅 카운터부터가 안 돈다. 즉 "안 누르면 아무 일도 안 일어난다" 가 그대로 성립한다.
#   창이 없는 실행(게이트·검증·CI)은 누를 사람이 없으니 자동으로 누른다.
#   PL_AUTOPLAY=1 / 0 으로 어느 쪽이든 강제할 수 있다.
_ap = os.environ.get("PL_AUTOPLAY")
AUTOPLAY = (not _has_window()) if _ap is None else (_ap not in ("0", "false", "no"))
# ★ 창도 없는데 자동 Play 를 끄면 **누를 사람이 없어 영원히 기다린다.**
#   실측 2026-08-24: 그렇게 띄운 헤드리스 하나가 15분째 살아서 GUI 시연과 GPU 를
#   나눠 쓰고 있었다 — 어제 knock 이 18 / 67 mm 로 갈렸던 바로 그 원인이다
#   (`_kitexit` docstring). 조용히 매달려 있느니 여기서 끊는다.
if not AUTOPLAY and not _has_window():
    print("\n!! 창이 없는 실행(--no-window)인데 PL_AUTOPLAY=0 이다.", flush=True)
    print("   Play 를 눌러 줄 사람이 없어서 영원히 기다린다. 그대로 두면 GPU 만 먹는다.",
          flush=True)
    print("   창 없이 돌릴 거면 PL_AUTOPLAY 를 빼라 (창 없는 실행은 원래 자동이다).",
          flush=True)
    _kitexit.shutdown(3, log=print)
    raise SystemExit
PICK_RETRY = int(os.environ.get("PL_PICK_RETRY", "2"))   # 흡착 실패 재시도 횟수
# ★ 팔 속도. **이 실행에서만** 덮어쓴다 — system.yaml 도 B 코드도 안 건드린다.
#   왜 필요한가: 이송 구간이 177도를 도는데 박스는 흡착 패드에 강성 100 · 감쇠 0
#   으로 매달려 있어 진자처럼 흔들린다. 감쇠가 0이라 하강하는 2.3 s 동안 안 잦아들고,
#   놓는 순간이 그 진자의 어느 위상에 걸리느냐로 착지가 갈린다
#   (`gripper.py` 주석의 "22~76 mm", 실측 2026-08-23 에는 -99.8 mm).
#   느리게 돌면 진자에 실리는 에너지가 줄어든다. 사이클은 길어진다.
#   안 주면 사양값 그대로다.
SPEED = _env("PL_SPEED") or None
# ★ 계량 운전(singulation). 픽업 구간에 이 수보다 많이 못 들어오게 `main` 을 세운다.
#   왜 필요한가 — 지금 라인은 **누적식**이다. 스토퍼가 선두를 잡고 벨트는 계속
#   밀어대니 뒤가 전부 붙는다. 물리적으로 맞는 동작이라 속도·간격 튜닝으로는
#   안 없어진다. 실물에서는 존마다 따로 세워 한 개씩 내려보내는 식으로 푼다
#   (zero-pressure / singulation). 우리는 구간이 둘이고 `cycle.py` 는 `b_pickup`
#   만 만지므로 `main` 이 비어 있다 — 거기를 쓴다.
#   **벨트를 세우면 박스는 그 자리에 선다.** 벽에 눌리는 게 아니라서 간격
#   (12 s x 0.5 m/s = 6 m)이 그대로 남는다.
#   0 이면 계량을 안 한다(예전 동작).
METER_MAX = int(os.environ.get("PL_METER", "1"))
# ★ 픽업 하향 카메라. **기본은 끔** — 씬에는 들어가 있지만(/World/CamPick,
#   씬 빌더가 만든다) 좌표 인식이 아직 안 끝났다. `PL_CAM=1` 로 켜면 박스마다
#   카메라가 본 것을 ground truth 와 나란히 찍는다. 판단에는 안 쓴다.
#   막힌 곳: `Camera.get_pointcloud(world_frame=True)` 가 Isaac 내부에서
#   NameError("name 'LOADER_DIR' is not defined") 를 낸다 (우리 코드에는 그 이름이
#   없다 — 포인트클라우드 annotator 경로 문제다). 우회로는 이미 있다:
#   `perception.CameraPerception._pointcloud_world` 가 depth + intrinsics 로
#   직접 투영한다. 그쪽으로 갈아끼우면 된다.
#   ★ 2026-08-24 기본을 **demo 켬 / gate 끔** 으로 바꿨다. 위 막힌 곳(포인트클라우드
#     API)은 원인이 Isaac 이 아니라 모듈 이름 충돌이었고 (`_cfgload` docstring),
#     고친 뒤 5/5 인식 · 종류 5/5 · 중심 오차 최대 6.5 mm 로 돈다. 시연에서 제일
#     설명하기 좋은 자리라 켜 둔다. 게이트는 끈다 — 판단에 안 쓰는 계측이 사이클에
#     끼면 베이스라인이 흔들린다.
_DEFAULTS["PL_CAM"] = ("1", "0")
CAM = _env("PL_CAM") not in ("0", "false", "no")
# ★ 화면 둘을 **다른 방식으로** 낸다 (2026-08-24).
#     view2 (Isaac 두 번째 뷰포트)  = 벨트 위 하향 카메라  /World/CamPick
#     rqt   (별도 프로세스)          = 그리퍼 손목 카메라  /World/robot/gripper/CamWrist
#
#   왜 손목만 ROS 로 빼는가. **뷰포트는 공짜가 아니다.** 하나 더 붙으면 렌더 틱이
#   느려지는데 벨트 IsaacConveyor 노드와 소켓 주기가 그 틱에 물려 있어 사이클 전체가
#   밀린다. 실측 2026-08-23 (11박스, 창 하나 추가):
#       없음    사이클 12.3 s · 벽시계 203 s · knock  0.7 mm
#       창 하나 사이클 16.6 s · 벽시계 268 s · knock 15.7 mm   (+35 %)
#   knock 경고선이 20 이라 창 둘은 못 띄운다. 그래서 하나(벨트)만 창으로 두고,
#   손목은 렌더 프로덕트 하나를 ROS 2 이미지 토픽에 실어 **다른 프로세스**(rqt)가
#   그리게 한다 — 그리는 비용이 Isaac 밖으로 나간다. 그래도 렌더 프로덕트 자체는
#   공짜가 아니라 해상도를 낮추고 frameSkipCount 로 발행 주기를 떨군다.
#   게이트(gate 모드)에서는 둘 다 끈다 — 수치를 재는 실행에 보여주기용 부하를
#   얹으면 비교가 안 된다.
# 창(view2)에 띄울 카메라. **실제 값은 설정을 읽은 뒤 아래에서 정해진다** —
# `cfg` 가 이 줄보다 아래에서 만들어지기 때문이다.
CAM_PICK_PRIM = "/World/CamPick/Camera"
CAM_WRIST_PRIM = ROBOT_PRIM + "/gripper/CamWrist/Camera"   # 손목 (ROS 토픽)
CAMVIEW_RES = (480, 270)          # 창 렌더 해상도. 창 크기와 무관하게 비용을 정한다.
_DEFAULTS["PL_CAMVIEW"] = ("1", "0")          # demo 만 켠다
CAMVIEW = _env("PL_CAMVIEW") not in ("0", "false", "no")
# ★ 손목 카메라를 ROS 2 이미지로 발행한다.  rqt_image_view 로 본다.
#     /wrist_cam/rgb   (sensor_msgs/Image)      <- 이걸 rqt 에 물린다
#     /wrist_cam/camera_info
#   rclpy 는 안 쓴다 — Isaac 은 3.11, Jazzy 의 rclpy 는 3.12 라 import 하면 죽는다
#   (docs/가이드/공통가이드_전원필독.md §3). 브릿지가 C++ 로 주는 OmniGraph 노드만 쓴다.
#   ⚠ 실행 **전에** 브릿지 .so 경로를 잡아야 한다. `.bashrc` 의 `isaac_ros` 함수다:
#       export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
#           $HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib
#     프로세스가 뜬 뒤에 os.environ 을 고쳐도 소용없다 (glibc 가 시작 때 읽는다).
WRIST_ROS_RES = (424, 240)
WRIST_ROS_NS = os.environ.get("PL_WRIST_TOPIC", "wrist_cam")
WRIST_ROS_SKIP = int(os.environ.get("PL_WRIST_SKIP", "3"))   # 3 = 네 틱에 한 장
WRIST_GRAPH = "/World/ROS2WristCam"
_DEFAULTS["PL_WRISTROS"] = ("1", "0")         # demo 만 켠다
WRISTROS = _env("PL_WRISTROS") not in ("0", "false", "no")

cfg = _paths.system_config()


# ★ 좌표 오버레이 (2026-08-24). system.yaml 은 "모든 숫자의 단일 출처"(철칙 3)이고
#   팀 4명이 공유한다. 그런데 라인 배치가 근본적으로 다른 씬(test_3 은 지선이 +Y)을
#   돌리려면 좌표가 통째로 달라야 한다. 원본을 고치면 test1-1 기준 게이트·베이스라인이
#   전부 무효가 되므로, **실행 시에만 얹는 별도 yaml** 로 푼다.
#     PL_CONFIG_OVERLAY=src/packing_bringup/config/test_3.yaml
#   ⚠ 오버레이를 쓴 실행은 **베이스라인이 아니다.** 시작 배너에 그렇게 찍는다.
def _deep_merge(base, over):
    """over 의 값으로 base 를 덮는다. dict 는 재귀, 그 외는 통째로 교체."""
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base


OVERLAY = os.environ.get("PL_CONFIG_OVERLAY")
if OVERLAY:
    import yaml
    _op = Path(OVERLAY)
    if not _op.is_absolute():
        _op = ROOT / _op
    if not _op.exists():
        raise SystemExit("PL_CONFIG_OVERLAY 파일이 없다: %s" % _op)
    _ov = yaml.safe_load(_op.read_text()) or {}
    _deep_merge(cfg, _ov)

# ★ 존별 카메라 (test_3 는 CamPick_A/B/C). 오버레이를 얹은 **뒤에** 정한다.
CAM_PICK_PRIM = ((cfg.get("cameras") or {}).get("pick_prim") or {}).get(
    ZONE, CAM_PICK_PRIM)
STOPPER_X = cfg["conveyor"]["stopper_x_mm"] / 1000.0
BELT_MPS = float(cfg["conveyor"]["belt_velocity_mps"])
ORIGIN_MM = cfg["zones"]["pallet_origin_world_mm"][ZONE]
# ★ 존별 팔레트 원점. 로봇 3 대를 동시에 돌릴 때 월드 좌표를 만드는 자리에서 쓴다.
#   ⚠ 로봇 기준 **상대** 배치는 세 존이 똑같다 (원점−베이스 = (−1200, −600) 고정).
#     그래서 `_az_local`·`_near_worst_mm` 같은 팔 기하 계산은 존을 안 가려도 된다.
#     존을 가려야 하는 건 **월드 좌표를 만드는 곳**뿐이다.
ORIGIN_BY_ZONE = dict(cfg["zones"]["pallet_origin_world_mm"])
S1 = cfg["boxes"]["stage1"]
BUFFER_CAP = int(cfg["zones"]["buffer_capacity"])        # 픽업 구간에 세울 수 있는 최대
N = N_BOXES or int(S1["total_count"])
SPAWN_DT = float(S1["spawn_interval_s"])
BASE_MM = cfg["zones"]["robot_base_world_mm"][ZONE]
PALLET_X = (ORIGIN_MM[0] / 1000.0, (ORIGIN_MM[0] + cfg["pallet"]["width_mm"]) / 1000.0)
PALLET_Y = (ORIGIN_MM[1] / 1000.0, (ORIGIN_MM[1] + cfg["pallet"]["depth_mm"]) / 1000.0)
PALLET_TOP_M = ORIGIN_MM[2] / 1000.0
BOX_H_MM = {t["name"]: float(t["h_mm"]) for t in cfg["boxes"]["types"]}

# ── 팔 간섭 제약 ────────────────────────────────────────────
# ★ knock 의 원인은 물리 튜닝이 아니라 **자리를 어디로 잡느냐**다.
#   B5 견본이 knock 을 115 mm -> 0.2 mm 로 떨어뜨린 것은 패커가 아니라
#   `build_plan` 안의 순서 규칙이었다: "로봇 쪽에 이미 기둥이 서 있으면 그 너머
#   먼 자리로 갈 때 아래팔(link_2)이 그 위를 쓸고 지나간다" (demo_stack15.print_plan
#   과 results/issues/18). 한계는 대략 페데스탈 상면 − 150 mm.
#   실측 2026-08-23 (results/stack15_swap_20260823.log): 12번째가 근거리
#   P(695,205) z680 5호 -> 상단 1020 mm, 13번째가 그 너머 far P(590,620) z840.
#   그 사이클에서 knock 67.3 mm 가 났고 피해자가 바로 그 근거리 기둥(BOX00011)이다.
#   한계 900−150=750 을 270 mm 넘겼다.
#   라인은 **도착 순서**를 못 고른다. 그러나 **자리**는 고를 수 있다 — 그래서
#   견본의 순서 규칙을 패커의 후보 거부 술어로 옮겨 담는다 (packing.find_placement
#   의 `reject` 훅). 제약을 만족하는 자리가 없으면 예전처럼 잡되 로그에 남긴다:
#   자리를 못 잡아 PALLET_FULL 을 내는 것보다 knock 을 감수하는 편이 낫다.
PED_MM = float(cfg["robot"]["pedestal_height_mm"])
# 이보다 높은 근거리 기둥 위는 못 지나간다. **팔레트 로컬 높이** 다.
# ★ 진짜 규칙은 "기둥 윗면(월드) <= 페데스탈 상면" 이다. 그래서 팔레트 상면을 뺀다.
#   test1-1 은 팔레트 상면이 150 이라 900-150 = 750 — **예전 식과 값이 같다**(회귀 없음).
#   test_3 는 팔레트가 셀 컨베이어 위(950)라 1250-950 = 300 이다. 예전 식(PED-150)이면
#   1100 이 나오는데, 그러면 기둥 윗면이 월드 2050 으로 팔 베이스보다 800 이나 높다.
#   실측 2026-08-25: 그 상태에서 560 mm 기둥 위를 팔이 쓸어 BOX00007 을 팔레트 밖으로
#   떨어뜨렸다 (knock 1452 mm).
NEAR_LIM_MM = PED_MM - float(ORIGIN_MM[2])
NEAR_DY_MM = 50.0                     # "로봇 쪽" 으로 칠 최소 y 차 (견본과 동일)
NEAR_HALF_MM = cfg["pallet"]["depth_mm"] / 2.0   # y < 400 이면 근거리 (견본과 동일)
# ★ 견본 `build_plan` 의 제약은 **둘**인데 처음에는 (a) 만 이식했다.
#   (a) 근거리 기둥 위를 아래팔(link_2)이 쓸고 지나간다     -> _sweep_risk / _builds_trap
#   (b) far 기둥이 높은데 그 앞 근거리에 **깊게 내려앉으면** 팔꿈치(link_3/4)가
#       far 기둥을 민다                                     -> _far_pillar_risk (여기)
#   (b) 는 잠복이 아니라 **활성**이었다. results/stack15_pass2_20260823.log:820
#   `knock 11.0 mm (BOX00007)` — 9번째가 근거리 P(185,140) z280 으로 내려갈 때
#   피해자가 far 기둥 꼭대기(P(590,620) 상단 840)다. demo_11box_meter 로그도 8.0 mm 로
#   같은 자리에서 재현된다. 그런데 그때 요약은 "근거리 기둥 위반 0 사이클" 이었다 —
#   진단이 **먼 쪽을 아예 안 봤기 때문**이다. 진단부터 고친다.
#   부채꼴 폭: 견본은 ±25° 인데 위 실측 사례는 133.3° vs 90.6° = 42.7° 차라 그걸로는
#   못 잡는다. 실측에 맞춰 넓힌다. 좁히면 놓치고 넓히면 자리를 잃으니 이 값은
#   실행으로 다시 재야 하는 종류다.
FAR_SECTOR_DEG = 50.0
SECTOR_DEG = 25.0                     # 방위각이 이 안이면 같은 통로다 (견본과 동일)
# ★★ 2026-08-26 기본값 "1" -> **"0" (해제)**. 2026-08-25 채택 결정이다.
#   기둥 억제를 푸는 것이 적재율의 가장 큰 레버였다
#   (docs/2026-08-25_협곡_낙하_채택.md):
#       기둥 억제 + 낙하 480   64.5%  ·  평균 24.3개
#       기둥 억제 **해제**     70.9%  ·  평균 26.7개
#       + 낙하 완화 200        75.5%  ·  평균 28.3개
#   대신 협곡 입구에서 **놓아 떨어뜨린다** — 팔이 안 내려가므로 걸릴 일이 없다
#   (PL_DROP_KG · PL_CONTACT_STOP · heightmap.drop_relief_mm).
#   ⚠ 억제가 막던 실패는 실재한다 (2026-08-23 knock 1401 mm). 되돌리려면
#     PL_ARM_GUARD=1. 비교 실험은 soft.
_AG = os.environ.get("PL_ARM_GUARD", "0")
ARM_GUARD = _AG not in ("0", "false", "no")
ARM_GUARD_SOFT = _AG == "soft"           # 함정을 만드는 자리도 허용 (비교 실험용)


def _az_local(cx_m, cy_m):
    """팔레트 로컬 [m] -> 로봇 베이스에서 본 방위각 [deg]. 견본 `_az` 와 같은 식."""
    return math.degrees(math.atan2(cy_m * 1000.0 + ORIGIN_MM[1] - BASE_MM[1],
                                   cx_m * 1000.0 + ORIGIN_MM[0] - BASE_MM[0]))


def _far_worst_mm(cx_m, cy_m, cols):
    """이 자리에 **내려앉을 때** 팔꿈치가 스칠 수 있는 far 기둥 최고 상단 [mm].

    `_near_worst_mm` 의 거울상이다 — 그쪽은 "가는 길"에 넘을 기둥(로봇 쪽)을 보고,
    이쪽은 "내려앉는 동안" 팔꿈치가 뒤로 뻗으며 미는 기둥(로봇 반대쪽)을 본다.
    """
    az = _az_local(cx_m, cy_m)
    worst = 0.0
    for qx, qy, top in cols:
        if qy * 1000.0 <= cy_m * 1000.0 + NEAR_DY_MM:
            continue                                     # 더 먼 쪽이 아니다
        if abs((_az_local(qx, qy) - az + 180.0) % 360.0 - 180.0) >= FAR_SECTOR_DEG:
            continue
        worst = max(worst, top)
    return worst


def _near_worst_mm(cx_m, cy_m, cols):
    """이 자리로 가려면 넘어야 하는 **근거리 기둥 최고 상단** [mm]. 0 이면 없다.

    `cols` 는 (cx_m, cy_m, top_mm) 목록. 진단과 제약이 같은 식을 쓰게 하나로 둔다.
    """
    az = _az_local(cx_m, cy_m)
    worst = 0.0
    for qx, qy, top in cols:
        if qy * 1000.0 >= cy_m * 1000.0 - NEAR_DY_MM:
            continue                                     # 로봇 쪽이 아니다
        if abs((_az_local(qx, qy) - az + 180.0) % 360.0 - 180.0) >= SECTOR_DEG:
            continue                                     # 다른 통로다
        worst = max(worst, top)
    return worst

_S = {"phase": "boot", "n": 0, "t0": time.time(), "done": False,
      "rows": [], "result": None}


def log(*a):
    print(*a, flush=True)


# ── 로그 파일 ───────────────────────────────────────────────
# 헤드리스로 5분 넘게 돌고 cycle/gripper/robot_motion 이 수백 줄을 쏟는다. 터미널
# 스크롤백은 다음 실행이 밀어낸다 — 실제로 knock 을 사후 분석하려다 로그가 없어
# 다시 돌린 적이 있다. B5(`demo_stack15.py`)와 같이 Tee 로 파일에 남긴다.
# `PL_LOG` 로 경로를 바꾼다.
# ⚠ Tee 는 sys.stdout 을 감싸므로 **파이썬 print 만** 잡는다. carb/Kit 의 C++ 로그까지
#   필요하면 셸에서 `2>&1 | tee` 를 따로 걸어야 한다.
_S["tee"] = D.V.start_log(os.environ.get("PL_LOG")
                          or ROOT / "results" / "demo_stack15_test1_1.txt")


# ── 씬 열기 + Play ──────────────────────────────────────────
omni.usd.get_context().open_stage(STAGE)
stage = omni.usd.get_context().get_stage()
log("\n" + "=" * 78)
log("15박스 연속 적재 — 실제 라인 (스폰 %.0f s 간격 · %d 개 · 존 %s)" % (SPAWN_DT, N, ZONE))
log("=" * 78)
log("모드      %s%s"
    % (MODE, "  (창을 띄우고 보여주는 실행 — 사양값이 아니다)" if MODE == "demo"
       else "  (전부 사양값 — 베이스라인)"))
log("설정      박스 %d · 순서 %s · 팔속도 %s · 계량 %s · 인식 %s"
    " · view2 %s · 손목ROS %s · 창유지 %s"
    % (N, ORDER, SPEED or "사양", METER_MAX or "끔", "켬" if CAM else "끔",
       "켬" if CAMVIEW else "끔", "켬" if WRISTROS else "끔",
       "켬" if HOLD else "끔"))
log("씬        %s" % STAGE)
log("팔레트    원점 %s mm   최대 적재 %s mm" % (ORIGIN_MM, cfg["pallet"]["max_stack_mm"]))
log("라인      스폰 %s -> 스토퍼 %.3f m   벨트 %.2f m/s"
    % (cfg["conveyor"]["spawn_point_mm"], STOPPER_X, BELT_MPS))
_chk_prims = []
for _z in ZONES:
    _chk_prims += [robot_prim(_z), robot_prim(_z) + "/SurfaceGripper",
                   stopper_prim(_z)]
for p in _chk_prims:
    ok = stage.GetPrimAtPath(p).IsValid()
    log("  %-28s %s" % (p, "있음" if ok else "!! 없음"))
    if not ok:
        log("  -> src/packing_sim/run_layout_to_spec.sh 를 먼저 돌려라.")
        if _S.get("tee"):
            _S["tee"].close()
        _kitexit.shutdown(2, log=log)
        raise SystemExit
log("-" * 78)

# ── view2 = 벨트 위 하향 카메라 창 ─────────────────────────
if CAMVIEW and stage.GetPrimAtPath(CAM_PICK_PRIM).IsValid():
    try:
        from omni.kit.viewport.utility import create_viewport_window
        # ★ 제목은 ASCII 만 쓴다. Kit 의 창 제목 폰트에 한글 글리프가 없어서
        #   "벨트 카메라 (RSD455 하향)" 이 `?? ??? (RSD455 ??)` 로 찍힌다
        #   (실측 2026-08-24, 시연 화면 녹화). 로그는 한글이어도 된다 — 터미널이다.
        _vw = create_viewport_window("view2 - Belt Cam (RSD455 top-down)",
                                     width=560, height=340)
        _vw.viewport_api.set_active_camera(CAM_PICK_PRIM)
        _vw.viewport_api.resolution = CAMVIEW_RES   # 렌더 비용을 창 크기에서 떼어낸다
        _S["camview"] = _vw
        log("뷰포트    view2 = %s  %dx%d" % (CAM_PICK_PRIM, *CAMVIEW_RES))
    except Exception as _e:
        # 헤드리스(--no-window)면 여기로 온다. 실패가 정상이므로 조용히 넘어간다.
        log("뷰포트    두 번째 창 없음 (%s) — 창 없는 실행이면 정상"
            % type(_e).__name__)
elif CAMVIEW:
    log("뷰포트    %s 가 씬에 없다 — run_layout_to_spec.sh 를 다시 돌려라"
        % CAM_PICK_PRIM)


# ── 손목 카메라 -> ROS 2 이미지 (rqt_image_view 로 본다) ────
def _wrist_cam_to_ros():
    """손목 카메라를 `/<ns>/rgb` 로 발행하는 액션 그래프를 이 실행에만 만든다.

    ★ 그래프는 **저장하지 않는다.** 이 드라이버는 `save_stage()` 를 한 번도 안
      부르므로 메모리 스테이지에만 남고 `test1-1.usd` 는 안 변한다. 씬 파일에
      박아 두지 않는 이유: 게이트 실행(gate 모드)은 이 부하가 없어야 하고,
      브릿지가 안 켜진 환경에서 씬을 열면 그래프가 매 틱 에러를 낸다.

    ★ 왜 나중에 만드는가. 이유가 둘이다.
      (1) `enable_extension` 직후에는 브릿지가 OmniGraph 노드 타입을 아직 등록
          중일 수 있다. 여기는 `--exec` 라 `simulation_app.update()` 손잡이가
          없어서 (`_kitexit` 과 같은 이유) 업데이트 이벤트를 받아 처리한다.
      (2) **부팅 프레임에서 만들면 안 된다.** 뷰포트(view2)를 막 띄운 그 프레임에
          렌더 프로덕트를 하나 더 붙이면 GPU 커맨드 리스트가 엉켜
              [Error] [gpu.foundation.plugin] make sure to wait for cmdlist
              completion ... actualSemaphore: 2 expectedSemaphore: 3
          가 뜬다. 실측 2026-08-24 A/B — 손목ROS 를 끈 실행에는 이 줄이 없고,
          켠 실행에는 렌더 프로덕트를 만드는 시각(15,135 ms)에 정확히 뜬다.
          그래서 **Play 를 누르고 렌더가 안정된 뒤**에 만든다. 손해는 없다 —
          `OnPlaybackTick` 은 어차피 Play 전에는 안 돈다.
    """
    from isaacsim.core.utils.extensions import enable_extension

    _lib = os.path.expanduser(
        "~/isaacsim/exts/isaacsim.ros2.bridge/%s/lib"
        % os.environ.get("ROS_DISTRO", "jazzy"))
    if _lib not in os.environ.get("LD_LIBRARY_PATH", ""):
        log("손목ROS   ⚠ LD_LIBRARY_PATH 에 브릿지 경로가 없다. 그래도 시도한다.")
        log("          안 되면 실행 전에:  isaac_ros      (.bashrc 함수)")

    enable_extension("isaacsim.core.nodes")
    if not enable_extension("isaacsim.ros2.bridge"):
        log("손목ROS   isaacsim.ros2.bridge 를 못 켰다 — 발행 안 한다.")
        log("          isaac_ros 를 부른 터미널에서 다시 실행해라.")
        return

    _st = {"sub": None, "n": 0}

    def _build(_e):
        # Play 전에는 세지도 않는다. Play 를 누른 뒤 렌더가 자리를 잡고 나서 만든다.
        if not omni.timeline.get_timeline_interface().is_playing():
            return
        _st["n"] += 1
        if _st["n"] < 30:                    # 노드 등록 + 첫 렌더 프레임이 지나갈 틈
            return
        try:
            _st["sub"].unsubscribe()
        except Exception:
            pass
        try:
            import omni.graph.core as og
            import usdrt.Sdf
            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": WRIST_GRAPH, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("Tick", "omni.graph.action.OnPlaybackTick"),
                        ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                        ("RP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        # camera_info 는 CameraHelper 가 안 쏜다. 별도 노드다.
                        # rqt 에는 필요 없지만 좌표 인식(camera_pick.py)이
                        # intrinsics 를 여기서 가져가게 같이 낸다. 렌더 프로덕트를
                        # 공유하므로 추가 렌더 비용은 없다.
                        ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ],
                    keys.SET_VALUES: [
                        # 도메인은 씬에 박지 않고 ROS_DOMAIN_ID 를 따른다 (팀 값 116).
                        ("Ctx.inputs:useDomainIDEnvVar", True),
                        ("RP.inputs:cameraPrim",
                         [usdrt.Sdf.Path(CAM_WRIST_PRIM)]),
                        ("RP.inputs:width", WRIST_ROS_RES[0]),
                        ("RP.inputs:height", WRIST_ROS_RES[1]),
                        ("Rgb.inputs:topicName", WRIST_ROS_NS + "/rgb"),
                        ("Rgb.inputs:type", "rgb"),
                        ("Rgb.inputs:frameId", "wrist_cam"),
                        ("Rgb.inputs:frameSkipCount", WRIST_ROS_SKIP),
                        ("Rgb.inputs:resetSimulationTimeOnStop", True),
                        ("Info.inputs:topicName", WRIST_ROS_NS + "/camera_info"),
                        ("Info.inputs:frameId", "wrist_cam"),
                        ("Info.inputs:frameSkipCount", WRIST_ROS_SKIP),
                        ("Info.inputs:resetSimulationTimeOnStop", True),
                    ],
                    keys.CONNECT: [
                        ("Tick.outputs:tick", "RP.inputs:execIn"),
                        ("RP.outputs:execOut", "Rgb.inputs:execIn"),
                        ("RP.outputs:renderProductPath",
                         "Rgb.inputs:renderProductPath"),
                        ("Ctx.outputs:context", "Rgb.inputs:context"),
                        ("RP.outputs:execOut", "Info.inputs:execIn"),
                        ("RP.outputs:renderProductPath",
                         "Info.inputs:renderProductPath"),
                        ("Ctx.outputs:context", "Info.inputs:context"),
                    ],
                },
            )
            log("손목ROS   /%s/rgb  %dx%d  frameSkip %d  도메인 %s"
                % (WRIST_ROS_NS, WRIST_ROS_RES[0], WRIST_ROS_RES[1],
                   WRIST_ROS_SKIP, os.environ.get("ROS_DOMAIN_ID", "0")))
            log("          다른 터미널에서:")
            log("            ros2 run rqt_image_view rqt_image_view /%s/rgb"
                % WRIST_ROS_NS)
        except Exception:
            import traceback
            # 화면 하나 못 띄운 것뿐이다. 적재는 계속 돌아야 한다.
            log("손목ROS   그래프 생성 실패 — 발행만 없다, 적재는 계속한다\n"
                + traceback.format_exc())

    _st["sub"] = omni.kit.app.get_app().get_update_event_stream() \
        .create_subscription_to_pop(_build, name="palletris-wristcam-ros")


if WRISTROS and stage.GetPrimAtPath(CAM_WRIST_PRIM).IsValid():
    _wrist_cam_to_ros()
elif WRISTROS:
    log("손목ROS   %s 가 씬에 없다 — src/packing_sim/run_layout_to_spec.sh 를 다시 돌려라"
        % CAM_WRIST_PRIM)

if AUTOPLAY:
    omni.timeline.get_timeline_interface().play()
else:
    log("")
    log("  >>> 준비 끝. **Play 를 눌러라.**  (자동으로 시작하지 않는다)")
    log("      일시정지와 Stop 은 아직 못 이어간다 — 누르면 그 실행은 거기서 끝이다.")
    log("      예전처럼 자동으로 시작하려면  PL_AUTOPLAY=1")
    log("")


def setup():
    """물리가 돌기 시작한 뒤에 부른다 (아티큘레이션 초기화가 필요하다)."""
    import conveyor_ctl
    import mainq as MQ
    import robot_motion as RM
    import spawner as SP
    import sim_server as SS
    import isaacsim.robot_motion.motion_generation as mg
    from isaacsim.core.prims import SingleArticulation

    if SPEED:
        cfg["robot"]["joint_speed_scale"] = float(SPEED)
        log("★ 팔 속도 %s 로 이 실행에서만 덮어썼다 (사양 %s). 베이스라인 아님."
            % (SPEED, _paths.system_config()["robot"].get("joint_speed_scale", 0.5)))

    tmp = tempfile.mkdtemp(prefix="h2017_lula_")
    urdf, _n, missing = IMP.make_abs_urdf(tmp)
    if missing:
        raise RuntimeError("메시 없음: %s" % missing[:3])

    q = MQ.MainQueue(timeout_s=float(cfg["service"]["timeout_s"]) * 0.8)
    # ★ 스포너는 **하나**다. 스폰은 한 라인에서 하고, 픽업 감지만 존별로 나뉜다.
    sp = SP.Spawner(stage, cfg, q, zone=ZONE, zones=ZONES)

    # ── 존마다 팔 한 벌 ─────────────────────────────────────
    # ★ URDF 는 한 번만 만든다. 로봇 3 대가 같은 기종이라 서술자를 공유해도 되고,
    #   Lula 솔버만 대마다 따로 둔다 (베이스 자세가 다르다).
    _ARMS = {}
    for _z in ZONES:
        _a = SingleArticulation(prim_path=robot_prim(_z), name="h2017_%s" % _z)
        _a.initialize()
        _l = mg.LulaKinematicsSolver(robot_description_path=str(_paths.DESCRIPTOR),
                                     urdf_path=str(urdf))
        _p, _q4 = _a.get_world_pose()
        _l.set_robot_base_pose(np.array(_p), np.array(_q4))
        _ARMS[_z] = {"art": _a, "lula": _l, "rm": None,
                     "prim": robot_prim(_z)}
        log("로봇 %s    베이스 %s mm  EE %s  (%s)"
            % (_z, [round(float(v) * 1000) for v in _p], EE_FRAME, robot_prim(_z)))
    art = _ARMS[ZONES[0]]["art"]
    lula = _ARMS[ZONES[0]]["lula"]

    def _gap_mm_for(zn):
        """존 `zn` 의 흡착면~박스 윗면 간극 [mm]. 아래 `_gap_mm` 주석 참조."""
        def _f():
            cur = _CUR_Z.get(zn) if MULTI else _CUR["box"]
            if cur is None:
                return None
            bid, half_h = cur
            e = sp._snap.get(bid)
            if e is None:
                return None
            _a = _ARMS[zn]["art"]
            _l = _ARMS[zn]["lula"]
            _r = _ARMS[zn]["rm"]
            if _r is None:
                return None
            q6 = np.asarray(_a.get_joint_positions()[:6], dtype=float)
            fk_p, fk_R = _l.compute_forward_kinematics(EE_FRAME, q6)
            zax = np.asarray(fk_R)[:, 2]
            face = np.asarray(fk_p, dtype=float) + zax * _r.grip_len
            c = np.asarray(e["pos_m"], dtype=float)
            return (float(np.dot(c - face, zax)) - half_h) * 1000.0
        return _f

    def _gap_mm():
        """흡착면~박스 윗면 간극 [mm]. 못 재면 None.

        ★ 이걸 안 주면 `gripper._grasp_verdict` 의 간극 검사(GRIP_TOL_MM 3 mm)가
          통째로 건너뛰어진다. 그러면 "is_holding 이 참이고 물린 프림 이름이 맞다"
          까지만 보고 흡착 성공으로 친다 — 박스가 흡착면에서 몇 mm 떠서 매달려도
          모른다. `cycle.py` 의 이송·배치 높이는 전부 "딱 붙었다" 를 가정하므로
          매달린 만큼 밑면이 계산보다 낮게 내려가 이웃 상면을 긁는다.
          `verify_suction.measure` 와 같은 식: 툴 +Z 축에 투영한다.
          `rm.suction_pose()` 는 위치만 주고 축을 안 줘서 여기 못 쓴다.
        rm.step() -> poll_grasp -> _grasp_verdict 경로라 **메인 스레드**다.
        """
        cur = _CUR["box"]
        if cur is None:
            return None
        bid, half_h = cur
        e = sp._snap.get(bid)
        if e is None:
            return None
        q6 = np.asarray(art.get_joint_positions()[:6], dtype=float)
        fk_p, fk_R = lula.compute_forward_kinematics(EE_FRAME, q6)
        zax = np.asarray(fk_R)[:, 2]
        face = np.asarray(fk_p, dtype=float) + zax * rm.grip_len
        c = np.asarray(e["pos_m"], dtype=float)
        return (float(np.dot(c - face, zax)) - half_h) * 1000.0

    for _z in ZONES:
        _ARMS[_z]["rm"] = RM.RobotMotion(
            _ARMS[_z]["art"], _ARMS[_z]["lula"], cfg, ee=EE_FRAME,
            gripper_path=(robot_prim(_z) + "/SurfaceGripper") if MULTI
            else GRIPPER_PRIM,
            gap_fn=_gap_mm_for(_z) if MULTI else _gap_mm)
    rm = _ARMS[ZONES[0]]["rm"]
    _S["arms"] = _ARMS
    cc = conveyor_ctl.ConveyorCtl(stage, cfg, q)

    if CAM:
        import camera_pick
        # ★ 카메라도 **존마다** 있어야 한다. 하나만 두고 3 존 박스를 다 재면 존 A
        #   카메라가 존 B·C 지선을 보게 되어 중심 오차가 11.5 m 로 나온다
        #   (실측 2026-08-25: 종류 맞춤 1/3).
        _CAMS = {}
        for _z in ZONES:
            _CAMS[_z] = camera_pick.PickCamera(stage, cfg, q, zone=_z)
        _S["cams"] = _CAMS
        _S["cam"] = _CAMS[ZONES[0]]

    try:
        server = SS.SimServer(port=PORT)
    except OSError as e:
        # ★ 조용히 넘어가면 안 된다. 예전에는 여기서 터져도 리포트가 "0/N" 만 찍고
        #   끝나서, 진짜 이유(포트 충돌)가 로그 수백 줄 안에 묻혔다.
        #   실측 2026-08-23: GUI 시연이 돌고 있는데 헤드리스 게이트를 같이 띄웠다.
        log("\n!! 포트 %d 를 못 잡았다: %s" % (PORT, e))
        log("   이미 다른 실행이 이 포트를 쓰고 있다. 확인:")
        log("     ss -ltnp | grep %d" % PORT)
        log("   둘을 같이 돌리려면 포트를 나눠라:  PL_PORT=8796 ...")
        log("   ⚠ 다만 Isaac 두 개가 같은 GPU 를 나눠 쓰면 물리 타이밍이 흔들려")
        log("     게이트 수치가 실행마다 달라진다. 게이트는 하나씩 돌려라.")
        _kitexit.shutdown(3, log=log)
        raise SystemExit
    def _h_prim_bbox(req):
        """{"cmd":"prim_bbox","path":"/World/Pallet_A"} -> {"min_m":[..],"max_m":[..]}

        ★ 팔레트 이송을 지켜보려고 넣었다. 박스가 아니라 `get_box_pose` 로는 못 본다.
          USD 에서 읽는다 — `SingleRigidPrim` 을 만들면 시뮬레이션 뷰가 무효화되는
          사고가 있었다 (spawner._refresh 주석).
        """
        path = req.get("path")

        def _rd():
            from pxr import Usd as _U, UsdGeom as _UG
            pr = stage.GetPrimAtPath(path)
            if not pr or not pr.IsValid():
                return {"ok": False, "error": "없는 prim %s" % path}
            bb = _UG.BBoxCache(_U.TimeCode.Default(),
                               [_UG.Tokens.default_, _UG.Tokens.render,
                                _UG.Tokens.proxy])
            r = bb.ComputeWorldBound(pr).ComputeAlignedRange()
            if r.IsEmpty():
                return {"ok": False, "error": "bbox 가 비었다 %s" % path}
            a, b = r.GetMin(), r.GetMax()
            return {"ok": True,
                    "min_m": [float(a[0]), float(a[1]), float(a[2])],
                    "max_m": [float(b[0]), float(b[1]), float(b[2])]}
        return q.run(_rd)

    def _h_set_kinematic(req):
        """{"cmd":"set_kinematic","paths":[..],"on":true} -> {"ok":..,"n":..}

        ★ 왜 필요한가. 팔레트 이송은 USD 좌표를 직접 쓴다(`move_prim`). 그런데 그 위
          박스는 **동적 강체**라 좌표를 써도 PhysX 가 매 스텝 자기 상태로 되돌린다.
          즉 팔레트만 빠져나가고 박스는 제자리에 남아 벨트 위로 떨어진다
          (2026-08-25 사용자 관측: "팔레트에 놓았는데 박스가 안착 안 하고 움직임").
          그래서 반출 직전에 적재물을 **키네마틱으로 얼려** 팔레트와 한 몸으로 옮긴다.
          실물로 치면 스트레치 랩으로 감아 통째로 옮기는 것과 같다.
        """
        paths = req.get("paths") or ([req["path"]] if req.get("path") else [])
        on = bool(req.get("on", True))

        def _wr():
            from pxr import UsdPhysics as _UP
            n = 0
            for pa in paths:
                pr = stage.GetPrimAtPath(pa)
                if not pr or not pr.IsValid():
                    continue
                rb = _UP.RigidBodyAPI(pr)
                if not rb:
                    continue
                at = rb.GetKinematicEnabledAttr()
                if not at:
                    at = rb.CreateKinematicEnabledAttr(on, True)
                at.Set(on)
                n += 1
            return {"ok": True, "n": n}
        return q.run(_wr)

    def _h_rotate_prim(req):
        """{"cmd":"rotate_prim","paths":[..],"center_m":[cx,cy],"deg":d} -> {"ok":..}

        ★ 코너 턴테이블용. 팔레트를 **월드 Z 축** 기준으로 `center_m` 둘레로 돌린다.
          1톤 트럭 적재함은 2800 이고 팔레트 Y 변은 1200 이라 3장(3600)이 안 들어간다.
          코너에서 90° 돌리면 그 치수가 800 이 되어 2400 으로 들어간다
          (CLAUDE.md 작업명세서 §3.2 · 팀 결정 2026-08-25).

        ★ prim 의 op 는 [translate, orient, scale] 이고 로컬 원점이 물체 중심이다.
          월드 회전 Rz 를 얹으려면 **둘 다** 손봐야 한다:
              translate' = c + Rz * (translate - c)      (자리를 원둘레로 옮기고)
              orient'    = orient * Rz                   (자세도 같이 돌린다)
          translate 만 돌리면 팔레트가 자세는 그대로인 채 원을 그리며 미끄러진다.
        """
        paths = req.get("paths") or ([req["path"]] if req.get("path") else [])
        c = req.get("center_m") or [0.0, 0.0]
        deg = float(req.get("deg") or 0.0)

        def _wr():
            import math as _m
            from pxr import Gf as _Gf, UsdGeom as _UG
            rad = _m.radians(deg)
            ca, sa = _m.cos(rad), _m.sin(rad)
            qz = _Gf.Quatd(_m.cos(rad / 2.0), _Gf.Vec3d(0, 0, _m.sin(rad / 2.0)))
            miss = []
            for pa in paths:
                pr = stage.GetPrimAtPath(pa)
                if not pr or not pr.IsValid():
                    miss.append(pa)
                    continue
                ops = {o.GetOpName(): o
                       for o in _UG.Xformable(pr).GetOrderedXformOps()}
                t = ops.get("xformOp:translate")
                if t is None:
                    miss.append(pa)
                    continue
                v = t.Get()
                dx, dy = float(v[0]) - c[0], float(v[1]) - c[1]
                t.Set(_Gf.Vec3d(c[0] + dx * ca - dy * sa,
                                c[1] + dx * sa + dy * ca, float(v[2])))
                o = ops.get("xformOp:orient")
                if o is not None:
                    q = o.Get()
                    # ★ 원래 타입(Quatf/Quatd)을 지켜야 한다. 섞으면 Set 이 거부한다.
                    cls = type(q)
                    qd = _Gf.Quatd(float(q.GetReal()),
                                   _Gf.Vec3d(*[float(x) for x in q.GetImaginary()]))
                    n = qd * qz
                    o.Set(cls(float(n.GetReal()),
                              type(q.GetImaginary())(*[float(x)
                                                       for x in n.GetImaginary()])))
            return {"ok": not miss, "missing": miss}
        return q.run(_wr)

    def _h_move_prim(req):
        """{"cmd":"move_prim","paths":[..],"d_m":[dx,dy,dz]} -> {"ok":..}

        ★ 팔레트 이송용. 팔레트는 **키네마틱** 이라 좌표로 옮긴다 — 동적 강체로 두면
          롤러 사이로 22 mm 가라앉아 배치 Z 오차가 허용을 넘고, 컨베이어 표면속도로는
          아예 안 움직였다 (실측 2026-08-25).
        ★ `paths` 로 여러 개를 받는다. 팔레트와 그 위 적재물을 **같은 틱에 같은 변위**로
          옮겨야 한다 — 한 번에 하나씩 부르면 호출 사이에 틱이 끼어 화물이 밀린다.
        ⚠ A47 벨트가 씬에 들어오면 이 자리를 그 벨트로 바꾼다.
        """
        paths = req.get("paths") or ([req["path"]] if req.get("path") else [])
        d = req.get("d_m") or [0.0, 0.0, 0.0]

        def _wr():
            from pxr import Gf as _Gf, UsdGeom as _UG
            miss = []
            for pa in paths:
                pr = stage.GetPrimAtPath(pa)
                if not pr or not pr.IsValid():
                    miss.append(pa)
                    continue
                xf = _UG.Xformable(pr)
                op = None
                for o in xf.GetOrderedXformOps():
                    if o.GetOpName() == "xformOp:translate":
                        op = o
                        break
                if op is None:
                    miss.append(pa)
                    continue
                v = op.Get()
                op.Set(_Gf.Vec3d(float(v[0]) + float(d[0]),
                                 float(v[1]) + float(d[1]),
                                 float(v[2]) + float(d[2])))
            return {"ok": not miss, "missing": miss}
        return q.run(_wr)

    server.register({"ping": lambda r: {"ok": True},
                     "prim_bbox": _h_prim_bbox,
                     "move_prim": _h_move_prim,
                     "rotate_prim": _h_rotate_prim,
                     "set_kinematic": _h_set_kinematic})
    server.register(sp.handlers())
    server.register(cc.handlers())
    if MULTI:
        # ★ 로봇이 여러 대면 핸들러 이름이 겹친다. 요청의 `zone` 을 보고 그 존의
        #   `RobotMotion` 으로 넘긴다. `cycle.py` 는 안 고친다 — 존을 자동으로
        #   채우는 `ZoneLink` 래퍼를 씌우면 요청에 zone 이 늘 실려 온다.
        _HT = {z: _ARMS[z]["rm"].handlers() for z in ZONES}

        def _mk(cmd):
            def _h(req):
                z = str((req or {}).get("zone") or ZONES[0]).upper()
                t = _HT.get(z)
                if t is None:
                    return {"ok": False, "accepted": False,
                            "error": "모르는 존 %s (있는 것: %s)"
                                     % (z, ",".join(ZONES))}
                return t[cmd](req)
            return _h
        server.register({c: _mk(c) for c in _HT[ZONES[0]]})
        log("소켓      존 디스패치 켬 — %s" % ", ".join(ZONES))
    else:
        server.register(rm.handlers())
    if _S.get("cam"):
        if MULTI:
            _CH = {z: c.handlers() for z, c in (_S.get("cams") or {}).items()}

            def _mkc(cmd):
                def _h(req):
                    z = str((req or {}).get("zone") or ZONES[0]).upper()
                    t = _CH.get(z)
                    if t is None:
                        return {"ok": False, "error": "모르는 존 %s" % z}
                    return t[cmd](req)
                return _h
            server.register({c: _mkc(c) for c in _CH[ZONES[0]]})
        else:
            server.register(_S["cam"].handlers())
    log("소켓      127.0.0.1:%d   cmd %d 개" % (PORT, len(server.cmds())))

    # ★ READY 로 순간이동. 안 하면 첫 이동이 179 deg 라 구간 예산(20 s)을 넘겨
    #   MOTION_TIMEOUT 이 난다 (verify_b4_test1_1.py 주석의 실측).
    import yaml
    from isaacsim.core.utils.types import ArticulationAction
    ready = np.array(yaml.safe_load(_paths.DESCRIPTOR.read_text())["default_q"], dtype=float)
    for _z in ZONES:
        _ARMS[_z]["art"].set_joint_positions(ready)
        _ARMS[_z]["art"].set_joint_velocities(np.zeros(6))
    _S["ready_action"] = ArticulationAction(joint_positions=ready)
    _S["ready_q"] = ready                                # 감긴 팔을 되돌릴 때 쓴다
    log("자세      READY 로 순간이동 %s deg" % np.round(np.degrees(ready), 1).tolist())

    # ★ Camera.initialize() 는 **타임라인이 돌기 시작한 뒤**여야 한다. setup() 은
    #   부팅 20스텝 뒤라 이미 Play 상태다.
    for _z, _c in list((_S.get("cams") or {}).items()):
        if not _c.start():
            log("   [카메라] 존 %s 시작 실패 — 이 존은 인식 없이 간다" % _z)
            (_S.get("cams") or {}).pop(_z, None)
    if not (_S.get("cams") or {}):
        _S["cam"] = None
    elif _S.get("cam") is not None and ZONES[0] not in (_S.get("cams") or {}):
        _S["cam"] = None

    _S.update(rm=rm, sp=sp, cc=cc, q=q, server=server, art=art)
    cc.stop_all()


class LinkPrim:
    """`PalletVision` 이 기대하는 최소 인터페이스 — **소켓으로** 읽는다.

    B5 는 `SingleRigidPrim` 을 직접 들고 읽지만, 여기서는 `get_box_pose` 로만 본다.
    (a) 실제 오케스트레이터가 가진 것이 그것뿐이고, (b) 스포너 스냅샷은 메인 스레드가
    갱신하므로 소켓 스레드에서 PhysX 를 건드리는 일이 없다.
    """

    def __init__(self, link, box_id):
        self.link, self.box_id = link, box_id

    def get_world_pose(self):
        r = self.link.call(cmd="get_box_pose", box_id=self.box_id)
        if not r.get("ok"):
            raise RuntimeError("get_box_pose(%s) -> %r" % (self.box_id, r))
        y = math.radians(float(r.get("yaw_deg", 0.0)))
        return (np.asarray(r["pos_m"], dtype=float),
                np.array([math.cos(y / 2.0), 0.0, 0.0, math.sin(y / 2.0)]))


def _line_state(link, ids):
    """라인에 남아 있는 박스들의 위치. 타임아웃 진단용."""
    st = link.call(cmd="spawn_status")
    out = []
    for bid in ids:
        r = link.call(cmd="get_box_pose", box_id=bid)
        if not r.get("ok"):
            continue                      # 놓였거나 despawn 된 것
        x, y, z = r["pos_m"]
        out.append("%s(%.2f,%.2f,%.2f)" % (bid[-3:], x, y, z))
    return "라인 %s개 · %s" % (st.get("on_belt", "?"), " ".join(out) if out else "비었다")


# ★ 픽업 구간 판정 (아래 "낀 박스" 탈출로가 쓴다). `spawner` 의 판정과 같은 기준이다.
_PU_AX = 0 if str(cfg["conveyor"].get("pickup_axis", "x")).lower() == "x" else 1
_PU_P0 = cfg["conveyor"]["pickup_range_mm"][0] / 1000.0
_PU_P1 = cfg["conveyor"]["stopper_x_mm"] / 1000.0
_PU_CROSS = (cfg["zones"].get("pickup_cross_mm") or {}).get(ZONE)
_PU_CROSS = (None if not _PU_CROSS
             else (min(_PU_CROSS) / 1000.0, max(_PU_CROSS) / 1000.0))


_STUCK_SINCE = {}                      # box_id -> 픽업 구간에서 처음 본 시각
# 오분류로 이미 치운 박스. 중복 집계를 막는다 (despawn 반영 지연).
_MISROUTED = set()


def _stuck_box(link, ids):
    """픽업 구간에 들어와 있는데 `pickup_check` 가 못 잡는 박스 중 **가장 앞선 것**.

    ★ 왜 필요한가. 흡착이 실패하면 박스가 살짝 기운 채 남는다. 그러면 `spawner` 의
      "벨트 위인가"(z 허용오차) 판정을 못 넘어 영영 `present` 가 안 되고, 사이클은
      집으러 오지 않는데 뒤차는 계속 밀려온다 — 라인이 통째로 굳는다
      (실측 2026-08-24: BOX00005 가 132 s 동안 스토퍼 앞을 막았다).
      CLAUDE.md §5 의 `NO_FEASIBLE_POSE` 와 같은 결로, **지우고 계속** 간다.
    """
    best = None
    for bid in ids:
        r = link.call(cmd="get_box_pose", box_id=bid)
        if not r.get("ok"):
            continue
        pos = r["pos_m"]
        if not (_PU_P0 <= pos[_PU_AX] <= _PU_P1):
            continue
        if _PU_CROSS is not None:
            c = pos[1 - _PU_AX]
            if not (_PU_CROSS[0] <= c <= _PU_CROSS[1]):
                continue
        if best is None or pos[_PU_AX] > best[1][_PU_AX]:
            best = (bid, pos)
    return best


class LineServices(D.OnlineServices):
    """B5 의 온라인 서비스 그대로 + **벨트만 진짜로 세운다.**

    부모의 `set_velocity` 는 기록만 하는 mock 이다 (B5 씬에는 컨베이어가 없다).
    실제 라인에서 그러면 `cycle.py` 가 "멈췄다" 고 믿고 흡착해 박스를 끌고 간다.
    """

    def __init__(self, link, *a, **kw):
        super().__init__(*a, **kw)
        self.link = link

    def set_velocity(self, seg, mps):
        self._rec("/conveyor/set_velocity", seg, mps)
        return bool(self.link.call(cmd="set_velocity", seg=seg, mps=mps).get("ok"))

    def reset_pallet(self):
        """`/algo/reset` 대역 — 하이트맵을 비운다.

        `plan_placement` 이 매번 `vision.scan()` 으로 하이트맵을 다시 세우므로,
        비전이 본 목록만 비우면 된다. **`swap_pallet` 과 반드시 짝으로** 부른다 —
        안 비우면 패커가 이미 내보낸 박스 위에 다음 자리를 잡는다.
        """
        self._rec("/algo/reset", "B")
        self.vision.placed.clear()
        self.last = None


def meter_thread():
    """픽업 구간에 한 대씩만 들여보낸다. **자기 소켓을 쓴다.**

    ★ 2026-08-25 에 방식을 바꿨다. 예전에는 픽업 구간이 차면 **본선(`main`)을
      통째로 세웠다.** 그래서 팔이 집는 동안 라인 전체가 멎었고(사용자 관측),
      로봇 3 대를 동시에 돌리면 한 존이 집는 사이 나머지 두 존이 굶는다.

      지금은 본선을 **안 세운다.** 대신 라우팅이 "그 존 픽업 구간이 찼으면
      **안 꺾는다**" 로 계량한다 — 안 꺾인 박스는 순환 루프를 한 바퀴 돌고
      다시 온다. 실제 물류 라인의 recirculation 과 같은 동작이다.

    ★ 그래서 이 스레드가 하는 일은 이제 **본선을 계속 돌려 두는 것** 뿐이다.
      (`PL_METER=0` 이면 아예 안 뜬다.)
    """
    import sim_link as SL
    link = SL.SimLink(port=PORT)
    _S["meter_link"] = link
    running = None
    try:
        if ROUTE_ON:
            # ★ 라우팅이 본선을 직접 여닫는다 (`router_thread` 의 "본선 혼잡 제어").
            #   여기서 또 건드리면 둘이 싸운다.
            log("   [계량] 본선 제어는 라우팅이 한다 — 계량은 손 뗀다")
            return
        # ── 라우팅이 없을 때(test1-1 등): 예전대로 본선을 여닫는다 ──
        # ★ 이 경로를 남겨 둬야 한다. 분기가 없으면 박스가 지선으로 **직진**하므로
        #   "안 꺾고 한 바퀴 돌린다" 라는 계량 수단 자체가 없다. 여기서 본선을
        #   안 세우면 픽업 구간에 박스가 줄줄이 밀려든다 (test1-1 기준선 회귀).
        while not _S["done"]:
            st = link.call(cmd="spawn_status")
            want = st.get("queue", 0) < METER_MAX
            if want != running:
                r = link.call(cmd="set_velocity", seg="main",
                              mps=BELT_MPS if want else 0.0)
                if r.get("ok"):
                    running = want
                    log("   [계량] main %s (픽업 구간 %s 개)"
                        % ("가동" if want else "정지", st.get("queue", "?")))
            time.sleep(0.2)
    except Exception as e:
        log("   [계량] !! %r" % (e,))


# ── 위치변환 레일 (색상 라우팅) ────────────────────────────────────────────
ROUTE = cfg.get("routing") or {}
ROUTE_ON = bool(ROUTE) and os.environ.get("PL_ROUTE", "1") not in ("", "0")


def _zone_of(specs, bid):
    """`bid` 가 **가야 할** 존. 모르면 None.

    ★ 왜 필요한가 (실측 2026-08-26). `pickup_check` 는 그 존 스토퍼에 멎은 박스를
      그냥 돌려준다 — **배송지를 안 본다.** 그래서 엉뚱한 지선에 올라탄 박스를
      로봇이 집어 그 존 팔레트에 놓는데, 어디에서도 에러가 안 난다.

      33 박스 실행에서 BOX00026(DAEJEON->C)이 A 팔레트에 올라갔다. 사슬은 이렇다:
        BOX00025 가 지선 A 입구 Y 0.598 에서 멎었다 (진입 판정은 Y 1.2)
        -> 라우터가 진입을 기다리느라 **소터 A 를 분기로 계속 열어 뒀다**
        -> 그 사이 지나가던 BOX00026 이 같이 밀려 A 지선에 올라탔다
           (BOX00026 에는 `-> 존 C` 라우팅 명령이 아예 없다)
        -> 배송지 검사가 없어 그대로 적재
    """
    if not ROUTE_ON:
        return None
    tbl, keyfn = _route_map()
    sp = specs.get(bid)
    if not sp:
        return None
    k = keyfn(sp)
    return tbl.get(str(k)) if k is not None else None


def _route_map():
    """분류 기준 -> (표, 박스에서 열쇠를 뽑는 함수).

    ★ 2026-08-25: 기본은 **배송지(QR)** 다. 우선순위 by_dest > by_color > by_type.
      - `by_dest`  2 차 사양. QR 이 배송지를 싣는다. 박스 색은 하나로 통일한다.
      - `by_color` 1 차 사양(HSV). 되돌릴 수 있게 남겨 둔다.
      - `by_type`  크기 기준. 색도 QR 도 없는 씬을 위한 하위호환.
    """
    bd = ROUTE.get("by_dest")
    if bd:
        return ({str(k): str(v) for k, v in bd.items()},
                lambda sp: sp.get("dest"))
    bc = ROUTE.get("by_color")
    if bc:
        return ({str(k): str(v) for k, v in bc.items()},
                lambda sp: sp.get("color"))
    bt = ROUTE.get("by_type") or {}
    return ({str(k): str(v) for k, v in bt.items()},
            lambda sp: sp.get("name"))


def router_thread(specs, ids):
    """배송지로 목적지 존을 정하고, 그 분기점의 **소터**로 박스를 지선에 밀어넣는다.

    ★ 소터 방식 (2026-08-25, `routing.use_sorter: true`).
      분기 조각이 `ConveyorBelt_A47` 이면 조각 안에 소터가 있다. 소터는 자기
      `IsaacConveyor` 를 가진 **독립 컨베이어**라, 소터만 +Y 로 돌리면 그 위의
      박스가 지선으로 밀려 올라가고 **뒤차는 본선을 그대로 흘러온다.**
      배타 구간이 소터 폭(1491.7 mm)뿐이다.

    ★ (구) 조각 꺾기 (`use_sorter: false`, A23 씬).
      본선 조각 **전체(6925 mm)** 의 표면속도를 옆으로 돌린다. 그래서 그 조각에
      박스가 둘 이상이면 못 꺾고, 안 꺾인 박스는 순환 루프를 돌아 다시 와서 더
      몰린다. 투입 간격을 20 s 까지 벌려야 했던 이유가 이것이다.

    ★ 어느 쪽이든 "밀리는 구간에 목표 박스만 있을 때" 꺾는 것은 같다. 다른 것은
      그 구간이 6925 mm 냐 1491.7 mm 냐다.
    """
    import sim_link as SL
    link = SL.SimLink(port=PORT)
    kind_of, key_of = _route_map()
    jx = {k: (v[0] / 1000.0, v[1] / 1000.0)
          for k, v in (ROUTE.get("junction_x_mm") or {}).items()}
    mseg = ROUTE.get("main_seg") or {}
    segx = {k: (min(v) / 1000.0, max(v) / 1000.0)
            for k, v in (ROUTE.get("main_seg_x_mm") or {}).items()}
    y_done = float(ROUTE.get("divert_done_y_mm", 1200)) / 1000.0
    _mly = ROUTE.get("main_line_y_mm") or [-500, 500]
    y_lo, y_hi = min(_mly) / 1000.0, max(_mly) / 1000.0
    thru = list(ROUTE.get("thru_world", [-1, 0, 0]))
    div = list(ROUTE.get("divert_world", [0, 1, 0]))
    # ── 소터 ──────────────────────────────────────────
    use_sorter = bool(ROUTE.get("use_sorter")) and bool(ROUTE.get("sorter_x_mm"))
    _sx_mm = ROUTE.get("sorter_x_mm") or {}
    guard = float(ROUTE.get("sorter_guard_mm", 1200)) / 1000.0
    # ★ 배타 구간. 소터 구간 + **상류(+X)** 여유. 본선이 -X 로 흐르므로 뒤차는
    #   X 가 큰 쪽에서 온다. 꺾는 동안 여기 들어오는 박스도 같이 밀린다.
    sortx = {k: (min(v) / 1000.0, max(v) / 1000.0 + guard)
             for k, v in _sx_mm.items()}

    def turn(zone, seg, want_div):
        """분류 명령. 소터가 있으면 소터만, 없으면 본선 조각을 꺾는다.

        ★ 꺾을 때 속도를 같이 준다. 계량이 소터를 세워 뒀을 수 있는데, 소터가
          서 있으면 박스가 지선으로 안 밀릴 뿐 아니라 **본선 롤러의 구멍 위에서
          그대로 멈춘다** (A47 은 소터 자리에 본선 롤러가 없다).
        """
        if use_sorter:
            # ★ 2026-08-26 분기할 때만 주던 것을 **항상** 준다 (김병직 지시서 §7②).
            #   A47 애셋의 `SorterSpeed` 기본값이 0.0 이고 `h_set_sorter` 규약이
            #   "mps 를 안 주면 건드리지 않는다" 라, 통과(divert=False)만 시키면
            #   소터가 영영 0 인 채로 남는다. 본선 롤러에는 소터 자리에 구멍이
            #   있어서 서 있는 소터는 그냥 지나가지지 않고 **라인을 막는다.**
            kw = {"cmd": "set_sorter", "zone": zone,
                  "divert": bool(want_div), "mps": BELT_MPS}
            return link.call(**kw)
        return link.call(cmd="set_direction", seg=seg,
                         world=(div if want_div else thru))
    # ★ 존별 계량. 그 존 픽업 구간이 차 있으면 **안 꺾는다** — 안 꺾인 박스는
    #   순환 루프를 돌고 다시 온다. 본선을 세우지 않으므로 로봇 3 대가 서로
    #   굶기지 않는다 (`meter_thread` 주석 참조).
    _cross = (cfg["zones"].get("pickup_cross_mm") or {})
    _pr = cfg["conveyor"].get("pickup_range_mm") or [0, 0]
    pk_y = (min(_pr) / 1000.0, max(_pr) / 1000.0)
    pk_x = {k: (min(v) / 1000.0, max(v) / 1000.0) for k, v in _cross.items()}

    def zone_load(zn, allpos):
        """`zn` 픽업 구간(지선)에 있는 박스 수."""
        r = pk_x.get(zn)
        if r is None:
            return 0
        return sum(1 for q in allpos.values()
                   if r[0] <= q[0] <= r[1] and pk_y[0] <= q[1] <= pk_y[1])

    state = {}                      # box_id -> "divert" | "done"
    laps = {}                       # box_id -> 리턴 라인을 돈 횟수
    in_loop = {}                    # box_id -> 지금 루프 안에 있나
    # PL_MAX_LAPS 로 덮어쓸 수 있다 — 소멸 경로를 짧은 실행에서 확인할 때 쓴다
    MAX_LAPS = int(os.environ.get("PL_MAX_LAPS")
                   or ROUTE.get("max_laps", 0) or 0)
    LOOP_Y = float(ROUTE.get("loop_y_mm", -1500)) / 1000.0
    t_div = {}                      # box_id -> 꺾기 시작한 시각
    t_log = {}                      # box_id -> 마지막 진행 로그 시각
    # ★ 꺾은 채로 두면 그 조각 위의 뒤차가 전부 옆으로 밀린다. 실측 2026-08-24:
    #   복귀를 안 시켰더니 박스 6 개가 분기점에 뭉쳐 라인이 막혔다.
    DIV_TIMEOUT_S = float(os.environ.get("PL_DIV_TIMEOUT", "45"))
    log("   [라우팅] 켬 (%s) — %s"
        % ("소터 · 배타 구간 %.0f mm" % ((max(_sx_mm["A"]) - min(_sx_mm["A"]))
                                        + guard * 1000)
           if use_sorter else "조각 꺾기",
           ", ".join("%s->%s" % kv for kv in sorted(kind_of.items()))))
    try:
        while not _S["done"]:
            # ★ **done 박스도** 읽는다. 지선에 올라탄 박스가 곧 그 존의 부하다.
            #   예전처럼 건너뛰면 존이 꽉 차 있어도 계속 꺾어 보낸다.
            allpos = {}
            for bid in ids:
                r = link.call(cmd="get_box_pose", box_id=bid)
                if r.get("ok"):
                    allpos[bid] = r["pos_m"]
            poses = {b: q for b, q in allpos.items() if state.get(b) != "done"}
            # ── 리턴 라인 재순환 카운트 → MAX_LAPS 바퀴면 치운다 ──────────
            # ★ 존이 영영 안 비면 박스가 **영원히 돈다.** 장시간 실행에서 prim 이
            #   쌓여 프레임이 떨어진다 (실측 2026-08-25: 12 개 중 8 개가 동시에
            #   루프를 돌았다). 한 바퀴 = 루프로 내려갔다가 본선 띠로 돌아오는 것.
            for _b, _q in poses.items():
                if _q[1] < LOOP_Y:
                    in_loop[_b] = True
                elif in_loop.get(_b) and y_lo <= _q[1] <= y_hi:
                    in_loop[_b] = False
                    laps[_b] = laps.get(_b, 0) + 1
                    log("   [라우팅] %s 리턴 %d 바퀴째" % (_b, laps[_b]))
                    if MAX_LAPS > 0 and laps[_b] >= MAX_LAPS:
                        rr = link.call(cmd="despawn_box", box_id=_b)
                        state[_b] = "done"
                        log("   [라우팅] %s %d 바퀴 -> 소멸 (%s)"
                            % (_b, laps[_b],
                               "ok" if rr.get("ok") else repr(rr)))
            for bid, p in list(poses.items()):
                z = kind_of.get(key_of(specs[bid]))
                if z is None or z not in jx or z not in mseg:
                    continue
                lo, hi = jx[z]
                seg = mseg[z]
                if p[1] >= y_done:                       # 지선에 올라탔다
                    if state.get(bid) == "divert":
                        turn(z, seg, False)
                        _S["diverting"] = max(0, _S.get("diverting", 1) - 1)
                        log("   [라우팅] %s 지선 %s 진입 (Y %.2f m) — 본선 복귀"
                            % (bid, z, p[1]))
                    state[bid] = "done"
                    continue
                if state.get(bid) == "divert":
                    # 진행을 1 초마다 남긴다. 어디서 걸리는지 봐야 고칠 수 있다.
                    now = time.time()
                    if now - t_log.get(bid, 0.0) >= 1.0:
                        t_log[bid] = now
                        log("   [라우팅]   %s 이동 중  X %.2f  Y %.3f  (%.0f s)"
                            % (bid, p[0], p[1], now - t_div.get(bid, now)))
                    if now - t_div.get(bid, 0.0) > DIV_TIMEOUT_S:
                        turn(z, seg, False)
                        _S["diverting"] = max(0, _S.get("diverting", 1) - 1)
                        state[bid] = "done"
                        log("   [라우팅] !! %s 가 %.0f s 안에 지선 %s 에 못 올라탔다 "
                            "(Y %.2f m) — 본선 복귀"
                            % (bid, DIV_TIMEOUT_S, z, p[1]))
                    continue
                # ★ 박스가 지선 폭 안에 **완전히** 들어왔을 때만 꺾는다.
                #   가장 큰 5호가 480 mm 라 240 을 물린다. 예전엔 분기점 경계에서
                #   바로 꺾어 몸통 일부가 레일 밑으로 들어가 걸렸다 (실측 X -18.89).
                if not (lo + 0.25 <= p[0] <= hi - 0.25):
                    continue
                # ★ 본선 위에 있을 때만. 순환 루프(Y -5.1)를 지나는 박스는 X 가
                #   분기점과 맞아도 그 조각 위에 없다 — 꺾어 봐야 안 움직인다.
                if not (y_lo <= p[1] <= y_hi):
                    continue
                # 같은 조각 위에 다른 박스가 있으면 지나갈 때까지 기다린다.
                # 꺾으면 그 **조각 전체**가 옆으로 흐른다. 조각 위에 다른 박스가
                # 하나라도 있으면 기다린다 (분기점 주변만 보면 놓친다).
                # ★ 소터면 **소터 구간 + 상류 여유** 만 비면 된다 (1491.7+1200).
                #   조각 꺾기면 조각 전체(6925)가 비어야 한다.
                slo, shi = (sortx.get(z) if use_sorter else None) \
                    or segx.get(z, (lo - 3.5, hi + 3.5))
                # ★ "같은 조각 위" 도 **본선 띠 안**이어야 한다. 순환 루프(Y -5.1)를
                #   도는 박스는 X 가 겹쳐도 이 조각에 없다. 띠를 안 보면 루프에 박스가
                #   하나라도 있는 한 영원히 안 꺾는다 (실측 2026-08-24: BOX00008 ·
                #   BOX00010 이 180 s 동안 못 들어와 중단).
                busy = [b2 for b2, q in poses.items()
                        if b2 != bid and y_lo <= q[1] <= y_hi
                        and slo <= q[0] <= shi]
                if busy:
                    continue
                # ★ 존 계량. 이미 그 존 지선에 METER_MAX 대가 있으면 안 꺾는다.
                #   박스는 루프를 한 바퀴 돌고 돌아온다 — 본선을 세우지 않는다.
                if METER_MAX > 0 and zone_load(z, allpos) >= METER_MAX:
                    if time.time() - t_log.get("z" + z, 0.0) >= 10.0:
                        t_log["z" + z] = time.time()
                        log("   [라우팅] 존 %s 가 차 있다 (%d 대) — %s 는 한 바퀴 돈다"
                            % (z, zone_load(z, allpos), bid))
                    continue
                r = turn(z, seg, True)
                if r.get("ok"):
                    state[bid] = "divert"
                    t_div[bid] = time.time()
                    _S["diverting"] = _S.get("diverting", 0) + 1
                    # 계량이 세워 뒀을 수 있으니 이 조각만 직접 돌린다.
                    link.call(cmd="set_velocity", seg=seg, mps=BELT_MPS)
                    log("   [라우팅] %s %-8s %-3s -> 존 %s   X %.2f m 에서 꺾는다"
                        % (bid, key_of(specs[bid]) or "-",
                           specs[bid]["name"], z, p[0]))
                else:
                    log("   [라우팅] !! 분류 명령 실패 (존 %s · %s) -> %r"
                        % (z, seg, r))
            # ── 본선 혼잡 제어 ──────────────────────────────
            # ★ 배타 구간에 박스가 둘 이상이면 그 사이 아무도 못 꺾고, 안 꺾인
            #   박스는 순환 루프를 돌아 다시 와서 더 몰린다 (실측 2026-08-25:
            #   조각 꺾기 시절 세 박스가 45 s 동안 한 번도 안 꺾였다).
            #   그래서 **어느 구간이든 2 개 이상이면 투입을 잡는다.**
            #   ⚠ 옛 계량처럼 "픽업 구간이 찼으면 정지" 가 아니다. 그건 한 로봇이
            #     집는 동안 나머지 두 대를 굶겼다. 이 기준은 로봇과 무관하다.
            #   ★ 소터에서는 그 구간이 6925 -> 2692 mm 로 줄어 훨씬 덜 걸린다.
            crowd = 0
            for _z2, _sx in (sortx if use_sorter else segx).items():
                n = sum(1 for q in allpos.values()
                        if y_lo <= q[1] <= y_hi and _sx[0] <= q[0] <= _sx[1])
                crowd = max(crowd, n)
            # ⚠ **본선을 세우면 안 된다.** 세우면 그 조각의 박스도 못 빠져나가서
            #   붐빔이 영영 안 풀린다 (실측 2026-08-25: `[본선] 정지 (최대 2 개)`
            #   에서 그대로 굳었다). 본선은 계속 돌리고 **투입**을 잡는다 —
            #   이미 라인에 있는 박스는 흘러서 분기점을 지나거나 루프로 빠진다.
            hold = crowd >= 2
            if hold != bool(_S.get("hold_spawn")):
                _S["hold_spawn"] = hold
                log("   [투입] %s (%s 최대 %d 개)"
                    % ("보류 — 본선이 붐빈다" if hold else "재개",
                       "소터 구간" if use_sorter else "분기 조각", crowd))
            time.sleep(0.15)
    except Exception as e:
        log("   [라우팅] !! %r" % (e,))


def spawner_thread(specs, ids):
    """사양 스케줄대로 박스를 라인에 올린다. **자기 소켓을 따로 쓴다** (스레드 안전)."""
    import sim_link as SL
    link = SL.SimLink(port=PORT)
    _S["spawn_link"] = link
    try:
        for i, bid in enumerate(ids):
            if _S["done"]:
                return
            if i:
                # 사양 간격이 **하한**이다.
                t_end = time.time() + SPAWN_DT
                while time.time() < t_end and not _S["done"]:
                    time.sleep(0.05)
                if _S["done"]:
                    return
                # ★ 백압 — 버퍼가 차 있으면 넣지 않는다 (설계결정서 §7:
                #   "목표 구역 버퍼에 여유 있음 -> 통과"). 사이클 14.5 s 인데 간격이
                #   12 s 라 무조건 넣으면 큐가 계속 길어진다. 그러면 벨트가 뒤에서
                #   계속 밀어 박스들이 서로 눌리고, 결국 솔버가 터져 박스가 초속
                #   수 km 로 날아간다 (2026-08-23 BOX00014 실측). 사양대로 막는다.
                t_wait = time.time()
                while not _S["done"]:
                    st = link.call(cmd="spawn_status")
                    # ★ 분기 조각이 붐비면 새 박스를 안 넣는다. 조각 전체가 옆으로
                    #   흐르는 꺾기 방식이라 조각에 둘 이상이면 못 꺾고, 안 꺾인
                    #   박스가 루프를 돌아 다시 와서 더 몰린다.
                    if _S.get("hold_spawn"):
                        time.sleep(0.5)
                        continue
                    # ★ 계량 운전에서는 `main` 이 서 있는 동안 앞차가 스폰점 근처에
                    #   그대로 남는다. 그때 다음 박스를 내면 **겹쳐 스폰된다.**
                    #   벨트가 계속 돌 때는 12 s 뒤 6 m 밖이라 없던 문제다.
                    if (st.get("queue", 0) < BUFFER_CAP
                            and st.get("spawn_clear", True)):
                        break
                    if time.time() - t_wait > 120.0:
                        log("   [투입] 버퍼가 120 s 째 안 빈다 — 그냥 넣는다")
                        break
                    time.sleep(0.5)
                held = time.time() - t_wait
                if held > 1.0:
                    log("   [투입] 백압 대기 %.0f s (버퍼 %d 대기 중)"
                        % (held, BUFFER_CAP))
            t = specs[bid]
            r = link.call(cmd="spawn_box", box_id=bid, type=t["name"], yaw_deg=0,
                          color=t.get("color"), dest=t.get("dest"))
            st = link.call(cmd="spawn_status")
            log("   [투입] %-9s %-8s %-3s  (라인 위 %s 개)"
                % (bid, t.get("dest", t.get("color", "-")), t["name"],
                   st.get("on_belt", "?")))
            if not r.get("ok"):
                log("   [투입] !! spawn_box %s -> %r" % (bid, r))
    except Exception as e:
        log("   [투입] !! %r" % (e,))


def ship_pallet(link, zone, log, cargo_ids=None):
    """팔레트를 셀 컨베이어 -> 합류 라인 -> 트럭으로 보낸다.

    ★ 순서가 있다. 셀은 +Y, 합류는 +X 다. 둘을 같이 돌리면 코너에서 대각선으로
      밀려 라인에 낀다. **셀로 코너까지 보낸 뒤 셀을 세우고** 합류를 돌린다.

    ★ 팔레트는 키네마틱이라 좌표로 옮긴다 (`move_prim` 주석 참조).

    ★ **적재물도 같이 옮긴다.** 팔레트만 옮기면 그 위 박스는 동적 강체라 PhysX 가
      제자리에 붙들어, 팔레트가 밑에서 빠져나가고 박스는 벨트 위로 떨어진다
      (2026-08-25 사용자 관측). 그래서 반출 직전에 적재물을 키네마틱으로 얼리고
      팔레트와 **같은 틱에 같은 변위**로 민다. 도착 뒤에도 풀지 않는다 — 팔레트가
      적재함 문턱에 걸쳐 서기 때문에 지금 풀면 그대로 미끄러진다.
      A47 벨트가 들어오면 이 이송을 그 벨트로 바꾼다.
    """
    z = zone.lower()
    pal = os.environ.get("PL_PALLET_PRIM", "/World/Pallet_%s" % zone)
    cargo = ["/World/boxes/%s" % b for b in (cargo_ids or [])]
    mps = SHIP_MPS

    def bb(path=None):
        r = link.call(cmd="prim_bbox", path=path or pal)
        return (r["min_m"], r["max_m"]) if r.get("ok") else (None, None)

    a0, b0 = bb()
    if a0 is None:
        log("   [반출] !! 팔레트 %s 를 못 읽는다 — 건너뛴다" % pal)
        return None
    log("   [반출] 시작 — %s  X %.2f~%.2f  Y %.2f~%.2f"
        % (pal, a0[0], b0[0], a0[1], b0[1]))

    # ★ 적재물을 얼린다. 얼리기 **전에** 팔레트 대비 상대위치를 재 둔다 — 도착 뒤
    #   같은 값이 나와야 "안착한 채로 실려갔다" 고 말할 수 있다.
    rel0 = {}
    for c in cargo:
        ca, cbx = bb(c)
        if ca is None:
            continue
        rel0[c] = [(ca[i] + cbx[i]) / 2.0 - (a0[i] + b0[i]) / 2.0 for i in range(3)]
    if cargo:
        r = link.call(cmd="set_kinematic", paths=cargo, on=True)
        log("   [반출] 적재물 %d 개 고정 (키네마틱 %s)"
            % (len(rel0), r.get("n", "?")))

    ma, mb = bb("/World/MergeLine_%s/Rollers" % zone)
    if ma is None:
        log("   [반출] !! 합류 라인을 못 읽는다")
        return None
    y_target = (ma[1] + mb[1]) / 2.0
    ta, tb = bb("/World/Truck_Bed")
    x_bed = ta[0] if ta is not None else -2.4

    def glide(d, label):
        """정렬 이동을 **반출 속도로 흘려보낸다.**

        ★ 한 틱에 옮기면 팔레트가 순간이동한다. 실측 2026-08-25: 커브 진입
          중심선으로 맞추느라 368 mm 를 한 번에 옮겼더니 화면에서 팔레트가
          그 자리에서 사라졌다 나타난다 (사용자 관측 "갑자기 순간이동").
          벨트 위 물건은 그렇게 안 움직인다.
        """
        n = math.hypot(d[0], d[1])
        if n < 0.001:
            return
        secs = max(0.25, n / mps if mps > 0 else 0.25)
        steps = max(3, int(secs / 0.05))
        for _ in range(steps):
            link.call(cmd="move_prim", paths=[pal] + cargo,
                      d_m=[d[0] / steps, d[1] / steps, d[2] / steps])
            time.sleep(secs / steps)
        log("   [반출]   %s [%+.0f %+.0f] mm (%.1f s)"
            % (label, d[0] * 1000, d[1] * 1000, secs))

    def run_axis(seg, axis, target, mode, label):
        """`axis` 방향으로 목표까지 민다. 반환: 도달 여부.

        `mode`  "mid" = 중심 기준 · "min" = **뒤쪽 끝** 기준.
        ★ 적재함은 "앞머리가 닿으면" 이 아니라 **뒤쪽 끝까지 들어가야** 실린 것이다.
          앞머리(max) 기준으로 멈추면 팔레트가 문턱에 걸쳐 선다 (2026-08-25 실측:
          X -3.19~-2.39, 적재함 -2.40~-0.80 — 10 mm 만 얹혀 있었다).
        """
        link.call(cmd="set_velocity", seg=seg, mps=mps)
        t0 = prev = time.time()
        last = 0.0
        hit = False
        while time.time() - t0 < SHIP_TIMEOUT_S and not _S["done"]:
            now = time.time()
            d = [0.0, 0.0, 0.0]
            d[axis] = mps * (now - prev)
            prev = now
            link.call(cmd="move_prim", paths=[pal] + cargo, d_m=d)
            a, b = bb()
            if a is None:
                break
            cur = a[axis] if mode == "min" else (a[axis] + b[axis]) / 2.0
            if now - last >= 2.0:
                last = now
                log("   [반출]   %s  %.2f -> %.2f" % (label, cur, target))
            if cur >= target:
                hit = True
                break
            time.sleep(0.03)
        link.call(cmd="set_velocity", seg=seg, mps=0.0)
        return hit

    # ── 커브 코너를 **따라간다** ─────────────────────────────────
    # ★★ 2026-08-25. 제자리에서 90° 돌리는 것을 그만둔다 — 씬에 코너 커브
    #   (`ConveyorBelt_A02`)가 들어왔으므로, 팔레트는 **그 벨트를 따라 돌면 된다.**
    #   실물 커브 컨베이어가 하는 일이 정확히 이것이다: 진행하면서 방향이 바뀐다.
    #
    #   이전 방식들이 왜 안 됐는지 (전부 실측):
    #     들어서 돌리기      리프트 장치가 씬에 없다 — 팔레트가 혼자 떠올랐다
    #     코너에서 제자리    회전 대각선 1442 > 합류 프레임 1000 -> 옆 레인을
    #                       220~260 mm 파고든다. 세 대가 동시에 반출하므로
    #                       팔레트끼리도 겹칠 수 있다
    #     셀 위에서 제자리   프레임 안에는 들어가지만 **돌리는 장치가 없다**
    #
    #   호는 이렇게 잡는다. 셀 중심선(X=cx0)을 +Y 로 들어와 합류 중심선(Y=y_merge)을
    #   +X 로 나간다. 오른쪽으로 도는 90° 이므로 회전 중심은 (cx0+R, y_merge-R) 이고
    #   팔레트 중심은 그 원을 각도 180° -> 90° 로 지난다.
    #   **R 은 커브 조각 bbox 에서 잰다** — 호가 커브 밖으로 나가면 안 된다.
    arc = (cfg["conveyor"].get("curve_arc") or {}).get(zone)
    if not arc:
        log("   [반출] !! 존 %s 의 curve_arc 설정이 없다 — 호를 못 그린다" % zone)
        return None
    cenx = float(arc["center_mm"][0]) / 1000.0
    ceny = float(arc["center_mm"][1]) / 1000.0
    R = float(arc["radius_mm"]) / 1000.0
    # 팔레트 반치수 — 씬에서 잰다 (설정과 어긋나도 씬이 맞다)
    p0a, p0b = bb()
    half_w = (p0b[0] - p0a[0]) / 2.0                 # 진입할 때 가로(X) 반치수
    half_l = (p0b[1] - p0a[1]) / 2.0                 # 진입할 때 세로(Y) 반치수
    # ★ 이 호로 돌 때 팔레트가 벨트 안에 있나 — **재서 남긴다.**
    #   안쪽은 짧은 변 중앙이, 바깥쪽은 **모서리**가 제일 멀리 나간다.
    r_in = R - half_w
    r_out = math.hypot(R + half_w, half_l)
    cvp = (cfg["conveyor"].get("curve_prim") or {}).get(zone, "")
    log("   [반출]   호  중심 (%.2f, %.2f) · 반지름 %.0f mm  ->  팔레트 안쪽 %.0f · "
        "바깥 모서리 %.0f mm  (벨트 %s)"
        % (cenx, ceny, R * 1000, r_in * 1000, r_out * 1000,
           cvp.rsplit("/", 1)[-1]))

    # ① 진입 중심선(X = 중심 - R)으로 옮기고, 셀을 따라 +Y 로 진입점까지
    ax, ay = cenx - R, ceny
    pa, pb = bb()
    dx = ax - (pa[0] + pb[0]) / 2.0
    glide([dx, 0.0, 0.0], "호 진입 중심선으로")
    ok1 = run_axis("%s_cell" % z, 1, ay, "mid", "셀 이송  Y")
    if not ok1:
        log("   [반출] !! 셀 구간에서 시간 초과")
    # ② 진입점으로 정확히 맞춘다 (이송은 스텝 단위라 조금 넘겨 선다)
    pa, pb = bb()
    d = [ax - (pa[0] + pb[0]) / 2.0, ay - (pa[1] + pb[1]) / 2.0, 0.0]
    glide(d, "커브 진입점 정렬")

    # 호를 따라간다. 매 스텝 **제자리에서 조금 돌리고 다음 호점으로 옮긴다.**
    # ★ 속도는 **반출 속도 그대로**다 (2026-08-25 사용자 요청 "커브 돌 때 느리게").
    #   호 길이 = R x pi/2 를 mps 로 나눠 걸리는 시간을 정한다 — 직선 구간과
    #   같은 속도로 도니까 코너에서만 빨라지거나 느려지지 않는다.
    #   TURN_S 는 **하한**으로만 쓴다 (너무 짧은 호에서 순간이동처럼 보이지 않게).
    ccx, ccy = ax, ay
    arc_len = R * math.pi / 2.0
    dur = max(TURN_S, arc_len / mps if mps > 0 else TURN_S)
    steps = max(12, int(dur / 0.08))
    log("   [반출]   호 길이 %.0f mm · %.1f s (%.2f m/s) · %d 스텝"
        % (arc_len * 1000, dur, arc_len / dur, steps))
    link.call(cmd="set_velocity", seg="%s_curve" % z, mps=mps)
    for i in range(1, steps + 1):
        if _S["done"]:
            break
        phi = math.radians(180.0 - 90.0 * (float(i) / steps))
        link.call(cmd="rotate_prim", paths=[pal] + cargo,
                  center_m=[ccx, ccy], deg=TURN_DEG / steps)
        tx = cenx + R * math.cos(phi)
        ty = ceny + R * math.sin(phi)
        link.call(cmd="move_prim", paths=[pal] + cargo,
                  d_m=[tx - ccx, ty - ccy, 0.0])
        ccx, ccy = tx, ty
        time.sleep(dur / steps)
    pa, pb = bb()
    log("   [반출]   커브 통과 — X %.2f~%.2f (%.0f) · Y %.2f~%.2f (%.0f)"
        % (pa[0], pb[0], (pb[0] - pa[0]) * 1000,
           pa[1], pb[1], (pb[1] - pa[1]) * 1000))

    # ★ 합류 롤러 한가운데로 Y 정렬한다. 셀 이송은 목표를 조금 넘겨 멈추므로
    #   (스텝 단위) 그대로 두면 수십 mm 어긋난 채 롤러에 앉는다.
    ma2, mb2 = bb("/World/MergeLine_%s/Rollers" % zone)
    if ma2 is not None:
        pa, pb = bb()
        dy = (ma2[1] + mb2[1]) / 2.0 - (pa[1] + pb[1]) / 2.0
        glide([0.0, dy, 0.0], "합류 롤러에 Y 정렬")

    pa, pb = bb()
    # ★ 커브를 돌고 나면 적재함을 가로지르는 치수(Y)가 1200 이 아니라 **800** 이어야
    #   한다. 그래야 세 장이 1톤 적재함 2800 에 들어간다.
    turned = abs((pb[0] - pa[0]) - 1.2) < 0.05 and abs((pb[1] - pa[1]) - 0.8) < 0.05
    log("   [반출]   커브 뒤 자세  X %.0f · Y %.0f mm   %s"
        % ((pb[0] - pa[0]) * 1000, (pb[1] - pa[1]) * 1000,
           "가로 800 확보" if turned else "!! 방향이 안 맞는다"))

    ok2 = run_axis("%s_merge" % z, 0, x_bed, "min", "적재 X뒤끝")
    a, b = bb()
    if a is None:
        return None
    log("   [반출] 끝 — 팔레트 X %.2f~%.2f · Y %.2f~%.2f · 적재함 %.2f~%.2f  %s"
        % (a[0], b[0], a[1], b[1], x_bed, (tb[0] if tb is not None else x_bed + 1.6),
           "완전히 실림" if ok2 else "!! 미도착"))

    # ★ 적재물이 팔레트를 따라왔는지 **실측**한다. 상대위치가 그대로면 안착이다.
    #   ⚠ 코너에서 90° 돌았으므로 상대위치도 **같이 돌아야** 정상이다. 안 돌리고
    #     비교하면 멀쩡히 실려 간 박스가 675 mm 미끄러진 것으로 나온다
    #     (실측 2026-08-25 — 지표가 틀린 것이지 박스가 움직인 게 아니었다).
    _rad = math.radians(TURN_DEG if turned else 0.0)
    _c, _s = math.cos(_rad), math.sin(_rad)
    slip_mm = 0.0
    for c, r0 in rel0.items():
        ca, cbx = bb(c)
        if ca is None:
            log("   [반출]   !! %s 를 잃었다" % c.rsplit("/", 1)[-1])
            slip_mm = max(slip_mm, 9999.0)
            continue
        r1 = [(ca[i] + cbx[i]) / 2.0 - (a[i] + b[i]) / 2.0 for i in range(3)]
        exp = [r0[0] * _c - r0[1] * _s, r0[0] * _s + r0[1] * _c, r0[2]]
        dmm = [(r1[i] - exp[i]) * 1000.0 for i in range(3)]
        n = max(abs(v) for v in dmm)
        slip_mm = max(slip_mm, n)
        log("   [반출]   %s 팔레트 대비 이동 [%+.1f %+.1f %+.1f] mm  %s"
            % (c.rsplit("/", 1)[-1], dmm[0], dmm[1], dmm[2],
               "안착" if n <= SHIP_SLIP_TOL_MM else "!! 미끄러짐"))
    return {"min_m": a, "max_m": b, "reached": bool(ok1 and ok2), "bed_x": x_bed,
            "slip_mm": slip_mm, "cargo": len(rel0)}


_PKM_LOCK = threading.Lock()
_PKM_CACHE = {}


def _packer_module():
    """패킹 모듈을 **한 번만** 동적 로드한다.

    ★ 워커 3 개가 동시에 `D._load_packer_module()` 을 부르면 두 번째·세 번째가
      **반쯤 초기화된 모듈**을 받는다 (실측 2026-08-25:
      `AttributeError: module '_packing_hm' has no attribute 'HeightmapPacker'`
      로 존 A·B 워커가 시작하자마자 죽었다). 락으로 감싸고 결과를 캐시한다.
    """
    with _PKM_LOCK:
        if "m" not in _PKM_CACHE:
            _PKM_CACHE["m"] = D._load_packer_module()
        return _PKM_CACHE["m"]


class ZoneLink(object):
    """요청에 `zone` 을 **자동으로** 채워 주는 `SimLink` 껍데기.

    ★ 이게 있어서 `cycle.py` 를 한 줄도 안 고치고 로봇 3 대를 돌린다. 사이클이
      보내는 모든 명령에 자기 존이 실려 가고, 서버 쪽 디스패처가 그 존의
      `RobotMotion` 으로 넘긴다. 이미 zone 을 넣어 보내는 명령(`pickup_check`)은
      `setdefault` 라 덮이지 않는다.
    """

    def __init__(self, link, zone):
        self._l = link
        self._z = zone

    def call(self, **kw):
        kw.setdefault("zone", self._z)
        return self._l.call(**kw)

    def call_slow(self, budget_s, **kw):
        kw.setdefault("zone", self._z)
        return self._l.call_slow(budget_s, **kw)

    def __getattr__(self, n):
        return getattr(self._l, n)


def worker(specs, ids, zone=None):
    """머리 쪽. 소켓으로만 말한다. `zone` 을 주면 그 로봇만 돌린다."""
    import cycle as CY
    import sim_link as SL
    _S["CY"] = CY                                        # report() 가 좌표 변환에 쓴다
    zone = (zone or ZONE).upper()

    try:
        # ★ 스레드마다 소켓을 따로 판다. `SimLink` 는 스레드 안전이 아니다.
        link = ZoneLink(SL.SimLink(port=PORT), zone) if MULTI \
            else SL.SimLink(port=PORT)
        _S.setdefault("links", {})[zone] = link
        _S["link"] = link

        by_id = {bid: LinkPrim(link, bid) for bid in ids}
        hm, pal = cfg["heightmap"], cfg["pallet"]
        PKM = _packer_module()

        class ArmAwarePacker(PKM.HeightmapPacker):
            """기하만 아는 패커에 **팔 사정**을 술어로 얹는다 (위 상수 주석 참고)."""

            def _sweep_risk(self, cx, cy, z, w, d, h, yaw):
                """이 자리로 **가는 길**에 넘어야 할 기둥이 너무 높은가."""
                cols = [(q.center_local[0], q.center_local[1], (q.z + q.h) * 1000.0)
                        for q in self.placements]
                return _near_worst_mm(cx, cy, cols) > NEAR_LIM_MM

            def _builds_trap(self, cx, cy, z, w, d, h, yaw):
                """이 자리에 놓으면 **나중에** 그 뒤로 못 가게 되는가.

                ★ 이게 더 중요하다. 함정은 빠진 뒤가 아니라 빠지기 전에 막아야 한다.
                  실측 2026-08-23 (stack15j): 12번째가 근거리 (695,205) 에 5호를
                  올려 상단 1020 mm 인 기둥을 세웠다. 그러자 13번째가 갈 수 있는 far
                  자리가 **하나도 안 남았고**, 제약을 풀고 놓았더니 아래팔이 그 위를
                  쓸어 far 쪽 4단 탑이 통째로 무너졌다 (knock 1401 mm · 박스 5개가
                  팔레트 밖으로). 그 한 수를 안 두게 하는 것이 답이다.
                  근거리 = 팔레트 앞 절반 (견본 `build_plan` 의 `y_mm < 400` 과 동일).
                """
                return cy * 1000.0 < NEAR_HALF_MM and (z + h) * 1000.0 > NEAR_LIM_MM

            def _far_pillar_risk(self, cx, cy, z, w, d, h, yaw):
                """근거리에 깊게 내려앉는데 그 너머 far 기둥이 너무 높은가 (제약 b)."""
                if cy * 1000.0 >= NEAR_HALF_MM:
                    return False                          # 근거리 배치가 아니다
                cols = [(q.center_local[0], q.center_local[1], (q.z + q.h) * 1000.0)
                        for q in self.placements]
                return _far_worst_mm(cx, cy, cols) > NEAR_LIM_MM

            def find_placement(self, box_size, box_mass=0.0, **kw):
                if not ARM_GUARD or kw.get("reject") is not None:
                    return super().find_placement(box_size, box_mass, **kw)

                # 1) 길도 막히지 않고, 함정도 안 만들고, 팔꿈치도 안 스치는 자리
                p = super().find_placement(
                    box_size, box_mass,
                    reject=lambda *a: (self._sweep_risk(*a) or self._builds_trap(*a)
                                       or self._far_pillar_risk(*a)))
                if p is not None:
                    return p
                # 2) 함정은 감수하되 지금 길은 열려 있는 자리.
                #    ★ 기본은 **쓰지 않는다.** 실측 2026-08-23 (stack15k): 이 완화가
                #      12·13번째를 근거리 z1020(상단 1300) 에 올렸고, 그 깊은 하강에서
                #      팔꿈치가 옆 기둥을 77.9 mm 밀었다 — 견본이 말한 시나리오 (b) 다.
                #      팔레트를 한 판 더 쓰는 편이 스택을 흔드는 것보다 싸다.
                #      비교 실험용으로만 `PL_ARM_GUARD=soft` 로 연다.
                if ARM_GUARD_SOFT:
                    p = super().find_placement(box_size, box_mass,
                                               reject=self._sweep_risk)
                    if p is not None:
                        log("            [svc] 근거리 기둥이 %.0f mm 를 넘게 된다 —"
                            " 다음 박스가 그 뒤로 못 갈 수 있다 (soft)" % NEAR_LIM_MM)
                        return p
                # 3) 길이 막혔다. **제약을 풀지 않는다.** 여기서 억지로 놓으면 팔이
                #    기둥 위를 쓸어 스택이 무너진다 — knock 몇 mm 가 아니라 판이 엎어진다.
                #    사양이 준 길로 간다: PALLET_FULL -> 팔레트 교체 (설계결정서 §7,
                #    "버퍼 우회는 정상적인 흐름 제어이지 실패가 아님").
                log("            [svc] !! 간섭 없이 갈 수 있는 자리가 없다 — 이 팔레트는"
                    " 여기까지다 (한계 %.0f mm)" % NEAR_LIM_MM)
                return None

        def factory():
            return ArmAwarePacker(
                size_xy=(pal["width_mm"] / 1000.0, pal["depth_mm"] / 1000.0),
                max_height=pal["max_stack_mm"] / 1000.0,
                cell=hm["cell_mm"] / 1000.0,
                yaws_deg=tuple(float(v) for v in hm["allowed_yaw_deg"]),
                min_support_ratio=float(hm["min_support_ratio"]),
                support_tol=hm["support_tol_mm"] / 1000.0,
                wall_margin=0.0, enforce_load_order=True,
                descent_halo=D.DESCENT_HALO_MM / 1000.0)

        origin_mm = ORIGIN_BY_ZONE.get(zone, ORIGIN_MM)
        vision = D.PalletVision(by_id, specs, origin_mm)
        svc = LineServices(link, specs, factory, vision, D.CLEARANCE_MM,
                           log=lambda *a: log("           ", *a), verbose=VERBOSE)
        cyc = CY.Cycle(link, svc, cfg, zone=zone, log=lambda *a: log("   [cycle] %s" % zone, *a))

        # 라인을 돌린다. 이후 정지/재가동은 `cycle.py` 가 <존>_pickup 만 만진다.
        # ★ 존 이름을 붙인다. 예전엔 "b_pickup" 이 박혀 있어서 PL_ZONE=A 로 돌리면
        #   `cycle.py` 는 a_pickup 을 끄고 켜는데 여기서 켠 것은 b_pickup 이라
        #   **지선이 영영 안 돌았다** (실측 2026-08-24: 박스가 스폰 자리에 150 s 정지).
        if zone == ZONES[0]:
            link.call(cmd="set_velocity", seg="main", mps=BELT_MPS)
            _S["main_on"] = True
            # ★ 순환 루프와 코너 커브도 같이 돌린다 (2026-08-25).
            #   루프는 분기점을 지나친 박스가 돌아오는 길이다 — 안 켜면 그 박스가
            #   라인 끝에 쌓여 뒤를 막는다. 커브는 셀과 합류를 잇는 조각이라
            #   안 켜면 그 구간이 죽은 벨트로 보인다.
            #   ⚠ 여기 안 적힌 seg 는 **아무도 안 켠다.** 설정에 seg 를 새로
            #     만들었으면 이 줄에도 넣어야 한다.
            # ★ 루프는 여기서 안 켠다 — `conveyor_ctl` 이 시작할 때 원시값으로
            #   써넣는다 (`belt_velocity_raw`). 배율 환산이 그 조각에서 안 맞는다.
            for _seg in ("a_curve", "b_curve", "c_curve"):
                _r = link.call(cmd="set_velocity", seg=_seg, mps=BELT_MPS)
                if not _r.get("ok"):
                    log("   [벨트] %s 못 켰다 — %s" % (_seg, _r.get("error")))
            # ★ 소터도 기동 때 켠다 (2026-08-26). `turn()` 은 박스가 분기점에
            #   닿아야 불리는데, 그 전에 지나가는 박스는 **선 소터에 막힌다.**
            #   `conveyor_ctl.run_all()` 이 하는 것과 같다 — 방향은 통과.
            #   실측 근거(김병직): PL_ZONES=A,B,C 로 60 s 뒤 박스 4 개가
            #   4.6 m 에서 1.6 m 로 뭉쳤다. 느려진 게 아니라 존 A 분기에서
            #   막혀 뒤가 밀린 것이다 (간격이 좁아지는 것이 근거다).
            #   ★ `ZONES` 가 아니라 A·B·C 전부다. 존 A 만 돌려도 분기를 놓친
            #     박스는 순환 루프로 가느라 **B·C 소터 위를 지나간다.**
            for _z in ("A", "B", "C"):
                _r = link.call(cmd="set_sorter", zone=_z, divert=False,
                               mps=BELT_MPS)
                if not _r.get("ok"):
                    log("   [소터] %s 못 켰다 — %s" % (_z, _r.get("error")))
        if ROUTE_ON:
            # 라우팅은 박스를 세 지선으로 나눠 보내므로 셋 다 돌아야 한다.
            for _z in sorted(set(_route_map()[0].values())):
                link.call(cmd="set_velocity", seg="%s_pickup" % _z.lower(),
                          mps=BELT_MPS)
        else:
            link.call(cmd="set_velocity", seg="%s_pickup" % zone.lower(),
                      mps=BELT_MPS)
        # ★ 투입·계량·라우팅은 **라인 전체에 하나씩**이다. 워커마다 띄우면 로봇
        #   3 대일 때 스포너가 3 개가 되어 박스가 3 배로 쏟아진다.
        #   첫 존이 대표로 띄운다.
        if zone == ZONES[0]:
            threading.Thread(target=spawner_thread, args=(specs, ids),
                             daemon=True, name="spawn").start()
            if METER_MAX > 0:
                threading.Thread(target=meter_thread, daemon=True,
                                 name="meter").start()
            if ROUTE_ON:
                threading.Thread(target=router_thread, args=(specs, ids),
                                 daemon=True, name="route").start()

        def recover():
            """실패한 사이클 뒤처리 — **치우거나 다시 하기 전에 반드시 부른다.**

            ★★ 그리퍼부터 연다. `cycle.py` 의 PICK_FAILED 경로는 `_abort` 를 안 거쳐
              그리퍼를 안 연다 (B 파일이라 안 고친다). 흡착이 `Closing`/`Closed` 이고
              물린 목록에 박스가 든 채로 남는데, 그 상태로 despawn 하면
              `Spawner._despawn` 이 박스를 라인 밖으로 순간이동시키면서
              **그리퍼가 붙잡은 채로 팔이 같이 끌려간다.** 실측 2026-08-23: 관절
              하나가 188 rad(10817 deg) 감겨 이후 모든 이동이 예산을 넘겼고, 팔이
              휘저으며 놓인 박스를 86 mm 밀고 벨트 위 박스를 튕겨냈다.
              열린 그리퍼에 release 를 걸어도 무해하니 무조건 연다.
            """
            rl = link.call_slow(cyc.release_budget_s, cmd="release")
            if not rl.get("ok"):
                log("       !! release %r (계속 진행한다)" % rl)

            # ★★ 관절을 **읽는 것도** 메인 스레드여야 한다. 여기는 워커 스레드다.
            #   쓰기만 큐로 보내고 읽기를 직접 하면, PR #9 가 없앤 바로 그 경합이
            #   되살아난다 (`robot_motion.py`: 물리 스텝 중 교차 접근은 undefined,
            #   증상은 `copyInternalStateToCache` 에러 홍수). 게다가 조용히 틀린다 —
            #   깨진 값을 읽으면 감김 판정이 오작동해서 **멀쩡한 팔에 READY
            #   순간이동을 걸고**, 그 순간이동 자체가 계단 입력이라 없던 사고를 만든다.
            #   읽기·판정·되돌리기를 한 덩어리로 메인 스레드에 보낸다 — 그래야 그
            #   사이에 물리가 끼어들지도 않는다.
            def _unwind():
                _art = (_S.get("arms") or {}).get(zone, {}).get("art") or _S["art"]
                q6 = np.asarray(_art.get_joint_positions()[:6], dtype=float)
                if float(np.max(np.abs(q6))) <= 2.0 * math.pi:
                    return {"ok": True, "wound": False}
                _art.set_joint_positions(_S["ready_q"])
                _art.set_joint_velocities(np.zeros(6))
                return {"ok": True, "wound": True,
                        "deg": np.round(np.degrees(q6), 0).tolist()}

            r = _S["q"].run(_unwind)
            if r.get("wound"):
                log("       !! 관절이 감겼다 %s deg — READY 로 되돌렸다" % r["deg"])
                time.sleep(1.0)                          # 드라이브가 따라올 시간
            elif not r.get("ok"):
                log("       !! 감김 확인 실패: %r" % r)

        rest = {}                                        # 놓인 뒤의 기준 자세 (knock 용)
        pick_retry = {}                                  # box_id -> 흡착 재시도 횟수
        on_pallet = []                                   # 지금 팔레트에 선 기둥 (팔 간섭 진단용)
        swapped_for = set()                              # 이 박스 때문에 이미 교체했다
        pallets = 1
        t_all = time.time()
        i = 0
        targets = (_S.get("targets_by_zone") or {}).get(zone) \
            or _S.get("targets") or list(ids)
        while i < len(targets) and not _S["done"]:
            # ── 다음 박스가 스토퍼에 설 때까지 ──────────────
            pick, t0, nag = None, time.time(), 0.0
            skipped = False
            t_scan = 0.0
            while time.time() - t0 < ARRIVE_TIMEOUT_S and not _S["done"]:
                c = link.call(cmd="pickup_check", zone=zone)
                if c.get("present"):
                    # ★ 이 박스가 **이 존 것인가.** 없으면 오분류가 조용히
                    #   정상 처리된다 (`_zone_of` 주석의 BOX00026 사례).
                    #   치우는 방식은 바로 아래 STUCK 과 같다 — 남겨 두면 뒤차가
                    #   밀려 라인이 굳는다. 세는 것이 중요하다: 지표에 안 남으면
                    #   "적재 22 개" 가 무엇을 담고 있는지 알 수 없다.
                    _bid = c.get("box_id")
                    _want = _zone_of(specs, _bid)
                    #   ⚠ 한 번만 센다. `despawn_box` 는 메인 스레드 큐를 거치므로
                    #     반영 전에 `pickup_check` 가 같은 박스를 몇 번 더 준다 —
                    #     그대로 두면 지표에 같은 박스가 여러 줄 쌓인다.
                    if _want and _want != zone and _bid in _MISROUTED:
                        continue
                    if _want and _want != zone:
                        _MISROUTED.add(_bid)
                        log("   !! %s 는 %s 행(%s)인데 존 %s 지선에 올라왔다 — "
                            "치우고 계속한다 (소터가 앞 박스 때문에 열려 있었을 것)"
                            % (_bid, _want,
                               specs.get(_bid, {}).get("dest", "?"), zone))
                        link.call(cmd="despawn_box", box_id=_bid)
                        _S["rows"].append(
                            {"id": _bid,
                             "type": specs.get(_bid, {}).get("name", "?"),
                             "res": "MISROUTE", "err": (float("nan"),) * 3,
                             "wall": 0.0, "wait": time.time() - t0,
                             "knock": 0.0, "who": "-",
                             "kvec": (0.0, 0.0, 0.0),
                             "near_mm": 0.0, "far_mm": 0.0, "place": None})
                        continue
                    pick = c
                    _STUCK_SINCE.clear()
                    break
                # ★ 낀 박스 탈출로. 픽업 구간에 박스가 **있는데** present 가 안 되면
                #   그 박스는 기울었거나 물리가 틀어진 것이다. 지우고 계속 간다.
                #   안 지우면 뒤차가 계속 밀려 라인이 통째로 굳는다.
                #
                #   ⚠ 타이머는 **그 박스가 구간에 머문 시간**이다. 처음에는 "작업자가
                #     기다린 시간" 으로 쟀는데, 그러면 앞 박스를 오래 기다린 뒤 막
                #     들어온 멀쩡한 박스가 즉시 지워진다 (실측 2026-08-25: BOX00008 이
                #     진입 7 s 만에 지워졌다). 지선을 3 m 달리는 데만 6 s 가 걸린다.
                now = time.time()
                if now - t_scan >= 1.0:
                    t_scan = now
                    sb = _stuck_box(link, ids)
                    if sb is None:
                        _STUCK_SINCE.clear()
                    else:
                        sid, spos = sb
                        for k in [k for k in _STUCK_SINCE if k != sid]:
                            del _STUCK_SINCE[k]
                        held = now - _STUCK_SINCE.setdefault(sid, now)
                        if held > STUCK_S:
                            del _STUCK_SINCE[sid]
                            rr = link.call(cmd="despawn_box", box_id=sid)
                            log("   !! %s 가 픽업 구간(%s %.2f m)에 %.0f s 째 "
                                "그대로인데 집을 수 없다 — 지우고 계속한다 (%s)"
                                % (sid, "xyz"[_PU_AX], spos[_PU_AX], held,
                                   "ok" if rr.get("ok") else rr))
                            _S["rows"].append(
                                {"id": sid,
                                 "type": specs.get(sid, {}).get("name", "?"),
                                 "res": "STUCK", "err": (float("nan"),) * 3,
                                 "wall": held, "wait": now - t0,
                                 "knock": 0.0, "who": "-",
                                 "kvec": (0.0, 0.0, 0.0),
                                 "near_mm": 0.0, "far_mm": 0.0, "place": None})
                            if sid in targets:
                                # 이 존 몫이었다. 처리한 것으로 세고 넘어간다.
                                i += 1
                                skipped = True
                                break
                # 안 오면 어디에 있는지 주기적으로 찍는다. 이게 없으면 타임아웃
                # 났을 때 "안 왔다" 는 사실만 남고 원인을 못 찾는다
                # (2026-08-23 15번째가 180 s 동안 안 와서 겪었다).
                if time.time() - t0 > nag + 15.0:
                    nag = time.time() - t0
                    log("   ...대기 %.0f s — %s" % (nag, _line_state(link, ids)))
                time.sleep(0.05)
            if skipped:
                continue
            if pick is None:
                log("   !! %d/%d 번째 박스가 %.0f s 안에 스토퍼에 안 왔다 — 중단"
                    % (i + 1, len(targets), ARRIVE_TIMEOUT_S))
                log("      마지막 라인 상태: %s" % _line_state(link, ids))
                _S["stalled"] = True
                break
            bid = pick["box_id"]
            t = specs[bid]
            _CUR["box"] = (bid, t["h_mm"] / 2000.0)      # gap_fn 이 읽는다
            _CUR_Z[zone] = _CUR["box"]
            wait_s = time.time() - t0

            log("\n[%2d/%d] %-9s %-3s %s mm %.0f kg   대기 %.1f s   앞면 %.4f m"
                % (i + 1, len(targets), bid, t["name"],
                   [t["w_mm"], t["d_mm"], t["h_mm"]], t["mass_g"] / 1000.0, wait_s,
                   pick["pose"]["pos_m"][0] + t["w_mm"] / 2000.0))

            # ── 카메라가 같은 것을 본다 ────────────────────
            # ★ `pickup_check` 는 씬 프림에서 자세를 그대로 읽는다 (ground truth).
            #   실물에는 그런 게 없다. 카메라가 같은 질문에 답하게 하고 **나란히**
            #   찍어 인식 오차를 남긴다. 지금은 판단에 쓰지 않는다 — 계측만 한다.
            if CAM:
                cm = link.call(cmd="camera_detect")
                if cm.get("present"):
                    gt = pick["pose"]["pos_m"]
                    dx = cm["top_center_mm"][0] - gt[0] * 1000.0
                    dy = cm["top_center_mm"][1] - gt[1] * 1000.0
                    dz = cm["top_center_mm"][2] - (gt[2] * 1000.0 + t["h_mm"] / 2.0)
                    log("        [카메라] %s %s mm yaw %+.1f  점 %d  채움 %.0f%%"
                        "   실제와 차이 [%+.1f %+.1f %+.1f] mm"
                        % (cm["type"], cm["size_mm"], cm["yaw_deg"],
                           cm["n_pts"], cm["fill"] * 100, dx, dy, dz))
                    _S.setdefault("cam_err", []).append(
                        (bid, t["name"], cm["type"], dx, dy, dz, cm["size_mm"]))
                else:
                    log("        [카메라] 못 봤다 — %s" % cm.get("error"))

            # ── 사이클 한 바퀴 — 실물 cycle.py 그대로 ───────
            tc = time.time()
            # 메인 스레드가 감시한다 (on_step). ★ 존별로 담는다 — 로봇이 여러
            # 대면 한 리스트에 덮어써서 남의 사이클 시간을 재게 된다.
            _S.setdefault("cyc", {})[zone] = (tc, bid)
            _S["cyc_t0"], _S["cyc_id"] = tc, bid
            res = cyc.run(bid, pick["pose"])
            wall = time.time() - tc
            _S["cyc_t0"] = None
            _S.setdefault("cyc", {}).pop(zone, None)
            _CUR["box"] = None                           # 사이클 밖에서 옛 박스로 재지 않는다
            _CUR_Z[zone] = None

            # ── 놓은 뒤 정착 ────────────────────────────────
            # ★ B5 견본(`demo_stack15.main`)은 사이클이 끝난 뒤 SETTLE_S 만큼 물리를
            #   더 돌린 **다음에** 잰다. 여기서는 그게 빠져 있어서 흔들리는 도중을
            #   찍고 있었다. 흡착을 떼는 순간 매달린 박스가 진동하는데(패드 rotXY
            #   드라이브 강성 100 · 감쇠 0), TCP 정지 판정은 **툴** 속도라 그걸 못 본다
            #   — `gripper.py` 의 RELEASE_MIN_HOLD_S 주석이 같은 얘기다.
            #   실측 2026-08-23: 3번 5호가 |dq| 177 deg 로 크게 돌아온 뒤 놓였고,
            #   즉시 재니 y 오차 -76.9 mm 였다. 사양 판정선(30 mm)을 넘는 값이라
            #   무엇을 재는지가 결과를 가른다. 물리는 메인 스레드가 계속 돌리므로
            #   여기서는 기다리기만 하면 된다.
            time.sleep(SETTLE_S)

            # ── 결과 · knock ────────────────────────────────
            # ★ `svc.last` 는 **성공한 계획**만 갱신된다. PALLET_FULL 이면 직전 박스의
            #   계획이 그대로 남아 있어서, 그걸 목표로 쓰면 오차가 -1130 mm 같은
            #   헛것이 찍힌다 (2026-08-23 첫 실행에서 실제로 그랬다).
            place = dict(svc.last) if (res == "PLACED" and svc.last) else None
            err = np.array([float("nan")] * 3)
            if place is not None:
                want = np.asarray(CY.pallet_place_to_world(place, t["h_mm"], origin_mm))
                got = by_id[bid].get_world_pose()[0]
                err = (got - want) * 1000.0
            knock, who, kvec = 0.0, "-", np.zeros(3)
            for pid, ref in rest.items():
                dv = (by_id[pid].get_world_pose()[0] - ref) * 1000.0
                dmm = float(np.linalg.norm(dv))
                if dmm > knock:
                    knock, who, kvec = dmm, pid, dv

            # ★ 이 자리로 가려면 로봇 쪽 기둥을 넘어야 했는가. 넘었다면 아래팔이
            #   그 위를 쓸고 지나갔다는 뜻이고, 그게 knock 의 코드상 원인이다.
            #   게이트에는 안 넣는다 — 상관을 먼저 실측으로 확인한다.
            near_mm = far_mm = 0.0
            if place is not None:
                _cols = [(q["x_mm"] / 1000.0, q["y_mm"] / 1000.0,
                          q["z_mm"] + BOX_H_MM[q["type"]]) for q in on_pallet]
                _cx, _cy = place["x_mm"] / 1000.0, place["y_mm"] / 1000.0
                near_mm = _near_worst_mm(_cx, _cy, _cols)
                far_mm = _far_worst_mm(_cx, _cy, _cols) if _cy * 1000.0 < NEAR_HALF_MM else 0.0
                if near_mm > NEAR_LIM_MM:
                    log("       !! 근거리 기둥 %.0f mm > 한계 %.0f (ped %.0f) — "
                        "아래팔이 그 위를 쓴다" % (near_mm, NEAR_LIM_MM, PED_MM))
                if far_mm > NEAR_LIM_MM:
                    log("       !! far 기둥 %.0f mm > 한계 %.0f — 근거리 깊은 하강에서 "
                        "팔꿈치가 그것을 민다" % (far_mm, NEAR_LIM_MM))

            if res == "PLACED":
                rest[bid] = by_id[bid].get_world_pose()[0].copy()
                if place is not None:
                    on_pallet.append(dict(place, type=t["name"], id=bid))

            _S["rows"].append({"zone": zone,
                               "id": bid, "type": t["name"], "res": res, "err": err,
                               "wall": wall, "wait": wait_s, "knock": knock, "who": who,
                               "kvec": kvec, "near_mm": near_mm, "far_mm": far_mm,
                               "place": place})
            log("       %-11s 오차 [%+.1f %+.1f %+.1f] mm   %.1f s   knock %.1f mm (%s)"
                % (res, err[0], err[1], err[2], wall, knock, who))

            # ★ 팔레트 1 장에 정해진 수만큼 놓았으면 트럭으로 내보내고 끝낸다.
            if PER_PALLET > 0 and res == "PLACED":
                _done_n = len([w for w in _S["rows"]
                               if w["res"] == "PLACED"
                               and w.get("zone", zone) == zone])
                if _done_n >= PER_PALLET:
                    log("   [반출] 팔레트에 %d 개 — 트럭으로 보낸다" % _done_n)
                    _cargo = [w["id"] for w in _S["rows"]
                              if w["res"] == "PLACED"
                              and w.get("zone", zone) == zone]
                    _S.setdefault("ships", {})[zone] = ship_pallet(
                        link, zone, log, _cargo)
                    _S["ship"] = _S["ships"][zone]
                    break

            # ── PALLET_FULL 은 **팔레트를 갈아서** 푼다 (사양 경로) ──
            # 트랙B 가이드 상태도: "PALLET_FULL: /sim/set_spawn_enabled(false)
            #   -> swap_pallet -> /algo/reset -> 재가동".
            # 이 박스는 아직 라인 위에 그대로 있다(스토퍼가 잡고 있다). 빈 팔레트를
            # 받은 뒤 **같은 박스를 다시** 집는다. 한 박스당 한 번만 교체한다 —
            # 빈 팔레트에도 안 들어가면 그건 팔레트 문제가 아니라 박스 문제다.
            if res == "PALLET_FULL" and bid not in swapped_for:
                swapped_for.add(bid)
                _S["rows"].pop()                     # 이건 최종 결과가 아니다
                log("       팔레트가 찼다 — 사양 경로로 교체한다 "
                    "(스폰 정지 -> swap_pallet -> algo reset -> 재가동)")
                link.call(cmd="set_spawn_enabled", enabled=False)
                sw = link.call(cmd="swap_pallet", zone=zone)
                svc.reset_pallet()
                link.call(cmd="set_spawn_enabled", enabled=True)
                if not sw.get("ok"):
                    log("       !! swap_pallet 실패: %r" % sw)
                else:
                    pallets += 1
                    log("       팔레트 #%d — 박스 %d 개를 내보냈다. %s 를 다시 집는다."
                        % (pallets, sw.get("removed", 0), bid))
                rest.clear()                          # 팔레트가 비었으니 knock 기준도 리셋
                on_pallet.clear()
                continue                              # i 를 안 올린다 = 같은 박스 재시도

            # ── 흡착 실패는 **다시 해 본다** (사양: 실패는 재시도 대상) ──
            # ★ 간극 허용치가 얇다. `cycle._descend_to_gap` 은 2.0 mm 를 목표로
            #   닫고 `gripper.GRIP_TOL_MM` 은 3.0 mm 라 여유가 1.0 mm 뿐이다.
            #   게다가 두 쪽이 재는 간극이 다르다 — cycle 은 **집기 시작 시점**의
            #   박스 상면(pick_pose)을 기준으로 수직 거리를, gripper 는 **지금**
            #   박스 중심에서 툴 +Z 축에 투영한 값을 쓴다. 그 사이 박스가 1 mm 만
            #   가라앉아도 판정이 갈린다. 실측 2026-08-23: 3.23 mm 로 0.23 mm
            #   차이에 거절됐다 (results/stack15_volume_fail_20260823.log).
            #   박스는 아직 스토퍼에 서 있으니 버릴 이유가 없다. 다시 내려간다.
            if res == "PICK_FAILED" and pick_retry.get(bid, 0) < PICK_RETRY:
                pick_retry[bid] = pick_retry.get(bid, 0) + 1
                _S["rows"].pop()                     # 이건 최종 결과가 아니다
                log("       흡착 실패 — 다시 해 본다 (%d/%d)"
                    % (pick_retry[bid], PICK_RETRY))
                recover()
                # 실패한 흡착이 박스를 접근 높이까지 들었다 놨을 수 있다. 가라앉기를 기다린다.
                time.sleep(SETTLE_S + 0.5)
                continue                              # i 를 안 올린다 = 같은 박스 재시도
            i += 1
            # ★ PALLET_FULL 에서 멈추지 않는다. 큰 박스가 못 들어갔다고 작은 박스도
            #   못 들어가는 게 아니다 — seed 42 에서 14번 5호는 자리가 없지만
            #   15번 3호는 들어간다 (scripts/eval_pack_online.py 로 확인). 예전 판은
            #   여기서 break 해서 멀쩡한 박스를 안 돌렸다.
            #   사양상으로도 이건 실패가 아니다: "버퍼 우회는 정상적인 흐름 제어이지
            #   실패가 아님" (설계결정서 §7 · bypass_count 를 retry_count 와 분리하는 이유).
            #   stage 2 에서는 Return Line 으로 재순환한다. stage 1 은 그 설비가 없어
            #   여기서 라인 밖으로 뺀다.
            # ★ 화이트리스트로 쓰면 안 된다. 실제로 `PICK_FAIL` 이라고 적어 뒀는데
            #   `cycle.py` 가 돌려주는 값은 `PICK_FAILED` 다(cycle.RESULTS). 문자열
            #   비교라 조용히 안 맞아서, 흡착 실패한 박스가 despawn 안 되고 스토퍼에
            #   남는다 — 그러면 뒤 박스가 영영 못 오고 pickup_check 는 같은 박스를
            #   계속 present 로 준다. 성공은 "PLACED" 하나뿐이니 그 반대로 판정한다.
            if res != "PLACED":
                if res == "PALLET_FULL":
                    log("       이 팔레트에 자리가 없다 — 우회 처리하고 다음 박스로 간다.")
                else:
                    log("       !! 실패. 라인에서 빼고 다음 박스로 넘어간다.")
                recover()                            # 그리퍼를 열고, 감겼으면 되돌린다

                # 스토퍼를 막고 있으면 다음 박스가 못 온다. 반드시 치운다.
                # ★ 단, 이탈 경로(`cycle._abort`)는 매달린 박스를 **그 자리에서** 놓는다.
                #   이송이나 배치 하강 중에 MOTION_TIMEOUT / IK_FAIL 이 나면 박스는
                #   적재물 위에 떨어져 있고, release 뒤 후퇴 이동이 타임아웃 나면
                #   **제대로 놓인 박스**가 BLOCKED 로 여기 온다. 그걸 개별 despawn 하면
                #   `Spawner._despawn` 이 프림을 kinematic 으로 바꿔 (-50,-50,-10) 로
                #   순간이동시키므로 그 위·옆 박스가 지지를 잃고 무너진다.
                #   팔레트 위에 있으면 팔레트째 정리한다 (swap_pallet 은 위를 다 치운다).
                pr = link.call(cmd="get_box_pose", box_id=bid)
                fell_on_pallet = False
                if pr.get("ok"):
                    _bx, _by, _bz = pr["pos_m"]
                    _px = (origin_mm[0] / 1000.0,
                           (origin_mm[0] + cfg["pallet"]["width_mm"]) / 1000.0)
                    _py = (origin_mm[1] / 1000.0,
                           (origin_mm[1] + cfg["pallet"]["depth_mm"]) / 1000.0)
                    fell_on_pallet = (_px[0] <= _bx <= _px[1]
                                      and _py[0] <= _by <= _py[1]
                                      and _bz >= origin_mm[2] / 1000.0 - 0.05)
                if fell_on_pallet:
                    log("       !! 팔레트 위에 떨어졌다 (%.3f %.3f %.3f) — 개별 despawn "
                        "대신 팔레트를 갈아낸다" % tuple(pr["pos_m"]))
                    link.call(cmd="set_spawn_enabled", enabled=False)
                    sw = link.call(cmd="swap_pallet", zone=zone)
                    svc.reset_pallet()
                    link.call(cmd="set_spawn_enabled", enabled=True)
                    if sw.get("ok"):
                        pallets += 1
                        log("       팔레트 #%d — 박스 %d 개를 내보냈다."
                            % (pallets, sw.get("removed", 0)))
                    else:
                        log("       !! swap_pallet 실패: %r" % sw)
                    rest.clear()          # 안 지우면 knock 루프가 없는 박스를 읽는다
                    on_pallet.clear()
                else:
                    link.call(cmd="despawn_box", box_id=bid)

        # ★ 마지막에 **물리에서** 한 번 더 잰다. 리포트의 "최상단" 을 계획값으로
        #   찍으면 그 게이트는 절대 안 떨어진다 — 계획은 언제나 한계 안이니까.
        #   내보낸 팔레트의 박스는 없으므로 빠진다(그게 맞다).
        final = {}
        for w in _S["rows"]:
            if w["res"] != "PLACED":
                continue
            try:
                final[w["id"]] = by_id[w["id"]].get_world_pose()[0].copy()
            except Exception:
                pass                                     # 내보낸 팔레트에 있던 것
        _S["final"] = final
        _S["result"] = {"wall": time.time() - t_all, "pallets": pallets}
    except BaseException:
        import traceback
        _tb = traceback.format_exc()
        # ★ **즉시** 찍는다. 존이 여럿이면 마지막 워커가 `_S["result"]` 를 덮어써서
        #   먼저 죽은 존의 traceback 이 통째로 사라진다 (실측 2026-08-25: 존 A·B 가
        #   시작 직후 끝났는데 로그에 아무 흔적이 없었다).
        log("   !! [존 %s] 워커가 죽었다\n%s" % (zone, _tb))
        _S.setdefault("traces", {})[zone] = _tb
        _S["result"] = {"trace": _tb}
    finally:
        # ★ 존이 여럿이면 **전부** 끝나야 시나리오가 끝난다. 한 대가 먼저 끝났다고
        #   done 을 세우면 나머지 로봇의 소켓이 닫혀 traceback 으로 죽는다.
        _S["workers_left"] = _S.get("workers_left", len(ZONES)) - 1
        if _S["workers_left"] <= 0:
            _S["done"] = True
        else:
            log("   [존 %s] 끝. 남은 로봇 %d 대" % (zone, _S["workers_left"]))


def report():
    r = _S.get("result") or {}
    rows = _S["rows"]
    log("\n" + "=" * 78)
    if SEED is not None:
        log("★ 이 실행은 seed %s 다 — 사양 seed 42 의 베이스라인이 아니다." % SEED)
    log("투입 순서     %s%s" % (ORDER,
        "  (부피 내림차순 — B5 데모와 같은 조건)" if ORDER == "volume" else
        "  (뽑힌 그대로 — 실제 라인 조건)"))
    if "trace" in r:
        log("!! 예외\n" + r["trace"])
    log("  #  존 박스       종류  결과         오차 XY / Z [mm]   대기 s  사이클 s  knock mm")
    log("-" * 78)
    for i, w in enumerate(rows, 1):
        exy = float(np.linalg.norm(w["err"][:2]))
        cell = ("%6.1f / %+6.1f" % (exy, w["err"][2])) if np.isfinite(exy) else "     -  /     - "
        log("%3d  %s  %-10s %-4s  %-11s %s   %6.1f   %6.1f   %7.1f"
            % (i, w.get("zone", ZONE), w["id"], w["type"], w["res"], cell,
               w["wait"], w["wall"], w["knock"]))
    log("-" * 78)

    placed = [w for w in rows if w["res"] == "PLACED"]
    bypassed = [w for w in rows if w["res"] == "PALLET_FULL"]
    failed = [w for w in rows if w["res"] not in ("PLACED", "PALLET_FULL")]
    ok = bool(placed)
    # ★ 분모는 **이 존 몫**이다. 라우팅을 켜면 남의 배송지 박스는 다른 지선으로 간다.
    #   팔레트당 개수를 정했으면(PL_PER_PALLET) 그만큼만 놓고 반출하므로 그 수가 분모다.
    # ★ 존이 여럿이면 **존별 목표의 합**이다. 한 존 몫만 세면 3 대가 각자 1 개를
    #   놓았는데 "3 / 1" 처럼 나온다.
    if MULTI:
        _target_n = sum(len(v) for v in (_S.get("targets_by_zone") or {}).values()) or N
    else:
        _target_n = len(_S.get("targets") or []) or N
    if PER_PALLET > 0:
        # 반출을 켜면 로봇 한 대가 PER_PALLET 개만 놓고 끝낸다 -> 존 수만큼.
        _target_n = min(_target_n, PER_PALLET * (len(ZONES) if MULTI else 1))
    log("처리한 것     %d / %d   (놓음 %d · 우회 %d · 실패 %d)"
        % (len(rows), _target_n, len(placed), len(bypassed), len(failed)))
    if bypassed:
        log("우회          %s — 이 팔레트에 자리가 없는 박스다. 설계결정서 §7 상"
            % ", ".join("%s(%s)" % (w["id"], w["type"]) for w in bypassed))
        log("              버퍼 우회는 흐름 제어이지 실패가 아니다(bypass_count).")
    if failed:
        log("!! 실패       %s"
            % ", ".join("%s(%s)=%s" % (w["id"], w["type"], w["res"]) for w in failed))
    if placed:
        zs = sorted({int(w["place"]["z_mm"]) for w in placed if w["place"]})
        tops = [w["place"]["z_mm"] + next(t["h_mm"] for t in cfg["boxes"]["types"]
                                          if t["name"] == w["type"])
                for w in placed if w["place"]]
        log("단            %d 단  (밑면 z %s mm)" % (len(zs), zs))
        log("최상단        %d mm  (한계 %d)" % (max(tops), cfg["pallet"]["max_stack_mm"]))
        exy = [float(np.linalg.norm(w["err"][:2])) for w in placed]
        ez = [abs(float(w["err"][2])) for w in placed]
        kn = max(w["knock"] for w in rows)
        cyc_s = [w["wall"] for w in placed]
        log("배치 오차     XY 최대 %.1f mm · Z 최대 %.1f mm  (허용 %d / %d)"
            % (max(exy), max(ez), D.PLACE_TOL_XY_MM, D.PLACE_TOL_Z_MM))
        kw = max(rows, key=lambda w: w["knock"])
        log("최대 knock    %.1f mm  (경고선 %.0f)   %s 가 %s 를 [%+.0f %+.0f %+.0f] mm"
            % (kn, D.KNOCK_WARN_MM, kw["id"], kw["who"], *kw["kvec"]))
        for _lbl, _k in (("근거리 기둥", "near_mm"), ("far 기둥   ", "far_mm")):
            nb = [w for w in rows if w.get(_k, 0.0) > NEAR_LIM_MM]
            log("%s   위반 %d 사이클 · 최고 %.0f mm  (한계 %.0f @ ped %.0f)%s"
                % (_lbl, len(nb), max([w.get(_k, 0.0) for w in rows] or [0.0]),
                   NEAR_LIM_MM, PED_MM,
                   "" if not nb else "  -> " + ", ".join(w["id"] for w in nb)))
        log("사이클        평균 %.1f s · 최대 %.1f s   (목표 %.0f s · 스폰 간격 %.0f s)"
            % (sum(cyc_s) / len(cyc_s), max(cyc_s),
               cfg["robot"]["cycle_time_target_s"], SPAWN_DT))
        ce = _S.get("cam_err") or []
        if ce:
            # ★ 이름을 반드시 나눠야 한다. 예전 판은 `exy`/`ez` 를 그대로 덮어써서,
            #   카메라를 켜는 순간 **합격 판정이 배치 오차가 아니라 카메라 인식
            #   오차로** 계산됐다. 표에는 진짜 배치 오차가 찍히고 RESULT 만 다른 값을
            #   보므로 로그만 봐서는 어긋난 게 안 보인다. 카메라는 "판단에 안 쓰고
            #   계측만 한다" 가 이 기능의 전제다 (위 CAM 주석).
            cam_exy = [max(abs(e[3]), abs(e[4])) for e in ce]
            cam_ez = [abs(e[5]) for e in ce]
            hit = sum(1 for e in ce if e[1] == e[2])
            log("카메라        %d 회 인식 · 종류 맞춤 %d/%d · 중심 오차 XY 최대 "
                "%.1f mm · 윗면 z 최대 %.1f mm"
                % (len(ce), hit, len(ce), max(cam_exy), max(cam_ez)))
        log("팔레트        %d 판" % r.get("pallets", 1))
        for _z, _s2 in sorted((_S.get("ships") or {}).items()):
            if not _s2:
                continue
            log("반출 %s        팔레트 X %.0f~%.0f · Y %.0f~%.0f mm -> %s · "
                "적재물 %d 개 · 이동 %.1f mm %s"
                % (_z, _s2["min_m"][0] * 1000, _s2["max_m"][0] * 1000,
                   _s2["min_m"][1] * 1000, _s2["max_m"][1] * 1000,
                   "실림" if _s2["reached"] else "미도착",
                   _s2.get("cargo", 0), _s2.get("slip_mm", 0.0),
                   "안착" if _s2.get("slip_mm", 0.0) <= SHIP_SLIP_TOL_MM
                   else "!! 미끄러짐"))
        _sh = _S.get("ship") if not MULTI else None
        if _sh:
            log("반출          팔레트 X %.0f~%.0f · Y %.0f~%.0f mm · 적재함 입구 %.0f "
                "-> %s"
                % (_sh["min_m"][0] * 1000, _sh["max_m"][0] * 1000,
                   _sh["min_m"][1] * 1000, _sh["max_m"][1] * 1000,
                   _sh["bed_x"] * 1000, "완전히 실림" if _sh["reached"] else "미도착"))
            log("적재물        %d 개 · 팔레트 대비 최대 이동 %.1f mm  %s  (허용 %.0f)"
                % (_sh.get("cargo", 0), _sh.get("slip_mm", 0.0),
                   "안착" if _sh.get("slip_mm", 0.0) <= SHIP_SLIP_TOL_MM
                   else "!! 미끄러짐", SHIP_SLIP_TOL_MM))
        log("벽시계        %.0f s" % r.get("wall", 0.0))

        # ── 끝난 뒤 **물리에서 잰** 자리 ────────────────────────────
        # ★ 위의 "단/최상단" 은 계획값이다. 계획은 언제나 한계 안이라 그 게이트는
        #   절대 안 떨어진다. 실제로 어디에 서 있는지는 여기서만 보인다.
        fin = _S.get("final") or {}
        if fin:
            log("-" * 78)
            log("끝난 뒤 실측  (내보낸 팔레트의 박스는 빠진다)")
            log("     박스       종류   계획 P(x,y) z    최종 이동 [mm]   상단 mm")
            top_meas = 0.0
            for w in placed:
                q = fin.get(w["id"])
                if q is None or not w["place"]:
                    continue
                want = np.asarray(_S["CY"].pallet_place_to_world(
                    w["place"], BOX_H_MM[w["type"]],
                    ORIGIN_BY_ZONE.get(w.get("zone", ZONE), ORIGIN_MM)))
                dv = (q - want) * 1000.0
                tmm = (q[2] * 1000.0 + BOX_H_MM[w["type"]] / 2.0
                       - ORIGIN_BY_ZONE.get(w.get("zone", ZONE), ORIGIN_MM)[2])
                top_meas = max(top_meas, tmm)
                log("     %-10s %-4s   P(%4d,%4d) %4d   [%+5.0f %+5.0f %+5.0f]   %5.0f"
                    % (w["id"], w["type"], w["place"]["x_mm"], w["place"]["y_mm"],
                       w["place"]["z_mm"], dv[0], dv[1], dv[2], tmm))
            log("     실측 최상단 %.0f mm  (계획 %d · 한계 %d)"
                % (top_meas, max(tops), cfg["pallet"]["max_stack_mm"]))
            if top_meas > cfg["pallet"]["max_stack_mm"]:
                log("     !! 실측이 한계를 넘었다")
        # 합격 기준. ★ "N 개 전부 놓임" 이 아니다 — 자리가 없어 우회한 것은 사양상
        #   실패가 아니고(설계결정서 §7), 도착 순서로 15 개가 다 들어가는지는 **패커의
        #   상한** 문제라 이 게이트가 물을 것이 아니다 (eval_pack_online.py 가 답한다).
        #   여기가 물어야 하는 것은 "라인이 15 개를 끝까지 처리했는가" 다.
        _tgt = _target_n
        all_handled = len(rows) == _tgt and not failed
        ok = (all_handled
              and max(exy) < D.PLACE_TOL_XY_MM and max(ez) < D.PLACE_TOL_Z_MM
              and kn < D.KNOCK_WARN_MM
              and max(tops) <= cfg["pallet"]["max_stack_mm"])
        for cond, msg in ((all_handled,
                           "%d 개를 끝까지 처리해야 한다 (놓거나 우회. BLOCKED 는 실패)"
                           % _tgt),
                          (max(exy) < D.PLACE_TOL_XY_MM, "XY 오차 %d mm 안" % D.PLACE_TOL_XY_MM),
                          (max(ez) < D.PLACE_TOL_Z_MM, "Z 오차 %d mm 안" % D.PLACE_TOL_Z_MM),
                          (kn < D.KNOCK_WARN_MM, "knock %d mm 안" % D.KNOCK_WARN_MM),
                          (max(tops) <= cfg["pallet"]["max_stack_mm"], "최대 적재 높이 안")):
            if not cond:
                log("  !! %s" % msg)
    log("-" * 78)
    log("RESULT: %s" % ("통과" if ok else "실패"))
    log("=" * 78 + "\n")
    for k in ("link", "spawn_link", "meter_link"):
        try:
            if _S.get(k):
                _S[k].close()
        except Exception:
            pass
    try:
        if _S.get("server"):
            _S["server"].close()
    except Exception:
        pass
    # ★ 여기서 timeline.stop() + post_quit() 만 부르면 **프로세스가 안 죽는다.**
    #   물리 스텝 콜백 안이라 kit 이 그 틱을 못 끝내 종료 이벤트를 못 뽑는다.
    #   근거와 실측은 scripts/_kitexit.py docstring.
    if _S.get("tee"):
        _S["tee"].close()                                # post_quit 전에 flush 해야 한다
    if HOLD:
        # 물리는 계속 돈다 (`on_step` 은 phase=="done" 이라 즉시 빠진다). 창을
        # 직접 닫으면 끝난다.
        print("\nPL_HOLD — 창을 닫지 않는다. 다 보고 창을 직접 닫아라.\n", flush=True)
        return
    _kitexit.shutdown(0 if ok else 1, physx_sub=_S.get("physx_sub"), log=log)


def on_step(dt):
    if _S["phase"] == "done":
        return
    if _S["phase"] == "boot":
        _S["n"] += 1
        if _S["n"] < BOOT_STEPS:
            return
        try:
            setup()
        except BaseException:
            import traceback
            log("!! setup 실패\n" + traceback.format_exc())
            _S["phase"] = "done"
            report()
            return
        _S["phase"] = "settle"
        _S["n"] = 0
        return
    if _S["phase"] == "settle":
        _S["n"] += 1
        # ★ setup() 이 이미 listen 중이다 — 이 창에 들어온 소켓 요청도 살려 둔다.
        #   근거는 verify_b4_test1_1.py 의 같은 자리 주석.
        _S["q"].pump()
        for _a in (_S.get("arms") or {}).values():
            _a["rm"].step()
            _a["art"].apply_action(_S["ready_action"])   # 안 걸면 옛 목표로 튄다
        _S["sp"].step()
        if _S["n"] >= SETTLE_STEPS:
            log("자세      수렴 %s"
                % {_z: np.round(np.degrees(
                    _a["art"].get_joint_positions()[:6]), 1).tolist()
                   for _z, _a in (_S.get("arms") or {}).items()})
            if SEED is not None:
                _c = dict(cfg)
                _c["boxes"] = dict(cfg["boxes"], random_seed=int(SEED))
                fed = D.manifest(_c, N)
                log("★ seed %s 로 덮어썼다 — 사양(42)이 아니다. 베이스라인 아님." % SEED)
            else:
                fed = D.manifest(cfg, N)
            if ORDER == "volume":
                fed = sorted(fed, key=lambda t: -(t["w_mm"] * t["d_mm"] * t["h_mm"]))
                log("★ 투입 순서 = 부피 내림차순 (B5 데모와 같은 조건). "
                    "상자 구성은 사양 seed 그대로다.")
            ids = [BOX_ID_FMT % i for i in range(N)]
            specs = {bid: dict(t) for bid, t in zip(ids, fed)}
            # ★ 배송지는 크기와 **독립**으로 뽑는다 (2026-08-25).
            #   QR 이 배송지를 싣고, 그 값이 목적지 존을 정한다. 그래서 한 존의
            #   팔레트에는 **그 배송지의 3·4·5 호가 섞여** 들어온다 — 그래야 패커가
            #   실제로 일한다 (한 가지 크기만 오면 최적화라고 할 게 없다).
            #   재현성을 위해 박스 구성과 같은 seed 를 쓴다.
            _dests = list(cfg["boxes"].get("dests") or [])
            if _dests:
                import random as _rnd
                _dr = _rnd.Random(int(SEED) if SEED is not None
                                  else int(cfg["boxes"].get("random_seed", 42)))
                for bid in ids:
                    specs[bid]["dest"] = _dr.choice(_dests)
                # ★ 배송지에 맞는 **색**도 같이 칠한다 (2026-08-25).
                #   스폰 카메라가 읽을 대상이다. 색 -> 배송지 판정을 붙이기 전까지는
                #   그림만 바뀌고 라우팅은 그대로 `by_dest` 를 쓴다.
                #   표를 새로 만들지 않고 `by_dest` + `by_color` 를 이어 유도한다:
                #     SEOUL -> (by_dest) A -> (by_color 역방향) red
                _z2c = {}
                for _c, _z in (ROUTE.get("by_color") or {}).items():
                    _z2c.setdefault(str(_z), str(_c))
                _pal = set((cfg["boxes"].get("colors") or {}).keys())
                _d2c = {str(d): _z2c.get(str(z))
                        for d, z in (ROUTE.get("by_dest") or {}).items()
                        if _z2c.get(str(z)) in _pal}
                if _d2c:
                    for bid in ids:
                        _c = _d2c.get(specs[bid].get("dest"))
                        if _c:
                            specs[bid]["color"] = _c
                    log("박스 색      %s (배송지에서 유도)"
                        % ", ".join("%s=%s" % kv for kv in sorted(_d2c.items())))
                else:
                    # 색표가 비었거나 존이 안 맞으면 조용히 넘기지 않는다 —
                    # 전부 기본색이 되어 카메라가 아무것도 못 가른다.
                    log("!! 박스 색을 못 정했다 (boxes.colors %s · by_color %s)"
                        % (sorted(_pal), ROUTE.get("by_color")))
            log("박스      %d 개  %s" % (N,
                ["%s/%s" % (t["name"], specs[b].get("dest", "-"))
                 for b, t in zip(ids, fed)]))
            # ★ 라우팅을 켜면 **이 존 색깔의 박스만** 이 로봇의 몫이다.
            #   나머지는 다른 지선으로 가서 그 존의 로봇을 기다린다 (1 존만 돌리는
            #   실행에서는 지선에 그대로 선다). 목표 개수를 N 으로 두면 남의 박스를
            #   기다리다 늘 실패로 끝난다.
            _map, _key = _route_map()
            _tz = {}
            for _z in ZONES:
                if ROUTE_ON and _map:
                    _tz[_z] = [b for b in ids if _map.get(_key(specs[b])) == _z]
                    log("존 %s 몫  %d 개 (%s)  · 나머지 %d 개"
                        % (_z, len(_tz[_z]),
                           ",".join(k for k, v in sorted(_map.items()) if v == _z),
                           len(ids) - len(_tz[_z])))
                else:
                    _tz[_z] = list(ids)
            _S["targets_by_zone"] = _tz
            _S["targets"] = _tz[ZONES[0]]                # 단일 존 하위 호환
            log("-" * 78)
            _S["phase"] = "run"
            _S["t0"] = time.time()
            _S["workers_left"] = len(ZONES)
            # ★ 존마다 워커 한 벌. 각자 자기 소켓·자기 팔·자기 팔레트를 쓴다.
            for _z in ZONES:
                threading.Thread(target=worker, args=(specs, ids, _z),
                                 daemon=True, name="stack15_%s" % _z).start()
        return
    _S["q"].pump()
    # ★ world.step() 앞, 메인 스레드. 팔이 여러 대면 전부 한 스텝씩 돌린다.
    for _a in (_S.get("arms") or {}).values():
        _a["rm"].step()
    _S["sp"].step()
    # ★ 사이클 하나가 굳으면 워치독(900 s)까지 아무도 안 끊는다. CYCLE_WALL_S 를
    #   선언만 해 두고 안 쓰고 있었다. 여기서 실제로 쓴다 — 굳은 사이클은 그 자리에서
    #   끊고 그때까지의 결과라도 남긴다.
    # ★ 존별로 본다. 한 대가 멈췄다고 전체를 끝내지 않는다 — 다른 로봇은 계속 돈다.
    _now = time.time()
    for _z, _c in list((_S.get("cyc") or {}).items()):
        if _now - _c[0] > CYCLE_WALL_S:
            log("   !! [존 %s] %s 사이클이 %.0f s 를 넘겼다"
                % (_z, _c[1], CYCLE_WALL_S))
            _S.setdefault("cyc", {}).pop(_z, None)
            _S["stalled"] = True
    if _S["done"] or time.time() - _S["t0"] > WATCHDOG_S:
        # ★ `_S["done"]` 을 먼저 세운다. 워치독(WATCHDOG_S)으로 들어오면 예전에는
        #   이게 False 라 워커·투입·계량 스레드가 계속 돌았고, 그 사이 report() 가
        #   소켓과 서버를 닫아 버려 워커가 닫힌 소켓으로 호출하다 예외로 죽으면서
        #   **이미 나간 리포트의 결과를 traceback 으로 덮어썼다.** rows 를 읽는
        #   report() 와 rows.pop() 을 하는 워커가 겹치기도 한다.
        _S["done"] = True
        _S["phase"] = "done"
        report()


# ── Stop 을 누르면 이 드라이버는 못 이어간다 ──────────────────
# ★ 타임라인을 Stop 하면 Isaac 이 물리 시뮬레이션 뷰를 무효화한다. 이 스크립트는
#   부팅 때 한 번 잡은 `SingleArticulation` · SurfaceGripper 뷰를 매 스텝 쓰므로,
#   Play 를 다시 눌러도 그 핸들이 이미 죽어 있다. 그대로 두면 `rm.step()` 이
#   매 스텝 예외를 뱉어 로그가 에러로 뒤덮이고, 원인이 "Stop 을 눌렀다" 라는 것이
#   그 안에 묻힌다. 재초기화는 아티큘레이션·그리퍼·스포너 스냅샷·패커 상태를
#   전부 되감아야 해서 별건이다 — 지금은 **왜 멈췄는지만 분명히 남긴다.**
try:
    _STOP_EVT = int(omni.timeline.TimelineEventType.STOP)
except Exception:                          # API 이름이 바뀌면 감지를 포기한다
    _STOP_EVT = None                       # (경고를 못 낼 뿐, 동작은 그대로)


def _on_timeline(e):
    if _STOP_EVT is None or e.type != _STOP_EVT:
        return
    if _S["phase"] in ("boot", "done") or _S.get("aborted"):
        return
    _S["aborted"] = True
    _S["done"] = True                      # 워커 스레드도 빠져나온다
    _S["phase"] = "done"                   # on_step 이 죽은 핸들을 안 만진다
    print("\n" + "!" * 78, flush=True)
    print("Stop 을 눌렀다 — 이 드라이버는 여기서 끝난다.", flush=True)
    print("  Play 를 다시 눌러도 **박스가 더 안 나온다.** Stop 이 물리 뷰를", flush=True)
    print("  무효화해서 부팅 때 잡은 로봇·그리퍼 핸들이 죽었고, 재초기화 경로가", flush=True)
    print("  없어서 스폰 루프가 다시 안 돈다. 쌓아 둔 팔레트도 Stop 이 되감아서", flush=True)
    print("  남지 않는다 — 즉 Stop 뒤에 볼 것은 없다.", flush=True)
    print("  다시 보려면 **창을 닫고** 다시 실행해라:", flush=True)
    print("      cd ~/palletris_ws && ./scripts/run_demo.sh", flush=True)
    print("!" * 78 + "\n", flush=True)
    # ★ 박스 정리는 **이 콜백 안에서 하면 안 된다.**
    #   Stop 은 Play 때 잡아 둔 초기 자세를 USD 에 **되써 넣는다.** 여기서 먼저
    #   좌표를 옮겨 봐야 그 되쓰기가 덮어버린다 — 실측 2026-08-24: Stop 뒤 화면에
    #   박스가 스폰 지점에 전부 겹쳐 서 있고, Play 를 누르면 벨트(OmniGraph 라
    #   이 드라이버와 무관하게 돈다)가 그 더미를 실어 날라 "안 나온 박스가 여러 개
    #   쏟아지는" 것처럼 보였다.
    #   그래서 **되쓰기가 끝난 뒤**(다음 앱 업데이트 몇 틱 뒤) 옮긴다.
    #   한 번으로 안 끝날 수 있어 두 번 친다 — park_all 은 멱등이다.
    _pk = {"sub": None, "n": 0}

    def _park_later(_ev):
        _pk["n"] += 1
        if _pk["n"] not in (3, 12):
            if _pk["n"] > 12:
                try:
                    _pk["sub"].unsubscribe()
                except Exception:
                    pass
            return
        sp = _S.get("sp")
        if sp is None:
            return
        try:
            n = sp.park_all()
            if _pk["n"] == 3:
                print("  박스 프림 %d 개를 라인 밖으로 치웠다 "
                      "(Stop 의 자세 되쓰기 뒤에)." % n, flush=True)
        except Exception as _e:
            print("  박스 정리 실패: %r" % (_e,), flush=True)

    _pk["sub"] = (omni.kit.app.get_app().get_update_event_stream()
                  .create_subscription_to_pop(_park_later,
                                              name="palletris-park-after-stop"))
    _S["park_sub"] = _pk["sub"]


_S["timeline_sub"] = (omni.timeline.get_timeline_interface()
                      .get_timeline_event_stream()
                      .create_subscription_to_pop(_on_timeline, name="palletris-stop"))


# ── 물리 스텝 구독 ────────────────────────────────────────
# ★ **Play 뒤에 붙여야 한다.** `subscribe_physics_step_events` 는 그 시점의 물리
#   시뮬레이션에 붙는데, Play 전에는 그 시뮬레이션이 아직 없다. 그래서 정지 상태에서
#   구독해 두면 나중에 Play 를 눌러도 콜백이 안 들어온다 — 이 드라이버는 전부
#   `on_step` 으로 도니까 **아무 일도 안 일어난다.**
#   실측 2026-08-24: 수동 Play 를 넣으면서 이 순서가 깨졌다. 예전 판은 스크립트가
#   `timeline.play()` 를 먼저 부르고 여기까지 앱 업데이트 없이 내려와서 우연히
#   "Play -> 구독" 순서가 지켜지고 있었다. 자동 Play 를 빼자 그 순서가 사라졌다.
#   증상: 씬은 뜨고 Play 를 눌러도 박스가 안 나오고 소켓 서버도 안 열린다.
def _sub_physx(_why):
    if _S.get("physx_sub") is not None:
        return
    _S["physx_sub"] = get_physx_interface().subscribe_physics_step_events(on_step)
    log("물리      스텝 구독 (%s)" % _why)


if AUTOPLAY:
    _sub_physx("자동 Play")
    log("Play 중 — 부팅 대기...\n")
else:
    # 사람이 Play 를 누르는 순간 붙인다. Stop -> Play 로 다시 눌러도 중복 구독은 안 한다.
    try:
        _PLAY_EVT = int(omni.timeline.TimelineEventType.PLAY)
    except Exception:                      # API 이름이 바뀌면 즉시 구독으로 물러선다
        _PLAY_EVT = None
        _sub_physx("PLAY 이벤트를 못 잡아 즉시 구독")

    if _PLAY_EVT is not None:
        def _on_play(e):
            if e.type != _PLAY_EVT:
                return
            _sub_physx("Play 눌림")
            try:
                _S["play_sub"].unsubscribe()   # 한 번이면 된다
            except Exception:
                pass
            _S["play_sub"] = None

        _S["play_sub"] = (omni.timeline.get_timeline_interface()
                          .get_timeline_event_stream()
                          .create_subscription_to_pop(_on_play,
                                                      name="palletris-play"))
    log("Play 를 기다린다...\n")
