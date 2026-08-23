"""15박스 연속 적재 — **실제 씬 `test1-1.usd` 위에서** (트랙 A 통합 시험).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec /home/rokey/palletris_ws/scripts/demo_stack15_test1_1.py

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
ROBOT_PRIM = "/World/robot"
GRIPPER_PRIM = ROBOT_PRIM + "/SurfaceGripper"
EE_FRAME = "tool0"
ZONE = "B"
BOX_ID_FMT = "BOX%05d"
BOOT_STEPS = 20
SETTLE_STEPS = 90                                        # READY 수렴 (1.5 s)
# ★ 지금 집으려는 박스. 워커 스레드가 쓰고 **메인 스레드**(gap_fn)가 읽는다.
#   튜플을 통째로 갈아끼운다 — 키 두 개를 따로 쓰면 새 박스 id 에 옛 높이가
#   붙는 순간이 생긴다.
_CUR = {"box": None}                                     # (box_id, 반높이 m) | None
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
VERBOSE = os.environ.get("PL_VERBOSE") not in (None, "", "0")   # 서비스 호출까지 전부
# ★ 시연용. 결과를 찍은 뒤 **창을 닫지 않는다** — 쌓인 팔레트를 보여줘야 하는데
#   헤드리스 게이트와 같이 즉시 종료해 버리면 볼 것이 없다. GUI 로 띄울 때만 쓴다.
HOLD = _env("PL_HOLD") not in ("", "0", "false", "no")
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
CAM = os.environ.get("PL_CAM", "0") not in ("0", "false", "no")

cfg = _paths.system_config()
STOPPER_X = cfg["conveyor"]["stopper_x_mm"] / 1000.0
BELT_MPS = float(cfg["conveyor"]["belt_velocity_mps"])
ORIGIN_MM = cfg["zones"]["pallet_origin_world_mm"][ZONE]
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
NEAR_LIM_MM = PED_MM - 150.0          # 이보다 높은 근거리 기둥 위는 못 지나간다
NEAR_DY_MM = 50.0                     # "로봇 쪽" 으로 칠 최소 y 차 (견본과 동일)
NEAR_HALF_MM = cfg["pallet"]["depth_mm"] / 2.0   # y < 400 이면 근거리 (견본과 동일)
SECTOR_DEG = 25.0                     # 방위각이 이 안이면 같은 통로다 (견본과 동일)
_AG = os.environ.get("PL_ARM_GUARD", "1")
ARM_GUARD = _AG not in ("0", "false", "no")
ARM_GUARD_SOFT = _AG == "soft"           # 함정을 만드는 자리도 허용 (비교 실험용)


def _az_local(cx_m, cy_m):
    """팔레트 로컬 [m] -> 로봇 베이스에서 본 방위각 [deg]. 견본 `_az` 와 같은 식."""
    return math.degrees(math.atan2(cy_m * 1000.0 + ORIGIN_MM[1] - BASE_MM[1],
                                   cx_m * 1000.0 + ORIGIN_MM[0] - BASE_MM[0]))


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
log("설정      박스 %d · 순서 %s · 팔속도 %s · 계량 %s · 카메라 %s · 창유지 %s"
    % (N, ORDER, SPEED or "사양", METER_MAX or "끔", "켬" if CAM else "끔",
       "켬" if HOLD else "끔"))
log("씬        %s" % STAGE)
log("팔레트    원점 %s mm   최대 적재 %s mm" % (ORIGIN_MM, cfg["pallet"]["max_stack_mm"]))
log("라인      스폰 %s -> 스토퍼 %.3f m   벨트 %.2f m/s"
    % (cfg["conveyor"]["spawn_point_mm"], STOPPER_X, BELT_MPS))
for p in (ROBOT_PRIM, GRIPPER_PRIM, "/World/Stopper"):
    ok = stage.GetPrimAtPath(p).IsValid()
    log("  %-28s %s" % (p, "있음" if ok else "!! 없음"))
    if not ok:
        log("  -> src/packing_sim/run_layout_to_spec.sh 를 먼저 돌려라.")
        if _S.get("tee"):
            _S["tee"].close()
        _kitexit.shutdown(2, log=log)
        raise SystemExit
log("-" * 78)

omni.timeline.get_timeline_interface().play()


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

    art = SingleArticulation(prim_path=ROBOT_PRIM, name="h2017")
    art.initialize()

    tmp = tempfile.mkdtemp(prefix="h2017_lula_")
    urdf, _n, missing = IMP.make_abs_urdf(tmp)
    if missing:
        raise RuntimeError("메시 없음: %s" % missing[:3])
    lula = mg.LulaKinematicsSolver(robot_description_path=str(_paths.DESCRIPTOR),
                                   urdf_path=str(urdf))
    pos, quat = art.get_world_pose()
    lula.set_robot_base_pose(np.array(pos), np.array(quat))
    log("로봇      베이스 %s mm  EE %s"
        % ([round(float(v) * 1000) for v in pos], EE_FRAME))

    q = MQ.MainQueue(timeout_s=float(cfg["service"]["timeout_s"]) * 0.8)
    sp = SP.Spawner(stage, cfg, q, zone=ZONE)

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

    rm = RM.RobotMotion(art, lula, cfg, ee=EE_FRAME, gripper_path=GRIPPER_PRIM,
                        gap_fn=_gap_mm)
    cc = conveyor_ctl.ConveyorCtl(stage, cfg, q)

    if CAM:
        import camera_pick
        cam = camera_pick.PickCamera(stage, cfg, q)
        _S["cam"] = cam

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
    server.register({"ping": lambda r: {"ok": True}})
    server.register(sp.handlers())
    server.register(cc.handlers())
    server.register(rm.handlers())
    if _S.get("cam"):
        server.register(_S["cam"].handlers())
    log("소켓      127.0.0.1:%d   cmd %d 개" % (PORT, len(server.cmds())))

    # ★ READY 로 순간이동. 안 하면 첫 이동이 179 deg 라 구간 예산(20 s)을 넘겨
    #   MOTION_TIMEOUT 이 난다 (verify_b4_test1_1.py 주석의 실측).
    import yaml
    from isaacsim.core.utils.types import ArticulationAction
    ready = np.array(yaml.safe_load(_paths.DESCRIPTOR.read_text())["default_q"], dtype=float)
    art.set_joint_positions(ready)
    art.set_joint_velocities(np.zeros(6))
    _S["ready_action"] = ArticulationAction(joint_positions=ready)
    _S["ready_q"] = ready                                # 감긴 팔을 되돌릴 때 쓴다
    log("자세      READY 로 순간이동 %s deg" % np.round(np.degrees(ready), 1).tolist())

    # ★ Camera.initialize() 는 **타임라인이 돌기 시작한 뒤**여야 한다. setup() 은
    #   부팅 20스텝 뒤라 이미 Play 상태다.
    if _S.get("cam") and not _S["cam"].start():
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
    """`main` 을 여닫아 픽업 구간에 한 대씩만 들여보낸다. **자기 소켓을 쓴다.**

    판단은 `spawn_status.queue` 하나로 한다 — 픽업 구간(7500~10000)에서 벨트
    높이에 있는 박스 수다. 그리퍼가 박스를 들어 올리는 순간 z 가 바뀌어 이 수가
    떨어지므로, **집자마자** 다음 박스가 출발한다. 사이클이 끝날 때까지 안 기다린다.
    """
    import sim_link as SL
    link = SL.SimLink(port=PORT)
    _S["meter_link"] = link
    running = None                                       # 마지막으로 건 상태
    try:
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
            r = link.call(cmd="spawn_box", box_id=bid, type=t["name"], yaw_deg=0)
            st = link.call(cmd="spawn_status")
            log("   [투입] %-9s %-3s  (라인 위 %s 개)"
                % (bid, t["name"], st.get("on_belt", "?")))
            if not r.get("ok"):
                log("   [투입] !! spawn_box %s -> %r" % (bid, r))
    except Exception as e:
        log("   [투입] !! %r" % (e,))


def worker(specs, ids):
    """머리 쪽. 소켓으로만 말한다."""
    import cycle as CY
    import sim_link as SL
    _S["CY"] = CY                                        # report() 가 좌표 변환에 쓴다

    try:
        link = SL.SimLink(port=PORT)
        _S["link"] = link

        by_id = {bid: LinkPrim(link, bid) for bid in ids}
        hm, pal = cfg["heightmap"], cfg["pallet"]
        PKM = D._load_packer_module()

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

            def find_placement(self, box_size, box_mass=0.0, **kw):
                if not ARM_GUARD or kw.get("reject") is not None:
                    return super().find_placement(box_size, box_mass, **kw)

                # 1) 길도 막히지 않고 함정도 안 만드는 자리
                p = super().find_placement(
                    box_size, box_mass,
                    reject=lambda *a: self._sweep_risk(*a) or self._builds_trap(*a))
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

        vision = D.PalletVision(by_id, specs, ORIGIN_MM)
        svc = LineServices(link, specs, factory, vision, D.CLEARANCE_MM,
                           log=lambda *a: log("           ", *a), verbose=VERBOSE)
        cyc = CY.Cycle(link, svc, cfg, zone=ZONE, log=lambda *a: log("   [cycle]", *a))

        # 라인을 돌린다. 이후 정지/재가동은 `cycle.py` 가 b_pickup 만 만진다.
        link.call(cmd="set_velocity", seg="main", mps=BELT_MPS)
        link.call(cmd="set_velocity", seg="b_pickup", mps=BELT_MPS)
        threading.Thread(target=spawner_thread, args=(specs, ids),
                         daemon=True, name="spawn").start()
        if METER_MAX > 0:
            threading.Thread(target=meter_thread, daemon=True, name="meter").start()

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
            q6 = np.asarray(_S["art"].get_joint_positions()[:6], dtype=float)
            if float(np.max(np.abs(q6))) > 2.0 * math.pi:
                log("       !! 관절이 감겼다 %s deg — READY 로 되돌린다"
                    % np.round(np.degrees(q6), 0).tolist())
                _S["q"].run(lambda: (_S["art"].set_joint_positions(_S["ready_q"]),
                                     _S["art"].set_joint_velocities(np.zeros(6)),
                                     {"ok": True})[-1])
                time.sleep(1.0)                          # 드라이브가 따라올 시간

        rest = {}                                        # 놓인 뒤의 기준 자세 (knock 용)
        pick_retry = {}                                  # box_id -> 흡착 재시도 횟수
        on_pallet = []                                   # 지금 팔레트에 선 기둥 (팔 간섭 진단용)
        swapped_for = set()                              # 이 박스 때문에 이미 교체했다
        pallets = 1
        t_all = time.time()
        i = 0
        while i < len(ids) and not _S["done"]:
            # ── 다음 박스가 스토퍼에 설 때까지 ──────────────
            pick, t0, nag = None, time.time(), 0.0
            while time.time() - t0 < ARRIVE_TIMEOUT_S and not _S["done"]:
                c = link.call(cmd="pickup_check", zone=ZONE)
                if c.get("present"):
                    pick = c
                    break
                # 안 오면 어디에 있는지 주기적으로 찍는다. 이게 없으면 타임아웃
                # 났을 때 "안 왔다" 는 사실만 남고 원인을 못 찾는다
                # (2026-08-23 15번째가 180 s 동안 안 와서 겪었다).
                if time.time() - t0 > nag + 15.0:
                    nag = time.time() - t0
                    log("   ...대기 %.0f s — %s" % (nag, _line_state(link, ids)))
                time.sleep(0.05)
            if pick is None:
                log("   !! %d 번째 박스가 %.0f s 안에 스토퍼에 안 왔다 — 중단"
                    % (i + 1, ARRIVE_TIMEOUT_S))
                log("      마지막 라인 상태: %s" % _line_state(link, ids))
                _S["stalled"] = True
                break
            bid = pick["box_id"]
            t = specs[bid]
            _CUR["box"] = (bid, t["h_mm"] / 2000.0)      # gap_fn 이 읽는다
            wait_s = time.time() - t0

            log("\n[%2d/%d] %-9s %-3s %s mm %.0f kg   대기 %.1f s   앞면 %.4f m"
                % (i + 1, len(ids), bid, t["name"],
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
            _S["cyc_t0"], _S["cyc_id"] = tc, bid     # 메인 스레드가 감시한다 (on_step)
            res = cyc.run(bid, pick["pose"])
            wall = time.time() - tc
            _S["cyc_t0"] = None
            _CUR["box"] = None                           # 사이클 밖에서 옛 박스로 재지 않는다

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
                want = np.asarray(CY.pallet_place_to_world(place, t["h_mm"], ORIGIN_MM))
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
            near_mm = 0.0
            if place is not None:
                near_mm = _near_worst_mm(
                    place["x_mm"] / 1000.0, place["y_mm"] / 1000.0,
                    [(q["x_mm"] / 1000.0, q["y_mm"] / 1000.0,
                      q["z_mm"] + BOX_H_MM[q["type"]]) for q in on_pallet])
                if near_mm > NEAR_LIM_MM:
                    log("       !! 근거리 기둥 %.0f mm > 한계 %.0f (ped %.0f) — "
                        "아래팔이 그 위를 쓴다" % (near_mm, NEAR_LIM_MM, PED_MM))

            if res == "PLACED":
                rest[bid] = by_id[bid].get_world_pose()[0].copy()
                if place is not None:
                    on_pallet.append(dict(place, type=t["name"], id=bid))

            _S["rows"].append({"id": bid, "type": t["name"], "res": res, "err": err,
                               "wall": wall, "wait": wait_s, "knock": knock, "who": who,
                               "kvec": kvec, "near_mm": near_mm, "place": place})
            log("       %-11s 오차 [%+.1f %+.1f %+.1f] mm   %.1f s   knock %.1f mm (%s)"
                % (res, err[0], err[1], err[2], wall, knock, who))

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
                sw = link.call(cmd="swap_pallet", zone=ZONE)
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
                    fell_on_pallet = (PALLET_X[0] <= _bx <= PALLET_X[1]
                                      and PALLET_Y[0] <= _by <= PALLET_Y[1]
                                      and _bz >= PALLET_TOP_M - 0.05)
                if fell_on_pallet:
                    log("       !! 팔레트 위에 떨어졌다 (%.3f %.3f %.3f) — 개별 despawn "
                        "대신 팔레트를 갈아낸다" % tuple(pr["pos_m"]))
                    link.call(cmd="set_spawn_enabled", enabled=False)
                    sw = link.call(cmd="swap_pallet", zone=ZONE)
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
        _S["result"] = {"trace": traceback.format_exc()}
    finally:
        _S["done"] = True


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
    log("  #  박스       종류  결과         오차 XY / Z [mm]   대기 s  사이클 s  knock mm")
    log("-" * 78)
    for i, w in enumerate(rows, 1):
        exy = float(np.linalg.norm(w["err"][:2]))
        cell = ("%6.1f / %+6.1f" % (exy, w["err"][2])) if np.isfinite(exy) else "     -  /     - "
        log("%3d  %-10s %-4s  %-11s %s   %6.1f   %6.1f   %7.1f"
            % (i, w["id"], w["type"], w["res"], cell,
               w["wait"], w["wall"], w["knock"]))
    log("-" * 78)

    placed = [w for w in rows if w["res"] == "PLACED"]
    bypassed = [w for w in rows if w["res"] == "PALLET_FULL"]
    failed = [w for w in rows if w["res"] not in ("PLACED", "PALLET_FULL")]
    ok = bool(placed)
    log("처리한 것     %d / %d   (놓음 %d · 우회 %d · 실패 %d)"
        % (len(rows), N, len(placed), len(bypassed), len(failed)))
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
        nb = [w for w in rows if w.get("near_mm", 0.0) > NEAR_LIM_MM]
        log("근거리 기둥   위반 %d 사이클 · 최고 %.0f mm  (한계 %.0f @ ped %.0f)%s"
            % (len(nb), max([w.get("near_mm", 0.0) for w in rows] or [0.0]),
               NEAR_LIM_MM, PED_MM,
               "" if not nb else "  -> " + ", ".join(w["id"] for w in nb)))
        log("사이클        평균 %.1f s · 최대 %.1f s   (목표 %.0f s · 스폰 간격 %.0f s)"
            % (sum(cyc_s) / len(cyc_s), max(cyc_s),
               cfg["robot"]["cycle_time_target_s"], SPAWN_DT))
        ce = _S.get("cam_err") or []
        if ce:
            exy = [max(abs(e[3]), abs(e[4])) for e in ce]
            ez = [abs(e[5]) for e in ce]
            hit = sum(1 for e in ce if e[1] == e[2])
            log("카메라        %d 회 인식 · 종류 맞춤 %d/%d · 중심 오차 XY 최대 "
                "%.1f mm · 윗면 z 최대 %.1f mm"
                % (len(ce), hit, len(ce), max(exy), max(ez)))
        log("팔레트        %d 판" % r.get("pallets", 1))
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
                    w["place"], BOX_H_MM[w["type"]], ORIGIN_MM))
                dv = (q - want) * 1000.0
                tmm = q[2] * 1000.0 + BOX_H_MM[w["type"]] / 2.0 - ORIGIN_MM[2]
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
        all_handled = len(rows) == N and not failed
        ok = (all_handled
              and max(exy) < D.PLACE_TOL_XY_MM and max(ez) < D.PLACE_TOL_Z_MM
              and kn < D.KNOCK_WARN_MM
              and max(tops) <= cfg["pallet"]["max_stack_mm"])
        for cond, msg in ((all_handled,
                           "%d 개를 끝까지 처리해야 한다 (놓거나 우회. BLOCKED 는 실패)" % N),
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
        _S["rm"].step()
        _S["sp"].step()
        _S["art"].apply_action(_S["ready_action"])       # 안 걸면 옛 목표로 튄다
        if _S["n"] >= SETTLE_STEPS:
            log("자세      수렴 %s deg"
                % np.round(np.degrees(_S["art"].get_joint_positions()[:6]), 1).tolist())
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
            log("박스      %d 개  %s" % (N, [t["name"] for t in fed]))
            log("-" * 78)
            _S["phase"] = "run"
            _S["t0"] = time.time()
            threading.Thread(target=worker, args=(specs, ids),
                             daemon=True, name="stack15").start()
        return
    _S["q"].pump()
    _S["rm"].step()                                      # ★ world.step() 앞, 메인 스레드
    _S["sp"].step()
    # ★ 사이클 하나가 굳으면 워치독(900 s)까지 아무도 안 끊는다. CYCLE_WALL_S 를
    #   선언만 해 두고 안 쓰고 있었다. 여기서 실제로 쓴다 — 굳은 사이클은 그 자리에서
    #   끊고 그때까지의 결과라도 남긴다.
    if _S.get("cyc_t0") and time.time() - _S["cyc_t0"] > CYCLE_WALL_S:
        log("   !! %s 사이클이 %.0f s 를 넘겼다 — 중단하고 결과를 낸다"
            % (_S.get("cyc_id"), CYCLE_WALL_S))
        _S["cyc_t0"] = None
        _S["stalled"] = True
        _S["done"] = True
    if _S["done"] or time.time() - _S["t0"] > WATCHDOG_S:
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
    print("  Play 를 다시 눌러도 안 움직인다. Stop 이 물리 뷰를 무효화해서", flush=True)
    print("  부팅 때 잡은 로봇·그리퍼 핸들이 죽었기 때문이다 (재초기화 경로 없음).", flush=True)
    print("  다시 보려면 **창을 닫고** 다시 실행해라:", flush=True)
    print("      cd ~/palletris_ws && ./scripts/run_demo.sh", flush=True)
    # ★ Stop 은 박스들을 **스폰 당시 USD 좌표**로 되돌린다 — 팔레트에 쌓인 것까지
    #   전부 스폰 지점에 겹쳐 선다. 그대로 Play 하면 벨트(OmniGraph 라 이 드라이버와
    #   무관하게 돈다)가 그것들을 실어 나가서, 안 나왔던 박스가 갑자기 움직이는
    #   것처럼 보인다. 여기서 좌표 자체를 라인 밖으로 적어 화면을 비운다.
    sp = _S.get("sp")
    if sp is not None:
        try:
            print("  박스 프림 %d 개를 라인 밖으로 치웠다 (Stop 이 스폰 좌표로 "
                  "되돌리는 것을 막는다)." % sp.park_all(), flush=True)
        except Exception as _e:
            print("  박스 정리 실패: %r" % (_e,), flush=True)
    print("!" * 78 + "\n", flush=True)


_S["timeline_sub"] = (omni.timeline.get_timeline_interface()
                      .get_timeline_event_stream()
                      .create_subscription_to_pop(_on_timeline, name="palletris-stop"))

_S["physx_sub"] = get_physx_interface().subscribe_physics_step_events(on_step)
log("Play — 부팅 대기...\n")
