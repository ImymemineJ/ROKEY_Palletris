"""A 소유 소켓 핸들러 — 박스 스폰 · 스토퍼 감지 · pose 조회 (작업명세서 §5.5~§5.6).

    q  = mainq.MainQueue()
    sp = Spawner(stage, cfg, q, zone="B")
    server.register(sp.handlers())
    while running:
        q.pump()                  # ★ 매 물리 스텝, 메인 스레드
        sp.step()
        world.step(render=True)

프로토콜 표(공통가이드 §3) 중 여기서 맡는 것:
    spawn_box · spawn_status · set_spawn_enabled · despawn_box
    pickup_check · get_box_pose

## ★ 박스 프림 경로는 `/World/boxes/<box_id>` 다 (규칙 5)

`robot_motion.py` 의 `BOX_PRIM_FMT` 와 **글자 하나까지** 같아야 한다. USD 경로는
대소문자를 구분한다 — 기존 `spawn_feeder.py` 는 `/World/Boxes/Box_00_No3` 에
만드는데, 그건 규칙 5 와 다르므로 B4 의 `grasp(box_id)` 가 프림을 못 찾는다.
**이 파일이 인터페이스 경로이고, `spawn_feeder.py` 는 단독 데모 경로다.** 섞지 않는다.

## ★ 스폰 z 는 밑면 기준이다 (§5.5, 함정 #7)

`conveyor.spawn_point_mm` 은 **박스 밑면 중심**이다. prim 의 translate 는 중심이므로
`+ h/2` 를 반드시 더한다. 810 을 그대로 쓰면 5호 밑면이 640 이 되어 벨트 상면(800)
보다 160 mm 아래 — 즉시 관통해서 물리가 터진다.

## ★ 물리 속성은 **박스마다** 건다 (§5.4)

`solverPositionIterationCount` 를 PhysicsScene 에 걸면 무시되고 스택이 무너진다.
리지드바디별 속성이라 여기 스포너에서 박스마다 걸어야 한다.

## 스레드 규약

핸들러는 **소켓 스레드**에서, `step()` 은 **메인(물리) 스레드**에서 돈다.
USD 를 고치는 일(스폰·디스폰)은 소켓 스레드에서 하면 안 되므로 `MainQueue` 에
넣고 메인 스레드가 처리한다 (`mainq.py` — 왜 그런지 실측 근거가 거기 있다).
읽기(pickup_check·get_box_pose)는 `step()` 이 갱신해 둔 스냅샷을 보므로
락 없이 안전하다.
"""
import math
import os
import time

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

BOX_ROOT = "/World/boxes"                 # ★ 규칙 5. 소문자다
BOX_PRIM_FMT = BOX_ROOT + "/%s"
LOOKS_ROOT = "/World/Looks"
QR_LOOKS_ROOT = LOOKS_ROOT + "/QR"        # 박스마다 텍스처가 달라 머티리얼도 박스마다다
PHYS_MAT_PATH = LOOKS_ROOT + "/PhysicsMaterial"          # 씬 빌더가 만든다. 없으면 여기서

# ── 박스 색 ────────────────────────────────────────────────
# ★ stage 1 은 **벨트 한 라인 · 로봇 한 대 · 팔레트 한 대**다. 색으로 나눠 보낼
#   곳이 없으므로 전부 한 색으로 낸다. 종류(3·4·5호)는 치수와 무게로 이미 구분되고,
#   `box_type` 속성으로도 씬에서 읽을 수 있다 (§5.5 ground truth).
#   종류별 색은 stage 2 에서 존 A/B/C 로 갈라 보낼 때 되살릴 것이라 지우지 않는다.
LINE_COLOR = (1.0, 0.15, 0.15)            # 색을 안 준 박스의 기본 — 빨강
LINE_MAT = LOOKS_ROOT + "/BoxLine"
# --- stage 2 (존 분류) 용. 지금은 안 쓴다 ---------------------------------
# BOX_SLUG = {"3호": "No3", "4호": "No4", "5호": "No5"}
# BOX_COLOR = {"3호": (1.0, 0.15, 0.15),     # 빨강
#              "4호": (0.15, 0.9, 0.25),     # 초록
#              "5호": (0.2, 0.35, 1.0)}      # 파랑
# 되살릴 때는 _ensure_roots 의 머티리얼 생성 루프와 _make_box 의 displayColor ·
# 바인딩 세 곳을 같이 되돌린다. 씬에 남아 있는 /World/Looks/BoxNo3·No4·No5 는
# 예전 판이 만든 것이라 지금은 아무도 안 쓴다.
# ── QR 라벨 ────────────────────────────────────────────────
# 윗면에 붙이는 흰 바탕 QR 스티커. **시각 전용**이다 — 콜라이더도 리지드바디도
# 걸지 않는다. 박스 프림의 자식이라 박스를 따라 같이 움직인다.
#
# ★ PNG 는 씬 밖에서 미리 굽는다 (`scripts/gen_qr_labels.py`).
#   Isaac 내장 파이썬(3.11)에 cv2 가 있다는 보장이 없어서다. 여기서 만들려다
#   실패하면 라벨 없는 박스로 데모가 돌고 인식이 통째로 죽는다.
LABEL_PRIM = "Label"                      # 박스 프림의 자식 이름
LABEL_LIFT_M = 0.001                      # 윗면에서 1 mm 띄운다 (z-fighting 방지)
# ★ 거울상 방지 스위치. QR 은 회전에는 강하지만 **좌우 반전되면 절대 안 읽힌다.**
#   USD 의 st 원점(좌하)과 PNG 원점(좌상)이 달라서 한 번 뒤집어 주는 것이 기본이다.
#   씬에서 라벨이 육안으로는 멀쩡한데 cam1 이 못 읽으면 **먼저 이 값을 뒤집어 볼 것.**
#
# ★★ True -> **False** (2026-08-26 김병직 실측, test_3 픽업 카메라 프레임).
#   True 로 두면 렌더 결과가 **거울상**이 된다. 눈으로는 완전히 멀쩡한 QR 인데
#   디코더가 통째로 실패한다 — 파인더 패턴이 좌상·좌하·**우하**에 찍힌다
#   (정상은 좌상·우상·좌하). 회전이면 디코더가 알아서 푸는데 반전은 못 푼다.
#   실측: 렌더 프레임을 어느 축으로든 한 번 뒤집으면 바로 'BOX005' 로 읽힌다.
#         원본 PNG 자체는 그대로 'BOX002' 로 읽힌다 — 굽는 쪽 문제가 아니다.
#   증상이 "QR 판독 실패 -> box_id 빈 채 발행 -> 관제탑이 무시" 로 이어져서
#   **인식이 다 되는데도 사이클이 한 번도 안 도는** 모습으로 나타난다.
LABEL_FLIP_V = False

# ── 분류 (2026-08-25) ──────────────────────────────────────
# ★ 분류 기준이 **색 -> QR(배송지)** 로 바뀌었다.
#   그래서 박스 색은 **하나로 통일**한다 (LINE_COLOR). 목적지는 프림의 `box_dest`
#   속성이 ground truth 이고, 실물에서는 QR 이 그 값을 싣는다.
#   색 기능(`boxes.colors` · spawn 의 color 인자)은 지우지 않고 남겨 둔다 —
#   1 차 색상 판정으로 되돌릴 수 있어야 한다 (CLAUDE.md §6: 1 차 색상 · 2 차 QR).
#   색표를 안 주면 아무도 색을 안 쓰고 전부 LINE_COLOR 다.
DEFAULT_COLORS = {"red": (1.0, 0.15, 0.15),
                  "green": (0.15, 0.9, 0.25),
                  "blue": (0.2, 0.35, 1.0)}
COLOR_MAT_FMT = LOOKS_ROOT + "/Box_%s"
# 스토퍼 앞 이 거리 안에 박스 **앞면**이 오고 거의 멎으면 픽업 대기로 본다.
PICKUP_MARGIN_M = 0.060                   # 트랙A 가이드 §A6~A8 의 60 mm
PICKUP_VEL_MPS = 0.01                     # |vx| < 1 cm/s
# 벨트 상면 기준 높이 허용오차. 흡착으로 들린 박스를 걸러 내는 것이 목적이라
# 넉넉해도 된다 — 가장 낮은 박스(3호 210)의 절반보다는 작아야 한다.
PICKUP_Z_TOL_M = 0.05
# despawn 한 박스를 옮겨 둘 자리. 라인·팔레트·로봇 어디에도 안 걸린다.
DESPAWN_XYZ_M = (-50.0, -50.0, -10.0)
# 이 밖으로 나가면 "라인 이탈" 로 본다. despawn 자리(-50)보다 넉넉히 밖이라
# despawn 한 박스를 이탈로 오인하지 않는다.
# 스폰 지점 앞뒤로 이만큼 안에 박스가 있으면 "안 비었다" 로 본다.
# 가장 긴 박스(5호 480) 보다 넉넉해야 겹치지 않는다.
SPAWN_CLEAR_M = 0.70
# ── 반출 연출 (2026-08-27 김병직). 만재 팔레트가 반출 컨베이어를 타고 나가는
#    모습을 시연에 넣기 위한 것. **기본 꺼짐** — 켜야만 동작이 바뀐다.
#    ★ 방향은 **+Y** 다 (씬 실측 2026-08-27). 팔레트(y 4.48~5.68)가 얹혀 있는
#      셀 컨베이어가 +Y 로 뻗는다: A `/ConveyorTrack_08` y 4.28~7.61 ·
#      B `/ConveyorTrack_07` y 4.28~8.50 · C `/World/ConveyorTrack_07` y 4.28~9.53.
#      (conveyor_ctl 의 `a_cell`·`b_cell`·`c_cell` 이 이미 "진행 +Y" 로 잡는 그 조각들이다)
#      ⚠ 처음에 -Y 로 넣었다가 박스가 **리턴 직선(`ConveyorTrack_05`, y -3.6~-2.4)**
#        쪽으로 갔다 — 정반대였다. 부호를 눈대중으로 정하지 말 것.
#    ★ 이동 거리 2.4 m: 셋 중 제일 짧은 A(7.61)에서도 벨트 위에 머무는 값이다
#      (5.076 + 2.4 = 7.48). 더 멀리 보내고 싶으면 존별 벨트 끝을 넘지 않게 올릴 것.
#    ⚠ 숫자를 코드에 박지 않는 규칙(§4)의 예외가 아니다 — 이건 **연출 파라미터**라
#      system.yaml(공용, 승인 필요)이 아니라 환경변수로 뺐다. 값을 굳히면
#      test_3.yaml 오버레이로 옮길 것.
# ★ 이 연출의 원본은 김학민 `ship_pallet()` 이다 (`scripts/demo_stack15_test1_1.py`,
#   커밋 `0610a00`·`e8a6c84`). 그쪽은 **소켓**(`move_prim`·`set_kinematic`·
#   `copy_prim`)으로 demo 하네스에서 돌아 ROS 통합 경로(`sim_main`)에는 안 붙어
#   있었다 — `/sim/swap_pallet` 은 `_swap_pallet`(제자리 despawn)만 불렀다.
#   그래서 같은 동작을 **인프로세스로** 옮겨 놓은 것이고, 그가 실측으로 얻은
#   교훈은 그대로 따랐다: 셀(+Y) 먼저 -> 합류(+X) 순서 · 적재물을 얼려 같은
#   변위로 이동 · 도착 뒤에도 안 풀기 · 트럭에 사본 남기기.
OUTFEED_ON = os.environ.get("PL_OUTFEED", "0") == "1"
# 김학민 정본은 0.5 (실물 팔레트 컨베이어 0.2~0.5 m/s). 여기 1.0 은 2026-08-27
# 강의실에서 김병직이 화면으로 보고 "속도 괜찮다" 한 값이다. 느리게 하려면 0.5.
OUTFEED_MPS = float(os.environ.get("PL_OUTFEED_MPS", "1.0"))
OUTFEED_TRAVEL_M = float(os.environ.get("PL_OUTFEED_TRAVEL_M", "2.4"))
OUTFEED_HOLD_S = float(os.environ.get("PL_OUTFEED_HOLD_S", "2.0"))
# ★ 출발 몇 초 뒤에 빈 팔레트를 다시 보일지 (2026-08-27 정정인). 0 이면 즉시.
#   자세한 사연은 `_outfeed_enqueue` 의 사본 주석 참조.
OUTFEED_RESPAWN_S = float(os.environ.get("PL_OUTFEED_RESPAWN_S", "3.0"))

# ── 리턴 라인 걸림 감시 (2026-08-27 정정인) ──
# 리턴 벨트는 어떤 제어도 세우지 않는다 (갭핑·점유 제어는 본선·지선 이야기다).
# 그런데도 이음매·전복으로 박스가 제자리에 서는 일이 남아 있다 — 지선의 12초
# 감시(관제탑)와 합류 창의 30초 감시(sim_main._Feeder)는 **그 구역만** 본다.
# 리턴 한복판에서 선 박스는 아무도 안 치워서 뒤차가 줄줄이 쌓였다 (강의실 실측).
# 여기서는 리턴 조각(belt_velocity_raw 의 프림들) bbox 안에서 STALL_DESPAWN_S
# 이상 제자리(누적 이동 < 30 mm)인 박스를 낙하물로 보고 치운다.
# 벽시계 기준이다 — 배속(PL_RENDER_EVERY)이 걸리면 sim 시간으로는 더 길어질
# 뿐이라 오탐 방향으로는 안 틀린다. 0 을 주면 끈다.
STALL_DESPAWN_S = float(os.environ.get("PL_STALL_DESPAWN_S", "20.0"))
STALL_MOVE_M = 0.03                        # 이보다 덜 움직이면 "제자리"
PALLET_PRIM_FMT = "/World/Pallet_%s"
# ★ 상차까지 가는 전체 경로 (씬 실측 2026-08-27). ㄱ 자다:
#     팔레트 -> (+Y) 셀 컨베이어·커브 -> (+X) 팔레트 레인 -> 트럭 적재함
#   꺾는 y 는 **팔레트 가이드 레일 두 개의 한가운데**로 잡는다 — 레인 폭이
#   0.83 m 라 가운데로 가야 가이드에 안 걸린다:
#     A 8.49/9.32 · B 9.49/10.32 · C 10.49/11.32
#   끝 x 는 **트럭 적재함 한가운데**다 (`Truck_Bed` x -2.4~-0.8, 상면 z 0.8 —
#   팔레트 밑면과 높이차 0 이라 그대로 밀어 넣으면 된다. CLAUDE.md §4).
#   좌표를 코드에 박지 않고 **씬에서 재서** 쓴다 — 씬이 바뀌면 따라간다.
PALLET_GUIDE_FMT = "/World/PalletGuide_%s_%s"    # (zone, lo|hi)
TRUCK_BED_PRIM = "/World/Truck_Bed"
LOST_XY_M = 200.0
LOST_Z_MIN_M = -50.0


def _yaw_deg(quat):
    """(w, x, y, z) -> yaw [deg]."""
    w, x, y, z = [float(v) for v in quat]
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


class Spawner:
    def __init__(self, stage, cfg, mainq, zone="B", log=print, zones=None):
        self.stage = stage
        self.cfg = cfg
        # ★ USD 쓰기(스폰·디스폰)는 메인 스레드에서. mainq.py docstring 참조.
        self.mainq = mainq
        self.zone = zone
        self._log = log

        self.dt = 1.0 / float(cfg["physics"]["scene"]["time_steps_per_second"])
        self.types = {t["name"]: t for t in cfg["boxes"]["types"]}
        sp = cfg["conveyor"]["spawn_point_mm"]
        self.spawn_xyz_m = [v / 1000.0 for v in sp]
        # ★ 픽업 축 (2026-08-24 추가). test1-1 은 벨트가 **+X** 로 흘러 스토퍼가
        #   x=10000 에 있지만, 셀마다 메인 라인에서 **+Y** 로 갈라지는 지선을 쓰는
        #   배치(test_3)도 있다. 축을 설정으로 받아 아래 판정이 pos[axis] 를 보게 한다.
        #   기본이 "x" 라 이 키가 없는 기존 씬은 동작이 하나도 안 바뀐다.
        #   ⚠ 두 축 모두 **진행이 증가 방향**(+X / +Y)이라는 전제다. 감소 방향
        #     라인이 생기면 부호까지 파라미터로 빼야 한다.
        _ax = str(cfg["conveyor"].get("pickup_axis", "x")).lower()
        if _ax not in ("x", "y"):
            raise ValueError("conveyor.pickup_axis 는 x | y 여야 한다: %r" % _ax)
        self.axis = 0 if _ax == "x" else 1
        self.axis_name = _ax
        # 스토퍼/픽업 구간은 **위 축을 따라** 잰 값이다 (키 이름은 옛 것을 유지한다).
        self.stopper_pos_m = cfg["conveyor"]["stopper_x_mm"] / 1000.0
        # 픽업 판정 구간. **위쪽 경계가 없으면 팔레트에 놓인 박스가 다시 잡힌다** —
        # 아래 `_refresh` 주석의 실측 참조.
        self.pickup_p0_m = cfg["conveyor"]["pickup_range_mm"][0] / 1000.0
        self.belt_top_m = cfg["conveyor"]["belt_top_z_mm"] / 1000.0
        # ★ 가로축 창 (2026-08-24 추가). 지선이 여러 갈래인 배치(test_3)에서는
        #   진행축만 봐서는 **어느 지선에 있는 박스인지** 구분이 안 된다.
        #   실측: 존 A 로 돌렸는데 존 B(X -12.9)·C(X -18.9) 지선에 놓인 박스를
        #   집으려다 IK_FAIL 3 연발이 났다. 이 창이 없으면 그 일이 다시 난다.
        #   키가 없으면 제한 없음 — test1-1 은 지선이 하나라 동작이 안 바뀐다.
        #
        # ★ 다존 (2026-08-25). 로봇 3 대를 동시에 돌리려면 픽업 감지가 **존마다**
        #   따로 있어야 한다. `zones` 를 주면 그만큼 창을 만든다. 안 주면 `zone`
        #   하나뿐이라 기존 동작이 하나도 안 바뀐다 (test1-1 기준선 보호).
        self.zones = [str(z) for z in (zones or [zone])]
        if zone not in self.zones:
            self.zones.insert(0, zone)
        _xtab = (cfg["zones"].get("pickup_cross_mm") or {})
        self.cross_axis = 1 - self.axis
        self.cross_by_zone = {}
        for _z in self.zones:
            _c = _xtab.get(_z)
            self.cross_by_zone[_z] = ((min(_c) / 1000.0, max(_c) / 1000.0)
                                      if _c else None)
        # 기본 존 창 — 이름을 유지해 기존 코드가 그대로 돈다.
        self.cross_range_m = self.cross_by_zone.get(zone)
        self.default_yaw = float(cfg["boxes"].get("spawn_yaw_deg", 0.0))
        # ── QR 라벨 설정. 없거나 꺼져 있으면 라벨 없이 그냥 돈다 ──
        _qr = (cfg.get("recognition") or {}).get("qr") or {}
        self.qr_on = bool(_qr.get("enabled", False))
        self.label_m = float(_qr.get("label_mm", 198)) / 1000.0
        _dir = str(_qr.get("label_dir", "qr_labels"))
        if not os.path.isabs(_dir):
            # ★ 홈 디렉터리를 박지 않는다 (packing_common/config.py 주석 참조).
            #   이 파일은 <ws>/src/packing_sim/isaac/spawner.py 라 3 단계 위가 ws 다.
            _ws = os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.realpath(__file__)))))
            _dir = os.path.join(_ws, _dir)
        self.label_dir = _dir
        self._label_warned = False
        # 색표. 설정에 없으면 **색을 아예 안 쓴다** — 박스는 전부 LINE_COLOR 다.
        #   (QR 분류로 바뀌면서 색은 통일했다. DEFAULT_COLORS 는 되돌릴 때 참고용.)
        _cols = cfg["boxes"].get("colors") or {}
        self.colors = {str(k): tuple(float(v) for v in rgb)
                       for k, rgb in _cols.items()}
        # 팔레트 영역 — `swap_pallet` 이 "이 팔레트에 실린 것" 을 가려내는 데 쓴다.
        # 원점은 **모서리**이고 z 는 **상면**이다 (작업명세서 §3.3).
        # ★★ **존마다** 만든다 (2026-08-26). 예전에는 `zone` 하나의 범위만 들고
        #   있어서, `swap_pallet(zone="B")` 를 불러도 늘 그 하나를 비웠다.
        #   실측(영상 claude_review): **B 를 상차하면 A 팔레트가 초기화됐다.**
        #   `h_swap_pallet` 이 zone 을 검사만 하고 `_swap_pallet` 에 안 넘긴 것도
        #   같이 고친다 — 검사만 하면 "받았지만 안 쓴다" 가 된다.
        _pal = cfg["pallet"]
        _POZ = cfg["zones"]["pallet_origin_world_mm"]
        self.pallet_box = {}
        for _z in self.zones:
            _p = _POZ.get(_z)
            if _p is None:
                continue
            self.pallet_box[_z] = (
                (_p[0] / 1000.0, (_p[0] + _pal["width_mm"]) / 1000.0),
                (_p[1] / 1000.0, (_p[1] + _pal["depth_mm"]) / 1000.0),
                _p[2] / 1000.0)
        _po = _POZ[zone]
        # 하위호환 — 옛 코드가 아직 이 셋을 본다
        self.pallet_x = (_po[0] / 1000.0, (_po[0] + _pal["width_mm"]) / 1000.0)
        self.pallet_y = (_po[1] / 1000.0, (_po[1] + _pal["depth_mm"]) / 1000.0)
        self.pallet_top_m = _po[2] / 1000.0

        phys = cfg["physics"]
        self.rb_pos_iter = int(phys["rigid_body"]["solver_position_iterations"])
        self.rb_vel_iter = int(phys["rigid_body"]["solver_velocity_iterations"])
        self.rest_offset_m = phys["collider"]["rest_offset_mm"] / 1000.0
        self.contact_offset_m = phys["collider"]["contact_offset_mm"] / 1000.0
        self.mu_s = float(phys["material"]["static_friction"])
        self.mu_d = float(phys["material"]["dynamic_friction"])

        self.enabled = True
        self.spawned = 0
        self.boxes = {}                    # box_id -> {"path", "dims", "type"}
        self.lost = []                     # 라인을 벗어난 박스 (물리 파탄)
        self._snap = {}                    # box_id -> {"pos_m", "yaw_deg", "vel_mps"}
        self._pickup = {"present": False, "box_id": "", "pose": None}
        self._pickup_by_zone = {}
        self._outfeed = []                 # 반출 중인 덩어리 (팔레트 + 박스)
        self._shipped_n = 0                # 트럭에 남긴 팔레트 사본 번호
        self._stall = {}                   # box_id -> [x, y, 마지막으로 움직인 시각]
        self._stall_rects = None           # 리턴 조각 bbox 들 (첫 step 에서 lazy)

        self._ensure_roots()
        self._log("[spawner] 스폰점 %s m (밑면) · 픽업 판정 %s %.3f~%.3f%s "
                  "· 벨트 상면 %.3f · 경로 %s"
                  % (self.spawn_xyz_m, self.axis_name, self.pickup_p0_m,
                     self.stopper_pos_m,
                     ("" if self.cross_range_m is None
                      else " · %s %.3f~%.3f" % ("xyz"[self.cross_axis],
                                                self.cross_range_m[0],
                                                self.cross_range_m[1])),
                     self.belt_top_m, BOX_ROOT))

    # ── 등록 ────────────────────────────────────────────────
    def handlers(self):
        return {"spawn_box": self.h_spawn_box,
                "spawn_status": self.h_spawn_status,
                "set_spawn_enabled": self.h_set_spawn_enabled,
                "despawn_box": self.h_despawn_box,
                "swap_pallet": self.h_swap_pallet,
                "pickup_check": self.h_pickup_check,
                "get_box_pose": self.h_get_box_pose}

    # ── 메인 스레드 ─────────────────────────────────────────
    def step(self):
        """매 물리 스텝 한 번. **메인 스레드 전용.**

        큐 처리는 `sim_main` 이 `mainq.pump()` 로 한다 — 여기서는 스냅샷만 갱신한다.
        """
        self._refresh()
        self._stall_watch()
        if OUTFEED_ON:
            # 물리 60 Hz 고정이라 dt 는 상수다 (sim_main 이 그렇게 띄운다).
            self._outfeed_step(1.0 / 60.0)

    def _stall_watch(self):
        """리턴 라인 위에서 오래 제자리인 박스를 치운다 (모듈 상수 주석 참조)."""
        if STALL_DESPAWN_S <= 0:
            return
        if self._stall_rects is None:
            # 리턴 조각 = belt_velocity_raw 의 프림들. 그래프 프림엔 형상이 없어
            # 부모(ConveyorTrack_xx)의 bbox 를 잰다. 여유 0.1 m.
            rects = []
            raw = ((self.cfg.get("conveyor") or {}).get("belt_velocity_raw") or {})
            for gp in raw:
                bb = self._bbox(str(gp).rsplit("/", 1)[0])
                if bb:
                    rects.append((bb[0] - 0.1, bb[1] + 0.1,
                                  bb[2] - 0.1, bb[3] + 0.1))
            self._stall_rects = rects
            if rects:
                self._log("[spawner] 리턴 걸림 감시 켬 — 조각 %d 개 · %d초 제자리면 치운다"
                          % (len(rects), int(STALL_DESPAWN_S)))
        if not self._stall_rects:
            return
        now = time.monotonic()
        for box_id, e in list(self._snap.items()):
            p = e.get("pos_m")
            if not (p and any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1
                              for x0, x1, y0, y1 in self._stall_rects)):
                self._stall.pop(box_id, None)
                continue
            h = self._stall.get(box_id)
            if h is None or abs(p[0] - h[0]) + abs(p[1] - h[1]) > STALL_MOVE_M:
                self._stall[box_id] = [p[0], p[1], now]
                continue
            if now - h[2] >= STALL_DESPAWN_S:
                # ★ 리턴 벨트에서 정지는 정상 흐름에 없다 — 걸렸거나 뒤집혔다.
                #   뒤차는 이걸 치우면 다시 흐르기 시작해 타이머가 리셋된다.
                self._log("[spawner] %s 가 리턴에서 %d초째 제자리 — 걸림/전복으로 "
                          "보고 치운다" % (box_id, int(STALL_DESPAWN_S)))
                self._stall.pop(box_id, None)
                self._despawn(box_id)

    def _refresh(self):
        """모든 박스의 pose/속도를 한 번 읽어 스냅샷과 픽업 상태를 갱신한다.

        ★ **USD 에서 읽는다. `SingleRigidPrim` 을 박스마다 만들지 않는다.**

        예전엔 박스마다 `SingleRigidPrim` 을 만들어 `get_world_pose()` 를 불렀다.
        두 가지가 터졌다 (실측 2026-08-22):

          1) 시뮬레이션이 도는 중에 만든 뷰가 **엉뚱한 값**을 돌려줬다 —
             x=0.5 에 스폰한 박스를 x=10.3 으로 보고했다.
          2) 그 뷰가 물고 있는 프림을 지우면 **시뮬레이션 뷰 전체가 무효화**된다:
               "prim was deleted while being used by a shape in a tensor view class"
               "Simulation view object is invalidated ... getDofPositions"
             즉 박스 하나 despawn 에 **로봇 아티큘레이션까지 같이 죽는다.**
             실제로 프로세스가 그대로 내려갔다.

        USD 는 물리 스텝마다 갱신되므로 pose 는 그대로 읽으면 되고, 속도는
        프레임 간 차분으로 낸다. 스토퍼 판정(|vx| < 1 cm/s)에는 이 정밀도로 충분하다.
        """
        from pxr import UsdGeom as _UG
        cache = _UG.XformCache()
        snap = {}
        best = {}          # zone -> (box_id, entry)  ★ 존마다 따로 고른다
        lost = []
        for box_id, rec in list(self.boxes.items()):
            prim = self.stage.GetPrimAtPath(rec["path"])
            if not prim or not prim.IsValid():
                continue
            m = cache.GetLocalToWorldTransform(prim)
            t = m.ExtractTranslation()
            pos = [float(t[0]), float(t[1]), float(t[2])]
            q = m.ExtractRotationQuat()
            im = q.GetImaginary()
            yaw = _yaw_deg((q.GetReal(), im[0], im[1], im[2]))
            prev = self._snap.get(box_id)
            if prev is None:
                vel = [0.0, 0.0, 0.0]
            else:
                vel = [(pos[i] - prev["pos_m"][i]) / self.dt for i in range(3)]
            # ★ 라인 이탈 감지. 물리가 터지면(깊은 관통 -> 거대 임펄스) 박스가 초속
            #   수 km 로 날아간다. 그러면 `pickup_check` 가 영원히 그 박스를 못 보고,
            #   부르는 쪽은 "다음 박스가 안 온다" 며 타임아웃까지 헛기다린다.
            #   실측 2026-08-23: BOX00014 가 (-256577, -119448, 0.16) 에 있었고
            #   드라이버가 180 s 를 기다리다 중단했다. 조용히 사라지는 대신 알린다.
            if (abs(pos[0]) > LOST_XY_M or abs(pos[1]) > LOST_XY_M
                    or pos[2] < LOST_Z_MIN_M):
                lost.append((box_id, pos))
                continue
            entry = {"pos_m": pos, "yaw_deg": yaw, "vel_mps": vel}
            snap[box_id] = entry
            # 스토퍼 판정 — 박스 **앞면**이 기준이다 (§5.6)
            #
            # ★★ 조건이 셋이다. 예전에는 "앞면이 스토퍼 근처보다 앞" + "거의 정지"
            #    둘뿐이었는데, **위쪽 경계가 없어서 팔레트에 놓인 박스가 계속
            #    present 로 잡혔다.** 팔레트는 x 10300~11500 이라 스토퍼(10000)보다
            #    앞이고, 놓인 박스는 당연히 멎어 있다. 그래서 `best` 가 늘 팔레트 위
            #    박스가 되어 로봇이 **방금 놓은 것을 다시 집었다**
            #    (2026-08-23 실측: 15박스 연속 중 2번째 사이클부터 BOX00000 을 계속
            #     재차 집어 knock 338 mm, 새 박스는 영영 안 집혔다).
            #
            #    1) 중심이 픽업 구간 [pickup_range[0], stopper_x] 안 — 팔레트를 제외한다
            #    2) 앞면이 스토퍼 마진 안 — 실제로 벽에 닿아 섰다
            #    3) 밑면이 벨트 상면 — 집혀서 들린 것과 라인 밖으로 떨어진 것을 제외한다
            #    여기에 원래의 "거의 정지" 가 더해진다.
            # ★ 진행방향 폭은 축에 따라 다르다. dims = (w, d, h) 이고 스폰 yaw 가 0
            #   이므로 +X 진행이면 w(=dims[0]), +Y 진행이면 d(=dims[1]) 가 진행방향이다.
            #   즉 dims[axis] 로 쓰면 두 축 모두 맞는다.
            a = self.axis
            w, h = rec["dims"][a], rec["dims"][2]
            front = pos[a] + w / 2.0
            on_line = self.pickup_p0_m <= pos[a] <= self.stopper_pos_m
            at_stopper = front > self.stopper_pos_m - w / 2.0 - PICKUP_MARGIN_M
            on_belt = abs(pos[2] - (self.belt_top_m + h / 2.0)) < PICKUP_Z_TOL_M
            if not (on_line and at_stopper and on_belt
                    and abs(entry["vel_mps"][a]) < PICKUP_VEL_MPS):
                continue
            # ★ 어느 존의 지선인지 가른다. 창이 없으면(test1-1) 모든 존이 받는다.
            for _z in self.zones:
                cr = self.cross_by_zone.get(_z)
                if cr is not None:
                    c = pos[self.cross_axis]
                    if not (cr[0] <= c <= cr[1]):
                        continue
                # 여러 개가 줄 서면 **가장 앞선 것**이 집힐 차례다
                cur = best.get(_z)
                if cur is None or pos[a] > cur[1]["pos_m"][a]:
                    best[_z] = (box_id, entry)
        for box_id, pos in lost:
            self.boxes.pop(box_id, None)
            self.lost.append(box_id)
            self._log("[spawner] !! %s 가 라인을 벗어났다 %s — 추적에서 뺀다. "
                      "물리가 터진 것이다 (관통 또는 큐 과압축)."
                      % (box_id, [round(v, 1) for v in pos]))
        self._snap = snap
        pk = {}
        for _z in self.zones:
            b = best.get(_z)
            if b is None:
                pk[_z] = {"present": False, "box_id": "", "pose": None}
            else:
                box_id, e = b
                pk[_z] = {"present": True, "box_id": box_id,
                          "pose": {"pos_m": e["pos_m"], "yaw_deg": e["yaw_deg"]}}
        self._pickup_by_zone = pk
        self._pickup = pk[self.zone]

    # ── 소켓 핸들러 ─────────────────────────────────────────
    def h_spawn_box(self, req):
        """{"cmd":"spawn_box","box_id":"BOX00000","type":"4호","yaw_deg":0}"""
        box_id = req.get("box_id")
        tname = req.get("type")
        if not box_id:
            return {"ok": False, "error": "box_id 가 없다"}
        if tname not in self.types:
            return {"ok": False, "error": "모르는 type %r (있는 것: %s)"
                    % (tname, sorted(self.types))}
        if not self.enabled:
            return {"ok": False, "error": "spawn disabled"}
        if box_id in self.boxes:
            return {"ok": False, "error": "이미 있는 box_id %s" % box_id}
        yaw = float(req.get("yaw_deg", self.default_yaw))
        color = req.get("color")
        if color is not None and color not in self.colors:
            return {"ok": False, "error": "모르는 color %r (있는 것: %s)"
                    % (color, sorted(self.colors))}
        dest = req.get("dest")
        return self.mainq.run(lambda: self._spawn(box_id, tname, yaw, color, dest))

    def h_despawn_box(self, req):
        box_id = req.get("box_id")
        if box_id not in self.boxes:
            return {"ok": False, "error": "없는 box_id %s" % box_id}
        return self.mainq.run(lambda: self._despawn(box_id))

    def h_swap_pallet(self, req=None):
        """{"cmd":"swap_pallet","zone":"B"} -> {"ok":true,"removed":N}

        **가득 찬 팔레트를 통째로 내보내고 빈 팔레트를 놓는다.** 사양이 정한
        PALLET_FULL 처리 경로다 (트랙B 가이드 §상태도:
        "PALLET_FULL: /sim/set_spawn_enabled(false) -> swap_pallet -> /algo/reset -> 재가동").

        여기서 하는 일은 **팔레트 위에 실린 박스를 전부 라인 밖으로 치우는 것**이다.
        팔레트 프림 자체는 그대로 둔다 — 같은 자리에 빈 팔레트가 새로 온 것과 같다.
        (실제 셀에서는 반출 컨베이어가 팔레트째 밀어내고 새 팔레트가 들어온다.
         `/World/PalletOutfeed` 가 그 상판이다. 구동은 아직 안 넣었다.)

        ★ 부르는 쪽이 반드시 같이 해야 하는 것: `/algo/reset` 으로 하이트맵을 비운다.
          안 비우면 패커가 **없어진 박스 위에** 다음 자리를 잡는다.
        """
        # ★ 대소문자 정규화 (2026-08-27). 관제탑 토픽 규약은 소문자('zone_b')라
        #   소문자 'b' 가 올 수 있다 — 실측: 거부돼 판갈이가 통째로 실패했다.
        zone = str((req or {}).get("zone", self.zone)).upper()
        if zone not in self.zones:
            return {"ok": False, "error": "이 스포너는 존 %s 다 (요청 %s)"
                    % (",".join(self.zones), zone), "removed": 0}
        return self.mainq.run(lambda: self._swap_pallet(zone))

    def h_spawn_status(self, req=None):
        """{"spawned":N, "enabled":bool, ...} — 표(공통가이드 §3)의 두 필드 + 진단용.

        ⚠ 예전 `on_belt` 는 `len(self.boxes)` 였는데, 놓인 박스는 despawn 하지 않으므로
          **팔레트 위 박스까지 세고 있었다.** 백압 판단(`buffer_capacity`)에 그대로 쓰면
          늘 가득 찬 것으로 보인다. 실제로 벨트에 있는 것만 센다.
        """
        on_belt = queue = 0
        for box_id, e in self._snap.items():
            a = self.axis
            p_axis, z = e["pos_m"][a], e["pos_m"][2]
            rec = self.boxes.get(box_id)
            if rec is None:
                continue
            if abs(z - (self.belt_top_m + rec["dims"][2] / 2.0)) > PICKUP_Z_TOL_M:
                continue                       # 벨트 위가 아니다 (팔레트·그리퍼·라인 밖)
            if 0.0 <= p_axis <= self.stopper_pos_m:
                # ★ 픽업 구간 계수도 **자기 지선만** 센다. 지선이 여러 갈래인
                #   배치에서 남의 지선 박스를 세면 백압이 잘못 걸려, 이 존은
                #   비어 있는데도 투입이 막힌다.
                if (self.cross_range_m is not None
                        and p_axis >= self.pickup_p0_m
                        and not (self.cross_range_m[0]
                                 <= e["pos_m"][self.cross_axis]
                                 <= self.cross_range_m[1])):
                    continue
                on_belt += 1
                # ★ **뒷면**이 경계를 넘어야 픽업 구간에 든 것으로 센다.
                #   중심으로 세면 안 된다 — 박스가 경계에 걸친 순간 이미 1로 세어져
                #   계량 운전이 `main` 을 세우는데, 그때 박스는 대부분 아직 main
                #   위에 있어서 **멈춘 롤러가 붙잡는다.** b_pickup 이 밀어도 안
                #   움직여 라인이 통째로 굳는다 (2026-08-23 실측: 5호가 x=7.59 에서
                #   165 s 동안 정지, 처리 2/11).
                if p_axis - rec["dims"][a] / 2.0 >= self.pickup_p0_m:
                    queue += 1                 # 픽업 구간 = 사양의 버퍼
        # ★ 스폰 지점이 비었는가. `main` 을 세워 두는 계량(metering) 운전에서는
        #   앞차가 안 빠진 채 다음 박스를 내면 **같은 자리에 겹쳐 스폰된다.**
        #   벨트가 돌 때는 12 s 뒤면 6 m 밖이라 문제가 없었지만, 세워 두면 그대로
        #   남는다. 그래서 인터록을 하나 준다 (실물 인피드도 같은 걸 본다).
        spawn_clear = True
        for box_id, e in self._snap.items():
            rec = self.boxes.get(box_id)
            if rec is None:
                continue
            x, _y, z = e["pos_m"]
            if abs(z - (self.belt_top_m + rec["dims"][2] / 2.0)) > PICKUP_Z_TOL_M:
                continue
            if abs(x - self.spawn_xyz_m[0]) < SPAWN_CLEAR_M:
                spawn_clear = False
                break
        return {"spawned": self.spawned, "enabled": bool(self.enabled),
                "on_belt": on_belt, "queue": queue, "tracked": len(self.boxes),
                "spawn_clear": spawn_clear, "lost": list(self.lost)}

    def h_set_spawn_enabled(self, req):
        self.enabled = bool(req.get("enabled", True))
        self._log("[spawner] 스폰 %s" % ("ON" if self.enabled else "OFF"))
        return {"ok": True}

    def h_pickup_check(self, req=None):
        """스토퍼에 도착해 멎은 박스를 보고한다 (§5.6).

        ★ zone 을 받아만 두고 무시하면 안 된다. 이 스포너는 `sim_main --zone` 으로
          존 **하나**를 맡는데, 다른 존을 물어도 제 것을 정상 응답으로 돌려주면
          stage 2 에서 브릿지가 엉뚱한 존의 박스를 집으라고 지시하게 된다. 오류가
          아니라 정상 응답이라 로그만 봐서는 안 보인다. 존이 다르면 그건 "브릿지가
          포트를 잘못 잡았다" 는 뜻이고, 그게 보여야 한다.
          응답 모양은 §3 표대로 세 키를 다 채운다 — 브릿지가 r["box_id"] 를 읽어도
          KeyError 가 안 나야 한다.
        """
        zone = (req or {}).get("zone", self.zone)
        if zone not in self.zones:
            return {"present": False, "box_id": "", "pose": None,
                    "error": "이 스포너가 아는 존은 %s 다 (요청 %s)"
                             % (",".join(self.zones), zone)}
        return dict(self._pickup_by_zone.get(zone) or
                    {"present": False, "box_id": "", "pose": None})

    def h_get_box_pose(self, req):
        box_id = req.get("box_id")
        e = self._snap.get(box_id)
        if e is None:
            return {"ok": False, "error": "없는 box_id %s" % box_id}
        return {"ok": True, "pos_m": e["pos_m"], "yaw_deg": e["yaw_deg"]}

    # ── 내부 ────────────────────────────────────────────────
    def _ensure_roots(self):
        if not self.stage.GetPrimAtPath(BOX_ROOT):
            UsdGeom.Xform.Define(self.stage, BOX_ROOT)
        if not self.stage.GetPrimAtPath(LOOKS_ROOT):
            UsdGeom.Scope.Define(self.stage, LOOKS_ROOT)
        # ★ 물리 머티리얼. 정본은 씬 빌더(layout_to_spec_test1_1.py)가 만들지만,
        #   씬을 아직 다시 안 만든 경우를 위한 보험으로 여기서도 확인한다.
        #   이게 없으면 박스가 PhysX 기본 마찰로 돌고, system.yaml 의 0.6/0.5 는
        #   아무데도 안 걸린다 — 스택이 서느냐 미끄러지느냐를 직접 가르는 값이다.
        if not self.stage.GetPrimAtPath(PHYS_MAT_PATH):
            _pm = UsdShade.Material.Define(self.stage, PHYS_MAT_PATH)
            _m = UsdPhysics.MaterialAPI.Apply(_pm.GetPrim())
            _m.CreateStaticFrictionAttr(self.mu_s)
            _m.CreateDynamicFrictionAttr(self.mu_d)
            _m.CreateRestitutionAttr(0.0)
            self._log("[spawner] 물리 머티리얼을 새로 만들었다 %s (정지 %.2f · 운동 %.2f)"
                      " — 씬 빌더를 다시 돌리는 편이 낫다"
                      % (PHYS_MAT_PATH, self.mu_s, self.mu_d))
        # 기본 색 하나 + 분류용 색 머티리얼들.
        _mats = [(LINE_MAT, LINE_COLOR)]
        _mats += [(COLOR_MAT_FMT % c, rgb) for c, rgb in sorted(self.colors.items())]
        for mp, rgb in _mats:
            if self.stage.GetPrimAtPath(mp):
                continue
            mat = UsdShade.Material.Define(self.stage, mp)
            sh = UsdShade.Shader.Define(self.stage, mp + "/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")

    def _spawn(self, box_id, tname, yaw_deg, color=None, dest=None):
        t = self.types[tname]
        dims = (t["w_mm"] / 1000.0, t["d_mm"] / 1000.0, t["h_mm"] / 1000.0)
        mass = t["mass_g"] / 1000.0
        x, y, z_bottom = self.spawn_xyz_m
        pos = (x, y, z_bottom + dims[2] / 2.0)          # ★ 밑면 -> 중심 (§5.5)
        path = BOX_PRIM_FMT % box_id
        self._make_box(path, tname, dims, pos, yaw_deg, mass, box_id, color,
                       dest)
        # ★ 프림을 지우지 않으므로 같은 경로가 이미 있을 수 있다 — 그 프림은
        #   `_detach` 로 **콜라이더가 꺼지고 키네마틱에 보이지도 않는** 상태다
        #   (despawn 또는 park_all 을 거친 것). 그대로 두면 스폰은 됐는데 벨트를
        #   뚫고 지나가거나 아예 안 보인다. 되살린다.
        #   stage 2 의 재순환(같은 박스를 다시 라인에 올리는 것)에도 이게 필요하다.
        _prim = self.stage.GetPrimAtPath(path)
        if _prim and _prim.IsValid():
            _c = UsdPhysics.CollisionAPI(_prim)
            if _c:
                _c.CreateCollisionEnabledAttr(True)
            _rb = UsdPhysics.RigidBodyAPI(_prim)
            if _rb:
                _rb.CreateKinematicEnabledAttr(False)
            UsdGeom.Imageable(_prim).MakeVisible()
        self.boxes[box_id] = {"path": path, "dims": dims, "type": tname,
                              "color": color, "dest": dest}
        self.spawned += 1
        self._log("[spawner] %s %s dest=%s %dx%dx%d mm %.1f kg yaw %+.0f -> %s"
                  % (box_id, tname, dest or "-", t["w_mm"], t["d_mm"],
                     t["h_mm"], mass, yaw_deg, path))
        return {"ok": True}

    def _detach(self, prim):
        """물리에서 떼어낸다. **콜라이더를 먼저 끄고** 키네마틱으로 바꾼다.

        ★ 순서가 중요하다. 콜라이더를 켠 채로 키네마틱 전환 + 순간이동을 하면,
          그 박스에 눌려 있던 이웃들이 접촉이 갑자기 풀리며 솔버가 폭발한다.
          실측 2026-08-23 (results/stack15_volume_fail_20260823.log): 픽업 구간에
          4개가 줄 서 있을 때 선두를 despawn 했더니 뒤 4개가 전부
          x ≈ -220 m 로 튕겨 나가 라인에서 사라졌다. 콜라이더를 먼저 끄면
          이웃은 그냥 접촉을 잃을 뿐이라 그 충격이 없다.
          가시성도 끈다 — 눈으로 봐도 사라진 것과 같아야 한다.
        """
        c = UsdPhysics.CollisionAPI(prim)
        if c:
            c.CreateCollisionEnabledAttr(False)
        rb = UsdPhysics.RigidBodyAPI(prim)
        if rb:
            rb.CreateKinematicEnabledAttr(True)
        UsdGeom.Imageable(prim).MakeInvisible()

    def park_all(self):
        """박스 프림을 **전부** 라인 밖으로 치운다. 타임라인 Stop 뒤 정리용.

        ★ 왜 필요한가 — 이 스포너는 프림을 **지우지 않는다** (시뮬 중에 물리 프림을
          지우면 PhysX 텐서 뷰가 무효화돼 로봇 아티큘레이션까지 죽는다. `_despawn`
          docstring 참조). 그래서 15개 프림이 스테이지에 그대로 남는다.
          그런데 타임라인을 Stop 하면 Isaac 이 시뮬레이션 상태를 버리고 **USD 에
          적혀 있는 자세**로 되돌린다. 팔레트에 놓인 박스는 PhysX 가 옮긴 것일 뿐
          USD 에는 스폰 당시 좌표가 적혀 있어서, Stop 하는 순간 전부 스폰 지점으로
          되돌아가 거기 겹쳐 선다. 그 상태로 Play 를 누르면 벨트(OmniGraph 라
          드라이버와 무관하게 돈다)가 그것들을 실어 나른다 — "안 나온 박스가
          갑자기 나와서 움직이는" 것처럼 보인다.
          그래서 Stop 이 감지되면 여기서 USD 좌표 자체를 라인 밖으로 적어 둔다.
        """
        root = self.stage.GetPrimAtPath(BOX_ROOT)
        n = 0
        if root and root.IsValid():
            for prim in root.GetChildren():
                if not prim.IsValid():
                    continue
                self._detach(prim)
                xf = UsdGeom.Xformable(prim)
                for op in xf.GetOrderedXformOps():
                    if op.GetOpName() == "xformOp:translate":
                        op.Set(Gf.Vec3d(DESPAWN_XYZ_M))
                        break
                n += 1
        # ★ 제자리에서 비우면 안 된다. `h_spawn_status` 는 **소켓 스레드**에서
        #   `self._snap.items()` 를 순회한다. 그 dict 를 메인 스레드가 줄이면
        #   RuntimeError: dictionary changed size during iteration 이 나고,
        #   `sim_server` 가 그걸 error 응답으로 삼켜 호출자는 기본값으로 떨어진다 —
        #   계량은 queue=0 으로 읽어 이미 한 대 있는데 main 을 다시 돌리고, 스폰은
        #   백압을 건너뛰어 겹쳐 스폰한다. `_refresh` 처럼 **통째로 교체**한다
        #   (그래서 읽기에 락이 없어도 되는 것이다).
        self.boxes = {}
        self._snap = {}
        self._pickup = {"present": False, "box_id": "", "pose": None}
        return n

    def _despawn(self, box_id):
        """박스를 작업 영역에서 치운다.

        ⚠ **프림을 지우지 않는다.** 시뮬레이션이 도는 중에 물리 프림을 지우면
          PhysX 텐서 뷰가 통째로 무효화돼 **로봇 아티큘레이션까지 죽는다**
          (실측 2026-08-22, `_refresh` docstring 참조). 그래서 대신
          작업 영역 밖으로 옮기고 목록에서 뺀다. 겉보기·계측 모두 사라진 것과
          같고, 프림은 다음 `world.reset()` 때 정리하면 된다.

        Day 4 재순환(`/sim/despawn_box`, §2.6)에는 이 동작으로 충분하다.
        """
        rec = self.boxes.pop(box_id, None)
        if rec is None:
            return {"ok": False, "error": "없는 box_id %s" % box_id}
        prim = self.stage.GetPrimAtPath(rec["path"])
        if prim and prim.IsValid():
            self._detach(prim)
            xf = UsdGeom.Xformable(prim)
            for op in xf.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    op.Set(Gf.Vec3d(DESPAWN_XYZ_M))
                    break
        # 제자리 삭제가 아니라 교체다 — 이유는 park_all 주석 참고.
        self._snap = {k: v for k, v in self._snap.items() if k != box_id}
        self._log("[spawner] despawn %s (라인 밖으로 이동)" % box_id)
        return {"ok": True}

    def _swap_pallet(self, zone=None):
        """**메인 스레드 전용.** **그 존** 팔레트 위 박스를 전부 치운다.

        ★ zone 을 반드시 받는다. 안 받으면 존이 셋이어도 늘 같은 팔레트를 비운다
          — 실측 2026-08-26: B 를 상차했는데 A 가 초기화됐다.

        ★ 반출 연출 (`PL_OUTFEED=1`, 2026-08-27 김병직): 박스를 그 자리에서
          지우면 화면에서 **한 프레임에 사라진다** — 사양 §7 의 "트럭 상차" 가
          시연에 안 보인다. 켜면 박스를 지우지 않고 **키네마틱으로 바꿔 반출
          방향으로 밀어**, 팔레트가 통째로 컨베이어를 타고 나가는 것처럼 보인다.
          다 나가면 그때 despawn 한다 (`_outfeed_step` 이 매 틱 민다).
          기본은 꺼짐 — 켜야만 동작이 바뀐다.
        """
        zone = str(zone or self.zone).upper()
        box = self.pallet_box.get(zone)
        if box is None:
            return {"ok": False, "removed": 0,
                    "error": "존 %s 의 팔레트 범위를 모른다 (아는 것: %s)"
                             % (zone, sorted(self.pallet_box))}
        (px0, px1), (py0, py1), ptop = box
        on_pallet = []
        for box_id, e in list(self._snap.items()):
            x, y, z = e["pos_m"]
            if px0 <= x <= px1 and py0 <= y <= py1 and z >= ptop - 0.05:
                on_pallet.append(box_id)
        if OUTFEED_ON:
            self._outfeed_begin(zone, on_pallet)
            self._log("[spawner] swap_pallet %s — 팔레트와 박스 %d 개를 반출 컨베이어로 "
                      "내보낸다 (+Y %.1f m) %s"
                      % (zone, len(on_pallet), OUTFEED_TRAVEL_M, on_pallet))
            return {"ok": True, "removed": len(on_pallet), "zone": zone}
        for box_id in on_pallet:
            self._despawn(box_id)
        self._log("[spawner] swap_pallet %s — 박스 %d 개를 팔레트째 내보냈다 %s"
                  % (zone, len(on_pallet), on_pallet))
        return {"ok": True, "removed": len(on_pallet), "zone": zone}

    # ── 반출 연출 (PL_OUTFEED=1) ─────────────────────────────
    @staticmethod
    def _translate_op(prim):
        """프림의 xformOp:translate 를 돌려준다. 없으면 None."""
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                return op
        return None

    @staticmethod
    def _orient_op(prim):
        """프림의 xformOp:orient 를 돌려준다. 없으면 None."""
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpName() == "xformOp:orient":
                return op
        return None

    def _rotate_prim_about_z(self, prim, cx, cy, dphi):
        """프림을 (cx, cy) 둘레로 Z 축 `dphi` [rad] 만큼 돌린다.

        위치와 **자세를 같이** 돌린다. 자세를 안 돌리면 팔레트가 게걸음으로
        커브를 지나 — 돌고 나면 가로 1.2 m 인 채 폭 0.83 m 레인에 들어가
        가이드를 뚫는다 (2026-08-27 김병직 관측 "물리적으로 말이 안 된다").
        """
        op_t = self._translate_op(prim)
        if op_t is None:
            return
        t = op_t.Get()
        c, s = math.cos(dphi), math.sin(dphi)
        dx, dy = t[0] - cx, t[1] - cy
        op_t.Set(Gf.Vec3d(cx + c * dx - s * dy, cy + s * dx + c * dy, t[2]))

        op_o = self._orient_op(prim)
        if op_o is None:
            return
        cur = op_o.Get()
        if cur is None:
            return
        try:
            half = dphi / 2.0
            qd = Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half)))
            new = qd * Gf.Quatd(cur.GetReal(), Gf.Vec3d(*cur.GetImaginary()))
            im = type(cur.GetImaginary())(*new.GetImaginary())
            op_o.Set(type(cur)(new.GetReal(), im))
        except Exception:
            pass          # 자세를 못 돌려도 이송은 계속한다

    def _bbox(self, path):
        """월드 bbox (x0, x1, y0, y1). 프림이 없거나 비면 None."""
        prim = self.stage.GetPrimAtPath(path)
        if not (prim and prim.IsValid()):
            return None
        rng = UsdGeom.Imageable(prim).ComputeWorldBound(
            Usd.TimeCode.Default(), "default").ComputeAlignedRange()
        if rng.IsEmpty():
            return None
        mn, mx = rng.GetMin(), rng.GetMax()
        return (mn[0], mx[0], mn[1], mx[1])

    def _outfeed_begin(self, zone, box_ids):
        """팔레트와 그 위 박스를 한 덩어리로 반출 대기열에 넣는다. **메인 스레드 전용.**

        ★ 팔레트도 같이 움직인다 (2026-08-27 김병직 — "팔레트와 같이 아래 컨베이어를
          타고 내려가는" 그림). 팔레트만 두고 박스만 흘리면 실물과 정반대다.
          팔레트 프림은 이미 **키네마틱**이라(씬 실측) translate 를 밀어도 안전하고,
          다 나가면 **제자리로 되돌린다** — 그게 "빈 팔레트가 새로 들어온" 그림이다.
          ⚠ 디스크의 USD 를 고치는 것이 아니라 **런타임 변환만** 바꾼다.
        ★ 박스를 키네마틱으로 바꾸는 이유: 팔레트 위 더미를 물리로 밀면 위층이
          무너지며 쏟아진다. 키네마틱이면 쌓인 모양 그대로 간다.
        ★ 콜라이더는 끈다. 켜 둔 채 지나가면 지선 컨베이어·기둥과 부딪혀
          더미가 흐트러진다 (`_detach` 와 같은 이유).
        ★ 목록에서는 지금 뺀다 (`self.boxes`) — 반출 중인 박스를 다음 사이클이
          픽업 후보로 보면 안 된다. 프림 경로는 이 대기열이 들고 있다.
        """
        train = {"zone": zone, "boxes": {}, "moved": 0.0,
                 "pallet": None, "home": None, "reveal": None,
                 "phase": "align", "turn_y": None, "end_x": None, "hold": 0.0,
                 "arc": None, "arc_done": 0.0, "align_left": 0.0,
                 "lane_y": None}
        for box_id in box_ids:
            rec = self.boxes.pop(box_id, None)
            if rec is None:
                continue
            prim = self.stage.GetPrimAtPath(rec["path"])
            if not (prim and prim.IsValid()):
                continue
            c = UsdPhysics.CollisionAPI(prim)
            if c:
                c.CreateCollisionEnabledAttr(False)
            rb = UsdPhysics.RigidBodyAPI(prim)
            if rb:
                rb.CreateKinematicEnabledAttr(True)
            train["boxes"][box_id] = rec["path"]
            self._snap.pop(box_id, None)

        pal_path = PALLET_PRIM_FMT % zone
        pal = self.stage.GetPrimAtPath(pal_path)
        op = self._translate_op(pal) if (pal and pal.IsValid()) else None
        if op is not None:
            # ★★ 나가는 것은 **사본**이고 원본은 제자리에 남는다 (2026-08-27 정정인).
            #   원본이 왕복(10~24초 + 정차)하는 동안 로봇 옆자리가 비는데, 그 사이
            #   사이클이 재개되면 다음 박스가 **셀 컨베이어에 바로 얹힌다** —
            #   계획 z 가 팔레트 상면(950) 기준이라서다 (실측 재현, 정정인).
            #   원본을 남겨 두면 콜라이더가 그대로라 언제 놓아도 제 높이에 앉는다.
            #
            #   보이는 그림: 출발 순간 원본을 **숨겼다가** OUTFEED_RESPAWN_S(3초) 뒤
            #   다시 보인다 — "빈 팔레트가 새로 들어온" 연출은 유지된다.
            #   콜라이더는 **안 끈다.** 숨긴 3초 사이 놓인 박스가 잠깐 떠 보이는
            #   쪽이, 컨베이어로 떨어지는(이 수정이 잡는 바로 그 버그) 쪽보다 낫다.
            #
            #   반출이 끝나면 사본은 트럭에 그대로 남는다 — 예전 finish 의
            #   "사본 뜨고 원본 복귀" 두 단계가 필요 없어진다.
            self._shipped_n += 1
            dst = "/World/PalletShipped_%s_%d" % (zone, self._shipped_n)
            try:
                lay = self.stage.GetEditTarget().GetLayer()
                Sdf.CopySpec(lay, Sdf.Path(pal_path), lay, Sdf.Path(dst))
                train["pallet"] = dst              # 사본이 반출 열차를 탄다
                UsdGeom.Imageable(pal).MakeInvisible()
                train["reveal"] = [pal_path, OUTFEED_RESPAWN_S]
            except Exception as e:
                # 사본을 못 뜨면 예전 동작(원본이 나갔다 돌아온다)으로 물러선다.
                self._log("[spawner] ⚠ 반출: 팔레트 사본 실패(%s) — 원본이 나간다" % e)
                train["pallet"], train["home"] = pal_path, op.Get()
                # ★ 자세도 같이 기억한다 (김병직 관측) — 호에서 90도 돌아간 채로
                #   위치만 되돌리면 새 빈 팔레트가 옆으로 누워 있다.
                _oo = self._orient_op(pal)
                train["home_q"] = _oo.Get() if _oo is not None else None
        else:
            self._log("[spawner] ⚠ 반출: 팔레트 프림 %s 를 못 찾았다 — 박스만 내보낸다"
                      % pal_path)

        # ── 상차 경로: 호는 **설정**에서, 트럭은 **씬**에서 (상수 주석 참조) ──
        arc = ((self.cfg.get("conveyor") or {}).get("curve_arc") or {}).get(zone)
        bed = self._bbox(TRUCK_BED_PRIM)
        if arc and bed:
            cx = float(arc["center_mm"][0]) / 1000.0
            cy = float(arc["center_mm"][1]) / 1000.0
            R = float(arc["radius_mm"]) / 1000.0
            train["end_x"] = (bed[0] + bed[1]) / 2.0
            # ★ 호를 나온 뒤 **레인 한가운데로 Y 를 다시 맞춘다** (김학민도 같은
            #   보정을 했다). 호 끝과 레인 중심이 15~39 mm 어긋나는데, 돌고 난
            #   팔레트 폭 0.80 에 레인 간격이 0.83 이라 **편도 여유가 15 mm뿐**이다
            #   — 안 맞추면 C 존이 가이드를 문다.
            lo = self._bbox(PALLET_GUIDE_FMT % (zone, "lo"))
            hi = self._bbox(PALLET_GUIDE_FMT % (zone, "hi"))
            if lo and hi:
                train["lane_y"] = ((lo[2] + lo[3]) + (hi[2] + hi[3])) / 4.0
                # ★★ 호 중심 y 는 설정이 아니라 **레인에서 유도한다** (2026-08-27
                #   정정인). 새 씬(배경판)에서 반출 어셈블리가 ~0.3 m 옮겨져
                #   config 의 호가 씬과 어긋났다 — 출구가 +283~335 mm 벗어나
                #   팔레트가 가이드에 270 mm 겹친 채 옆으로 갈리며 들어갔다
                #   (실측: 옛 씬 레인 정렬 +15~35 mm · 새 씬 +283~335 mm).
                #   출구 y = cy + R 이므로 cy = lane_y - R 로 두면 출구가
                #   **정의상 0** 이고, 씬이 또 옮겨져도 따라간다. 레인 중심은
                #   이미 씬 프림(PalletGuide bbox)에서 재고 있다.
                #   가이드가 없으면(폴백) config 값 그대로 쓴다.
                # ★★★ 반지름도 **씬에서 잰다** (2026-08-27 저녁, 정정인 실측).
                #   위 보정만으로는 안 맞았다. 실측: 배경 씬과 옛 씬이 트럭 위치
                #   (x 7.04 vs -1.60)까지 다른데 레인 어긋남은 **mm 단위로 같다**
                #   (B +268 · C +300 · A +302). 씬이 옮겨진 게 아니라 **config 의
                #   radius_mm 이 원래 씬 기하와 다르다**는 뜻이다.
                #   회전은 거리를 보존하므로 출구 y = cy + (진입 시 중심까지 거리)다.
                #   그 거리는 설정 R 이 아니라 **팔레트 home 에서 호 중심까지**이고,
                #   역산하면 1.91 이 아니라 약 1.61 이었다 — 차이가 곧 300 mm 다.
                #   그래서 R 대신 실측 d 를 쓴다. d == R 인 씬에서는 아무 변화 없다.
                #   ⚠ home 이 없으면(팔레트 프림 못 찾음) 예전대로 R 을 쓴다.
                _d = (abs(train["home"][0] - cx)
                      if train["home"] is not None else R)
                cy_scene = train["lane_y"] - _d
                if abs(_d - R) > 0.02:
                    self._log("[spawner] 반출 %s — 호 반지름을 씬에서 잰다 "
                              "%.3f -> %.3f (설정과 %+.0f mm 차 · 레인 어긋남의 정체)"
                              % (zone, R, _d, (_d - R) * 1000))
                if abs(cy_scene - cy) > 0.05:
                    self._log("[spawner] 반출 %s — 호 중심 y 를 레인에 맞춘다 "
                              "%.2f -> %.2f (씬-설정 어긋남 %+.0f mm)"
                              % (zone, cy, cy_scene, (cy_scene - cy) * 1000))
                cy = cy_scene
                # 회전은 이 반지름으로 돈다 — cy 와 짝이 안 맞으면 다시 어긋난다.
                R = _d
            train["arc"] = (cx, cy, R)
            train["turn_y"] = cy                  # 호 진입점의 y
            if train["home"] is not None:
                # 진입 중심선(x = 중심 - R)까지 옆으로 얼마나 가야 하나
                train["align_left"] = (cx - R) - train["home"][0]
            self._log("[spawner] 반출 %s — 호 중심 (%.2f, %.2f) R %.2f · "
                      "진입 y %.2f · 트럭 x %.2f · 옆정렬 %+.0f mm"
                      % (zone, cx, cy, R, cy, train["end_x"],
                         train["align_left"] * 1000))
        else:
            # 못 재면 예전 동작(직진)으로 물러선다. 데모가 서는 것보다 낫다.
            train["phase"] = "y"
            self._log("[spawner] ⚠ 반출: 호 설정(%s) 또는 트럭(%s) 을 못 찾았다 "
                      "— 직진 %.1f m 로 대신한다"
                      % (bool(arc), bool(bed), OUTFEED_TRAVEL_M))
        self._outfeed.append(train)

    def _outfeed_step(self, dt):
        """반출 중인 덩어리를 반출 방향(+Y)으로 민다. **메인 스레드 전용.**

        `OUTFEED_TRAVEL_M` 만큼 가면 박스는 despawn 하고 팔레트는 제자리로
        되돌린다 — 안 그러면 프림이 무한히 멀어지며 남는다.
        """
        if not self._outfeed:
            return
        step = OUTFEED_MPS * dt
        for train in list(self._outfeed):
            # ── 빈 팔레트 재투입: 출발 OUTFEED_RESPAWN_S 뒤 원본을 다시 보인다 ──
            rv = train.get("reveal")
            if rv is not None:
                rv[1] -= dt
                if rv[1] <= 0.0:
                    p = self.stage.GetPrimAtPath(rv[0])
                    if p and p.IsValid():
                        UsdGeom.Imageable(p).MakeVisible()
                    train["reveal"] = None
                    self._log("[spawner] 반출 %s — 빈 팔레트 투입 (출발 %.1f초 뒤)"
                              % (train["zone"], OUTFEED_RESPAWN_S))
            # ── 트럭에 들어간 뒤 잠깐 세워 둔다 (상차 장면) ──
            if train["phase"] == "hold":
                train["hold"] += dt
                if train["hold"] < OUTFEED_HOLD_S:
                    continue
                self._outfeed_finish(train)
                continue

            prims = self._outfeed_prims(train)
            done = False

            # ── ③ 커브: 호를 따라 돌면서 **자세도 같이 90° 돈다** ──
            if train["phase"] == "arc":
                cx, cy, R = train["arc"]
                dphi = -step / R          # 시계 방향 (φ 180도 -> 90도)
                left = (math.pi / 2.0) - train["arc_done"]
                if abs(dphi) > left:
                    dphi = -left
                train["arc_done"] += abs(dphi)
                for prim in prims:
                    self._rotate_prim_about_z(prim, cx, cy, dphi)
                if train["arc_done"] >= math.pi / 2.0 - 1e-6:
                    pal_t = self._outfeed_pallet_t(train)
                    if train["lane_y"] is not None and pal_t is not None:
                        train["align_left"] = train["lane_y"] - pal_t[1]
                        train["phase"] = "lane"
                        self._log("[spawner] 반출 %s — 커브 통과 (90도) · 레인 정렬 "
                                  "%+.0f mm" % (train["zone"],
                                                train["align_left"] * 1000))
                    else:
                        train["phase"] = "x"
                        self._log("[spawner] 반출 %s — 커브 통과 (90도) · 트럭으로"
                                  % train["zone"])
                continue

            # ── 그 밖: 축 이동 ──
            dx = dy = 0.0
            if train["phase"] == "align":
                # ① 호 진입 중심선으로 옆정렬. 한 틱에 옮기면 순간이동으로 보인다
                #    (김학민 `glide` 주석) — 반출 속도로 흘려보낸다.
                left = train["align_left"]
                dx = math.copysign(min(step, abs(left)), left) if left else 0.0
                train["align_left"] = left - dx
                if abs(train["align_left"]) < 1e-4:
                    train["phase"] = "y"
            elif train["phase"] == "lane":
                # ③-b 호를 나온 뒤 레인 한가운데로 Y 보정 (여유 15 mm)
                left = train["align_left"]
                dy = math.copysign(min(step, abs(left)), left) if left else 0.0
                train["align_left"] = left - dy
                if abs(train["align_left"]) < 1e-4:
                    train["phase"] = "x"
            elif train["phase"] == "y":
                dy = step                 # ② 셀 컨베이어를 타고 호 진입점까지
            else:
                dx = step                 # ④ 합류 레인을 타고 트럭까지

            for prim in prims:
                op = self._translate_op(prim)
                if op is None:
                    continue
                t = op.Get()
                op.Set(Gf.Vec3d(t[0] + dx, t[1] + dy, t[2]))

            pal_t = self._outfeed_pallet_t(train)
            if pal_t is None:             # 팔레트가 없다 — 직진 폴백
                train["moved"] += step
                done = train["moved"] >= OUTFEED_TRAVEL_M
            elif train["turn_y"] is None:
                train["moved"] += step
                done = train["moved"] >= OUTFEED_TRAVEL_M
            elif train["phase"] == "y" and pal_t[1] >= train["turn_y"]:
                train["phase"] = "arc"
                self._log("[spawner] 반출 %s — 커브 진입 (y %.2f)"
                          % (train["zone"], pal_t[1]))
            elif train["phase"] == "x" and pal_t[0] >= train["end_x"]:
                train["phase"] = "hold"
                self._log("[spawner] 반출 %s — 트럭 적재 완료 (x %.2f)"
                          % (train["zone"], pal_t[0]))

            if done:
                self._outfeed_finish(train)

    def _outfeed_prims(self, train):
        """반출 덩어리의 프림 목록 (팔레트 + 적재물). 사라진 박스는 정리한다."""
        out = []
        for box_id, path in list(train["boxes"].items()):
            prim = self.stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                out.append(prim)
            else:
                train["boxes"].pop(box_id, None)
        if train["pallet"]:
            pal = self.stage.GetPrimAtPath(train["pallet"])
            if pal and pal.IsValid():
                out.append(pal)
        return out

    def _outfeed_pallet_t(self, train):
        """팔레트의 지금 위치. 없으면 None."""
        if not train["pallet"]:
            return None
        pal = self.stage.GetPrimAtPath(train["pallet"])
        if not (pal and pal.IsValid()):
            return None
        op = self._translate_op(pal)
        return op.Get() if op is not None else None

    def _outfeed_finish(self, train):
        """반출 끝 — **실린 팔레트는 트럭에 남기고**, 빈 팔레트를 제자리에 놓는다.

        ★ 방법은 김학민 `e8a6c84` 그대로다 (`demo_stack15_test1_1.py` 의
          `_h_copy_prim`). 원본을 그냥 되돌리면 **트럭 위가 비어서 "상차했다"
          가 화면에 안 남는다.** 그래서
            ① 지금 자리(트럭 위)에 `Sdf.CopySpec` 으로 **사본**을 뜨고
            ② **원본**을 로봇 옆 제자리로 되돌린다
          순서가 중요하다 — 반대로 하면 사본이 빈 팔레트 자리에 생긴다.
        ★ 적재물 박스는 despawn 하지 않는다. 이미 트럭 위로 옮겨져 있고,
          그대로 둬야 "실려 있는" 그림이 남는다. 키네마틱·콜라이더 꺼짐이라
          물리에 관여하지 않고, `self.boxes`·`_snap` 에서는 이미 빠져 있어
          다음 사이클이 픽업 후보로 보지도 않는다.
        """
        # ★ 사본 모드(기본)에서는 여기서 할 일이 거의 없다 — 나간 것이 사본이라
        #   그대로 트럭에 남기면 되고, 빈 팔레트(원본)는 출발 3초 뒤 이미 보였다.
        #   아래 "사본 뜨고 원본 복귀" 블록은 enqueue 때 사본이 실패한
        #   **폴백(원본이 나간 경우)** 에만 돈다 — home 이 그때만 채워진다.
        rv = train.get("reveal")
        if rv is not None:                 # 여정이 3초보다 짧았던 극단 보호
            p = self.stage.GetPrimAtPath(rv[0])
            if p and p.IsValid():
                UsdGeom.Imageable(p).MakeVisible()
            train["reveal"] = None
        if train["pallet"] and train["home"] is not None:
            pal = self.stage.GetPrimAtPath(train["pallet"])
            if pal and pal.IsValid():
                self._shipped_n += 1
                dst = "/World/PalletShipped_%s_%d" % (train["zone"],
                                                      self._shipped_n)
                try:
                    lay = self.stage.GetEditTarget().GetLayer()
                    Sdf.CopySpec(lay, Sdf.Path(train["pallet"]),
                                 lay, Sdf.Path(dst))
                except Exception as e:      # 사본 실패해도 복귀는 해야 한다
                    self._log("[spawner] ⚠ 반출: 팔레트 사본 실패 — %s" % e)
                op = self._translate_op(pal)
                if op is not None:
                    op.Set(train["home"])
                # ★ 자세도 되돌린다 — 호에서 90도 돈 채로 두면 새 빈 팔레트가
                #   옆으로 누워 나온다 (2026-08-27 김병직 관측).
                _oo = self._orient_op(pal)
                if _oo is not None and train.get("home_q") is not None:
                    _oo.Set(train["home_q"])
        if train in self._outfeed:
            self._outfeed.remove(train)
        self._log("[spawner] 반출 완료 %s — 트럭에 팔레트를 남기고 빈 판을 놓았다 "
                  "(적재물 %d 개)" % (train["zone"], len(train["boxes"])))

    def _attach_label(self, box_path, box_id, hz):
        """박스 윗면에 QR 스티커(시각 전용 쿼드)를 붙인다.

        ★ 콜라이더를 걸지 않는다. 걸면 흡착 패드가 1 mm 떠 있는 스티커에 먼저
          닿아 박스를 그만큼 덜 눌러 잡는다 — 들다 놓치는 쪽이다.

        ★ 부모(박스)에 RigidBodyAPI 가 걸려 있지만 이 자식은 CollisionAPI 가
          없으므로 물리에 참여하지 않는다. 순수 시각 프림이다.
        """
        png = os.path.join(self.label_dir, box_id + ".png")
        if not os.path.exists(png):
            if not self._label_warned:      # 15 개가 다 없으면 로그가 15 줄 나온다
                self._label_warned = True
                self._log("[spawner] ⚠ QR 라벨 PNG 가 없다: %s\n"
                          "           먼저: python3 scripts/gen_qr_labels.py\n"
                          "           라벨 없이 계속한다 — cam1 이 등록을 못 해 "
                          "전 박스가 RETURN 된다" % png)
            return

        half = self.label_m / 2.0
        lp = box_path + "/" + LABEL_PRIM
        m = UsdGeom.Mesh.Define(self.stage, lp)
        z = hz + LABEL_LIFT_M
        m.CreatePointsAttr([Gf.Vec3f(-half, -half, z), Gf.Vec3f(half, -half, z),
                            Gf.Vec3f(half, half, z), Gf.Vec3f(-half, half, z)])
        m.CreateFaceVertexCountsAttr([4])
        m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        m.CreateExtentAttr([Gf.Vec3f(-half, -half, z), Gf.Vec3f(half, half, z)])
        m.CreateSubdivisionSchemeAttr("none")
        # ★ st 는 정점 순서와 짝을 맞춘다. v 를 뒤집는 이유는 LABEL_FLIP_V 주석 참조.
        v0, v1 = (1.0, 0.0) if LABEL_FLIP_V else (0.0, 1.0)
        st = UsdGeom.PrimvarsAPI(m.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set([Gf.Vec2f(0.0, v0), Gf.Vec2f(1.0, v0),
                Gf.Vec2f(1.0, v1), Gf.Vec2f(0.0, v1)])

        if not self.stage.GetPrimAtPath(QR_LOOKS_ROOT):
            UsdGeom.Scope.Define(self.stage, QR_LOOKS_ROOT)
        mp = QR_LOOKS_ROOT + "/" + box_id
        mat = UsdShade.Material.Define(self.stage, mp)
        sh = UsdShade.Shader.Define(self.stage, mp + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        # ★ 종이 라벨이라 반사가 거의 없어야 한다. 광택이 있으면 조명이 QR 위에
        #   하이라이트를 만들어 흰 모듈과 검은 모듈의 대비가 무너진다.
        sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.02, 0.02, 0.02))

        reader = UsdShade.Shader.Define(self.stage, mp + "/stReader")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        tex = UsdShade.Shader.Define(self.stage, mp + "/Tex")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(png)
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result")
        # ★ clamp. repeat 로 두면 가장자리에서 QR 이 한 줄 되풀이돼 흰 여백을
        #   갉아먹고, 여백이 좁아지면 디텍터가 QR 경계를 못 잡는다.
        for w in ("wrapS", "wrapT"):
            tex.CreateInput(w, Sdf.ValueTypeNames.Token).Set("clamp")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tex.ConnectableAPI(), "rgb")
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")

        UsdShade.MaterialBindingAPI.Apply(m.GetPrim())
        UsdShade.MaterialBindingAPI(m.GetPrim()).Bind(mat)

    def _attach_label(self, box_path, box_id, hz):
        """박스 윗면에 QR 스티커(시각 전용 쿼드)를 붙인다.

        ★ 콜라이더를 걸지 않는다. 걸면 흡착 패드가 1 mm 떠 있는 스티커에 먼저
          닿아 박스를 그만큼 덜 눌러 잡는다 — 들다 놓치는 쪽이다.

        ★ 부모(박스)에 RigidBodyAPI 가 걸려 있지만 이 자식은 CollisionAPI 가
          없으므로 물리에 참여하지 않는다. 순수 시각 프림이다.
        """
        png = os.path.join(self.label_dir, box_id + ".png")
        if not os.path.exists(png):
            if not self._label_warned:      # 15 개가 다 없으면 로그가 15 줄 나온다
                self._label_warned = True
                self._log("[spawner] ⚠ QR 라벨 PNG 가 없다: %s\n"
                          "           먼저: python3 scripts/gen_qr_labels.py\n"
                          "           라벨 없이 계속한다 — cam1 이 등록을 못 해 "
                          "전 박스가 RETURN 된다" % png)
            return

        half = self.label_m / 2.0
        lp = box_path + "/" + LABEL_PRIM
        m = UsdGeom.Mesh.Define(self.stage, lp)
        z = hz + LABEL_LIFT_M
        m.CreatePointsAttr([Gf.Vec3f(-half, -half, z), Gf.Vec3f(half, -half, z),
                            Gf.Vec3f(half, half, z), Gf.Vec3f(-half, half, z)])
        m.CreateFaceVertexCountsAttr([4])
        m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        m.CreateExtentAttr([Gf.Vec3f(-half, -half, z), Gf.Vec3f(half, half, z)])
        m.CreateSubdivisionSchemeAttr("none")
        # ★ st 는 정점 순서와 짝을 맞춘다. v 를 뒤집는 이유는 LABEL_FLIP_V 주석 참조.
        v0, v1 = (1.0, 0.0) if LABEL_FLIP_V else (0.0, 1.0)
        st = UsdGeom.PrimvarsAPI(m.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set([Gf.Vec2f(0.0, v0), Gf.Vec2f(1.0, v0),
                Gf.Vec2f(1.0, v1), Gf.Vec2f(0.0, v1)])

        if not self.stage.GetPrimAtPath(QR_LOOKS_ROOT):
            UsdGeom.Scope.Define(self.stage, QR_LOOKS_ROOT)
        mp = QR_LOOKS_ROOT + "/" + box_id
        mat = UsdShade.Material.Define(self.stage, mp)
        sh = UsdShade.Shader.Define(self.stage, mp + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        # ★ 종이 라벨이라 반사가 거의 없어야 한다. 광택이 있으면 조명이 QR 위에
        #   하이라이트를 만들어 흰 모듈과 검은 모듈의 대비가 무너진다.
        sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.02, 0.02, 0.02))

        reader = UsdShade.Shader.Define(self.stage, mp + "/stReader")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        tex = UsdShade.Shader.Define(self.stage, mp + "/Tex")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(png)
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result")
        # ★ clamp. repeat 로 두면 가장자리에서 QR 이 한 줄 되풀이돼 흰 여백을
        #   갉아먹고, 여백이 좁아지면 디텍터가 QR 경계를 못 잡는다.
        for w in ("wrapS", "wrapT"):
            tex.CreateInput(w, Sdf.ValueTypeNames.Token).Set("clamp")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tex.ConnectableAPI(), "rgb")
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")

        UsdShade.MaterialBindingAPI.Apply(m.GetPrim())
        UsdShade.MaterialBindingAPI(m.GetPrim()).Bind(mat)

    def _make_box(self, path, tname, dims, pos, yaw_deg, mass, box_id,
                  color=None, dest=None):
        """치수를 지오메트리에 굽는다 — `xformOp:scale` 을 안 쓴다.

        `spawn_feeder._make_box` 와 같은 형상이다. 거기 주석대로 scale 자체는
        흡착과 무관하지만(2026-08-22 정정), 치수를 구운 Mesh 가 어차피 깔끔하고
        `belt_cycle._box_dims` 도 extent 를 먼저 본다.

        여기에만 있는 것: **§5.4 의 박스별 물리 속성**과 `box_id` 커스텀 어트리뷰트.
        """
        hx, hy, hz = dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        prim = mesh.GetPrim()

        pts = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
               (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
        mesh.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
        mesh.CreateFaceVertexCountsAttr([4] * 6)
        mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4,
                                          1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
        mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
        mesh.CreateSubdivisionSchemeAttr("none")     # 없으면 모서리가 둥글어진다
        # displayColor 도 같이 준다 — 머티리얼을 못 찾는 뷰어에서도 색이 보인다.
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*self.colors.get(color, LINE_COLOR))])

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        half = math.radians(yaw_deg) / 2.0
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half))))
        # ★ AddScaleOp 을 넣지 않는다.

        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateRestOffsetAttr(self.rest_offset_m)         # ★ 단위 m (§5.4)
        col.CreateContactOffsetAttr(self.contact_offset_m)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        # ★ 박스마다. PhysicsScene 에 걸면 무시되고 스택이 무너진다 (§5.4)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(self.rb_pos_iter)
        rb.CreateSolverVelocityIterationCountAttr(self.rb_vel_iter)

        # ground truth 조회용 (§5.5). DB 가 없어도 씬만으로 id 를 되찾을 수 있다.
        prim.CreateAttribute("box_id", Sdf.ValueTypeNames.String).Set(box_id)
        prim.CreateAttribute("box_type", Sdf.ValueTypeNames.String).Set(tname)
        # ★ ground truth. 색은 1 차(HSV) 판정용, dest 는 2 차(QR) 판정용이다.
        #   인식이 이 값을 맞히는지 대조한다.
        prim.CreateAttribute("box_color", Sdf.ValueTypeNames.String).Set(color or "")
        prim.CreateAttribute("box_dest", Sdf.ValueTypeNames.String).Set(dest or "")

        _mp = COLOR_MAT_FMT % color if color in self.colors else LINE_MAT
        mat = UsdShade.Material(self.stage.GetPrimAtPath(_mp))
        if mat:
            UsdShade.MaterialBindingAPI.Apply(prim)
            UsdShade.MaterialBindingAPI(prim).Bind(mat)

        # ★ 라벨은 머티리얼 바인딩 **뒤에** 붙인다. USD 의 머티리얼 바인딩은
        #   자손이 물려받으므로, 먼저 붙이면 라벨이 빨강으로 덮인다.
        if self.qr_on:
            self._attach_label(path, box_id, hz)

        # ★ 물리 바인딩은 시각 머티리얼과 **별개 purpose** 다. `if mat:` 안에 넣으면
        #   안 된다 — 색이 없어도 마찰은 걸려야 한다.
        _pmp = self.stage.GetPrimAtPath(PHYS_MAT_PATH)
        if _pmp:
            UsdShade.MaterialBindingAPI.Apply(prim)
            UsdShade.MaterialBindingAPI(prim).Bind(
                UsdShade.Material(_pmp), UsdShade.Tokens.weakerThanDescendants, "physics")
