"""pick & place 한 사이클 — 작업명세서 §2.5 / §2.6 (B4).

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : ⑤ 벨트 정지 → ⑦ 집기 → ⑧ 놓기 → ⑩ 재가동 (한 사이클 전체)
    ║     불러 주는 쪽        : cmd_adapter (관제탑 명령 1건 = 이 클래스 호출 1회)
    ║     이 파일이 부르는 쪽 : robot_motion(관절) · gripper(흡착) · 씬 서비스
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

**전송에 의존하지 않는다.** 소켓은 `link`(→ `robot_motion.py`), ROS 서비스는 `svc`
(덕 타이핑)로 받는다. `orchestrator.py` 가 rclpy 구현을 꽂고, `scripts/verify_b4_cycle.py`
가 §2.7 Mock 을 꽂는다. **검증이 실제 상태머신을 그대로 돌린다** — 테스트 전용 사본이
따로 있으면 그 통과가 실물의 통과를 보장하지 못한다.

## 호출 규약

`run()` 은 **블로킹**이다. 명세서 §1.5 방법 1(`ReentrantCallbackGroup` +
`MultiThreadedExecutor`)에서 부르거나 별도 워커 스레드에서 불러야 한다.
**어떤 콜백 안에서도 `spin_until_future_complete` 를 부르지 않는다** — `svc` 구현이
그 규약을 지키는 것이 전제다.

## 소켓으로 직접 부르는 것과 아닌 것 (공통가이드 §3 규칙 4)

  소켓 O : move_ee · motion_status · grasp · release · is_holding · tcp_speed · get_box_pose
  소켓 X : 벨트 · 팔레트 · 스폰 — **ROS 서비스(`/conveyor/*`, `/sim/*`)를 경유한다.**
           Mock 으로 갈아끼울 수 있어야 병렬 개발이 성립한다.

## 좌표

`PlacePose` 의 `x,y` 는 박스 **중심**, `z` 는 박스 **밑면**이다 (§3.3).
`h_mm // 2` 를 빼먹으면 박스가 팔레트에 반쯤 파묻히거나 공중에 뜬다.

`move_ee` 의 목표는 **흡착면**이다 (tool0 아님 — `robot_motion.py` 가 그리퍼 길이를 더한다).
"""
import os
import time

RESULTS = ("PLACED", "PICK_FAILED", "PALLET_FULL", "BLOCKED", "IK_FAIL")

MOTION_TIMEOUT_S = 20.0          # 한 구간 이동에 이보다 오래 걸리면 이상한 것이다
MOTION_POLL_S = 0.02
# ★ 이송 구간 허용 오차. 상공 이동·후퇴는 mm 로 도착할 이유가 없다 — 그런데
#   지금까지 집는/놓는 하강과 **같은 정밀도**를 요구하며 기다렸다.
#   2026-08-25 실측: 웨이포인트 8개 중 6개가 이송인데, 정착 대기가 사이클당 5.26 s
#   (웨이포인트당 653 ms)였다. 기본값은 `REACH_TOL_M`(2mm) 과 같게 두어 **켜기 전엔
#   예전 동작 그대로**다. 데모가 `--transit-tol` 로 덮어써 값을 재고 정한다.
#   ⚠ 올리면 다음 호가 그만큼 낮거나 옆에서 시작한다. 이격 400 여유가 +32.9 mm 라
#     10 mm 는 그 1/3 을 먹는다 — 실측으로 정할 것.
TRANSIT_TOL_MM = 2.0

GRASP_GAP_MM = 2.0               # 흡착 직전 흡착면을 박스 상면 위 이만큼에 둔다
                                 #   ★ 패드 transZ 슬롯이 5 mm 다. 더 벌리면 떠서 붙는다
                                 #     (packing_robot.gripper.APPROACH_GAP_MM 과 같은 값)
GAP_FIX_TOL_MM = 0.5             # 실측 간극이 목표에서 이보다 벗어나면 한 번 보정한다

# 가로 이송 구간에 더하는 여유 [mm]. **기본 0** — 지금 기하에서는 필요 없다.
#
#   증상(error1.mp4 38~40 s): joint_1 이 픽(-X)에서 배치(+Y)로 90도 넘게 도는 동안
#   link_2/link_3 이 수평 원호를 그리며 로봇에 가장 가까운 기둥을 쓸어 밀어버린다.
#   웨이포인트 **끝점은 양쪽 다 깨끗**해서 정지 자세 검사로는 안 잡힌다.
#
#   ★ 이 값은 대증요법이었고, 근본 해결은 **로봇 이격 300 -> 400 mm** 다.
#     실측 (scripts/verify_transit_sweep_isaac.py, 구간 4 최악 여유):
#       이격 300 · lift   0 -> -100.5      이격 300 · lift 200 -> +4.9 (아슬)
#       이격 300 · lift 310/330/350 -> -20.6 / -20.5 / -20.3  (고도에 거의 무반응)
#       **이격 400 · lift 0 -> 5호 3단 +32.9 · 혼합 16박스 +29.7  전 구간 통과**
#     고도는 200 근처에서만 잠깐 통과하고 300 에서 다시 실패한다 — 팔꿈치 IK 가지가
#     고도에 따라 튀기 때문이다. "높을수록 안전"이 성립하지 않으니 이 값에 기대지 말 것.
#
#   ⚠ 로봇을 **뒤로** 빼는 것(이격을 더 늘리는 것)은 300 -> 450 구간에서 역효과였다
#     (최악 -100.5 -> -203.5). 400 이 측정된 최선이다.
TRANSIT_LIFT_MM = float(os.environ.get("PL_TRANSIT_LIFT_MM", "0"))

# ★★ 협곡 낙하 (2026-08-25, 김병직 제안).
#
#   왜: 통로 규칙이 존재하는 이유는 **팔이 박스를 따라 내려가야 하기 때문**이다.
#   협곡 입구 높이에서 흡착을 풀면 팔은 안 내려가도 된다 — 박스는 사방이 벽이라
#   스스로 자리를 잡는다. 실제 팔레타이징에서 마지막 칸에 쓰는 방법이다.
#
#   왜 지금: 계획을 조여서 협곡을 피하는 쪽은 한계를 봤다 (2026-08-25 Isaac, 시드 7).
#     통로 반경 200 일괄 : 팔 걸림 **0회** · knock 0.8mm · 그러나 **적재 22개**
#     2단 250/271        : 적재 25개 · 그러나 걸림 1회 · knock 14.8mm
#   안전과 적재율을 반경 하나로 동시에 만족시키는 값이 없다.
#
#   ⚠ 무겁게 떨어뜨리면 스택을 친다. **가벼운 박스만** 허용하고 낙하도 짧게 자른다.
#     3호가 3 kg 이라 그것만 대상이 된다 (4호 5 · 5호 9 kg).
#   ⚠ 근거 없음 — 화면으로 확인해야 하는 종류다 (CLAUDE.md §10-3).
#   기본 0 = 꺼짐. 켜려면 `PL_DROP_KG=3.5`.
DROP_MAX_KG = float(os.environ.get("PL_DROP_KG", "0"))
# ★★ 예측형 낙하(_drop_face_m)는 **기본 꺼짐**이다 (2026-08-25 실측으로 기각).
#   둘레 최고높이(rim)로 "여기는 못 내려간다" 를 미리 판단하는데, **크게 과대평가**한다.
#   같은 실행 안에 반증이 있다:
#     협곡 700 mm -> 상한에 걸려 그냥 내려갔는데 **안 닿았고** 오차 [+0.3 +0.4] mm
#     협곡 480 mm -> 미리 떨궈서 오차 [+10.9 -22.3] mm 로 흩어졌다
#   그리퍼 몸통(Ø245)은 슬롯에 들어가고, 걸리는 것은 **한참 위의 팔뚝**이다.
#   rim 만 봐서는 그 높이를 알 수 없다 — 접촉 감지가 정확히 안다 (robot_motion CONTACT_*).
#   -> 그래서 한때 껐다가, **반경을 좁히는 쪽으로 되살렸다** (2026-08-25 김병직 판단).
#      480 mm 에서 떨군 건 방식이 틀려서가 아니라 **둘레를 250 mm 나 봐서** 멀리 있는
#      높은 기둥까지 "입구" 로 쳤기 때문이다. 반경을 130 으로 좁히면 실제로 슬롯을
#      막는 가까운 벽만 보고, 낙하 지점이 그만큼 내려온다.
#      끄려면 `PL_DROP_PREDICT=0`.
DROP_PREDICT = os.environ.get("PL_DROP_PREDICT", "1") not in ("0", "", "off")
DROP_MAX_MM = float(os.environ.get("PL_DROP_MM", "500"))   # 낙하 상한
#   ⚠ 300 으로 잡았더니 실측 협곡 **480 mm** 가 전부 걸러졌다 (시드 7, 10·11번).
#     걸러진 결과가 오차 40.7 mm · knock 31.5 mm 라 떨어뜨리는 편이 나았다.
#     다만 3 kg 이 480 mm 면 착지 속도가 3 m/s 다 — 화면으로 확인할 것.
DROP_CLEAR_MM = 10.0             # 그리퍼 길이 한계에 두는 안전 여유
#   ⚠ 둘레를 볼 반경. **20 으로 잡았다가 낙하가 한 번도 안 걸렸다** (2026-08-25 실측).
#     실제로 팔이 걸린 벽이 발자국에서 145 mm 밖에 있었는데 20 으로는 안 보인다 —
#     통로 검사가 반경 122 로 못 봤던 것과 같은 실수다. 팔은 손목 부근이 폭 320 이라
#     그만큼 밖까지 봐야 한다. 과하게 잡아도 손해가 없다 — 입구가 낮으면 낙하거리가
#     저절로 0 에 가까워져 예전 동작과 같아진다.
#   ★ 반경 이력 (전부 실측): 20 -> 낙하가 **한 번도 안 걸렸다** (벽이 145mm 밖).
#     250 -> 걸리긴 하는데 **480~490 mm 위에서** 떨궈 너무 높았다 (오차 -22.3 mm).
#     둘레를 넓게 볼수록 더 높은 벽을 "입구"로 잡아 낙하 지점이 올라간다.
#     -> 그리퍼 몸통 반경(122)에 맞춰 **130** 으로 좁힌다. 실제로 슬롯을 막는
#        가까운 벽만 보고, 멀리 있는 높은 기둥은 입구로 안 친다.
RIM_MARGIN_MM = float(os.environ.get("PL_DROP_RIM_MM", "130"))

# ★★ 반응형 낙하 (2026-08-25, 김병직 제안 "팔뚝이 닿는 걸 인식해서 그때 흡착을 푼다").
#
#   위 기하 판정(_drop_face_m)은 **예측**이라 반경을 맞춰야 하고, 실제로 두 번 틀렸다
#   (통로 122 · 둘레 20 — 둘 다 실제 벽 145mm 를 못 봤다). 반응형은 그 튜닝이 없다.
#
#   신호는 이미 있다: 팔이 걸리면 수렴이 멈추고 `정착 XX mm — 더 안 줄어 종료` 가 뜬다.
#   그 순간 흡착면 위치를 `motion_status.face_m` 으로 읽을 수 있다.
#
#   ⚠ **가로로 밀린 것과 세로로 못 내려간 것을 반드시 가른다.** 실측에서 걸린 사례는
#     오차 [+7.9, +49.4, 0] — 세로가 아니라 **가로로 49mm** 밀린 것이었다. 그 자리에서
#     놓으면 박스가 슬롯을 빗나가 떨어진다. 가로 오차가 작을 때만 낙하로 본다.
DROP_STALL_MM = 10.0             # 목표보다 이만큼 이상 높이 멈췄으면 "걸린 것"
DROP_LAT_MM = 15.0               # 이 안으로 들어와야 놓는다 (슬롯을 빗나가면 안 된다)
# ★ 가로 오차는 **원인이 아니라 증상**이다 (2026-08-25 실측). 팔이 벽을 밀면서 옆으로
#   29.5 mm 밀려났고, 그 오차 때문에 낙하가 거부됐다 — 막으려던 충돌이 거부 사유를
#   만드는 자기모순이다. 거부한 결과가 오차 40.7 mm · knock 31.5 mm 로 더 나빴다.
#   -> 포기하지 말고 **위로 올려 가로를 되잡은 뒤** 떨어뜨린다. 조금만 올리면 벽을
#      벗어나 같은 XY 가 그냥 잡힌다 (바로 위 높이는 같은 XY 로 문제없이 지나갔다).
DROP_RECENTER_MM = 60.0          # 한 번에 이만큼 올려서 다시 XY 를 잡는다
DROP_RECENTER_N = 4              # 최대 몇 번까지 (60 x 4 = 240 mm)


def pallet_place_to_world(place, h_mm, origin_mm):
    """PlacePose(밑면) → 월드 박스 **중심** (m). 작업명세서 §3.3 그대로.

    ⚠ 원래 자리는 `packing_common/frames.py` 이고 B·C 가 같은 것을 import 해야 한다
      (§3.3). C0 가 아직 안 들어와서 여기 두었다 — 들어오면 **지우고 import 로 바꾼다.**
    """
    return (float(place["x_mm"] + origin_mm[0]) / 1000.0,
            float(place["y_mm"] + origin_mm[1]) / 1000.0,
            float(place["z_mm"] + origin_mm[2] + h_mm // 2) / 1000.0)


def world_to_pallet_place(center_xyz_m, yaw_deg, h_mm, origin_mm):
    """Isaac 에서 읽은 박스 pose(중심) → PlacePose(밑면). §3.3 그대로."""
    return {"x_mm": int(round(center_xyz_m[0] * 1000)) - origin_mm[0],
            "y_mm": int(round(center_xyz_m[1] * 1000)) - origin_mm[1],
            "z_mm": int(round(center_xyz_m[2] * 1000)) - origin_mm[2] - h_mm // 2,
            "yaw_deg": float(yaw_deg)}


def place_to_aabb_m(place, w_mm, d_mm, h_mm, origin_mm):
    """PlacePose -> 월드 AABB (lo, hi) [m]. yaw 0/90 만 (heightmap.allowed_yaw_deg).

    로봇이 피해야 할 덩어리를 그대로 준다. 회전은 축정렬 사각형으로 본다 — 45도 같은
    중간각은 계약에 없다.
    """
    w, d = (w_mm, d_mm) if round(place.get("yaw_deg", 0)) % 180 == 0 else (d_mm, w_mm)
    cx = place["x_mm"] + origin_mm[0]
    cy = place["y_mm"] + origin_mm[1]
    z0 = place["z_mm"] + origin_mm[2]
    return ([(cx - w / 2.0) / 1000.0, (cy - d / 2.0) / 1000.0, z0 / 1000.0],
            [(cx + w / 2.0) / 1000.0, (cy + d / 2.0) / 1000.0, (z0 + h_mm) / 1000.0])


class MotionError(Exception):
    """이동이 실패했다. state 에 motion_status 값이 들어 있다."""

    def __init__(self, state):
        super().__init__(state)
        self.state = state


def nearest_equiv_yaw(target_deg, ref_deg):
    """`target_deg` 와 같은 배치가 되는 각 중 `ref_deg` 에서 가장 가까운 것.

    박스도 흡착 패드(210×160 사각)도 **180° 회전 대칭**이다 — yaw 와 yaw±180 은
    물리적으로 완전히 같은 배치다. 계획값을 그대로 따라가면 픽(≈0°)에서 배치(90°)로
    갈 때 손목이 먼 쪽으로 최대 180° 돈다.

    ★ 그러면 관절 속도 한계에 걸린다. `robot_motion` 의 도달 판정
      (`REACH_TOL_M`)은 **흡착면 위치만** 보고 자세는 안 본다 — 위치가 들어오면
      손목이 덜 돌았어도 `REACHED` 를 낸다. 실측(2026-08-24)에서 구간이
      `속도초과 1 스텝` 으로 끝나고 다음 이동이 `yaw 112.9` 에서 시작했고,
      그 박스가 삐딱하게 놓여 넘어졌다.

    등가 각 중 가까운 쪽을 고르면 회전량이 **최대 90°** 로 줄어 이 상황이 안 생긴다.
    적재 결과는 완전히 동일하다.
    """
    best, best_d = float(target_deg), None
    for cand in (target_deg - 180.0, float(target_deg), target_deg + 180.0):
        d = abs((cand - ref_deg + 180.0) % 360.0 - 180.0)
        if best_d is None or d < best_d:
            best, best_d = float(cand), d
    return best


class Cycle:
    """구역 하나의 pick & place. **구역당 하나를 만들어 끝까지 재사용한다.**

    ★ 명령마다 새로 만들면 안 된다. 아래 상태가 PICK 과 PLACE 사이를 넘어가고,
      그중 `_stack_top_m` 이 0 으로 돌아가면 **팔이 적재물 위를 낮게 지나 스택을 친다** —
      CLAUDE.md §10-3 이 기록한 knock 700.8 mm 사고가 정확히 그 경로다.

        _stack_top_m   이송 고도의 바닥. 갈 때도 **올 때도** 이 높이를 넘어야 한다
        _obstacles     놓은 박스들의 월드 AABB. `move_ee` 가 IK 가지를 고를 때 쓴다
        _pick_yaw      PLACE 의 yaw 기준. 없으면 손목이 먼 쪽으로 180° 돈다
        _pick_c/_top   이송 고도 계산의 한 항

    ★ 흡착 직전 **간극을 실측해 한 번 보정한다** (`GRASP_GAP_MM` · `GAP_FIX_TOL_MM`).
      패드의 transZ 슬롯이 5 mm 뿐이라, 더 벌어진 채 닫으면 박스가 떠서 붙는다.

    ★ 가벼운 박스(`drop_max_kg` 이하)는 **협곡 입구에서 놓고 떨어뜨린다.**
      `_corridor_ok` 가 존재하는 이유가 "팔이 박스를 따라 내려가야 해서" 인데,
      떨어뜨리면 팔이 안 내려가므로 그 제약이 풀린다. 적재율 70.8 → 75.5 %.
      ⚠ 무거운 박스는 낙하 충격이 스택을 친다 — 그래서 질량 상한이 있다.
    """

    def __init__(self, link, svc, cfg, zone="B", log=print):
        self.link = link
        self.svc = svc
        self.cfg = cfg
        self.zone = zone
        self._log = log
        z = cfg["zones"]
        self.origin_mm = list(z["pallet_origin_world_mm"][zone])
        # ★ 놓은 박스의 월드 AABB [m]. `move_ee` 에 실어 보내 로봇이 **IK 가지를 고를 때**
        #   쓴다 (robot_motion 은 씬을 직접 안 읽는다 — 무엇을 피할지는 이쪽 판단이다).
        #   지금 들고 있는 박스는 여기 없다 — 놓고 난 뒤에 넣는다.
        self._obstacles = []
        self.approach_m = cfg["robot"]["approach_offset_mm"] / 1000.0
        # ★ 낙하 판정용 질량표. 치수 -> kg. yaml 이 정본이라 여기 숫자를 안 적는다.
        self._mass_kg = {}
        for b in cfg["boxes"]["types"]:
            self._mass_kg[(b["w_mm"], b["d_mm"], b["h_mm"])] = b["mass_g"] / 1000.0
        self.grip_len_m = cfg["robot"]["gripper_length_mm"] / 1000.0
        self._last_put_yaw = 0.0
        self.belt_seg = "%s_pickup" % zone.lower()
        self.belt_mps = float(cfg["conveyor"]["belt_velocity_mps"])
        self.grasp_budget_s = float(cfg["robot"]["suction"]["retry_interval_s"]) + 3.0
        r = cfg["robot"]
        self.release_budget_s = (r["release_wait_timeout_s"] + r["release_damping_duration_s"]
                                 + r["settle_wait_s"] + 3.0)
        # ★ 적재 최상단을 **로봇이 직접 최대치로 들고 간다** (1차 구현 합의, 2026-08-24).
        #   `stack_top_mm` 을 C 가 채워 주길 기다리지 않는다 — 지금까지 이 사이클이
        #   실제로 놓은 것들의 최고 상면을 누적해 그중 **최대**를 쓴다. 계획이 z 를
        #   오르내려도(온라인/비전 경로가 그렇다) 이송이 항상 적재물을 넘는다.
        #   메시지에 `stack_top_mm` 이 실려 오면 그것과도 max 를 취하므로 둘 다 산다.
        #   ⚠ 팔레트를 갈면 리셋해야 한다 -> `reset_stack()`
        self._stack_top_m = 0.0
        # ★ 집기가 놓기에 넘겨주는 값 3개. 원래 `_run` 안의 지역변수였는데,
        #   관제탑이 PICK/PLACE 를 **따로** 보내면서 호출이 둘로 갈려 사라진다.
        #     _pick_yaw  놓기 yaw 의 기준 (nearest_equiv_yaw). 없으면 손목이 180° 돈다
        #     _pick_top  이송 고도 max(pick_top, put_top, stack_top) 의 한 항
        #     _pick_c    이송 첫 구간의 xy (그 자리에서 수직 상승한다)
        self._pick_c = [0.0, 0.0, 0.0]
        self._pick_top = 0.0
        self._pick_yaw = 0.0

    def _rim_m(self, cx, cy, w, d):
        """놓을 자리 **둘레**에서 가장 높은 이웃 상면 [m]. 이웃이 없으면 None.

        발자국을 RIM_MARGIN 만큼 부풀려 겹치는 장애물만 본다. 바로 밑에서 받쳐 주는
        박스도 여기 들어오는데, 그 상면은 새 박스 밑면과 같아서 판정에 영향이 없다.
        """
        hw = w / 2000.0 + RIM_MARGIN_MM / 1000.0
        hd = d / 2000.0 + RIM_MARGIN_MM / 1000.0
        top = None
        for lo, hi in self._obstacles:
            if hi[0] < cx - hw or lo[0] > cx + hw:
                continue
            if hi[1] < cy - hd or lo[1] > cy + hd:
                continue
            top = float(hi[2]) if top is None else max(top, float(hi[2]))
        return top

    def _drop_face_m(self, cx, cy, w, d, h, put_top):
        """협곡이면 **떨어뜨릴** 흡착면 높이 [m], 아니면 None.

        -> (흡착면 z, 낙하거리 m) / None
        """
        if DROP_MAX_KG <= 0 or not DROP_PREDICT:
            return None
        kg = self._mass_kg.get((w, d, h))
        if kg is None or kg > DROP_MAX_KG:
            return None                       # 무거운 건 안 떨어뜨린다 (조용히 — 흔하다)
        rim = self._rim_m(cx, cy, w, d)
        if rim is None:
            return None
        # 팔이 따라 내려갈 수 있는 벽 높이 = 내 윗면 + 그리퍼 길이. 그 위면 협곡이다.
        if rim <= put_top + self.grip_len_m:
            return None
        bottom = put_top - h / 1000.0         # 원래 놓으려던 박스 밑면
        # ★ 놓는 높이는 **박스 윗면**으로 정한다 (2026-08-25 김병직 요청 "더 낮출 수 없나").
        #   예전엔 박스 **밑면**을 입구 위 10 mm 에 뒀는데, 그럴 이유가 없다 —
        #   그리퍼 몸통(Ø245)은 슬롯 안으로 들어간다. `_corridor_ok` 주석의 근거 그대로:
        #     "박스 윗면부터 그리퍼 길이(150)까지는 몸통만 지나가고, 그 몸통은 모든
        #      박스 발자국 안에 들어간다 (3호 반폭 125 > 몸통 반경 122.5)"
        #   못 들어가는 건 플랜지 **위**의 손목이다. 그러니 박스 윗면을 `rim - 150` 까지
        #   내려도 된다. 실측 480 mm 짜리가 이 계산으로 약 120 mm 가 된다.
        rel_top = rim - self.grip_len_m + DROP_CLEAR_MM / 1000.0
        rel_bottom = rel_top - h / 1000.0
        if rel_bottom <= bottom:
            return None                       # 그냥 끝까지 내려갈 수 있다
        drop = rel_bottom - bottom
        if drop <= 0:
            return None
        if drop > DROP_MAX_MM / 1000.0:
            # ★ 걸러진 것도 남긴다 — 낙하가 "안 걸린" 건지 "막힌" 건지 로그로 갈린다.
            self._log("[cycle] 협곡 %.0f mm — 낙하 상한 %.0f 초과, 그냥 내려간다"
                      % (drop * 1000, DROP_MAX_MM))
            return None
        return rel_top + GRASP_GAP_MM / 1000.0, drop

    def _stall_drop_ok(self, cx, cy, w, d, h, tgt_face_z, err):
        """걸려서 멈춘 자리에서 **놓아도 되나**. -> True 면 낙하로 처리하고 계속 간다.

        `err` 가 None 이면 도착은 했는데 잔차가 남은 경우다 (기록만 남긴다).
        """
        if DROP_MAX_KG <= 0:
            return False
        st = self.link.call(cmd="motion_status") or {}
        face = st.get("face_m")
        if not face or len(face) != 3:
            return False
        lat = ((float(face[0]) - cx) ** 2 + (float(face[1]) - cy) ** 2) ** 0.5
        short = float(face[2]) - tgt_face_z          # + 면 목표보다 높이 멈춘 것
        if short < DROP_STALL_MM / 1000.0:
            return False                             # 걸린 게 아니다
        kg = self._mass_kg.get((w, d, h), 99.0)
        if kg > DROP_MAX_KG:
            self._log("[cycle] 걸림 %.0f mm 인데 낙하 불가 — %.1f kg 은 무겁다"
                      % (short * 1000, kg))
            return False
        # ★ 가로로 밀렸으면 **위로 올려 되잡는다** (위 DROP_RECENTER 주석).
        z_now = float(face[2])
        for _ in range(DROP_RECENTER_N):
            if lat <= DROP_LAT_MM / 1000.0:
                break
            z_now += DROP_RECENTER_MM / 1000.0
            self._log("[cycle] 가로 %.1f mm 밀림 — %.0f mm 올려 되잡는다"
                      % (lat * 1000, DROP_RECENTER_MM))
            try:
                self._move([cx, cy, z_now], self._last_put_yaw)
            except MotionError:
                pass                      # 못 가도 아래에서 **실제 위치**로 다시 판단한다
            st = self.link.call(cmd="motion_status") or {}
            face = st.get("face_m") or face
            lat = ((float(face[0]) - cx) ** 2 + (float(face[1]) - cy) ** 2) ** 0.5
            z_now = float(face[2])
        short = z_now - tgt_face_z
        if lat > DROP_LAT_MM / 1000.0:
            self._log("[cycle] %.0f mm 올려도 가로 %.1f mm — 슬롯을 빗나간다, 포기"
                      % (DROP_RECENTER_MM * DROP_RECENTER_N, lat * 1000))
            return False
        if short > DROP_MAX_MM / 1000.0:
            self._log("[cycle] 걸림 %.0f mm 인데 낙하 불가 — 상한 %.0f 초과"
                      % (short * 1000, DROP_MAX_MM))
            return False
        self._log("[cycle] 팔뚝 걸림 감지 (%.0f mm 남음 · 가로 %.1f mm · %.1f kg) "
                  "— 그 자리에서 놓는다%s"
                  % (short * 1000, lat * 1000, kg, "" if err is None else "  [%s]" % err))
        return True

    def reset_stack(self):
        """팔레트를 갈았다 — 누적해 온 적재 최상단을 버린다."""
        self._stack_top_m = 0.0

    def retreat_home(self):
        """팔을 적재물 위 안전 고도로 **수직 상승만** 시킨다.

        ⚠ 관제탑의 `RobotCmd.HOME` 은 본래 "대기 자세로 복귀"지만, **그 관절값이
          아무 데도 없다** — 문서에도, `robot_motion.handlers()` 에도 (move_ee ·
          motion_status · grasp · release · is_holding · tcp_speed 여섯뿐이다).
          없는 값을 지어내지 않는다 (CLAUDE.md §11).
          지금 확실히 할 수 있는 것은 **적재물 위로 팔을 빼는 것**이고 그것만 한다.
          진짜 홈 자세가 정해지면 여기를 채운다.
        """
        cur = self._face_now()
        if cur is None:
            return
        z = max(self._stack_top_m, cur[2]) + self.approach_m
        if z > cur[2] + 1e-3:
            self._move([cur[0], cur[1], z], self._last_put_yaw, TRANSIT_TOL_MM)

    # ── 한 사이클 ────────────────────────────────────────────
    def run(self, box_id, pick_pose):
        """박스 하나를 집어서 놓는다. 반환은 RESULTS 중 하나.

        Args:
            box_id: `/zone_b/box_at_pickup` 이 준 id
            pick_pose: 같은 메시지의 pose — `{"pos_m": [x,y,z], "yaw_deg": f}`
                (박스 **중심**)
        """
        t0 = time.time()
        try:
            return self._run(box_id, pick_pose, t0)
        except MotionError as e:
            self._log("[cycle] 이동 실패 %s — 정리하고 종료" % e.state)
            self._abort(box_id)
            return "IK_FAIL" if e.state == "IK_FAIL" else "BLOCKED"
        except Exception as e:                       # 소켓 끊김·서비스 타임아웃 등
            self._log("[cycle] !! %r — BLOCKED" % (e,))
            self._abort(box_id)
            return "BLOCKED"

    def _run(self, box_id, pick_pose, t0):
        """한 사이클의 본체. `run()` 이 예외를 잡아 `BLOCKED` 로 바꾸는 안쪽이다.

            벨트 정지 → 크기 조회 → 자리 계산 → 접근·흡착 → 이송 → 하강·해제
            → 실측 pose 회수 → heightmap 갱신 → 벨트 재가동

        ★ 어느 단계에서 빠져나가든 `_abort()` 를 거쳐 **`cancel_plan` 을 부른다**.
          안 부르면 알고리즘이 stale plan 을 들고 있게 되고, 다음 박스의 계획이
          그 위에 쌓인다.
        """
        # 1) 벨트 정지. ★ ok 를 확인한 뒤 진행한다 — 안 멈춘 채 흡착하면 박스가 끌려간다.
        if not self.svc.set_velocity(self.belt_seg, 0.0):
            self._log("[cycle] 벨트 정지 실패")
            return "BLOCKED"

        # 2) 크기는 **순서가 아니라 id 로** 받는다 (§2.5)
        info = self.svc.get_box_info(box_id)
        if not info:
            self._log("[cycle] get_box_info(%s) found=false" % box_id)
            self._resume_belt()
            return "BLOCKED"
        w, d, h = info["w_mm"], info["d_mm"], info["h_mm"]

        # 3) 배치 계획
        ok, place, msg = self.svc.plan_placement(self.zone, box_id, w, d, h)
        if not ok:
            self._log("[cycle] plan_placement -> %s" % msg)
            self.svc.cancel_plan(self.zone, box_id)
            self._resume_belt()
            return "PALLET_FULL" if msg == "PALLET_FULL" else "BLOCKED"

        # 4) 집기
        if self.pick(box_id, pick_pose, h) != "PICKED":
            # §2.5 — 픽 실패는 계획을 버리고 벨트를 다시 돌린 뒤 종료한다.
            self.svc.set_status(box_id, "PICK_FAILED")
            self.svc.cancel_plan(self.zone, box_id)
            self._resume_belt()
            return "PICK_FAILED"
        self.svc.set_status(box_id, "PICKED")

        # 5) 놓기
        res, actual, _measured = self.place(box_id, place, w, d, h)
        if res != "PLACED":
            return res

        # 6) heightmap 갱신 — **실제** pose 로 (§7.6)
        ok_hm, fill = self.svc.update_heightmap(self.zone, box_id, w, d, h, actual)
        if not ok_hm:
            # yaw 오차 초과 경고. 로그만 남기고 계속 간다 (§2.2)
            self._log("[cycle] update_heightmap ok=false — 계속 진행")
        self.svc.mark_placed(box_id, actual)
        self._resume_belt()
        self._log("[cycle] %s 완료  %.2f s  적재율 %s" % (box_id, time.time() - t0, fill))
        return "PLACED"

    # ══ 관제탑이 부르는 반쪽 두 개 ══════════════════════════════
    # ★ 왜 나눴나: 관제탑은 RobotCmd(PICK/PLACE)로 **사이클 단위**로 말하는데
    #   `_run` 은 집기와 놓기를 한 호출 안에서 끝낸다. 그 사이에 벨트·DB·algo
    #   호출이 섞여 있는데 **그건 관제탑이 이미 한다** (CLAUDE.md §8).
    #   -> 모션만 떼어 두 조각으로 만들고, `_run` 은 그 둘을 부르는 껍데기로 남긴다.
    #   **동작은 한 줄도 바꾸지 않았다.** 수용 기준은 시드 42 가 28/29 · 80.3% 로
    #   좌표까지 재현되는 것이다 (2026-08-25 실측, docs/2026-08-25_실행기록.md).
    #
    # ⚠ 경계를 넘는 상태가 4개다. **그래서 Cycle 을 명령마다 새로 만들면 안 된다:**
    #     _stack_top_m   이송 고도의 바닥. 0 이 되면 팔이 적재물 위를 낮게 지난다 (§10-3)
    #     _obstacles     move_ee 의 IK 가지 선택용
    #     _pick_yaw      놓기 yaw 의 기준. 없으면 손목이 먼 쪽으로 180° 돈다
    #     _pick_c/_top   이송 고도와 첫 상승 지점

    def pick(self, box_id, pick_pose, h):
        """집기만 한다. "PICKED" | "PICK_FAILED". 이동 실패는 MotionError 로 올라간다.

        ★ set_status·cancel_plan·벨트는 **부르지 않는다** — 관제탑 몫이다.
        """
        pick_c = list(pick_pose["pos_m"])
        pick_top = pick_c[2] + h / 2000.0                    # 박스 상면 z [m]
        pick_yaw = float(pick_pose.get("yaw_deg", 0.0))
        # ★ 픽 상공으로 **곧장 가지 않는다.** 호 이송은 z 를 선형 보간하므로, 직전 배치의
        #   후퇴 자리가 낮으면(예: 1층 배치 후 0.69 m) 호 중간이 픽업 대기 박스 윗면
        #   (1.141 m)보다 낮아지고, 그리퍼 몸통(Ø245)이 박스 모서리를 쓸어 밀어 버린다.
        #   2026-08-24 실측: 후퇴 0.69 에서 온 사이클만 호 67% 지점(흡착면 z 1.125,
        #   몸통 가장자리가 박스 모서리 안쪽 7.7 mm)에서 박스를 30 mm 밀었다. 후퇴 0.91
        #   에서 온 같은 자리 사이클은 멀쩡했다. 밀린 박스를 그대로 잡아 그대로 놓으므로
        #   배치 오차가 곧 그 30 mm 가 된다 (results/issues/21).
        #   -> 배치 구간과 같은 규약으로 **먼저 수직 상승**한 뒤 가로로 온다.
        # ★★ 픽 상공만 보면 안 된다 — **이미 쌓인 것의 최상단**도 넘어야 한다.
        #   배치 구간(아래 `transit`)은 `stack_top` 을 넣는데 **여기만 pick_top** 이었다.
        #   그래서 갈 때와 올 때의 고도가 달랐다.
        #   2026-08-25 실측 (`results/ov5_s99.txt`, 김병직 육안 확인):
        #     사이클 18  갈 때 z 1.58  (스택 기준)
        #     사이클 19  올 때 z 1.28  (픽 기준)   <- 300 mm 낮다
        #     그때 스택 최상단 1.172 — 흡착면은 108 mm 뜨지만 **팔 링크가 그 높이에서
        #     훨씬 노출된다.** 궤적 스윕이 매번 link_2 를 지목한 것이 이 구간이다.
        #   결과: 18번이 밀려 배치 오차 +7.5/+10.5 mm (update_heightmap 거부), 19번까지
        #   흐트러졌다.
        #   `_stack_top_m` 은 **실측 상면**으로 누적된다 (아래 참조) — 이미 있는 값이다.
        pick_air = max(pick_top, self._stack_top_m) + self.approach_m
        cur = self._face_now()
        if cur is not None and cur[2] < pick_air - 1e-3:
            self._move([cur[0], cur[1], pick_air], pick_yaw, TRANSIT_TOL_MM)
        self._move([pick_c[0], pick_c[1], pick_air], pick_yaw, TRANSIT_TOL_MM)
        self._descend_to_gap(box_id, [pick_c[0], pick_c[1]], pick_top, pick_yaw)

        # ★ place() 가 쓴다. 지역변수로 두면 반쪽으로 갈렸을 때 사라진다.
        self._pick_c, self._pick_top, self._pick_yaw = pick_c, pick_top, pick_yaw

        if not self._grasp(box_id):
            # §2.5 — 픽 실패는 팔만 빼고 나간다. 계획 폐기·상태·벨트는 관제탑 몫이다.
            self._move([pick_c[0], pick_c[1], pick_top + self.approach_m], pick_yaw, TRANSIT_TOL_MM)
            return "PICK_FAILED"
        return "PICKED"

    def place(self, box_id, place, w, d, h):
        """놓기만 한다. `(결과, actual, 실측여부)`.

        `실측여부=False` 면 actual 은 **계획값**이다 — 씬에서 못 읽었다는 뜻이므로
        `RobotEvent.has_actual` 에 그대로 실으면 안 된다 (§3-4 규약 3).

        ★ update_heightmap·mark_placed·벨트는 **부르지 않는다** — 관제탑 몫이다.
          `has_actual=true` 로 관제탑에 실어 보내면 그쪽이 UpdateHeightmap 을 부른다.
        """
        # 집기가 남겨 둔 값들. Cycle 을 새로 만들면 여기가 0 이 된다.
        pick_c, pick_top, pick_yaw = self._pick_c, self._pick_top, self._pick_yaw
        cx, cy, cz = pallet_place_to_world(place, h, self.origin_mm)
        put_top = cz + h / 2000.0
        # ★ 손목을 먼 쪽으로 180° 돌리지 않는다 — nearest_equiv_yaw 주석 참조.
        put_yaw = nearest_equiv_yaw(place.get("yaw_deg", 0.0), pick_yaw)
        self._last_put_yaw = put_yaw          # 되잡기 이동이 같은 손목 자세를 쓴다

        # ★ 상승 -> **가로 이송** -> 하강. 세 구간이다 (§2.5).
        #   집자마자 배치 상공으로 바로 가면 관절공간 직선이 대각선을 그어서, 매달린
        #   박스가 벨트를 뚫고 지나간다. 벨트를 확실히 넘는 높이에서 가로로 옮긴 뒤
        #   내려가야 한다. 매달린 박스는 흡착면보다 h 만큼 아래에 있으므로 그만큼 더 든다.
        #   그리고 **이미 쌓인 것의 최상단**도 넘어야 한다. 계획이 z 를 오르내리면
        #   목적지가 최상단보다 낮아서 이송 중 적재물을 관통한다. 정본 계산은
        #   check_place_plan.py:176 `carry_bottom = max(obstacle, top) + safe_mm` 다.
        #   `stack_top_mm` 은 **선택**이다 — 실려 오면 쓰고(=/algo 가 heightmap 에서 읽은
        #   값, 단위는 팔레트 상면 기준 mm), 없어도 **우리가 누적한 최대치**가 있으므로
        #   이송은 언제나 적재물을 넘는다 (1차 구현 합의: 최상단은 최대한으로 본다).
        stack_top = max((float(place.get("stack_top_mm", 0.0)) + self.origin_mm[2]) / 1000.0,
                        self._stack_top_m)
        # TRANSIT_LIFT_MM 은 **팔꿈치 간섭용**이다 (위 상수 주석 참조).
        transit = (max(pick_top, put_top, stack_top) + self.approach_m + h / 1000.0
                   + TRANSIT_LIFT_MM / 1000.0)
        self._move([pick_c[0], pick_c[1], transit], pick_yaw, TRANSIT_TOL_MM)
        self._move([cx, cy, transit], put_yaw, TRANSIT_TOL_MM)
        # ★ 배치 approach 도 **매달린 박스 밑면** 기준이다 — 여기도 h 를 더한다.
        #   흡착면 기준으로 잡으면 박스 밑면이 `approach - h` 만큼만 뜬다. 박스가
        #   approach_offset(200)보다 높으면 밑면이 **같은 층 이웃 상면보다 아래**로 내려가
        #   슬롯에 들어가기도 전에 이웃에 얹힌다 (3호 −10 · 4호 −80 · 5호 −140 mm).
        #   2026-08-22 실측: 15박스 4층에서 13번째(양옆이 다 찬 자리)가 여기서 걸려
        #   도착 잔차 208.9 mm · 이웃을 103.7 mm 밀어냄 · 그 위에 얹혀 Z 오차 +208.8 mm.
        #   정본은 check_place_plan.py:176 `carry_bottom = max(obstacle, top) + safe_mm`
        #   즉 h_safe(흡착면) = ... + safe + h 다. 152행 transit 은 이미 그렇게 돼 있었다.
        self._move([cx, cy, put_top + self.approach_m + h / 1000.0], put_yaw, TRANSIT_TOL_MM)
        # ★ 협곡이면 입구에서 놓아 **떨어뜨린다** (위 DROP_* 주석). 팔이 안 내려가므로
        #   팔뚝이 벽에 걸릴 일이 없다. 꺼져 있거나 조건이 안 맞으면 None 이라 예전 그대로다.
        drop = self._drop_face_m(cx, cy, w, d, h, put_top)
        tgt_face_z = put_top + GRASP_GAP_MM / 1000.0
        if drop is not None:
            face_z, dist = drop
            self._log("[cycle] 협곡 낙하 %.0f mm (%.1f kg) — 입구에서 놓는다"
                      % (dist * 1000, self._mass_kg.get((w, d, h), 0.0)))
            self._move([cx, cy, face_z], put_yaw)
        else:
            # ★ 여기서 팔뚝이 벽에 걸릴 수 있다. 걸리면 `IK_FAIL` 이 오는데, 그때
            #   박스는 이미 **슬롯 바로 위**에 있다 — 버리지 말고 놓을 수 있는지 본다.
            try:
                self._move([cx, cy, tgt_face_z], put_yaw)
            except MotionError as e:
                if not self._stall_drop_ok(cx, cy, w, d, h, tgt_face_z, str(e)):
                    raise
            else:
                self._stall_drop_ok(cx, cy, w, d, h, tgt_face_z, None)

        # TCP 정지 대기 · damping 스파이크 · settle 은 robot_motion 안에서 돈다 (§6.5)
        if not self._release():
            self._log("[cycle] release 실패")
            self._abort(box_id)
            return "BLOCKED", None, False
        # ★ 후퇴는 **절대 내려가지 않는다** (2026-08-25 김병직 지적).
        #   `put_top + approach` 로 고정하면, 협곡 입구에서 떨어뜨렸을 때 팔이 이미
        #   그보다 위에 있어서 **놓고 나서 협곡 안으로 다시 내려간다.** §10-3 과 같은
        #   부류의 버그다 — 갈 때는 최상단을 넘는데 올 때만 안 넘던 그것.
        #   지금 흡착면 높이 · 방금 놓은 박스 상면 · 누적 최상단, 셋 중 가장 높은 곳으로.
        retreat_z = max(put_top + self.approach_m, self._stack_top_m + self.approach_m)
        _st = self.link.call(cmd="motion_status") or {}
        _f = _st.get("face_m")
        if _f and len(_f) == 3:
            retreat_z = max(retreat_z, float(_f[2]))
        self._move([cx, cy, retreat_z], put_yaw, TRANSIT_TOL_MM)

        # 6) **실제** pose 로 갱신한다 — 계획값이 아니라 (§7.6)
        # ★ 셋째 반환값이 필요한 이유: 못 읽으면 아래에서 **계획값**으로 대체하는데,
        #   그걸 `RobotEvent.has_actual=true` 로 실어 보내면 §3-4 규약 3 위반이다 —
        #   계획값을 되보내면 오차가 누적돼 알고리즘은 1300 안이라 믿는데 실물이 넘는다.
        #   `_run`(데스크 경로)은 예전처럼 계획값으로 갱신한다 — 동작을 안 바꾼다.
        measured = self._actual_place(box_id, h)
        actual = measured or place
        # ★ 적재 최상단 누적. **실제 상면**으로 올린다 — 계획대로 안 앉았으면
        #   그만큼 더 높을 수 있고, 이송은 그 실물을 넘어야 한다.
        act_top = (float(actual.get("z_mm", place.get("z_mm", 0.0))) + h
                   + self.origin_mm[2]) / 1000.0
        self._stack_top_m = max(self._stack_top_m, act_top)
        # ★ 놓은 뒤에 누적한다. 이송 중에는 들고 있으므로 장애물이 아니다.
        #   (원문에서는 update_heightmap 다음 줄이었다. 그 사이에 _move 가 없어서
        #    순서를 앞당겨도 결과가 같다 — _obstacles 는 _move 만 읽는다.)
        self._obstacles.append(place_to_aabb_m(actual, w, d, h, self.origin_mm))
        return "PLACED", actual, measured is not None


    # ── 조각 ─────────────────────────────────────────────────
    def _face_now(self):
        """지금 흡착면 월드 위치 [m]. 못 읽으면 None (호출부가 건너뛴다)."""
        try:
            f = self.link.call(cmd="motion_status").get("face_m")
            return [float(v) for v in f] if f else None
        except Exception:
            return None

    def _move(self, face_m, yaw_deg, tol_mm=None):
        """흡착면을 face_m 으로 보낸다. 도착까지 기다린다.

        `tol_mm` 을 주면 그 구간만 허용 오차를 바꾼다 (이송은 느슨하게, 하강은 그대로).
        """
        r = self.link.call(cmd="move_ee", target_m=[float(v) for v in face_m],
                           yaw_deg=float(yaw_deg), obstacles=self._obstacles,
                           tol_m=None if tol_mm is None else float(tol_mm) / 1000.0)
        if not r.get("accepted"):
            raise MotionError(r.get("error", "rejected"))
        t0 = time.time()
        while time.time() - t0 < MOTION_TIMEOUT_S:
            st = self.link.call(cmd="motion_status").get("state")
            if st == "REACHED":
                return
            if st == "IK_FAIL":
                raise MotionError("IK_FAIL")
            time.sleep(MOTION_POLL_S)
        raise MotionError("MOTION_TIMEOUT")

    def _descend_to_gap(self, box_id, xy, top_m, yaw):
        """흡착면을 박스 상면 위 GRASP_GAP_MM 로 내린다. **실측하고 한 번 보정한다.**

        열린 루프로 내려가면 도착 잔차(최대 1 mm 대)가 그대로 간극에 얹혀 허용치
        2 mm 를 넘긴다. 실제로 GUI 실행에서 2.41 mm 로 흡착이 거절됐다 —
        헤드리스는 폴링이 성겨서 팔이 우연히 더 정착했을 뿐이라 **벽시계에 의존하는
        판정**이었다. 여기서 닫아 준다.
        """
        target = GRASP_GAP_MM / 1000.0
        self._move([xy[0], xy[1], top_m + target], yaw)
        gap = self._gap_m(box_id, top_m)
        if gap is None:
            return
        err = gap - target
        if abs(err) * 1000.0 <= GAP_FIX_TOL_MM:
            return
        self._log("[cycle] 접근 간극 %.2f mm (목표 %.1f) — %+.2f mm 보정"
                  % (gap * 1000, GRASP_GAP_MM, -err * 1000))
        self._move([xy[0], xy[1], top_m + target - err], yaw)
        gap2 = self._gap_m(box_id, top_m)
        if gap2 is not None:
            self._log("[cycle] 보정 후 간극 %.2f mm" % (gap2 * 1000))

    def _gap_m(self, box_id, top_m):
        """지금 흡착면과 박스 상면 사이 [m]. 못 재면 None."""
        st = self.link.call(cmd="motion_status")
        face = st.get("face_m")
        if not face:
            return None
        return float(face[2]) - top_m

    def _grasp(self, box_id):
        """흡착. **`call_slow` 로 부른다** — 그리퍼가 닫기를 시도하는 총 시간
        (`suction.retry_interval_s`, 기본 10 s)만큼 걸릴 수 있어서, 기본 2 s 타임아웃으로
        부르면 정상 동작인데도 실패로 오판한다."""
        return bool(self.link.call_slow(self.grasp_budget_s,
                                        cmd="grasp", box_id=box_id).get("ok"))

    def _release(self):
        """흡착 해제. `_grasp` 와 같은 이유로 `call_slow` 다.

        해제 뒤 곧바로 팔을 빼면 박스가 딸려 온다. 실측(놓은 뒤 100 ms 구간 이동량):
        0~0.5 s 는 0.00~0.03 mm 로 **아예 안 움직이다가**, 0.5 s 흡착 해제 →
        0.7 s 에 자리잡고(0.39~2.26 mm) 0.8 s 부터 조용하다. 그 값들이
        `release_*`/`settle_wait_s` 로 yaml 에 있다.
        """
        return bool(self.link.call_slow(self.release_budget_s, cmd="release").get("ok"))

    def _actual_place(self, box_id, h_mm):
        """놓인 자리를 씬에서 직접 읽어 PlacePose 로 바꾼다. 못 읽으면 None."""
        r = self.link.call(cmd="get_box_pose", box_id=box_id)
        if not r.get("ok"):
            self._log("[cycle] get_box_pose 실패 — 계획값으로 갱신한다")
            return None
        return world_to_pallet_place(r["pos_m"], r.get("yaw_deg", 0.0), h_mm, self.origin_mm)

    def _resume_belt(self):
        """벨트 재가동. **모든 이탈 경로에서 불러야 한다** — 안 부르면 라인이 선 채로
        관제탑만 IDLE 로 돌아가고, 다음 박스가 영영 안 온다."""
        self.svc.set_velocity(self.belt_seg, self.belt_mps)

    def _abort(self, box_id):
        """이탈 경로 정리. **모든 경로에서 cancel_plan 을 부른다** (§6.6).

        안 부르면 C 가 stale plan 을 들고 있게 되고, 다음 박스의 계획이 그 위에 쌓인다.
        """
        try:
            if self.link.call(cmd="is_holding").get("holding"):
                self.link.call_slow(self.release_budget_s, cmd="release")
        except Exception:
            pass
        for fn in (lambda: self.svc.cancel_plan(self.zone, box_id), self._resume_belt):
            try:
                fn()
            except Exception:
                pass
