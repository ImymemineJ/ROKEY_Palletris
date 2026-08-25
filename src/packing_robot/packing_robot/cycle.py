"""pick & place 한 사이클 — 작업명세서 §2.5 / §2.6 (B4).

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

    def reset_stack(self):
        """팔레트를 갈았다 — 누적해 온 적재 최상단을 버린다."""
        self._stack_top_m = 0.0

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

        # 4) 집기 — 상공 접근 후 수직 하강
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
        pick_air = pick_top + self.approach_m
        cur = self._face_now()
        if cur is not None and cur[2] < pick_air - 1e-3:
            self._move([cur[0], cur[1], pick_air], pick_yaw, TRANSIT_TOL_MM)
        self._move([pick_c[0], pick_c[1], pick_air], pick_yaw, TRANSIT_TOL_MM)
        self._descend_to_gap(box_id, [pick_c[0], pick_c[1]], pick_top, pick_yaw)

        if not self._grasp(box_id):
            # §2.5 — 픽 실패는 계획을 버리고 벨트를 다시 돌린 뒤 종료한다.
            self.svc.set_status(box_id, "PICK_FAILED")
            self.svc.cancel_plan(self.zone, box_id)
            self._move([pick_c[0], pick_c[1], pick_top + self.approach_m], pick_yaw, TRANSIT_TOL_MM)
            self._resume_belt()
            return "PICK_FAILED"
        self.svc.set_status(box_id, "PICKED")

        # 5) 놓기 — 팔레트 좌표를 월드로 바꾸고 상공 접근 후 하강
        cx, cy, cz = pallet_place_to_world(place, h, self.origin_mm)
        put_top = cz + h / 2000.0
        # ★ 손목을 먼 쪽으로 180° 돌리지 않는다 — nearest_equiv_yaw 주석 참조.
        put_yaw = nearest_equiv_yaw(place.get("yaw_deg", 0.0), pick_yaw)

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
        self._move([cx, cy, put_top + GRASP_GAP_MM / 1000.0], put_yaw)

        # TCP 정지 대기 · damping 스파이크 · settle 은 robot_motion 안에서 돈다 (§6.5)
        if not self._release():
            self._log("[cycle] release 실패")
            self._abort(box_id)
            return "BLOCKED"
        self._move([cx, cy, put_top + self.approach_m], put_yaw, TRANSIT_TOL_MM)

        # 6) **실제** pose 로 갱신한다 — 계획값이 아니라 (§7.6)
        actual = self._actual_place(box_id, h) or place
        # ★ 적재 최상단 누적. **실제 상면**으로 올린다 — 계획대로 안 앉았으면
        #   그만큼 더 높을 수 있고, 이송은 그 실물을 넘어야 한다.
        act_top = (float(actual.get("z_mm", place.get("z_mm", 0.0))) + h
                   + self.origin_mm[2]) / 1000.0
        self._stack_top_m = max(self._stack_top_m, act_top)
        ok_hm, fill = self.svc.update_heightmap(self.zone, box_id, w, d, h, actual)
        if not ok_hm:
            # yaw 오차 초과 경고. 로그만 남기고 계속 간다 (§2.2)
            self._log("[cycle] update_heightmap ok=false — 계속 진행")
        # ★ 놓은 뒤에 누적한다. 이송 중에는 들고 있으므로 장애물이 아니다.
        self._obstacles.append(place_to_aabb_m(actual, w, d, h, self.origin_mm))
        self.svc.mark_placed(box_id, actual)
        self._resume_belt()
        self._log("[cycle] %s 완료  %.2f s  적재율 %s" % (box_id, time.time() - t0, fill))
        return "PLACED"

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
        return bool(self.link.call_slow(self.grasp_budget_s,
                                        cmd="grasp", box_id=box_id).get("ok"))

    def _release(self):
        return bool(self.link.call_slow(self.release_budget_s, cmd="release").get("ok"))

    def _actual_place(self, box_id, h_mm):
        """놓인 자리를 씬에서 직접 읽어 PlacePose 로 바꾼다. 못 읽으면 None."""
        r = self.link.call(cmd="get_box_pose", box_id=box_id)
        if not r.get("ok"):
            self._log("[cycle] get_box_pose 실패 — 계획값으로 갱신한다")
            return None
        return world_to_pallet_place(r["pos_m"], r.get("yaw_deg", 0.0), h_mm, self.origin_mm)

    def _resume_belt(self):
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
