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
import time

RESULTS = ("PLACED", "PICK_FAILED", "PALLET_FULL", "BLOCKED", "IK_FAIL")

MOTION_TIMEOUT_S = 20.0          # 한 구간 이동에 이보다 오래 걸리면 이상한 것이다
MOTION_POLL_S = 0.02
GRASP_GAP_MM = 2.0               # 흡착 직전 흡착면을 박스 상면 위 이만큼에 둔다
                                 #   ★ 패드 transZ 슬롯이 5 mm 다. 더 벌리면 떠서 붙는다
                                 #     (packing_robot.gripper.APPROACH_GAP_MM 과 같은 값)
GAP_FIX_TOL_MM = 0.5             # 실측 간극이 목표에서 이보다 벗어나면 한 번 보정한다


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


class MotionError(Exception):
    """이동이 실패했다. state 에 motion_status 값이 들어 있다."""

    def __init__(self, state):
        super().__init__(state)
        self.state = state


class Cycle:
    def __init__(self, link, svc, cfg, zone="B", log=print):
        self.link = link
        self.svc = svc
        self.cfg = cfg
        self.zone = zone
        self._log = log
        z = cfg["zones"]
        self.origin_mm = list(z["pallet_origin_world_mm"][zone])
        self.approach_m = cfg["robot"]["approach_offset_mm"] / 1000.0
        self.belt_seg = "%s_pickup" % zone.lower()
        self.belt_mps = float(cfg["conveyor"]["belt_velocity_mps"])
        self.grasp_budget_s = float(cfg["robot"]["suction"]["retry_interval_s"]) + 3.0
        r = cfg["robot"]
        self.release_budget_s = (r["release_wait_timeout_s"] + r["release_damping_duration_s"]
                                 + r["settle_wait_s"] + 3.0)

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
        self._move([pick_c[0], pick_c[1], pick_top + self.approach_m], pick_yaw)
        self._descend_to_gap(box_id, [pick_c[0], pick_c[1]], pick_top, pick_yaw)

        if not self._grasp(box_id):
            # §2.5 — 픽 실패는 계획을 버리고 벨트를 다시 돌린 뒤 종료한다.
            self.svc.set_status(box_id, "PICK_FAILED")
            self.svc.cancel_plan(self.zone, box_id)
            self._move([pick_c[0], pick_c[1], pick_top + self.approach_m], pick_yaw)
            self._resume_belt()
            return "PICK_FAILED"
        self.svc.set_status(box_id, "PICKED")

        # 5) 놓기 — 팔레트 좌표를 월드로 바꾸고 상공 접근 후 하강
        cx, cy, cz = pallet_place_to_world(place, h, self.origin_mm)
        put_top = cz + h / 2000.0
        put_yaw = float(place.get("yaw_deg", 0.0))

        # ★ 상승 -> **가로 이송** -> 하강. 세 구간이다 (§2.5).
        #   집자마자 배치 상공으로 바로 가면 관절공간 직선이 대각선을 그어서, 매달린
        #   박스가 벨트를 뚫고 지나간다. 벨트를 확실히 넘는 높이에서 가로로 옮긴 뒤
        #   내려가야 한다. 매달린 박스는 흡착면보다 h 만큼 아래에 있으므로 그만큼 더 든다.
        transit = max(pick_top, put_top) + self.approach_m + h / 1000.0
        self._move([pick_c[0], pick_c[1], transit], pick_yaw)
        self._move([cx, cy, transit], put_yaw)
        self._move([cx, cy, put_top + self.approach_m], put_yaw)
        self._move([cx, cy, put_top + GRASP_GAP_MM / 1000.0], put_yaw)

        # TCP 정지 대기 · damping 스파이크 · settle 은 robot_motion 안에서 돈다 (§6.5)
        if not self._release():
            self._log("[cycle] release 실패")
            self._abort(box_id)
            return "BLOCKED"
        self._move([cx, cy, put_top + self.approach_m], put_yaw)

        # 6) **실제** pose 로 갱신한다 — 계획값이 아니라 (§7.6)
        actual = self._actual_place(box_id, h) or place
        ok_hm, fill = self.svc.update_heightmap(self.zone, box_id, w, d, h, actual)
        if not ok_hm:
            # yaw 오차 초과 경고. 로그만 남기고 계속 간다 (§2.2)
            self._log("[cycle] update_heightmap ok=false — 계속 진행")
        self.svc.mark_placed(box_id, actual)
        self._resume_belt()
        self._log("[cycle] %s 완료  %.2f s  적재율 %s" % (box_id, time.time() - t0, fill))
        return "PLACED"

    # ── 조각 ─────────────────────────────────────────────────
    def _move(self, face_m, yaw_deg):
        """흡착면을 face_m 으로 보낸다. 도착까지 기다린다."""
        r = self.link.call(cmd="move_ee", target_m=[float(v) for v in face_m],
                           yaw_deg=float(yaw_deg))
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
