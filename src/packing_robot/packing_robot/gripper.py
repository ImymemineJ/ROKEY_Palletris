"""흡착 그리퍼 제어 — `grasp` / `release` / `is_holding` (작업명세서 6.4 · 6.5)

씬 쪽 구성은 `scripts/build_gripper_usd.py` 가 만든다. 이 모듈은 **런타임 조작만** 한다.
검증은 `scripts/verify_suction.py` 가 이 클래스를 그대로 써서 돈다 — 별도 배선이 없다.

## rclpy 를 import 하지 않는다

isaac_python(3.11)과 ROS 2 Jazzy(3.12)는 파이썬이 달라 한 프로세스에 못 섞인다.
관제탑 보고는 `on_status` 콜백으로 밖에 넘긴다 (`rmpflow_controller.py` 와 같은 원칙).

## 실측으로 알게 된 것 (2026-08-22, verify_suction.py)

- **`Closed` 를 믿으면 안 된다.** 상태가 `Closed` 로 바뀌고 `grippedObjects` 에 박스가
  올라와 있는데도 팔을 올리면 박스만 그 자리에 남는 경우가 실제로 나왔다
  (패드 D6 가 자유 조인트였을 때). 그래서 `is_holding()` 은 상태와 잡은 목록을 **둘 다** 본다.
  그것도 부족하면 `gap_fn` 으로 실제 간극까지 재게 해 뒀다.
- **`apply_gripper_action` 직후 상태를 읽으면 항상 `Closing`** 이다. 물리 스텝이 한 번도
  안 지났기 때문. `grasp()` 는 스텝을 돌리며 기다린다.
- **접근 간극은 2 mm 이하여야 한다.** 패드 D6 의 `transZ` 슬롯이 5 mm 라 그보다 멀리서
  잡으면 구속이 그 자리에 서고 드라이브가 못 끌어당긴다.
  실측: 접근 0 mm -> 흡착 후 −0.03 mm · 2 mm -> +1.24 mm · **5 mm -> +6.40 mm**.
- **질량은 병목이 아니다.** 9 kg 5호도 300 mm 중 299 mm 따라 올라온다. 힘 상한은 1e6 이라
  물리로는 안 걸린다 — 16 kgf 규제는 `system.yaml` 의
  `robot.suction.controller_limits` 로 **여기(컨트롤러)에서** 한다. `payload_reason()` 참조.

## 액션 값

`apply_gripper_action([+1])` = 닫기, `[-1]` = 열기. NVIDIA 자체 테스트도 ±0.5 를 쓴다
(부호만 본다).
"""

APPROACH_GAP_MM = 2.0        # 접근 시 남길 간극. transZ 슬롯(5 mm)보다 작아야 한다
GRIP_TOL_MM = 2.0            # 흡착 후 허용 간극. 넘으면 "떠서 붙은" 것으로 본다

_STATUS = ("Open", "Closing", "Closed")


def payload_reason(mass_kg, cfg):
    """집으면 안 되는 이유. 집어도 되면 None.

    씬의 `coaxialForceLimit` / `shearForceLimit` 은 1e6 이라 **물리로는 안 걸린다.**
    구 저장소가 두 갈래(`coaxialForceLimit`, `breakForce`)로 시도했다가 둘 다 재현성이
    없어서 컨트롤러 규제로 결론냈다 (`흡착_그리퍼_트러블슈팅` 문제②).

    집었다가 떨어뜨리면 박스가 깨지고 쌓아 둔 적재물까지 무너뜨린다. **시도 자체를 막는다.**
    """
    lim = cfg["robot"]["suction"]["controller_limits"]
    if mass_kg > lim["suction_limit_kgf"]:
        return "흡착력 상한 %.0f kgf 초과 (%.1f kg) — 붙여도 유지되지 않는다" % (
            lim["suction_limit_kgf"], mass_kg)
    if mass_kg > lim["box_mass_limit_kg"]:
        return "운용 상한 %.0f kg 초과 (%.1f kg) — 상판이 뜯어진다" % (
            lim["box_mass_limit_kg"], mass_kg)
    return None


class Gripper:
    """SurfaceGripper 하나를 잡는다.

    Args:
        prim_path: SurfaceGripper 프림 경로 (예: `/World/h2017/SurfaceGripper`)
        articulation: `SingleArticulation`. release 의 damping 스파이크에 쓴다.
        step: 물리 한 스텝 진행 (예: `lambda: world.step(render=False)`)
        cfg: `system.yaml` 딕셔너리
        gap_fn: () -> 흡착면~박스 윗면 간극 [mm]. 주면 `grasp` 이 결과를 실측으로 검증한다.
        tool_pose_fn: () -> 툴 월드 위치 (3,) [m]. 주면 release 가 TCP 속도를 기다린다.
        on_status: (str) -> None. "PICKED" / "PICK_FAILED" / "PLACED" 보고.
            ★ 안 붙이면 명세서 13 의 pick&place 성공률이 구조적으로 100% 가 된다.
        log: 로그 함수
    """

    def __init__(self, prim_path, articulation=None, step=None, cfg=None,
                 gap_fn=None, tool_pose_fn=None, on_status=None, log=print):
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.robot.surface_gripper")
        from isaacsim.robot.surface_gripper import GripperView

        self.path = prim_path
        self.art = articulation
        self._step = step or (lambda: None)
        self.cfg = cfg
        self._gap_fn = gap_fn
        self._tool_pose_fn = tool_pose_fn
        self._on_status = on_status
        self._log = log
        # reset_xform_op_properties 는 기본값(True)으로 둔다. GripperView 가 SurfaceGripper
        # 프림에 translate/orient/scale 을 심는데, 흡착 대상이 아니라 무해하다 (실측 확인).
        self.view = GripperView(paths=prim_path)   # 힘 상한 조정 등 원시 접근용

        sc = cfg["robot"]["suction"] if cfg else {}
        self.dt = 1.0 / cfg["physics"]["scene"]["time_steps_per_second"] if cfg else 1.0 / 60
        self.retry_s = float(sc.get("retry_interval_s", 10.0))
        r = cfg["robot"] if cfg else {}
        self.release_speed_m_s = float(r.get("release_speed_threshold_mm_s", 5.0)) / 1000.0
        self.release_timeout_s = float(r.get("release_wait_timeout_s", 3.0))
        self.release_damping = float(r.get("release_damping", 5.0))
        self.release_damping_s = float(r.get("release_damping_duration_s", 0.5))
        self.settle_s = float(r.get("settle_wait_s", 0.5))

    # ── 조회 ─────────────────────────────────────────────────
    def status(self):
        """"Open" | "Closing" | "Closed". 액션 직후에 부르면 항상 이전 값이다."""
        s = self.view.get_surface_gripper_status()[0]
        return s if isinstance(s, str) else _STATUS[int(s)]

    def gripped(self):
        """지금 잡고 있는 프림 경로 목록."""
        g = self.view.get_gripped_objects()[0]
        if isinstance(g, (list, tuple)):
            return [x for x in g if x]
        return [g] if g else []

    def is_holding(self):
        """정말 잡고 있는가.

        ★ 상태만 보면 안 된다. `Closed` 인데 실제로는 안 잡힌 경우가 실측으로 나왔다.
        """
        return self.status() == "Closed" and bool(self.gripped())

    def gap_mm(self):
        """흡착면~박스 윗면 간극 [mm]. `gap_fn` 을 안 줬으면 None."""
        return self._gap_fn() if self._gap_fn else None

    # ── 조작 ─────────────────────────────────────────────────
    def grasp(self, box_prim_path=None, timeout_s=None, steps_settle=None):
        """흡착한다. 붙을 때까지 물리를 돌린다.

        ★ 부르기 전에 흡착면을 박스 윗면 **APPROACH_GAP_MM 이내**로 내려놓아야 한다.
          5 mm 에서 부르면 6 mm 떠서 붙고, 그대로 성공으로 보고된다.

        Args:
            box_prim_path: 주면 **그 박스를 잡았는지**까지 확인한다. 옆 박스를 문 것을
                성공으로 세지 않으려면 줘야 한다.
            timeout_s: 기본은 `suction.retry_interval_s` (그리퍼가 닫기를 시도하는 총 시간)
            steps_settle: 붙은 뒤 더 돌릴 스텝 수. 기본 0.2 s
        Returns:
            bool
        """
        timeout_s = self.retry_s if timeout_s is None else timeout_s
        self.view.apply_gripper_action([1.0])

        n = max(1, int(timeout_s / self.dt))
        for _ in range(n):
            self._step()
            if self.is_holding():
                break

        for _ in range(steps_settle if steps_settle is not None else int(0.2 / self.dt)):
            self._step()

        ok, why = self._grasp_verdict(box_prim_path)
        if ok:
            self._report("PICKED")
            self._log("[gripper] 흡착 성공  %s%s"
                      % (self.gripped(), self._gap_str()))
        else:
            self._report("PICK_FAILED")
            self._log("[gripper] !! 흡착 실패 — %s  (상태 %s, 잡은 것 %s%s)"
                      % (why, self.status(), self.gripped() or "없음", self._gap_str()))
        return ok

    def _grasp_verdict(self, box_prim_path):
        if not self.is_holding():
            return False, "안 잡혔다"
        if box_prim_path and not any(box_prim_path in g for g in self.gripped()):
            return False, "다른 것을 물었다 (기대 %s)" % box_prim_path
        gap = self.gap_mm()
        if gap is not None and abs(gap) > GRIP_TOL_MM:
            # 여기서 걸리면 십중팔구 접근 간극이 멀었던 것이다. 패드 transZ 슬롯이 5 mm 다.
            return False, "%.2f mm 떠서 붙었다 (허용 %.1f)" % (gap, GRIP_TOL_MM)
        return True, ""

    def release(self, extra_settle_s=None):
        """놓는다. 박스가 튀지 않게 명세서 6.5 순서를 지킨다.

            TCP 가 멈출 때까지 대기 (타임아웃 있음) -> 릴리스 -> damping 스파이크
            -> 게인 원복 -> settle 대기

        ⚠ 타임아웃이 **반드시** 있어야 한다. 팔이 미세하게 계속 떨면 임계값에 영원히
          못 내려가서 무한루프가 된다 (명세서 6.5).
        """
        waited = self._wait_until_still()
        self.view.apply_gripper_action([-1.0])
        self._step()

        self._damping_spike()

        for _ in range(int((extra_settle_s if extra_settle_s is not None
                            else self.settle_s) / self.dt)):
            self._step()

        ok = not self.is_holding()
        self._log("[gripper] %s  (정지 대기 %.2f s, 상태 %s)"
                  % ("놓음" if ok else "!! 안 놓아진다", waited, self.status()))
        if ok:
            self._report("PLACED")
        return ok

    def open(self):
        """무조건 연다. 예외 경로에서 쓴다 — 대기도 스파이크도 없다."""
        self.view.apply_gripper_action([-1.0])
        self._step()

    # ── 내부 ─────────────────────────────────────────────────
    def _wait_until_still(self):
        """TCP 속도가 임계값 밑으로 내려갈 때까지 물리를 돌린다. 돌린 시간(s)을 돌려준다."""
        if self._tool_pose_fn is None:
            return 0.0
        import numpy as np
        t = 0.0
        prev = np.asarray(self._tool_pose_fn(), dtype=float)
        while t < self.release_timeout_s:
            self._step()
            t += self.dt
            cur = np.asarray(self._tool_pose_fn(), dtype=float)
            speed = float(np.linalg.norm(cur - prev)) / self.dt
            prev = cur
            if speed < self.release_speed_m_s:
                return t
        self._log("[gripper] release 대기 타임아웃 %.1f s — 강제 진행" % self.release_timeout_s)
        return t

    def _damping_spike(self):
        """놓는 순간 팔이 반동으로 흔들리지 않게 damping 을 잠깐 올렸다 되돌린다."""
        if self.art is None:
            for _ in range(int(self.release_damping_s / self.dt)):
                self._step()
            return
        ctl = self.art.get_articulation_controller()
        kps, kds = ctl.get_gains()
        try:
            ctl.set_gains(kds=kds * self.release_damping)
            for _ in range(int(self.release_damping_s / self.dt)):
                self._step()
        finally:
            ctl.set_gains(kps=kps, kds=kds)          # ★ 예외가 나도 반드시 되돌린다

    def _gap_str(self):
        gap = self.gap_mm()
        return "" if gap is None else "  간극 %+.2f mm" % gap

    def _report(self, status):
        if self._on_status:
            self._on_status(status)
