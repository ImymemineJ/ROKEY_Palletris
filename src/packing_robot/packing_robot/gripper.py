"""흡착 그리퍼 제어 — `grasp` / `release` / `is_holding` (작업명세서 6.4 · 6.5)

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : ⑦ 흡착 · ⑧ 해제
    ║     불러 주는 쪽        : cycle
    ║     이 파일이 부르는 쪽 : Isaac SurfaceGripper 프림
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

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
# 흡착 후 허용 간극. **패드 transZ 슬롯 5 mm 에서 온 값이다** — 임의로 정한 것이 아니다.
#   접근 2 mm 로 잡으면 1.2~1.4 mm, 접근 5 mm 로 잡으면 6.4 mm 가 나온다. 3 mm 가 그 사이다.
#   ★ 2.0 으로 뒀다가 멀쩡한 흡착(2.41 mm)을 실패로 판정했다. 도착 잔차가 얹히기 때문이다.
GRIP_TOL_MM = 3.0

# 릴리스 전 최소 유지 시간 [s]. TCP 정지 판정은 **툴** 속도라 매달린 박스의 진동을 못 본다.
#   패드 rotXY 드라이브가 강성 100 · 감쇠 0 (h2017_cell.usd 정본값)이라 5호(9 kg ·
#   340 mm)는 흔들림이 잘 안 죽는데, 릴리스가 어느 위상에 걸리느냐로 착지가 22~76 mm
#   갈렸다 (2026-08-22 실측 — 지령·간극이 동일한 두 실행이 0.4 mm 와 27 mm 로 갈림).
#   바닥 2 mm 위에서 이 시간만큼 머물면 박스가 표면에 얹혀(±3° 순응 안에서) 진동이
#   접촉 마찰로 죽는다. 정본 패드값은 건드리지 않는다.
RELEASE_MIN_HOLD_S = 0.5

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
        self._g = None                       # begin_grasp 진행 상태
        self._r = None                       # begin_release 진행 상태

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

    # ── 조작 (비차단) ────────────────────────────────────────
    #
    #  `robot_motion.py` 는 Isaac 메인 루프를 세우면 안 된다 (공통가이드 §3 규칙 1).
    #  그래서 **상태를 뱉는 begin/poll 이 본체**이고, 아래 grasp()/release() 는 그걸
    #  자기가 스텝 돌려 감싼 편의 함수다. 구현이 하나여야 검증도 하나로 끝난다.

    def begin_grasp(self, box_prim_path=None, timeout_s=None):
        """흡착을 건다. **기다리지 않는다.** 이후 `poll_grasp()` 로 결과를 본다."""
        self.view.apply_gripper_action([1.0])
        self._g = {"box": box_prim_path,
                   "left": max(1, int((self.retry_s if timeout_s is None else timeout_s)
                                      / self.dt)),
                   "settle": int(0.2 / self.dt), "done": None}

    def poll_grasp(self):
        """None = 진행 중, True/False = 판정 끝. 물리 스텝마다 한 번씩 부른다."""
        g = self._g
        if g is None:
            return None
        if g["done"] is not None:
            return g["done"]
        if not self.is_holding():
            g["left"] -= 1
            if g["left"] > 0:
                return None
        elif g["settle"] > 0:                 # 붙었으면 조금 더 안정화시킨 뒤 잰다
            g["settle"] -= 1
            return None
        ok, why = self._grasp_verdict(g["box"])
        g["done"] = ok
        self._report("PICKED" if ok else "PICK_FAILED")
        if ok:
            self._log("[gripper] 흡착 성공  %s%s" % (self.gripped(), self._gap_str()))
        else:
            self._log("[gripper] !! 흡착 실패 — %s  (상태 %s, 잡은 것 %s%s)"
                      % (why, self.status(), self.gripped() or "없음", self._gap_str()))
        return ok

    def begin_release(self, extra_settle_s=None):
        """놓기를 시작한다. **기다리지 않는다.** 명세서 6.5 순서를 poll 이 진행한다.

            TCP 정지 대기(타임아웃 있음) -> 릴리스 -> damping 스파이크 -> 게인 원복
            -> settle 대기
        """
        settle = self.settle_s if extra_settle_s is None else extra_settle_s
        self._r = {"phase": "wait", "left": int(self.release_timeout_s / self.dt),
                   "hold": int(RELEASE_MIN_HOLD_S / self.dt),
                   "spike": int(self.release_damping_s / self.dt),
                   "settle": int(settle / self.dt), "prev": None, "waited": 0.0,
                   "gains": None, "done": None}

    def poll_release(self):
        """None = 진행 중, True/False = 판정 끝."""
        import numpy as np
        r = self._r
        if r is None:
            return None
        if r["done"] is not None:
            return r["done"]

        if r["phase"] == "wait":
            # ⚠ 타임아웃이 반드시 있어야 한다. 팔이 미세하게 계속 떨면 임계값에 영원히
            #   못 내려가서 무한루프가 된다 (명세서 6.5).
            still = self._tool_pose_fn is None
            if not still:
                cur = np.asarray(self._tool_pose_fn(), dtype=float)
                if r["prev"] is not None:
                    still = float(np.linalg.norm(cur - r["prev"])) / self.dt \
                        < self.release_speed_m_s
                r["prev"] = cur
            if r["hold"] > 0:                  # 최소 유지 — 박스 진동은 TCP 로 안 보인다
                r["hold"] -= 1
                still = False
            r["left"] -= 1
            r["waited"] += self.dt
            if not still and r["left"] > 0:
                return None
            if not still:
                self._log("[gripper] release 대기 타임아웃 %.1f s — 강제 진행"
                          % self.release_timeout_s)
            self.view.apply_gripper_action([-1.0])
            r["phase"] = "spike"
            r["gains"] = self._spike_on()
            return None

        if r["phase"] == "spike":
            r["spike"] -= 1
            if r["spike"] > 0:
                return None
            self._spike_off(r["gains"])       # ★ 반드시 되돌린다. 안 그러면 팔이 굳는다
            r["phase"] = "settle"
            return None

        r["settle"] -= 1
        if r["settle"] > 0:
            return None
        ok = not self.is_holding()
        r["done"] = ok
        self._log("[gripper] %s  (정지 대기 %.2f s, 상태 %s)"
                  % ("놓음" if ok else "!! 안 놓아진다", r["waited"], self.status()))
        if ok:
            self._report("PLACED")
        return ok

    # ── 조작 (차단 — 단일 스레드 호출자용) ──────────────────
    def grasp(self, box_prim_path=None, timeout_s=None, steps_settle=None):
        """흡착한다. 붙을 때까지 **자기가 물리를 돌린다.**

        ★ 부르기 전에 흡착면을 박스 윗면 **APPROACH_GAP_MM 이내**로 내려놓아야 한다.
          5 mm 에서 부르면 6 mm 떠서 붙고, 그대로 성공으로 보고된다.
        """
        self.begin_grasp(box_prim_path, timeout_s)
        if steps_settle is not None:
            self._g["settle"] = steps_settle
        while True:
            self._step()
            r = self.poll_grasp()
            if r is not None:
                return r

    def release(self, extra_settle_s=None):
        """놓는다. 명세서 6.5 순서를 지키며 **자기가 물리를 돌린다.**"""
        self.begin_release(extra_settle_s)
        while True:
            self._step()
            r = self.poll_release()
            if r is not None:
                return r

    def open(self):
        """무조건 연다. 예외 경로에서 쓴다 — 대기도 스파이크도 없다.

        진행 중이던 release 의 damping 스파이크가 걸려 있으면 먼저 되돌린다.
        """
        if self._r and self._r["phase"] == "spike":
            self._spike_off(self._r["gains"])
        self._g = self._r = None
        self.view.apply_gripper_action([-1.0])
        self._step()

    # ── 내부 ─────────────────────────────────────────────────
    def _grasp_verdict(self, box_prim_path):
        """붙었다고 볼 수 있는가. **상태만으로 판정하지 않는다.**"""
        if not self.is_holding():
            return False, "안 잡혔다"
        if box_prim_path and not any(box_prim_path in g for g in self.gripped()):
            return False, "다른 것을 물었다 (기대 %s)" % box_prim_path
        gap = self.gap_mm()
        if gap is not None and abs(gap) > GRIP_TOL_MM:
            # 여기서 걸리면 십중팔구 접근 간극이 멀었던 것이다. 패드 transZ 슬롯이 5 mm 다.
            return False, "%.2f mm 떠서 붙었다 (허용 %.1f)" % (gap, GRIP_TOL_MM)
        return True, ""

    def _spike_on(self):
        """damping 을 잠깐 올린다. 원래 게인을 돌려준다 (없으면 None)."""
        if self.art is None:
            return None
        ctl = self.art.get_articulation_controller()
        kps, kds = ctl.get_gains()
        ctl.set_gains(kds=kds * self.release_damping)
        return (kps, kds)

    def _spike_off(self, gains):
        if gains is None:
            return
        self.art.get_articulation_controller().set_gains(kps=gains[0], kds=gains[1])

    def _gap_str(self):
        gap = self.gap_mm()
        return "" if gap is None else "  간극 %+.2f mm" % gap

    def _report(self, status):
        if self._on_status:
            self._on_status(status)
