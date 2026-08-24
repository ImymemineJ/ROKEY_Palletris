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
REACH_TOL_M = 0.001                       # 흡착면이 목표에서 이 안이면 REACHED
SETTLE_MAX_S = 1.0                        # 궤적을 다 보낸 뒤 수렴을 기다리는 한도
SETTLE_HOLD = 3                           # 연속 이만큼 만족해야 REACHED
ARC_MIN_R_M = 0.15                        # 반경이 이보다 작으면 각도가 무의미 — 직선
BISECT = 10                               # 속도한계를 넘으면 u 증분을 이만큼까지 반절
MOVE_LIMIT_S = 20.0                       # 한 구간 상한. 무한루프 방지
LOG_RESIDUAL_M = 0.005                    # 이보다 큰 잔차만 로그. 1~2 mm 는 정상 처짐이다
# ★ 잔차가 이보다 크면 **REACHED 로 보고하지 않는다.** 정상 처짐(1~2 mm)이나
#   RMP 계열 정상상태 오차(check_place_plan 의 RMP_SAG_MM 35)보다 훨씬 큰 값이라
#   여기 걸리면 '못 갔다'가 맞다. 이게 없어서 343 mm 를 못 간 채 REACHED 를 주고,
#   cycle 이 그 자리에서 박스를 놓아 팔레트 밖으로 떨어뜨렸다 (0824error1.mp4).
REACH_ABORT_M = 0.05                      # 50 mm

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
#   ★★ 기본 keep — **이 구현은 아직 쓰면 안 된다.** (2026-08-24, 0824error1.mp4)
#     `_advance_move` 는 매 스텝 IK 를 **직전 해**(p["prev"], q0 에서 출발)로 체인한다.
#     여기서 고른 가지는 `q_end` 에만 들어가고 **경로에는 안 쓰인다** — 오직
#     steps 계산(dq = |q_end - q0|)에만 반영된다. 즉 가지는 안 바뀌고 페이싱만 어긋난다.
#     실측 결과: 'wflip' 을 고른 구간에서 도착 잔차 **343 mm**, 그 자리에서 놓아
#     박스가 팔레트 밖으로 떨어졌다.
#
#     측정 도구(verify_transit_sweep_isaac.py --seeds multi)는 **시드에서 체인을
#     시작**하므로 +32.9 -> +70.2 mm 가 나왔다. 컨트롤러는 q0 에서 시작한다. 이 차이다.
#
#     제대로 하려면 고른 가지로 팔을 **실제로 재구성하는 이동**이 먼저 필요하다
#     (안전 고도에서 관절공간 이동). 그건 별개 작업이라 지금은 끈다.
IK_SEEDS = os.environ.get("PL_IK_SEEDS", "keep")
SEED_PROBE_N = 8                          # 시드 점수용 경로 표본 수 (성기게)
# ★ 현행 가지(keep)가 이만큼 여유로우면 **갈아타지 않는다.**
#   가지를 바꾸면 관절 이동량이 커져 사이클이 길어진다 — 실측 6박스에서
#   13~17 s -> 12~32 s 로 거의 2배가 됐다 (IK 계산 비용이 아니라 j1flip 이 팔을
#   반대편으로 돌리기 때문이다). 여유가 이미 충분하면 그 비용을 낼 이유가 없다.
#   값은 배치 오차(실측 7 mm)와 RMP 처짐(35 mm)을 합친 것보다 넉넉하게 잡는다.
SEED_KEEP_OK_M = 0.06                     # 60 mm


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


def _quat_down(yaw_deg):
    """툴 z 축이 월드 −Z 를 보고, 그 축 둘레로 yaw 만큼 돌린 자세. (w, x, y, z)"""
    h = math.radians(yaw_deg) / 2.0
    # R = Rz(yaw) · Ry(pi). Ry(pi) = (w,x,y,z) = (0,0,1,0) 이므로
    #   q = qz * qy = (0, -sin(h), cos(h), 0)
    return np.array([0.0, -math.sin(h), math.cos(h), 0.0])


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
        self._hold = 0
        self._settle = 0
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
            self._req = (target, float(req.get("yaw_deg", 0.0)))
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

    def suction_pose(self):
        """흡착면 월드 위치 (3,) [m]. 툴 +Z 로 그리퍼 길이만큼 나간 점."""
        q = np.asarray(self.art.get_joint_positions()[:6], dtype=float)
        p, R = self.lula.compute_forward_kinematics(self.ee, q)
        return np.asarray(p, dtype=float) + np.asarray(R)[:, 2] * self.grip_len

    def _update_tcp_speed(self):
        cur = self.tool_pose()
        with self._lk:
            if self._prev_tool is not None:
                self._tcp_mps = float(np.linalg.norm(cur - self._prev_tool)) / self.dt
            self._prev_tool = cur

    def _ik(self, face, yaw_deg, seed):
        """흡착면 목표 -> (q, ok). tool0 목표는 흡착면 + 그리퍼 길이다.

        `ArticulationKinematicsSolver` 를 쓰지 않는다 — 그건 현재 관절각을 시드로
        **강제**해서(`articulation_kinematics_solver.py:94`) 시드를 고를 수가 없다.
        """
        tgt = np.asarray(face, dtype=float) + np.array([0.0, 0.0, self.grip_len])
        q, ok = self.lula.compute_inverse_kinematics(
            frame_name=self.ee, target_position=tgt,
            target_orientation=_quat_down(yaw_deg),
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

    def _pick_seed(self, face, yaw_to, q0):
        """끝점과 경로를 함께 보고 가장 여유로운 가지를 고른다. -> (q_end, ok, 이름).

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
                    return q_keep, True, "keep"
        for name, sd in bank:
            q_end, ok = self._ik(face, yaw_to, sd)
            if not ok:
                continue
            fk = np.asarray(self.lula.compute_forward_kinematics(self.ee, q_end)[0])
            if np.linalg.norm(fk - (np.asarray(face)
                                    + np.array([0.0, 0.0, self.grip_len]))) > IK_TOL_M:
                continue
            score, q = self._arm_clearance(q_end), q_end
            if self._obst and self.spheres:
                for k in range(1, SEED_PROBE_N):
                    u = k / float(SEED_PROBE_N)
                    qi, oki = self._ik(f0 + (np.asarray(face) - f0) * u, yaw_to, q)
                    if not oki:
                        continue
                    q = qi
                    score = min(score, self._arm_clearance(qi))
            if best is None or score > best[0]:
                best = (score, q_end, name)
        if best is None:
            return None, False, ""
        return best[1], True, best[2]

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
        face, yaw_req = req
        q0 = np.asarray(self.art.get_joint_positions()[:6], dtype=float)
        f0 = self.suction_pose()
        _, R0 = self.lula.compute_forward_kinematics(self.ee, q0)
        yaw_from = _tool_yaw_deg(R0)
        # 직사각 박스는 yaw 와 yaw+180 이 같은 자리다. 손목을 덜 돌리는 쪽으로 잡는다.
        yaw_to = _near(yaw_req, yaw_from, 180.0)

        # ★ 가지를 여기서 한 번 고른다 (구간 내내 유지된다). 장애물이나 충돌구체가
        #   없으면 _seed_bank 가 keep 하나만 주므로 예전과 완전히 같은 경로다.
        q_end, ok, seed_name = self._pick_seed(face, yaw_to, q0)
        if ok and seed_name and seed_name != "keep":
            self._log("[robot_motion] IK 가지 '%s' 선택 (여유 %+.0f mm)"
                      % (seed_name, self._arm_clearance(q_end) * 1000))
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
        dq = np.abs(q_end - q0)
        secs = float(np.max(dq / (self.vlim * self.speed_scale))) if np.any(dq) else 0.0
        steps = max(1, int(math.ceil(secs * self.hz)))
        self._path = {"f0": f0, "f1": np.asarray(face, dtype=float),
                      "arc": r0 > ARC_MIN_R_M and r1 > ARC_MIN_R_M,
                      "r0": r0, "r1": r1, "th0": th0,
                      "dth": _wrap(math.atan2(v1[1], v1[0]) - th0),
                      "yaw0": yaw_from, "yaw1": yaw_to,
                      "u": 0.0, "du": 1.0 / steps, "prev": q0, "k": 0,
                      "miss": 0, "over": 0}
        self._goal_face = np.asarray(face, dtype=float)
        self._hold = 0
        self._settle = int(SETTLE_MAX_S * self.hz)
        self._state = "MOVING"
        self._log("[robot_motion] move -> %s yaw %.1f -> %.1f  %s  |dq| %.1f deg  n %d"
                  % (np.round(face, 4), yaw_from, yaw_to,
                     "호" if self._path["arc"] else "직선",
                     float(np.rad2deg(np.max(dq))), steps))

    def _advance_move(self):
        """호를 한 스텝 전진시킨다. **IK 는 스텝당 최소 1회.**"""
        p = self._path
        if p is None:
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
            cand, cok = self._ik(self._at(p, u2),
                                 p["yaw0"] + (p["yaw1"] - p["yaw0"]) * u2, p["prev"])
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
        if p["k"] > int(MOVE_LIMIT_S * self.hz):
            self._state = "IK_FAIL"
            self._path = None
            self._log("[robot_motion] 구간이 %.0f s 를 넘었다 (u %.2f, IK 실패 %d)"
                      % (MOVE_LIMIT_S, p["u"], p["miss"]))

    def _settle_move(self, p):
        """끝점 지령을 유지하며 흡착면이 실제로 목표에 들어오는지 본다."""
        self.art.apply_action(self._Action(joint_positions=p["prev"]))
        err = float(np.linalg.norm(self.suction_pose() - self._goal_face))
        if err < REACH_TOL_M:
            self._hold += 1
            if self._hold >= SETTLE_HOLD:
                self._reach_done(err, p)
            return
        self._hold = 0
        self._settle -= 1
        if self._settle <= 0:
            if err > LOG_RESIDUAL_M:
                self._warn_residual(p, err)
            if err > REACH_ABORT_M:
                # ★ 여기서 REACHED 를 주면 호출부가 **그 자리에서 놓는다.**
                #   실패로 알려 cycle 이 이탈 경로(cancel_plan -> 복귀)를 타게 한다.
                self._log("[robot_motion] 잔차 %.0f mm > 한도 %.0f mm — 실패로 알린다"
                          % (err * 1000, REACH_ABORT_M * 1000))
                self._state = "IK_FAIL"
                self._path = None
                return
            self._reach_done(err, p)

    def _warn_residual(self, p, err):
        # ⚠ 여기서 영원히 기다리면 안 된다 — 정상상태 오차가 남는 제어기에서는
        #   임계값에 영영 못 들어간다 (구 저장소 RMPflow 교훈). 한도를 두고
        #   잔차를 로그로 남긴 뒤 넘어간다.
        self._log("[robot_motion] 도착 잔차 %.2f mm (허용 %.1f) — 한도 초과, 진행\n"
                  "               흡착면 %s -> 목표 %s  (IK 실패 %d · 속도초과 %d)"
                  % (err * 1000, REACH_TOL_M * 1000, np.round(self.suction_pose(), 4),
                     np.round(self._goal_face, 4), p["miss"], p["over"]))

    def _reach_done(self, err, p=None):
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
