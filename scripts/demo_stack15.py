#!/usr/bin/env python3
"""영상용 데모 — 박스 15개를 **실제 물리로** 팔레트에 쌓는다 (기본 계획 4층 · 최상단 1040 mm).

    # 1) 계획만 만들고 책상에서 거른다 (Isaac 필요 없다, 1초)
    python3 scripts/demo_stack15.py --plan-only --dump-plan results/stack15.json
    python3 scripts/check_place_plan.py results/stack15.json

    # 2) Isaac 에서 실제로 쌓는다
    ~/isaac-sim-5.1.0/python.sh scripts/demo_stack15.py --gui --hold
    ~/isaac-sim-5.1.0/python.sh scripts/demo_stack15.py            # 헤드리스 (판정용)

## 새로 만든 것은 **글루뿐**이다

이 스크립트는 B3·B4 산출물을 **그대로** 15번 돌린다. 검증 전용 사본을 만들지 않는다는
`verify_b4_cycle.py` 의 원칙을 그대로 잇는다.

    packing_robot/cycle.py            상태머신          그대로
    packing_sim/isaac/robot_motion.py 호 이송 · 명령     그대로
    packing_robot/gripper.py          흡착              그대로
    packing_robot/sim_link.py         소켓              그대로
    verify_b4_cycle.StandInServer     소켓 서버 대역     그대로 import
    verify_suction.make_*             씬 빌더           그대로 import
    verify_lula_ik.bring_up           Isaac 배선        그대로 import
    packing_sim/packing.HeightmapPacker  배치 계획      그대로 import

여기서 새로 쓴 것은 (a) 15개용 `PlanServices`, (b) 박스 15개 스폰·급송, (c) knock 측정뿐이다.

## 배치 계획을 `mixed_plan` 으로 하지 않는 이유

`verify_cycle.py:123 mixed_plan()` 은 **드라이런용 더미**다. 물리가 없는 곳에서만 검증됐고,
실제로 seed 42 · 15박스 계획을 지지율로 재 보면 **15개 중 4개가 70% 미만**이다
(28% · 49% · 51% · 52%). 드라이런에서는 박스가 VisualCuboid 라 안 보이지만 물리로 쌓으면
그 4개가 넘어간다. 그래서 진짜 패커인 `packing.HeightmapPacker` 를 쓴다 —
`enforce_load_order` 로 무거운 박스를 가벼운 것 위에 안 올리고 지지율도 스스로 지킨다.

## 급송 순서 — 부피 큰 것부터다

15개를 **도착 순서 그대로** 넣으면 패커가 14/15 밖에 못 넣는다 (5호 하나 탈락).
부피 내림차순으로 정렬하면 15/15 가 들어간다. 이 데모는 오프라인 계획이므로 정렬해서 급송한다.
실물은 버퍼(zones.buffer_capacity 4)나 C 의 온라인 패커가 그 자리를 맡는다.

## knock 을 반드시 본다

구 저장소는 2박스째에서 **들고 가던 박스가 앞서 놓은 박스를 1,061 mm 밀어 팔레트 밖으로
떨어뜨렸다.** 원인은 이송 높이를 흡착면 기준으로 잡아 매달린 박스 높이를 빼먹은 것이다.
우리 `cycle.py:153` 은 `+ h/1000.0` 으로 그걸 넣어 뒀지만, **박스 하나짜리 시험으로는
확인할 수 없다 — 칠 것이 없기 때문이다.** 그래서 사이클마다 이미 놓인 박스가 얼마나
움직였는지 재고 표에 찍는다.

## 실측 (2026-08-24, 헤드리스, 페데스탈 900, 2회) — **RESULT: 통과**

    놓은 것 15/15 · 7 단 (밑면 z 0/280/340/560/680/770/840) · 최상단 1050 mm
    배치 오차 14개가 4.4 mm 안 · Z 오차 전부 0.0
    **최대 knock 0.1 ~ 3.8 mm** (경고선 20) — 팔이 적재물을 쓸던 문제(26~115 mm)는 사라졌다
    joint_5 −90 ~ +23 deg (한계 ±360 — 15회 돌려도 안 감긴다)
    32 s 벽시계 · 257 s 시뮬 (사이클 평균 17.1 s)

    ⚠ 남은 변동: 13번(3호 P(700,185) z680)이 실행에 따라 0.4 ~ 12.3 mm 로 흔들린다.
      **마지막 하강 구간에서** 생기고(구간별 추적: 6번 0.9 mm -> 7번 9.2 mm), 밑에 깔린
      7번 박스를 3.8 mm 민다. 허용(30)·경고선(20) 안이라 게이트는 통과하지만
      "하강이 지지면을 스친다" 계열이라 지켜볼 것.

여기까지 오는 데 밟은 순서 문제 셋 (전부 results/issues/18):

    z 오름차순      → 근거리 기둥(890)이 먼저 서고, far 배치 때 link_2 가 쓸었다 (115 mm)
    far→near 통짜   → far 기둥(980)이 먼저 서고, 근거리 (765,205) 깊은 하강 때
                      팔꿈치(link_3/4)가 목표 너머로 튀어나가 밀었다 (94 mm)
    끼워넣기(현행)  → far 하단 → 근거리 5호 → far 꼭대기 → 근거리 3호. knock 0.2 mm

⚠ **페데스탈 900 이상이 전제다** — system.yaml 이 2026-08-22 에 900 으로 확정됐다
(issues/18). 700 에서는 팔꿈치가 560 짜리 기둥에도 −65 mm 라 어떤 순서로도 안 나온다.
다른 높이를 시험할 때만 `--pedestal` 로 덮어쓴다 (system.yaml 은 안 건드린다).

★ (765,205) z340 5호가 28~33 mm 로 앉던 결함은 **2026-08-24 해결됐다** (results/issues/21).
원인은 배치→픽 **복귀호에 안전 고도가 없던 것**이다. 호는 z 를 선형 보간하므로 직전 배치가
1층이면 후퇴가 낮고(0.69), 픽 상공(1.34)으로 오는 중간이 **픽업 대기 박스 상면(1.141)보다
낮아져** 그리퍼 몸통(Ø245)이 박스를 쓸어 밀었다. 밀린 박스를 그대로 잡아 그대로 놓으니
배치 오차 = 밀린 양이 됐다. `cycle.py` 가 이제 복귀에도 **먼저 수직 상승**을 넣는다
(30 mm -> 1.5 mm, knock 22~32 -> 0.1 mm). 대가는 사이클 +1.1 s 다.

## 계획을 어떻게 만드나

기본은 **오프라인**이다 — `HeightmapPacker` 로 15개를 미리 배치하고 **밑면 z 오름차순**으로
실행한다. 좌표는 안 바꾸고 순서만 바꾼다. `cycle.py` 의 이송 높이가 적재 최상단을
모르기 때문에(issues/17) z 가 오르내리면 이송 중 관통한다.

`--plan-mode online` 은 **카메라가 붙었을 때의 구조**다 — 사이클마다 팔레트를 다시 보고
(`PalletVision.scan()`) 하이트맵을 새로 세워 그 자리에서 다음 자리를 정하고,
`stack_top_mm` 을 `cycle.py` 에 실어 보낸다. `cycle.py` 는 이미 실제 pose 를 돌려주므로
루프가 닫힌다. **다만 지금은 오프라인보다 나쁘다** (knock 956 mm, 박스가 팔레트 밖으로
떨어진다) — 관측 오차가 자리마다 누적되고 HeightGreedy 가 파편화된 웅덩이를 만들기
때문으로 보인다. → results/issues/19

## 알려진 제약

* 박스는 벨트를 타고 오지 않는다. 대기 구역에 스폰해 두고 사이클마다 픽업 자리로 옮긴다
  (`world.reset()` 을 한 번만 부르기 위해서다 — 중간에 부르면 이미 쌓은 박스가 되돌아간다).
  대기 구역은 기본 카메라 **뒤**라 화면에 안 나온다. `--cam` 을 바꾸면 나올 수 있다.
* 손목 되감기(`unwrap_wrist`)가 production 경로에 없다. `verify_cycle.py:556` 에만 있다.
  15회 실측으로는 감기지 않았지만(−104 ~ +16°) 코드가 없는 것은 사실이라 범위를 찍는다.
* `cycle.py` 를 두 군데 고쳤다 (**미커밋**) — 배치 approach 에 `+h`, 이송 높이에
  선택 필드 `stack_top_mm`. 둘 다 정본 `check_place_plan.py:176` 과 맞춘 것이고
  B4 회귀는 통과한다.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                          # noqa: E402
import verify_lula_ik as V                             # Isaac 배선  # noqa: E402
import verify_suction as VS                            # 씬 빌더  # noqa: E402
import verify_b4_cycle as B4                           # StandInServer 대역  # noqa: E402
import verify_reach as VR                              # 충돌구체 로더  # noqa: E402

PORT = 8798                                            # 8765=A, 8799=verify_b4_cycle
GRIPPER = "/World/h2017/SurfaceGripper"
BOX_PRIM_FMT = "/World/boxes/%s"                       # robot_motion.BOX_PRIM_FMT 와 같아야 한다
BOX_ID_FMT = "BOX%05d"

# 계획 파라미터. ★ system.yaml 에 없다 — config.py 의 PACK_BOX_CLEARANCE 0.030 /
#   PACK_DESCENT_HALO 0.040 이 유일한 출처다. 3층 성패를 가르는 값이라 등재가 필요하다
#   (results/issues 참조). 여기서는 CLI 로 덮어쓸 수 있게 두고 기본값만 박는다.
CLEARANCE_MM = 30.0        # 이웃 간격 = config.py PACK_BOX_CLEARANCE.
#   ★ 20 으로 줄이면 층이 4단으로 깔끔해지지만 이웃 간격이 20 mm(한쪽 10 mm)가 되고,
#     210 mm 짜리 박스를 그 구석에 수직으로 꽂다가 모서리에 걸려 이웃 위로 올라탄다
#     (2026-08-22 실측: 13번째 박스 Z 오차 +208 mm, 이웃 100 mm 밀림).
#     30 이면 간격 30 mm(한쪽 15 mm)이고 양쪽이 갇힌 채 놓는 박스가 3개 -> 1개로 준다.
DESCENT_HALO_MM = 40.0     # 하강 통로 = config.py PACK_DESCENT_HALO. 0 이면 옆을 스친다

KNOCK_WARN_MM = 20.0       # 이미 놓인 박스가 이만큼 움직이면 경고
PLACE_TOL_XY_MM = 30.0
PLACE_TOL_Z_MM = 20.0
CYCLE_WALL_S = 120.0       # 사이클 하나당 벽시계 상한
SETTLE_S = 0.5             # 사이클 사이 정착

# 대기 구역 — 지면(z=0) 위에 세워 둔다. ★ 기본 카메라(eye = base + [1.7, 3.4, 1.9])
#   **뒤쪽**이라야 화면에 안 나온다. (4.0, -6.0) 에 두면 시선 방향이라 박스 15개가
#   배경에 줄지어 떠 있는 것처럼 찍힌다 — 영상용으로 못 쓴다.
STAGE_XY = (14.5, 4.2)
STAGE_PITCH = 0.75
STAGE_COLS = 5

# ★ 버림(despawn) 자리 — `Spawner._despawn` 이 하는 일을 이 데모 안에서 한다.
#   이 데모는 `sim_main` 소켓을 안 거치고 프림을 직접 들고 있으므로 서비스 호출 대신
#   프림을 라인 밖으로 옮긴다. 벨트(x 9.65 ±0.6)·팔레트(10.3,0.3)·로봇(10.9,-0.1)
#   어디에도 안 걸리고, 대기 스테이징(+Y 4.2)과도 반대쪽이라 겹치지 않는다.
PARK_XY = (14.5, -3.0)


PHYS_MAT_PATH = "/World/Looks/PhysicsMaterial"          # spawner.py 와 같은 경로


def _phys_material(stage, cfg):
    """`system.yaml` 의 마찰을 담은 물리 머티리얼. 없으면 만든다 (spawner.py:370 과 동일).

    ★ 이게 없으면 박스가 PhysX **기본 마찰**로 돌고 `physics.material` 0.6/0.5 는
      아무데도 안 걸린다. 스택이 서느냐 미끄러지느냐를 직접 가르는 값이다.
    """
    from pxr import UsdGeom, UsdPhysics, UsdShade
    if not stage.GetPrimAtPath("/World/Looks"):
        UsdGeom.Scope.Define(stage, "/World/Looks")
    prim = stage.GetPrimAtPath(PHYS_MAT_PATH)
    if not prim:
        m = cfg["physics"]["material"]
        pm = UsdShade.Material.Define(stage, PHYS_MAT_PATH)
        api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
        api.CreateStaticFrictionAttr(float(m["static_friction"]))
        api.CreateDynamicFrictionAttr(float(m["dynamic_friction"]))
        api.CreateRestitutionAttr(0.0)
        prim = pm.GetPrim()
    return prim


def _apply_physics(stage, prim, cfg, rigid=True):
    """`spawner.py` 가 박스마다 거는 물리 속성을 그대로 건다.

    ★ 왜 필요한가 — 이 데모는 스포너를 안 거치고 `verify_suction.make_box` 로 씬을
      직접 만든다. 그 함수는 콜라이더·리지드바디·질량만 걸고 **솔버 반복수·오프셋·
      물리 머티리얼을 안 건다.** 그래서 데모 박스만 PhysX 기본값으로 돌았다:
      solverPositionIterationCount 기본(4) vs 규정 24. 놓고 난 박스가 접촉을 다 풀지
      못해 여러 스텝에 걸쳐 **조금씩 미끄러진다** (CLAUDE.md §9 "쌓은 박스가 저절로
      무너진다"). 2026-08-25 실측: #14 에서 BOX00012 가 70.7 mm 흘렀는데 팔은 그
      근처에 없었다.

    ★ 솔버 반복수는 **박스마다** 건다. PhysicsScene 에 걸면 무시된다 (spawner.py:30).
    """
    from pxr import PhysxSchema, UsdShade
    ph = cfg["physics"]
    col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    col.CreateRestOffsetAttr(ph["collider"]["rest_offset_mm"] / 1000.0)
    col.CreateContactOffsetAttr(ph["collider"]["contact_offset_mm"] / 1000.0)
    if rigid:
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(
            int(ph["rigid_body"]["solver_position_iterations"]))
        rb.CreateSolverVelocityIterationCountAttr(
            int(ph["rigid_body"]["solver_velocity_iterations"]))
    mat = _phys_material(stage, cfg)
    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(
        UsdShade.Material(mat), UsdShade.Tokens.weakerThanDescendants, "physics")


def _link_dists(rm, lo, hi):
    """지금 자세에서 **링크별로** 피해 박스 AABB 까지의 최소거리 [mm]. 가까운 순.

    `robot_motion._arm_clearance` 와 같은 계산인데, 최악값 하나가 아니라 **어느 링크인지**
    를 남긴다. "박스가 혼자 밀렸다" 와 "팔이 쳤다" 를 가르는 게 이 목록이다.
    """
    q = np.asarray(rm.art.get_joint_positions()[:6], dtype=float)
    lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    out = []
    for link, sph in (rm.spheres or {}).items():
        if link == "base_link":
            continue
        try:
            pos, rot = rm.lula.compute_forward_kinematics(link, q)
        except Exception:
            continue
        pos, rot = np.asarray(pos), np.asarray(rot)
        best = float("inf")
        for c, r in sph:
            w = pos + rot @ np.asarray(c, dtype=float)
            over = np.maximum(lo - w, w - hi)
            d = (float(np.linalg.norm(np.maximum(over, 0.0)))
                 if np.any(over > 0) else float(np.max(over))) - r
            best = min(best, d)
        if best < float("inf"):
            out.append((best * 1000.0, link))
    out.sort()
    return out


def _park(rp, dm, idx):
    """거부·실패한 박스를 라인 밖으로 옮긴다. `Spawner._despawn` 과 같은 역할이다.

    속도를 0 으로 지우지 않으면 옮긴 자리에서 튄다 (급송 코드와 같은 이유).
    """
    px = PARK_XY[0] + (idx % STAGE_COLS) * STAGE_PITCH
    py = PARK_XY[1] - (idx // STAGE_COLS) * STAGE_PITCH
    rp.set_world_pose(position=np.array([px, py, dm[2] / 2.0 + 0.002]),
                      orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    rp.set_linear_velocity(np.zeros(3))
    rp.set_angular_velocity(np.zeros(3))


# ─────────────────────────────────────────────────────────────
# 배치 계획 — Isaac 없이 돈다
# ─────────────────────────────────────────────────────────────
def _load_packer_module():
    """`src/packing_sim/packing.py` 를 sys.path 오염 없이 읽는다.

    `src/packing_sim` 을 통째로 sys.path 에 넣으면 거기 `config.py` 가 있어서
    다른 `config` 를 가린다.
    """
    import importlib.util
    if "_packing_hm" in sys.modules:
        return sys.modules["_packing_hm"]
    path = _paths.ROOT / "src/packing_sim/packing.py"
    spec = importlib.util.spec_from_file_location("_packing_hm", path)
    mod = importlib.util.module_from_spec(spec)
    # ★ exec_module 전에 등록해야 한다. @dataclass 가 cls.__module__ 로 sys.modules 를
    #   되짚어 보기 때문에, 등록 안 하면 AttributeError 로 죽는다.
    sys.modules["_packing_hm"] = mod
    spec.loader.exec_module(mod)
    return mod


def manifest(cfg, n):
    """`boxes.random_seed` + `size_distribution` 으로 박스 n 개를 뽑는다.

    ⚠ `spawn_feeder.py` 는 `random.Random(seed).choices`, `mixed_plan` 은
      `np.random.default_rng(seed).choice` 를 쓴다. 같은 seed 라도 결과가 다르다.
      여기서는 패커·mixed_plan 과 같은 numpy 쪽을 쓴다.
    """
    types = cfg["boxes"]["types"]
    rng = np.random.default_rng(cfg["boxes"]["random_seed"])
    p = np.array(cfg["boxes"]["size_distribution"], dtype=float)
    p = p / p.sum()
    return [types[int(i)] for i in rng.choice(len(types), int(n), p=p)]


def build_plan(cfg, n, clearance_mm=CLEARANCE_MM, halo_mm=DESCENT_HALO_MM, order="volume",
               ped_mm=None):
    """`HeightmapPacker` 로 n 개짜리 다층 계획을 만든다.

    반환은 `check_place_plan.py` / `cycle.py` 가 그대로 먹는 형식이다 —
    팔레트 로컬 mm, x·y 는 박스 **중심**, z 는 박스 **밑면**.
    같이 돌려주는 `boxes` 는 **급송 순서**로 정렬된 박스 스펙이다 (계획과 1:1).
    """
    PK = _load_packer_module()
    hm, pal = cfg["heightmap"], cfg["pallet"]
    pk = PK.HeightmapPacker(
        size_xy=(pal["width_mm"] / 1000.0, pal["depth_mm"] / 1000.0),
        max_height=pal["max_stack_mm"] / 1000.0,
        cell=hm["cell_mm"] / 1000.0,
        yaws_deg=tuple(float(v) for v in hm["allowed_yaw_deg"]),
        min_support_ratio=float(hm["min_support_ratio"]),
        support_tol=hm["support_tol_mm"] / 1000.0,
        wall_margin=0.0,
        enforce_load_order=True,                 # 9 kg 5호가 3 kg 3호를 깔고 앉지 않게
        descent_halo=halo_mm / 1000.0,
    )
    boxes = manifest(cfg, n)
    if order == "volume":
        boxes.sort(key=lambda b: -(b["w_mm"] * b["d_mm"] * b["h_mm"]))
    c = clearance_mm / 1000.0

    plan, fed, dropped = [], [], []
    for b in boxes:
        pl = pk.place((b["w_mm"] / 1000.0 + c, b["d_mm"] / 1000.0 + c, b["h_mm"] / 1000.0),
                      b["mass_g"] / 1000.0)
        if pl is None:
            dropped.append(b["name"])
            continue
        cx, cy, _ = pl.center_local
        plan.append({"box_type": b["name"],
                     "x_mm": int(round(cx * 1000.0)),
                     "y_mm": int(round(cy * 1000.0)),
                     "z_mm": int(round(pl.z * 1000.0)),
                     "yaw_deg": int(round(pl.yaw_deg)),
                     "support": round(float(pl.support_ratio), 3)})
        fed.append(b)

    # ★ 실행 순서 — **팔이 기둥과 만나지 않게** 끼워넣는다. 좌표는 하나도 안 바꾼다.
    #
    #   제약이 양방향이라 (2026-08-22 책상 스윕 + Isaac 실측, results/issues/18):
    #     (a) 근거리(로봇 쪽) 기둥이 서 있으면 그 너머 far 배치 때 **link_2** 가 쓸어버린다
    #         → 실측: 근거리 890 에서 26~80 mm 밀림. far 를 먼저 놓아야 한다.
    #     (b) far 기둥이 로봇 베이스 근처보다 높으면(top > ped−50) 그 앞 근거리에 깊이
    #         내려앉을 때 **팔꿈치(link_3/4)** 가 목표 너머로 튀어나가 그 기둥을 민다
    #         → 실측: far 오른쪽 980 이 선 채 (765,205) 배치 시 −4~−65 mm.
    #   단순 far→near 도, near→far 도 안 된다. **높은 far 기둥만 뒤로 미룬다**:
    #
    #     0. far 하단 (팔꿈치 부채꼴 밖이거나 낮은 것)      — 근거리가 없어 (a) 자유
    #     1. 근거리 5호 기둥                                  — 높은 far 가 아직 없어 (b) 자유
    #     2. 미뤄둔 far 꼭대기 (z 오름차순)                  — 근거리 680 은 link_2 가 넘는다
    #                                                          (책상 +180 @ ped 900)
    #     3. 근거리 3호 줄                                    — 이후 far 배치가 없어 (a) 무관
    #
    #   미루는 판정은 **박스 단위**다: 그 박스가 만들 상단(z+h) > ped − 150 이고
    #   방위각이 근거리 슬롯 ±25° 안이면 phase 2 로 미룬다.
    #   ⚠ 두 번 밟았다 —
    #     · 문턱 ped−50: far 왼쪽 840 이 바로 밑에 남아 근거리 z340 배치 때 62 mm 밀림.
    #       실제 Lula 해는 책상 최적화 해와 팔꿈치 가지가 달라 책상 +70 도 실기에선 닿는다.
    #     · 기둥 단위 판정: 기둥 하단(z0)까지 통째로 미뤄져, 이번엔 그 하단 배치가
    #       근거리 680 위를 지나며 79 mm 오차를 냈다. 하단은 남기고 꼭대기만 미뤄야 한다.
    #   책상 검증 (ped 900): 국면1 (765,205) −4 → **+70**, 국면2 far 꼭대기 −9 → z순서로 회피.
    #   ⚠ ped 700 에서는 어떤 순서로도 안 나온다 — (b)가 560 짜리 기둥에서도 −65 다.
    #     이 계획은 **--pedestal 900 이상**과 함께 쓸 것.
    import math as _m
    DIMS = {t["name"]: (t["w_mm"], t["d_mm"], t["h_mm"]) for t in cfg["boxes"]["types"]}
    ped = float(ped_mm if ped_mm is not None else cfg["robot"]["pedestal_height_mm"])
    base_xy = cfg["zones"]["robot_base_world_mm"]["B"][:2]
    org = cfg["zones"]["pallet_origin_world_mm"]["B"]

    def _az(q):
        return _m.degrees(_m.atan2(q["y_mm"] + org[1] - base_xy[1],
                                   q["x_mm"] + org[0] - base_xy[0]))

    def _wh(q):
        w, d = DIMS[q["box_type"]][:2]
        return (d, w) if q["yaw_deg"] == 90 else (w, d)

    near = [q for q in plan if q["y_mm"] < 400]
    big_near_az = [_az(q) for q in near if DIMS[q["box_type"]][2] >= 300]   # 깊이 내려앉는 5호
    def _phase(q):
        if q["y_mm"] >= 400:                           # far
            top = q["z_mm"] + DIMS[q["box_type"]][2]   # 이 박스가 만들 상단
            in_sector = any(abs((_az(q) - az + 180) % 360 - 180) < 25 for az in big_near_az)
            return 2 if (top > ped - 150 and in_sector) else 0
        return 1 if DIMS[q["box_type"]][2] >= 300 else 3

    def _overlaps(u, v):
        uw, ud = _wh(u)
        vw, vd = _wh(v)
        return (abs(u["x_mm"] - v["x_mm"]) < (uw + vw) / 2 - 1 and
                abs(u["y_mm"] - v["y_mm"]) < (ud + vd) / 2 - 1)

    sup = {i: {j for j, q in enumerate(plan)
               if j != i and abs(q["z_mm"] + DIMS[q["box_type"]][2] - b_["z_mm"]) < 1
               and _overlaps(b_, q)}
           for i, b_ in enumerate(plan)}
    done, idx = set(), []
    while len(idx) < len(plan):                        # 지지 위상을 지키며 phase 순으로
        ready = [i for i in range(len(plan)) if i not in done and sup[i] <= done]
        # 같은 phase·z 에서는 **x 큰(로봇 기준 오른쪽·짧은 리치) 것 먼저**.
        #   5호 쌍의 두 번째는 첫 번째 옆 30 mm 틈에 꽂히는데, (765,205) 를 두 번째로
        #   놓으면 매 실행 22~30 mm 미끄러지고 이웃을 최대 76 mm 밀었다 (비결정적).
        #   오른쪽을 먼저 놓으면 두 번째가 되는 왼쪽 (255,205) 은 리치가 길어 자세가
        #   달라진다 — 실측으로 확인한다.
        i = min(ready, key=lambda k: (_phase(plan[k]), plan[k]["z_mm"],
                                      -plan[k]["y_mm"], -plan[k]["x_mm"], k))
        idx.append(i)
        done.add(i)
    plan = [plan[i] for i in idx]
    fed = [fed[i] for i in idx]

    # 각 배치 시점의 **이미 쌓인 것의 최상단** (팔레트 상면 기준 mm).
    #   먼 쪽을 먼저 쌓으면 z 가 단조가 아니게 되므로 `cycle.py` 이송 높이에 이게 필요하다
    #   (issues/17). 선택 필드라 이 값이 없으면 cycle.py 는 예전과 똑같이 동작한다.
    top = 0
    for q in plan:
        q["stack_top_mm"] = int(top)
        top = max(top, q["z_mm"] + DIMS[q["box_type"]][2])

    return plan, fed, dropped, pk


DIMS_G = {}          # print_plan 용 박스 높이 표 (main 에서 채운다)


def print_plan(plan, pk=None, dropped=(), ped_mm=700.0):
    zs = sorted({p["z_mm"] for p in plan})
    print("  #  종류  yaw    중심 mm      밑면 z  지지")
    for i, p in enumerate(plan, 1):
        print("%3d  %-4s %3d   (%4d,%4d)   %5d   %.2f"
              % (i, p["box_type"], p["yaw_deg"], p["x_mm"], p["y_mm"],
                 p["z_mm"], p.get("support", float("nan"))))
    print("\n  층(밑면 z) %s  ->  %d 층" % (zs, len(zs)))
    # ★ 놓으러 갈 때 **로봇 쪽에 이미 서 있는 기둥**을 넘어가야 하는지 본다.
    #   넘어가면 아래쪽 축(link_2)이 그 위를 쓸고 지나간다 (results/issues/18).
    worst, seen = 0, []
    for q in plan:
        near = [r["z_mm"] + DIMS_G[r["box_type"]] for r in seen if r["y_mm"] < q["y_mm"] - 50]
        worst = max(worst, max(near) if near else 0)
        seen.append(q)
    # link_2 가 넘을 수 있는 한계 ≈ ped − 150 (책상 스윕: ped 900 에서 680 은 +180, 890 은 −9)
    lim = ped_mm - 150.0
    print("  넘어가야 하는 근거리 기둥 최고 %d mm (한계 ~%.0f @ ped %.0f)%s"
          % (worst, lim, ped_mm,
             "  ✓" if worst <= lim else "   !! link_2 가 그 위를 쓴다 (issues/18)"))
    if pk is not None:
        print("  최상단 %.0f mm / max_stack %.0f mm   부피활용 %.1f%%"
              % (pk.current_height * 1000.0, pk.max_height * 1000.0,
                 pk.volume_utilization() * 100.0))
    if dropped:
        print("  !! 계획에 못 넣은 박스 %s" % dropped)


# ─────────────────────────────────────────────────────────────
def _load_algo_module():
    """트랙 C 코어(`src/packing_algo/packing_algo/heightmap_core.py`)를 **파일로** 읽는다.

    ★ `import packing_algo...` 를 쓰지 않는 이유 두 가지.
      1. 그 패키지의 `algo_service_node` 가 `packing_msgs` 를 끌어오는데, Isaac 내장
         파이썬은 3.11 이고 ROS 2 Jazzy 는 3.12 라 import 가 죽는다 (CLAUDE.md §2).
      2. `install/` 이 source 돼 있으면 소스본과 설치본 중 어느 사본이 잡히는지가
         셸 상태에 달린다. 검증에 쓰는 계산기가 실행마다 달라지면 안 된다.
    `heightmap_core` 의 의존은 math·numpy·scipy 뿐이라 3.11 에서 그대로 돈다.
    """
    import importlib.util
    if "_algo_hm" in sys.modules:
        return sys.modules["_algo_hm"]
    path = _paths.ROOT / "src/packing_algo/packing_algo/heightmap_core.py"
    spec = importlib.util.spec_from_file_location("_algo_hm", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["_algo_hm"] = mod
    spec.loader.exec_module(mod)
    return mod


class PalletVision:
    """손목 카메라 대역 — 팔레트 위 박스들의 **실제** 자세를 돌려준다.

    실물에서는 로봇팔에 달린 카메라 + 인식이 이 자리를 맡는다. `HeightmapPacker`
    (heightmap-greedy) 는 애초에 **매번 실제 적재 상태를 다시 보는 것**을 전제로 만들어졌다 —
    `packing.py:272 undo_last()` 주석이 그렇게 적어 뒀다:
    "로봇이 박스를 놓치거나 엉뚱한 곳에 놓으면 하이트맵이 실제와 달라진다.
     그 상태로 다음 박스를 계산하면 있지도 않은 지지면 위에 쌓게 된다."

    여기서는 물리 자세를 그대로 읽으므로 **오차 없는 비전**을 가정한 상한이다.
    인식 오차·가림·놓친 박스를 넣고 싶으면 `scan()` 안에 넣으면 된다.
    """

    def __init__(self, by_id, specs, origin_mm):
        self.by_id, self.specs, self.origin = by_id, specs, list(origin_mm)
        self.placed = []                                # 팔레트에 올라간 box_id (순서)

    def saw_placed(self, box_id):
        if box_id not in self.placed:
            self.placed.append(box_id)

    def scan(self):
        """카메라 한 장. 팔레트 로컬 [m] 로 (중심, 치수, yaw, 질량)."""
        out = []
        for bid in self.placed:
            pos, quat = self.by_id[bid].get_world_pose()
            s = self.specs[bid]
            out.append({
                "cx": float(pos[0]) - self.origin[0] / 1000.0,
                "cy": float(pos[1]) - self.origin[1] / 1000.0,
                "cz": float(pos[2]) - self.origin[2] / 1000.0,
                "w": s["w_mm"] / 1000.0, "d": s["d_mm"] / 1000.0, "h": s["h_mm"] / 1000.0,
                "yaw": _yaw_of(quat), "mass": s["mass_g"] / 1000.0,
            })
        return out


def _obs_to_placement(PK, pk, o, clearance_m):
    """관측 하나를 `Packing.Placement` 로. 팔레트 밖으로 나간 것은 버린다."""
    w, d = o["w"], o["d"]
    if int(round(abs(o["yaw"]) / 90.0)) % 2 == 1:       # yaw 90/270 이면 축이 바뀐다
        w, d = d, w
    w += clearance_m
    d += clearance_m
    x, y = o["cx"] - w / 2.0, o["cy"] - d / 2.0
    z = o["cz"] - o["h"] / 2.0
    # ★ commit() 은 x/cell 을 그대로 인덱스로 쓴다. 음수면 numpy 가 **뒤에서부터** 잘라
    #   팔레트 반대편에 유령 기둥이 선다. 반드시 클램프한다.
    x, y = max(0.0, x), max(0.0, y)
    if x >= pk.size_xy[0] or y >= pk.size_xy[1] or z < -0.05 or z > pk.max_height:
        return None                                     # 팔레트 밖으로 떨어진 박스
    w = min(w, float(pk.size_xy[0]) - x)
    d = min(d, float(pk.size_xy[1]) - y)
    return PK.Placement(x=x, y=y, z=max(0.0, z), w=w, d=d, h=o["h"],
                        yaw_deg=0.0, support_ratio=1.0, flatness=0.0, mass=o["mass"])


class OnlineServices:
    """비전 → 하이트맵 → 다음 자리. **박스마다 다시 계산한다.**

    오프라인 계획(`--plan-mode offline`)과 다른 점은 하나뿐이다 — `plan_placement` 이
    미리 만든 표를 읽는 대신 **그 순간 카메라가 본 적재 상태**로 하이트맵을 다시 세우고
    거기서 자리를 찾는다. 박스가 밀렸으면 밀린 채로 반영된다.

    `stack_top_mm` 을 같이 실어 보낸다 — `cycle.py` 가 이송 높이에 쓴다. 온라인에서는
    z 가 단조라는 보장이 없으므로(HeightGreedy 는 낮은 자리를 먼저 채운다) 이게 필요하다.
    """

    def __init__(self, specs, factory, vision, clearance_mm, log=print, verbose=False):
        self.specs, self.factory, self.vision = dict(specs), factory, vision
        self.c = clearance_mm / 1000.0
        self.calls, self._log, self._v = [], log, verbose
        self.last = None                                 # 마지막 계획 (요약표용)
        self.last_plan_msg = None                        # 직전 plan_placement 의 msg (§5 판정용)

    def _rec(self, name, *a):
        self.calls.append((name,) + a)
        if self._v:
            self._log("             [svc] %s%s" % (name, a if a else ()))

    def set_velocity(self, seg, mps):
        self._rec("/conveyor/set_velocity", seg, mps)
        return True

    def get_box_info(self, box_id):
        self._rec("/db/get_box_info", box_id)
        s = self.specs.get(box_id)
        return None if s is None else {"w_mm": s["w_mm"], "d_mm": s["d_mm"],
                                       "h_mm": s["h_mm"], "zone": "B"}

    def plan_placement(self, zone, box_id, w, d, h):
        self._rec("/algo/plan_placement", zone, box_id)
        PK = _load_packer_module()
        pk = self.factory()
        top = 0.0
        seen, lost = 0, 0
        for o in self.vision.scan():
            pl = _obs_to_placement(PK, pk, o, self.c)
            if pl is None:
                lost += 1
                continue
            pk.commit(pl)
            top = max(top, pl.z + pl.h)
            seen += 1
        mass = self.specs[box_id]["mass_g"] / 1000.0
        p = pk.find_placement((w / 1000.0 + self.c, d / 1000.0 + self.c, h / 1000.0), mass)
        if p is None:
            self._log("             [svc] 자리 없음 (관측 %d개, 최상단 %.0f mm)"
                      % (seen, top * 1000))
            self.last_plan_msg = "PALLET_FULL"
            return False, None, "PALLET_FULL"
        self.last_plan_msg = "OK"
        cx, cy, _ = p.center_local
        place = {"x_mm": int(round(cx * 1000.0)), "y_mm": int(round(cy * 1000.0)),
                 "z_mm": int(round(p.z * 1000.0)), "yaw_deg": int(round(p.yaw_deg)),
                 "stack_top_mm": int(round(top * 1000.0))}
        self.last = dict(place, support=round(float(p.support_ratio), 3))
        self._log("             [svc] 비전 %d개%s -> P(%d,%d) z%d yaw%d  지지 %.2f  최상단 %d"
                  % (seen, "" if not lost else " (팔레트 밖 %d)" % lost,
                     place["x_mm"], place["y_mm"], place["z_mm"], place["yaw_deg"],
                     p.support_ratio, place["stack_top_mm"]))
        return True, place, "OK"

    def set_status(self, box_id, status):
        self._rec("/db/set_status", box_id, status)
        return True

    def mark_placed(self, box_id, pose):
        self._rec("/db/mark_placed", box_id)
        self.vision.saw_placed(box_id)                   # 다음 계산부터 카메라에 잡힌다
        return True

    def update_heightmap(self, zone, box_id, w, d, h, actual):
        # 온라인에서는 하이트맵을 **매번 카메라로 다시 세우므로** 누적할 게 없다.
        self._rec("/algo/update_heightmap", box_id)
        return True, 0.0

    def cancel_plan(self, zone, box_id):
        self._rec("/algo/cancel_plan", box_id)
        return True


# ─────────────────────────────────────────────────────────────
class AlgoServices:
    """**트랙 C(정정인) 코어를 그대로 꽂는다.** 데모의 사본 패커 대신 진짜를 쓴다.

    ★ 왜 따로 있나 — `demo_stack15` 는 지금까지 `packing_sim/packing.py` 의 **사본**
      패커를 썼다. 통합 검증에서 확인해야 할 것은 그 사본이 아니라 `packing_algo` 다.

    벨트·DB 4개(`set_velocity`·`get_box_info`·`set_status`·`mark_placed`)는 algo 의
    책임이 아니므로(CLAUDE.md §3) `PlanServices` 와 같은 스텁을 쓴다. algo 가 답하는
    것은 `plan_placement`·`update_heightmap`·`cancel_plan` 셋뿐이다.
    """

    def __init__(self, specs, cfg, clearance_mm=0.0, log=print, verbose=False):
        PKA = _load_algo_module()
        self.pk = PKA.HeightmapPacker(cfg)          # ★ cfg 통째로 받는다 (사본과 시그니처가 다르다)
        self.specs = dict(specs)
        self.c = int(clearance_mm)
        self.calls, self._log, self._v = [], log, verbose
        self.last = None                            # 온라인 경로가 실측 대조에 쓴다
        self.last_plan_msg = None                   # 직전 plan_placement 의 msg (§5 판정용)

    def _rec(self, name, *a):
        self.calls.append((name,) + a)
        if self._v:
            self._log("             [svc] %s%s" % (name, a if a else ""))

    # ── algo 가 답하는 것 ────────────────────────────────────
    def plan_placement(self, zone, box_id, w, d, h):
        self._rec("/algo/plan_placement", zone, box_id)
        # ★ 맨 앞에서 지운다. 안 지우면 계획에 실패한 박스가 **직전 박스의 좌표**를
        #   물려받아 대조표에 찍힌다 (아래 online 경로가 `svc.last` 를 그대로 읽는다).
        self.last = None
        self.last_plan_msg = None
        ok, pose, _score, msg = self.pk.plan(box_id, int(w) + self.c,
                                             int(d) + self.c, int(h))
        self.last_plan_msg = msg
        if not ok:
            self._log("             [algo] %s" % msg)
            return False, None, msg
        # ★ np.int64 로 나온다. 지금 경로는 산술뿐이라 안 터지지만 json 을 태우면 죽는다.
        place = {"x_mm": int(pose["x_mm"]), "y_mm": int(pose["y_mm"]),
                 "z_mm": int(pose["z_mm"]), "yaw_deg": float(pose["yaw_deg"]),
                 "stack_top_mm": int(self.pk.hm.max())}
        self.last = dict(place)
        return True, place, "OK"

    def update_heightmap(self, zone, box_id, w, d, h, actual):
        self._rec("/algo/update_heightmap", box_id)
        # ★ clearance 를 plan 과 **같이** 더해야 한다. 한쪽만 부풀리면
        #   heightmap_core 의 `i = round((actual.x_mm - w/2)/cell)` 이 셀 단위로 어긋나
        #   발자국이 실제와 다른 자리에 찍힌다.
        ok, fill, _n = self.pk.update(box_id, int(w) + self.c, int(d) + self.c,
                                      int(h), actual)
        return ok, fill                              # cycle.py 는 2개만 받는다

    def cancel_plan(self, zone, box_id):
        self._rec("/algo/cancel_plan", box_id)
        self.pk.cancel(box_id)
        return True

    # ── algo 책임이 아닌 것 (PlanServices 와 동일한 스텁) ────
    def set_velocity(self, seg, mps):
        self._rec("/conveyor/set_velocity", seg, mps)
        return True

    def get_box_info(self, box_id):
        self._rec("/db/get_box_info", box_id)
        s = self.specs.get(box_id)
        if s is None:
            return None
        return {"w_mm": s["w_mm"], "d_mm": s["d_mm"], "h_mm": s["h_mm"], "zone": "B"}

    def set_status(self, box_id, status):
        self._rec("/db/set_status", box_id, status)
        return True

    def mark_placed(self, box_id, pose):
        self._rec("/db/mark_placed", box_id)
        return True


class PlanServices:
    """§2.7 Mock 을 **계획으로 구동**한다. `verify_b4_cycle.MockServices` 의 15개판.

    `MockServices` 는 항상 4호 · 항상 (205,155,0) 를 돌려주므로 15개가 한 자리에 겹친다.
    여기서는 박스 id 로 스펙과 배치를 각각 찾아 준다.
    """

    def __init__(self, specs, places, log=print, verbose=False):
        self.specs = dict(specs)                    # box_id -> system.yaml 박스 스펙
        self.places = dict(places)                  # box_id -> PlacePose
        self.last_plan_msg = None                   # 직전 plan_placement 의 msg (§5 판정용)
        self.calls = []
        self._log = log
        self._v = verbose

    def _rec(self, name, *a):
        self.calls.append((name,) + a)
        if self._v:
            self._log("             [svc] %s%s" % (name, a if a else ""))

    def set_velocity(self, seg, mps):
        self._rec("/conveyor/set_velocity", seg, mps)
        return True

    def get_box_info(self, box_id):
        self._rec("/db/get_box_info", box_id)
        s = self.specs.get(box_id)
        if s is None:
            return None
        return {"w_mm": s["w_mm"], "d_mm": s["d_mm"], "h_mm": s["h_mm"], "zone": "B"}

    def plan_placement(self, zone, box_id, w, d, h):
        self._rec("/algo/plan_placement", zone, box_id)
        p = self.places.get(box_id)
        if p is None:
            self.last_plan_msg = "PALLET_FULL"
            return False, None, "PALLET_FULL"
        self.last_plan_msg = "OK"
        return True, dict(p), "OK"

    def set_status(self, box_id, status):
        self._rec("/db/set_status", box_id, status)
        return True

    def mark_placed(self, box_id, pose):
        self._rec("/db/mark_placed", box_id)
        return True

    def update_heightmap(self, zone, box_id, w, d, h, actual):
        self._rec("/algo/update_heightmap", box_id)
        return True, 0.0

    def cancel_plan(self, zone, box_id):
        self._rec("/algo/cancel_plan", box_id)
        return True


# ─────────────────────────────────────────────────────────────
def _j5(S, acc):
    """joint_5 절대각을 누적한다 [deg].

    `verify_cycle.py:556 unwrap_wrist()` 는 박스를 놓은 뒤 j5 를 등가각으로 되감는데,
    **그 코드가 `robot_motion.py`/`cycle.py` 로는 이식되지 않았다** (grep 확인).
    B4 검증은 박스 1개라 누적을 볼 수 없었다. 15회를 돌리면 여기서 드러난다 —
    한계는 URDF ±360°, 90% 를 넘으면 손목이 지령을 못 따라가 박스가 뒤집힌다
    (results/issues/11 실측 178.5°).
    """
    v = math.degrees(float(S.art.get_joint_positions()[4]))
    acc[0] = min(acc[0], v)
    acc[1] = max(acc[1], v)


def _yaw_of(quat):
    w, x, y, z = [float(v) for v in quat]
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--boxes", type=int, default=None,
                    help="기본은 system.yaml boxes.stage1.total_count (=15)")
    ap.add_argument("--plan-mode", default="offline",
                    choices=["offline", "online", "algo"],
                    help="offline = 15개 계획을 미리 만들어 z 오름차순으로 실행한다 (기본, 검증됨). "
                         "online = 박스마다 비전으로 적재 상태를 다시 보고 자리를 정한다 "
                         "— 카메라가 붙었을 때의 구조다. **지금은 오프라인보다 나쁘다**, "
                         "results/issues/19 참조")
    ap.add_argument("--plan", default=None, help="계획 JSON 을 직접 넣는다 (offline 강제)")
    ap.add_argument("--dump-plan", default=None, help="쓴 계획을 JSON 으로 저장")
    ap.add_argument("--plan-only", action="store_true",
                    help="계획만 만들고 끝낸다. Isaac 을 안 띄운다 (python3 로 돈다)")
    ap.add_argument("--clearance", type=float, default=CLEARANCE_MM, metavar="MM")
    ap.add_argument("--halo", type=float, default=DESCENT_HALO_MM, metavar="MM")
    ap.add_argument("--order", default="volume",
                    choices=["volume", "small-first", "arrival", "shuffle"],
                    help="급송 순서. volume=큰 것부터(기본) · small-first=작은 것부터 · "
                         "arrival=도착순 그대로(일부 탈락한다) · shuffle=무작위(seed 고정). "
                         "★ 트랙 C 코어에는 **하중 순서 로직이 없다** — 큰 것부터 넣는 것이 "
                         "무거운 박스를 아래에 두는 유일한 장치다. 바꾸면 9 kg 5호가 "
                         "3 kg 3호 위에 올라갈 수 있다")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--diag-tcp", action="store_true",
                    help="사이클마다 **하강 바닥에서 지령 대비 실제 흡착면 위치**를 찍는다. "
                         "박스 착지가 어긋날 때 팔이 못 간 것인지(팔) 박스가 흘러간 것인지"
                         "(흡착)를 가른다 — 두 값이 같은데 박스만 어긋나면 흡착 쪽이다")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--hold", action="store_true", help="--gui 와 같이. 끝나고 창을 열어 둔다")
    ap.add_argument("--cam", type=float, nargs=6, default=None,
                    metavar=("EX", "EY", "EZ", "TX", "TY", "TZ"),
                    help="카메라 eye/target [m]. 기본은 팔레트를 앞에 둔 적재 시점")
    ap.add_argument("--pedestal", type=float, default=None, metavar="MM",
                    help="페데스탈(로봇 베이스) 높이를 덮어쓴다. system.yaml 은 안 건드린다. "
                         "★ 700 이면 적재 상단(월드 1200)이 어깨보다 500 mm 높아 link_2 가 "
                         "이송 중 적재물을 스친다 (results/issues/18). 1000 이면 여유 +49 mm")
    ap.add_argument("--vel", type=float, default=None, metavar="SCALE",
                    help="관절 속도 배율(joint_speed_scale)을 덮어쓴다. system.yaml 은 "
                         "안 건드린다. 기본 0.5. 시연을 빨리 볼 때 0.8~1.0. "
                         "⚠ 올리면 릴리스 시점 잔여속도가 커져 박스가 놓인 뒤 흐를 수 "
                         "있다 — 착지 정밀도를 볼 때는 기본값으로 돌릴 것")
    ap.add_argument("--algo-clearance", type=float, default=None, metavar="MM",
                    help="계획 간격을 **덮어쓴다** (기본은 system.yaml 의 plan_clearance_mm=10). "
                         "발자국을 이만큼 부풀려 박스 사이에 실제 틈을 남긴다. "
                         "0 은 슬롯이 박스와 같은 크기라 물리적으로 성립하지 않는다 — "
                         "실측에서 9kg 5호가 338mm 위에서 막히고 이웃을 1046mm 밀어냈다")
    ap.add_argument("--standoff", type=float, default=None, metavar="MM",
                    help="로봇을 팔레트 긴 변에서 뒤로 뺀 거리. 기본은 구 사양서 §3-1 의 300. "
                         "600 이면 여유 +126 mm. ※ 둘 다 G1 재검증이 필요하다")
    ap.add_argument("--kit-log", action="store_true",
                    help="Kit 의 경고/에러 로그를 터미널에 그대로 둔다. 기본은 숨긴다 — "
                         "영상에 터미널이 같이 찍히기 때문. 스레드 경합(PhysX 홍수)은 "
                         "robot_motion 캐시로 근본을 고쳤고, 이 플래그는 기동 시 에셋 "
                         "경고(Unresolved reference 등)까지 마저 감추는 용도다")
    ap.add_argument("--diag-release", action="store_true",
                    help="`release` 를 부른 뒤 박스가 **언제 멈추는지** 100ms 구간으로 잰다. "
                         "RELEASE_MIN_HOLD_S · release_damping_duration_s · settle_wait_s "
                         "세 개(각 0.5s)가 다 필요한지 여기서만 갈린다")
    ap.add_argument("--poll-ms", type=float, default=None, metavar="MS",
                    help="`cycle.MOTION_POLL_S` 를 덮어쓴다 (기본 20ms). 헤드리스는 시뮬이 "
                         "실시간의 7배로 돌아 20ms 벽시계가 **138ms 시뮬**이 된다 — "
                         "사이클 시간의 3분의 1이 이 인공물이다. 실제 로봇은 실시간이라 "
                         "20ms 가 그냥 20ms 다")
    ap.add_argument("--transit-tol", type=float, default=None, metavar="MM",
                    help="이송 구간(상공 이동·후퇴) 허용 오차. 기본은 하강과 같은 2mm 라 "
                         "예전 동작 그대로다. 올리면 정착 대기가 줄지만 다음 호가 "
                         "그만큼 낮게 시작한다 — 이격 400 여유가 +32.9mm 다")
    ap.add_argument("--diag-time", action="store_true",
                    help="사이클 시간이 **어느 소켓 명령 밑에** 쌓이는지 스텝 수로 적는다. "
                         "목표 10.0 s 대비 어디를 깎을지 정하는 근거")
    ap.add_argument("--render-every", type=int, default=1, metavar="N",
                    help="N 스텝에 한 번만 화면을 그린다. **물리는 한 스텝도 안 바뀐다** — "
                         "결과가 비트 단위로 같다. 2~4 면 벽시계가 짧아지고 화면이 그만큼 "
                         "빨라 보인다. 궤적을 바꾸는 --vel 과 다르다")
    ap.add_argument("--hm", action="append", metavar="KEY=VAL",
                    help="heightmap 가중치를 **계획용 사본에만** 덮어쓴다. 여러 번 쓸 수 있다. "
                         "예: --hm w_wall=0.0 --hm w_far=0.0 --hm w_height=1.2. "
                         "system.yaml 은 안 고친다")
    ap.add_argument("--corridor-gap", type=float, default=None, metavar="MM",
                    help="_corridor_ok 가 허용하는 주변 기둥 높이 여유. 기본은 "
                         "gripper_length_mm(150) — 박스 윗면보다 160 mm 높은 벽까지 통과한다. "
                         "0 이면 주변이 내 윗면을 못 넘는다 (협곡 금지)")
    ap.add_argument("--overhang", type=float, default=0.0, metavar="MM",
                    help="박스가 팔레트 밖으로 이만큼 나가도 된다고 보고 계획한다. "
                         "네 변 모두. 벽 쪽에서 --algo-clearance 로 잃는 자리를 되찾는다. "
                         "★ --plan-mode algo 전용. 씬의 팔레트는 실물 크기 그대로다")
    ap.add_argument("--diag-knock", action="store_true",
                    help="이미 놓인 박스가 **처음 밀리는 순간**을 잡는다 — 그때의 팔 자세로 "
                         "링크별 거리를 내서 '팔이 쳤나 / 혼자 움직였나' 를 가른다")
    ap.add_argument("--verbose", action="store_true", help="서비스 호출까지 전부 찍는다")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    _paths.check()
    cfg = _paths.system_config()
    DIMS_G.update({t["name"]: t["h_mm"] for t in cfg["boxes"]["types"]})
    n = a.boxes if a.boxes is not None else int(cfg["boxes"]["stage1"]["total_count"])

    # ── 계획 ───────────────────────────────────────────────
    pk = None
    # "online" 의 실제 뜻은 **미리 만든 계획표가 없다** 이다. algo 모드도 같으므로
    # 여기 한 줄만 넓히면 아래 급송 순서·대조표 경로가 전부 따라온다.
    online = a.plan_mode in ("online", "algo") and not a.plan and not a.plan_only
    if online:
        # 온라인은 미리 만드는 계획이 없다. 급송 순서(= 박스 목록)만 정한다.
        fed = manifest(cfg, n)
        # ★ 여기가 하드코딩이었다 — --order 를 로그에만 찍고 정렬은 항상 부피 내림차순이라
        #   online/algo 모드에서 --order arrival 이 **조용히 무시**됐다.
        _vol = lambda b: b["w_mm"] * b["d_mm"] * b["h_mm"]
        if a.order == "volume":
            fed.sort(key=lambda b: -_vol(b))
        elif a.order == "small-first":
            fed.sort(key=_vol)
        elif a.order == "shuffle":
            np.random.default_rng(cfg["boxes"]["random_seed"]).shuffle(fed)
        # arrival 은 manifest 가 뽑은 순서 그대로 — 정렬하지 않는다
        plan, dropped = [], []
        print("계획  온라인 (비전 -> 하이트맵 -> 다음 자리)  seed %s · 간격 %.0f mm · 급송 %s"
              % (cfg["boxes"]["random_seed"], a.clearance, a.order))
        print("  급송 순서 %s" % [b["name"] for b in fed])
        print("  ※ 자리는 사이클마다 정해진다. `--plan-mode offline` 이면 미리 만든 표를 쓴다.")
    elif a.plan:
        plan = json.loads(Path(a.plan).read_text())[:n]
        types = {t["name"]: t for t in cfg["boxes"]["types"]}
        fed = [types[p["box_type"]] for p in plan]
        dropped = []
        print("계획  %s (%d 박스)" % (a.plan, len(plan)))
    else:
        ped_now = a.pedestal if a.pedestal is not None else cfg["robot"]["pedestal_height_mm"]
        plan, fed, dropped, pk = build_plan(cfg, n, a.clearance, a.halo, a.order,
                                            ped_mm=ped_now)
        if float(ped_now) < 900:
            print("  ⚠ 페데스탈 %s — 팔꿈치가 far 기둥에 −65 mm 까지 파고든다 (issues/18)."
                  " --pedestal 900 권장" % ped_now)
        print("계획  오프라인 HeightmapPacker  seed %s · 간격 %.0f mm · 하강통로 %.0f mm · 급송 %s"
              % (cfg["boxes"]["random_seed"], a.clearance, a.halo, a.order))
    if not online:
        print_plan(plan, pk, dropped,
                   float(a.pedestal if a.pedestal is not None
                         else cfg["robot"]["pedestal_height_mm"]))

    if a.dump_plan and not online:
        out = Path(a.dump_plan)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(plan, ensure_ascii=False, indent=1))
        print("\n계획 저장 -> %s" % out)
        print("  다음: python3 scripts/check_place_plan.py %s" % out)
    if a.plan_only:
        return 0

    if not V.ASSET_SUCTION.exists():
        sys.exit("no asset: %s\n  -> scripts/build_gripper_usd.py 를 먼저 돌릴 것"
                 % V.ASSET_SUCTION)

    # ── Isaac ──────────────────────────────────────────────
    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})
    if not a.kit_log:
        # 터미널을 영상에 같이 찍으므로 Kit 로그는 Fatal 만 남긴다.
        # 우리 print() 는 carb 로그가 아니라서 영향 없다.
        try:
            import carb
            carb.settings.get_settings().set("/log/outputStreamLevel", "Fatal")
        except Exception:
            pass
    tee = V.start_log(a.log or _paths.ROOT / "results" / "demo_stack15.txt")

    import robot_motion as RM                            # packing_sim/isaac
    import cycle as CY                                   # packing_robot
    import sim_link as SL

    if a.pedestal is not None or a.standoff is not None or a.vel is not None:
        # ★ system.yaml 을 안 고치고 로봇만 옮겨 본다 (철칙 3 — 숫자 변경은 팀 결정이다).
        #   `bring_up` 이 `_paths.system_config()` 로 base 를 읽으므로 그 함수만 감싼다.
        _orig_cfg = _paths.system_config

        def _patched():
            c = _orig_cfg()
            b = c["zones"]["robot_base_world_mm"][a.zone]
            pal = c["zones"]["pallet_origin_world_mm"][a.zone]
            if a.pedestal is not None:
                b[2] = float(a.pedestal)
                c["robot"]["pedestal_height_mm"] = float(a.pedestal)
            if a.standoff is not None:
                # 팔레트 긴 변(y = pallet_origin.y)에서 -Y 로 standoff 만큼 떨어뜨린다
                b[1] = float(pal[1]) - float(a.standoff)
            if a.vel is not None:
                # robot_motion 이 cfg["robot"]["joint_speed_scale"] 로 스텝 수를 정한다
                c["robot"]["joint_speed_scale"] = float(a.vel)
            return c

        _paths.system_config = _patched
        print("★ 덮어쓴다 (system.yaml 은 그대로): 페데스탈 %s · 이격 %s · 속도배율 %s"
              % (a.pedestal if a.pedestal is not None else "기본",
                 a.standoff if a.standoff is not None else "기본",
                 a.vel if a.vel is not None else "기본"))

    ok_all = True
    server = link = None
    try:
        S = V.bring_up(zone=a.zone, render=a.gui, asset=V.ASSET_SUCTION)
        stage, cfg = S.world.stage, S.cfg
        belt_z = cfg["conveyor"]["belt_top_z_mm"] / 1000.0
        pick_x = cfg["conveyor"]["stopper_x_mm"] / 1000.0
        origin = cfg["zones"]["pallet_origin_world_mm"][a.zone]

        # ── 팔레트 밖 여유 (--overhang) ─────────────────────
        # ★ 씬의 팔레트는 **실물 크기 그대로** 두고, **계획만** 크게 본다. 그래서
        #   박스가 가장자리에서 실제로 조금 밖으로 나간다. 벽에는 낄 이웃이 없으므로
        #   --algo-clearance 를 벽에서까지 물릴 이유가 없다 — 그 손해를 되찾는 값이다.
        # ⚠ 먼 변(+Y)이 늘면 **도달범위**를 먹는다. 이격 400 재검증에서 B존 먼 코너
        #   여유가 +13 mm 뿐이었다 (CLAUDE.md §1). 5 mm 를 그냥 써도 되는지는 재봐야 한다.
        ov = float(a.overhang)
        cfg_plan, plan_origin = cfg, origin
        if ov > 0 or a.hm or a.corridor_gap is not None or a.algo_clearance is not None:
            import copy
            if a.plan_mode != "algo":
                raise SystemExit("계획 덮어쓰기(--overhang/--hm/--corridor-gap)는 "
                                 "--plan-mode algo 에서만 배선돼 있다")
            # ★ **계획용 사본**이다. 씬과 robot_motion 은 진짜 cfg 를 그대로 쓴다.
            cfg_plan = copy.deepcopy(cfg)
        if ov > 0:
            _cell = cfg["heightmap"]["cell_mm"]
            if int(round(2 * ov)) % int(_cell):
                print("!! --overhang %.1f: 2x%.1f = %d 가 격자 %d mm 의 배수가 아니다 — "
                      "격자가 잘려 실제 여유가 줄어든다"
                      % (ov, ov, int(round(2 * ov)), _cell))
            cfg_plan["pallet"]["width_mm"] = int(round(cfg["pallet"]["width_mm"] + 2 * ov))
            cfg_plan["pallet"]["depth_mm"] = int(round(cfg["pallet"]["depth_mm"] + 2 * ov))
            plan_origin = [origin[0] - ov, origin[1] - ov, origin[2]]
            cfg_plan["zones"]["pallet_origin_world_mm"][a.zone] = list(plan_origin)
            print("팔레트  계획 %d x %d mm (실물 %d x %d · 밖으로 %.1f) · 계획원점 %s"
                  % (cfg_plan["pallet"]["width_mm"], cfg_plan["pallet"]["depth_mm"],
                     cfg["pallet"]["width_mm"], cfg["pallet"]["depth_mm"], ov,
                     [round(v, 1) for v in plan_origin]))

        if a.algo_clearance is not None:
            cfg_plan["heightmap"]["plan_clearance_mm"] = int(a.algo_clearance)

        # ── 계획 가중치 덮어쓰기 (--hm) ─────────────────────
        # ★ system.yaml 은 공용 파일이다 (철칙 10). 팀 결정 없이 안 고친다.
        #   여기서 **계획용 사본만** 바꿔 값을 재 보고, 숫자를 들고 트랙 C 와 얘기한다.
        for _kv in (a.hm or []):
            _k, _, _v = _kv.partition("=")
            _k = _k.strip()
            # system.yaml 에 아직 없는 **선택 키**는 코어가 기본값으로 읽는다.
            # 그런 키까지 막으면 실험 자체가 안 되므로 화이트리스트로 통과시킨다.
            _OPT = {"corridor_far_ratio": 1.0, "corridor_near_gap_mm": 150.0}
            if _k not in cfg_plan["heightmap"] and _k not in _OPT:
                raise SystemExit("--hm %s: heightmap 에 없는 키다. 있는 키 %s (+선택 %s)"
                                 % (_k, sorted(cfg_plan["heightmap"]), sorted(_OPT)))
            _old = cfg_plan["heightmap"].get(_k, _OPT.get(_k))
            cfg_plan["heightmap"][_k] = type(_old)(float(_v)) if isinstance(_old, int) \
                else float(_v)
            print("계획값  heightmap.%s  %s -> %s" % (_k, _old, cfg_plan["heightmap"][_k]))

        # ── 협곡 허용 깊이 (--corridor-gap) ─────────────────
        # ★ `_corridor_ok` 는 "주변 기둥이 **박스 윗면 + gripper_length_mm(150) + tol**
        #   까지는 괜찮다" 로 본다. 즉 지금은 박스보다 160 mm 높은 벽에 둘러싸인 구덩이도
        #   통과한다. 이 값을 줄이면 협곡이 얕아진다. 0 이면 "주변이 내 윗면을 넘으면 안
        #   된다" 가 된다.
        # ⚠ **계획 사본에만** 건다. robot_motion 은 같은 키로 툴 길이를 읽으므로
        #   (robot_motion.py:191) 진짜 cfg 를 건드리면 흡착면 위치가 통째로 틀어진다.
        if a.corridor_gap is not None:
            _old = cfg_plan["robot"]["gripper_length_mm"]
            cfg_plan["robot"]["gripper_length_mm"] = float(a.corridor_gap)
            print("계획값  협곡 허용 %s -> %s mm (툴 길이는 %s 그대로)"
                  % (_old, a.corridor_gap, _old))

        if a.gui:
            # bring_up 의 기본 시점은 로봇 정면(-Y)이라 팔레트가 로봇 뒤에 가린다.
            # 적재가 보이도록 팔레트 쪽(+Y)에서 되돌아본다.
            from isaacsim.core.utils.viewports import set_camera_view
            if a.cam:
                set_camera_view(eye=np.array(a.cam[:3]), target=np.array(a.cam[3:]))
            else:
                set_camera_view(eye=S.base + np.array([1.7, 3.4, 1.9]),
                                target=S.base + np.array([-0.3, 0.4, 0.2]))

        # ── 씬 ─────────────────────────────────────────────
        # 벨트가 팔레트를 덮으면 놓으러 내려갈 때 팔이 벨트 콜라이더에 막힌다 (B4 에서 밟았다).
        VS.make_belt(stage, belt_z, pick_x - 0.35, 0.0, size=(1.2, 1.0))
        VS.make_pallet(stage, origin, cfg)
        VS.make_pedestal(stage, S.base, cfg)
        # ★ 마찰은 **양쪽 면**의 머티리얼로 정해진다. 박스에만 걸고 팔레트·벨트를
        #   기본값으로 두면 1층이 규정 마찰로 안 선다.
        for _p in ("/World/TestPallet", VS.BELT):
            _pr = stage.GetPrimAtPath(_p)
            if _pr:
                _apply_physics(stage, _pr, cfg, rigid=False)

        from pxr import Gf, UsdGeom
        COLOR = {"3호": (0.86, 0.42, 0.20), "4호": (0.78, 0.66, 0.36),
                 "5호": (0.55, 0.35, 0.24)}
        ids, dims_m, specs = [], [], {}
        for i, b in enumerate(fed):
            bid = BOX_ID_FMT % i
            path = BOX_PRIM_FMT % bid
            dm = [b["w_mm"] / 1000.0, b["d_mm"] / 1000.0, b["h_mm"] / 1000.0]
            sx = STAGE_XY[0] + (i % STAGE_COLS) * STAGE_PITCH
            sy = STAGE_XY[1] + (i // STAGE_COLS) * STAGE_PITCH
            VS.BOX = path                                # make_box 는 모듈 전역을 쓴다
            prim = VS.make_box(stage, "mesh", dm, (sx, sy, dm[2] / 2.0 + 0.002),
                               b["mass_g"] / 1000.0)
            UsdGeom.Gprim(prim).GetDisplayColorAttr().Set(
                [Gf.Vec3f(*COLOR.get(b["name"], (0.85, 0.25, 0.20)))])
            _apply_physics(stage, prim, cfg)          # ★ 솔버 반복수·오프셋·마찰
            ids.append(bid)
            dims_m.append(dm)
            specs[bid] = b

        from isaacsim.core.prims import SingleRigidPrim
        S.world.reset()                                   # ★ 15개를 다 만든 뒤 딱 한 번
        S.art.initialize()
        rps = []
        for i, bid in enumerate(ids):
            rp = SingleRigidPrim(prim_path=BOX_PRIM_FMT % bid, name="box_%02d" % i,
                                 reset_xform_properties=False)
            rp.initialize()
            rps.append(rp)
        by_id = dict(zip(ids, rps))
        _dim_of = dict(zip(ids, dims_m))                  # bid -> [w,d,h] m (--diag-knock)

        V.teleport(S, S.ready, steps=int(0.3 * S.hz))
        for _ in range(int(0.7 * S.hz)):
            S.world.step(render=S.render)

        print("\nASSET   %s" % V.ASSET_SUCTION)
        print("ZONE    %s   base %s mm   팔레트 원점 %s mm"
              % (a.zone, list(np.round(S.base * 1000).astype(int)), origin))
        print("박스    %d 개  %s" % (len(ids), [b["name"] for b in fed]))
        _ph = cfg["physics"]
        print("물리    솔버 %d/%d · 오프셋 rest %.0f · contact %.0f mm · 마찰 %.2f/%.2f"
              % (_ph["rigid_body"]["solver_position_iterations"],
                 _ph["rigid_body"]["solver_velocity_iterations"],
                 _ph["collider"]["rest_offset_mm"], _ph["collider"]["contact_offset_mm"],
                 _ph["material"]["static_friction"], _ph["material"]["dynamic_friction"]))

        _nt = [0]        # 물리 스텝 카운터 (렌더 솎기 · 시간 계측 공용)

        # ── 몸: robot_motion + 소켓 서버 대역 ───────────────
        cur = {"rp": rps[0], "half_h": dims_m[0][2] / 2.0}
        # ★ 충돌구체와 READY 를 넘긴다 — robot_motion 이 IK **가지**를 고를 때 쓴다.
        #   안 넘기면 시드 선택이 꺼지고 예전 동작 그대로다 (sim_main 등 다른 호출부).
        #   descriptor 경로를 robot_motion 이 알 방법이 없어 여기서 읽어 준다.
        rm = RM.RobotMotion(S.art, S.lula, cfg, ee=S.ee, gripper_path=GRIPPER,
                            gap_fn=lambda: VS.measure(S, cur["rp"], cur["half_h"])[0],
                            collision_spheres=VR.load_spheres(), ready_q=S.ready)
        handlers = dict(rm.handlers())

        def h_get_box_pose(req):                          # A 소유 명령. 대역이라 여기 둔다
            # ★ 소켓 스레드다. 현재 박스는 메인 루프(tick)가 갱신하는 캐시를 준다 —
            #   물리 스텝 중 PhysX 를 읽으면 undefined behavior 다 (robot_motion 주석).
            bid = (req or {}).get("box_id")
            if bid == cur.get("bid") and cur.get("pose") is not None:
                pos, yaw = cur["pose"]
                return {"ok": True, "pos_m": list(pos), "yaw_deg": yaw}
            rp = by_id.get(bid)                           # 폴백 — 사이클은 현재 박스만 묻는다
            if rp is None:
                return {"ok": False, "error": "unknown box_id"}
            pos, quat = rp.get_world_pose()
            return {"ok": True, "pos_m": [float(v) for v in pos], "yaw_deg": _yaw_of(quat)}

        handlers["get_box_pose"] = h_get_box_pose
        handlers["ping"] = lambda req: {"ok": True}

        # ── 계측: 지령 흡착면 vs 실제 흡착면 ────────────────
        #   move_ee 목표를 지나가며 받아 적기만 한다 (소켓 스레드 — PhysX 안 만진다).
        #   실제 위치 비교는 메인 스레드 tick() 에서 한다.
        #   ★ "사이클 중 최저점" 으로 잡으면 안 된다 — 사이클 시작 시 팔이 직전 사이클의
        #     후퇴 높이에 남아 있어서 그게 최저점으로 잡힌다 (2026-08-24 실측에서 밟았다).
        #     대신 **새 move_ee 가 올 때 직전 목표의 도달 결과를 확정**한다. 사이클의
        #     마지막 항목이 곧 배치 하강 바닥이다 (그 뒤 move 는 놓고 나서의 후퇴다).
        diag = {"tgt": None, "face": None, "box": None, "log": []}
        # ★ --diag-knock 도 이 훅이 필요하다 — 박스가 밀린 순간이 **어느 구간**인지
        #   말하려면 그때 진행 중이던 move_ee 목표를 알아야 한다.
        if a.diag_tcp or a.diag_knock:
            _h_move = handlers["move_ee"]

            def h_move_diag(req):
                t = (req or {}).get("target_m")
                if diag["tgt"] is not None and diag["face"] is not None:
                    diag["log"].append((diag["tgt"], diag["face"], diag["box"]))
                if t is not None:
                    diag["tgt"] = [float(v) for v in t]
                return _h_move(req)

            handlers["move_ee"] = h_move_diag
        # ── 놓고 나서 박스가 언제 멈추나 (--diag-release) ──
        # ★ `release` 는 세 단계(정지대기 -> damping 스파이크 -> 정착)로 1.5 s 를 쓴다.
        #   화면에서는 팔이 완전히 멈춰 있어 제일 눈에 띄는 구간이다. 박스가 실제로
        #   언제 멈추는지 100ms 구간으로 재야 어느 단계가 값을 하는지 갈린다.
        rel = {"n0": None, "p0": None, "bins": []}
        if a.diag_release:
            _h_rel = handlers["release"]

            def h_rel(req=None):
                rel["n0"], rel["bins"] = _nt[0], []
                rel["p0"] = np.asarray(cur["rp"].get_world_pose()[0], dtype=float)
                return _h_rel(req)

            handlers["release"] = h_rel

        # ── 사이클 시간이 어디로 가나 (--diag-time) ────────
        # ★ `n` 합계(실제 이동)는 한 사이클 19.5초 중 4.8초뿐이다. 나머지 15초가
        #   어디로 가는지 로그로는 안 보인다. 소켓 명령마다 **그 명령이 걸려 있는
        #   동안 흐른 물리 스텝**을 적어 둔다 — 대기가 어느 명령 밑에 쌓이는지 보인다.
        #   소켓 스레드에서 도는 코드라 int 하나만 읽는다 (PhysX 접근 없음).
        tm = {"cur": "(사이클 시작)", "at": 0, "acc": {}}
        if a.diag_time:
            def _timed(name, fn):
                def h(req=None):
                    tm["acc"][tm["cur"]] = tm["acc"].get(tm["cur"], 0) + (_nt[0] - tm["at"])
                    tm["cur"], tm["at"] = name, _nt[0]
                    return fn(req)
                return h
            handlers = {k: _timed(k, v) for k, v in handlers.items()}

        server = B4.StandInServer(handlers, port=a.port)
        print("소켓    127.0.0.1:%d  cmd %s" % (a.port, sorted(handlers)))

        # ── 머리 ───────────────────────────────────────────
        link = SL.SimLink(port=a.port)
        if online and a.plan_mode == "algo":
            # ★ 트랙 C 코어를 그대로 쓴다. 비전 루프(PalletVision)를 안 태운다 —
            #   algo 는 자기 heightmap 을 소유하고 실측 pose 로 갱신한다 (CLAUDE.md D3).
            # ★ 간격은 **코어가 건다** (heightmap_core.plan_clearance_mm). 여기서 또
            #   더하면 두 번 걸린다. --algo-clearance 는 cfg 사본을 덮어쓸 뿐이다.
            svc = AlgoServices(specs, cfg_plan, 0.0, verbose=a.verbose)
            places = {}
            print("계획  ★트랙 C 코어 (packing_algo/heightmap_core.py) · 간격 %d mm%s"
                  % (cfg_plan["heightmap"].get("plan_clearance_mm", 0),
                     "" if a.algo_clearance is None else " (--algo-clearance 로 덮어씀)"))
        elif online:
            PKM = _load_packer_module()
            hm, pal = cfg["heightmap"], cfg["pallet"]

            def factory():
                return PKM.HeightmapPacker(
                    size_xy=(pal["width_mm"] / 1000.0, pal["depth_mm"] / 1000.0),
                    max_height=pal["max_stack_mm"] / 1000.0,
                    cell=hm["cell_mm"] / 1000.0,
                    yaws_deg=tuple(float(v) for v in hm["allowed_yaw_deg"]),
                    min_support_ratio=float(hm["min_support_ratio"]),
                    support_tol=hm["support_tol_mm"] / 1000.0,
                    wall_margin=0.0, enforce_load_order=True,
                    descent_halo=a.halo / 1000.0)

            vision = PalletVision(by_id, specs, plan_origin)
            svc = OnlineServices(specs, factory, vision, a.clearance, verbose=a.verbose)
            places = {}
        else:
            places = {bid: {k: p[k] for k in
                            ("x_mm", "y_mm", "z_mm", "yaw_deg", "stack_top_mm") if k in p}
                      for bid, p in zip(ids, plan)}
            svc = PlanServices(specs, places, verbose=a.verbose)
        if a.diag_time:
            RM.SETTLE_LOG = True          # 구간마다 정착 틱·잔차를 한 줄씩 찍는다
        if a.poll_ms is not None:
            # ★ 모듈 상수만 덮어쓴다. cycle.py 정본은 그대로다.
            CY.MOTION_POLL_S = float(a.poll_ms) / 1000.0
            print("폴링      %.1f ms (기본 20.0) — 시뮬이 실시간보다 빠를 때만 의미 있다"
                  % a.poll_ms)
        if a.transit_tol is not None:
            # ★ 모듈 상수만 덮어쓴다. cycle.py 정본은 안 고친다 (기본값 = 예전 동작).
            CY.TRANSIT_TOL_MM = float(a.transit_tol)
            print("이송 허용  %.1f mm (하강은 %.1f mm 그대로)"
                  % (a.transit_tol, 2.0))
        cyc = CY.Cycle(link, svc, cfg_plan, zone=a.zone)

        rows, rest = [], {}                               # rest: 놓인 뒤의 기준 자세
        parked, rejected = [], set()                      # 라인 밖으로 치운 것 / 그중 §5 거부
        # 이미 놓인 박스가 처음 밀리는 순간 (--diag-knock). 사이클마다 초기화한다.
        if a.render_every < 1:
            raise SystemExit("--render-every 는 1 이상이어야 한다")
        if a.render_every > 1:
            print("렌더    %d 스텝에 1번만 그린다 (물리는 그대로 — 결과 동일)"
                  % a.render_every)
        kw = {"hit": None, "n": 0}
        KNOCK_TRIP_MM = 1.0                               # 이 이상 움직이면 "밀렸다" 로 본다
        j5 = [1e9, -1e9]                                  # 손목 감김 추적 (아래 주석)

        # ★ 물리를 **벽시계에 묶는다** — 재현성의 핵심이다.
        #
        #   메인 스레드는 `while th.is_alive(): tick()` 으로 물리를 최대 속도로 돌리는데,
        #   사이클 스레드(cycle._move)는 `time.sleep(0.02)` **벽시계**로 motion_status 를
        #   폴링한다. 헤드리스에서는 0.02 초에 물리가 수십~수백 스텝 나가므로 **명령 사이에
        #   흐르는 스텝 수가 매 실행 다르다.** 흡착 간극·정착 상태가 달라지고, 온라인
        #   계획이라 한 번 어긋나면 이후 배치가 통째로 갈린다.
        #
        #   실측: 같은 명령이 20/20(knock 1071) · 20/20(knock 2146) · 17/20 · 11/20 · 4/20
        #   으로 흩어졌다. 이 상태로는 **어떤 변경도 데모 통과율로 판정할 수 없다.**
        #
        #   여기서 1/hz 만큼 자면 물리 1 스텝 = 벽시계 1/hz 가 되어 폴링 간격(0.02 s)이
        #   항상 같은 스텝 수(hz*0.02 = 1.2 스텝 @60Hz)에 대응한다. `--gui` 는 렌더가
        #   이미 비슷한 페이스라 영향이 적고, 헤드리스를 그 조건에 맞추는 셈이다.
        #
        #   ★★ 그런데 **효과가 없었다** (2026-08-24 실측). 기본 끔.
        #     실시간으로 묶고 돌려도 4/20 이 나왔고, 실패는 전부 위치 잔차(297·79·72 mm)
        #     = 물리적 막힘이지 타이밍이 아니었다. 타임아웃도 안 걸렸다.
        #
        #     진짜 원인은 따로 있다: **계획이 5번째 박스부터 갈린다.** 두 실행의 최종
        #     대조표를 비교하면 앞 4개는 같고 5번째부터 완전히 다르다. 앞 배치의 미세한
        #     실측 오차가 heightmap 에 기록되고(CLAUDE.md D3 — 실측 pose 로 갱신 =
        #     자기교정), 그게 다음 계획을 바꾸고, 온라인이라 이후가 통째로 발산한다.
        #     즉 **자기교정 구조의 대가**이지 페이싱 문제가 아니다.
        #
        #     노브는 남긴다 — PL_REALTIME=1 이면 물리를 벽시계에 묶는다. 눈으로 볼 때나
        #     타이밍 가설을 다시 검증할 때 쓴다. 대신 실행이 8분대로 느려진다.
        realtime = os.environ.get("PL_REALTIME", "0") != "0"
        step_dt = 1.0 / S.hz

        def tick():
            """물리 한 스텝. 소켓 스레드가 읽을 캐시(현재 박스 pose)도 여기서 갱신한다."""
            t_step = time.time()
            rm.step()
            pos, quat = cur["rp"].get_world_pose()
            cur["pose"] = ([float(v) for v in pos], _yaw_of(quat))
            # ★ 렌더만 건너뛴다. `world.step` 은 매번 부르므로 물리 스텝 수는 그대로다 —
            #   결과가 비트 단위로 같다. 궤적을 바꾸는 --vel 과 여기가 다르다.
            _nt[0] += 1
            S.world.step(render=S.render and (_nt[0] % a.render_every == 0))
            _j5(S, j5)
            if realtime:
                # 남은 시간만 잔다. 이미 늦었으면 안 잔다 (따라잡기를 시도하지 않는다 —
                # 그러면 다시 스텝 수가 실행마다 달라진다).
                lag = step_dt - (time.time() - t_step)
                if lag > 0:
                    time.sleep(lag)
            if a.diag_release and rel["n0"] is not None:
                k = int((_nt[0] - rel["n0"]) / (0.1 * S.hz))    # 100ms 구간 번호
                if k >= len(rel["bins"]) and k < 25:            # 2.5 초까지만
                    now = np.asarray(cur["rp"].get_world_pose()[0], dtype=float)
                    rel["bins"].append(float(np.linalg.norm(now - rel["p0"]) * 1000.0))
                    rel["p0"] = now
            if a.diag_knock and kw["hit"] is None:
                # ★ **이미 놓인** 박스만 본다 (rest). 매달린 박스는 --diag-tcp 몫이다.
                #   첫 1 mm 에서 멈춘다 — 다 밀린 뒤의 자세는 범인을 안 가리킨다.
                kw["n"] += 1
                for _pid, _ref in rest.items():
                    _p = np.asarray(by_id[_pid].get_world_pose()[0], dtype=float)
                    _d = (_p - _ref) * 1000.0
                    if float(np.linalg.norm(_d)) <= KNOCK_TRIP_MM:
                        continue
                    _w, _d, _h = _dim_of[_pid]
                    # yaw 90/270 이면 x·y 반폭이 바뀐다. 안 바꾸면 AABB 가 틀려
                    # 링크 거리가 통째로 어긋난다.
                    if int(round(abs(_yaw_of(by_id[_pid].get_world_pose()[1])) / 90.0)) % 2:
                        _w, _d = _d, _w
                    _hf = np.array([_w, _d, _h], dtype=float) / 2.0
                    kw["hit"] = (kw["n"], _pid, [float(v) for v in _d],
                                 [float(v) for v in rm.suction_pose()],
                                 _link_dists(rm, _p - _hf, _p + _hf)[:3],
                                 list(diag["tgt"]) if diag["tgt"] else None)
                    break
            if a.diag_tcp:
                # 현재 흡착면(FK)과 **매달린 박스 중심**을 갱신한다.
                # 짝짓기는 h_move_diag 가 한다 (소켓 스레드는 이 값을 읽기만 한다).
                diag["face"] = [float(v) for v in rm.suction_pose()]
                diag["box"] = list(cur["pose"][0]) if cur.get("pose") else None
                if diag.get("p0") and diag["box"] and diag.get("hit") is None:
                    diag["nt"] += 1
                    d0 = (np.array(diag["box"][:2]) - np.array(diag["p0"][:2])) * 1000.0
                    if float(np.hypot(*d0)) > 1.0:      # 1 mm 넘게 움직인 첫 순간
                        diag["hit"] = (diag["nt"], list(diag["face"]), [float(v) for v in d0])

        if not a.kit_log:
            # 기동 로그(확장 deprecation 등 ~100줄)를 화면에서 치우고 헤더만 다시 찍는다.
            # 영상에 터미널이 같이 찍히므로 여기부터 깨끗하게 시작한다. 파일 로그는 그대로다.
            print("\033[2J\033[H", end="")
            print("━" * 66)
            print(" 팔레타이징 데모 — 박스 %d개 · %d단 적재 · zone %s · 페데스탈 %.0f mm"
                  % (len(ids), len(sorted({q["z_mm"] for q in plan})) if plan else 0,
                     a.zone, float(a.pedestal if a.pedestal is not None
                                   else cfg["robot"]["pedestal_height_mm"])))
            print(" H2017 · Lula IK · 호 이송 · SurfaceGripper 흡착 · HeightmapPacker 계획")
            print("━" * 66)

        t_all = time.time()
        for i, bid in enumerate(ids):
            spec, dm = specs[bid], dims_m[i]
            rp = by_id[bid]

            # 급송 — 벨트 스토퍼 자리로 옮긴다. 속도를 0 으로 지우지 않으면 튄다.
            pick_c = np.array([pick_x - dm[0] / 2.0, 0.0, belt_z + dm[2] / 2.0 + 0.001])
            rp.set_world_pose(position=pick_c, orientation=np.array([1.0, 0.0, 0.0, 0.0]))
            rp.set_linear_velocity(np.zeros(3))
            rp.set_angular_velocity(np.zeros(3))
            cur["rp"], cur["half_h"], cur["bid"] = rp, dm[2] / 2.0, bid
            cur["pose"] = None                            # 이번 스텝 tick 이 채운다
            diag["log"].clear()                           # 사이클마다 초기화 (--diag-tcp)
            kw["hit"], kw["n"] = None, 0                  # 사이클마다 초기화 (--diag-knock)
            tm["acc"].clear(); tm["cur"], tm["at"] = "(사이클 시작)", _nt[0]
            for _ in range(int(0.3 * S.hz)):
                tick()

            pick_pose = {"pos_m": [float(v) for v in rp.get_world_pose()[0]], "yaw_deg": 0.0}
            if a.diag_tcp:
                # ★ 픽 목표를 읽은 이 순간부터 박스가 **언제 · 팔이 어디 있을 때** 움직이는지
                #   잡는다. 접근 중에 밀린다면 팔이 스친 것이고, 그 순간의 흡착면 위치가
                #   범인을 가리킨다.
                diag["p0"] = list(pick_pose["pos_m"])
                diag["hit"] = None
                diag["nt"] = 0
            if online:
                print("\n[%2d/%d] %s %s  %s mm  %.0f kg"
                      % (i + 1, len(ids), bid, spec["name"],
                         [spec["w_mm"], spec["d_mm"], spec["h_mm"]], spec["mass_g"] / 1000.0))
            else:
                print("\n[%2d/%d] %s %s  %s mm  %.0f kg   -> P(%d,%d) z%d yaw%d"
                      % (i + 1, len(ids), bid, spec["name"],
                         [spec["w_mm"], spec["d_mm"], spec["h_mm"]], spec["mass_g"] / 1000.0,
                         places[bid]["x_mm"], places[bid]["y_mm"],
                         places[bid]["z_mm"], places[bid]["yaw_deg"]))

            got = {}
            th = threading.Thread(target=lambda: got.update(r=cyc.run(bid, pick_pose)),
                                  daemon=True)
            th.start()
            steps0, t0 = 0, time.time()
            while th.is_alive() and time.time() - t0 < CYCLE_WALL_S:
                tick()
                steps0 += 1
            th.join(timeout=5.0)
            res = got.get("r", "TIMEOUT")

            for _ in range(int(SETTLE_S * S.hz)):         # 놓은 뒤 정착
                tick()
                steps0 += 1

            if online:
                places[bid] = dict(svc.last) if svc.last else {
                    "x_mm": 0, "y_mm": 0, "z_mm": 0, "yaw_deg": 0}
            pos = np.asarray(rp.get_world_pose()[0], dtype=float)
            want = np.asarray(CY.pallet_place_to_world(places[bid], spec["h_mm"],
                                                       plan_origin))
            err = (pos - want) * 1000.0

            # knock — 이미 놓인 박스가 이번 사이클에 얼마나 밀렸나
            knock, who = 0.0, "-"
            for pid, ref in rest.items():
                dmm = float(np.linalg.norm(
                    np.asarray(by_id[pid].get_world_pose()[0], dtype=float) - ref) * 1000.0)
                if dmm > knock:
                    knock, who = dmm, pid
            # ★ **놓은 것만** 기준에 등록한다. rest 는 "이미 놓인 박스" 라는 뜻인데,
            #   결과와 무관하게 넣으면 못 놓은 박스(NO_FEASIBLE_POSE 로 벨트에 남은 것)가
            #   들어가고, 그 박스가 벨트로 흘러간 거리가 다음 사이클의 knock 으로 찍힌다.
            #   실측: 거부된 5호가 knock 1925 mm 로 잡혀 표가 통째로 못 읽게 됐다.
            if res == "PLACED":
                rest[bid] = pos.copy()

            # ── §5 판정 — `NO_FEASIBLE_POSE` ≠ `PALLET_FULL` ────────────────
            # ★ `cycle.run` 은 둘 다 뭉뚱그려 돌려준다(`BLOCKED`). 어느 쪽인지는
            #   svc 가 받은 msg 만 안다 — orchestrator.py:240 과 같은 방식이다.
            #   여기서 안 치우면 거부된 박스가 **스토퍼에 그대로 서 있고**, 다음
            #   박스를 같은 자리로 급송하면서 겹친다. 실측: 14번 NO_FEASIBLE_POSE
            #   직후 15번이 간극 -13.95 mm 로 흡착 실패했다.
            pmsg = getattr(svc, "last_plan_msg", None)
            if res != "PLACED" and res != "PALLET_FULL" and pmsg == "NO_FEASIBLE_POSE":
                res = "REJECTED"                      # 정상 판단이다. 실패로 세지 않는다
            if res != "PLACED":
                _park(rp, dm, len(parked))            # 라인 밖으로 — 다음 박스 길을 연다
                parked.append(bid)
                if res == "REJECTED":
                    rejected.add(bid)
                    places.pop(bid, None)             # 계획이 없다. 대조표에서 빼야 한다

            rows.append({"id": bid, "type": spec["name"], "res": res, "err": err,
                         "sim_s": steps0 / S.hz, "knock": knock, "who": who})
            print("       %s  오차 [%+.1f %+.1f %+.1f] mm  %.1f s 시뮬  knock %.1f mm (%s)"
                  % (res, err[0], err[1], err[2], steps0 / S.hz, knock, who))
            if a.diag_release and rel["bins"]:
                print("       ⏸ release 이후 박스 이동 (100ms 구간, mm)")
                print("          " + "  ".join("%5.2f" % v for v in rel["bins"][:15]))
                print("          %s"
                      % "  ".join("%5s" % ("%.1fs" % ((i + 1) * 0.1))
                                  for i in range(min(15, len(rel["bins"])))))
                rel["n0"] = None
            if a.diag_time:
                # 마지막 명령 이후 남은 스텝까지 닫는다
                tm["acc"][tm["cur"]] = tm["acc"].get(tm["cur"], 0) + (_nt[0] - tm["at"])
                tot = sum(tm["acc"].values()) or 1
                items = sorted(tm["acc"].items(), key=lambda kv: -kv[1])
                print("       ⏱ 사이클 %.1f s — 명령별 (그 명령이 걸려 있는 동안 흐른 시간)"
                      % (tot / S.hz))
                for k2, v2 in items:
                    if v2 <= 0:
                        continue
                    print("          %-16s %6.2f s  %4.1f%%  %s"
                          % (k2, v2 / S.hz, 100.0 * v2 / tot, "█" * int(30.0 * v2 / tot)))
            if a.diag_knock and kw["hit"]:
                n0, vid, dmm, face, links, tgt = kw["hit"]
                print("       ★ 놓인 박스가 처음 밀린 순간: %s 가 스텝 %d (%.2f s) 에 "
                      "[%+.1f %+.1f %+.1f] mm"
                      % (vid, n0, n0 / S.hz, dmm[0], dmm[1], dmm[2]))
                print("         그때 흡착면 %s   진행 중이던 move_ee 목표 %s"
                      % (np.round(np.array(face), 3),
                         np.round(np.array(tgt), 3) if tgt else "-"))
                # ★ 여기가 판정이다. 음수면 **그 링크가 박스 안에 들어가 있다** = 팔이 쳤다.
                #   전부 넉넉히 양수면 팔은 안 닿았다 = 물리(이웃 접촉·지지 부족)다.
                print("         링크별 거리  %s"
                      % "  ".join("%s %+.1f mm" % (ln, dv) for dv, ln in links))
                if not links:
                    print("         -> 충돌구체가 안 실렸다. 링크 판정 불가")
                elif links[0][0] > 20.0:
                    print("         -> 팔은 %s 가 %.1f mm 로 가장 가깝다. **안 닿았다** — "
                          "이웃 접촉이나 지지 부족을 봐야 한다" % (links[0][1], links[0][0]))
                else:
                    print("         -> **%s 가 쳤다** (%.1f mm)" % (links[0][1], links[0][0]))
            if a.diag_tcp and diag["log"]:
                tg, fc, bx = diag["log"][-1]
                tg, fc = np.array(tg, dtype=float), np.array(fc, dtype=float)
                d = (fc - tg) * 1000.0
                print("       TCP  지령 %s / 실제 %s  ->  **팔 오차** "
                      "[%+.1f %+.1f %+.1f] mm  |xy| %.1f"
                      % (np.round(tg, 3), np.round(fc, 3), d[0], d[1], d[2],
                         float(np.hypot(d[0], d[1]))))
                if bx is not None:
                    # 매달린 박스 중심이 흡착면 중심에서 얼마나 벗어나 있나.
                    #   팔 오차가 작은데 이게 크면 **박스가 패드 위에서 밀린 것**이다.
                    o = (np.array(bx, dtype=float)[:2] - fc[:2]) * 1000.0
                    print("       BOX  놓는 순간 박스중심 − 흡착면 = [%+.1f %+.1f] mm  |xy| %.1f"
                          % (o[0], o[1], float(np.hypot(*o))))
                    # ★ 구간별로 편다 — **팔 오차와 박스 오프셋을 같이** 봐야 범인이 갈린다.
                    #   각 줄은 그 구간이 끝난 시점의 값이다. 첫 줄은 직전 사이클의
                    #   잔여 위치라 무시한다 (박스를 아직 안 잡았다).
                    if diag.get("hit"):
                        n0, f0, d0 = diag["hit"]
                        print("       ★ 박스가 처음 움직인 순간: 픽목표 읽은 뒤 %d 스텝 (%.2f s)"
                              % (n0, n0 / S.hz))
                        print("         그때 흡착면 %s  ·  박스 이동 [%+.1f %+.1f] mm"
                              % (np.round(np.array(f0), 3), d0[0], d0[1]))
                    print("       ↳ 구간별   팔오차(실제-지령) | 박스오프셋(박스-흡착면)")
                    for k, (t2, f2, b2) in enumerate(diag["log"], 1):
                        if b2 is None:
                            continue
                        t2 = np.array(t2, dtype=float)
                        f2 = np.array(f2, dtype=float)
                        ae = (f2 - t2) * 1000.0
                        oo = (np.array(b2, dtype=float)[:2] - f2[:2]) * 1000.0
                        print("          %d) 지령 %-24s 팔 [%+6.1f %+6.1f] |%5.1f|"
                              "   박스 [%+7.1f %+7.1f] |%5.1f|"
                              % (k, str(np.round(t2, 3)), ae[0], ae[1],
                                 float(np.hypot(ae[0], ae[1])),
                                 oo[0], oo[1], float(np.hypot(*oo))))
            if res == "REJECTED":
                print("       ·· 이 박스는 자리가 없다 (NO_FEASIBLE_POSE) — "
                      "버리고 계속한다 (누적 %d)" % len(rejected))
            elif res == "PALLET_FULL":
                print("       == PALLET_FULL — 시나리오를 여기서 끝낸다 (CLAUDE.md §5)")
                break
            elif res != "PLACED":
                print("       !! 이 박스는 실패다. 다음으로 넘어간다")

        # ── 표 ─────────────────────────────────────────────
        wall = time.time() - t_all
        print("\n" + "=" * 74)
        print("  #  박스      종류  결과       오차 XY / Z [mm]    시뮬 s   knock mm")
        for i, r in enumerate(rows, 1):
            print("%3d  %-9s %-4s  %-9s  %6.1f / %+6.1f   %6.1f   %7.1f"
                  % (i, r["id"], r["type"], r["res"],
                     float(np.linalg.norm(r["err"][:2])), r["err"][2],
                     r["sim_s"], r["knock"]))

        placed = [r for r in rows if r["res"] == "PLACED"]
        top_mm = 0.0
        for r, dm in zip(rows, dims_m):
            if r["res"] != "PLACED":
                continue
            z = float(by_id[r["id"]].get_world_pose()[0][2]) * 1000.0 + dm[2] * 500.0
            top_mm = max(top_mm, z - origin[2])
        layers = sorted({places[r["id"]]["z_mm"] for r in rows if r["id"] in places})
        max_knock = max([r["knock"] for r in rows] or [0.0])
        off = [r for r in rows
               if r["res"] == "PLACED" and (float(np.linalg.norm(r["err"][:2])) > PLACE_TOL_XY_MM
                                            or abs(r["err"][2]) > PLACE_TOL_Z_MM)]

        # ★ 버림은 실패가 아니다 (§5). 뭉뚱그리면 "실패 N건" 이 부풀려 나오고
        #   발표에서 알고리즘 정상 판단과 진짜 고장을 구분할 수 없다.
        n_rej = len([r for r in rows if r["res"] == "REJECTED"])
        n_fail = len(rows) - len(placed) - n_rej
        print("\n놓은 것       %d / %d   (적재 %d · 버림 %d · 실패 %d)"
              % (len(placed), len(rows), len(placed), n_rej, n_fail))
        print("%s   %d 단 (밑면 z %s)"
              % ("배치 단수    " if online else "계획 층수    ", len(layers), layers))
        print("실측 최상단   %.0f mm (팔레트 상면 기준) / max_stack %d mm"
              % (top_mm, cfg["pallet"]["max_stack_mm"]))
        print("최대 knock    %.1f mm (경고선 %.0f)" % (max_knock, KNOCK_WARN_MM))

        # ── 팔레트 밖으로 실제로 얼마나 나갔나 ──────────────
        # ★ --overhang 을 안 써도 찍는다. 배치 오차만으로도 나갈 수 있고, 트럭 상차가
        #   되느냐는 **계획값이 아니라 실측 외곽**이 정한다 (팔레트 3장 x 800 = 2400,
        #   트럭 바닥 2800 — 여유 400 mm).
        _px0, _py0 = origin[0], origin[1]
        _px1 = _px0 + cfg["pallet"]["width_mm"]
        _py1 = _py0 + cfg["pallet"]["depth_mm"]
        _out, _worst = 0.0, "-"
        for r in placed:
            _c = np.asarray(by_id[r["id"]].get_world_pose()[0], dtype=float) * 1000.0
            _w, _d, _ = [v * 1000.0 for v in _dim_of[r["id"]]]
            if int(round(abs(_yaw_of(by_id[r["id"]].get_world_pose()[1])) / 90.0)) % 2:
                _w, _d = _d, _w
            _e = max(_px0 - (_c[0] - _w / 2.0), (_c[0] + _w / 2.0) - _px1,
                     _py0 - (_c[1] - _d / 2.0), (_c[1] + _d / 2.0) - _py1)
            if _e > _out:
                _out, _worst = _e, r["id"]
        print("팔레트 밖     %.1f mm (%s)%s"
              % (max(0.0, _out), _worst,
                 "   ← --overhang %.1f 로 허용한 값" % ov if ov > 0 else ""))

        # ── 최종 자리 대조 ─────────────────────────────────
        # knock 은 "얼마나 움직였나"만 말하고 **어디로 갔는지**를 안 말한다. 사고가 나면
        # 그게 필요하다 — 어느 방향으로 밀렸는지 알아야 무엇이 쳤는지 좁혀진다.
        print("\n최종 자리 (계획 대비, 팔레트 로컬 mm)")
        print("  #  박스      종류  계획 P(x,y) z      실제 어긋남 dx  dy  dz")
        for i, (bid, dm) in enumerate(zip(ids, dims_m), 1):
            pl = places.get(bid)
            if bid in rejected:
                print("%3d  %-9s %-4s  (자리 없음 — 버림. 라인 밖으로 치웠다)"
                      % (i, bid, specs[bid]["name"]))
                continue
            if bid in parked:
                # ★ 치운 박스는 **주차 자리**에 있다. 계획 대비 어긋남을 찍으면 수 m 짜리
                #   숫자가 나오고 "밀림" 표시까지 붙어 진짜 밀린 박스를 가린다.
                _plan = ("(%4d,%4d) %4d" % (pl["x_mm"], pl["y_mm"], pl["z_mm"])
                         if pl else "(계획 없음)")
                print("%3d  %-9s %-4s  %-16s 실패 — 라인 밖으로 치웠다"
                      % (i, bid, specs[bid]["name"], _plan))
                continue
            if pl is None:
                print("%3d  %-9s %-4s  (계획 없음 — 사이클이 배치 전에 끝났다)"
                      % (i, bid, specs[bid]["name"]))
                continue
            want = np.asarray(CY.pallet_place_to_world(pl, specs[bid]["h_mm"], plan_origin))
            now = np.asarray(by_id[bid].get_world_pose()[0], dtype=float)
            dd = (now - want) * 1000.0
            mark = "  <-- 밀림" if float(np.linalg.norm(dd)) > KNOCK_WARN_MM else ""
            print("%3d  %-9s %-4s  (%4d,%4d) %4d   %+7.1f %+7.1f %+7.1f%s"
                  % (i, bid, specs[bid]["name"], pl["x_mm"], pl["y_mm"], pl["z_mm"],
                     dd[0], dd[1], dd[2], mark))
        print("joint_5 범위  %.1f ~ %.1f deg (한계 ±360, 되감기 미구현)" % (j5[0], j5[1]))
        print("총 시간       %.1f s 벽시계 · %.1f s 시뮬 (사이클 평균 %.1f s)"
              % (wall, sum(r["sim_s"] for r in rows),
                 sum(r["sim_s"] for r in rows) / max(1, len(rows))))

        for cond, msg in (
                (len(placed) + n_rej == len(rows),
                 "%d 개가 PLACED 도 REJECTED 도 아니다 (진짜 실패)" % n_fail),
                (not off, "배치 오차 초과 %s" % [r["id"] for r in off]),
                (max_knock <= KNOCK_WARN_MM,
                 "이미 놓인 박스가 %.1f mm 밀렸다 — 이송 높이를 볼 것" % max_knock),
                (len(layers) >= 3, "계획이 %d 층뿐이다" % len(layers)),
                (max(abs(j5[0]), abs(j5[1])) < 324.0,
                 "joint_5 가 %.0f deg — 한계 360 의 90%% 를 넘었다. 되감기가 필요하다"
                 % max(abs(j5[0]), abs(j5[1])))):
            if not cond:
                print("  !! %s" % msg)
            ok_all &= cond

        print("\nRESULT: %s" % ("통과" if ok_all else "실패"))
        if a.hold and a.gui:
            V.hold_open(S)
    except BaseException:
        import traceback
        print("\n!! 예외\n" + traceback.format_exc())
        raise
    finally:
        if link is not None:
            link.close()
        if server is not None:
            server.close()
        import shutil
        try:
            shutil.rmtree(S.tmpdir, ignore_errors=True)
        except NameError:
            pass
        tee.close()
        kit.close()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
