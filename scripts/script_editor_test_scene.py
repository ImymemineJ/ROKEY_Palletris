"""Isaac Sim Script Editor 용 — **최소 테스트 씬**을 만든다 (씬 담당 없이 혼자 돌려보기).

    Isaac Sim GUI ▸ File ▸ New ▸ Window ▸ Script Editor 에 붙여넣고 **Ctrl+Enter**.

## 왜 있나

`script_editor_qr_boxes.py` 로 QR 박스를 스폰해 보려면 **벨트가 있어야** 한다.
김학민의 실제 씬(`test1-1.usd`)을 안 받아도 혼자 확인할 수 있도록,
`system.yaml` 의 숫자만으로 벨트·바닥·스토퍼를 그린다.

**이건 확인용 껍데기지 실제 씬이 아니다.** 없는 것:
컨베이어 구동(`IsaacConveyor`) · 로봇 · 카메라 4종 · tf · Action Graph.
전부 트랙 A 소관이다. 여기서 보는 것은 **박스가 벨트에 제대로 앉는가**뿐이다.

## 쓰는 순서

    1) File ▸ New  (빈 스테이지)
    2) 이 파일 붙여넣고 Ctrl+Enter          → 벨트·바닥·스토퍼가 생긴다
    3) script_editor_qr_boxes.py 붙여넣고 Ctrl+Enter  → QR 박스가 벨트 위에 뜬다
    4) ▶ Play                                → 박스가 벨트에 내려앉는다 (10mm 낙하)
    5) (덤) PUSH_MPS 를 켜고 이 파일을 다시 Ctrl+Enter → 박스가 +X 로 흘러간다

## 좌표 규약 (CLAUDE.md §4)

    z = 0        도크 바닥 = 트럭 적재함 바닥   ← 야드 지면이 아니다
    z = 800      벨트 상면 (롤러 면)
    z = 810      스폰 지점, 박스 **밑면**
    x = 0~7500   main 컨베이어 / 7500~10000 픽업 구간 / 10000 스토퍼
"""
import os
import sys

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

# ══════════════════════════════════════════════════════════════════════════
PUSH_MPS = 0.0        # > 0 이면 이미 스폰된 박스에 +X 초기속도를 준다 (벨트 흉내)
                      #   ★ 진짜 컨베이어가 아니다. 마찰로 곧 선다 — 눈으로 보는 용도.
SHOW_PALLET = True    # 팔레트 3장을 참고용으로 그린다 (물리 없음)
# ══════════════════════════════════════════════════════════════════════════


def load_spec():
    """`system.yaml` 을 읽는다 (`packing_common.config` 사용).

    Isaac 내장 파이썬(3.11)은 ROS 워크스페이스(3.12)를 안 보므로 경로를 붙여 준다.
    Script Editor 는 `__file__` 이 없을 수 있어 cwd 에서도 훑는다.
    """
    starts = [os.getcwd()]
    if "__file__" in globals():
        starts.insert(0, os.path.dirname(os.path.realpath(__file__)))
    for start in starts:
        d = start
        for _ in range(8):
            cand = os.path.join(d, "src", "packing_common")
            if os.path.isdir(cand):
                if cand not in sys.path:
                    sys.path.insert(0, cand)
                break
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    try:
        from packing_common.config import load
        return load()
    except Exception as e:                       # noqa: BLE001
        raise RuntimeError(
            f"system.yaml 을 못 읽었다 ({e}).\n"
            "  환경변수로 알려 줄 것:  PALLETRIS_CONFIG=~/palletris_ws/src/"
            "packing_bringup/config/system.yaml") from e


CFG = load_spec()
CONV = CFG["conveyor"]
MM = 1000.0

BELT_TOP = CONV["belt_top_z_mm"]                      # 800
MAIN_LO, MAIN_HI = CONV["stage1"]["main_range_mm"]    # 0 ~ 7500
PICK_LO, PICK_HI = CONV["pickup_range_mm"]            # 7500 ~ 10000
STOPPER_X = CONV["stopper_x_mm"]                      # 10000

# ★ 벨트 폭은 yaml 에 없다 — CLAUDE.md §4 가 정본이다 (롤러 면 1546).
#   ⚠ bbox 1970 을 쓰면 안 된다. 그건 난간 포함 값이다.
BELT_W = CONV.get("belt_width_mm")
if BELT_W is None:
    BELT_W = 1546
    print(f"[WARN] system.yaml 에 conveyor.belt_width_mm 이 없다 → CLAUDE.md §4 값 {BELT_W} 사용.")
    print("       (bbox 1970 은 난간 포함이라 사양이 아니다 — §4 'bbox 를 그대로 쓰지 마라')")

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("열린 스테이지가 없다. File > New 로 만들 것.")

if not stage.GetPrimAtPath("/World").IsValid():
    UsdGeom.Xform.Define(stage, "/World")

# ── 물리 · 조명 ──────────────────────────────────────────────────
if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
if not stage.GetPrimAtPath("/World/DomeLight").IsValid():
    # §5: 오프스크린 렌더는 GUI 조명 리그를 못 받는다. 돔 라이트가 꺼져 있으면
    #     눈으로는 멀쩡한데 카메라 이미지가 캄캄하다.
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)


def slab(path, x0, x1, y0, y1, top_z, thick_mm, color, collide=True):
    """축정렬 직육면체 하나. x/y 는 범위(mm), top_z 는 **상면** 높이(mm)."""
    c = UsdGeom.Cube.Define(stage, path)
    c.GetSizeAttr().Set(1.0)
    c.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
    c.CreateDisplayColorAttr([color])
    api = UsdGeom.XformCommonAPI(c)
    api.SetTranslate(Gf.Vec3d((x0 + x1) / 2 / MM, (y0 + y1) / 2 / MM,
                              (top_z - thick_mm / 2) / MM))
    api.SetScale(Gf.Vec3f((x1 - x0) / MM, (y1 - y0) / MM, thick_mm / MM))
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())    # RigidBodyAPI 없음 = 정적
    return c


# ── 도크 바닥 (z = 0) ────────────────────────────────────────────
slab("/World/TestScene/DockFloor", -1000, 13000, -3000, 3000, 0, 200,
     Gf.Vec3f(0.22, 0.23, 0.26))

# ── 벨트 두 조각 (상면 z = 800) ──────────────────────────────────
half_w = BELT_W / 2
slab("/World/TestScene/Belt_main", MAIN_LO, MAIN_HI, -half_w, half_w,
     BELT_TOP, 120, Gf.Vec3f(0.30, 0.32, 0.36))
slab("/World/TestScene/Belt_pickup", PICK_LO, PICK_HI, -half_w, half_w,
     BELT_TOP, 120, Gf.Vec3f(0.38, 0.40, 0.45))

# ── 스토퍼 (상면 z = 900) ────────────────────────────────────────
slab("/World/TestScene/Stopper", STOPPER_X, STOPPER_X + 100, -half_w, half_w,
     900, 100, Gf.Vec3f(0.75, 0.35, 0.25))

# ── 팔레트 3장 (참고용, 물리 없음) ───────────────────────────────
if SHOW_PALLET:
    P = CFG["pallet"]
    for z, org in CFG["zones"]["pallet_origin_world_mm"].items():
        slab(f"/World/TestScene/Pallet_{z}", org[0], org[0] + P["width_mm"],
             org[1], org[1] + P["depth_mm"], org[2], P["thickness_mm"],
             Gf.Vec3f(0.55, 0.45, 0.30), collide=False)

print(f"[OK] 테스트 씬 — 바닥 z=0 · 벨트 상면 z={BELT_TOP} (폭 {BELT_W}) · "
      f"main {MAIN_LO}~{MAIN_HI} · 픽업 {PICK_LO}~{PICK_HI} · 스토퍼 {STOPPER_X}")

# ── 덤: 이미 스폰된 박스를 +X 로 밀어 본다 ───────────────────────
if PUSH_MPS > 0:
    grp = stage.GetPrimAtPath("/World/QR_Boxes")
    n = 0
    if grp.IsValid():
        for prim in grp.GetChildren():
            rb = UsdPhysics.RigidBodyAPI(prim)
            if rb:
                rb.CreateVelocityAttr().Set(Gf.Vec3f(PUSH_MPS, 0.0, 0.0))
                n += 1
    print(f"[OK] 박스 {n}개에 +X {PUSH_MPS} m/s 초기속도.")
    print("     ⚠ 진짜 컨베이어가 아니다 — 마찰로 곧 선다. IsaacConveyor 는 트랙 A 소관.")

print()
print("다음: script_editor_qr_boxes.py 를 붙여넣고 Ctrl+Enter → ▶ Play")
