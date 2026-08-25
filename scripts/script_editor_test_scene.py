"""Isaac Sim Script Editor 용 — **최소 테스트 씬**을 만든다 (씬 담당 없이 혼자 돌려보기).

    Isaac Sim GUI ▸ File ▸ New ▸ Window ▸ Script Editor 에 붙여넣고 **Ctrl+Enter**.

## 왜 있나

`script_editor_qr_boxes.py` 로 QR 박스를 스폰해 보려면 **벨트가 있어야** 한다.
김학민의 실제 씬(`test1-1.usd`)을 안 받아도 혼자 확인할 수 있도록,
`system.yaml` 의 숫자만으로 벨트·바닥·스토퍼를 그린다.

**이건 확인용 껍데기지 실제 씬이 아니다.** 없는 것:
로봇 · 카메라 4종 · tf · Action Graph — 전부 트랙 A 소관이다.
여기서 보는 것은 **박스가 벨트에 제대로 앉아서 흘러가는가**뿐이다.

### 벨트는 어떻게 도나

`IsaacConveyor` OmniGraph 노드를 쓰지 않는다. 그 노드가 하는 일은 결국
매 틱 `physxSurfaceVelocity` 를 써 주는 것뿐이라(`verify_belt_speed.py` 참조),
여기서는 **그 속성을 정적 콜라이더에 직접 한 번 건다.** 형상은 안 움직이고
표면만 흐르는 것처럼 동작한다 — 진짜 컨베이어와 같은 원리다.
확장 기능(`isaacsim.asset.gen.conveyor`)이 없어도 돈다.

## 쓰는 순서

    1) File ▸ New  (빈 스테이지)
    2) 이 파일 붙여넣고 Ctrl+Enter          → 벨트·바닥·스토퍼가 생긴다
    3) script_editor_qr_boxes.py 붙여넣고 Ctrl+Enter  → QR 박스가 벨트 위에 뜬다
    4) ▶ Play                                → 박스가 벨트에 내려앉는다 (10mm 낙하)
    → 박스가 +X 로 흘러가 스토퍼(x=10000)에 선다

## 좌표 규약 (CLAUDE.md §4)

    z = 0        도크 바닥 = 트럭 적재함 바닥   ← 야드 지면이 아니다
    z = 800      벨트 상면 (롤러 면)
    z = 810      스폰 지점, 박스 **밑면**
    x = 0~7500   main 컨베이어 / 7500~10000 픽업 구간 / 10000 스토퍼
"""
import os

import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

# ══════════════════════════════════════════════════════════════════════════
DRIVE_BELT = True     # 벨트를 실제로 굴린다 (physxSurfaceVelocity). 속도는 config 값.
SHOW_PALLET = True    # 팔레트 3장을 참고용으로 그린다 (물리 없음)
# ══════════════════════════════════════════════════════════════════════════


# ══════════════════════════════════════════════════════════════════════════
# ★ 저장소 위치 — **여기만 맞으면 된다**
#
#   Script Editor 는 붙여넣기라 `__file__` 이 없고, `os.getcwd()` 가 Isaac 설치
#   폴더다. 그래서 자동 탐색이 실패한다 (`No module named 'packing_common'`).
#   저장소를 다른 데 뒀으면 이 줄만 고칠 것.
# ══════════════════════════════════════════════════════════════════════════
WS_ROOT = "~/palletris_ws"

REL_CONFIG = os.path.join("src", "packing_bringup", "config", "system.yaml")


def load_spec():
    """`system.yaml` 을 읽는다. **`packing_common` 을 import 하지 않는다.**

    ★ 전에는 `packing_common.config.load()` 를 썼는데 Script Editor 에서
      `No module named 'packing_common'` 로 죽었다 — 붙여넣기라 `__file__` 이 없고
      cwd 가 Isaac 설치 폴더라 저장소를 못 찾는다.
      그 함수가 하는 일은 결국 `yaml.safe_load` 하나뿐이라(config.py:load) 여기서 직접 한다.
    """
    import yaml                                  # Isaac 내장 파이썬에 들어 있다

    tried = []
    def cands():
        yield os.path.join(os.path.expanduser(WS_ROOT), REL_CONFIG)
        env = os.environ.get("PALLETRIS_CONFIG")
        if env:
            yield os.path.expanduser(env)
        starts = [os.getcwd()]
        if "__file__" in globals():
            starts.insert(0, os.path.dirname(os.path.realpath(__file__)))
        for start in starts:
            d = start
            for _ in range(8):
                yield os.path.join(d, REL_CONFIG)
                parent = os.path.dirname(d)
                if parent == d:
                    break
                d = parent

    for c in cands():
        if os.path.isfile(c):
            print(f"[INFO] system.yaml: {c}")
            with open(c, encoding="utf-8") as f:
                return yaml.safe_load(f)
        tried.append(c)

    raise RuntimeError(
        "system.yaml 을 못 찾았다.\n"
        f"  → 이 파일 위쪽의 WS_ROOT 를 실제 저장소 경로로 고칠 것 (지금: {WS_ROOT})\n"
        "  찾아본 곳:\n    " + "\n    ".join(list(dict.fromkeys(tried))[:8]))

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

# ── 벨트 구동 — physxSurfaceVelocity ────────────────────────────
#   IsaacConveyor 노드가 매 틱 하는 일을 정적 콜라이더에 한 번 걸어 둔다.
#   형상은 안 움직이고 표면만 흐른다 (verify_belt_speed.py 의 설명과 같은 원리).
def drive(prim, mps):
    """+X 로 mps 만큼 흐르는 표면으로 만든다. 성공하면 True."""
    try:
        api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
        api.CreateSurfaceVelocityEnabledAttr(True)
        api.CreateSurfaceVelocityAttr(Gf.Vec3f(mps, 0.0, 0.0))
        # 월드 기준으로 못 박는다 — 로컬이면 벨트를 회전시켰을 때 방향이 달라진다.
        api.CreateSurfaceVelocityLocalSpaceAttr(False)
        return True
    except Exception as e:                            # noqa: BLE001
        print(f"[WARN] PhysxSurfaceVelocityAPI 실패 ({e}) — 원시 속성으로 재시도")
        try:
            prim.ApplyAPI("PhysxSurfaceVelocityAPI")
            prim.CreateAttribute("physxSurfaceVelocity:surfaceVelocityEnabled",
                                 Sdf.ValueTypeNames.Bool).Set(True)
            prim.CreateAttribute("physxSurfaceVelocity:surfaceVelocity",
                                 Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(mps, 0.0, 0.0))
            return True
        except Exception as e2:                       # noqa: BLE001
            print(f"[ERROR] 벨트 구동 실패: {e2}")
            return False


if DRIVE_BELT:
    mps = CONV["belt_velocity_mps"]
    # ★ 마찰이 없으면 표면이 흘러도 박스를 못 끈다. 벨트에 물리 재질을 준다.
    #   ★ 물리 재질은 **UsdShade.Material 프림**이어야 한다. Scope 에 MaterialAPI 만
    #     걸면 바인딩이 안 풀려서 조용히 기본 마찰로 돈다.
    mat_path = "/World/TestScene/BeltMaterial"
    belt_mat = UsdShade.Material.Define(stage, mat_path)
    pm = UsdPhysics.MaterialAPI.Apply(belt_mat.GetPrim())
    pm.CreateStaticFrictionAttr(0.8)
    pm.CreateDynamicFrictionAttr(0.7)
    pm.CreateRestitutionAttr(0.0)                     # 튀지 않게

    ok = 0
    for name in ("Belt_main", "Belt_pickup"):
        prim = stage.GetPrimAtPath(f"/World/TestScene/{name}")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            belt_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        ok += 1 if drive(prim, mps) else 0
    print(f"[OK] 벨트 {ok}/2 구동 — +X {mps} m/s (physxSurfaceVelocity)")
    print(f"     박스는 스토퍼 x={STOPPER_X}mm 에서 선다.")
else:
    print("[INFO] DRIVE_BELT=False — 벨트가 멈춰 있다.")

print("다음: script_editor_qr_boxes.py 를 붙여넣고 Ctrl+Enter → ▶ Play")
