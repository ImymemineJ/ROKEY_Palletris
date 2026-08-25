"""Isaac Sim Script Editor — 씬 + QR 박스 + 벨트 구동 + 자가진단, 한 파일.

    Isaac Sim GUI(`isaac-sim.sh`) ▸ File ▸ New ▸ Window ▸ Script Editor
    ▸ 붙여넣고 Ctrl+Enter ▸ ▶ Play

    ┌───────────────────────────────────────────────────────────────┐
    │  손댈 곳은 아래 두 줄뿐이다 (바로 밑에 있다)                    │
    │                                                               │
    │    RUN   = "build"   씬·박스·벨트를 만든다   → 그다음 Play     │
    │          = "check"   Play 뒤에 실제로 움직였는지 본다           │
    │                                                               │
    │    DRIVE = "auto"    표면속도 → 안 되면 자동으로 push          │
    │          = "push"    박스 속도를 직접 세팅. 마찰과 무관하게 돈다 │
    └───────────────────────────────────────────────────────────────┘

자세한 설명은 파일 맨 아래 NOTES 를 볼 것.
"""
import os
import random

import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

# ██████████████████████  손댈 곳은 여기 두 줄  ██████████████████████
RUN = "build"             # "build" (만들기)  |  "check" (Play 뒤 확인)
DRIVE = "auto"            # "auto" | "surface" | "push" | "off"
# ████████████████████████████████████████████████████████████████████

SPAWN_COUNT = 8
SHOW_PALLET = True
WS_ROOT = "~/palletris_ws"     # ★ 저장소가 다른 데 있으면 이 줄만 고칠 것

QR_DIR = "/home/rokey/Downloads"
QR_CODES = [
    {"file": "qrcode_390440233_9a0fe13a95a623f42c4ae6443fac57b2.png",
     "payload": "TODO-QR-PAYLOAD-1", "dest": "A"},
    {"file": "qrcode_390443182_98c53133a43e86da032b7dbe25475d15.png",
     "payload": "TODO-QR-PAYLOAD-2", "dest": "B"},
    {"file": "qrcode_390443245_cdaa800198d93d60377bcdd0328e9636.png",
     "payload": "TODO-QR-PAYLOAD-3", "dest": "C"},
]
# ══════════════════════════════════════════════════════════════════════════

BOXES_GROUP = "/World/QR_Boxes"
SCENE_GROUP = "/World/TestScene"
LOOKS_PATH = "/World/Looks"
MM = 1000.0
REL_CONFIG = os.path.join("src", "packing_bringup", "config", "system.yaml")

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No stage open. Use File > New first.")


def load_spec():
    """system.yaml 을 직접 읽는다 (packing_common 을 import 하지 않는다).

    Script Editor 는 붙여넣기라 __file__ 이 없고 cwd 가 Isaac 설치 폴더다.
    """
    import yaml
    tried = []
    cands = [os.path.join(os.path.expanduser(WS_ROOT), REL_CONFIG)]
    env = os.environ.get("PALLETRIS_CONFIG")
    if env:
        cands.append(os.path.expanduser(env))
    d = os.getcwd()
    for _ in range(8):
        cands.append(os.path.join(d, REL_CONFIG))
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    for c in cands:
        if os.path.isfile(c):
            print(f"[INFO] system.yaml: {c}")
            with open(c, encoding="utf-8") as f:
                return yaml.safe_load(f)
        tried.append(c)
    raise RuntimeError("system.yaml not found.\n"
                       f"  -> Fix WS_ROOT at the top of this file (now: {WS_ROOT})\n"
                       "  Tried:\n    " + "\n    ".join(list(dict.fromkeys(tried))[:6]))


CFG = load_spec()
CONV = CFG["conveyor"]
BOX_TYPES = CFG["boxes"]["types"]
MIX = CFG["boxes"]["size_distribution"]
SEED = CFG["boxes"]["random_seed"]
BELT_TOP = CONV["belt_top_z_mm"]
BELT_MPS = CONV["belt_velocity_mps"]
SPAWN_MM = CONV["spawn_point_mm"]
MAIN_LO, MAIN_HI = CONV["stage1"]["main_range_mm"]
PICK_LO, PICK_HI = CONV["pickup_range_mm"]
STOPPER_X = CONV["stopper_x_mm"]
BELT_W = CONV.get("belt_width_mm", 1546)     # yaml 에 없다 — CLAUDE.md 4 가 정본

ASCII_TYPE = {"3호": "No.3", "4호": "No.4", "5호": "No.5"}


def box_x_mm():
    """벨트 위 박스들의 x 좌표(mm). 진단용."""
    grp = stage.GetPrimAtPath(BOXES_GROUP)
    out = []
    if grp.IsValid():
        for prim in grp.GetChildren():
            a = prim.GetAttribute("xformOp:translate")
            t = a.Get() if a and a.IsValid() else None
            if t is not None:
                out.append((prim.GetName(), t[0] * MM, t[2] * MM))
    return sorted(out)


# ══════════════════════════════════════════════════════════════════════════
#  RUN = "check" — Play 뒤에 실제로 움직였는지만 본다
# ══════════════════════════════════════════════════════════════════════════
if RUN == "check":
    prev = globals().get("_spawn_x") or {}
    now = box_x_mm()
    print("\n" + "=" * 62)
    print("  CHECK — did the boxes move?")
    print("=" * 62)
    if not now:
        print("  no boxes found. Run with RUN='build' first.")
    else:
        moved = 0
        print(f"  {'box':10} {'x now (mm)':>12} {'x spawn':>10} {'moved':>9}  z(mm)")
        for name, x, z in now:
            x0 = prev.get(name)
            dx = (x - x0) if x0 is not None else float("nan")
            if x0 is not None and abs(dx) > 5:
                moved += 1
            print(f"  {name:10} {x:12.0f} "
                  f"{('%10.0f' % x0) if x0 is not None else '         ?'} "
                  f"{('%+9.0f' % dx) if x0 is not None else '        ?'}  {z:.0f}")
        print()
        if moved:
            print(f"  ==> {moved}/{len(now)} boxes moved. THE BELT WORKS.")
        else:
            print("  ==> NOTHING MOVED.")
            print("      Fix: scroll to the top, set  DRIVE = \"push\"")
            print("           then Ctrl+Enter, press Stop, press Play again.")
            print("           push sets box velocity directly - it ignores friction.")
    raise SystemExit  # noqa: PLR1722  (Script Editor 에서는 여기서 끝내는 게 맞다)


# ══════════════════════════════════════════════════════════════════════════
#  RUN = "build"
# ══════════════════════════════════════════════════════════════════════════
if not stage.GetPrimAtPath("/World").IsValid():
    UsdGeom.Xform.Define(stage, "/World")
if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
if not stage.GetPrimAtPath("/World/DomeLight").IsValid():
    # 5: 오프스크린 렌더는 GUI 조명 리그를 못 받는다.
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)


def slab(path, x0, x1, y0, y1, top_z, thick, color, collide=True):
    """축정렬 직육면체. x/y 는 범위(mm), top_z 는 **상면** 높이(mm)."""
    c = UsdGeom.Cube.Define(stage, path)
    c.GetSizeAttr().Set(1.0)
    c.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
    c.CreateDisplayColorAttr([color])
    api = UsdGeom.XformCommonAPI(c)
    api.SetTranslate(Gf.Vec3d((x0 + x1) / 2 / MM, (y0 + y1) / 2 / MM,
                              (top_z - thick / 2) / MM))
    api.SetScale(Gf.Vec3f((x1 - x0) / MM, (y1 - y0) / MM, thick / MM))
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


half = BELT_W / 2
slab(f"{SCENE_GROUP}/DockFloor", -1000, 13000, -3000, 3000, 0, 200,
     Gf.Vec3f(0.22, 0.23, 0.26))
slab(f"{SCENE_GROUP}/Belt_main", MAIN_LO, MAIN_HI, -half, half, BELT_TOP, 120,
     Gf.Vec3f(0.30, 0.32, 0.36))
slab(f"{SCENE_GROUP}/Belt_pickup", PICK_LO, PICK_HI, -half, half, BELT_TOP, 120,
     Gf.Vec3f(0.38, 0.40, 0.45))
slab(f"{SCENE_GROUP}/Stopper", STOPPER_X, STOPPER_X + 100, -half, half, 900, 100,
     Gf.Vec3f(0.75, 0.35, 0.25))
if SHOW_PALLET:
    P = CFG["pallet"]
    for zn, org in CFG["zones"]["pallet_origin_world_mm"].items():
        slab(f"{SCENE_GROUP}/Pallet_{zn}", org[0], org[0] + P["width_mm"],
             org[1], org[1] + P["depth_mm"], org[2], P["thickness_mm"],
             Gf.Vec3f(0.55, 0.45, 0.30), collide=False)

# ── QR 이미지 ────────────────────────────────────────────────────
VALID = []
for q in QR_CODES:
    p = os.path.join(QR_DIR, q["file"])
    if not os.path.exists(p):
        print(f"[WARN] QR file not found, skipped: {p}")
        continue
    if any(ch in q["file"] for ch in " ()"):
        print(f"[WARN] space/parenthesis in filename breaks the resolver: {q['file']}")
        continue
    VALID.append(dict(q, path=p))
qr_note = "ok"
if not VALID:
    qr_note = "NO QR IMAGE - boxes will be plain colour"
    print(f"[WARN] {qr_note}. Check QR_DIR / filenames.")
elif any(str(q["payload"]).startswith("TODO") for q in VALID):
    qr_note = "payload placeholders still present"


def qr_material(img_path, idx):
    """UsdPreviewSurface 로 직접 조립 — MDL 비동기 로드 함정 회피."""
    mp = f"{LOOKS_PATH}/QR_Mat_{idx}"
    if stage.GetPrimAtPath(mp).IsValid():
        return UsdShade.Material(stage.GetPrimAtPath(mp))
    mat = UsdShade.Material.Define(stage, mp)
    rd = UsdShade.Shader.Define(stage, f"{mp}/stReader")
    rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    rd.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tx = UsdShade.Shader.Define(stage, f"{mp}/diffuseTex")
    tx.CreateIdAttr("UsdUVTexture")
    tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(img_path))
    tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        rd.ConnectableAPI(), "result")
    tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sf = UsdShade.Shader.Define(stage, f"{mp}/Surface")
    sf.CreateIdAttr("UsdPreviewSurface")
    sf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tx.ConnectableAPI(), "rgb")
    # 하이라이트가 뜨면 QR 디코딩이 깨진다.
    sf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    sf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sf.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.0)
    sf.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(sf.ConnectableAPI(), "surface")
    return mat


TYPE_COLOR = {"3호": Gf.Vec3f(0.35, 0.62, 0.47),
              "4호": Gf.Vec3f(0.92, 0.64, 0.26),
              "5호": Gf.Vec3f(0.30, 0.52, 0.78)}


def phys_material(path, static_f, dynamic_f, restitution=0.0):
    """물리 재질 하나. **UsdShade.Material 프림이어야 한다** — Scope 에 MaterialAPI 만
    걸면 바인딩이 안 풀려 조용히 기본값으로 돈다."""
    m = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(static_f)
    api.CreateDynamicFrictionAttr(dynamic_f)
    api.CreateRestitutionAttr(restitution)
    return m


def bind_phys(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")


# ★★ 여기가 "벨트가 안 끄는" 진짜 원인이었다.
#    UsdPhysics 의 **기본 재질은 마찰이 0** 이다. 박스에 물리 재질을 안 붙이면
#    벨트에만 0.8 을 줘도 접촉쌍의 마찰이 0 이 되어, 표면속도가 흘러도 못 끈다.
#    벨트와 박스 **양쪽에** 붙여야 한다.
BELT_MAT = phys_material(f"{SCENE_GROUP}/BeltMaterial", 0.9, 0.8)
BOX_MAT = phys_material(f"{SCENE_GROUP}/BoxMaterial", 0.6, 0.5)


def spawn(n, box, qr, qr_idx, x_mm, y_mm, yaw):
    w, d, h = box["w_mm"], box["d_mm"], box["h_mm"]
    rp = f"{BOXES_GROUP}/Box_{n:03d}"
    # ★★ config 의 810 은 **밑면**이고 Cube 는 중심 기준이다.
    #    10 의 함정: "박스가 벨트를 뚫고 떨어진다 = 스폰 z 에 h/2 안 더함"
    z_mm = SPAWN_MM[2] + h / 2.0

    root = UsdGeom.Xform.Define(stage, rp)
    UsdGeom.XformCommonAPI(root).SetTranslate(
        Gf.Vec3d(x_mm / MM, y_mm / MM, z_mm / MM))
    UsdGeom.XformCommonAPI(root).SetRotate((0.0, 0.0, yaw))
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(box["mass_g"] / 1000.0)
    px = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    px.CreateSolverPositionIterationCountAttr(16)
    px.CreateSolverVelocityIterationCountAttr(1)

    body = UsdGeom.Cube.Define(stage, f"{rp}/Body")
    body.GetSizeAttr().Set(1.0)
    body.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
    body.CreateDisplayColorAttr([TYPE_COLOR.get(box["name"], Gf.Vec3f(0.8, 0.7, 0.5))])
    UsdGeom.XformCommonAPI(body).SetScale(Gf.Vec3f(w / MM, d / MM, h / MM))
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    bind_phys(body.GetPrim(), BOX_MAT)      # ★ 없으면 마찰 0 이라 벨트가 못 끈다

    if qr is not None:
        mesh = UsdGeom.Mesh.Define(stage, f"{rp}/QR_Face")
        mesh.CreateDoubleSidedAttr(True)
        fh = min(w, d) * 0.9 / 2.0 / MM
        z = (h / 2.0 + 2.0) / MM
        mesh.CreatePointsAttr([Gf.Vec3f(-fh, -fh, z), Gf.Vec3f(fh, -fh, z),
                               Gf.Vec3f(fh, fh, z), Gf.Vec3f(-fh, fh, z)])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateExtentAttr([Gf.Vec3f(-fh, -fh, z), Gf.Vec3f(fh, fh, z)])
        mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        stv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        stv.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
            qr_material(qr["path"], qr_idx))

    prim = root.GetPrim()
    for k, v, t in (("palletris:box_type", box["name"], Sdf.ValueTypeNames.String),
                    ("palletris:qr_payload", str(qr["payload"]) if qr else "",
                     Sdf.ValueTypeNames.String),
                    ("palletris:dest", qr["dest"] if qr else "",
                     Sdf.ValueTypeNames.String),
                    ("palletris:size_w_mm", int(w), Sdf.ValueTypeNames.Int),
                    ("palletris:size_d_mm", int(d), Sdf.ValueTypeNames.Int),
                    ("palletris:size_h_mm", int(h), Sdf.ValueTypeNames.Int)):
        prim.CreateAttribute(k, t).Set(v)
    return dict(box_id=f"Box_{n:03d}", t=ASCII_TYPE.get(box["name"], box["name"]),
                x=round(x_mm), y=round(y_mm), yaw=round(yaw, 1),
                dest=qr["dest"] if qr else "-",
                payload=qr["payload"] if qr else "-")


# ── 스폰 ─────────────────────────────────────────────────────────
UsdGeom.Scope.Define(stage, BOXES_GROUP)
rng = random.Random(SEED)
yj = CFG["boxes"].get("spawn_y_jitter_mm", 150)      # yaml 에 아직 없다 (15 미결)
wj = CFG["boxes"].get("spawn_yaw_jitter_deg", 15)
spacing = BELT_MPS * CFG["boxes"]["stage1"]["spawn_interval_s"] * MM
usable = MAIN_HI - SPAWN_MM[0]
narrowed = False
if SPAWN_COUNT > 1 and spacing * (SPAWN_COUNT - 1) > usable:
    spacing = usable / (SPAWN_COUNT - 1)
    narrowed = True

rows = []
for i in range(SPAWN_COUNT):
    b = rng.choices(BOX_TYPES, MIX)[0]
    qi = rng.randrange(len(VALID)) if VALID else None
    rows.append(spawn(i, b, VALID[qi] if VALID else None, qi,
                      SPAWN_MM[0] + spacing * i,
                      SPAWN_MM[1] + rng.uniform(-yj, yj),
                      CFG["boxes"]["spawn_yaw_deg"] + rng.uniform(-wj, wj)))

_spawn_x = {r["box_id"]: r["x"] for r in rows}       # RUN='check' 가 쓴다

# ── 벨트 구동 ────────────────────────────────────────────────────
_sub = globals().get("_belt_sub")
if _sub is not None:
    try:
        _sub.unsubscribe()
    except Exception:                                # noqa: BLE001
        pass
_belt_sub = None
drive_report = []


def try_surface(prim):
    """표면속도를 걸고 **되읽어 확인**한다."""
    try:
        api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
        api.CreateSurfaceVelocityEnabledAttr(True)
        api.CreateSurfaceVelocityAttr(Gf.Vec3f(BELT_MPS, 0.0, 0.0))
        try:
            api.CreateSurfaceVelocityLocalSpaceAttr(False)
        except Exception:                            # noqa: BLE001
            pass
    except Exception as e:                           # noqa: BLE001
        drive_report.append(f"surface: {type(e).__name__}: {e}")
        return False
    a = prim.GetAttribute("physxSurfaceVelocity:surfaceVelocity")
    got = a.Get() if a and a.IsValid() else None
    if got is None:
        drive_report.append("surface: attribute did not stick")
        return False
    drive_report.append(f"surface: {prim.GetName()} = {tuple(round(v, 3) for v in got)}")
    return True


def start_push():
    """물리 스텝마다 벨트 위 박스의 +X 속도를 맞춘다 (보험).

    ★ USD 속성 쓰기가 Fabric 때문에 안 먹는 판이 있어 isaacsim.core 를 먼저 쓴다.
    """
    setter = None
    state = {"logged": False, "errors": 0}

    try:
        from isaacsim.core.prims import RigidPrim         # Isaac 5.x
        import numpy as np
        rp = RigidPrim(f"{BOXES_GROUP}/Box_.*")

        # ★ 판마다 API 가 다르다. 있는 것을 쓴다.
        #   - get_linear_velocities() -> (N, 3)
        #   - get_velocities()        -> (N, 6)  선속도 0:3 · 각속도 3:6
        #   ⚠ 2026-08-25: get_velocities()[0] 로 첫 행만 집어 1차원으로 만들어 놓고
        #     v[:, 0] 을 했다가 "too many indices for array" 로 매 스텝 터졌다.
        #     반환값은 이미 (N, k) 다 — 인덱싱하지 말 것.
        lin = hasattr(rp, "get_linear_velocities") and hasattr(rp, "set_linear_velocities")

        def setter(_):                                    # noqa: E306
            try:
                v = rp.get_linear_velocities() if lin else rp.get_velocities()
                if v is None:
                    return
                v = np.asarray(v)
                if v.ndim == 1:                           # 박스가 하나뿐인 경우
                    v = v.reshape(1, -1)
                if not state["logged"]:
                    state["logged"] = True
                    print(f"[INFO] push: velocity array shape {v.shape} "
                          f"({'linear' if lin else 'lin+ang'})")
                v[:, 0] = BELT_MPS                        # x 만 고정, 나머지는 물리에 맡긴다
                (rp.set_linear_velocities if lin else rp.set_velocities)(v)
            except Exception as e:                        # noqa: BLE001
                state["errors"] += 1
                if state["errors"] <= 3:                  # 매 스텝 도배하지 않는다
                    print(f"[ERROR] push setter: {type(e).__name__}: {e}")

        drive_report.append(
            f"push: RigidPrim ({'set_linear_velocities' if lin else 'set_velocities'})")
    except Exception as e1:                               # noqa: BLE001
        drive_report.append(f"push: RigidPrim unavailable ({type(e1).__name__}: {e1})")

        def setter(_):                                    # noqa: E306
            grp = stage.GetPrimAtPath(BOXES_GROUP)
            if not grp.IsValid():
                return
            for prim in grp.GetChildren():
                va = prim.GetAttribute("physics:velocity")
                if va and va.IsValid():
                    v = va.Get() or Gf.Vec3f(0, 0, 0)
                    va.Set(Gf.Vec3f(BELT_MPS, v[1], v[2]))
        drive_report.append("push: raw USD physics:velocity")

    try:
        import omni.physx
        return omni.physx.get_physx_interface().subscribe_physics_step_events(setter)
    except Exception as e:                                # noqa: BLE001
        drive_report.append(f"push: subscribe failed ({e})")
        return None


mode_used = "off"
if DRIVE != "off":
    ok = 0
    if DRIVE in ("auto", "surface"):
        for nm in ("Belt_main", "Belt_pickup"):
            pr = stage.GetPrimAtPath(f"{SCENE_GROUP}/{nm}")
            bind_phys(pr, BELT_MAT)
            ok += 1 if try_surface(pr) else 0
        if ok == 2:
            mode_used = "surface"
    if DRIVE == "push" or (DRIVE == "auto" and ok < 2):
        _belt_sub = start_push()
        if _belt_sub is not None:
            mode_used = "push"

# ══════════════════════════════════════════════════════════════════════════
#  진단 — 여기만 읽으면 된다
# ══════════════════════════════════════════════════════════════════════════
print()
print(f"{'box_id':10} {'type':6} {'Px(mm)':>8} {'Py(mm)':>8} {'yaw':>7}  {'dest':4} payload")
for r in rows:
    print(f"{r['box_id']:10} {r['t']:6} {r['x']:8d} {r['y']:8d} {r['yaw']:6.1f}d  "
          f"{r['dest']:4} {r['payload']}")

print()
print("=" * 62)
print("  DIAGNOSIS")
print("=" * 62)
print(f"  scene    : floor z=0 | belt top z={BELT_TOP} w={BELT_W} | "
      f"main {MAIN_LO}-{MAIN_HI} | pickup {PICK_LO}-{PICK_HI} | stopper {STOPPER_X}")
print(f"  boxes    : {len(rows)} spawned | bottom z={SPAWN_MM[2]} "
      f"(={BELT_TOP}+10 -> 10mm drop) | spacing {spacing / MM:.2f} m"
      + (" (narrowed to fit belt)" if narrowed else ""))
print(f"  qr       : {len(VALID)} image(s) | {qr_note}")
print("  friction : belt 0.9/0.8  box 0.6/0.5  (UsdPhysics default is 0 - "
      "without this the belt cannot drag anything)")
print(f"  DRIVE    : {mode_used.upper()}   (requested '{DRIVE}', {BELT_MPS} m/s +X)")
for line in drive_report:
    print(f"             {line}")
print("=" * 62)
if mode_used == "off":
    print("  !! belt is NOT driven. Boxes will just sit there.")
else:
    print("  NEXT: press Play. Boxes should slide +X and stop at the stopper.")
    print("        Then set RUN='check' at the top and Ctrl+Enter to verify.")


# ══════════════════════════════════════════════════════════════════════════
#  NOTES
# ══════════════════════════════════════════════════════════════════════════
#
# ## 벨트를 어떻게 굴리나
#
# `IsaacConveyor` OmniGraph 노드를 쓰지 않는다. 그 노드가 하는 일은 결국 매 틱
# `physxSurfaceVelocity` 를 써 주는 것뿐이라(`verify_belt_speed.py` 참조),
# 여기서는 그 속성을 정적 콜라이더에 한 번 건다. 형상은 안 움직이고 표면만 흐른다.
#
# ★★ 표면속도는 **마찰로** 끈다. 그런데 `UsdPhysics` 의 **기본 재질은 마찰이 0** 이라,
#    박스에 물리 재질을 안 붙이면 벨트에만 0.9 를 줘도 접촉쌍 마찰이 0 이 되어
#    표면이 흘러도 박스가 그대로 서 있다. 2026-08-25 에 실제로 이 증상을 겪었다 —
#    표면속도는 (0.5,0,0) 으로 잘 걸렸는데 박스가 1mm 도 안 움직였다.
#    그래서 **벨트와 박스 양쪽에** 재질을 붙인다 (BELT_MAT / BOX_MAT).
#
# 그래도 안 돌면 `DRIVE = "push"` — 물리 스텝마다 박스 속도를 직접 세팅한다.
# Isaac 은 Fabric 을 쓰므로 USD 속성 쓰기가 시뮬레이션에 안 먹는 판이 있어,
# `isaacsim.core.prims.RigidPrim` 을 먼저 시도하고 안 되면 원시 USD 로 내려간다.
#
# ## 좌표 규약 (CLAUDE.md §4)
#
#     z = 0        도크 바닥 = 트럭 적재함 바닥   ← 야드 지면이 아니다
#     z = 800      벨트 상면 (롤러 면)
#     z = 810      스폰 지점, 박스 **밑면**  (Cube 는 중심 기준이라 h/2 를 더한다)
#     x = 0~7500   main / 7500~10000 픽업 / 10000 스토퍼
#
# 확인 요령: 3호(h=210)가 벨트에 앉으면 중심 z 가 800 + 105 = **905** 여야 한다.
