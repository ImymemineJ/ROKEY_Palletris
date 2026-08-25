"""Isaac Sim Script Editor 용 — **사양대로** QR 박스를 벨트에 스폰한다.

    Isaac Sim GUI ▸ Window ▸ Script Editor 에 통째로 붙여넣고 **Ctrl+Enter**.
    (`isaac-sim.sh` 로 띄운 창이어야 한다. `python.sh --gui` 창에는 Script Editor 가 없다)

## 원본(단독 낙하 테스트) 대비 바뀐 것 — 전부 CLAUDE.md 사양에 맞춘 것이다

  · 박스가 **0.3 m 정육면체가 아니라 3종**이다 — 340×250×210 · 410×310×280 ·
    480×380×340 (mm). 혼입비 0.5 / 0.3 / 0.2. 질량도 config 값(3·5·9 kg).
  · **원점 상공이 아니라 벨트 위**에 스폰한다 — `conveyor.spawn_point_mm` (500, 0, 810).
  · **랜덤 지터** y ±150 mm · yaw ±15° (§4 "동적 대응" 평가 항목).
  · **QR payload 를 기록한다** — §3-1 `BoxInfo.qr_payload` 와 §3-3 `VerifyBox` 가
    그 값을 쓴다. 어느 박스에 뭐가 붙었는지 안 남기면 cam2 검증을 못 한다.
  · **숫자를 코드에 안 박는다** — 전부 `system.yaml` 에서 읽는다 (§4 단위 규칙).

## 원본에서 그대로 가져온 것 (좋은 부분)

  · MDL(OmniPBR) 비동기 로드 함정 회피 → `UsdPreviewSurface` 를 직접 조립.
    `CreateMdlMaterialPrimCommand` 는 셰이더 프림이 늦게 생겨 텍스처 설정이
    조용히 건너뛰어진다. UsdPreviewSurface 는 순수 USD 라 100% 동기다.
  · Cube 는 gprim(리프)이라 자식을 못 가진다 → Xform 루트 밑에 Body 와 QR_Face 를
    형제로 둔다. RigidBodyAPI 는 Xform 에 건다.
  · Cube 의 `extent` 를 크기에 맞춰 갱신 (기본값 ±1 이라 안 하면 바운드가 2 m 로 잡힌다).
  · `roughness 1.0 / specular 0` — QR 에 하이라이트가 뜨면 디코딩이 깨진다.
  · 파일명에 공백/괄호가 있으면 에셋 리졸버가 실패 → 걸러낸다.

## ★ 김학민에게 — 쓰기 전에 두 가지만

  1. **`QR_CODES` 의 `payload` 를 실제 QR 내용으로 채울 것.** 지금은 자리표시자다.
     이미지가 실제로 무엇을 인코딩하는지는 만든 사람만 안다. 여기가 틀리면
     cam2 의 `VerifyBox` 가 항상 불일치를 낸다.
  2. **QR 이미지를 저장소로 옮길 것.** 지금 경로가 `/home/rokey/Downloads/...` 라
     다른 PC 에서 안 돈다. `QR_DIR` 만 바꾸면 되게 해 뒀다.

  스폰 담당은 §12 상 `src/packing_sim/isaac/spawner.py` 다. 통합하면 그쪽으로 간다.
  이 파일은 **QR 텍스처가 제대로 붙는지 눈으로 보는 단독 테스트**다.
"""
import os
import random

import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

# ══════════════════════════════════════════════════════════════════════════
# 1. QR 코드 — payload 와 목적지를 **여기서 짝지어 둔다**
#
#    §3-1 BoxInfo 는 qr_payload 와 dest 를 따로 갖는다. QR 이미지가 실제로
#    무엇을 인코딩하는지 코드가 알 방법이 없으므로 사람이 적어 줘야 한다.
# ══════════════════════════════════════════════════════════════════════════
QR_DIR = "/home/rokey/Downloads"          # ★ 저장소로 옮기면 여기만 바꾸면 된다

QR_CODES = [
    {"file": "qrcode_390440233_9a0fe13a95a623f42c4ae6443fac57b2.png",
     "payload": "TODO-실제-QR-내용-1", "dest": "A"},
    {"file": "qrcode_390443182_98c53133a43e86da032b7dbe25475d15.png",
     "payload": "TODO-실제-QR-내용-2", "dest": "B"},
    {"file": "qrcode_390443245_cdaa800198d93d60377bcdd0328e9636.png",
     "payload": "TODO-실제-QR-내용-3", "dest": "C"},
]

SPAWN_COUNT = 8           # 몇 개 뿌려 볼지 (단독 테스트용. 사양은 stage1.total_count)
MAKE_GROUND = False       # ★ 실제 씬에는 이미 벨트/바닥이 있다. 빈 스테이지에서만 True
LOOKS_PATH = "/World/Looks"
BOXES_GROUP_PATH = "/World/QR_Boxes"

# 종류별 색 — §15 "색 팔레트 공유" 가 아직 미결이라 여기서 임시로 준다.
# 인식 노드와 값을 맞춰야 한다 (스폰 색상 == 인식 색상).
TYPE_COLOR = {
    "3호": Gf.Vec3f(0.35, 0.62, 0.47),
    "4호": Gf.Vec3f(0.92, 0.64, 0.26),
    "5호": Gf.Vec3f(0.30, 0.52, 0.78),
}

# ══════════════════════════════════════════════════════════════════════════
# 2. 사양 읽기 — 숫자를 코드에 박지 않는다 (§4)
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
BOXES = CFG["boxes"]["types"]
MIX = CFG["boxes"]["size_distribution"]
SEED = CFG["boxes"]["random_seed"]                    # 42 — §11 이 변경 금지
CONV = CFG["conveyor"]
SPAWN_MM = CONV["spawn_point_mm"]                     # (500, 0, 810) 박스 **밑면** 중심
BELT_MPS = CONV["belt_velocity_mps"]
INTERVAL_S = CFG["boxes"]["stage1"]["spawn_interval_s"]
MAIN_LO, MAIN_HI = CONV["stage1"]["main_range_mm"]

# ★ 지터 키가 아직 yaml 에 없다 (CLAUDE.md §15 미결 — 김학민).
#   없으면 §4 에 적힌 값으로 돈다. 키가 생기면 자동으로 그 값을 쓴다.
def _spawn_key(name, doc_default):
    v = CFG["boxes"].get(name)
    if v is None:
        print(f"[WARN] system.yaml 에 boxes.{name} 이 없다 → CLAUDE.md §4 값 {doc_default} 사용.")
        print(f"       (§15 미결: 'spawn_y_jitter_mm · spawn_yaw_jitter_deg yaml + spawner.py 적용')")
        return doc_default
    return v


Y_JITTER_MM = _spawn_key("spawn_y_jitter_mm", 150)
YAW_JITTER_DEG = _spawn_key("spawn_yaw_jitter_deg", 15)

MM = 1000.0                                            # mm -> m (§4: Isaac 은 m float)

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("열린 스테이지가 없다. 씬을 먼저 열 것 (File > Open).")

# ── QR 이미지 검증 ────────────────────────────────────────────────
VALID = []
for q in QR_CODES:
    p = os.path.join(QR_DIR, q["file"])
    if not os.path.exists(p):
        print(f"[WARN] 파일 없음, 건너뜀: {p}")
        continue
    if any(c in q["file"] for c in " ()"):
        # Omniverse 에셋 리졸버가 이스케이프 안 된 공백/괄호에서 실패한다.
        print(f"[WARN] 파일명에 공백/괄호 — 텍스처 로드가 실패할 수 있다: {q['file']}")
        print("       → 파일명을 바꾸고 다시 넣을 것. 일단 건너뛴다.")
        continue
    if str(q["payload"]).startswith("TODO"):
        print(f"[WARN] payload 가 아직 자리표시자다: {q['file']}")
        print("       → 실제 QR 내용으로 채우지 않으면 VerifyBox 가 항상 불일치를 낸다.")
    VALID.append(dict(q, path=p))

if not VALID:
    raise RuntimeError("쓸 수 있는 QR 이미지가 없다. QR_DIR / 파일명을 확인할 것.")
print(f"[INFO] QR {len(VALID)}종 · 박스 {len(BOXES)}종 · 시드 {SEED}")

rng = random.Random(SEED)

# ══════════════════════════════════════════════════════════════════════════
# 3. 조명 · 물리 — §5 "오프스크린 렌더는 GUI 조명 리그를 못 받는다"
# ══════════════════════════════════════════════════════════════════════════
if not stage.GetPrimAtPath("/World/DomeLight").IsValid():
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)
    print("[INFO] DomeLight 생성 (intensity 1000).")

if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

if MAKE_GROUND and not stage.GetPrimAtPath("/World/GroundPlane").IsValid():
    # 두꺼운 정적 슬래브. 상면이 z=0 (§4 도크 바닥)에 오도록 절반만큼 내린다.
    g = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
    gs = 20.0
    g.GetSizeAttr().Set(gs)
    g.CreateExtentAttr([Gf.Vec3f(-gs / 2), Gf.Vec3f(gs / 2)])
    g.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.25, 0.28)])
    UsdGeom.XformCommonAPI(g).SetTranslate(Gf.Vec3d(0.0, 0.0, -gs / 2))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())        # RigidBodyAPI 없음 = 정적
    print("[INFO] GroundPlane 생성 (상면 z=0).")

# ══════════════════════════════════════════════════════════════════════════
# 4. QR 재질 — UsdPreviewSurface (전부 동기)
# ══════════════════════════════════════════════════════════════════════════
def get_or_create_qr_mat(img_path, idx):
    mat_path = f"{LOOKS_PATH}/QR_Mat_{idx}"
    if stage.GetPrimAtPath(mat_path).IsValid():
        return UsdShade.Material(stage.GetPrimAtPath(mat_path))

    mat = UsdShade.Material.Define(stage, mat_path)

    reader = UsdShade.Shader.Define(stage, f"{mat_path}/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex = UsdShade.Shader.Define(stage, f"{mat_path}/diffuseTex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(img_path))
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    # roughness 1.0 / specular 0 — 하이라이트가 뜨면 QR 디코딩이 깨진다.
    surf = UsdShade.Shader.Define(stage, f"{mat_path}/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    surf.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.0)
    surf.CreateOutput("surface", Sdf.ValueTypeNames.Token)

    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    return mat


# ══════════════════════════════════════════════════════════════════════════
# 5. 박스 하나 — Xform(리지드바디) ▸ Body(Cube, 비균등 스케일) + QR_Face(Mesh)
# ══════════════════════════════════════════════════════════════════════════
def spawn_box(n, box, qr, qr_idx, x_mm, y_mm, yaw_deg):
    """x/y 는 박스 **중심**, z 는 config 의 밑면 높이에서 h/2 를 더해 놓는다."""
    w, d, h = box["w_mm"], box["d_mm"], box["h_mm"]
    root_path = f"{BOXES_GROUP_PATH}/Box_{n:03d}"

    # ★★ 밑면 → 중심 변환. §10 의 함정: "박스가 벨트를 뚫고 떨어진다 = 스폰 z 에 h/2 안 더함"
    z_mm = SPAWN_MM[2] + h / 2.0

    root = UsdGeom.Xform.Define(stage, root_path)
    UsdGeom.XformCommonAPI(root).SetTranslate(
        Gf.Vec3d(x_mm / MM, y_mm / MM, z_mm / MM))
    UsdGeom.XformCommonAPI(root).SetRotate((0.0, 0.0, yaw_deg))

    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(box["mass_g"] / 1000.0)
    # 스택이 무너지지 않도록 리지드바디별로 건다 (PhysicsScene 에 걸면 무시된다)
    px = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    px.CreateSolverPositionIterationCountAttr(16)
    px.CreateSolverVelocityIterationCountAttr(1)

    # ── 몸통: 단위 Cube 를 w×d×h 로 스케일 ──
    body = UsdGeom.Cube.Define(stage, f"{root_path}/Body")
    body.GetSizeAttr().Set(1.0)
    # ★ extent 는 size 를 따라가지 않는다. 단위 큐브이므로 ±0.5.
    body.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
    body.CreateDisplayColorAttr([TYPE_COLOR.get(box["name"], Gf.Vec3f(0.8, 0.7, 0.5))])
    UsdGeom.XformCommonAPI(body).SetScale(Gf.Vec3f(w / MM, d / MM, h / MM))
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())

    # ── QR 면: 윗면 위 2 mm. **Body 의 형제**라 스케일을 안 받는다 ──
    mesh = UsdGeom.Mesh.Define(stage, f"{root_path}/QR_Face")
    mesh.CreateDoubleSidedAttr(True)
    fh = min(w, d) * 0.9 / 2.0 / MM               # 짧은 변의 90% 를 한 변으로
    z = (h / 2.0 + 2.0) / MM
    mesh.CreatePointsAttr([Gf.Vec3f(-fh, -fh, z), Gf.Vec3f(fh, -fh, z),
                           Gf.Vec3f(fh, fh, z), Gf.Vec3f(-fh, fh, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr([Gf.Vec3f(-fh, -fh, z), Gf.Vec3f(fh, fh, z)])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        get_or_create_qr_mat(qr["path"], qr_idx))

    # ── 인식/DB 가 쓸 값을 프림에 실어 둔다 (§3-1 BoxInfo) ──
    #   USD 커스텀 속성이라 Action Graph 나 다른 스크립트가 그대로 읽을 수 있다.
    prim = root.GetPrim()
    for key, val, typ in (
            ("palletris:box_type", box["name"], Sdf.ValueTypeNames.String),
            ("palletris:qr_payload", str(qr["payload"]), Sdf.ValueTypeNames.String),
            ("palletris:dest", qr["dest"], Sdf.ValueTypeNames.String),
            ("palletris:size_w_mm", int(w), Sdf.ValueTypeNames.Int),
            ("palletris:size_d_mm", int(d), Sdf.ValueTypeNames.Int),
            ("palletris:size_h_mm", int(h), Sdf.ValueTypeNames.Int)):
        prim.CreateAttribute(key, typ).Set(val)

    return dict(box_id=f"Box_{n:03d}", box_type=box["name"],
                qr_payload=qr["payload"], dest=qr["dest"],
                x_mm=round(x_mm), y_mm=round(y_mm), yaw_deg=round(yaw_deg, 1))


# ══════════════════════════════════════════════════════════════════════════
# 6. 스폰 — 벨트를 따라 사양 간격으로 늘어놓는다
# ══════════════════════════════════════════════════════════════════════════
UsdGeom.Scope.Define(stage, BOXES_GROUP_PATH)

# 사양 간격 = 벨트 속도 × 스폰 주기. 12 s · 0.5 m/s → 6 m.
spacing_mm = BELT_MPS * INTERVAL_S * MM
usable_mm = MAIN_HI - SPAWN_MM[0]
if spacing_mm * (SPAWN_COUNT - 1) > usable_mm and SPAWN_COUNT > 1:
    spacing_mm = usable_mm / (SPAWN_COUNT - 1)
    print(f"[INFO] 사양 간격 {BELT_MPS * INTERVAL_S:.1f} m 로는 {SPAWN_COUNT}개가 "
          f"벨트({usable_mm / MM:.1f} m)를 넘는다 → {spacing_mm / MM:.2f} m 로 좁힌다.")

manifest = []
for i in range(SPAWN_COUNT):
    box = rng.choices(BOXES, MIX)[0]
    qr_idx = rng.randrange(len(VALID))
    manifest.append(spawn_box(
        i, box, VALID[qr_idx], qr_idx,
        x_mm=SPAWN_MM[0] + spacing_mm * i,
        y_mm=SPAWN_MM[1] + rng.uniform(-Y_JITTER_MM, Y_JITTER_MM),
        yaw_deg=CFG["boxes"]["spawn_yaw_deg"] + rng.uniform(-YAW_JITTER_DEG, YAW_JITTER_DEG),
    ))

# ── 매니페스트 — 어느 박스에 어느 QR 이 붙었나 (§3-1 qr_payload · dest) ──
print()
print(f"{'box_id':10} {'종류':5} {'Px(mm)':>8} {'Py(mm)':>8} {'yaw':>7}  {'dest':4} payload")
for m in manifest:
    print(f"{m['box_id']:10} {m['box_type']:5} {m['x_mm']:8d} {m['y_mm']:8d} "
          f"{m['yaw_deg']:6.1f}°  {m['dest']:4} {m['qr_payload']}")
print()
print(f"[SUCCESS] {SPAWN_COUNT}개 스폰 — 벨트 상면 z={CONV['belt_top_z_mm']}mm 위, "
      f"밑면 z={SPAWN_MM[2]}mm")
print("          같은 값이 각 프림의 palletris:* 커스텀 속성에도 들어 있다.")
print("          Play 를 눌러 벨트를 타는지 확인할 것.")
