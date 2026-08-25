"""Isaac Sim Script Editor 용 — QR 텍스처 붙은 박스 랜덤 스폰 (단독 테스트).

원본 대비 고친 것:
  1) MDL(OmniPBR) 비동기 로드 함정 회피 → UsdPreviewSurface 를 직접 조립.
     CreateMdlMaterialPrimCommand 는 셰이더 프림이 늦게 생겨서 텍스처 설정이
     조용히 건너뛰어진다. UsdPreviewSurface 는 순수 USD 라 100% 동기다.
  2) Cube 는 gprim(리프)이라 자식을 못 가진다 → Xform 루트 밑에
     Body(Cube) 와 QR_Face(Mesh) 를 형제로 둔다. RigidBodyAPI 는 Xform 에.
  3) 그라운드 콜라이더가 없으면 리지드바디가 무한 낙하한다 → 없으면 만든다.
  4) 경로에 공백/괄호가 있으면 에셋 리졸버가 실패 → 그런 파일은 걸러낸다.
  5) Cube 의 extent 를 size 에 맞춰 갱신 (기본값은 ±1 이다).
"""
# ─────────────────────────────────────────────────────────────────────────────
# ★ 트랙 A(김학민) 인계 메모 — 정정인, 2026-08-25
#
# **이 파일은 단독 테스트다.** Script Editor 에 붙여넣어 QR 텍스처가 제대로 붙는지
# 확인하는 용도이고, 씬에 통합하려면 아래 5개를 맞춰야 한다 (§ = CLAUDE.md).
#
#  1. **QR 이미지 경로가 절대경로다** (`/home/rokey/Downloads/...`).
#     다른 PC 에서 안 돈다. 이미지를 저장소에 넣거나(예: `assets/qr/`) 경로를 인자로.
#  2. **BOX_SIZE 0.3 m 정육면체는 사양이 아니다.** §4 의 박스는 3종이다 —
#     340×250×210 · 410×310×280 · 480×380×340 (mm).
#     §4 단위 규칙: "코드에 숫자를 박지 않는다. 전부 system.yaml 에서 읽는다."
#  3. **어느 박스에 어느 QR 이 붙었는지 안 남긴다.** §3-1 의 `BoxInfo.qr_payload` 와
#     §3-3 의 `VerifyBox(box_id, qr_payload)` 가 그 값을 쓴다.
#     기록이 없으면 cam2 검증(§6 2번)을 할 수가 없다. 스폰할 때 payload 를 같이 내보낼 것.
#  4. **QR 이 목적지를 담아야 한다.** §3-1 `BoxInfo.dest` 는 "A"|"B"|"C" 다.
#     지금은 이미지 3장을 무작위로 고르기만 하고 그 안에 뭐가 들었는지 모른다.
#  5. **스폰 위치가 다르다.** §4 의 스폰 지점은 (500, 0, 810) mm — 벨트 위다.
#     지금은 원점 주변 ±0.8 m 상공이다 (단독 테스트라 당연하다).
#
# 잘 되어 있는 것 — 그대로 가져가면 좋겠다:
#   · `RANDOM_SEED = 42` (§11 이 변경 금지로 못 박은 값과 같다)
#   · DomeLight 생성 — §5 의 "오프스크린 렌더는 GUI 조명 리그를 못 받는다" 함정 회피
#   · Cube extent 갱신 — 안 하면 바운드가 2 m 로 잡힌다
#   · roughness 1.0 / specular 0 — QR 에 하이라이트가 뜨면 디코딩이 깨진다
#
# 스폰 담당은 §12 상 `src/packing_sim/isaac/spawner.py` 다. 통합하면 그쪽으로 간다.
# ─────────────────────────────────────────────────────────────────────────────
import os
import random

import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

# ==========================================
# 1. 설정
# ==========================================
QR_IMAGE_PATHS = [
    "/home/rokey/Downloads/qrcode_390440233_9a0fe13a95a623f42c4ae6443fac57b2.png",
    "/home/rokey/Downloads/qrcode_390443182_98c53133a43e86da032b7dbe25475d15.png",
    "/home/rokey/Downloads/qrcode_390443245_cdaa800198d93d60377bcdd0328e9636.png",
]

SPAWN_COUNT = 8
BOX_SIZE = 0.3            # m (스테이지가 미터 단위라는 전제)
LOOKS_PATH = "/World/Looks"
BOXES_GROUP_PATH = "/World/QR_Boxes"
MAKE_GROUND = True        # 이미 바닥이 있는 씬에 붙일 땐 False
RANDOM_SEED = 42          # 재현성 — CLAUDE.md §4 와 같은 값

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("열린 스테이지가 없다. File > New 로 스테이지부터 만들 것.")

# ── 경로 검증: 없는 파일 + 공백/괄호 있는 파일은 뺀다 ──────────────
VALID_PATHS = []
for p in QR_IMAGE_PATHS:
    if not os.path.exists(p):
        print(f"[WARN] 파일 없음, 건너뜀: {p}")
        continue
    base = os.path.basename(p)
    if any(c in base for c in " ()"):
        # Omniverse 에셋 리졸버가 이스케이프 안 된 공백/괄호에서 실패한다.
        print(f"[WARN] 파일명에 공백/괄호가 있어 텍스처 로드가 실패할 수 있다: {base}")
        print("       → 파일명을 바꾸고 다시 넣을 것. 일단 건너뛴다.")
        continue
    VALID_PATHS.append(p)

if not VALID_PATHS:
    raise RuntimeError("쓸 수 있는 QR 이미지가 하나도 없다. 경로/파일명을 확인할 것.")
print(f"[INFO] QR 이미지 {len(VALID_PATHS)} 개 사용.")

random.seed(RANDOM_SEED)

# ==========================================
# 2. 조명 — 오프스크린 렌더는 GUI 조명 리그를 못 받는다
# ==========================================
if not stage.GetPrimAtPath("/World/DomeLight").IsValid():
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)

# ==========================================
# 3. 물리 씬 + 바닥 — 없으면 박스가 무한 낙하한다
# ==========================================
if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

if MAKE_GROUND and not stage.GetPrimAtPath("/World/GroundPlane").IsValid():
    # 두꺼운 정적 슬래브로 만든다. 상면이 z=0 에 오도록 절반만큼 내린다.
    # (평면 콜라이더보다 이쪽이 어느 물리 백엔드에서도 확실하다)
    g = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
    gs = 10.0
    g.GetSizeAttr().Set(gs)
    g.CreateExtentAttr([Gf.Vec3f(-gs / 2), Gf.Vec3f(gs / 2)])
    g.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.25, 0.28)])
    UsdGeom.XformCommonAPI(g).SetTranslate(Gf.Vec3d(0.0, 0.0, -gs / 2))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())   # RigidBodyAPI 없음 = 정적
    print("[INFO] GroundPlane 생성 (상면 z=0).")

# ==========================================
# 4. UsdPreviewSurface 재질 — 전부 동기, 비동기 함정 없음
# ==========================================
def get_or_create_qr_mat(stage, img_path, idx):
    mat_path = f"{LOOKS_PATH}/QR_Mat_{idx}"
    if stage.GetPrimAtPath(mat_path).IsValid():
        return UsdShade.Material(stage.GetPrimAtPath(mat_path))

    mat = UsdShade.Material.Define(stage, mat_path)

    # st(UV) 리더 — 메시의 "st" primvar 를 읽어 온다
    reader = UsdShade.Shader.Define(stage, f"{mat_path}/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    # 텍스처 노드
    tex = UsdShade.Shader.Define(stage, f"{mat_path}/diffuseTex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(img_path))
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    # 서피스 — roughness 1.0 / metallic 0 으로 반사광을 없앤다.
    # QR 은 카메라가 읽어야 하므로 하이라이트가 뜨면 디코딩이 깨진다.
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


# ==========================================
# 5. 박스 하나 — Xform(리지드바디) ▸ Body(Cube) + QR_Face(Mesh)
# ==========================================
def spawn_single_qr_box(stage, box_id, pos, rot_z_deg, qr_img_path, qr_idx):
    root_path = f"{BOXES_GROUP_PATH}/Box_{box_id:03d}"

    # ── 루트 Xform 에 리지드바디. Cube 에 직접 걸고 자식을 달면 안 된다 ──
    root = UsdGeom.Xform.Define(stage, root_path)
    UsdGeom.XformCommonAPI(root).SetTranslate(pos)
    UsdGeom.XformCommonAPI(root).SetRotate((0.0, 0.0, rot_z_deg))

    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(1.0)
    # 스택이 무너지지 않도록 리지드바디별로 건다 (PhysicsScene 에 걸면 무시된다)
    px = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    px.CreateSolverPositionIterationCountAttr(16)
    px.CreateSolverVelocityIterationCountAttr(1)

    # ── 몸통 ──
    half = BOX_SIZE / 2.0
    body = UsdGeom.Cube.Define(stage, f"{root_path}/Body")
    body.GetSizeAttr().Set(BOX_SIZE)
    # ★ extent 는 size 를 따라가지 않는다. 기본값 ±1 을 그대로 두면
    #   바운드/컬링이 2 m 박스로 잡힌다.
    body.CreateExtentAttr([Gf.Vec3f(-half, -half, -half),
                           Gf.Vec3f(half, half, half)])
    body.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.7, 0.45)])
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())

    # ── QR 면 (윗면 위 2 mm) ──
    mesh = UsdGeom.Mesh.Define(stage, f"{root_path}/QR_Face")
    mesh.CreateDoubleSidedAttr(True)

    fh = (BOX_SIZE * 0.95) / 2.0
    z = half + 0.002
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

    mat = get_or_create_qr_mat(stage, qr_img_path, qr_idx)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)


# ==========================================
# 6. 스폰 루프
# ==========================================
UsdGeom.Scope.Define(stage, BOXES_GROUP_PATH)

for i in range(SPAWN_COUNT):
    qr_idx = random.randrange(len(VALID_PATHS))
    spawn_single_qr_box(
        stage, i,
        Gf.Vec3d(random.uniform(-0.8, 0.8),
                 random.uniform(-0.8, 0.8),
                 random.uniform(0.6, 1.5)),
        random.uniform(0.0, 360.0),
        VALID_PATHS[qr_idx], qr_idx,
    )

print(f"[SUCCESS] QR 박스 {SPAWN_COUNT} 개 스폰 완료. Play 를 눌러 낙하 확인.")
