"""Isaac Sim Script Editor — **이미 스폰된 박스에 QR 텍스처를 붙인다**.

    김학민의 씬(`test_3.usd` 등)을 열고 ▶ Play 로 박스가 벨트에 올라온 뒤,
    Script Editor 에 붙여넣고 **Ctrl+Enter**. 몇 번을 돌려도 안전하다
    (이미 붙은 박스는 건너뛴다).

## 왜 이 모양인가

씬은 이미 QR **배송지 분류**를 한다 — `by_dest` (SEOUL→A · BUSAN→B · DAEJEON→C).
다만 커밋 `215a28c` 이 이렇게 적어 뒀다:

    ⚠ 실제 QR 텍스처·디코딩은 아직 없다. 지금은 씬의 ground truth 로 라우팅한다.

즉 **비어 있는 건 QR 그림 한 장뿐**이다. 그래서 이 스크립트는
박스를 새로 스폰하지 않고, `spawner.py` 가 이미 만든 `/World/boxes/<box_id>` 를
찾아 **`box_dest` 를 읽고 그에 맞는 QR 을 윗면에 얹는다.**

  · 박스 프림 경로 · 속성 이름 전부 `spawner.py` 규약을 그대로 따른다
    (`box_id` · `box_type` · `box_color` · `box_dest` — 접두사 없음)
  · 배송지 값은 `test_3.yaml` 의 `boxes.dests` 와 같아야 한다

## payload 는 실제로 디코딩해서 채웠다 (2026-08-25)

`cv2.wechat_qrcode` 로 세 장을 직접 읽었다. 아래 값이 그 결과다 —
**추측이 아니라 실측이므로 그대로 두면 된다.**

  ⚠ 다만 이 QR 들은 생성기(me-qr.com) 가 만든 **단축링크**라 내용이
    배송지와 무관하다. 지금은 `QR_BY_DEST` 표가 링크↔배송지를 이어주므로
    통합 테스트에 지장이 없다. 최종 발표용으로는 배송지 문자열
    ("SEOUL" 등) 을 직접 인코딩한 QR 로 바꾸는 편이 낫다 — 링크보다
    짧아서 모듈이 성기고, 표 없이도 뜻이 통한다.

QR 을 새로 만들면 `payload` 도 같이 고칠 것. 여기가 틀리면 인식이 읽은
값과 씬의 `box_dest` 가 안 맞아 `VerifyBox` 가 항상 불일치를 낸다.
"""
import os

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdShade

# ██████████████████████████  설정  ██████████████████████████████████
QR_DIR = "/home/rokey/Downloads"

# 배송지 -> QR 이미지. payload 는 **그 이미지를 디코딩하면 나오는 문자열**이다.
QR_BY_DEST = {
    "SEOUL":   {"file": "qrcode_390440233_9a0fe13a95a623f42c4ae6443fac57b2.png",
                "payload": "https://q.me-qr.com/0wfauk13"},
    "BUSAN":   {"file": "qrcode_390443182_98c53133a43e86da032b7dbe25475d15.png",
                "payload": "https://q.me-qr.com/n7gnvazl"},
    "DAEJEON": {"file": "qrcode_390443245_cdaa800198d93d60377bcdd0328e9636.png",
                "payload": "https://q.me-qr.com/bv4lums8"},
}

BOX_ROOT = "/World/boxes"      # ★ spawner.py 규칙 5. **소문자**다
LOOKS_ROOT = "/World/Looks"
FACE_RATIO = 0.9               # QR 한 변 = 박스 짧은 변의 이 비율
LIFT_MM = 2.0                  # 윗면에서 이만큼 띄운다 (z-fighting 방지)
REDO = False                   # True 면 이미 붙은 것도 다시 만든다
# ★ 스포너는 박스를 8 초 간격으로 하나씩 내보낸다 (test_3.yaml spawn_interval_s).
#   WATCH 를 켜 두면 새로 나온 박스에도 계속 붙는다. 끄면 지금 있는 것만 1 회.
WATCH = True
WATCH_PERIOD_S = 1.0           # 이 주기로 새 박스가 있는지 본다
# ████████████████████████████████████████████████████████████████████

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No stage open.")

root = stage.GetPrimAtPath(BOX_ROOT)
if not root.IsValid():
    raise RuntimeError(
        f"{BOX_ROOT} not found.\n"
        "  -> Open the scene and press Play so the spawner creates boxes first.\n"
        "  -> Path is lower-case 'boxes' (spawner.py rule 5).")

# ── QR 이미지 검증 ───────────────────────────────────────────────
ready, notes = {}, []
for dest, q in QR_BY_DEST.items():
    p = os.path.join(QR_DIR, q["file"])
    if not os.path.exists(p):
        notes.append(f"[WARN] {dest}: file not found -> {p}")
        continue
    if any(ch in q["file"] for ch in " ()"):
        # Omniverse 에셋 리졸버가 이스케이프 안 된 공백/괄호에서 실패한다.
        notes.append(f"[WARN] {dest}: space/parenthesis in filename -> rename it")
        continue
    if str(q["payload"]).startswith("TODO"):
        notes.append(f"[WARN] {dest}: payload is still a placeholder")
    ready[dest] = dict(q, path=p)
for n in notes:
    print(n)
if not ready:
    raise RuntimeError("No usable QR image. Check QR_DIR and the filenames.")


def qr_material(dest):
    """UsdPreviewSurface 로 직접 조립한다.

    ★ MDL(OmniPBR) 은 셰이더 프림이 **비동기로** 생겨서, 만든 직후 텍스처를
      설정하면 조용히 건너뛰어진다. UsdPreviewSurface 는 순수 USD 라 100% 동기다.
    """
    mp = f"{LOOKS_ROOT}/QR_{dest}"
    if stage.GetPrimAtPath(mp).IsValid():
        return UsdShade.Material(stage.GetPrimAtPath(mp))
    mat = UsdShade.Material.Define(stage, mp)
    rd = UsdShade.Shader.Define(stage, f"{mp}/stReader")
    rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    rd.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tx = UsdShade.Shader.Define(stage, f"{mp}/diffuseTex")
    tx.CreateIdAttr("UsdUVTexture")
    tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(ready[dest]["path"]))
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
    # ★ roughness 1 · specular 0 — 하이라이트가 뜨면 카메라가 QR 을 못 읽는다.
    sf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    sf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sf.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.0)
    sf.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(sf.ConnectableAPI(), "surface")
    return mat


def local_bbox(prim):
    """프림의 로컬 바운드 (m). 박스 크기와 윗면 z 를 여기서 얻는다.

    ★ 씬의 박스는 Mesh 이고 종류마다 크기가 다르다. 속성(box_type)으로 표를
      찾는 대신 **실제 형상**을 재는 편이 안전하다 — 스케일이 걸려 있어도 맞는다.
    """
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    r = cache.ComputeLocalBound(prim).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


def attach_pass(verbose=True):
    """지금 씬에 있는 박스를 훑어 QR 이 없는 것에만 붙인다. 여러 번 불러도 안전하다."""
    made = skipped = nodest = 0
    rows = []
    for prim in root.GetChildren():
        a = prim.GetAttribute("box_dest")
        dest = a.Get() if a and a.IsValid() else None
        if not dest:
            nodest += 1
            continue
        if dest not in ready:
            rows.append((prim.GetName(), dest, "no QR for this dest"))
            continue

        face_path = f"{prim.GetPath()}/QR_Face"
        if stage.GetPrimAtPath(face_path).IsValid() and not REDO:
            skipped += 1
            continue

        lo, hi = local_bbox(prim)
        w, d = hi[0] - lo[0], hi[1] - lo[1]
        if w <= 0 or d <= 0:
            rows.append((prim.GetName(), dest, "zero-size bbox, skipped"))
            continue
        cx, cy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
        z = hi[2] + LIFT_MM / 1000.0
        fh = min(w, d) * FACE_RATIO / 2.0

        mesh = UsdGeom.Mesh.Define(stage, face_path)
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreatePointsAttr([Gf.Vec3f(cx - fh, cy - fh, z), Gf.Vec3f(cx + fh, cy - fh, z),
                               Gf.Vec3f(cx + fh, cy + fh, z), Gf.Vec3f(cx - fh, cy + fh, z)])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateExtentAttr([Gf.Vec3f(cx - fh, cy - fh, z), Gf.Vec3f(cx + fh, cy + fh, z)])
        mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        stv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        stv.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(qr_material(dest))

        # ★ 콜라이더를 걸지 않는다. QR 면은 **그림일 뿐**이라 물리에 끼면
        #   흡착 판정과 적재 높이가 2mm 씩 어긋난다.

        # 인식이 맞혔는지 대조할 값. 씬의 ground truth 옆에 나란히 둔다.
        prim.CreateAttribute("box_qr_payload", Sdf.ValueTypeNames.String).Set(
            ready[dest]["payload"])
        made += 1
        rows.append((prim.GetName(), dest, f"{w * 1000:.0f}x{d * 1000:.0f}mm "
                                           f"top z={hi[2] * 1000:.0f}mm"))

    if verbose:
        print()
        print("=" * 66)
        print("  ATTACH QR")
        print("=" * 66)
        for name, dest, note in rows[:20]:
            print(f"  {name:22} {dest:9} {note}")
        if len(rows) > 20:
            print(f"  ... and {len(rows) - 20} more")
        print()
        print(f"  attached {made} | already had one {skipped} | no box_dest {nodest}")
        print(f"  QR images ready: {sorted(ready)}")
        print("=" * 66)
    elif made:
        # 감시 모드에서는 새로 붙은 것만 한 줄씩. 조용히 지나가면 도는지 알 수 없다.
        for name, dest, note in rows:
            if "mm" in note:
                print(f"  [QR] {name:22} {dest:9} {note}")
    return made, skipped, nodest


# ── 1 회 실행 ─────────────────────────────────────────────────────
made, skipped, nodest = attach_pass()

if made == 0 and skipped == 0:
    print("  Nothing attached. Is the scene playing and are boxes spawned?")
else:
    print("  Look at a box from above - the QR should be crisp, no highlight.")
    print("  Each box now also carries box_qr_payload for VerifyBox.")

# ── 감시 모드 ─────────────────────────────────────────────────────
# ★ 왜 필요한가: 스포너는 박스를 **한 번에 다 만들지 않는다**
#   (`test_3.yaml  spawn_interval_s: 8`). 그래서 위의 1 회 실행은 그 순간
#   벨트에 있던 박스만 잡는다. 12 개짜리 데모라면 96 초에 걸쳐 나오므로
#   거의 다 놓친다. 이 구독이 새로 생긴 박스에 계속 붙여 준다.
#
#   앱 업데이트 스트림에 건다 — 물리 스텝 콜백이 아니라서 메인(물리) 스레드를
#   붙들지 않는다 (CLAUDE.md §9).
if WATCH:
    import omni.kit.app

    _S = globals().setdefault("_QR_WATCH", {})
    if _S.get("sub") is not None:
        # 다시 실행하면 옛 구독을 먼저 끊는다. 안 그러면 겹쳐 돌면서
        # 스텝마다 두 번씩 훑는다.
        _S["sub"].unsubscribe()
        _S["sub"] = None
        print("  [watch] previous watcher stopped")

    _S["acc"] = 0.0

    def _on_update(e):
        _S["acc"] += e.payload.get("dt", 0.0)
        if _S["acc"] < WATCH_PERIOD_S:
            return
        _S["acc"] = 0.0
        try:
            attach_pass(verbose=False)
        except Exception as ex:      # 콜백에서 터지면 조용히 죽는다. 찍고 계속.
            print(f"  [watch] error: {ex}")

    _S["sub"] = omni.kit.app.get_app().get_update_event_stream() \
        .create_subscription_to_pop(_on_update, name="palletris_qr_watch")
    print(f"  [watch] ON - checks every {WATCH_PERIOD_S:g}s for newly spawned boxes.")
    print("  [watch] to stop:  _QR_WATCH['sub'].unsubscribe(); _QR_WATCH['sub']=None")
