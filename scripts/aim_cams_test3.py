"""픽업·라인 카메라를 **벨트 위 박스가 보이게** 놓는다 (test_3).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/aim_cams_test3.py

## 무엇을 하나

  [1] 원점(0,0,0)에 놓다 만 리그를 지운다
  [2] 리그 5 대를 **정확히 아래(-Z)** 로 향하게 돌린다
  [3] XY 를 그 카메라가 봐야 할 자리로 옮긴다 (좌표는 설정에서 유도한다)
  [4] 저장 · 벨트면 조준점과 시야를 다시 재서 검증

## 왜 재고 고치기(measure-and-correct)인가

rsd455 는 **리그**다. `/World/rsd455_a/RSD455/Camera_*` 처럼 렌즈가 두 단계
아래에 있고 그 사이에 자체 변환이 끼어 있다. 그래서 리그의 `translate` 에
원하는 좌표를 그냥 써넣으면 **렌즈는 딴 데 간다.** 렌즈의 월드 좌표를 재서
차이만큼 리그를 미는 것을 반복한다 (`setup_test3_scene.py` [8] 과 같은 식).

회전은 반대로 **한 번에 계산된다.** 리그->렌즈의 고정 회전을 R_in 이라 하면
렌즈 월드 회전 = R_rig · R_in 이므로, 렌즈를 단위행렬(= 광축이 월드 -Z,
이미지 위쪽이 +Y)로 만들려면 R_rig = R_in^-1 이다.

  ★ 이미지 위쪽이 +Y 인 것이 맞다 — 지선이 +Y 로 흐른다.

## 어디로 옮기나 (숫자를 박지 않는다)

  픽업 카메라 A/B/C : X = 지선 중심 (`zones.pickup_cross_mm` 의 중앙)
                      Y = 스토퍼 - PL_PICK_BACK_MM (박스 중심이 서는 자리)
  라인 카메라 1/2   : X 는 **그대로 둔다** (놓으신 자리가 뜻이 있다)
                      Y = 본선 중심 (`conveyor.spawn_point_mm[1]`)

  높이(z)는 둘 다 **건드리지 않는다** — 어느 높이든 아래를 보면 벨트가 잡힌다.
  시야가 모자라면 [4] 가 알려 준다.

    PL_AIM_DRY=1        저장하지 않고 계산만 보여 준다
    PL_PICK_BACK_MM     스토퍼에서 박스 중심까지 (기본 150)
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import omni.usd                                                # noqa: E402
from pxr import Gf, Usd, UsdGeom                               # noqa: E402

import _kitexit                                                # noqa: E402
import _paths                                                  # noqa: E402

STAGE = os.environ.get(
    "PL_STAGE", "/home/rokey/isaac_scenes/Collected_test_3/test_3.usd")
DRY = os.environ.get("PL_AIM_DRY", "0") not in ("", "0", "false", "no")
BACK_MM = float(os.environ.get("PL_PICK_BACK_MM", "150"))
LENS_PREF = "Color"          # 리그의 대표 렌즈. RGB 하나로 depth 도 렌더한다.

cfg = _paths.system_config()
BELT = float(cfg["conveyor"]["belt_top_z_mm"])
STOP_Y = float(cfg["conveyor"]["stopper_x_mm"])       # 진행축(Y) 기준
CROSS = cfg["zones"]["pickup_cross_mm"]
MAIN_Y = float(cfg["conveyor"]["spawn_point_mm"][1])

_fail = []


def say(*a):
    print(*a, flush=True)


def chk(name, ok, detail=""):
    say("     %-34s %s %s" % (name, "OK" if ok else "X", detail))
    if not ok:
        _fail.append(name)


def die(msg, code=1):
    say("\n!! %s" % msg)
    _kitexit.shutdown(code)
    raise SystemExit(code)


omni.usd.get_context().open_stage(STAGE)
st = omni.usd.get_context().get_stage()
if st is None:
    die("씬을 못 열었다: %s" % STAGE)
say("\n씬  %s" % STAGE)
say("벨트면 z=%.0f · 스토퍼 Y=%.0f · 본선 중심 Y=%.0f" % (BELT, STOP_Y, MAIN_Y))


def rig_of(cam):
    """`cam` 이 속한 `/World` 바로 아래 prim."""
    p = cam
    while p and p.IsValid():
        par = p.GetParent()
        if not par or not par.IsValid() or par.IsPseudoRoot():
            return None
        if str(par.GetPath()) == "/World":
            return p
        p = par
    return None


def lens_of(cams):
    return next((c for c in cams if LENS_PREF in c.GetName()), cams[0])


def world(prim):
    return UsdGeom.XformCache().GetLocalToWorldTransform(prim)


def report(name, lens, target=None):
    """벨트면 조준점 · 시야 · 목표와의 어긋남."""
    M = world(lens)
    t, R = M.ExtractTranslation(), M.ExtractRotationMatrix()
    fwd = [-float(R[2][j]) for j in range(3)]
    c = UsdGeom.Camera(lens)
    fl = float(c.GetFocalLengthAttr().Get() or 0) or 1.0
    ha = float(c.GetHorizontalApertureAttr().Get() or 0)
    va = float(c.GetVerticalApertureAttr().Get() or 0)
    hf = 2 * math.degrees(math.atan(ha / (2 * fl)))
    vf = 2 * math.degrees(math.atan(va / (2 * fl)))
    cx, cy, cz = t[0] * 1000, t[1] * 1000, t[2] * 1000
    say("   %-11s (%8.0f,%8.0f,%8.0f) 광축(%+.2f,%+.2f,%+.2f) HFOV %.1f VFOV %.1f"
        % (name, cx, cy, cz, fwd[0], fwd[1], fwd[2], hf, vf))
    if abs(fwd[2]) < 1e-6 or (BELT - cz) / fwd[2] <= 0:
        say("               ★ 벨트면을 안 본다")
        return False
    s = (BELT - cz) / fwd[2]
    gx, gy = cx + fwd[0] * s, cy + fwd[1] * s
    d = math.sqrt(sum((fwd[k] * s) ** 2 for k in range(3)))
    hw, hh = d * math.tan(math.radians(hf / 2)), d * math.tan(math.radians(vf / 2))
    say("               조준 (%.0f, %.0f) · 거리 %.0f · 시야 %.0f x %.0f mm"
        % (gx, gy, d, hw * 2, hh * 2))
    if target:
        ok = abs(target[0] - gx) <= hw and abs(target[1] - gy) <= hh
        say("               목표 (%.0f, %.0f) -> 어긋남 X %+.0f Y %+.0f   %s"
            % (target[0], target[1], target[0] - gx, target[1] - gy,
               "시야 안" if ok else "★ 시야 밖"))
        return ok
    return True


# ── 리그 수집 ──────────────────────────────────────────────────────────────
rigs = {}
for p in st.TraverseAll():
    if p.GetTypeName() == "Camera":
        r = rig_of(p)
        if r is not None:
            rigs.setdefault(str(r.GetPath()), []).append(p)

# ── [1] 원점 리그 삭제 ─────────────────────────────────────────────────────
say("\n[1] 원점에 놓다 만 리그 지우기")
dead = []
for path in sorted(rigs):
    t = world(lens_of(rigs[path])).ExtractTranslation()
    if max(abs(v) * 1000 for v in t) < 1.0:
        dead.append(path)
for path in dead:
    say("   %s 삭제 (원점 0,0,0)" % path)
    rigs.pop(path, None)
    if not DRY:
        st.RemovePrim(path)
if not dead:
    say("   없다")

# ── 목표 자리 ──────────────────────────────────────────────────────────────
# ★ 이름 끝 글자로 존을 가린다. 존이 아니면 본선 카메라로 본다.
wants = {}
for path in rigs:
    nm = path.rsplit("/", 1)[-1]
    z = nm[-1].upper()
    if z in CROSS:
        wants[path] = (sum(CROSS[z]) / 2.0, STOP_Y - BACK_MM, "존 %s 픽업" % z)
    else:
        cur = world(lens_of(rigs[path])).ExtractTranslation()[0] * 1000
        wants[path] = (cur, MAIN_Y, "본선 (X 유지)")

say("\n[2] 지금 상태")
for path in sorted(rigs):
    report(path.rsplit("/", 1)[-1], lens_of(rigs[path]), wants[path][:2])

if DRY:
    say("\nPL_AIM_DRY — 저장하지 않는다.")
    _kitexit.shutdown(0)
    raise SystemExit(0)

# ── [3] 아래를 보게 돌리고 자리로 옮긴다 ───────────────────────────────────
say("\n[3] 아래(-Z)로 돌리고 자리로 옮긴다")
for path in sorted(rigs):
    rig = st.GetPrimAtPath(path)
    lens = lens_of(rigs[path])
    tx, ty, why = wants[path]

    # ── 회전 ──
    # ★ **이미 아래를 보고 있으면 손대지 않는다** (실측 2026-08-26: 다섯 대 다
    #   광축 (0,0,-1) 이었다). GUI 가 남긴 op 조합을 다시 짜면 위험만 있고 얻는
    #   게 없다 — 실제로 `AddOrientOp(PrecisionDouble)` 이 기존 `quatf` 와
    #   부딪혀 죽었다. 필요할 때만, **있는 op 의 타입 그대로** 쓴다.
    R_len = world(lens).ExtractRotationMatrix()
    fwd0 = [-float(R_len[2][j]) for j in range(3)]
    if fwd0[2] > -0.9999:                      # 아래를 안 보고 있다
        R_rig = world(rig).ExtractRotationMatrix()
        R_in = R_rig.GetInverse() * R_len
        q = Gf.Matrix4d(R_in.GetInverse(),
                        Gf.Vec3d(0, 0, 0)).ExtractRotationQuat()
        a = rig.GetAttribute("xformOp:orient")
        if a and a.IsValid():
            # 기존 op 의 정밀도를 따른다 (quatf / quatd)
            a.Set(Gf.Quatf(q) if a.GetTypeName().type.typeName.endswith("quatf")
                  else Gf.Quatd(q))
        else:
            UsdGeom.Xformable(rig).AddOrientOp().Set(Gf.Quatf(q))
        say("   %-11s 광축을 아래로 돌렸다" % path.rsplit("/", 1)[-1])

    op_t = rig.GetAttribute("xformOp:translate")
    if not (op_t and op_t.IsValid()):
        op_t = UsdGeom.Xformable(rig).AddTranslateOp(
            UsdGeom.XformOp.PrecisionDouble).GetAttr()

    # ── 위치: 재고 고치기 (리그 안쪽 변환 때문에 한 번에 안 맞는다) ──
    zw = world(lens).ExtractTranslation()[2] * 1000.0      # 높이는 유지
    for _ in range(8):
        t = world(lens).ExtractTranslation()
        d = [tx - t[0] * 1000, ty - t[1] * 1000, zw - t[2] * 1000]
        if max(abs(v) for v in d) < 0.05:
            break
        c = op_t.Get()
        op_t.Set(Gf.Vec3d(*[float(c[k]) + d[k] / 1000.0 for k in range(3)]))
    say("   %-11s -> %s" % (path.rsplit("/", 1)[-1], why))

# ── [4] 저장 · 검증 ────────────────────────────────────────────────────────
say("\n[4] 저장하고 다시 잰다")
st.GetRootLayer().Save()
allok = True
for path in sorted(rigs):
    allok &= bool(report(path.rsplit("/", 1)[-1], lens_of(rigs[path]),
                         wants[path][:2]))
chk("다섯 대 모두 목표가 시야 안", allok)
st2 = Usd.Stage.Open(STAGE)
for path in dead:
    chk("삭제 %s" % path.rsplit("/", 1)[-1],
        not st2.GetPrimAtPath(path).IsValid())

say("\nRESULT: %s" % ("전부 통과" if not _fail else "실패 %d — %s"
                     % (len(_fail), ", ".join(_fail))))
_kitexit.shutdown(0 if not _fail else 1)
