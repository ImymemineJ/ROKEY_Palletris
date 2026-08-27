"""픽업 카메라가 **실제로 픽업 지점을 보는지** 잰다 (읽기 전용).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/check_cam_aim.py

★ 반드시 Isaac 안에서 돌린다. rsd455 는 **페이로드**라 맨 pxr 로는 리그 안의
  렌즈가 안 보인다 (실측 2026-08-26: standalone 은 Camera 0 개로 나온다).

★ 무엇을 재나. 광축(로컬 -Z)을 벨트면(z = belt_top_z_mm)까지 연장해 조준점을
  구하고, 그 존의 픽업점(지선 중심 X · 스토퍼 바로 앞 Y)과 얼마나 어긋나는지
  본다. 시야 반각으로 "프레임 안인가" 까지 판정한다.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import omni.usd                                                # noqa: E402
from pxr import UsdGeom                                        # noqa: E402

import _kitexit                                                # noqa: E402
import _paths                                                  # noqa: E402

STAGE = os.environ.get(
    "PL_STAGE", "/home/rokey/isaac_scenes/Collected_test_3/test_3.usd")
# 박스 중심이 스토퍼에서 얼마나 뒤인가 (3호 250 ~ 5호 380 의 절반쯤)
BACK_MM = float(os.environ.get("PL_PICK_BACK_MM", "150"))

cfg = _paths.system_config()
BELT = float(cfg["conveyor"]["belt_top_z_mm"])
STOP_Y = float(cfg["conveyor"]["stopper_x_mm"])        # 진행축(Y) 기준 스토퍼
CROSS = cfg["zones"]["pickup_cross_mm"]

omni.usd.get_context().open_stage(STAGE)
st = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()

print("\n벨트면 z=%.0f · 스토퍼 %.0f · 픽업점은 스토퍼에서 %.0f 뒤로 잡았다\n"
      % (BELT, STOP_Y, BACK_MM), flush=True)

# 리그 -> 존. 이름 규약을 코드가 정하지 않는다. 존 문자로 끝나면 그 존으로 본다.
rigs = {}
for p in st.TraverseAll():
    if p.GetTypeName() != "Camera":
        continue
    q = p
    while q.GetParent() and str(q.GetParent().GetPath()) != "/World":
        q = q.GetParent()
        if q.GetParent() is None or q.GetParent().IsPseudoRoot():
            q = None
            break
    if q is None:
        continue
    rigs.setdefault(str(q.GetPath()), []).append(p)

for path in sorted(rigs):
    name = path.rsplit("/", 1)[-1]
    zone = name[-1].upper() if name[-1].upper() in CROSS else None
    # 리그의 대표 렌즈 — 색 카메라가 있으면 그것, 없으면 첫 번째
    lens = next((c for c in rigs[path] if "Color" in c.GetName()), rigs[path][0])
    M = xc.GetLocalToWorldTransform(lens)
    t = M.ExtractTranslation()
    R = M.ExtractRotationMatrix()
    fwd = [-float(R[2][j]) for j in range(3)]          # 광축 = 로컬 -Z
    c = UsdGeom.Camera(lens)
    fl = float(c.GetFocalLengthAttr().Get() or 0) or 1.0
    ha = float(c.GetHorizontalApertureAttr().Get() or 0)
    va = float(c.GetVerticalApertureAttr().Get() or 0)
    hf = 2 * math.degrees(math.atan(ha / (2 * fl)))
    vf = 2 * math.degrees(math.atan(va / (2 * fl)))
    cx, cy, cz = t[0] * 1000, t[1] * 1000, t[2] * 1000
    print("%-12s (%8.0f, %8.0f, %8.0f)  광축 (%+.2f,%+.2f,%+.2f)  HFOV %.1f · VFOV %.1f"
          % (name, cx, cy, cz, fwd[0], fwd[1], fwd[2], hf, vf), flush=True)
    print("             렌즈 %s  f=%.2f ap=%.2f" % (lens.GetName(), fl, ha), flush=True)
    if abs(fwd[2]) < 1e-6:
        print("             ★ 광축이 수평이라 벨트면과 안 만난다\n", flush=True)
        continue
    s = (BELT - cz) / fwd[2]
    if s <= 0:
        print("             ★ 광축이 벨트면 반대쪽을 본다\n", flush=True)
        continue
    gx, gy = cx + fwd[0] * s, cy + fwd[1] * s
    d = math.sqrt((fwd[0] * s) ** 2 + (fwd[1] * s) ** 2 + (fwd[2] * s) ** 2)
    hw = d * math.tan(math.radians(hf / 2))
    hh = d * math.tan(math.radians(vf / 2))
    print("             벨트면 조준 (%.0f, %.0f) · 거리 %.0f · 시야 약 %.0f x %.0f mm"
          % (gx, gy, d, hw * 2, hh * 2), flush=True)
    if zone:
        px = sum(CROSS[zone]) / 2.0
        py = STOP_Y - BACK_MM
        inside = abs(px - gx) <= hw and abs(py - gy) <= hh
        print("             존 %s 픽업점 (%.0f, %.0f) -> 어긋남 X %+.0f · Y %+.0f mm   %s"
              % (zone, px, py, px - gx, py - gy,
                 "시야 안" if inside else "★ 시야 밖"), flush=True)
    print("", flush=True)

_kitexit.shutdown(0)
