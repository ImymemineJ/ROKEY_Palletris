"""test_3.usd 의 카메라·ROS2 그래프 실측 스캔 (읽기 전용).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/scan_cams_test3.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
from pxr import Usd, UsdGeom, Gf                               # noqa: E402

STAGE = os.environ.get("PL_STAGE",
                       "/home/rokey/isaac_scenes/Collected_test_3/test_3.usd")


def main():
    st = Usd.Stage.Open(STAGE)
    if st is None:
        raise SystemExit("씬을 못 열었다: %s" % STAGE)
    xc = UsdGeom.XformCache()
    cams, rsd, graphs, ros = [], [], [], []
    for p in st.Traverse():
        path, tn = str(p.GetPath()), p.GetTypeName()
        low = path.lower()
        if tn == "Camera":
            cams.append(p)
        if "rsd455" in low or "realsense" in low or "d455" in low:
            rsd.append(path)
        if tn == "OmniGraph":
            graphs.append(path)
        if "ros2" in str(tn).lower() or "ROS2" in path:
            ros.append((path, str(tn)))
    print("\n===== Camera prim %d 개 =====" % len(cams))
    for p in cams:
        m = xc.GetLocalToWorldTransform(p)
        t = m.ExtractTranslation()
        # 광축(-Z 로컬)의 월드 방향
        R = m.ExtractRotationMatrix()
        fwd = [-float(R[2][j]) for j in range(3)]
        c = UsdGeom.Camera(p)
        fl = c.GetFocalLengthAttr().Get()
        ha = c.GetHorizontalApertureAttr().Get()
        print("  %-58s  t=(%.0f, %.0f, %.0f)  광축=(%.2f,%.2f,%.2f)  f=%s ap=%s"
              % (str(p.GetPath()), t[0], t[1], t[2], fwd[0], fwd[1], fwd[2], fl, ha))
    print("\n===== rsd455 / realsense 관련 prim %d 개 =====" % len(rsd))
    for s in rsd[:40]:
        print("  " + s)
    if len(rsd) > 40:
        print("  ... 외 %d 개" % (len(rsd) - 40))
    print("\n===== OmniGraph %d 개 =====" % len(graphs))
    for s in graphs:
        print("  " + s)
    print("\n===== ROS2 관련 prim %d 개 =====" % len(ros))
    for s, tn in ros[:40]:
        print("  %-70s %s" % (s, tn))


main()
try:
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import _kitexit                                            # noqa: E402
    _kitexit.bye()
except Exception:
    import omni.kit.app
    omni.kit.app.get_app().post_quit()
