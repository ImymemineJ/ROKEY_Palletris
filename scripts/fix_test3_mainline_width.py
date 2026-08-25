"""test_3.usd — 본선 롤러 폭을 전 구간 같게 맞춘다 (2026-08-25).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/fix_test3_mainline_width.py

★ 무엇이 안 맞았나 (실측).

      /World/ConveyorTrack       scale.y 1.0     롤러 폭  903.0   <- 스폰 구간
      /World/ConveyorTrack_09    scale.y 1.7117  롤러 폭 1546.0   <- 분기 A
      /World/ConveyorTrack_08    scale.y 1.7117  롤러 폭 1546.0   <- 분기 B
      /World/ConveyorTrack_03    scale.y 1.7117  롤러 폭 1546.0   <- 분기 C
      /World/ConveyorTrack_05    scale.y 1.7117  롤러 폭 1545.5   <- 순환 루프

  스폰 구간만 903 이고 나머지는 1546 이다. 한 라인인데 폭이 두 가지다.

★ 왜 **넓은 쪽**으로 맞추나. 좁은 쪽(1.0)으로 내리려면 분기 조각의 scale.y 를
  내려야 하는데, 그 값은 **폭만이 아니라 지선 길이까지** 지배한다 (A23/A47 은
  L 자 한 덩어리라 로컬 Y 가 본선 폭 + 지선 길이를 겸한다):

      지선 길이 3303.2 x (1.0 / 1.7117) = 1929.6
      -> 지선이 Y 878~2807 에서 끝난다. 스토퍼는 Y 4181 이다. **1374 mm 모자란다.**
         박스가 로봇 앞까지 못 가고, 로봇은 아무것도 못 집는다.
      순환 루프(_04)도 같은 이유로 끊긴다.

  그래서 스폰 구간 하나를 **1.7117 로 올린다.** 지선·루프는 손대지 않는다.

★ 폭을 바꾸면 중심이 밀린다. 롤러 중심 Y 를 0 으로 다시 맞춘다 — 안 맞추면
  박스가 벨트 한쪽으로 치우쳐 흐르고, 분기점 창(`main_line_y_mm`)을 벗어난다.
"""
import os
import sys

from pxr import Gf, Usd, UsdGeom

CUR = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test_3.usd")
REF = "/World/ConveyorTrack_09"          # 폭의 정본 = 분기 조각
TARGETS = ("/World/ConveyorTrack",)      # 폭을 맞출 조각
PU = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]

st = Usd.Stage.Open(CUR)
_fail = []


def bb(path):
    c = UsdGeom.BBoxCache(Usd.TimeCode.Default(), PU)   # ★ 매번 새로
    p = st.GetPrimAtPath(path)
    if not p or not p.IsValid():
        return None
    r = c.ComputeWorldBound(p).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    a, b = r.GetMin(), r.GetMax()
    return [v * 1000.0 for v in (a[0], a[1], a[2], b[0], b[1], b[2])]


def chk(name, ok, detail=""):
    print("     %-32s %s %s" % (name, "OK" if ok else "X", detail))
    sys.stdout.flush()
    if not ok:
        _fail.append(name)


ref_sc = st.GetPrimAtPath(REF).GetAttribute("xformOp:scale").Get()
ref_y = float(ref_sc[1])
ref_r = bb(REF + "/Rollers")
print("기준 %s  scale.y %.4f · 롤러 폭 %.1f · 중심 Y %.1f"
      % (REF, ref_y, ref_r[4] - ref_r[1], (ref_r[1] + ref_r[4]) / 2.0))
sys.stdout.flush()

print("\n[1] 폭 맞추기")
sys.stdout.flush()
for p in TARGETS:
    prim = st.GetPrimAtPath(p)
    a = prim.GetAttribute("xformOp:scale")
    s0 = a.Get()
    r0 = bb(p + "/Rollers")
    t0 = bb(p)
    a.Set(Gf.Vec3f(float(s0[0]), ref_y, float(s0[2])))
    # ★ 중심 되맞추기. scale 은 로컬 원점 기준이라 폭을 키우면 한쪽으로 밀린다.
    r1 = bb(p + "/Rollers")
    tr = prim.GetAttribute("xformOp:translate")
    v = tr.Get()
    dy = -((r1[1] + r1[4]) / 2.0)
    tr.Set(Gf.Vec3d(float(v[0]), float(v[1]) + dy / 1000.0, float(v[2])))
    # ★ 폭을 키우면 z 도 같이 안 변하지만 롤러 윗면은 재확인한다.
    r2 = bb(p + "/Rollers")
    t2 = bb(p)
    print("   %-26s scale.y %.4f -> %.4f" % (p, float(s0[1]), ref_y))
    print("        롤러 폭 %.1f -> %.1f · 중심 Y %.1f -> %.1f · 윗면 %.1f"
          % (r0[4] - r0[1], r2[4] - r2[1],
             (r0[1] + r0[4]) / 2.0, (r2[1] + r2[4]) / 2.0, r2[5]))
    print("        프레임 Y %.1f~%.1f -> %.1f~%.1f"
          % (t0[1], t0[4], t2[1], t2[4]))
    sys.stdout.flush()

st.GetRootLayer().Save()
print("\n저장했다: %s" % CUR)
sys.stdout.flush()

print("\n=== 검증 ===")
sys.stdout.flush()
ref_w = ref_r[4] - ref_r[1]
ref_top = ref_r[5]
for p in list(TARGETS) + [REF, "/World/ConveyorTrack_08", "/World/ConveyorTrack_03",
                          "/World/ConveyorTrack_05"]:
    r = bb(p + "/Rollers")
    if r is None:
        chk("%s 롤러" % p.rsplit("/", 1)[-1], False, "못 읽음")
        continue
    print("--- %s" % p)
    sys.stdout.flush()
    chk("롤러 폭 = 기준", abs((r[4] - r[1]) - ref_w) < 2,
        "%.1f (기준 %.1f)" % (r[4] - r[1], ref_w))
    chk("롤러 윗면 = 800", abs(r[5] - 800.0) < 0.5, "%.1f" % r[5])

# ★ 지선과 순환 루프는 **안 건드렸다**. 그대로인지 확인한다.
print("--- 손대지 않은 것")
sys.stdout.flush()
for z, p in (("A", "/World/ConveyorTrack_09"), ("B", "/World/ConveyorTrack_08"),
             ("C", "/World/ConveyorTrack_03")):
    s1 = bb(p + "/Rollers_01")
    chk("지선 %s 가 스토퍼까지" % z, s1 is not None and s1[4] >= 4180.0,
        "Y %.1f~%.1f" % (s1[1], s1[4]) if s1 else "없다")
lp = bb("/World/ConveyorTrack_04/Rollers")
chk("순환 루프 코너 그대로", lp is not None and (lp[4] - lp[1]) > 6000,
    "폭 %.1f" % (lp[4] - lp[1]) if lp else "없다")

# 스폰점이 넓어진 벨트 위인지
sp = st.GetPrimAtPath("/World/ConveyorTrack/Rollers")
r = bb("/World/ConveyorTrack/Rollers")
chk("스폰점(-409, -66)이 롤러 위", r[1] <= -66 <= r[4],
    "롤러 Y %.1f~%.1f" % (r[1], r[4]))

print("=" * 70)
print("RESULT: %s" % ("통과" if not _fail else "실패 — %s" % ", ".join(_fail)))
print("=" * 70)
sys.stdout.flush()
os._exit(0)
