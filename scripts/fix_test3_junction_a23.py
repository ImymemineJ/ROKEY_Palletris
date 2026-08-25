"""test_3.usd — 분기점 벨트를 **ConveyorBelt_A23 으로 되돌린다** (2026-08-25).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/fix_test3_junction_a23.py

★★ 왜 A47 을 물렸다 되돌리나 — 실측으로 드러난 사실.

  A47 은 "소터가 달린 분기 모듈" 이라 분류에 딱 맞아 보였고, bbox 도 A23 과
  완전히 같아 좌표를 안 깨고 갈아탈 수 있었다. 그런데 **소터 구간(로컬 X
  1670~2528)에는 본선 롤러가 없다.** 그 자리를 받치고 미는 것이 소터 자신이다.

      1) 소터를 그대로 두면 소터 롤러가 본선 롤러보다 28.9 mm 높아 턱이 된다
         -> 박스가 X −6.62 에서 165 s 동안 멈췄다.
      2) 소터를 비활성화하면 그 구간에 **지지면이 사라진다**
         -> 박스가 바닥까지 떨어졌다 (z 0.14 m).
      3) 소터를 내려 높이를 맞추면 박스가 뜨지도 걸리지도 않지만, 소터는
         `IsaacConveyor` 의 `conveyorPrim` 에 안 물려 있어 **표면속도를 못 준다**
         -> 관성으로 굴러가다 소터 구간 끝(X −6.83)에서 멎는다.

  즉 **A47 은 소터 ActionGraph 를 실제로 구동해야 쓸 수 있다.** 그건 "분류를
  `set_direction` 에서 소터로 옮기는" 별도 과제다. 그때까지는 롤러가 연속인
  A23 이 맞다.

★ 되돌릴 때 같이 푸는 것
  - 롤러면 보정 +21.5 mm. A47 이 롤러를 낮게 두어 넣었던 값이다. A23 은 원래
    800 이라 빼야 한다. **재면서** 맞춘다 (하드코딩하지 않는다).
  - 프레임 충돌 끄기는 A23 이름으로 다시 건다 (롤러면 위 23 mm 턱 — 문서 §3).
"""
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

CUR = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test_3.usd")
A23 = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
       "/Assets/Isaac/5.1/Isaac/Props/Conveyors/ConveyorBelt_A23.usd")
JUNCTIONS = ("/World/ConveyorTrack_09",      # 분기 A
             "/World/ConveyorTrack_08",      # 분기 B
             "/World/ConveyorTrack_03")      # 분기 C
REF_ROLLER = "/World/ConveyorTrack/Rollers"  # 롤러면 높이 기준
EXPECT_W_MM, EXPECT_D_MM = 6955.1, 5181.5
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


print("[-] A23 캐시 데우기 ...")
sys.stdout.flush()
if Usd.Stage.Open(A23) is None:
    raise SystemExit("A23 을 못 열었다: %s" % A23)
print("    열림")
sys.stdout.flush()

print("\n[1] 참조를 A23 으로 되돌린다")
sys.stdout.flush()
for p in JUNCTIONS:
    prim = st.GetPrimAtPath(p)
    prim.GetReferences().ClearReferences()
    prim.GetReferences().AddReference(A23)
    print("   %-26s -> ConveyorBelt_A23" % p)
    sys.stdout.flush()

print("\n[2] A47 시절 override 청소")
# ★ A47 자식 이름으로 걸어 둔 것들이 A23 위에 남으면 유령 prim 이 된다.
_lay = st.GetRootLayer()
for p in JUNCTIONS:
    sp = _lay.GetPrimAtPath(Sdf.Path(p))
    if sp is None:
        continue
    for nm in list(sp.nameChildren.keys()):
        if "A47" in nm or nm == "Sorter":
            del sp.nameChildren[nm]
            print("   %-26s %s 제거" % (p, nm))
            sys.stdout.flush()

print("\n[3] 롤러면 높이 다시 맞추기")
sys.stdout.flush()
_ref = bb(REF_ROLLER)
if _ref is None:
    raise SystemExit("기준 벨트를 못 읽었다: %s" % REF_ROLLER)
_top = _ref[5]
print("   기준 롤러 윗면 %.1f mm" % _top)
sys.stdout.flush()
for p in JUNCTIONS:
    r = bb(p + "/Rollers")
    dz = _top - r[5]
    if abs(dz) < 0.1:
        print("   %-26s 이미 맞다 (%.1f)" % (p, r[5]))
        sys.stdout.flush()
        continue
    a = st.GetPrimAtPath(p).GetAttribute("xformOp:translate")
    v = a.Get()
    a.Set(Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]) + dz / 1000.0))
    print("   %-26s 롤러 윗면 %.1f -> %.1f  (%+.1f mm)"
          % (p, r[5], bb(p + "/Rollers")[5], dz))
    sys.stdout.flush()

print("\n[4] 프레임 충돌 끄기 (A23 이름)")
sys.stdout.flush()
for p in JUNCTIONS:
    prim = st.GetPrimAtPath(p)
    hit = 0
    for ch in prim.GetChildren():
        n = ch.GetName()
        if not n.startswith("SM_ConveyorBelt_"):
            continue
        if not UsdPhysics.CollisionAPI(ch):
            UsdPhysics.CollisionAPI.Apply(ch)
        a = ch.GetAttribute("physics:collisionEnabled")
        if not a:
            a = ch.CreateAttribute("physics:collisionEnabled",
                                   Sdf.ValueTypeNames.Bool, True)
        a.Set(False)
        hit += 1
        print("   %-26s %s 충돌 끔" % (p, n))
        sys.stdout.flush()
    if hit == 0:
        print("   %-26s !! 프레임 메시를 못 찾았다" % p)
        sys.stdout.flush()

st.GetRootLayer().Save()
print("\n저장했다: %s" % CUR)
sys.stdout.flush()

print("\n=== 검증 ===")
sys.stdout.flush()
for p in JUNCTIONS:
    print("--- %s" % p)
    sys.stdout.flush()
    v, r, r1 = bb(p), bb(p + "/Rollers"), bb(p + "/Rollers_01")
    chk("전체 bbox = 정본", v is not None
        and abs((v[3] - v[0]) - EXPECT_W_MM) < 2
        and abs((v[4] - v[1]) - EXPECT_D_MM) < 2,
        "%.1f x %.1f" % (v[3] - v[0], v[4] - v[1]) if v else "없다")
    chk("롤러 윗면 = 기준", r is not None and abs(r[5] - _top) < 0.5,
        "%.1f (기준 %.1f)" % (r[5], _top) if r else "없다")
    chk("본선 롤러 폭 = 1546", r is not None and abs((r[4] - r[1]) - 1546.0) < 2,
        "%.1f" % (r[4] - r[1]) if r else "없다")
    chk("지선이 스토퍼까지", r1 is not None and r1[4] >= 4180.0,
        "Y %.1f~%.1f" % (r1[1], r1[4]) if r1 else "없다")
    chk("소터가 없다 (A23)", not st.GetPrimAtPath(p + "/Sorter").IsValid())
    ge = [ch.GetName() for ch in st.GetPrimAtPath(p).GetChildren()
          if ch.GetName().startswith("ConveyorBeltGraph")]
    chk("벨트 그래프 살아 있다", len(ge) >= 2, "%s" % ge)
    for ch in st.GetPrimAtPath(p).GetChildren():
        if ch.GetName().startswith("SM_ConveyorBelt_"):
            en = ch.GetAttribute("physics:collisionEnabled")
            chk("프레임 %s 충돌 끔" % ch.GetName()[-9:],
                en is not None and en.Get() is False)
    # ★ 그래프가 가리키는 롤러가 실제로 있어야 한다. 참조를 갈면 이게 조용히 끊긴다.
    for g, tgt in ((p + "/ConveyorBeltGraph", p + "/Rollers"),
                   (p + "/ConveyorBeltGraph_01", p + "/Rollers_01")):
        nd = st.GetPrimAtPath(g + "/ConveyorNode")
        tg = []
        if nd and nd.IsValid():
            rel = nd.GetRelationship("inputs:conveyorPrim")
            tg = [str(x) for x in rel.GetTargets()] if rel else []
        chk("%s -> 롤러" % g.rsplit("/", 1)[-1],
            tg == [tgt] and st.GetPrimAtPath(tgt).IsValid(), "%s" % tg)

print("=" * 70)
print("RESULT: %s" % ("통과" if not _fail else "실패 — %s" % ", ".join(_fail)))
print("=" * 70)
sys.stdout.flush()
os._exit(0)
