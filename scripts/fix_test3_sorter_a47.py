"""test_3.usd — 분기점을 **ConveyorBelt_A47(소터 모듈)** 로 바꾸고 소터를 켠다.

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/fix_test3_sorter_a47.py

★★ 왜 다시 A47 인가 — 2026-08-25 오전의 실패를 뒤집은 실측.

  오전에는 A47 을 세 번 시도해 세 번 다 라인이 막혔고 A23 으로 되돌렸다
  (`fix_test3_junction_a23.py`). 그때 몰랐던 것이 애셋을 열어 보니 나왔다.

  1) **본선 롤러에 구멍이 있다.** A47 의 `Rollers` 는 로컬 X 1651.4~2555.3,
     즉 **903.9 mm 가 비어 있다.** 그 자리를 채우는 것이 소터다. 그래서
     소터를 비활성화하면 박스가 바닥까지 떨어졌다 — 당연했다.

  2) **소터는 자기만의 `IsaacConveyor` 를 가진 독립 컨베이어다.**

         Sorter/ActionGraph/conveyor_belt   (omni.isaac.conveyor.IsaacConveyor)
             inputs:conveyorPrim -> Sorter/Sorter_physics
             inputs:velocity     <- SorterSpeed(ConstantFloat) = **0.0**
             inputs:direction    =  (1,0,0)   ← 아무것도 연결돼 있지 않다
             inputs:onStep       <- on_tick

     **`SorterSpeed` 가 0 이다.** 그래서 "높이를 맞춰도 관성으로 굴러가다 멎었다."
     소터가 안 돈 게 아니라 **0 으로 돌고 있었다.**

  3) 애셋에는 분류 배선이 반쯤 들어 있다 — `reroute`(ConstantBool) -> `to_float`
     -> x`Direction`(90°) -> 축(0,0,1) -> `rotate_vector` 로 (1,0,0) 을 돌린다.
     그런데 **`rotate_vector.outputs:result` 가 어디에도 연결돼 있지 않다.**
     그래서 `reroute` 를 켜도 아무 일이 안 난다.

     여기서는 그 끊긴 사슬을 잇는 대신 `conveyor_belt.inputs:direction` 을
     **직접 쓴다.** 본선에서 이미 검증된 것과 **똑같은 경로**이기 때문이다
     (`conveyor_ctl.set_direction`). 끊긴 사슬을 이으면 `rotate_vector` 의
     회전 규약(도/라디안·축 순서)을 오프라인에서 확인할 방법이 없다.
     `reroute` 값은 애셋 상태가 실제와 어긋나 보이지 않도록 같이 맞춰만 둔다.

★ 이게 왜 중요한가 — 지금 분류(`set_direction`)의 한계가 사라진다.

  지금은 **본선 조각 전체(6925 mm)** 의 표면속도를 꺾는다. 그래서 그 조각에
  박스가 둘이면 못 꺾고, 못 꺾인 박스는 순환 루프를 돌아 다시 와 더 몰린다.
  투입 간격을 20 s 까지 벌려야 했던 이유가 이것이다 (test_3.yaml `spawn_interval_s`).

  소터는 꺾이는 구간이 **약 1492 mm** 뿐이다 (로컬 857.9 x scale.x 1.7387).
  뒤차는 본선을 그대로 흘러온다. 배타 구간이 4.6 배 짧아진다.

★ 높이. 애셋 그대로면 소터 롤러 윗면이 본선 롤러보다 **7.2 mm 높다**
  (776.5 vs 769.3). 오전에 "립 7 mm" 로 잰 그 값이다. `Sorter` 를 그만큼
  내린다. 밴드(774.4)는 롤러보다 2.1 mm 낮은데, 박스는 롤러 위에 얹히므로 맞다.

★ A23 시절 override 청소. A23 은 고무밴드가 두 줄(`Rubberbands`,
  `Rubberbands_01`)이고 A47 은 한 줄이다. 남은 `Rubberbands_01` override 가
  유령 prim 이 되어 존 bbox 를 8591.8 x 11023.6 으로 부풀렸다 (오전 실측).
  ⚠ `Rollers` / `Rollers_01` 은 **건드리지 않는다** — 씬의 벨트 그래프가
    그 경로를 가리키고 있어 지우면 본선이 조용히 선다.
"""
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

CUR = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test_3.usd")
A47 = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
       "/Assets/Isaac/5.1/Isaac/Props/Conveyors/ConveyorBelt_A47.usd")
JUNCTIONS = (("A", "/World/ConveyorTrack_09"),
             ("B", "/World/ConveyorTrack_08"),
             ("C", "/World/ConveyorTrack_03"))
REF_ROLLER = "/World/ConveyorTrack/Rollers"     # 롤러면 높이의 정본
# ★ 분기 조각의 **정본 치수**. A23 과 A47 이 같다 — 그래서 좌표가 안 깨진다.
EXPECT_W_MM, EXPECT_D_MM = 6955.1, 5181.5
NOMINAL_MPS = 0.7                                # test_3.yaml belt_velocity_mps
# ★ 본선 진행 방향(월드). test_3 은 스폰 X -409 에서 존 C X -19596 으로 가므로 -X 다.
#   A47 애셋의 기본 direction 은 로컬 (1,0,0) = 월드 **+X** 라 그대로 두면 소터가
#   본선과 맞서 민다 (실측 2026-08-25: 박스가 소터 위 X -7.11 에서 75 s 멎었다).
THRU_WORLD = (-1.0, 0.0, 0.0)
# A23 에만 있던 자식. 남으면 유령이 된다.
STALE = ("Rubberbands_01",)
PU = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]

_fail = []


def bb(st, path):
    c = UsdGeom.BBoxCache(Usd.TimeCode.Default(), PU)   # ★ 매번 새로 (캐시가 상한다)
    p = st.GetPrimAtPath(path)
    if not p or not p.IsValid():
        return None
    r = c.ComputeWorldBound(p).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    a, b = r.GetMin(), r.GetMax()
    return [v * 1000.0 for v in (a[0], a[1], a[2], b[0], b[1], b[2])]


def chk(name, ok, detail=""):
    print("     %-36s %s %s" % (name, "OK" if ok else "X", detail))
    sys.stdout.flush()
    if not ok:
        _fail.append(name)


def gain_of(st, node_path, conveyor_path):
    """IsaacConveyor 의 velocity -> 월드 m/s 배율.

    `conveyor_ctl._measure_gain` 과 **같은 식이다.** direction 은 conveyorPrim 의
    로컬축이고, 표면속도는 그 조각의 월드 변환만큼 배로 커진다.
    """
    nd = st.GetPrimAtPath(node_path)
    roll = st.GetPrimAtPath(conveyor_path)
    if not nd or not nd.IsValid() or not roll or not roll.IsValid():
        return None
    d = nd.GetAttribute("inputs:direction").Get()
    if d is None:
        return None
    M = UsdGeom.XformCache().GetLocalToWorldTransform(roll).ExtractRotationMatrix()
    wv = [sum(float(d[k]) * float(M[k][j]) for k in range(3)) for j in range(3)]
    return (wv[0] ** 2 + wv[1] ** 2 + wv[2] ** 2) ** 0.5


# ── A47 캐시 데우기 ────────────────────────────────────────────────
# ★ 원격 S3 애셋이라 첫 접근이 몇 분 걸린다. 여기서 먼저 받아 두지 않으면
#   참조를 건 직후 컴포즈에서 멎는데, **에러 없이** 조용히 멎어 오전에
#   11 분을 CPU 0% 로 날렸다.
print("[-] A47 캐시 데우기 ...")
sys.stdout.flush()
_a = Usd.Stage.Open(A47)
if _a is None:
    raise SystemExit("A47 을 못 열었다: %s" % A47)
print("    열림")
sys.stdout.flush()

st = Usd.Stage.Open(CUR)

print("\n[1] 참조를 A47 로 바꾼다")
sys.stdout.flush()
for z, p in JUNCTIONS:
    prim = st.GetPrimAtPath(p)
    prim.GetReferences().ClearReferences()
    prim.GetReferences().AddReference(A47)
    print("   %-2s %-26s -> ConveyorBelt_A47" % (z, p))
    sys.stdout.flush()

print("\n[2] A23 시절 override 청소")
sys.stdout.flush()
_lay = st.GetRootLayer()
for z, p in JUNCTIONS:
    sp = _lay.GetPrimAtPath(Sdf.Path(p))
    if sp is None:
        continue
    for nm in list(sp.nameChildren.keys()):
        if nm in STALE or "A23" in nm:
            del sp.nameChildren[nm]
            print("   %-2s %-26s %s 제거" % (z, p, nm))
            sys.stdout.flush()

print("\n[3] 본선 롤러면 높이 맞추기")
sys.stdout.flush()
_ref = bb(st, REF_ROLLER)
if _ref is None:
    raise SystemExit("기준 벨트를 못 읽었다: %s" % REF_ROLLER)
_top = _ref[5]
print("   기준 롤러 윗면 %.1f mm" % _top)
sys.stdout.flush()
for z, p in JUNCTIONS:
    r = bb(st, p + "/Rollers")
    dz = _top - r[5]
    if abs(dz) < 0.05:
        print("   %-2s %-26s 이미 맞다 (%.1f)" % (z, p, r[5]))
        sys.stdout.flush()
        continue
    a = st.GetPrimAtPath(p).GetAttribute("xformOp:translate")
    v = a.Get()
    a.Set(Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]) + dz / 1000.0))
    print("   %-2s %-26s 롤러 윗면 %.1f -> %.1f  (%+.1f mm)"
          % (z, p, r[5], bb(st, p + "/Rollers")[5], dz))
    sys.stdout.flush()

print("\n[4] 소터 립 없애기 — 소터 롤러 윗면을 본선에 맞춘다")
sys.stdout.flush()
for z, p in JUNCTIONS:
    sp = p + "/Sorter"
    prim = st.GetPrimAtPath(sp)
    if not prim.IsValid():
        chk("%s 소터 존재" % z, False, "없다")
        continue
    order = prim.GetAttribute("xformOpOrder")
    print("   %-2s xformOpOrder = %s" % (z, list(order.Get() or [])))
    sys.stdout.flush()
    jsc = st.GetPrimAtPath(p).GetAttribute("xformOp:scale").Get()
    print("   %-2s 분기 scale = (%.4f, %.4f, %.4f)"
          % (z, jsc[0], jsc[1], jsc[2]))
    main_top = bb(st, p + "/Rollers")[5]
    s_top0 = bb(st, sp)[5]
    a = prim.GetAttribute("xformOp:translate")
    if not a:
        # 애셋이 translate op 을 안 들고 있으면 만들어서 순서 맨 앞에 넣는다.
        xf = UsdGeom.Xformable(prim)
        op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        op.Set(Gf.Vec3d(0, 0, 0))
        cur = [x for x in (order.Get() or []) if x != op.GetOpName()]
        xf.SetXformOpOrder([op] + [UsdGeom.XformOp(prim.GetAttribute(n))
                                   for n in cur])
        a = prim.GetAttribute("xformOp:translate")
        print("   %-2s translate op 을 새로 만들었다" % z)
    # ★ 한 번에 못 맞춘다. 여기 쓰는 값은 **소터의 로컬 z** 이고 분기 조각의
    #   scale.z 만큼 곱해져 월드로 나간다. 오전에 이걸 몰라 1.2 mm 가 남았다.
    #   배율을 계산해서 나누는 대신 **재고 고치기를 반복** 한다 — 변환 스택이
    #   어떻게 생겼든 맞는다.
    for _it in range(6):
        cur_top = bb(st, sp)[5]
        dz = main_top - cur_top
        if abs(dz) < 0.05:
            break
        v = a.Get() or Gf.Vec3d(0, 0, 0)
        a.Set(Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]) + dz / 1000.0))
    print("   %-2s 소터 윗면 %.1f -> %.1f  (본선 %.1f · %d 회 보정)"
          % (z, s_top0, bb(st, sp)[5], main_top, _it + 1))
    sys.stdout.flush()

print("\n[5] 소터를 돌린다 — SorterSpeed 0 -> 공칭 %.2f m/s" % NOMINAL_MPS)
sys.stdout.flush()
for z, p in JUNCTIONS:
    g = p + "/Sorter/ActionGraph"
    node, spd = g + "/conveyor_belt", g + "/SorterSpeed"
    gn = gain_of(st, node, p + "/Sorter/Sorter_physics")
    if gn is None or gn <= 0:
        chk("%s 소터 배율" % z, False, "%r" % gn)
        continue
    st.GetPrimAtPath(spd).GetAttribute("inputs:value").Set(NOMINAL_MPS / gn)
    # 진행 방향을 본선과 맞춘다 (월드 -> conveyorPrim 의 로컬 단위축)
    conv = st.GetPrimAtPath(p + "/Sorter/Sorter_physics")
    M = UsdGeom.XformCache().GetLocalToWorldTransform(conv).ExtractRotationMatrix()
    loc = []
    for k in range(3):
        row = [float(M[k][j]) for j in range(3)]
        nrm = (row[0] ** 2 + row[1] ** 2 + row[2] ** 2) ** 0.5 or 1.0
        loc.append(sum(THRU_WORLD[j] * row[j] for j in range(3)) / nrm)
    i = max(range(3), key=lambda k: abs(loc[k]))
    out = [0.0, 0.0, 0.0]
    out[i] = 1.0 if loc[i] > 0 else -1.0
    st.GetPrimAtPath(node).GetAttribute("inputs:direction").Set(Gf.Vec3f(*out))
    print("   %-2s direction -> %s (월드 %s)" % (z, out, list(THRU_WORLD)))
    # ★ 애셋 상태가 실제와 어긋나 보이지 않게 reroute 도 "직진" 으로 맞춰 둔다.
    #   (지금 이 값은 아무 데도 안 이어져 있어 동작에는 영향이 없다 — 위 docstring)
    rr = st.GetPrimAtPath(g + "/reroute").GetAttribute("inputs:value")
    if rr:
        rr.Set(False)
    print("   %-2s 배율 %.4f -> SorterSpeed %.4f (월드 %.2f m/s)"
          % (z, gn, NOMINAL_MPS / gn, NOMINAL_MPS))
    sys.stdout.flush()

print("\n[6] 소터 리지드바디를 **본선 롤러와 같게** 맞춘다")
sys.stdout.flush()
# ★ 값을 지어내지 않는다. 같은 씬에서 이미 잘 도는 본선 롤러가 어떻게 돼 있는지
#   읽어서 소터에 그대로 건다. (Sorter_physics 자신이 리지드바디다 — 그 밑에
#   `physics` 라는 자식은 없다. 애셋 조사 때 경로를 잘못 잘라 읽은 것이다.)
for z, p in JUNCTIONS:
    ph = st.GetPrimAtPath(p + "/Sorter/" + "Sorter_physics")
    ro = st.GetPrimAtPath(p + "/Rollers")
    if not ph.IsValid():
        chk("%s 소터 Sorter_physics" % z, False, "없다")
        continue
    rk = ro.GetAttribute("physics:kinematicEnabled")
    want = rk.Get() if rk else None
    k = ph.GetAttribute("physics:kinematicEnabled")
    cur = k.Get() if k else None
    print("   %-2s 본선 롤러 kinematic=%r · 소터 kinematic=%r · 소터 Rigid=%s"
          % (z, want, cur, ph.HasAPI(UsdPhysics.RigidBodyAPI)))
    if want is not None and cur != want:
        if not k:
            k = ph.CreateAttribute("physics:kinematicEnabled",
                                   Sdf.ValueTypeNames.Bool, True)
        k.Set(bool(want))
        print("   %-2s   -> %r 로 맞췄다" % (z, want))
    sys.stdout.flush()

print("\n[7] 프레임 충돌 끄기 (A47 이름)")
sys.stdout.flush()
# ★ 롤러면 위 23 mm 턱이 꺾인 박스를 막았다 (문서 §3). 참조를 갈면 자식 이름이
#   A23_* -> A47_* 로 바뀌므로 다시 걸어야 한다. 안 하면 고쳤던 문제가 되살아난다.
for z, p in JUNCTIONS:
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
        print("   %-2s %s 충돌 끔" % (z, n))
        sys.stdout.flush()
    if hit == 0:
        print("   %-2s !! 프레임 메시를 못 찾았다" % z)
        sys.stdout.flush()

st.GetRootLayer().Save()
print("\n저장했다: %s" % CUR)
sys.stdout.flush()

# ── 검증 ────────────────────────────────────────────────────────────
print("\n=== 검증 ===")
sys.stdout.flush()
_sorter_span = {}
for z, p in JUNCTIONS:
    print("--- 존 %s  %s" % (z, p))
    sys.stdout.flush()
    v = bb(st, p)
    r = bb(st, p + "/Rollers")
    r1 = bb(st, p + "/Rollers_01")
    so = bb(st, p + "/Sorter")
    chk("전체 bbox = 정본", v is not None
        and abs((v[3] - v[0]) - EXPECT_W_MM) < 2
        and abs((v[4] - v[1]) - EXPECT_D_MM) < 2,
        "%.1f x %.1f" % (v[3] - v[0], v[4] - v[1]) if v else "없다")
    chk("본선 롤러 윗면 = 기준", r is not None and abs(r[5] - _top) < 0.5,
        "%.1f (기준 %.1f)" % (r[5], _top) if r else "없다")
    chk("본선 롤러 폭 = 1546", r is not None and abs((r[4] - r[1]) - 1546.0) < 2,
        "%.1f" % (r[4] - r[1]) if r else "없다")
    chk("지선이 스토퍼까지", r1 is not None and r1[4] >= 4180.0,
        "Y %.1f~%.1f" % (r1[1], r1[4]) if r1 else "없다")
    chk("소터 윗면 = 본선", so is not None and abs(so[5] - r[5]) < 0.5,
        "%.1f (본선 %.1f)" % (so[5], r[5]) if so else "없다")
    if so:
        _sorter_span[z] = (so[0], so[3])
        print("     소터 구간 X %.1f ~ %.1f  (폭 %.1f)"
              % (so[0], so[3], so[3] - so[0]))
        sys.stdout.flush()
    # 소터 그래프
    g = p + "/Sorter/ActionGraph"
    nd = st.GetPrimAtPath(g + "/conveyor_belt")
    chk("소터 IsaacConveyor 있다", nd.IsValid())
    if nd.IsValid():
        tg = [str(x) for x in nd.GetRelationship("inputs:conveyorPrim").GetTargets()]
        chk("소터 conveyorPrim", tg == [p + "/Sorter/Sorter_physics"], "%s" % tg)
        sv = st.GetPrimAtPath(g + "/SorterSpeed").GetAttribute("inputs:value").Get()
        gn = gain_of(st, g + "/conveyor_belt", p + "/Sorter/Sorter_physics")
        chk("SorterSpeed != 0", sv and abs(sv * gn - NOMINAL_MPS) < 0.01,
            "%.4f x %.4f = %.3f m/s" % (sv or 0, gn or 0, (sv or 0) * (gn or 0)))
        # ★ 로컬 (-1,0,0) 이어야 월드 -X 로 민다. (1,0,0) 이면 본선과 맞선다.
        chk("소터 direction = 본선 진행",
            tuple(nd.GetAttribute("inputs:direction").Get()) == (-1.0, 0.0, 0.0),
            "%s" % (nd.GetAttribute("inputs:direction").Get(),))
    ph = st.GetPrimAtPath(p + "/Sorter/Sorter_physics")
    ro = st.GetPrimAtPath(p + "/Rollers")
    _rk = ro.GetAttribute("physics:kinematicEnabled")
    _sk = ph.GetAttribute("physics:kinematicEnabled")
    chk("소터 kinematic = 본선 롤러",
        (_rk.Get() if _rk else None) == (_sk.Get() if _sk else None),
        "롤러 %r · 소터 %r" % (_rk.Get() if _rk else None,
                              _sk.Get() if _sk else None))
    chk("소터가 리지드바디", ph.IsValid() and ph.HasAPI(UsdPhysics.RigidBodyAPI))
    # 본선 벨트 그래프가 여전히 롤러를 가리키는지 — 참조를 갈면 조용히 끊긴다
    for gp, tgt in ((p + "/ConveyorBeltGraph", p + "/Rollers"),
                    (p + "/ConveyorBeltGraph_01", p + "/Rollers_01")):
        n2 = st.GetPrimAtPath(gp + "/ConveyorNode")
        tg = []
        if n2 and n2.IsValid():
            rel = n2.GetRelationship("inputs:conveyorPrim")
            tg = [str(x) for x in rel.GetTargets()] if rel else []
        chk("%s -> 롤러" % gp.rsplit("/", 1)[-1],
            tg == [tgt] and st.GetPrimAtPath(tgt).IsValid(), "%s" % tg)
    for ch in st.GetPrimAtPath(p).GetChildren():
        if ch.GetName().startswith("SM_ConveyorBelt_"):
            en = ch.GetAttribute("physics:collisionEnabled")
            chk("프레임 %s 충돌 끔" % ch.GetName()[-9:],
                en is not None and en.Get() is False)
    chk("유령 Rubberbands_01 없다",
        not st.GetPrimAtPath(p + "/Rubberbands_01").IsValid())

print("\n--- 설정에 넣을 값 (test_3.yaml routing.sorter_x_mm)")
for z in ("A", "B", "C"):
    if z in _sorter_span:
        print("    %s: [%.0f, %.0f]" % (z, _sorter_span[z][0], _sorter_span[z][1]))
sys.stdout.flush()

print("=" * 70)
print("RESULT: %s" % ("통과" if not _fail else "실패 — %s" % ", ".join(_fail)))
print("=" * 70)
sys.stdout.flush()
os._exit(0)
