"""test_3.usd — **씬 설정을 한 번에 걸고 검증한다.** (2026-08-25)

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/setup_test3_scene.py

GUI 로 씬을 만지고 나면 이걸 돌린다. 전부 **재고 고치기(measure-and-correct)** 라
몇 번을 돌려도 같은 결과가 된다. 값은 `config/test_3.yaml` 에서 읽는다 —
여기에 좌표를 지어 넣지 않는다.

═══ 무엇을 거는가 ═════════════════════════════════════════════════════════════

  [1] 분기 3 곳을 `ConveyorBelt_A47`(소터 모듈)로 맞춘다
  [2] 본선 롤러면을 800 에 맞춘다
  [3] 소터를 켠다 — 립 제거 · 속도 · 방향 · kinematic
  [4] 프레임 충돌을 끈다 (분기 · 커브 · 본선 메움)
  [5] 지선을 **한 조각으로** 스토퍼까지 늘리고 로봇 중심에 맞춘다
  [6] 연장 조각을 비활성화한다
  [7] 지선 레일을 롤러 가장자리(±RAIL_HALF)로 맞춘다
  [8] 전부 검증한다

═══ 왜 이렇게 하는가 — 실측으로 얻은 것들 ═══════════════════════════════════

★ 소터 (A47). 오전에 세 번 붙였다 세 번 다 라인이 막혀 A23 으로 되돌렸었다.
  애셋을 열어 보고 원인이 나왔다:
    · **본선 롤러에 구멍이 있다** (로컬 X 1651~2555). 그 자리를 받치는 것이
      소터다. 소터를 끄면 박스가 바닥까지 떨어지고(z 0.14), 속도 0 으로 서
      있으면 라인이 거기서 막힌다(실측 165 s 정지).
    · **`SorterSpeed` 의 애셋 기본값이 0.0 이다.** "높이를 맞춰도 관성으로
      굴러가다 멎는다" 의 정체가 이것이었다.
    · **기본 `direction` 이 로컬 (1,0,0) = 월드 +X 인데 이 라인은 -X 로 흐른다.**
      그대로 켜면 소터가 본선과 맞서 민다(실측 X -7.11 에서 75 s 정지).
    · 소터 롤러가 본선보다 7.2 mm 높다 -> 턱. 내릴 때 분기 scale.z(1.0399)만큼
      곱해져 나가므로 **재고 고치기를 반복**해야 한다(한 번에 쓰면 1.2 mm 남는다).
  애셋 안에 분류 배선이 반쯤 있지만(`reroute`->`to_float`->x90°->`rotate_vector`)
  **`rotate_vector` 출력이 어디에도 안 이어져 있어** `reroute` 는 아무 일도 안 한다.
  그래서 `conveyor_ctl.set_sorter` 가 `inputs:direction` 을 직접 쓴다.

★ 지선을 왜 **늘리나** (연장 조각을 왜 안 쓰나).
  분기를 scale 1.0 으로 줄이면서 지선이 3303 -> 1930 이 됐고, 모자란 만큼을
  A05 직선 조각으로 이어 붙였었다. 그런데 **그 연장 조각이 박스를 벨트 속도로
  옆으로 민다** (실측: 이음매에서 0.5 초에 X +322 mm — 표류가 아니라 구동이다).
  `inputs:direction` 을 (1,0,0)/(0,1,0) 양쪽으로 넣어도 각각 +X / -X 로 밀렸다.
  이 조각은 회전이 **부모 prim** 에 걸려 있는데(orient 90°) `IsaacConveyor` 가
  그 회전을 일관되게 안 따른다 — 방향으로는 못 맞춘다.
  그래서 지선 prim(`Rollers_01`) 하나만 진행축으로 늘려 **이음매를 없앤다.**
  분기 조각 전체를 키우면 본선 폭까지 903 -> 1546 이 된다(L 자 한 덩어리라
  scale.y 가 둘을 겸한다) — 그건 안 한다.
  ⚠ 롤러도 같이 늘어나 원통이 길쭉해 보인다. 물리에는 지장 없다(윗면 800 유지).
  ⚠ 프레임은 안 늘어나므로 지선 뒤쪽엔 프레임이 없다. 박스는 `SpurRail_*` 이 잡는다.

★ 레일을 왜 롤러 가장자리로.
  원래 레일은 롤러 가장자리에서 **289~389 mm 바깥**이었다. 그 사이로 박스가
  빠진다(실측: 존 C 가 +597 mm 밀려 벨트 밖으로 나갔다. A·B 는 122~137 mm 라
  우연히 버텼다). 롤러 반폭 바로 바깥에 두면 밀려도 안 빠진다.

★ 프레임 충돌을 왜 끄나.
  A05/A47 프레임 메시는 롤러면 위로 **400 mm 넘게** 솟아 있고, 조각 끝에서는
  그것이 벽이 된다. 조각을 이어 붙이면 그 벽이 이음매 한가운데에 선다
  (실측: 박스가 Y 2.36 에서 120 s 넘게 정지). 롤러 충돌은 **안 건드린다** —
  그게 박스를 받치는 면이다.

═══ 여기에 **없는** 것 ════════════════════════════════════════════════════════

  트럭 적재함(1톤 2800) · 합류 라인 배치 · 커브 코너 · 셀 벨트 폭 — 이것들은
  씬에 이미 들어 있고 이 스크립트가 다시 만들지 않는다. **씬 파일이 정본이다.**
  되돌릴 지점: `test_3_v1.usd` (3 존 완주 `RESULT 통과` 시점).
"""
import os
import sys

import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

CUR = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test_3.usd")
OVL = os.environ.get("PL_CONFIG_OVERLAY",
                     "/home/rokey/palletris_ws/src/packing_bringup/config/test_3.yaml")
A47 = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
       "/Assets/Isaac/5.1/Isaac/Props/Conveyors/ConveyorBelt_A47.usd")

JUNC = {"A": "/World/ConveyorTrack_09", "B": "/World/ConveyorTrack_08",
        "C": "/World/ConveyorTrack_03"}
CURVE = {"A": "/World/ConveyorTrack_06", "B": "/ConveyorTrack_06",
         "C": "/ConveyorTrack_09"}
# 본선 사이를 메우는 직선 조각 (분기가 좁아지며 생긴 틈)
FILLER = ("/ConveyorTrack", "/ConveyorTrack_01")
# 지선을 한 조각으로 늘리면서 필요 없어진 연장 조각 — 지우지 않고 끈다
EXT_OFF = ("/World/ConveyorTrack_10", "/ConveyorTrack_10", "/ConveyorTrack_11")
REF_ROLLER = "/World/ConveyorTrack/Rollers"      # 롤러면 높이의 정본
A23_STALE = ("Rubberbands_01",)                  # A23 에만 있던 자식
THRU_WORLD = (-1.0, 0.0, 0.0)                    # 본선 진행 (스폰 -> 존 C)
RAIL_HALF_MM = float(os.environ.get("PL_RAIL_HALF", "470"))
PU = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]

cfg = yaml.safe_load(open(OVL, encoding="utf-8"))
NOMINAL_MPS = float(cfg["conveyor"]["belt_velocity_mps"])
ROBOT_X = {k: float(v[0]) for k, v in cfg["zones"]["robot_base_world_mm"].items()}
STOP_Y = float(cfg["conveyor"]["stopper_x_mm"])

st = Usd.Stage.Open(CUR)
_fail = []


def bb(path):
    """월드 bbox [x0,y0,z0,x1,y1,z1] (mm). ★ 캐시를 매번 새로 만든다 — 재사용하면
    값을 바꿔도 옛 bbox 가 나온다."""
    c = UsdGeom.BBoxCache(Usd.TimeCode.Default(), PU)
    p = st.GetPrimAtPath(path)
    if not p or not p.IsValid():
        return None
    r = c.ComputeWorldBound(p).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    a, b = r.GetMin(), r.GetMax()
    return [v * 1000.0 for v in (a[0], a[1], a[2], b[0], b[1], b[2])]


def chk(name, ok, detail=""):
    print("     %-40s %s %s" % (name, "OK" if ok else "X", detail))
    sys.stdout.flush()
    if not ok:
        _fail.append(name)


def say(msg):
    print(msg)
    sys.stdout.flush()


def xop(prim, name, default):
    """xformOp 이 없으면 만들어 준다 (순서 맨 앞)."""
    a = prim.GetAttribute("xformOp:" + name)
    if a:
        return a
    xf = UsdGeom.Xformable(prim)
    op = (xf.AddScaleOp() if name == "scale"
          else xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble))
    op.Set(default)
    return prim.GetAttribute("xformOp:" + name)


def local_axis(conv_path, world_v):
    """월드 방향을 conveyorPrim 의 **로컬 단위축**으로. `conveyor_ctl` 과 같은 식."""
    M = UsdGeom.XformCache().GetLocalToWorldTransform(
        st.GetPrimAtPath(conv_path)).ExtractRotationMatrix()
    loc = []
    for k in range(3):
        row = [float(M[k][j]) for j in range(3)]
        n = (row[0] ** 2 + row[1] ** 2 + row[2] ** 2) ** 0.5 or 1.0
        loc.append(sum(world_v[j] * row[j] for j in range(3)) / n)
    i = max(range(3), key=lambda k: abs(loc[k]))
    out = [0.0, 0.0, 0.0]
    out[i] = 1.0 if loc[i] > 0 else -1.0
    return out


def gain_of(node_path, conv_path):
    """IsaacConveyor 의 velocity -> 월드 m/s 배율."""
    nd = st.GetPrimAtPath(node_path)
    roll = st.GetPrimAtPath(conv_path)
    if not nd or not nd.IsValid() or not roll or not roll.IsValid():
        return None
    d = nd.GetAttribute("inputs:direction").Get()
    if d is None:
        return None
    M = UsdGeom.XformCache().GetLocalToWorldTransform(roll).ExtractRotationMatrix()
    wv = [sum(float(d[k]) * float(M[k][j]) for k in range(3)) for j in range(3)]
    return (wv[0] ** 2 + wv[1] ** 2 + wv[2] ** 2) ** 0.5


def frames_off(root, label):
    """조각의 프레임 메시 충돌만 끈다. 롤러는 안 건드린다."""
    prim = st.GetPrimAtPath(root)
    if not prim.IsValid() or not prim.IsActive():
        return 0
    n = 0
    for ch in prim.GetChildren():
        nm = ch.GetName()
        if not (nm.startswith("SM_ConveyorBelt_") and "Decal" not in nm):
            continue
        if not UsdPhysics.CollisionAPI(ch):
            UsdPhysics.CollisionAPI.Apply(ch)
        a = ch.GetAttribute("physics:collisionEnabled")
        if not a:
            a = ch.CreateAttribute("physics:collisionEnabled",
                                   Sdf.ValueTypeNames.Bool, True)
        was = a.Get()
        a.Set(False)
        v = bb(str(ch.GetPath()))
        say("   %-12s %-26s %-28s 충돌 %s -> False (롤러면 위 %.0f)"
            % (label, root, nm, was, (v[5] - 800.0) if v else 0))
        n += 1
    return n


# ══ [1] 분기를 A47 로 ═══════════════════════════════════════════════════════
say("[1] 분기 3 곳을 ConveyorBelt_A47 로 맞춘다")
need = []
for z, p in sorted(JUNC.items()):
    prim = st.GetPrimAtPath(p)
    got = []
    for spec in prim.GetPrimStack():
        for r in (list(spec.referenceList.prependedItems)
                  + list(spec.referenceList.explicitItems)):
            got.append(os.path.basename(r.assetPath))
    if "ConveyorBelt_A47.usd" in got:
        say("   %s %-26s 이미 A47" % (z, p))
    else:
        need.append((z, p, got))
if need:
    # ★ 원격 S3 애셋이라 첫 접근이 몇 분 걸린다. 먼저 받아 두지 않으면 참조를 건
    #   직후 컴포즈에서 **에러 없이** 멎는다 (오전에 11 분을 CPU 0% 로 날렸다).
    say("   A47 캐시 데우기 ...")
    if Usd.Stage.Open(A47) is None:
        raise SystemExit("A47 을 못 열었다: %s" % A47)
    for z, p, got in need:
        prim = st.GetPrimAtPath(p)
        prim.GetReferences().ClearReferences()
        prim.GetReferences().AddReference(A47)
        say("   %s %-26s %s -> A47" % (z, p, got))
    # A23 시절 override 청소 (남으면 유령 prim 이 되어 bbox 를 부풀린다)
    lay = st.GetRootLayer()
    for z, p, _ in need:
        sp = lay.GetPrimAtPath(Sdf.Path(p))
        if sp is None:
            continue
        for nm in list(sp.nameChildren.keys()):
            if nm in A23_STALE or "A23" in nm:
                del sp.nameChildren[nm]
                say("   %s %-26s %s 제거" % (z, p, nm))

# ══ [2] 본선 롤러면 800 ═════════════════════════════════════════════════════
say("\n[2] 본선 롤러면을 기준에 맞춘다")
ref = bb(REF_ROLLER)
if ref is None:
    raise SystemExit("기준 벨트를 못 읽었다: %s" % REF_ROLLER)
TOP = ref[5]
say("   기준 %s 윗면 %.1f" % (REF_ROLLER, TOP))
for z, p in sorted(JUNC.items()):
    a = xop(st.GetPrimAtPath(p), "translate", Gf.Vec3d(0, 0, 0))
    v0 = bb(p + "/Rollers")
    for _it in range(6):
        r = bb(p + "/Rollers")
        dz = TOP - r[5]
        if abs(dz) < 0.05:
            break
        t = a.Get()
        a.Set(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2]) + dz / 1000.0))
    say("   %s %-26s 윗면 %.1f -> %.1f" % (z, p, v0[5], bb(p + "/Rollers")[5]))

# ══ [3] 소터 ════════════════════════════════════════════════════════════════
say("\n[3] 소터 — 립 제거 · 속도 · 방향 · kinematic")
for z, p in sorted(JUNC.items()):
    sp = p + "/Sorter"
    prim = st.GetPrimAtPath(sp)
    if not prim.IsValid():
        chk("존 %s 소터" % z, False, "없다")
        continue
    # 립: 소터 윗면을 본선 롤러면에. 부모 scale.z 가 섞이므로 반복해서 맞춘다.
    main_top = bb(p + "/Rollers")[5]
    a = xop(prim, "translate", Gf.Vec3d(0, 0, 0))
    s0 = bb(sp)[5]
    for _it in range(6):
        dz = main_top - bb(sp)[5]
        if abs(dz) < 0.05:
            break
        t = a.Get()
        a.Set(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2]) + dz / 1000.0))
    # 속도 · 방향
    g = sp + "/ActionGraph"
    node, conv = g + "/conveyor_belt", sp + "/Sorter_physics"
    gn = gain_of(node, conv)
    if gn is None or gn <= 0:
        chk("존 %s 소터 배율" % z, False, "%r" % gn)
        continue
    st.GetPrimAtPath(g + "/SorterSpeed").GetAttribute("inputs:value") \
        .Set(NOMINAL_MPS / gn)
    st.GetPrimAtPath(node).GetAttribute("inputs:direction") \
        .Set(Gf.Vec3f(*local_axis(conv, THRU_WORLD)))
    rr = st.GetPrimAtPath(g + "/reroute").GetAttribute("inputs:value")
    if rr:
        rr.Set(False)          # 애셋 상태를 실제와 맞춰만 둔다 (동작에는 영향 없음)
    # kinematic 은 **본선 롤러와 같게** 맞춘다 (값을 지어내지 않는다)
    ph, ro = st.GetPrimAtPath(conv), st.GetPrimAtPath(p + "/Rollers")
    rk = ro.GetAttribute("physics:kinematicEnabled")
    want = rk.Get() if rk else None
    k = ph.GetAttribute("physics:kinematicEnabled")
    if want is not None and (k.Get() if k else None) != want:
        if not k:
            k = ph.CreateAttribute("physics:kinematicEnabled",
                                   Sdf.ValueTypeNames.Bool, True)
        k.Set(bool(want))
    say("   %s 윗면 %.1f -> %.1f (본선 %.1f) · 배율 %.4f -> SorterSpeed %.4f "
        "· direction %s"
        % (z, s0, bb(sp)[5], main_top, gn, NOMINAL_MPS / gn,
           st.GetPrimAtPath(node).GetAttribute("inputs:direction").Get()))

# ══ [4] 프레임 충돌 ═════════════════════════════════════════════════════════
say("\n[4] 프레임 충돌 끄기 (롤러는 그대로)")
for z, p in sorted(JUNC.items()):
    frames_off(p, "분기 %s" % z)
for z, p in sorted(CURVE.items()):
    frames_off(p, "커브 %s" % z)
for p in FILLER:
    frames_off(p, "본선 메움")

# ══ [5] 지선 늘리기 ═════════════════════════════════════════════════════════
say("\n[5] 지선을 한 조각으로 스토퍼까지 (진행축 = Rollers_01 로컬 X)")
for z, p in sorted(JUNC.items()):
    sp = p + "/Rollers_01"
    prim = st.GetPrimAtPath(sp)
    v0 = bb(sp)
    want_len = STOP_Y - v0[1]          # 시작점은 그대로, 끝만 스토퍼에
    sa = xop(prim, "scale", Gf.Vec3f(1, 1, 1))
    ta = xop(prim, "translate", Gf.Vec3d(0, 0, 0))
    for _it in range(10):
        v = bb(sp)
        cur = v[4] - v[1]
        if abs(cur - want_len) > 0.05 and cur > 1e-6:
            s = sa.Get()
            sa.Set(Gf.Vec3f(float(s[0]) * (want_len / cur), float(s[1]), float(s[2])))
        v = bb(sp)
        dx = ROBOT_X[z] - (v[0] + v[3]) / 2.0
        dy = STOP_Y - v[4]
        if abs(dx) > 0.05 or abs(dy) > 0.05:
            t = ta.Get()
            ta.Set(Gf.Vec3d(float(t[0]) + dx / 1000.0,
                            float(t[1]) + dy / 1000.0, float(t[2])))
        v = bb(sp)
        if (abs((v[4] - v[1]) - want_len) < 0.05
                and abs(ROBOT_X[z] - (v[0] + v[3]) / 2.0) < 0.05
                and abs(STOP_Y - v[4]) < 0.05):
            break
    v1 = bb(sp)
    say("   %s Y %.1f~%.1f (%.1f -> %.1f) · X 중심 %.1f (로봇 %.1f) · 폭 %.1f · %d 회"
        % (z, v1[1], v1[4], v0[4] - v0[1], v1[4] - v1[1],
           (v1[0] + v1[3]) / 2.0, ROBOT_X[z], v1[3] - v1[0], _it + 1))

# ══ [6] 연장 조각 끄기 ══════════════════════════════════════════════════════
say("\n[6] 연장 조각 비활성화 (지우지 않는다)")
for p in EXT_OFF:
    prim = st.GetPrimAtPath(p)
    if not prim.IsValid():
        say("   %-26s 없다" % p)
        continue
    was = prim.IsActive()
    prim.SetActive(False)
    say("   %-26s active %s -> False" % (p, was))

# ══ [7] 지선 레일 ═══════════════════════════════════════════════════════════
say("\n[7] 지선 레일을 지선 중심 ± %.0f mm 로" % RAIL_HALF_MM)
for z, p in sorted(JUNC.items()):
    sv = bb(p + "/Rollers_01")
    cx = (sv[0] + sv[3]) / 2.0
    for suf, sign in (("lo", -1.0), ("hi", +1.0)):
        rp = "/World/SpurRail_%s_%s" % (z, suf)
        prim = st.GetPrimAtPath(rp)
        if not prim.IsValid():
            chk("%s 존재" % rp, False)
            continue
        a = xop(prim, "translate", Gf.Vec3d(0, 0, 0))
        v0 = bb(rp)
        want = cx + sign * RAIL_HALF_MM
        for _it in range(6):
            v = bb(rp)
            d = want - (v[0] + v[3]) / 2.0
            if abs(d) < 0.05:
                break
            t = a.Get()
            a.Set(Gf.Vec3d(float(t[0]) + d / 1000.0, float(t[1]), float(t[2])))
        v1 = bb(rp)
        say("   %s_%s X 중심 %+9.1f -> %+9.1f (지선 대비 %+.1f)"
            % (z, suf, (v0[0] + v0[3]) / 2.0, (v1[0] + v1[3]) / 2.0,
               (v1[0] + v1[3]) / 2.0 - cx))

st.GetRootLayer().Save()
say("\n저장했다: %s" % CUR)

# ══ [8] 검증 ════════════════════════════════════════════════════════════════
say("\n" + "=" * 74)
say("=== 검증 ===")
for z, p in sorted(JUNC.items()):
    say("--- 존 %s  %s" % (z, p))
    m = bb(p + "/Rollers")
    s = bb(p + "/Rollers_01")
    so = bb(p + "/Sorter")
    stp = bb("/World/Stopper_%s" % z)
    lo, hi = bb("/World/SpurRail_%s_lo" % z), bb("/World/SpurRail_%s_hi" % z)
    chk("본선 롤러 윗면 = 기준", abs(m[5] - TOP) < 0.5, "%.1f (기준 %.1f)" % (m[5], TOP))
    chk("본선 롤러 Y 중심 0", abs((m[1] + m[4]) / 2.0) < 3.0, "%.1f~%.1f" % (m[1], m[4]))
    chk("지선 끝 = 스토퍼", abs(s[4] - STOP_Y) < 1.0, "%.1f vs %.1f" % (s[4], STOP_Y))
    chk("지선 중심 = 로봇 X", abs((s[0] + s[3]) / 2.0 - ROBOT_X[z]) < 1.0,
        "%.1f vs %.1f" % ((s[0] + s[3]) / 2.0, ROBOT_X[z]))
    chk("지선이 한 조각", (s[4] - s[1]) >= (STOP_Y - s[1]) - 1.0,
        "Y %.1f~%.1f 길이 %.1f" % (s[1], s[4], s[4] - s[1]))
    chk("지선 윗면 = 본선", abs(s[5] - m[5]) < 0.5, "%.1f vs %.1f" % (s[5], m[5]))
    chk("스토퍼가 지선 끝에", stp is not None and abs(stp[1] - s[4]) < 2,
        "Y %.1f" % stp[1] if stp else "없다")
    # 소터
    g = p + "/Sorter/ActionGraph"
    nd = st.GetPrimAtPath(g + "/conveyor_belt")
    chk("소터 IsaacConveyor", nd.IsValid())
    if nd.IsValid():
        tg = [str(x) for x in nd.GetRelationship("inputs:conveyorPrim").GetTargets()]
        chk("소터 conveyorPrim", tg == [p + "/Sorter/Sorter_physics"], "%s" % tg)
        chk("소터 윗면 = 본선", so is not None and abs(so[5] - m[5]) < 0.5,
            "%.1f vs %.1f" % (so[5], m[5]) if so else "없다")
        sv = st.GetPrimAtPath(g + "/SorterSpeed").GetAttribute("inputs:value").Get()
        gn = gain_of(nd.GetPath().pathString, p + "/Sorter/Sorter_physics")
        chk("소터 속도 = 공칭", sv and gn and abs(sv * gn - NOMINAL_MPS) < 0.01,
            "%.4f x %.4f = %.3f m/s" % (sv or 0, gn or 0, (sv or 0) * (gn or 0)))
        want_d = tuple(local_axis(p + "/Sorter/Sorter_physics", THRU_WORLD))
        chk("소터 direction = 본선 진행",
            tuple(nd.GetAttribute("inputs:direction").Get()) == want_d,
            "%s" % (nd.GetAttribute("inputs:direction").Get(),))
    # 벨트 그래프가 여전히 롤러를 가리키는가 (참조를 갈면 조용히 끊긴다)
    for gp, tgt in ((p + "/ConveyorBeltGraph", p + "/Rollers"),
                    (p + "/ConveyorBeltGraph_01", p + "/Rollers_01")):
        n2 = st.GetPrimAtPath(gp + "/ConveyorNode")
        tg = []
        if n2 and n2.IsValid():
            rel = n2.GetRelationship("inputs:conveyorPrim")
            tg = [str(x) for x in rel.GetTargets()] if rel else []
        chk("%s -> 롤러" % gp.rsplit("/", 1)[-1],
            tg == [tgt] and st.GetPrimAtPath(tgt).IsValid(), "%s" % tg)
    # 프레임
    for ch in st.GetPrimAtPath(p).GetChildren():
        nm = ch.GetName()
        if nm.startswith("SM_ConveyorBelt_") and "Decal" not in nm:
            en = ch.GetAttribute("physics:collisionEnabled")
            chk("프레임 %s 충돌 끔" % nm[-10:], en is not None and en.Get() is False)
    # 레일
    if lo and hi:
        clo, chi = (lo[0] + lo[3]) / 2.0, (hi[0] + hi[3]) / 2.0
        chk("레일 간격 = %.0f" % (2 * RAIL_HALF_MM),
            abs((chi - clo) - 2 * RAIL_HALF_MM) < 1.0,
            "%.1f (롤러 %.1f)" % (chi - clo, s[3] - s[0]))
        chk("지선 롤러가 레일 안", clo <= s[0] + 1 and s[3] <= chi + 1,
            "지선 %.1f~%.1f · 레일 %.1f~%.1f" % (s[0], s[3], clo, chi))
        chk("레일이 스토퍼까지", lo[4] >= STOP_Y - 1 and hi[4] >= STOP_Y - 1,
            "lo ~%.1f · hi ~%.1f" % (lo[4], hi[4]))

say("--- 끈 조각")
for p in EXT_OFF:
    prim = st.GetPrimAtPath(p)
    chk("%s 꺼짐" % p, (not prim.IsValid()) or (not prim.IsActive()))

# 본선이 스폰 구간부터 이어지는가
say("--- 본선 연결")
segs = sorted([(bb("/World/ConveyorTrack/Rollers")[0],
                bb("/World/ConveyorTrack/Rollers")[3], "스폰")]
              + [(bb(p + "/Rollers")[0], bb(p + "/Rollers")[3], "분기 " + z)
                 for z, p in JUNC.items()]
              + [(bb(p + "/Rollers")[0], bb(p + "/Rollers")[3], "메움")
                 for p in FILLER if bb(p + "/Rollers")], reverse=True)
prev = None
for a0, a1, lbl in sorted(segs, key=lambda t: -t[1]):
    gap = "" if prev is None else ("틈 %.1f mm" % (prev - a1) if prev > a1
                                   else "겹침 %.1f mm" % (a1 - prev))
    say("     %-8s X %9.1f~%9.1f   %s" % (lbl, a0, a1, gap))
    if prev is not None:
        chk("본선 %s 이어짐" % lbl, a1 >= prev - 1, gap)
    prev = min(prev, a0) if prev is not None else a0

say("=" * 74)
say("RESULT: %s" % ("통과" if not _fail else "실패 — %s" % ", ".join(_fail)))
say("=" * 74)
os._exit(0)
