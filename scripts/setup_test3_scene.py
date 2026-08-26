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
  [8] 스폰부 카메라를 만든다 (광학값은 CamPick 에서 복사)
  [9] 순환 루프 (방향 · 높이 · 프레임 · 레일)
  [10] 팔레트 가이드 벽 (overhang 7.8 고려 · 편도 여유 5)
  [11] ROS 2 브리지 — /clock · /tf
  [12] 적재 스테이션 가이드 벽 (로봇 쪽은 비움 · 끄면 치운다)
  [13] 로봇 · 페데스탈 배치 (이격 — 기둥도 같이 옮긴다)
  [14] 손목 카메라 3 대 (system.yaml 정본)
  [15] 전부 검증한다

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
import math
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

# ══ [8] 스폰부 카메라 ════════════════════════════════════════════════════════
# ★ 광학값을 **지어내지 않는다.** 이미 검증된 `CamPick_A` (HFOV 정확히 60°,
#   마지막 완주 3/3 · 오차 4.0 mm)에서 초점거리·조리개·클리핑을 그대로 복사한다.
#   높이도 픽업 카메라와 같게 둔다 — 벨트면이 같은 800 이라 시야 폭이 같아진다.
# ★ 위치는 `conveyor.spawn_point_mm` 에서 끌어온다. X 는 스폰 지점, Y 는 본선 중심
#   (스폰 y 는 -66 인데 지터가 +-150 이라 벨트 가운데가 더 안전하다).
say("\n[8] 스폰부 카메라")
CAM_SPAWN = str(cfg.get("cameras", {}).get("spawn_prim")
                or "/World/CamSpawn/Camera")
SRC_CAM = str(sorted(cfg["cameras"]["pick_prim"].values())[0])

src = st.GetPrimAtPath(SRC_CAM)
if not src.IsValid():
    chk("본보기 카메라 %s" % SRC_CAM, False)
else:
    spawn_mm = [float(v) for v in cfg["conveyor"]["spawn_point_mm"]]
    # 픽업 카메라의 월드 높이가 정본이다
    zw = UsdGeom.XformCache().GetLocalToWorldTransform(src) \
        .ExtractTranslation()[2] * 1000.0
    want = (spawn_mm[0], 0.0, zw)

    parent = CAM_SPAWN.rsplit("/", 1)[0]
    UsdGeom.Xform.Define(st, parent)
    cam = UsdGeom.Camera.Define(st, CAM_SPAWN)
    # 광학값 복사 — 없는 속성은 만들지 않는다 (본보기에 있는 것만 따라간다)
    for nm in ("focalLength", "horizontalAperture", "verticalAperture",
               "horizontalApertureOffset", "verticalApertureOffset",
               "clippingRange", "focusDistance", "fStop", "projection"):
        a = src.GetAttribute(nm)
        if a and a.IsValid() and a.Get() is not None:
            cam.GetPrim().CreateAttribute(nm, a.GetTypeName()).Set(a.Get())

    # ★ 자세는 부모 Xform 에 둔다 (픽업 카메라와 같은 구조: translate/orient/scale).
    #   기본 자세 그대로가 -Z 하향이라 회전을 안 준다 — 픽업 카메라도 그렇다.
    pp = st.GetPrimAtPath(parent)
    a = xop(pp, "translate", Gf.Vec3d(0, 0, 0))
    # 카메라 prim 자체에 오프셋이 있을 수 있으므로 **재고 고치기**로 맞춘다
    for _it in range(6):
        t = UsdGeom.XformCache().GetLocalToWorldTransform(cam.GetPrim()) \
            .ExtractTranslation()
        d = [want[k] - t[k] * 1000.0 for k in range(3)]
        if max(abs(v) for v in d) < 0.05:
            break
        c = a.Get()
        a.Set(Gf.Vec3d(*[float(c[k]) + d[k] / 1000.0 for k in range(3)]))
    t = UsdGeom.XformCache().GetLocalToWorldTransform(cam.GetPrim()) \
        .ExtractTranslation()
    fl = cam.GetPrim().GetAttribute("focalLength").Get()
    ha = cam.GetPrim().GetAttribute("horizontalAperture").Get()
    hfov = 2.0 * math.degrees(math.atan(float(ha) / (2.0 * float(fl))))
    belt = float(cfg["conveyor"]["belt_top_z_mm"])
    half = (t[2] * 1000.0 - belt) * math.tan(math.radians(hfov / 2.0))
    say("   %s  월드 (%.1f, %.1f, %.1f)" % (CAM_SPAWN, t[0] * 1000,
                                           t[1] * 1000, t[2] * 1000))
    say("   HFOV %.1f deg · 벨트면까지 %.0f mm · 가로 시야 %.0f mm (본선 폭 903)"
        % (hfov, t[2] * 1000 - belt, half * 2))

# ══ [9] 순환 루프 ═══════════════════════════════════════════════════════════
# 박스가 리턴 벨트에서 멈추던 것. 원인이 **6 개**였고 전부 실측으로 갈랐다
# (2026-08-25). 하나만 고치면 다음 것에서 다시 선다 — 순서대로 다 걸어야 한다.
#
#   1. `_01` 롤러면이 769.5 로 30.5 mm 낮다
#      들어갈 땐 떨어지지만 **나올 땐 못 올라온다.**
#   2. `_01` 커브 direction 부호가 반대
#      커브의 direction 은 진행 방향이 아니라 **회전축**이다.
#      -37 → 커브 입구 X -3620 에서 정지 · 속도를 올리면 벨트 밖으로 이탈.
#      +37 → 커브 완주.
#   3. 루프 조각의 **프레임 충돌이 켜져 있다**
#      프레임 꼭대기가 1166~1213 = 롤러면 위 366~413 mm 라 조각 이음매에서
#      벽이 된다. `_01`(Y 끝 -1395.2) 과 `_02`(Y 시작 -1394.3) 가 맞닿아 있었다.
#      본선 스폰 조각도 같이 끈다 — 그 옆벽(Y +-575.5)이 루프의 **합류**를 막았다
#      (실측: 박스가 Y -671 에서 정지, 앞면 -546 이 벽을 파고든 자리).
#   4. `Rail_C_far` 가 Y -790 에 서서 **루프 입구를 막는다**
#      X -23204~-16279 이라 `_04` 커브 위를 가로지른다. 본선 끝까지만 남긴다.
#   5. 3 번의 부작용 — 프레임을 끄면 **옆가드도 사라진다**
#      `_01` 커브 출구에서 박스가 +X 로 빠져 바닥까지 떨어졌다 (Z 105).
#      지선에서 쓴 방법과 같게 가드 레일을 세운다.
#   6. ★ `_02` 의 direction 이 틀렸다 — **`_local_axis` 환산을 믿으면 안 된다**
#      `_02` 롤러는 로컬축 길이가 0.472 / 1.000 / 1.040 로 심하게 비균등하다.
#      환산은 (1,0,0) 을 주는데 실측하면 그게 **월드 +X** 다:
#          (1,0,0)  dx +698  dy   -1   → 벨트 밖으로 떨어짐
#          (0,1,0)  dx   +0  dy +444   → 정상
#      이것 때문에 박스가 `_02` 위에 올라가고도 안 움직였다.
#
# ⚠ 한때 "`_02` 를 커브 출구(X -2208)로 옮긴다" 를 시도했는데 **필요 없었다.**
#   6 번을 고치자 원래 자리에서 그대로 통과했다. 증상이 비슷하다고 배치를
#   건드리면 진짜 원인을 덮는다.
#
# 검증: 본선 X -21931 에서 박스 하나를 놓고 `_04 -> _05 -> _01 -> _02 -> 본선`
#       을 다 지나는지 본다 (실측 27 초, 최저 Z 894.6 = 안 떨어짐).
say("\n[9] 순환 루프")
LOOP = (cfg["conveyor"].get("loop") or {})
if not LOOP:
    say("   설정에 conveyor.loop 가 없다 — 건너뛴다")
else:
    ref_top = bb(REF_ROLLER)[5]

    # 1. 롤러면을 기준(본선)에 맞춘다
    for tp in LOOP.get("level_to_ref") or []:
        prim = st.GetPrimAtPath(tp)
        if not prim.IsValid():
            chk("%s 존재" % tp, False)
            continue
        a = xop(prim, "translate", Gf.Vec3d(0, 0, 0))
        z0 = bb(tp + "/Rollers")[5]
        for _it in range(6):
            v = bb(tp + "/Rollers")
            d = ref_top - v[5]
            if abs(d) < 0.05:
                break
            t = a.Get()
            a.Set(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2]) + d / 1000.0))
        say("   %-30s 롤러면 %.1f -> %.1f" % (tp, z0, bb(tp + "/Rollers")[5]))

    # 2·6. direction — 커브는 회전축, 직선은 실측 월드축. 둘 다 값 그대로 쓴다.
    for key in ("curve_dir", "straight_dir"):
        for tp, d in (LOOP.get(key) or {}).items():
            nd = st.GetPrimAtPath(tp + "/ConveyorBeltGraph/ConveyorNode")
            if not nd or not nd.IsValid():
                chk("%s 의 ConveyorNode" % tp, False)
                continue
            before = nd.GetAttribute("inputs:direction").Get()
            nd.GetAttribute("inputs:direction").Set(
                Gf.Vec3f(*[float(x) for x in d]))
            say("   %-30s direction %s -> %s" % (tp, before, tuple(d)))

    # 3. 프레임 충돌 끄기
    noff = 0
    for tp in LOOP.get("frames_off") or []:
        prim = st.GetPrimAtPath(tp)
        if not prim.IsValid():
            chk("%s 존재" % tp, False)
            continue
        for ch in prim.GetChildren():
            nm = ch.GetName()
            if not nm.startswith("SM_") or "Decal" in nm:
                continue
            at = ch.GetAttribute("physics:collisionEnabled")
            if not at or not at.IsValid():
                at = ch.CreateAttribute("physics:collisionEnabled",
                                        Sdf.ValueTypeNames.Bool)
            at.Set(False)
            noff += 1
    say("   프레임 충돌 끔 %d 개" % noff)

    # 4. Rail_C_far 를 본선 끝까지 줄인다 (+X 끝은 그대로)
    rx = LOOP.get("rail_c_far_end_x_mm")
    RC = "/World/Rail_C_far"
    prim = st.GetPrimAtPath(RC)
    if rx is not None and prim.IsValid():
        v0 = bb(RC)
        sa = xop(prim, "scale", Gf.Vec3f(1, 1, 1))
        ta = xop(prim, "translate", Gf.Vec3d(0, 0, 0))
        keep, want = v0[3], v0[3] - float(rx)
        for _it in range(8):
            v = bb(RC)
            cur = v[3] - v[0]
            if abs(cur - want) > 0.05 and cur > 1e-6:
                sc = sa.Get()
                sa.Set(Gf.Vec3f(float(sc[0]) * (want / cur),
                                float(sc[1]), float(sc[2])))
            v = bb(RC)
            d = keep - v[3]
            if abs(d) > 0.05:
                t = ta.Get()
                ta.Set(Gf.Vec3d(float(t[0]) + d / 1000.0,
                                float(t[1]), float(t[2])))
        v1 = bb(RC)
        say("   Rail_C_far X %.1f~%.1f -> %.1f~%.1f" % (v0[0], v0[3], v1[0], v1[3]))
    elif rx is not None:
        chk("%s 존재" % RC, False)

    # 5. 가드 레일 (프레임을 끈 자리를 대신한다)
    def rail(name, x0, x1, y0, y1, z0, z1):
        """월드 좌표로 직육면체 가드. 재고 고치기로 정확히 맞춘다."""
        cu = UsdGeom.Cube.Define(st, name)
        cu.GetSizeAttr().Set(1.0)
        pr = cu.GetPrim()
        ta = xop(pr, "translate", Gf.Vec3d(0, 0, 0))
        sa = xop(pr, "scale", Gf.Vec3f(1, 1, 1))
        sa.Set(Gf.Vec3f((x1 - x0) / 1000.0, (y1 - y0) / 1000.0,
                        (z1 - z0) / 1000.0))
        ta.Set(Gf.Vec3d((x0 + x1) / 2000.0, (y0 + y1) / 2000.0,
                        (z0 + z1) / 2000.0))
        if not pr.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(pr)
        v = bb(name)
        say("   %-28s X %9.1f~%9.1f  Y %8.1f~%8.1f  Z %6.1f~%6.1f"
            % (name.rsplit("/", 1)[-1], v[0], v[3], v[1], v[4], v[2], v[5]))

    g = LOOP.get("guard_rail") or {}
    if g:
        v1r, v2r = bb("/World/ConveyorTrack_01/Rollers"), \
            bb("/World/ConveyorTrack_02/Rollers")
        th = float(g.get("thickness_mm", 20))
        gx = max(v1r[3], v2r[3])          # 안쪽 면을 롤러 가장자리에 붙인다
        rail(str(g["name"]), gx, gx + th,
             float(g["y_mm"][0]), float(g["y_mm"][1]),
             float(g["z_mm"][0]), float(g["z_mm"][1]))

    m = LOOP.get("main_rail") or {}
    if m:
        th = float(m.get("thickness_mm", 20))
        y, z0, z1 = float(m["y_mm"]), float(m["z_mm"][0]), float(m["z_mm"][1])
        x0, x1 = float(m["x_mm"][0]), float(m["x_mm"][1])
        gap = [float(v) for v in (m.get("merge_gap_x_mm") or [0, 0])]
        # +Y 쪽은 통으로. 루프는 -Y 에서 합류하므로 막지 않는다.
        rail("/World/MainRail_spawn_near", x0, x1, y, y + th, z0, z1)
        # -Y 쪽은 합류 창만 비운다
        rail("/World/MainRail_spawn_far0", x0, gap[0], -y - th, -y, z0, z1)
        rail("/World/MainRail_spawn_far1", gap[1], x1, -y - th, -y, z0, z1)

# ══ [10] 팔레트 가이드 벽 ═══════════════════════════════════════════════════
# 합류 라인에서 팔레트가 옆으로 흔들리지 않게 잡는다.
#
# ★★ 기준을 **롤러가 아니라 팔레트 가장자리로** 잡는다.
#    합류 롤러는 784.4 인데 팔레트는 800 이라 양쪽으로 각 **7.8 mm 씩 롤러 밖으로
#    나온다**(overhang, 실측). 롤러 가장자리(중심 +-392.2)에 가이드를 세우면
#    가이드가 팔레트를 물어 버린다.
#    -> 팔레트 가장자리(중심 +-400) 바깥으로 `clearance_mm` 만큼 띄운다.
#
# 지금 팔레트를 막는 것은 조각 프레임(중심 +-500)뿐이라 편도 여유가 100 mm 다.
# 가이드를 세우면 편도 5 mm 가 된다.
#
# 높이는 기존 `Rail_*` 와 같은 800~950 — 팔레트 몸통만 잡고 그 위 적재물
# (최대 1950)은 안 건드린다.
say("\n[10] 팔레트 가이드 벽")
PG = (cfg.get("truck", {}).get("pallet_guide") or {})
if not PG.get("enabled"):
    say("   꺼져 있다 (truck.pallet_guide.enabled)")
else:
    clr = float(PG.get("clearance_mm", 5))
    th = float(PG.get("thickness_mm", 20))
    z0, z1 = [float(v) for v in PG.get("z_mm", [800, 950])]
    fmt = str(PG.get("name_fmt", "/World/PalletGuide_%s_%s"))
    # 팔레트의 **가로(Y)** 치수. 커브를 돌고 나면 이 값이 라인을 가로지른다.
    pal_y = float(cfg["pallet"]["width_mm"])
    half = pal_y / 2.0 + clr

    for z in ("A", "B", "C"):
        mp = "/World/MergeLine_%s/Rollers" % z
        v = bb(mp)
        if not v:
            chk("%s 존재" % mp, False)
            continue
        cy = (v[1] + v[4]) / 2.0
        roll_w = v[4] - v[1]
        over = (pal_y - roll_w) / 2.0
        for suf, sign in (("lo", -1.0), ("hi", +1.0)):
            name = fmt % (z, suf)
            inner = cy + sign * half
            y0, y1 = sorted((inner, inner + sign * th))
            cu = UsdGeom.Cube.Define(st, name)
            cu.GetSizeAttr().Set(1.0)
            pr = cu.GetPrim()
            # ★ 순서가 반드시 [translate, scale] 이어야 한다.
            #   [scale, translate] 로 만들면 translate 가 scale 만큼 곱해져 나간다
            #   (실측 2026-08-25: X -4528.9 를 넣었는데 -19206.3 = x4.241 로 나왔다).
            #   이미 잘못 만들어진 것도 고쳐야 하므로 **매번 지우고 다시 쌓는다.**
            xf = UsdGeom.Xformable(pr)
            xf.ClearXformOpOrder()
            for a in ("xformOp:translate", "xformOp:scale"):
                if pr.HasAttribute(a):
                    pr.RemoveProperty(a)
            xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d((v[0] + v[3]) / 2000.0, (y0 + y1) / 2000.0,
                         (z0 + z1) / 2000.0))
            xf.AddScaleOp().Set(
                Gf.Vec3f((v[3] - v[0]) / 1000.0, (y1 - y0) / 1000.0,
                         (z1 - z0) / 1000.0))
            if not pr.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(pr)
        va, vb = bb(fmt % (z, "lo")), bb(fmt % (z, "hi"))
        say("   %s  롤러 %.1f (중심 %.1f) · 팔레트 %.0f -> overhang %+.2f mm/편"
            % (z, roll_w, cy, pal_y, over))
        say("       가이드 안쪽 %.1f ~ %.1f  (간격 %.1f · 편도 여유 %.1f)"
            % (va[4], vb[1], vb[1] - va[4], clr))
        say("       X %.1f~%.1f  Z %.1f~%.1f" % (va[0], va[3], va[2], va[5]))

# ══ [11] ROS 2 브리지 — /clock · /tf ════════════════════════════════════════
# ★ 실측 2026-08-26: 씬의 OmniGraph 26 개가 **전부 컨베이어**였고 ROS2 브리지
#   노드가 하나도 없었다. `/clock` 도 `/tf` 도 아무도 안 낸다.
#   그래서 `ros2 topic info /tf` 가 **Publisher 0 · Subscription 4** 였다 —
#   관제탑과 인식 노드 넷이 구독만 한 채 영원히 기다린다. 에러는 안 난다.
#   CLAUDE.md §3-4 규약 1 이 "모든 노드에 use_sim_time" 인데 `/clock` 이 없으면
#   그 노드들의 시계가 0 에 멈춘다.
#
# ★ 프레임 이름은 **prim 이름**이 된다. 그래서 카메라는 `.../Camera` 자식이 아니라
#   **부모 Xform** 을 넣는다 — 자식은 넷 다 이름이 `Camera` 라 프레임이 겹친다.
#
# ⚠ 카메라 이미지(`/cam1/rgb` 등)는 여기서 안 만든다. `ROS2CameraHelper` 는
#   `renderProductPath` 가 필요한데 그건 런타임에 만들어지는 물건이라 USD 에
#   그냥 적어 둘 수 없다. 별건으로 남긴다.
say("\n[11] ROS 2 브리지 — /clock · /tf")
ROS = (cfg.get("ros") or {})
G = str(ROS.get("graph_path", "/World/ROS2Bridge"))
tfc = ROS.get("tf") or {}
if not ROS:
    say("   설정에 ros 가 없다 — 건너뛴다")
else:
    import omni.graph.core as og

    K, C = og.Controller.Keys, og.Controller
    NS = "isaacsim.ros2.bridge."
    try:
        C.edit(
            {"graph_path": G, "evaluator_name": "execution"},
            {K.CREATE_NODES: [("Tick", "omni.graph.action.OnPlaybackTick"),
                              ("Ctx", NS + "ROS2Context"),
                              ("Clock", NS + "ROS2PublishClock"),
                              ("Tf", NS + "ROS2PublishTransformTree")],
             K.CONNECT: [("Tick.outputs:tick", "Clock.inputs:execIn"),
                         ("Tick.outputs:tick", "Tf.inputs:execIn"),
                         ("Ctx.outputs:context", "Clock.inputs:context"),
                         ("Ctx.outputs:context", "Tf.inputs:context")],
             # ★ ROS_DOMAIN_ID 를 씬에 박지 않는다 — 팀이 값을 바꾸면 씬도 고쳐야 한다
             K.SET_VALUES: [("Ctx.inputs:useDomainIDEnvVar", True),
                            ("Clock.inputs:topicName",
                             str(ROS.get("clock_topic", "/clock"))),
                            ("Tf.inputs:topicName",
                             str(tfc.get("topic", "/tf")))]})
    except Exception as e:
        say("   !! 그래프 생성 실패: %r" % (e,))

    # ★ 확장이 안 켜져 있으면 노드가 안 만들어진다 (헤드리스에서 브리지 .so 경로가
    #   없을 때). 그대로 만지면 `Accessed invalid null prim` 으로 **죽는다** —
    #   실측 2026-08-26. 그러면 뒤 단계까지 통째로 못 돈다.
    tf_node = st.GetPrimAtPath(G + "/Tf")
    if not tf_node.IsValid() or not st.GetPrimAtPath(G + "/Clock").IsValid():
        chk("ROS2 브리지 노드 생성", False,
            "확장 미기동 — LD_LIBRARY_PATH 에 "
            "$ISAAC_DIR/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib 를 넣고 다시 돌려라")
    else:
        par = str(tfc.get("parent_prim", "/World"))
        if st.GetPrimAtPath(par).IsValid():
            tf_node.GetRelationship("inputs:parentPrim").SetTargets(
                [Sdf.Path(par)])
        else:
            chk("tf parentPrim %s" % par, False)

        tgts, miss = [], []
        for tp in (tfc.get("target_prims") or []):
            (tgts if st.GetPrimAtPath(str(tp)).IsValid()
             else miss).append(str(tp))
        tf_node.GetRelationship("inputs:targetPrims").SetTargets(
            [Sdf.Path(t) for t in tgts])

        say("   그래프 %s" % G)
        say("   /clock  %s" % st.GetPrimAtPath(G + "/Clock")
            .GetAttribute("inputs:topicName").Get())
        say("   /tf     %s  · 부모 프레임 %s"
            % (tf_node.GetAttribute("inputs:topicName").Get(),
               par.rsplit("/", 1)[-1]))
        for t in tgts:
            say("       -> 프레임 %s" % t.rsplit("/", 1)[-1])
        for t in miss:
            chk("tf 대상 %s" % t, False, "씬에 없다")

# ══ [12] 적재 스테이션 가이드 벽 ═══════════════════════════════════════════
# 팔레트 옆에 세워 **적재 중 박스 낙하를 막는다.**
# 반출용 합류 라인 가이드([10])와 다른 물건이다 — 그건 팔레트를 트럭까지
# 곧게 보내는 것이고, 이건 로봇이 쌓는 동안 박스를 잡아 주는 것이다.
#
# ★★ 좌표를 여기 박지 않는다. `pallet_origin_world_mm` + `pallet.width/depth` 에서
#    유도한다. 두 설정의 **팔레트 방향이 서로 다르기 때문**이다:
#        system.yaml  1200(X) x 800(Y)   · 원점 z 150 (바닥)
#        test_3.yaml   800(X) x 1200(Y)  · 원점 z 950 (셀 컨베이어 위)
#    숫자를 박으면 한쪽이 반드시 틀린다.
#
# ★ 벽은 **진행 방향과 나란한 두 면**에만 선다. 팔레트가 셀을 타고 +Y 로
#   빠져나가야 하므로 앞뒤를 막으면 반출이 안 된다.
#
# ★ 로봇 쪽은 **비운다** (팀 결정 2026-08-26). 흡착 툴 반경 122.5 에 가장 좁은
#   박스 변 250 이면 가장자리 배치에서 여유가 7.5 mm 뿐이다:
#       박스 중심 400 - 125 = 275  ->  툴 바깥 275 + 122.5 = 397.5  vs  벽 405
#   어느 쪽이 로봇 쪽인지는 `robot_base_world_mm` 과 팔레트 중심을 비교해 정한다.
#
# ★ 유효 적재 면적은 **안 줄어든다** — 벽이 팔레트 바깥에 서므로 배치 가능
#   면적은 팔레트 그대로다. clearance 는 면적이 아니라 **허용 오버행**이다.
say("\n[12] 적재 스테이션 가이드 벽")
GW = ((cfg.get("pallet") or {}).get("guide_wall") or {})
WALL_FMT = str(GW.get("name_fmt", "/World/PalletWall_%s_%s"))
WALL_SUF = ("xlo", "xhi", "ylo", "yhi", "lo", "hi")   # "lo/hi" 는 2 면 시절 옛 이름
if not GW.get("enabled"):
    # ★ 끄면 **치워야** 한다. 예전에는 say 만 하고 넘어가서, enabled 를 false 로
    #   바꿔도 씬에 이미 선 벽이 그대로 남았다 — 껐는데 안 없어지는 상태가 된다.
    _off = []
    for z in sorted((cfg.get("zones") or {}).get("pallet_origin_world_mm") or {}):
        for suf in WALL_SUF:
            pr = st.GetPrimAtPath(WALL_FMT % (z, suf))
            if pr.IsValid() and pr.IsActive():
                pr.SetActive(False)
                _off.append("%s_%s" % (z, suf))
    say("   꺼져 있다 (pallet.guide_wall.enabled) — 치운 벽 %d 개%s"
        % (len(_off), (": " + ", ".join(_off)) if _off else ""))
else:
    pal = cfg["pallet"]
    pw, pd = float(pal["width_mm"]), float(pal["depth_mm"])
    clr = float(GW.get("clearance_mm", 5))
    th = float(GW.get("thickness_mm", 20))
    z0 = float(GW.get("z_from_mm", cfg["conveyor"]["belt_top_z_mm"]))
    fmt = WALL_FMT
    origins = cfg["zones"]["pallet_origin_world_mm"]
    robots = cfg["zones"]["robot_base_world_mm"]
    say("   팔레트 %.0f(X) x %.0f(Y) · 여유 %.0f -> 벽 안쪽 간격 **%.0f x %.0f**"
        % (pw, pd, clr, pw + 2 * clr, pd + 2 * clr))
    say("   유효 적재 면적은 팔레트 그대로 %.0f x %.0f (%.0f mm2)" % (pw, pd, pw * pd))

    # 옛 이름(2 면 시절)을 끈다 — 재고 고치기라 여러 번 돌아도 안전해야 한다
    for z in sorted(origins):
        for suf in ("lo", "hi"):
            old = st.GetPrimAtPath(fmt % (z, suf))
            if old.IsValid():
                old.SetActive(False)

    for z in sorted(origins):
        ox, oy, oz = [float(v) for v in origins[z]]
        cx, cy = ox + pw / 2.0, oy + pd / 2.0
        z1 = oz + float(pal["max_stack_mm"]) if str(GW.get("z_to")) == "stack" \
            else float(GW.get("z_to_mm", oz))
        rx, ry = float(robots[z][0]), float(robots[z][1])
        # 비울 면을 **유도**한다. 두 곳이다:
        #   (1) 로봇 베이스가 붙은 면
        #   (2) ★ 픽업 쪽 면 — 로봇이 박스를 **지선에서 집어 와** 팔레트로
        #       넘긴다. 그 사이에 벽을 세우면 팔이 박스를 들고 못 넘어간다.
        #       실측 2026-08-26: -Y 면을 세웠더니 **12/12 IK_FAIL** 이 났다
        #       (픽업 Y 4181 · 팔레트 Y 4476~ · 벽 Y 4451~4471 · 높이 800~1950).
        open_sides = set()
        if abs(rx - cx) >= abs(ry - cy):
            open_sides.add("xhi" if rx > cx else "xlo")
        else:
            open_sides.add("yhi" if ry > cy else "ylo")
        if GW.get("skip_pick_side", True):
            py = float(cfg["conveyor"]["stopper_x_mm"])      # 지선 끝 = 픽업 Y
            px = float(robots[z][0])                         # 지선 중심 = 로봇 X
            if abs(px - cx) >= abs(py - cy):
                open_sides.add("xhi" if px > cx else "xlo")
            else:
                open_sides.add("yhi" if py > cy else "ylo")
        robot_side = "/".join(sorted(open_sides))

        # 네 면. 모서리가 맞물리게 Y 면은 두께만큼 더 길게 뺀다.
        hx, hy = pw / 2.0 + clr, pd / 2.0 + clr
        faces = {
            "xlo": (cx - hx - th, cx - hx, cy - hy - th, cy + hy + th),
            "xhi": (cx + hx, cx + hx + th, cy - hy - th, cy + hy + th),
            "ylo": (cx - hx, cx + hx, cy - hy - th, cy - hy),
            "yhi": (cx - hx, cx + hx, cy + hy, cy + hy + th),
        }
        made = []
        for suf, (x0, x1, y0, y1) in faces.items():
            name = fmt % (z, suf)
            prim = st.GetPrimAtPath(name)
            if GW.get("skip_robot_side", True) and suf in open_sides:
                if prim.IsValid():
                    prim.SetActive(False)
                continue
            cu = UsdGeom.Cube.Define(st, name)
            cu.GetSizeAttr().Set(1.0)
            pr = cu.GetPrim()
            pr.SetActive(True)
            # ★ 순서는 반드시 [translate, scale] — 반대면 translate 가 scale 만큼
            #   곱해져 나간다 (실측 2026-08-25, [10] 에서 겪었다).
            xf = UsdGeom.Xformable(pr)
            xf.ClearXformOpOrder()
            for a in ("xformOp:translate", "xformOp:scale"):
                if pr.HasAttribute(a):
                    pr.RemoveProperty(a)
            xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d((x0 + x1) / 2000.0, (y0 + y1) / 2000.0,
                         (z0 + z1) / 2000.0))
            xf.AddScaleOp().Set(Gf.Vec3f((x1 - x0) / 1000.0, (y1 - y0) / 1000.0,
                                         (z1 - z0) / 1000.0))
            if not pr.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(pr)
            made.append(suf)
        say("   %s  팔레트 중심 (%.0f, %.0f) · 로봇 (%.0f, %.0f) -> **%s 비움**"
            % (z, cx, cy, rx, ry, robot_side))
        say("       세운 면 %s  ·  Z %.1f~%.1f" % ("/".join(sorted(made)), z0, z1))

# ══ [13] 로봇 · 페데스탈 배치 (이격) ════════════════════════════════════════
# ★ 로봇만 옮기면 **기둥 위에서 벗어나 허공에 선다.** 둘을 같이 옮긴다
#   (김병직 ee25497 이 test1-1 에서 겪은 것과 같은 함정).
# ★ 재고 고치기다 — 씬의 현재 위치를 재서 config 값까지의 차이만큼 민다.
#   여러 번 돌려도 같은 결과가 된다.
# ★ 지선 중심 = 로봇 X 라, 이 값을 바꾸면 [5]·[7] 이 지선·레일·스토퍼를 따라
#   옮긴다. 그래서 이 단계는 [5] 보다 **뒤에 와도 된다** — [5] 는 씬이 아니라
#   config 의 ROBOT_X 를 보기 때문이다.
say("\n[13] 로봇 · 페데스탈 배치")
_RB = cfg["zones"]["robot_base_world_mm"]
for z in sorted(_RB):
    want = [float(v) for v in _RB[z]]
    rp, pp = "/World/robot_%s" % z, "/World/Pedestal_%s" % z
    rprim = st.GetPrimAtPath(rp)
    if not rprim.IsValid():
        chk("로봇 %s" % rp, False, "없다")
        continue
    a = xop(rprim, "translate", Gf.Vec3d(0, 0, 0))
    cur = a.Get()
    dx = want[0] / 1000.0 - float(cur[0])
    dy = want[1] / 1000.0 - float(cur[1])
    dz = want[2] / 1000.0 - float(cur[2])
    if max(abs(dx), abs(dy), abs(dz)) < 1e-6:
        say("   %s 이미 맞다 (%.0f, %.0f, %.0f)" % (z, want[0], want[1], want[2]))
        continue
    a.Set(Gf.Vec3d(float(cur[0]) + dx, float(cur[1]) + dy, float(cur[2]) + dz))
    # 기둥도 **같은 만큼** 민다. 절대좌표로 쓰지 않는다 — 기둥은 원점이 밑면이라
    # 로봇 베이스(윗면)와 z 기준이 다르다.
    pprim = st.GetPrimAtPath(pp)
    if pprim.IsValid():
        b = xop(pprim, "translate", Gf.Vec3d(0, 0, 0))
        c2 = b.Get()
        b.Set(Gf.Vec3d(float(c2[0]) + dx, float(c2[1]) + dy, float(c2[2]) + dz))
    else:
        chk("페데스탈 %s" % pp, False, "없다 — 로봇만 옮기면 허공에 선다")
    say("   %s 로봇 %+.0f, %+.0f, %+.0f mm -> (%.0f, %.0f, %.0f) · 기둥 같이 이동"
        % (z, dx * 1000, dy * 1000, dz * 1000, want[0], want[1], want[2]))


# ══ [14] 손목 카메라 (구역 3 대) ═════════════════════════════════════════════
# ★ 왜 필요한가: `test1-1.usd` 에는 `/World/robot/gripper/CamWrist` 가 있는데
#   test_3 의 로봇 3 대에는 **하나도 없었다** (실측 2026-08-26). 로봇 애셋 참조와
#   SurfaceGripper 설정은 두 씬이 같으므로, 트랙 B 작업물 중 test_3 로 안 넘어온
#   것은 이 카메라뿐이다.
# ★ 값은 `system.yaml` 의 `recognition.wrist_cam` 정본을 읽는다. offset 170 도
#   tilt 18 도 실측으로 정해진 값이라 여기서 지어내지 않는다 — 100 이면 카메라가
#   그리퍼 몸통(반경 122.8) 안에 파묻혀 화소가 전부 0 으로 나온다.
# ★ 물리는 안 붙인다. 링크에 콜라이더나 질량을 더하면 아티큘레이션 동역학이 바뀐다.
say("\n[14] 손목 카메라")
_BASE_YAML = os.path.join(os.path.dirname(OVL), "system.yaml")
try:
    _sys = yaml.safe_load(open(_BASE_YAML, encoding="utf-8"))
except OSError as e:
    _sys = {}
    chk("system.yaml 읽기", False, str(e))
_rec = (_sys.get("recognition") or {})
_wc = (_rec.get("wrist_cam") or {})
if not _wc.get("enabled"):
    say("   꺼져 있다 (recognition.wrist_cam.enabled)")
else:
    wo = [float(v) / 1000.0 for v in _wc.get("offset_mm", [170, 0, 30])]
    whw, whd, whh = (float(v) / 1000.0 for v in _wc.get("housing_mm", [124, 26, 29]))
    tilt = math.radians(float(_wc.get("tilt_deg", 18)))
    res = [int(v) for v in _rec.get("resolution", [1280, 720])]
    hfov = float(_rec.get("hfov_deg", 60))
    ap_h = 36.0
    focal = ap_h / (2.0 * math.tan(math.radians(hfov) / 2.0))
    made = []
    for z in sorted(cfg["zones"]["robot_base_world_mm"]):
        gp = "/World/robot_%s/gripper" % z
        if not st.GetPrimAtPath(gp).IsValid():
            chk("그리퍼 %s" % gp, False, "없다 — 로봇 참조를 먼저 확인하라")
            continue
        cw = gp + "/CamWrist"
        # 재고 고치기: 이미 있으면 지우고 다시 만든다 (여러 번 돌려도 같은 결과)
        if st.GetPrimAtPath(cw).IsValid():
            st.RemovePrim(cw)
        xf = UsdGeom.Xform.Define(st, cw)
        pr = xf.GetPrim()
        xfa = UsdGeom.Xformable(pr)
        xfa.ClearXformOpOrder()
        xfa.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(wo[0], wo[1], wo[2]))
        # 시선 기울기 — 툴 +Y 둘레로 -tilt. (w,x,y,z) = (cos t/2, 0, -sin t/2, 0)
        xfa.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(math.cos(tilt / 2.0),
                     Gf.Vec3d(0.0, -math.sin(tilt / 2.0), 0.0)))

        body = UsdGeom.Cube.Define(st, cw + "/Body")
        body.GetSizeAttr().Set(1.0)
        bp = body.GetPrim()
        bxf = UsdGeom.Xformable(bp)
        bxf.ClearXformOpOrder()
        bxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, 0))
        # 긴 변(124)을 접선 방향으로 눕힌다 — 반경으로 두면 몸통을 파고든다
        if _wc.get("housing_tangential", True):
            bxf.AddScaleOp().Set(Gf.Vec3f(whh, whw, whd))
        else:
            bxf.AddScaleOp().Set(Gf.Vec3f(whw, whh, whd))
        bp.CreateAttribute("primvars:displayColor",
                           Sdf.ValueTypeNames.Color3fArray).Set(
            [Gf.Vec3f(0.10, 0.10, 0.12)])

        cam = UsdGeom.Camera.Define(st, cw + "/Camera")
        cp = cam.GetPrim()
        cp.CreateAttribute("focalLength", Sdf.ValueTypeNames.Float).Set(focal)
        cp.CreateAttribute("horizontalAperture", Sdf.ValueTypeNames.Float).Set(ap_h)
        cp.CreateAttribute("verticalAperture", Sdf.ValueTypeNames.Float).Set(
            ap_h * float(res[1]) / float(res[0]))
        cp.CreateAttribute("clippingRange", Sdf.ValueTypeNames.Float2).Set(
            Gf.Vec2f(0.02, 6.0))
        cxf = UsdGeom.Xformable(cp)
        cxf.ClearXformOpOrder()
        cxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.0, 0.0, whd / 2.0))
        # USD 카메라는 자기 -Z 를 본다. X 로 180도 돌려 부모 +Z(흡착 방향)를 보게 한다
        cxf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(0.0, Gf.Vec3d(1.0, 0.0, 0.0)))
        made.append(z)
    say("   만든 것 %s  ·  offset (%.0f, %.0f, %.0f) mm · 기울기 %.0f deg · "
        "%s %dx%d hfov %.0f deg"
        % ("/".join(made) or "없음", wo[0] * 1000, wo[1] * 1000, wo[2] * 1000,
           math.degrees(tilt), _wc.get("model"), res[0], res[1], hfov))


st.GetRootLayer().Save()
say("\n저장했다: %s" % CUR)

# ══ [15] 검증 ════════════════════════════════════════════════════════════════
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
