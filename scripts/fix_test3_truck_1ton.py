"""test_3.usd — 1톤 트럭 규격(적재함 2800)으로 되돌린다 + 턴테이블 전제 (2026-08-25).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/fix_test3_truck_1ton.py

★ 왜. 사용자가 GUI 에서 적재함을 3600 -> 4565.7 로 넓히고 합류 라인 세 줄도
  1200 -> 1726.4 로 늘렸다. 그런데 세 줄 합(4722.7)이 적재함 안쪽(4438.9)보다 커서
  줄끼리 겹치고(A∩B 201.8 · B∩C 254.8) C 는 적재함 밖으로 166.2 mm 튀어나갔다.
  게다가 **1톤 트럭 규격은 2800** 이다 (작업명세서 §3.2 · CLAUDE.md).

★ 2800 에 팔레트 3장을 넣으려면 **코너에서 90° 돌려야 한다** (팀 결정 2026-08-25).
  안 돌리면 트럭을 가로지르는 치수가 팔레트 Y 변 1200 이라 3 x 1200 = 3600 > 2800.
  돌리면 그 치수가 800 이 되어 3 x 800 = 2400, 피치 1000 으로 2800 에 들어간다.
  회전 자체는 `ship_pallet` 이 키네마틱으로 돌린다 — 이 스크립트는 **치수만** 맞춘다.

      팔레트 자리  A 8501~9301 · B 9501~10301 · C 10501~11301  (각 800)
      합류 라인    중심 8901 / 9901 / 10901 · 폭 950 (사이 50)
      적재함      Y 8501~11301 = 2800   (벽 두께 50 은 바깥으로)

★ 같이 되돌리는 것 — Ctrl+S 가 **시뮬레이션 잔해까지** 저장했다.
      /World/Pallet_A   반출 끝 지점(트럭)에 굳음 -> 집으로
      /World/boxes      스폰된 박스 6 개가 본선 위에 남음 -> 삭제
      /World/CamPick_B  6.8 mm 돌아감 -> 09:49 자세로
"""
import os

from pxr import Gf, Sdf, Usd, UsdGeom

SC = ("/tmp/claude-1000/-home-rokey-palletris-ws/"
      "faffa520-b250-4933-87dd-59bf14c77599/scratchpad")
CUR = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test_3.usd")
# 합류 라인 폭·X 위치가 성한 마지막 판. 여기서 xform 을 되가져온 뒤 다시 재단한다.
REF = os.environ.get("PL_REF", SC + "/test_3.0825_0949.usd")

# ★ 회전 후 팔레트가 라인을 가로지르는 치수는 800 이다. 라인을 800 으로 두면
#   여유가 **0** 이라 조금만 틀어져도 반쪽이 허공에 뜬다. 피치 1000 안에서
#   최대한 넓힌 값이 950 (라인 사이 50 mm). 팔레트 기준 여유 150 mm.
MERGE_W_MM = 950.0
PITCH_MM = 1000.0         # 트럭 2800 에 3장(각 800)을 넣는 간격. 바꾸면 안 된다.
# 코너 회전판. 회전 전 팔레트는 X800 x Y1200 이고 돌면 X1200 x Y800 이다.
# 두 자세를 다 담으려면 한 변이 대각선 sqrt(1200^2+800^2)=1442 이상이어야 한다.
TURNTABLE_MM = 1500.0
TURNTABLE_TOP_MM = 797.5  # 롤러면(800)보다 2.5 낮게 — 브리지 판과 같은 규칙
PALLET_W_MM = 800.0       # 회전 후 팔레트가 라인을 가로지르는 치수
Y0_MM = 8501.0            # 팔레트 A 의 시작. v13 에서 검증된 값이다.
# 팔레트 중심 = 8901 / 9901 / 10901 -> 3장이 8501~11301 = 2800 에 꼭 맞는다.
# ★ 라인은 이 **중심**에 맞춘다. 라인 시작을 8501 에 맞추면 중심이 8976 으로 밀려
#   C 팔레트가 적재함을 75 mm 넘는다.
BED_LEN_MM = 2800.0       # 1톤 트럭 적재함
WALL_T_MM = 50.0          # 옆벽 두께 (바깥으로 붙인다)
CELL_LEN_PER_SCALE = 2000.0
CELL_Y0_MM = 4281.0       # 셀 시작 = 스토퍼 뒷면. 그대로.

ZONES = ("A", "B", "C")
CELLS = {"A": "/ConveyorTrack_08",
         "B": "/ConveyorTrack_07",
         "C": "/World/ConveyorTrack_07"}
PU = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]

st = Usd.Stage.Open(CUR)
ref = Usd.Stage.Open(REF)
_fail = []


def bb(stage, path):
    """월드 bbox [mm]. ★ 캐시를 매번 새로 만든다 — 재사용하면 옛 값이 나온다."""
    c = UsdGeom.BBoxCache(Usd.TimeCode.Default(), PU)
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid():
        return None
    r = c.ComputeWorldBound(p).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    a, b = r.GetMin(), r.GetMax()
    return [v * 1000.0 for v in (a[0], a[1], a[2], b[0], b[1], b[2])]


def at(stage, path, name):
    return stage.GetPrimAtPath(path).GetAttribute(name)


def shift_y(path, dy_mm):
    a = at(st, path, "xformOp:translate")
    v = a.Get()
    a.Set(Gf.Vec3d(float(v[0]), float(v[1]) + dy_mm / 1000.0, float(v[2])))


def shift_z(path, dz_mm):
    a = at(st, path, "xformOp:translate")
    v = a.Get()
    a.Set(Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]) + dz_mm / 1000.0))


def shift_axis(path, axis, d_mm):
    a = at(st, path, "xformOp:translate")
    v = [float(a.Get()[i]) for i in range(3)]
    v[axis] += d_mm / 1000.0
    a.Set(Gf.Vec3d(*v))


def set_span(path, axis, p0, p1):
    """bbox 의 `axis` 방향 폭을 (p1-p0) 으로 맞추고 시작을 p0 에 둔다."""
    return _set_span(path, axis, p0, p1)


def set_y_span(path, y0, y1):
    """bbox 의 Y 폭을 (y1-y0) 으로 맞추고 시작을 y0 에 둔다.

    ★ scale 의 **어느 성분이 Y 폭인지 모른다.** 조각마다 로컬 축이 다르다.
      그래서 성분을 하나씩 흔들어 보고 Y 폭이 실제로 변하는 놈을 찾는다.
      (추측해서 엉뚱한 축을 줄이면 라인이 납작해지는데 에러는 안 난다.)
    """
    return _set_span(path, 1, y0, y1)


def _set_span(path, ax_w, p0, p1):
    a = at(st, path, "xformOp:scale")
    s0 = a.Get()
    base = bb(st, path)
    w0 = base[ax_w + 3] - base[ax_w]
    axis = None
    for i in range(3):
        v = [float(s0[0]), float(s0[1]), float(s0[2])]
        v[i] *= 1.1
        a.Set(Gf.Vec3f(*v))
        nb = bb(st, path)
        w = nb[ax_w + 3] - nb[ax_w]
        a.Set(Gf.Vec3f(float(s0[0]), float(s0[1]), float(s0[2])))
        if abs(w - w0) > 1.0:
            axis = i
            break
    if axis is None:
        raise SystemExit("%s 에서 폭(축 %d)을 바꾸는 scale 성분을 못 찾았다"
                         % (path, ax_w))
    v = [float(s0[0]), float(s0[1]), float(s0[2])]
    v[axis] = float(s0[axis]) * (p1 - p0) / w0
    a.Set(Gf.Vec3f(*v))
    b = bb(st, path)
    shift_axis(path, ax_w, p0 - b[ax_w])
    return axis


def chk(name, ok, detail=""):
    print("     %-32s %s %s" % (name, "OK" if ok else "X", detail))
    if not ok:
        _fail.append(name)


# ── 1) 합류 라인: 09:49 xform 으로 되돌린 뒤 폭·피치 재단 ──────────
print("[1] 합류 라인 — 폭 %.0f · 피치 %.0f" % (MERGE_W_MM, PITCH_MM))
want, pal_c = {}, {}
for i, z in enumerate(ZONES):
    c = Y0_MM + PALLET_W_MM / 2.0 + i * PITCH_MM
    pal_c[z] = c
    want[z] = (c - MERGE_W_MM / 2.0, c + MERGE_W_MM / 2.0)
dy_by_zone = {}
for z in ZONES:
    path = "/World/MergeLine_%s" % z
    old = bb(st, path)
    src = UsdGeom.Xformable(ref.GetPrimAtPath(path))
    for o in src.GetOrderedXformOps():
        a = at(st, path, o.GetOpName())
        if a:
            a.Set(o.Get())
        else:
            print("     ! %s 에 %s 가 없다" % (path, o.GetOpName()))
    mid_ref = bb(st, path)
    y0, y1 = want[z]
    ax = set_y_span(path, y0, y1)
    b = bb(st, path)
    dy_by_zone[z] = (b[1] + b[4]) / 2.0 - (mid_ref[1] + mid_ref[4]) / 2.0
    print("   %s  Y %.1f~%.1f (폭 %.1f) · X %.1f~%.1f  [scale 축 %d]"
          % (z, b[1], b[4], b[4] - b[1], b[0], b[3], ax))
    print("        전 %.1f~%.1f (폭 %.1f)" % (old[1], old[4], old[4] - old[1]))

# ── 2) 적재함을 2800 으로 ────────────────────────────────────────
# ★ 팔레트 3장이 서는 구간(8501~11301)에 정확히 맞춘다. 벽은 바깥으로 붙인다.
print("\n[2] 적재함 — 1톤 규격 %.0f" % BED_LEN_MM)
bed0, bed1 = Y0_MM, Y0_MM + BED_LEN_MM
for path, (y0, y1) in (
        ("/World/Truck_Bed", (bed0, bed1)),
        ("/World/Truck_WallFar", (bed0, bed1)),
        ("/World/Truck_WallY0", (bed0 - WALL_T_MM, bed0)),
        ("/World/Truck_WallY1", (bed1, bed1 + WALL_T_MM))):
    if not st.GetPrimAtPath(path).IsValid():
        print("   %s 없다 — 건너뛴다" % path)
        continue
    old = bb(st, path)
    set_y_span(path, y0, y1)
    b = bb(st, path)
    print("   %-22s Y %.1f~%.1f (%.1f)   전 %.1f~%.1f (%.1f)"
          % (path.rsplit("/", 1)[-1], b[1], b[4], b[4] - b[1],
             old[1], old[4], old[4] - old[1]))

# ── 3) 코너 = 턴테이블 자리. 회전 반경을 담게 키운다 ──────────────
# ★ 왜 키우나. 코너 발판은 원래 셀 끝과 합류 시작 사이 틈만 메우는 판이었다
#   (905 x 267). 그런데 팔레트를 여기서 90° 돌리기로 했으므로 **두 자세를 다
#   담아야** 한다 — 회전 전 X800xY1200, 회전 후 X1200xY800. 한 변 1500 이면
#   대각선 1442 도 들어간다.
print("\n[3] 코너 = 턴테이블 (%.0f x %.0f)" % (TURNTABLE_MM, TURNTABLE_MM))
for z in ZONES:
    path = "/World/Corner_%s" % z
    if not st.GetPrimAtPath(path).IsValid():
        print("   %s 없다 — 건너뛴다" % z)
        continue
    cell = bb(st, CELLS[z])
    mg = bb(st, "/World/MergeLine_%s" % z)
    cx = (cell[0] + cell[3]) / 2.0          # 셀 컨베이어 X 중심 = 회전 중심 X
    cy = (mg[1] + mg[4]) / 2.0              # 합류 라인 Y 중심 = 회전 중심 Y
    h = TURNTABLE_MM / 2.0
    old_b = bb(st, path)
    set_span(path, 0, cx - h, cx + h)
    set_span(path, 1, cy - h, cy + h)
    b = bb(st, path)
    shift_z(path, TURNTABLE_TOP_MM - b[5])
    b = bb(st, path)
    print("   %s  X %.1f~%.1f · Y %.1f~%.1f · 상면 %.1f   전 X %.1f~%.1f · Y %.1f~%.1f"
          % (z, b[0], b[3], b[1], b[4], b[5],
             old_b[0], old_b[3], old_b[1], old_b[4]))

# ── 4) 셀 컨베이어 끝을 새 합류 시작에 맞춘다 ────────────────────
# ★ 어긋나면 팔레트가 이음매에서 떨어진다.
print("\n[4] 셀 컨베이어")
for z in ZONES:
    cell = CELLS[z]
    y_end = bb(st, "/World/MergeLine_%s" % z)[1]
    old = bb(st, cell)
    a = at(st, cell, "xformOp:scale")
    s0 = a.Get()
    a.Set(Gf.Vec3f((y_end - CELL_Y0_MM) / CELL_LEN_PER_SCALE,
                   float(s0[1]), float(s0[2])))
    b = bb(st, cell)
    shift_y(cell, CELL_Y0_MM - b[1])
    b2 = bb(st, cell)
    print("   %s  %-24s Y %.1f~%.1f (길이 %.1f)   전 %.1f~%.1f"
          % (z, cell, b2[1], b2[4], b2[4] - b2[1], old[1], old[4]))

# ── 5) Ctrl+S 가 박제한 시뮬레이션 잔해 ──────────────────────────
print("\n[5] 시뮬레이션 잔해")
cur = bb(st, "/World/Pallet_A")
tgt = bb(ref, "/World/Pallet_A")
tr = at(st, "/World/Pallet_A", "xformOp:translate")
t = tr.Get()
tr.Set(Gf.Vec3d(float(t[0]) + ((tgt[0] + tgt[3]) - (cur[0] + cur[3])) / 2000.0,
                float(t[1]) + ((tgt[1] + tgt[4]) - (cur[1] + cur[4])) / 2000.0,
                float(t[2]) + ((tgt[2] + tgt[5]) - (cur[2] + cur[5])) / 2000.0))
b = bb(st, "/World/Pallet_A")
print("   팔레트 A  X %.0f~%.0f · Y %.0f~%.0f  (집 %.0f~%.0f / %.0f~%.0f)"
      % (b[0], b[3], b[1], b[4], tgt[0], tgt[3], tgt[1], tgt[4]))

bx = st.GetPrimAtPath("/World/boxes")
if bx and bx.IsValid():
    names = [c.GetName() for c in bx.GetChildren()]
    st.RemovePrim(Sdf.Path("/World/boxes"))
    print("   /World/boxes 삭제 — 박제된 박스 %d 개 %s" % (len(names), names))
else:
    print("   /World/boxes 없다")

for o in UsdGeom.Xformable(ref.GetPrimAtPath("/World/CamPick_B")).GetOrderedXformOps():
    a = at(st, "/World/CamPick_B", o.GetOpName())
    if a:
        a.Set(o.Get())
b = bb(st, "/World/CamPick_B")
rb = bb(ref, "/World/CamPick_B")
print("   CamPick_B  Y %.1f~%.1f  (09:49 판 %.1f~%.1f)" % (b[1], b[4], rb[1], rb[4]))

st.GetRootLayer().Save()
print("\n저장했다: %s" % CUR)

# ── 6) 검증 ────────────────────────────────────────────────────
print("\n=== 검증 ===")
ms = {z: bb(st, "/World/MergeLine_%s" % z) for z in ZONES}
bed = bb(st, "/World/Truck_Bed")
for z in ZONES:
    m = ms[z]
    print("--- %s  합류 Y %.1f~%.1f" % (z, m[1], m[4]))
    chk("합류 폭 = %.0f" % MERGE_W_MM, abs((m[4] - m[1]) - MERGE_W_MM) < 3,
        "%.1f" % (m[4] - m[1]))
    # ★ 적재함 안에 들어가야 하는 건 **팔레트 자리**(중심 ±400)다. 라인 자체는
    #   적재함 밖(X < -2400)에 있으므로 폭이 조금 넘쳐도 벽에 안 닿는다.
    p0, p1 = pal_c[z] - PALLET_W_MM / 2.0, pal_c[z] + PALLET_W_MM / 2.0
    chk("팔레트 자리가 적재함 안", p0 >= bed[1] - 1 and p1 <= bed[4] + 1,
        "자리 %.1f~%.1f · 적재함 %.1f~%.1f" % (p0, p1, bed[1], bed[4]))
    c = bb(st, CELLS[z])
    chk("셀 끝 = 합류 시작", abs(c[4] - m[1]) < 3,
        "셀 %.1f · 합류 %.1f" % (c[4], m[1]))
    p = bb(st, "/World/Pallet_%s" % z)
    chk("팔레트가 셀 위", c[1] - 1 <= p[1] and p[4] <= c[4] + 1,
        "셀 %.0f~%.0f · 팔레트 %.0f~%.0f" % (c[1], c[4], p[1], p[4]))
    chk("합류가 적재함 입구까지", abs(m[3] - bed[0]) < 5,
        "합류 끝 X %.1f · 적재함 %.1f" % (m[3], bed[0]))

print("--- 전체")
chk("적재함 = %.0f" % BED_LEN_MM, abs((bed[4] - bed[1]) - BED_LEN_MM) < 3,
    "%.1f" % (bed[4] - bed[1]))
chk("A n B 안 겹친다", ms["B"][1] >= ms["A"][4] - 1,
    "간격 %.1f" % (ms["B"][1] - ms["A"][4]))
chk("B n C 안 겹친다", ms["C"][1] >= ms["B"][4] - 1,
    "간격 %.1f" % (ms["C"][1] - ms["B"][4]))
# ★ 적재함에 들어가는 건 라인이 아니라 **팔레트**다. 라인은 적재함 밖에 선다.
chk("회전 후 팔레트 3장 <= 적재함", 3 * PALLET_W_MM <= BED_LEN_MM,
    "%.0f <= %.0f  (여유 %.0f)"
    % (3 * PALLET_W_MM, BED_LEN_MM, BED_LEN_MM - 3 * PALLET_W_MM))
chk("/World/boxes 없다", not st.GetPrimAtPath("/World/boxes").IsValid())

print("=" * 70)
print("RESULT: %s" % ("통과" if not _fail else "실패 — %s" % ", ".join(_fail)))
print("=" * 70)
os._exit(0)
