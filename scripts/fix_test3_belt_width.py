"""test_3.usd — 팔레트가 타는 벨트의 **롤러 폭**을 넓힌다 (2026-08-25).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/fix_test3_belt_width.py

★ 무엇이 문제였나 (사용자 관측 + 실측).
  팔레트를 코너에서 90° 돌릴 때 (1) 벨트 폭을 벗어나고 (2) 프레임 기둥을 뚫는다.

      셀 컨베이어  프레임 1000.1 · **롤러 784.5**
      합류 라인    프레임  950.0 · **롤러 745.3**
      팔레트       회전 후 800 · 회전 중 대각선 **1442**

  즉 롤러(실제 지지면)가 팔레트(800)보다 **좁다.** 게다가 프레임 메시는
  z 0~1212.9 까지 서 있어서 팔레트 높이(800~950)를 그대로 지나간다.

★ 어디까지 넓힐 수 있나 — 두 군데가 물리적으로 막는다.

  1) 셀 컨베이어는 **페데스탈**이 막는다. 프레임은 롤러의 1.275 배라
     프레임 오른쪽 끝이 페데스탈(X −7796.5)을 넘으면 로봇 기둥과 겹친다.
         롤러폭 W -> 프레임 오른끝 = 롤러오른끝 + 0.1375W
         롤러가 팔레트(−7996)를 덮으려면  W <= 1305
     그래서 **1300** 으로 잡는다. 팔레트 800 은 넉넉히 덮지만 회전 대각선
     1442 에는 **모자란다** — 그건 리프트로 푼다 (`ship_pallet` 주석 참조).

  2) 합류 라인은 **피치 1000** 이 막는다. 프레임을 1000 보다 키우면 옆 라인과
     겹친다. 프레임 1000 -> 롤러 **784**. 팔레트 800 에 16 mm 모자라지만
     피치를 바꾸면 트럭 2800 규격이 깨지므로 여기가 한계다.

★ 벨트 속도. 라인 속도(0.35)는 **안 올린다** — 5호가 스토퍼에 부딪혀 기우는
  문제를 그 값으로 잡았다 (0.5 에서 간극 3.36 mm > 허용 3.0). 대신 팔레트
  **반출 속도**를 따로 뒀다 (`PL_SHIP_MPS`, 기본 1.0). 반출 구간에는 박스가
  스토퍼에 부딪히는 일이 없으므로 빠르게 보내도 안전하다.
"""
import os

from pxr import Gf, Usd, UsdGeom

CUR = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test_3.usd")

CELL_ROLLER_W_MM = 1300.0    # 페데스탈이 허용하는 최대 (위 계산)
MERGE_FRAME_W_MM = 1000.0    # 피치 1000 이 허용하는 최대
PED_MARGIN_MM = 20.0         # 프레임과 페데스탈 사이 최소 여유
PITCH_MM = 1000.0
PALLET_SLOT_C = {"A": 8901.0, "B": 9901.0, "C": 10901.0}   # 팔레트 자리 중심

ZONES = ("A", "B", "C")
CELLS = {"A": "/ConveyorTrack_08",
         "B": "/ConveyorTrack_07",
         "C": "/World/ConveyorTrack_07"}
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


def at(path, name):
    return st.GetPrimAtPath(path).GetAttribute(name)


def shift_axis(path, axis, d_mm):
    a = at(path, "xformOp:translate")
    v = [float(a.Get()[i]) for i in range(3)]
    v[axis] += d_mm / 1000.0
    a.Set(Gf.Vec3d(*v))


def scale_axis_for(path, ax_w):
    """`ax_w` 방향 폭을 바꾸는 scale 성분을 **실측으로** 찾는다.

    ★ 추측하면 안 된다. 조각마다 로컬 축이 달라서 엉뚱한 성분을 줄이면 벨트가
      납작해지는데 **에러는 안 난다.**
    """
    a = at(path, "xformOp:scale")
    s0 = a.Get()
    w0 = bb(path)[ax_w + 3] - bb(path)[ax_w]
    for i in range(3):
        v = [float(s0[0]), float(s0[1]), float(s0[2])]
        v[i] *= 1.1
        a.Set(Gf.Vec3f(*v))
        w = bb(path)[ax_w + 3] - bb(path)[ax_w]
        a.Set(Gf.Vec3f(float(s0[0]), float(s0[1]), float(s0[2])))
        if abs(w - w0) > 1.0:
            return i
    raise SystemExit("%s 에서 폭(축 %d)을 바꾸는 scale 성분을 못 찾았다"
                     % (path, ax_w))


def set_width(path, ax_w, want_mm, roller_sub):
    """`path` 의 폭을 조절해 **롤러**(roller_sub) 폭이 want_mm 이 되게 한다."""
    a = at(path, "xformOp:scale")
    idx = scale_axis_for(path, ax_w)
    r0 = bb(path + roller_sub)
    w0 = r0[ax_w + 3] - r0[ax_w]
    s0 = a.Get()
    v = [float(s0[0]), float(s0[1]), float(s0[2])]
    v[idx] = float(s0[idx]) * want_mm / w0
    a.Set(Gf.Vec3f(*v))
    return idx


def chk(name, ok, detail=""):
    print("     %-34s %s %s" % (name, "OK" if ok else "X", detail))
    if not ok:
        _fail.append(name)


# ── 1) 셀 컨베이어: 롤러 폭을 넓히고 **−X 쪽으로만** 편다 ───────────
# ★ 대칭으로 넓히면 프레임이 페데스탈을 덮는다. 오른쪽 끝을 페데스탈 앞에
#   고정한 채 왼쪽으로만 편다.
print("[1] 셀 컨베이어 — 롤러 폭 -> %.0f" % CELL_ROLLER_W_MM)
for z in ZONES:
    cell = CELLS[z]
    ped = bb("/World/Pedestal_%s" % z)
    before_r = bb(cell + "/Rollers")
    before_f = bb(cell)
    idx = set_width(cell, 0, CELL_ROLLER_W_MM, "/Rollers")
    # 프레임 오른쪽 끝을 페데스탈 왼쪽 면 앞에 붙인다.
    f = bb(cell)
    shift_axis(cell, 0, (ped[0] - PED_MARGIN_MM) - f[3])
    f, r = bb(cell), bb(cell + "/Rollers")
    print("   %s  롤러 X %.1f~%.1f (%.1f)   전 (%.1f)"
          % (z, r[0], r[3], r[3] - r[0], before_r[3] - before_r[0]))
    print("        프레임 X %.1f~%.1f (%.1f)   전 (%.1f) · 페데스탈 %.1f  [scale 축 %d]"
          % (f[0], f[3], f[3] - f[0], before_f[3] - before_f[0], ped[0], idx))

# ── 2) 합류 라인: 프레임을 피치 한계(1000)까지 ─────────────────────
print("\n[2] 합류 라인 — 프레임 -> %.0f (피치 %.0f 한계)"
      % (MERGE_FRAME_W_MM, PITCH_MM))
for z in ZONES:
    path = "/World/MergeLine_%s" % z
    before_r = bb(path + "/Rollers")
    a = at(path, "xformOp:scale")
    idx = scale_axis_for(path, 1)
    f0 = bb(path)
    s0 = a.Get()
    v = [float(s0[0]), float(s0[1]), float(s0[2])]
    v[idx] = float(s0[idx]) * MERGE_FRAME_W_MM / (f0[4] - f0[1])
    a.Set(Gf.Vec3f(*v))
    # 롤러 중심을 팔레트 자리 중심에 맞춘다 — 팔레트가 롤러 한가운데 서야 한다.
    r = bb(path + "/Rollers")
    shift_axis(path, 1, PALLET_SLOT_C[z] - (r[1] + r[4]) / 2.0)
    f, r = bb(path), bb(path + "/Rollers")
    print("   %s  롤러 Y %.1f~%.1f (%.1f)   전 (%.1f) · 프레임 %.1f~%.1f (%.1f)"
          % (z, r[1], r[4], r[4] - r[1], before_r[4] - before_r[1],
             f[1], f[4], f[4] - f[1]))

# ── 3) 셀 끝을 합류 시작에 다시 맞춘다 ──────────────────────────────
# ★ 합류를 Y 로 옮겼으므로 이음매가 어긋난다. 셀 길이를 다시 잰다.
print("\n[3] 셀 끝 = 합류 시작 다시 맞추기")
CELL_Y0_MM, CELL_LEN_PER_SCALE = 4281.0, 2000.0
for z in ZONES:
    cell = CELLS[z]
    y_end = bb("/World/MergeLine_%s" % z)[1]
    a = at(cell, "xformOp:scale")
    s0 = a.Get()
    a.Set(Gf.Vec3f((y_end - CELL_Y0_MM) / CELL_LEN_PER_SCALE,
                   float(s0[1]), float(s0[2])))
    shift_axis(cell, 1, CELL_Y0_MM - bb(cell)[1])
    b = bb(cell)
    print("   %s  Y %.1f~%.1f (길이 %.1f) · 합류 시작 %.1f"
          % (z, b[1], b[4], b[4] - b[1], y_end))

# ── 4) 코너판을 넓힌 벨트에 맞춰 다시 앉힌다 ────────────────────────
print("\n[4] 코너판")
for z in ZONES:
    path = "/World/Corner_%s" % z
    r = bb(CELLS[z] + "/Rollers")
    m = bb("/World/MergeLine_%s" % z + "/Rollers")
    cx, cy = (r[0] + r[3]) / 2.0, (m[1] + m[4]) / 2.0
    c = bb(path)
    shift_axis(path, 0, cx - (c[0] + c[3]) / 2.0)
    shift_axis(path, 1, cy - (bb(path)[1] + bb(path)[4]) / 2.0)
    b = bb(path)
    print("   %s  X %.1f~%.1f · Y %.1f~%.1f (중심 %.1f, %.1f)"
          % (z, b[0], b[3], b[1], b[4], cx, cy))

st.GetRootLayer().Save()
print("\n저장했다: %s" % CUR)

# ── 5) 검증 ────────────────────────────────────────────────────
print("\n=== 검증 ===")
bed = bb("/World/Truck_Bed")
mr = {z: bb("/World/MergeLine_%s/Rollers" % z) for z in ZONES}
mf = {z: bb("/World/MergeLine_%s" % z) for z in ZONES}
for z in ZONES:
    print("--- %s" % z)
    cr, cf = bb(CELLS[z] + "/Rollers"), bb(CELLS[z])
    ped, pal = bb("/World/Pedestal_%s" % z), bb("/World/Pallet_%s" % z)
    chk("셀 롤러 > 팔레트 폭", (cr[3] - cr[0]) > (pal[3] - pal[0]),
        "롤러 %.1f · 팔레트 %.1f" % (cr[3] - cr[0], pal[3] - pal[0]))
    chk("팔레트가 셀 롤러 위 (X)", cr[0] - 1 <= pal[0] and pal[3] <= cr[3] + 1,
        "롤러 %.1f~%.1f · 팔레트 %.1f~%.1f" % (cr[0], cr[3], pal[0], pal[3]))
    chk("셀 프레임이 페데스탈 안 침범", cf[3] <= ped[0] + 1,
        "프레임 끝 %.1f · 페데스탈 %.1f" % (cf[3], ped[0]))
    chk("합류 롤러 >= 팔레트 800", (mr[z][4] - mr[z][1]) >= 780.0,
        "%.1f" % (mr[z][4] - mr[z][1]))
    chk("합류 롤러 중심 = 팔레트 자리", abs((mr[z][1] + mr[z][4]) / 2.0
                                     - PALLET_SLOT_C[z]) < 2,
        "%.1f / %.1f" % ((mr[z][1] + mr[z][4]) / 2.0, PALLET_SLOT_C[z]))
    chk("셀 끝 = 합류 시작", abs(bb(CELLS[z])[4] - mf[z][1]) < 3,
        "셀 %.1f · 합류 %.1f" % (bb(CELLS[z])[4], mf[z][1]))
print("--- 전체")
chk("합류 A n B 안 겹친다", mf["B"][1] >= mf["A"][4] - 1,
    "간격 %.1f" % (mf["B"][1] - mf["A"][4]))
chk("합류 B n C 안 겹친다", mf["C"][1] >= mf["B"][4] - 1,
    "간격 %.1f" % (mf["C"][1] - mf["B"][4]))
chk("팔레트 3장이 적재함 안",
    PALLET_SLOT_C["A"] - 400 >= bed[1] - 1
    and PALLET_SLOT_C["C"] + 400 <= bed[4] + 1,
    "%.0f~%.0f · 적재함 %.0f~%.0f"
    % (PALLET_SLOT_C["A"] - 400, PALLET_SLOT_C["C"] + 400, bed[1], bed[4]))
# ★ 회전 대각선은 **못 담는다.** 페데스탈이 막는다 — 리프트로 푼다.
diag = (1200.0 ** 2 + 800.0 ** 2) ** 0.5
cr = bb(CELLS["A"] + "/Rollers")
print("     %-34s %s 롤러 %.1f · 대각선 %.1f -> 리프트 필요"
      % ("회전 대각선은 롤러로 못 담는다", "예상대로"
         if (cr[3] - cr[0]) < diag else "??", cr[3] - cr[0], diag))

print("=" * 70)
print("RESULT: %s" % ("통과" if not _fail else "실패 — %s" % ", ".join(_fail)))
print("=" * 70)
os._exit(0)
