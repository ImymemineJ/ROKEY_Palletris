"""Isaac Sim Script Editor — 벨트가 실제로 박스를 옮겼나 확인만 한다.

    `script_editor_belt_demo.py` 를 돌리고 ▶ Play 를 몇 초 둔 뒤,
    Script Editor **새 탭(+ 버튼)** 에 이 파일을 붙여넣고 Ctrl+Enter.

씬을 건드리지 않고 읽기만 한다. 몇 번을 돌려도 안전하다.

스폰 당시 x 는 demo 스크립트가 각 박스 프림에 `palletris:spawn_x_mm` 로
적어 뒀다 — 그래서 **다른 탭에서도** 비교가 된다 (전역변수는 탭을 못 넘는다).
"""
import omni.usd
from pxr import UsdPhysics

MM = 1000.0
BOXES_GROUP = "/World/QR_Boxes"
BELT = "/World/TestScene/Belt_main"

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No stage open.")

grp = stage.GetPrimAtPath(BOXES_GROUP)
rows = []
if grp.IsValid():
    for prim in grp.GetChildren():
        t = prim.GetAttribute("xformOp:translate")
        t = t.Get() if t and t.IsValid() else None
        if t is None:
            continue
        s = prim.GetAttribute("palletris:spawn_x_mm")
        s = s.Get() if s and s.IsValid() else None
        v = prim.GetAttribute("physics:velocity")
        v = v.Get() if v and v.IsValid() else None
        rows.append((prim.GetName(), t[0] * MM, t[2] * MM, s,
                     v[0] if v is not None else None))

print("=" * 66)
print("  BELT CHECK")
print("=" * 66)
if not rows:
    print("  No boxes under /World/QR_Boxes.")
    print("  -> run script_editor_belt_demo.py first.")
else:
    moved = 0
    print(f"  {'box':10} {'x now':>9} {'x spawn':>9} {'moved':>9} {'z':>7} {'vx':>7}")
    for name, x, z, sx, vx in sorted(rows):
        d = (x - sx) if sx is not None else None
        if d is not None and abs(d) > 5:
            moved += 1
        print(f"  {name:10} {x:9.0f} "
              f"{('%9.0f' % sx) if sx is not None else '        ?'} "
              f"{('%+9.0f' % d) if d is not None else '        ?'} "
              f"{z:7.0f} {('%7.2f' % vx) if vx is not None else '      ?'}")

    # 벨트 쪽 상태 — 왜 안 도는지 좁히는 데 필요하다
    bp = stage.GetPrimAtPath(BELT)
    sv = None
    if bp.IsValid():
        a = bp.GetAttribute("physxSurfaceVelocity:surfaceVelocity")
        sv = a.Get() if a and a.IsValid() else None
    print()
    print(f"  belt surfaceVelocity : {tuple(round(q, 3) for q in sv) if sv else 'NOT SET'}")
    print()
    if moved:
        print(f"  ==> {moved}/{len(rows)} boxes moved.  THE BELT WORKS.")
    else:
        print("  ==> NOTHING MOVED.")
        print("      1) Play 를 눌렀나? (안 눌렀으면 물리가 아예 안 돈다)")
        print("      2) vx 열이 0 이면 속도가 안 들어간 것 —")
        print("         demo 의 DRIVE 를 \"push\" 로 바꾸고 Ctrl+Enter, Stop, Play.")
        print("      3) vx 가 0.5 인데 x 가 그대로면 무언가 붙잡고 있는 것 —")
        print("         박스가 벨트에 파묻혔는지(z 확인) 보라.")
print("=" * 66)
