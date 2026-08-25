"""씬의 카메라를 **찍어서 PNG 로 저장한다.** (2026-08-25)

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/shoot_cams.py

    PL_STAGE   찍을 씬 (기본: test_3.usd)
    PL_OUT     PNG 를 넣을 디렉터리 (기본: ~/palletris_ws/results/cams)
    PL_WARM    렌더 워밍업 프레임 수 (기본 60)
    PL_SPAWN   1 이면 박스를 하나 스폰해 두고 찍는다 (기본 1)

무엇을 찍나 — `cameras.spawn_prim` 1 대 + `cameras.pick_prim` 3 대.

★ 왜 워밍업이 필요한가. 오프스크린 렌더는 첫 프레임이 비어 있거나 절반만
  칠해져 나온다. 몇 프레임 돌린 뒤에 읽어야 제 그림이 나온다.
★ 조명. 오프스크린 렌더는 GUI 조명 리그를 못 받는다 (system.yaml §조명).
  씬에 돔이 꺼져 있으면 눈으로는 멀쩡한데 사진이 캄캄하다 — 밝기를 같이 찍어
  그 상황을 바로 알 수 있게 한다.
★ 이 스크립트는 **씬을 저장하지 않는다.** 박스를 스폰해도 메모리에서만이다.
"""
import os
import sys

import numpy as np
import yaml

ROOT = "/home/rokey/palletris_ws"
CUR = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test_3.usd")
OVL = os.environ.get("PL_CONFIG_OVERLAY",
                     ROOT + "/src/packing_bringup/config/test_3.yaml")
SYS = ROOT + "/src/packing_bringup/config/system.yaml"
OUT = os.environ.get("PL_OUT", ROOT + "/results/cams")
WARM = int(os.environ.get("PL_WARM", "60"))
DO_SPAWN = os.environ.get("PL_SPAWN", "1") not in ("0", "false", "")


def say(s):
    print(s)
    sys.stdout.flush()


cfg = yaml.safe_load(open(OVL, encoding="utf-8"))
sysc = yaml.safe_load(open(SYS, encoding="utf-8"))
RES = tuple(int(v) for v in sysc.get("recognition", {})
            .get("resolution", [1280, 720]))
os.makedirs(OUT, exist_ok=True)

import omni.usd
from isaacsim.core.utils.stage import open_stage
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

open_stage(CUR)
st = omni.usd.get_context().get_stage()
say("=== 씬 %s ===" % CUR)

cams = {}
sp = cfg.get("cameras", {}).get("spawn_prim")
if sp:
    cams["spawn"] = str(sp)
for z, p in sorted(cfg["cameras"]["pick_prim"].items()):
    cams["pick_" + str(z).upper()] = str(p)

missing = [k for k, v in cams.items() if not st.GetPrimAtPath(v).IsValid()]
for k in missing:
    say("  !! %-8s %s 가 씬에 없다 — setup_test3_scene.py 를 먼저 돌려라"
        % (k, cams[k]))
    cams.pop(k)
if not cams:
    say("찍을 카메라가 없다")
    os._exit(1)

# ── 박스를 하나 놓아 스폰 카메라에 뭔가 잡히게 한다 (메모리에서만) ──────────
if DO_SPAWN:
    sxyz = [float(v) for v in cfg["conveyor"]["spawn_point_mm"]]
    bt = sysc["boxes"]["types"][0]
    w, d, h = (float(bt["w_mm"]) / 1000.0, float(bt["d_mm"]) / 1000.0,
               float(bt["h_mm"]) / 1000.0)
    # ★ 색표가 있으면 **색마다 한 개씩** 늘어놓는다. 다음 단계(색 -> 배송지)에서
    #   HSV 로 가를 수 있는지 눈으로 먼저 확인하려는 것이다.
    pal = (cfg["boxes"].get("colors") or sysc["boxes"].get("colors") or {})
    items = sorted(pal.items()) or [("기본", (0.85, 0.15, 0.15))]
    pitch = 420.0
    x0 = sxyz[0] - pitch * (len(items) - 1) / 2.0
    for i, (cname, rgb) in enumerate(items):
        c = UsdGeom.Cube.Define(st, "/World/_shot_box_%d" % i)
        c.GetSizeAttr().Set(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d((x0 + pitch * i) / 1000.0,
                                        sxyz[1] / 1000.0,
                                        sxyz[2] / 1000.0 + h / 2.0))
        c.AddScaleOp().Set(Gf.Vec3f(w, d, h))
        UsdGeom.Gprim(c).CreateDisplayColorAttr(
            [Gf.Vec3f(*[float(v) for v in rgb])])
    say("  스폰 지점에 시험 박스 %d 개 (%s · %.0fx%.0fx%.0f mm · %s)"
        % (len(items), bt["name"], w * 1000, d * 1000, h * 1000,
           ", ".join(k for k, _ in items)))

# ── 렌더 ────────────────────────────────────────────────────────────────────
import omni.kit.app
import omni.replicator.core as rep
from isaacsim.sensors.camera import Camera

app = omni.kit.app.get_app()

# ★ 타임라인이 **돌기 시작한 뒤** 카메라를 초기화해야 한다.
#   멈춰 있으면 렌더 프로덕트가 갱신되지 않아 `get_rgba()` 가 영영 빈 배열이다
#   (실측 2026-08-25: 180 프레임을 기다려도 안 나왔다).
#   `camera_pick.start()` 가 "타임라인이 돌기 시작한 뒤 부른다" 인 이유가 이것이다.
import omni.timeline

tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(10):
    app.update()
say("  타임라인 play (렌더 프로덕트 갱신에 필요)")

objs = {}
for k, p in cams.items():
    cam = Camera(prim_path=p, resolution=RES)
    cam.initialize()
    objs[k] = cam
    say("  준비 %-8s %s" % (k, p))

say("  워밍업 %d 프레임 (오프스크린 첫 프레임은 비어 나온다)" % WARM)
for _ in range(WARM):
    app.update()


def grab(cam, tries=180):
    """제대로 된 (H, W, 4) 프레임이 나올 때까지 기다린다.

    ★ `initialize()` 직후에는 `get_rgba()` 가 **빈 1 차원 배열**을 준다
      (실측 2026-08-25: 워밍업 60 프레임으로도 부족해 AxisError 로 죽었다).
      프레임 수를 늘려 찍는 대신 **모양을 보고 될 때까지** 돌린다.
    """
    for i in range(tries):
        a = cam.get_rgba()
        if a is not None:
            a = np.asarray(a)
            if a.ndim == 3 and a.shape[0] > 1 and a.shape[2] >= 3:
                return a, i
        app.update()
    return None, tries

try:
    from PIL import Image
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False
    say("  !! PIL 이 없다 — .npy 로 떨군다")

say("")
say("%-9s %-34s %8s %8s %s" % ("이름", "prim", "평균밝기", "검정비율", "파일"))
say("-" * 92)
ok = True
for k, cam in objs.items():
    a, waited = grab(cam)
    if a is None:
        say("  %-8s 프레임 없음 (대기해도 안 나옴)" % k)
        ok = False
        continue
    rgb = a[..., :3]
    if rgb.dtype != np.uint8:
        rgb = (np.clip(rgb, 0.0, 1.0) * 255.0).astype(np.uint8)
    mean = float(rgb.mean())
    black = float((rgb.max(axis=2) == 0).mean())
    fn = os.path.join(OUT, "%s.png" % k)
    if HAVE_PIL:
        Image.fromarray(rgb).save(fn)
    else:
        fn = os.path.join(OUT, "%s.npy" % k)
        np.save(fn, rgb)
    say("%-9s %-34s %8.1f %8.2f %s%s"
        % (k, cams[k], mean, black, fn,
           "" if waited == 0 else "  (+%d 프레임)" % waited))
    # 캄캄하면 조명 문제다 (system.yaml §조명). 실측 기준: 픽업 196 · 손목 233
    if mean < 20.0:
        say("          !! 캄캄하다 — 돔 라이트가 꺼졌거나 카메라가 형상 안에 묻혔다")
        ok = False

say("")
say("=" * 92)
say("RESULT: %s — 카메라 %d 대" % ("통과" if ok else "실패", len(objs)))
say("=" * 92)
say("@@@ 끝 — 씬을 저장하지 않는다")
os._exit(0)
