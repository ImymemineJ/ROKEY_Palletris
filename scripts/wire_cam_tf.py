"""씬의 카메라를 **`/tf` 에 올린다** (+ `/clock`). 기하는 안 건드린다.

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/wire_cam_tf.py

★ 왜 `setup_test3_scene.py` 를 안 쓰나. 그쪽 [11] 이 같은 그래프를 만들지만
  [8] 스폰부 카메라·[14] 손목 카메라·[13] 로봇 배치까지 같이 돈다. 사람이 방금
  GUI 로 놓은 카메라를 그 단계들이 **옮기거나 덮어쓴다.** 그래서 TF 만 뗀다.

★ 프레임 이름 = **prim 이름** (`ROS2PublishTransformTree` 규칙).
  카메라 본체는 보통 다섯 다 이름이 `Camera` 라 프레임이 충돌한다. 그래서
  TF 대상에는 **부모 Xform** 을 넣는다 — 그 이름이 프레임이 된다.
  `vision_node.py:161` 이 이미지 헤더의 `frame_id` 로 tf 를 조회하므로,
  나중에 이미지 토픽의 `frameId` 를 이 이름과 같게 주면 그대로 맞물린다.

★ `/clock` 을 같이 내는 이유. 팀 규약이 "모든 노드에 `use_sim_time:=true`" 인데
  `/clock` 발행자가 없으면 ROS 시계가 0 에 멎어 **타이머가 하나도 안 뜬다**
  (CLAUDE.md §10). tf 도 timeStamp 가 있어야 조회가 된다.

    PL_STAGE       열 씬 (기본 Collected_test_3/test_3.usd)
    PL_TF_DRY=1    저장하지 않고 무엇을 넣을지만 보여 준다
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import omni.usd                                                # noqa: E402
from pxr import Sdf, Usd, UsdGeom                              # noqa: E402

import _kitexit                                                # noqa: E402
import _paths                                                  # noqa: E402

STAGE = os.environ.get(
    "PL_STAGE", "/home/rokey/isaac_scenes/Collected_test_3/test_3.usd")
DRY = os.environ.get("PL_TF_DRY", "0") not in ("", "0", "false", "no")

cfg = _paths.system_config()
ROS = (cfg.get("ros") or {})
G = str(ROS.get("graph_path", "/World/ROS2Bridge"))
tfc = ROS.get("tf") or {}
PARENT = str(tfc.get("parent_prim", "/World"))

_fail = []


def say(*a):
    print(*a, flush=True)


def die(msg, code=1):
    """★ `raise SystemExit` 은 Kit 의 `--exec` 에서 **앱을 안 닫는다** (실측
    2026-08-26: 크래시한 헤드리스 두 판이 GPU 를 쥔 채 계속 떠 있었다).
    반드시 `_kitexit.shutdown` 으로 끊는다."""
    say("\n!! %s" % msg)
    _kitexit.shutdown(code)
    raise SystemExit(code)          # ★ shutdown 은 비동기다. 여기서 같이 멈춘다


def chk(name, ok, detail=""):
    say("     %-40s %s %s" % (name, "OK" if ok else "X", detail))
    if not ok:
        _fail.append(name)


omni.usd.get_context().open_stage(STAGE)
st = omni.usd.get_context().get_stage()
if st is None:
    die("씬을 못 열었다: %s" % STAGE)
say("\n씬  %s" % STAGE)

# ── [1] 카메라를 찾는다 ────────────────────────────────────────────────────
# ★ 이름을 지어내지 않는다. 씬의 Camera prim 을 전수로 잡고, 그것이 속한
#   **`/World` 바로 아래 prim** 을 프레임으로 쓴다.
#
#   ⚠ 한 단계만 올라가면 안 된다 (2026-08-26 실측). rsd455 는 **리그**라
#     구조가 `/World/rsd455_a/RSD455/Camera_Pseudo_Depth` 다. 부모는
#     `RSD455` 이고 그 이름은 **여섯 리그가 전부 같다** — 프레임이 통째로
#     충돌한다. 실제로 다섯이 다 `RSD455` 로 나왔다.
#
#   ⚠ `/OmniverseKit_Persp` 같은 세션 카메라는 `/World` 밖이라 조상 사슬이
#     pseudo-root 로 끝난다. 그걸 relationship 에 넣으면 USD 가 죽는다
#     ("Relationship target paths must be absolute prim..."). 그래서 걸러낸다.
say("\n[1] 카메라 찾기")


def frame_prim(cam):
    """`cam` 이 속한 **`/World` 바로 아래** prim. 그 이름이 tf 프레임이 된다."""
    p = cam
    while p and p.IsValid():
        par = p.GetParent()
        if not par or not par.IsValid() or par.IsPseudoRoot():
            return None                      # /World 밖 (뷰포트 카메라 등)
        if str(par.GetPath()) == PARENT:
            return p
        p = par
    return None


cams = [p for p in st.TraverseAll() if p.GetTypeName() == "Camera"]
rigs, skipped = {}, []
xc = UsdGeom.XformCache()
for c in cams:
    f = frame_prim(c)
    if f is None:
        skipped.append(str(c.GetPath()))
        continue
    rigs.setdefault(str(f.GetPath()), []).append(c)

frames = []
for path in sorted(rigs):
    lenses = rigs[path]
    t = xc.GetLocalToWorldTransform(lenses[0]).ExtractTranslation()
    mm = (t[0] * 1000, t[1] * 1000, t[2] * 1000)
    # ★ 원점에 그대로 있는 리그는 **놓다 만 것**이다. tf 에 올리면 프레임이
    #   월드 원점에 생겨 인식이 엉뚱한 자세를 읽는다. 크게 알리고 뺀다.
    if max(abs(v) for v in mm) < 1.0:
        say("   %-24s ⚠ 원점 (0,0,0) 에 있다 — 놓다 만 리그로 보고 **건너뛴다**"
            % path.rsplit("/", 1)[-1])
        skipped.append(path + "  (원점)")
        continue
    frames.append(path)
    say("   %-24s (%8.0f, %8.0f, %8.0f) mm   렌즈 %d 개"
        % (path.rsplit("/", 1)[-1], mm[0], mm[1], mm[2], len(lenses)))
    for c in lenses:
        say("        %s" % str(c.GetPath()).rsplit("/", 1)[-1])
if not frames:
    say("   !! 쓸 수 있는 카메라가 없다 — 씬을 저장했는지 확인해라")
for sp in skipped:
    say("   (건너뜀) %s" % sp)

# ★ 카메라 말고도 tf 에 올릴 것 (로봇 등)은 설정이 갖고 있다. 씬에 있는 것만 남긴다.
seen = set(frames)
extra = [str(t) for t in (tfc.get("target_prims") or [])
         if st.GetPrimAtPath(str(t)).IsValid() and str(t) not in seen]
for e in extra:
    say("   %-24s (설정 target_prims)" % e.rsplit("/", 1)[-1])
targets = frames + extra

if DRY:
    say("\nPL_TF_DRY — 저장하지 않는다. 대상 %d 개." % len(targets))
    _kitexit.shutdown(0)
    raise SystemExit(0)

# ── [2] ROS 2 브리지 그래프 ────────────────────────────────────────────────
say("\n[2] ROS 2 브리지 — /clock · /tf")
from isaacsim.core.utils.extensions import enable_extension        # noqa: E402

_lib = os.path.expanduser("~/isaacsim/exts/isaacsim.ros2.bridge/%s/lib"
                          % os.environ.get("ROS_DISTRO", "jazzy"))
if _lib not in os.environ.get("LD_LIBRARY_PATH", ""):
    say("   ⚠ LD_LIBRARY_PATH 에 브릿지 경로가 없다. 실패하면 먼저:  isaac_ros")
enable_extension("isaacsim.core.nodes")
if not enable_extension("isaacsim.ros2.bridge"):
    die("isaacsim.ros2.bridge 를 못 켰다 — isaac_ros 를 부른 터미널에서 다시 돌려라")

import omni.graph.core as og                                       # noqa: E402

K, C = og.Controller.Keys, og.Controller
NS = "isaacsim.ros2.bridge."

# ★ 이미 있으면 지우고 다시 만든다. `og.Controller.edit` 는 같은 자리에 그래프가
#   있으면 **"A graph already exists at this path" 로 실패**한다 (실측 2026-08-26).
#   이 스크립트는 몇 번을 돌려도 같은 결과여야 하므로(재고 고치기), 우리가 만든
#   그래프면 통째로 갈아끼우는 편이 확실하다 — 자리는 설정의 `ros.graph_path` 다.
if st.GetPrimAtPath(G).IsValid():
    say("   기존 그래프 %s 를 지우고 다시 만든다" % G)
    st.RemovePrim(G)
try:
    C.edit(
        {"graph_path": G, "evaluator_name": "execution"},
        {K.CREATE_NODES: [("Tick", "omni.graph.action.OnPlaybackTick"),
                          ("Ctx", NS + "ROS2Context"),
                          # ★ 시간 소스가 없으면 /clock 이 **계속 0** 을 낸다.
                          #   토픽도 보이고 발행자도 1 이라 됐다고 착각하기 쉬운데,
                          #   구독 노드의 시계가 0 에 멎어 타이머가 안 뜬다.
                          ("SimTime", "isaacsim.core.nodes."
                                      "IsaacReadSimulationTime"),
                          ("Clock", NS + "ROS2PublishClock"),
                          ("Tf", NS + "ROS2PublishTransformTree")],
         K.CONNECT: [("Tick.outputs:tick", "Clock.inputs:execIn"),
                     ("Tick.outputs:tick", "Tf.inputs:execIn"),
                     ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
                     ("SimTime.outputs:simulationTime", "Tf.inputs:timeStamp"),
                     ("Ctx.outputs:context", "Clock.inputs:context"),
                     ("Ctx.outputs:context", "Tf.inputs:context")],
         # ★ ROS_DOMAIN_ID 를 씬에 박지 않는다 — 팀이 값을 바꾸면 씬도 고쳐야 한다
         K.SET_VALUES: [("Ctx.inputs:useDomainIDEnvVar", True),
                        ("Clock.inputs:topicName",
                         str(ROS.get("clock_topic", "/clock"))),
                        ("Tf.inputs:topicName", str(tfc.get("topic", "/tf")))]})
except Exception as e:
    die("그래프 생성 실패: %r" % (e,))

tf_node = st.GetPrimAtPath(G + "/Tf")
if not tf_node.IsValid() or not st.GetPrimAtPath(G + "/Clock").IsValid():
    die("브리지 노드가 안 만들어졌다 — 확장 미기동. isaac_ros 후 다시.")

if st.GetPrimAtPath(PARENT).IsValid():
    tf_node.GetRelationship("inputs:parentPrim").SetTargets([Sdf.Path(PARENT)])
else:
    chk("tf parentPrim %s" % PARENT, False, "씬에 없다")
tf_node.GetRelationship("inputs:targetPrims").SetTargets(
    [Sdf.Path(t) for t in targets])

say("   그래프 %s" % G)
say("   /clock  %s" % st.GetPrimAtPath(G + "/Clock")
    .GetAttribute("inputs:topicName").Get())
say("   /tf     %s   · 부모 프레임 %s"
    % (tf_node.GetAttribute("inputs:topicName").Get(), PARENT.rsplit("/", 1)[-1]))
for t in targets:
    say("       -> %s" % t.rsplit("/", 1)[-1])

# ── [3] 저장 ───────────────────────────────────────────────────────────────
say("\n[3] 저장")
st.GetRootLayer().Save()
chk("test_3.usd 저장", True, "%d 프레임" % len(targets))

# ── [4] 검증 — 다시 읽어서 확인한다 ────────────────────────────────────────
say("\n[4] 검증")
st2 = Usd.Stage.Open(STAGE)
n2 = st2.GetPrimAtPath(G + "/Tf")
chk("Tf 노드", n2.IsValid())
if n2.IsValid():
    got = [str(p) for p in n2.GetRelationship("inputs:targetPrims").GetTargets()]
    chk("tf 대상 수", len(got) == len(targets), "%d / %d" % (len(got), len(targets)))
chk("Clock 노드", st2.GetPrimAtPath(G + "/Clock").IsValid())

say("\nRESULT: %s" % ("전부 통과" if not _fail else "실패 %d — %s"
                     % (len(_fail), ", ".join(_fail))))
say("\n확인 (Play 를 누른 뒤, isaac_ros 를 부른 터미널에서):")
say("   ros2 topic echo /clock --once")
say("   ros2 run tf2_tools view_frames")

_kitexit.shutdown(0 if not _fail else 1)
