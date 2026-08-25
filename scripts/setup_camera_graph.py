#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera.usd 에 ROS2 카메라 그래프를 얹어 토픽 3개를 발행한다. (C안 — 확인용)

발행되는 토픽:
    /<ns>/rgb           sensor_msgs/Image        rgb8
    /<ns>/depth         sensor_msgs/Image        32FC1, 단위 m (distance to image plane)
    /<ns>/camera_info   sensor_msgs/CameraInfo   fx, fy, cx, cy

★ 이 파일은 Isaac 프로세스에서 돈다 — rclpy 를 절대 import 하지 않는다 (철칙 1).
  ROS 쪽 구독/발행은 packing_vision/vision_node.py 가 따로 맡는다.

★ 씬 파일(camera.usd)을 고치지 않는다. 열어서 그래프만 메모리 위에 얹고 돌린다.
  (철칙 2 — 씬 USD 편집은 트랙 A 소관)

실행:
    ~/isaacsim/python.sh scripts/setup_camera_graph.py           # 창 없이(headless)
    ~/isaacsim/python.sh scripts/setup_camera_graph.py --gui     # 창 띄우기
"""

import argparse
import os
import sys

# SimulationApp 을 만들기 전에 인자를 먼저 읽는다 — 만든 뒤에는 argv 가 kit 쪽으로 넘어간다.
_p = argparse.ArgumentParser()
_p.add_argument("--usd", default=os.path.expanduser("~/palletris_ws/camera.usd"))
_p.add_argument("--camera", default="/World/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
                help="렌더에 쓸 카메라 prim 경로. ★ Camera_Pseudo_Depth 는 depth 가 전부 무효라 쓰면 안 된다")
_p.add_argument("--ns", default="cam_scan", help="토픽 네임스페이스 → /<ns>/rgb 등")
_p.add_argument("--width", type=int, default=1280)
_p.add_argument("--height", type=int, default=720)
_p.add_argument("--cam-pos", default=None, metavar="X,Y,Z",
                help="카메라 리그를 이 월드 좌표(m)로 옮긴다. 안 주면 씬에 저장된 자리 그대로")
_p.add_argument("--aim-at", default=None, metavar="X,Y,Z",
                help="카메라가 이 월드 좌표(m)를 보도록 리그를 회전시킨다")
_p.add_argument("--dome-light", type=float, default=None, metavar="INTENSITY",
                help="DomeLight 를 추가한다 (씬이 어두워 색이 안 잡힐 때)")
_p.add_argument("--no-freeze-camera", action="store_true",
                help="카메라 고정을 하지 않는다. 기본은 고정 — 붙이면 Play 시 카메라가 떨어진다")
_p.add_argument("--make-camera", default=None, metavar="X,Y,Z",
                help="이 좌표(m)에 바로 아래(-Z)를 내려다보는 단순 Camera prim 을 새로 만들어 쓴다. "
                     "rsd455 리그를 건드리지 않아도 되므로 확인용으로 가장 확실하다")
_p.add_argument("--make-camera-focal", type=float, default=18.0,
                help="--make-camera 로 만든 카메라의 초점거리(mm). 작을수록 넓게 본다. 18mm = 화각 약 60도")
_p.add_argument("--render-mode", choices=("replicator", "viewport"), default="replicator",
                help="replicator = headless 에서도 확실히 렌더된다(기본). "
                     "viewport = GUI 마법사가 만드는 것과 같은 구성")
_p.add_argument("--gui", action="store_true", help="붙이면 창을 띄운다")
ARGS, _ = _p.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": not ARGS.gui})

import omni                                  # noqa: E402
import omni.graph.core as og                 # noqa: E402
import usdrt.Sdf                             # noqa: E402
from isaacsim.core.api import SimulationContext          # noqa: E402
from isaacsim.core.utils import extensions, stage        # noqa: E402
from omni.kit.viewport.utility import get_active_viewport  # noqa: E402

# ROS2 브릿지 확장을 켜야 ROS2CameraHelper 노드가 존재한다
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ── 1. 씬 열기 ────────────────────────────────────────────────────────────────
if not os.path.exists(ARGS.usd):
    print(f"[setup_camera_graph] ✗ USD 가 없다: {ARGS.usd}")
    simulation_app.close()
    sys.exit(1)

print(f"[setup_camera_graph] 씬 여는 중: {ARGS.usd}")
stage.open_stage(ARGS.usd)
for _ in range(10):          # rsd455 는 원격 URL payload 라 로드에 몇 프레임 걸린다
    simulation_app.update()

usd_stage = omni.usd.get_context().get_stage()

# ── 2. 카메라 prim 확인 ──────────────────────────────────────────────────────
# 여기서 미리 확인하지 않으면, 경로가 틀려도 그래프는 조용히 만들어지고
# 토픽에 새까만 화면만 나온다. 원인 찾기가 매우 어려워지므로 먼저 막는다.
cameras = [p.GetPath().pathString for p in usd_stage.Traverse()
           if p.GetTypeName() == "Camera"]
print(f"[setup_camera_graph] 씬에서 찾은 Camera prim {len(cameras)}개:")
for c in cameras:
    print(f"    {c}")

if ARGS.camera not in cameras:
    print(f"[setup_camera_graph] ✗ --camera 경로를 씬에서 못 찾았다: {ARGS.camera}")
    print("    위 목록 중 하나를 --camera 로 넘겨라.")
    simulation_app.close()
    sys.exit(1)

print(f"[setup_camera_graph] 사용할 카메라: {ARGS.camera}")

# ── 2b. 카메라 위치/방향 잡기 ────────────────────────────────────────────────
# camera.usd 에 저장된 rsd455 는 큐브 반대쪽(-X, 아래)을 본다. 그대로 두면
# RGB 는 새까맣고 depth 는 전부 무효(inf)라 아무것도 검출되지 않는다.
# ★ 파일은 건드리지 않는다. 메모리 위 stage 의 transform 만 바꾼다.
from pxr import Gf, Usd, UsdGeom      # noqa: E402

# --make-camera 를 주면 여기서 카메라를 새로 정의하고 그걸 쓴다.
# rsd455 는 카메라가 하우징 메시 안에 들어 있어서, 리그를 돌리면 케이스가 렌즈를 가린다
# (depth 가 0.1m 로 나오는 증상). 단순 Camera prim 은 그 문제가 없다.
if ARGS.make_camera:
    from pxr import Gf, UsdGeom                # noqa: E402
    xyz = [float(v) for v in ARGS.make_camera.split(",")]
    ARGS.camera = "/World/VisionCam"
    _cam = UsdGeom.Camera.Define(usd_stage, ARGS.camera)
    # 회전을 주지 않으면 카메라는 로컬 -Z 를 본다 = Z-up 씬에서 '바로 아래'
    UsdGeom.XformCommonAPI(_cam).SetTranslate(Gf.Vec3d(*xyz))
    _cam.CreateFocalLengthAttr(float(ARGS.make_camera_focal))
    _cam.CreateHorizontalApertureAttr(20.955)      # 35mm 필름 기준 표준값
    _cam.CreateVerticalApertureAttr(20.955 * ARGS.height / ARGS.width)
    _cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    simulation_app.update()
    _hfov = 2 * __import__("math").degrees(
        __import__("math").atan(20.955 / (2 * ARGS.make_camera_focal)))
    print(f"[setup_camera_graph] 카메라 새로 생성: {ARGS.camera} @ {tuple(xyz)} "
          f"· 초점 {ARGS.make_camera_focal}mm · 수평화각 {_hfov:.1f}° · 바로 아래를 본다")
    ARGS.cam_pos = None            # 새 카메라는 이미 원하는 자리에 있다
    ARGS.aim_at = None             # 그리고 이미 아래를 본다

cam_prim = usd_stage.GetPrimAtPath(ARGS.camera)
# 리그 = 카메라를 품고 있는 최상위 Xform (예: /World/rsd455)
rig_prim = cam_prim
while rig_prim.GetParent().GetPath().pathString not in ("/World", "/"):
    rig_prim = rig_prim.GetParent()
xf_cache = UsdGeom.XformCache(Usd.TimeCode.Default())


def _parse_xyz(text):
    return [float(v) for v in text.split(",")]


def _cam_pose():
    """카메라의 월드 위치와 시선 방향. USD 카메라는 로컬 -Z 를 본다."""
    xf_cache.Clear()
    m = xf_cache.GetLocalToWorldTransform(cam_prim)
    r = m.ExtractRotationMatrix()
    fwd = Gf.Vec3d(-r[2][0], -r[2][1], -r[2][2]).GetNormalized()
    return m.ExtractTranslation(), fwd


def _set_rig_orient(quat):
    a = rig_prim.GetAttribute("xformOp:orient")
    a.Set(Gf.Quatf(quat) if "quatf" in str(a.GetTypeName()) else Gf.Quatd(quat))


pos0, fwd0 = _cam_pose()
print(f"[setup_camera_graph] 원래 카메라 위치 {tuple(round(v, 3) for v in pos0)} "
      f"시선 {tuple(round(v, 3) for v in fwd0)}")

if ARGS.cam_pos:
    xyz = _parse_xyz(ARGS.cam_pos)
    rig_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*xyz))
    print(f"[setup_camera_graph] 리그 {rig_prim.GetPath()} 를 {tuple(xyz)} 로 이동")

if ARGS.aim_at:
    tgt = Gf.Vec3d(*_parse_xyz(ARGS.aim_at))
    # 회전시키면 카메라 위치도 같이 움직이므로(리그 원점에서 떨어져 있다) 몇 번 반복해 수렴시킨다
    for _ in range(8):
        pos, fwd = _cam_pose()
        want = (tgt - pos).GetNormalized()
        if Gf.Dot(fwd, want) > 0.9999999:
            break
        xf_cache.Clear()
        rig_rot = xf_cache.GetLocalToWorldTransform(rig_prim).ExtractRotation()
        delta = Gf.Rotation(fwd, want)          # 월드에서 fwd 를 want 로 돌리는 회전
        m_new = Gf.Matrix4d().SetRotate(rig_rot) * Gf.Matrix4d().SetRotate(delta)
        _set_rig_orient(m_new.ExtractRotationQuat())
    pos, fwd = _cam_pose()
    print(f"[setup_camera_graph] 조준 완료 → 위치 {tuple(round(v, 3) for v in pos)} "
          f"시선 {tuple(round(v, 3) for v in fwd)} · 목표까지 {(tgt - pos).GetLength():.3f}m")

if ARGS.dome_light is not None:
    from pxr import UsdLux                      # noqa: E402
    light = UsdLux.DomeLight.Define(usd_stage, "/World/VisionDomeLight")
    light.CreateIntensityAttr(float(ARGS.dome_light))
    print(f"[setup_camera_graph] DomeLight 추가 (intensity={ARGS.dome_light})")

simulation_app.update()

# ── 2c. 카메라 고정 — Play 눌러도 안 떨어지게 ────────────────────────────────
# rsd455 는 /World/rsd455/RSD455 에 RigidBodyAPI 가 붙어 있는데 콜라이더가 하나도 없다.
# 그래서 Play 를 누르면 중력을 받고, 받아줄 콜리전이 없으니 바닥을 통과해 무한히 떨어진다.
#
# kinematicEnabled = True 로 두면 리지드바디는 남지만 물리가 이 물체를 밀지 못한다.
# (rigidBodyEnabled = False 로 꺼도 되지만, 나중에 스크립트로 카메라를 움직일 여지를
#  남기려면 kinematic 쪽이 낫다 — 물리에는 안 밀리고 transform 으로는 움직인다)
if not ARGS.no_freeze_camera:
    from pxr import UsdPhysics                  # noqa: E402

    def _freeze(prim):
        """이 prim 의 리지드바디를 kinematic 으로 만든다."""
        rb = UsdPhysics.RigidBodyAPI(prim)
        attr = rb.GetKinematicEnabledAttr()
        if not attr:
            attr = rb.CreateKinematicEnabledAttr()
        attr.Set(True)

    frozen = []
    for _p in usd_stage.Traverse():
        if "PhysicsRigidBodyAPI" not in [str(a) for a in _p.GetAppliedSchemas()]:
            continue
        # 이 리지드바디 아래에 Camera 가 들어 있으면 = 카메라 리그다
        has_cam = any(c.GetTypeName() == "Camera" for c in Usd.PrimRange(_p))
        if has_cam:
            _freeze(_p)
            frozen.append(_p.GetPath().pathString)

    if frozen:
        print("[setup_camera_graph] 카메라 고정(kinematic) 적용:")
        for f in frozen:
            print(f"    {f}")
    else:
        print("[setup_camera_graph] 고정할 카메라 리지드바디 없음 (떨어질 카메라가 없다)")
    simulation_app.update()

# ── 3. ROS2 카메라 그래프 만들기 ─────────────────
# 헬퍼 노드 3개(rgb/depth/camera_info)에 '렌더 프로덕트 경로'를 물려주면 토픽이 나온다.
# 그 경로를 얻는 방법이 두 가지다:
#
#   replicator (기본) : rep.create.render_product() 로 카메라에서 직접 만든다.
#                       headless 에서도 확실히 렌더된다.
#   viewport          : createViewport -> getRenderProduct -> setCamera 체인.
#                       GUI 마법사가 만드는 구성과 같지만, headless 에서는 뷰포트
#                       확장이 없어 이미지가 전부 0(새까맣게) 으로 나온다.
keys = og.Controller.Keys

if ARGS.render_mode == "replicator":
    import omni.replicator.core as rep      # noqa: E402

    render_product = rep.create.render_product(ARGS.camera,
                                               resolution=(ARGS.width, ARGS.height))
    rp_path = render_product.path
    print(f"[setup_camera_graph] render product = {rp_path}")
    for _ in range(5):
        simulation_app.update()

    create_nodes = [
        ("OnTick", "omni.graph.action.OnTick"),
        ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]
    connect = [
        ("OnTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
        ("OnTick.outputs:tick", "cameraHelperInfo.inputs:execIn"),
        ("OnTick.outputs:tick", "cameraHelperDepth.inputs:execIn"),
    ]
    extra_values = [
        ("cameraHelperRgb.inputs:renderProductPath", rp_path),
        ("cameraHelperInfo.inputs:renderProductPath", rp_path),
        ("cameraHelperDepth.inputs:renderProductPath", rp_path),
    ]
else:
    create_nodes = [
        ("OnTick", "omni.graph.action.OnTick"),
        ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
        ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
        ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
        ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]
    connect = [
        ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
        ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
        ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
        ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
        ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
        ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
        ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
        ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
        ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
        ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
        ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
    ]
    extra_values = [
        ("createViewport.inputs:viewportId", 0),
        ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(ARGS.camera)]),
    ]

(ros_camera_graph, _, _, _) = og.Controller.edit(
    {
        "graph_path": "/ROS_Camera",
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: create_nodes,
        keys.CONNECT: connect,
        keys.SET_VALUES: [
            ("cameraHelperRgb.inputs:frameId", ARGS.ns),
            ("cameraHelperRgb.inputs:nodeNamespace", ARGS.ns),
            ("cameraHelperRgb.inputs:topicName", "rgb"),
            ("cameraHelperRgb.inputs:type", "rgb"),
            # camera_info 가 없으면 vision_node 가 픽셀->mm 환산을 못 한다
            ("cameraHelperInfo.inputs:frameId", ARGS.ns),
            ("cameraHelperInfo.inputs:nodeNamespace", ARGS.ns),
            ("cameraHelperInfo.inputs:topicName", "camera_info"),
            # depth 는 32FC1, 값의 단위는 m
            ("cameraHelperDepth.inputs:frameId", ARGS.ns),
            ("cameraHelperDepth.inputs:nodeNamespace", ARGS.ns),
            ("cameraHelperDepth.inputs:topicName", "depth"),
            ("cameraHelperDepth.inputs:type", "depth"),
        ] + extra_values,
    },
)

# 한 번 돌려야 SDGPipeline 안에 실제 ROS 퍼블리셔가 생성된다
og.Controller.evaluate_sync(ros_camera_graph)
simulation_app.update()

if ARGS.render_mode == "viewport":
    viewport_api = get_active_viewport()
    if viewport_api is not None:
        viewport_api.set_texture_resolution((ARGS.width, ARGS.height))
        simulation_app.update()

# ── 4. 재생 ──────────────────────────────────────────────────────────────────
simulation_context = SimulationContext(stage_units_in_meters=1.0)
simulation_context.initialize_physics()
simulation_context.play()

print("[setup_camera_graph] READY — 발행 중인 토픽:")
print(f"    /{ARGS.ns}/rgb")
print(f"    /{ARGS.ns}/depth")
print(f"    /{ARGS.ns}/camera_info")
print(f"    해상도 {ARGS.width}x{ARGS.height} · ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(미설정)')}")
sys.stdout.flush()

try:
    while simulation_app.is_running():
        simulation_context.step(render=True)
except KeyboardInterrupt:
    pass
finally:
    simulation_context.stop()
    simulation_app.close()
