#!/usr/bin/env python3
"""ROS 2 그래프 전 구간 — 박스가 인식부터 적재까지 실제로 흘러가는지.

    ros2 run packing_db gen_master_db
    ros2 launch packing_bringup bringup.launch.py &
    python3 scripts/verify_ros_graph.py

★ 무엇을 검증하나
  진짜로 도는 것: 인식(vision_node) · DB(db_service_node) · 알고리즘(algo_service_node)
                  · 관제탑(orchestrator). 넷이 ROS 2 로만 대화한다.
  이 스크립트가 대신하는 것: **Isaac 이 하는 일 전부** —
      /clock                     (규약 1. 없으면 use_sim_time 노드의 시계가 0 에 멈춘다)
      /tf_static                 카메라 자세
      <ns>/rgb · camera_info     cam1 · pick_cam
      <ns>/depth                 **16UC1 · mm** (깊이 교차검증의 입력)
      /conveyor/set_velocity     벨트
      /sim/despawn_box 외 3종     씬 서비스
      /zone_*/robot/cmd → /zone_*/robot/event  로봇 팔

★ 왜 대역을 세우나 (CLAUDE.md §11 "mock 을 임의로 만들어 붙이기" 와 다른 점)
  운영 그래프에 가짜를 끼워 넣는 게 아니다. scripts/ 에 있는 **검증 하네스**이고,
  같은 선례가 이미 있다 — verify_b4_cycle.py 의 StandInServer 가 A 의 sim_server
  대역이자 "명령 프로토콜의 참조 구현" 이다. 여기서는 그 ROS 2 판이다.
  씬 USD 가 이 저장소에 없어서(sync_scene.sh 배포) 이 방법 말고는 관제탑
  상태머신을 한 번도 못 돌려 본다.
"""
import math
import os
import sqlite3
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                "..", "src", "packing_common"))
from packing_common.config import load                                  # noqa: E402
from packing_common.ids import box_id as make_box_id                    # noqa: E402
from packing_vision.label import LABEL_MM, make_payload, render_qr      # noqa: E402
from packing_msgs.msg import RobotCmd, RobotEvent, VisionCheck           # noqa: E402
from packing_msgs.srv import (DespawnBox, SetSpawnEnabled, SetVelocity,  # noqa: E402
                              SwapPallet)

CFG = load()
W, H = CFG["recognition"]["resolution"]
HFOV = CFG["recognition"]["hfov_deg"]
BELT = CFG["conveyor"]["belt_top_z_mm"] / 1000.0
TYPES = CFG["boxes"]["types"]
STOPPER = CFG["conveyor"]["stopper_x_mm"] / 1000.0
# ★ 로봇 토픽·픽업 카메라는 **구역별**이다. 구역이 셋인데 토픽이 하나면
#   A 구역 명령을 C 구역 로봇도 받아 세 대가 같은 자리로 간다.
#   어느 구역을 흉내낼지는 boxes_master 가 정한다 — 박스마다 담당 구역이 다르다.
ZONES = [str(z).lower() for z in CFG["zones"]["list"]] \
    if int(CFG["stage"]) >= 2 else [str(CFG["boxes"]["stage1"]["zone"]).lower()]

# 구역별 픽업 카메라. 씬에 아직 1 대뿐이라 좌표는 같게 두고 토픽만 가른다
# (⚠ 트랙 A 가 3 대를 놓으면 그때 y 를 구역별 팔레트 쪽으로 옮긴다).
CAMS = {"cam1": np.array([CFG["recognition"]["cam1_x_mm"] / 1000.0, 0.0, 2.300])}
if len(ZONES) == 1:
    CAMS["pick_cam"] = np.array([9.795, 0.0, 2.300])
    PICK_CAM = {ZONES[0]: "pick_cam"}
else:
    PICK_CAM = {}
    for _z in ZONES:
        CAMS[f"pick_cam_{_z}"] = np.array([9.795, 0.0, 2.300])
        PICK_CAM[_z] = f"pick_cam_{_z}"
R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])

# ★ 실측 오차. 로봇이 계획대로 정확히 놓지 않는다는 것을 재현한다.
#   has_actual=true 로 이 값을 돌려줘야 heightmap 이 실제 더미를 따라간다.
#   계획값을 되보내면 오차가 누적돼 알고리즘은 1300 안이라 믿는데 실제는 넘는다.
PLACE_ERR_MM = (3, -2, 1)

# ★ 카메라 발행 주기. 카메라 4 대가 rgb(2.7 MB) + depth(1.8 MB) 를 함께 내므로
#   10 Hz 면 초당 180 MB 다 — 루프백 DDS 가 못 버텨 **rgb 도 절반이 유실됐다**
#   (실측: 10 Hz 로 쐈는데 수신 5.2 Hz, 깊이 짝은 41 %).
#   인식 노드는 어차피 5 Hz 로만 추론한다. 그보다 빨리 쏠 이유가 없다.
PUMP_PERIOD_S = 0.2


def K_mat():
    fx = (W / 2.0) / math.tan(math.radians(HFOV) / 2.0)
    return np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])


def _project(loc_xy, z, center_xy, yaw_deg, t_cam):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    P = np.hstack([(np.asarray(loc_xy) @ np.array([[c, -s], [s, c]]).T)
                   + np.asarray(center_xy, float), np.full((len(loc_xy), 1), z)])
    uv = (K_mat() @ ((P - t_cam) @ R_DOWN).T).T
    return uv[:, :2] / uv[:, 2:3]


def render(box_type, center_xy, yaw_deg, t_cam, payload=None):
    """빨간 윗면 + (payload 가 있으면) 원근 투영된 QR 라벨.

    ★ 라벨을 안 그리면 cam1 이 QR 을 못 읽어 **한 개도 등록되지 않는다.**
      QR 이 정본이 된 뒤로는 이 하네스도 라벨을 그려야 진짜 경로를 검증한다.
    """
    img = np.full((H, W, 3), (90, 90, 90), np.uint8)
    if box_type is None:
        return img
    w, d = box_type["w_mm"] / 1000.0, box_type["d_mm"] / 1000.0
    z = BELT + box_type["h_mm"] / 1000.0
    loc = np.array([[w / 2, d / 2], [-w / 2, d / 2], [-w / 2, -d / 2], [w / 2, -d / 2]])
    uv = _project(loc, z, center_xy, yaw_deg, t_cam)
    cv2.fillPoly(img, [np.round(uv).astype(np.int32)], (40, 40, 235))
    if payload:
        half = LABEL_MM / 2000.0
        lab_loc = np.array([[-half, half], [half, half], [half, -half], [-half, -half]])
        dst = _project(lab_loc, z, center_xy, yaw_deg, t_cam).astype(np.float32)
        lab = cv2.cvtColor(render_qr(payload, module_px=16), cv2.COLOR_GRAY2BGR)
        n = lab.shape[0]
        src = np.array([[0, 0], [n, 0], [n, n], [0, n]], np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        mask = cv2.warpPerspective(np.full((n, n), 255, np.uint8), M, (W, H))
        img[mask > 0] = cv2.warpPerspective(lab, M, (W, H))[mask > 0]
    return img


_DEPTH_CACHE = {}


def render_depth(box_type, center_xy, yaw_deg, t_cam):
    """같은 장면의 깊이 이미지 — **16UC1 · mm**.

    ★ 왜 32FC1 이 아니라 16UC1 인가 (둘 다 노드가 받는다)
      1. **대역폭이 절반이다.** 1280x720 float32 는 장당 3.7 MB 다. 카메라 4 대가
         10 Hz 로 내면 깊이만 148 MB/s 라 루프백 DDS 가 못 버틴다 — 실측에서
         깊이 짝이 절반만 맞았다. 16UC1 은 1.8 MB 다.
      2. **실기 경로를 덮는다.** Isaac 은 32FC1(m)을 내지만 RealSense 드라이버
         기본값은 16UC1(mm)이다. 여기서 16UC1 을 쓰면 `verify_vision.py`(32FC1)와
         합쳐 **두 인코딩이 모두 검증된다.** 한쪽만 재면 실기로 옮길 때 깊이가
         에러 없이 0 이 된다.
      ⚠ 1 mm 로 양자화된다. 윗면 z 판정에는 충분하다 (허용 45 mm).

    ★ 씬의 `type:"depth"` 와 같은 **광축 방향 거리**다 (distance_to_image_plane).
      시선 거리로 내면 화면 가장자리에서만 어긋나 눈으로는 안 보인다 —
      여기서 규약을 정확히 흉내 내야 노드 쪽 `depth.is_range` 가 진짜로 검증된다.

    ★ **광축 방향 거리**다 (distance_to_image_plane). 시선 거리로 내면 화면
      가장자리에서만 어긋나 눈으로는 안 보인다 — 여기서 규약을 정확히 흉내 내야
      노드 쪽 `depth.is_range` 설정이 진짜로 검증된다.

    ★ 캐시하는 이유: 프레임마다 1280x720 메시그리드를 만들면 하네스가 느려져
      BEST_EFFORT 깊이가 실제로 유실된다. 그러면 **하네스가 느린 것을 노드 결함
      (STALE)으로 오독한다.** verify_vision.py 에서 실제로 그랬다.
    """
    key = (round(t_cam[0], 4), None if box_type is None else box_type["name"],
           round(center_xy[0], 4), round(center_xy[1], 4), round(yaw_deg, 3))
    if key in _DEPTH_CACHE:
        return _DEPTH_CACHE[key]
    K = K_mat()
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    d_cam = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)
    d_w = d_cam @ R_DOWN.T
    s_belt = (BELT - t_cam[2]) / d_w[..., 2]
    if box_type is None:
        out = _to_mm16(s_belt)
    else:
        z_top = BELT + box_type["h_mm"] / 1000.0
        s_top = (z_top - t_cam[2]) / d_w[..., 2]
        P = t_cam.reshape(1, 1, 3) + s_top[..., None] * d_w
        c, sn = math.cos(math.radians(-yaw_deg)), math.sin(math.radians(-yaw_deg))
        dx, dy = P[..., 0] - center_xy[0], P[..., 1] - center_xy[1]
        lx, ly = dx * c - dy * sn, dx * sn + dy * c
        inside = ((np.abs(lx) <= box_type["w_mm"] / 2000.0)
                  & (np.abs(ly) <= box_type["d_mm"] / 2000.0))
        out = _to_mm16(np.where(inside, s_top, s_belt))
    _DEPTH_CACHE[key] = out
    return out


def _to_mm16(depth_m):
    """m(float) → mm(uint16). 65535 mm 를 넘으면 잘리는데, 우리 장면은 최대 2.3 m 다."""
    return np.clip(np.round(np.asarray(depth_m) * 1000.0), 0, 65535).astype(np.uint16).tobytes()


class IsaacStandIn(Node):
    """Isaac 이 ROS 2 로 내는 것 전부를 대신한다."""

    def __init__(self, plan):
        super().__init__("isaac_stand_in")
        cb = ReentrantCallbackGroup()
        self.plan = list(plan)          # [(type, center_xy, yaw, payload), ...]
        self.log = self.get_logger()

        # ── /clock — 규약 1 ──────────────────────────────────────
        # use_sim_time 인 노드들의 시계가 여기서 온다. 없으면 0 에 멈춘다.
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.t0 = time.time()
        self.create_timer(0.02, self._tick_clock, callback_group=cb)

        # ── 카메라 ────────────────────────────────────────────────
        self.pub_img, self.pub_info, self.pub_depth = {}, {}, {}
        for nm in CAMS:
            self.pub_img[nm] = self.create_publisher(Image, f"/{nm}/rgb",
                                                     qos_profile_sensor_data)
            self.pub_info[nm] = self.create_publisher(CameraInfo, f"/{nm}/camera_info",
                                                      qos_profile_sensor_data)
            self.pub_depth[nm] = self.create_publisher(Image, f"/{nm}/depth",
                                                       qos_profile_sensor_data)
        # ★ 깊이 교차검증 진단 채널을 모은다. 로봇 동작 경로가 아니므로 여기서
        #   무엇을 하든 사이클에 영향이 없다 — 끝에서 지표로만 쓴다.
        self.vchecks = []
        for t in [f"/zone_{z}/vision_check" for z in ZONES] + ["/line/vision_check"]:
            self.create_subscription(VisionCheck, t,
                                     lambda m: self.vchecks.append(m), 10,
                                     callback_group=cb)
        self.tfb = StaticTransformBroadcaster(self)
        tfs = []
        for nm, t in CAMS.items():
            m = TransformStamped()
            m.header.frame_id, m.child_frame_id = "World", nm
            m.transform.translation.x = float(t[0])
            m.transform.translation.y = float(t[1])
            m.transform.translation.z = float(t[2])
            m.transform.rotation.x, m.transform.rotation.w = 1.0, 0.0
            tfs.append(m)
        self.tfb.sendTransform(tfs)

        # ── 씬 서비스 4종 ─────────────────────────────────────────
        self.belt_mps = 0.0
        self.despawned = []
        self.create_service(SetVelocity, "/conveyor/set_velocity",
                            self._set_velocity, callback_group=cb)
        self.create_service(DespawnBox, "/sim/despawn_box",
                            self._despawn, callback_group=cb)
        self.create_service(SetSpawnEnabled, "/sim/set_spawn_enabled",
                            lambda q, r: (setattr(r, "ok", True), r)[1], callback_group=cb)
        self.create_service(SwapPallet, "/sim/swap_pallet",
                            lambda q, r: (setattr(r, "ok", True), r)[1], callback_group=cb)

        # ── 로봇 — 구역마다 한 대 ─────────────────────────────────
        self.pub_ev = {z: self.create_publisher(
            RobotEvent, f"/zone_{z}/robot/event", 10) for z in ZONES}
        for z in ZONES:
            self.create_subscription(
                RobotCmd, f"/zone_{z}/robot/cmd",
                lambda m, _z=z: self._on_cmd(m, _z), 10, callback_group=cb)

        # ── 라인 상태 ─────────────────────────────────────────────
        # gap → cam1 → pick → (집힘) → gap → ...
        #
        # ★ gap(빈 벨트)이 왜 필요한가
        #   line_detector 는 **상승 에지**로 박스를 센다 — 빈 화면을 한 번 봐야
        #   다음 박스를 새 박스로 친다. 안 그러면 같은 박스를 매 프레임 등록한다.
        #   실제 라인은 12 초 간격 · 6 m 간격이라 빈 구간이 길게 보장되지만,
        #   이 하네스는 이송을 압축해 박스가 순간이동한다. gap 을 안 넣으면
        #   cam1 이 한 프레임도 비지 않아 두 번째 박스부터 등록이 안 된다.
        self.stage = "gap"
        self.cur = self.plan.pop(0) if self.plan else None
        self.placed = []
        # ★ 카메라 펌프만 **배타 그룹**에 둔다. ReentrantCallbackGroup +
        #   MultiThreadedExecutor 조합에서는 같은 타이머가 겹쳐 돌 수 있다 —
        #   실측: 0.1 초 타이머인데 `/cam1/rgb` 가 **54.6 Hz** 로 나갔다.
        #   그 대역폭이 깊이(장당 3.7 MB)를 밀어내 짝이 절반만 맞았다.
        #   씬 대역이 실기보다 빠르게 도는 것은 검증에 아무 도움이 안 된다.
        self.create_timer(0.1, self._pump_cameras,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.5, self._advance, callback_group=cb)
        self._t_cam1 = time.time()

    # ── /clock ────────────────────────────────────────────────────
    def _tick_clock(self):
        el = time.time() - self.t0
        msg = Clock()
        msg.clock.sec = int(el)
        msg.clock.nanosec = int((el % 1.0) * 1e9)
        self.pub_clock.publish(msg)

    # ── 카메라 ────────────────────────────────────────────────────
    def _send(self, nm, bt, center, yaw, payload=None):
        img = render(bt, center, yaw, CAMS[nm], payload)
        m = Image()
        m.header.frame_id, m.height, m.width = nm, H, W
        m.encoding, m.step, m.data = "bgr8", W * 3, img.tobytes()
        # ★ stamp 를 넣는다. 인식 노드가 rgb 와 depth 를 **같은 프레임인지**
        #   이 값으로 가르기 때문이다 (허용 시차 0.02 초 = 한 프레임 안).
        #   이 노드는 use_sim_time 이 아니라 벽시계를 쓰지만, rgb 와 depth 가
        #   둘 다 같은 시계에서 나오므로 **차이**를 재는 데는 문제가 없다.
        m.header.stamp = self.get_clock().now().to_msg()
        info = CameraInfo()
        info.header.frame_id, info.height, info.width = nm, H, W
        info.k = K_mat().reshape(-1).tolist()
        info.distortion_model = "plumb_bob"
        self.pub_info[nm].publish(info)
        self.pub_img[nm].publish(m)

        d = Image()
        d.header.frame_id, d.height, d.width = nm, H, W
        d.encoding, d.step = "16UC1", W * 2      # mm. 32FC1 대비 절반 크기다
        d.header.stamp = m.header.stamp        # RGB 와 같은 시각 = 같은 프레임
        d.data = render_depth(bt, center, yaw, CAMS[nm])
        self.pub_depth[nm].publish(d)

    def _pump_cameras(self):
        # ★ 벽시계로 스스로 막는다. 타이머 주기(0.1 초)만 믿으면 안 된다 —
        #   실측: MultiThreadedExecutor 밑에서 이 타이머가 **초당 55 회** 돌았고
        #   (`/cam1/rgb` 1093 건 / 20 초), 그 대역이 깊이(장당 1.8 MB)를 밀어내
        #   같은 프레임 짝이 60 % 밖에 안 맞았다. 박스가 스토퍼에 서 있는 동안
        #   추론 기회가 한 번뿐이라, 짝 하나를 놓치면 그 박스는 깊이 확인 없이 지나간다.
        #   씬 카메라는 30 Hz 이고 인식은 5 Hz 로만 추론한다 — 10 Hz 면 충분하다.
        now = time.time()
        if now - getattr(self, "_t_pump", 0.0) < PUMP_PERIOD_S:
            return
        self._t_pump = now
        if self.cur:
            bt, center, yaw, pay, zone = self.cur
        else:
            bt, center, yaw, pay, zone = None, (0, 0), 0.0, None, ZONES[0]
        x1 = CFG["recognition"]["cam1_x_mm"] / 1000.0
        self._send("cam1", bt if self.stage == "cam1" else None,
                   (x1, center[1]), yaw, pay)
        # ★ 픽업 카메라는 **그 박스의 구역** 것만 박스를 보여 준다.
        #   전부에 보여 주면 세 구역이 같은 박스를 동시에 집으려 든다.
        for z in ZONES:
            show = bt if (self.stage == "pick" and z == zone) else None
            self._send(PICK_CAM[z], show, center, yaw, pay)

    def _advance(self):
        el = time.time() - self._t_cam1
        if self.stage == "gap" and self.cur and el > 1.2:
            # 빈 벨트를 충분히 보여 줬다 — 이제 투입부에 박스를 올린다
            self.stage = "cam1"
            self._t_cam1 = time.time()
        elif self.stage == "cam1" and self.cur and el > 2.0:
            # cam1 에서 잠깐 보여 준 뒤 스토퍼로 옮긴다 (벨트 이송을 압축한 것)
            self.stage = "pick"
            self.log.info(f"[씬] 박스가 스토퍼에 도착 — {self.cur[0]['name']} "
                          f"→ 구역 {self.cur[4].upper()}")

    # ── 씬 서비스 ─────────────────────────────────────────────────
    def _set_velocity(self, req, res):
        self.belt_mps = float(req.velocity_mps)
        res.ok, res.actual_mps = True, self.belt_mps
        return res

    def _despawn(self, req, res):
        # ★ 실제로 치운다. 안 치우면 스토퍼에 그대로 남아 detector 가 계속
        #   present=True 를 내고, 관제탑이 같은 박스를 무한히 다시 집으려 한다.
        #   씬이 despawn 을 무시하면 라인이 이렇게 막힌다 (CLAUDE.md §7).
        self.despawned.append(req.box_id)
        self.log.info(f"[씬] despawn {req.box_id} — 스토퍼를 비운다")
        self.cur = self.plan.pop(0) if self.plan else None
        self.stage = "gap" if self.cur else "gone"
        self._t_cam1 = time.time()
        res.ok = True
        return res

    # ── 로봇 ──────────────────────────────────────────────────────
    def _on_cmd(self, cmd, zone):
        """PICK/PLACE 를 받아 성공을 돌려준다. ★ seq 를 반드시 되돌려준다."""
        ev = RobotEvent()
        ev.seq = cmd.seq                 # ← 이게 없으면 관제탑이 결과를 못 짝짓는다
        ev.action = cmd.action
        ev.box_id = cmd.box_id
        ev.ok = True
        ev.reason = ""
        ev.duration_s = 1.0

        if cmd.action == RobotCmd.PICK:
            self.log.info(f"[로봇 {zone}] PICK {cmd.box_id} "
                          f"({cmd.x_mm},{cmd.y_mm},{cmd.z_mm}) yaw {cmd.yaw_deg:.1f}")
            self.stage = "gone"          # 집혔으니 스토퍼가 빈다
        elif cmd.action == RobotCmd.PLACE:
            # ★ 실측을 돌려준다. 계획값을 그대로 되보내면 오차가 안 쌓여
            #   heightmap 이 실제보다 낮다고 믿는다 (규약 3).
            ev.has_actual = True
            ev.actual_x_mm = cmd.x_mm + PLACE_ERR_MM[0]
            ev.actual_y_mm = cmd.y_mm + PLACE_ERR_MM[1]
            ev.actual_z_mm = cmd.z_mm + PLACE_ERR_MM[2]
            ev.actual_yaw_deg = cmd.yaw_deg
            self.log.info(f"[로봇 {zone}] PLACE {cmd.box_id} → 팔레트 "
                          f"({cmd.x_mm},{cmd.y_mm},{cmd.z_mm})")
            self.placed.append(cmd.box_id)
            # 다음 박스를 투입한다
            self.cur = self.plan.pop(0) if self.plan else None
            self.stage = "gap" if self.cur else "gone"      # ★ 빈 벨트를 먼저 보여 준다
            self._t_cam1 = time.time()
        self.pub_ev[zone].publish(ev)


def main():
    n_box = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rng = np.random.default_rng(CFG["boxes"]["random_seed"])

    # ★ 박스가 어느 구역으로 가는지는 **boxes_master 가 정본**이다.
    #   씬이 따로 정하면 DB 가 아는 구역과 어긋나 로봇이 남의 팔레트에 쌓는다.
    _db = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "palletris.db")
    _c = sqlite3.connect(f"file:{_db}?mode=ro", uri=True)
    zone_of = dict(_c.execute("SELECT box_id, zone FROM boxes_master").fetchall())
    _c.close()

    plan = []
    for i in range(n_box):
        bt = TYPES[int(rng.integers(0, len(TYPES)))]
        y = float(rng.uniform(-0.10, 0.10))
        # ★ box_id 는 boxes_master 와 같은 규칙이어야 한다 — QR 이 정본이므로
        #   여기서 다른 규칙을 쓰면 DB 와 씬이 서로 다른 박스를 말하게 된다.
        box_id = make_box_id(i)
        z = str(zone_of.get(box_id, ZONES[0])).lower()
        if z not in ZONES:
            sys.exit(f"{box_id} 의 구역 {z!r} 이 담당 구역 {ZONES} 에 없다")
        plan.append((bt, (STOPPER - 0.22, y), float(rng.uniform(-15, 15)),
                     make_payload(box_id), z))

    rclpy.init()
    scene = IsaacStandIn(plan)
    ex = MultiThreadedExecutor()
    ex.add_node(scene)

    print(f"\n박스 {n_box} 개를 그래프에 흘려보낸다 "
          f"({', '.join(b[0]['name'] + '→' + b[4].upper() for b in plan)})\n")
    deadline = time.time() + 20 + 15 * n_box
    while time.time() < deadline and len(scene.placed) < n_box:
        ex.spin_once(timeout_sec=0.1)
    # ★ 로봇이 PLACE 를 답한 시점과 DB 에 PLACED 가 남는 시점은 다르다.
    #   그 사이에 UPDATING(→ /algo/update_heightmap → /db/mark_placed)이 있고
    #   전부 call_async 라 비동기다. 바로 DB 를 읽으면 마지막 박스가 아직 SCANNED 다.
    tail = time.time() + 5.0
    while time.time() < tail:
        ex.spin_once(timeout_sec=0.1)
    placed = list(scene.placed)
    vchecks = list(scene.vchecks)
    scene.destroy_node()
    rclpy.shutdown()

    db = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "palletris.db")
    c = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    rows = c.execute("SELECT box_id, status, place_x_mm, place_y_mm, place_z_mm "
                     "FROM boxes_sorted ORDER BY db_seq").fetchall()

    ok = True

    def chk(label, cond, detail=""):
        nonlocal ok
        ok = ok and cond
        print(f"  {'✓' if cond else '✗ 실패'} {label}{(' — ' + detail) if detail else ''}")

    # ★ NO_FEASIBLE_POSE 로 버려진 박스는 **실패가 아니다** (CLAUDE.md §7).
    #   "이 박스만 자리가 없다" 는 정상 이탈 경로이고, 그때 라인을 세우지 않고
    #   계속 도는 것이 요구사항이다. 예전 판은 "전부 PLACED" 를 요구해서
    #   설계대로 동작한 실행을 실패로 셌다 — 그러면 진짜 버그와 구분이 안 된다.
    #   대신 **놓인 것 + 버린 것 = 전부** 인지를 본다.
    done = [r for r in rows if r[1] in ("PLACED", "REJECTED")]
    kept = [r for r in rows if r[1] == "PLACED"]
    dropped = [r for r in rows if r[1] == "REJECTED"]

    print()
    chk(f"로봇이 놓은 수 = PLACED 수", len(placed) == len(kept),
        f"로봇 {len(placed)} · DB {len(kept)}")
    chk("DB 에 행이 다 생겼다", len(rows) == n_box, f"{len(rows)} 행")
    chk("모든 박스가 결말을 봤다 (PLACED 또는 REJECTED)", len(done) == n_box,
        ", ".join(f"{r[0]}={r[1]}" for r in rows if r[1] not in ("PLACED", "REJECTED"))
        or f"적재 {len(kept)} · 폐기 {len(dropped)}")
    chk("실측 pose 가 기록됐다", all(r[2] is not None for r in kept))

    # ── 깊이 교차검증 지표 ────────────────────────────────────────────────
    # ★ 여기는 **로봇 동작 경로가 아니다.** 아래가 다 실패해도 위 적재 결과는
    #   그대로여야 한다 — 그게 "확인 채널" 이라는 말의 뜻이고, 실제로 그런지도
    #   같이 보이는 자리다. 깊이를 끄고 돌리려면 depth:=false 로 launch 한다.
    picks = [c for c in vchecks if c.source == "pick_cam"]
    used = [c for c in picks if c.depth_ok]
    print()
    if not vchecks:
        print("  · VisionCheck 이 한 건도 안 왔다 — 깊이가 꺼져 있거나 "
              "인식 노드가 이 판이 아니다 (적재 결과와는 무관하다)")
    else:
        chk("깊이 교차검증이 돌았다", bool(picks), f"pick_cam {len(picks)} 건")
        chk("깊이로 확인된 박스가 있다", bool(used), f"{len(used)} 건")
        # ★ **커버리지는 통과 기준이 아니라 보고 항목이다.**
        #   왜 기준으로 못 쓰나: 박스는 관제탑이 IDLE 인 동안에만 추론되므로
        #   **박스당 기회가 사실상 한 번**이고, 그 한 번에 같은 프레임의 깊이가
        #   와 있어야 한다. 그런데 이 하네스는 카메라 4 대가 rgb(2.7 MB) +
        #   depth(1.8 MB) 를 함께 내보내 UDP 단편 손실이 난다 — **rgb 도 같이 잃는다.**
        #   즉 커버리지는 인식 코드의 성질이 아니라 **호스트 DDS 대역의 성질**이다.
        #   실측: 기본 버퍼 4/8 → FASTRTPS 송수신 버퍼 16 MB 로 6/8.
        #   진짜 씬에서는 rgb 와 depth 가 **한 render product 에서 같은 틱에** 나가
        #   stamp 가 정확히 같으므로 이 손실 자체가 없다.
        #   그래서 여기서 재는 것은 "확인된 것이 맞았는가" 이고, 커버리지는 숫자로만 남긴다.
        ids = {p[3] for p in plan}
        covered = {c.box_id for c in used if c.box_id}
        missing = sorted(ids - covered)
        rate = (len(used) / len(picks)) if picks else 0.0
        print(f"  · 깊이 확인 커버리지 {len(covered)}/{len(ids)} 박스 "
              f"(프레임 {rate:.0%}) — 통과 기준 아님")
        if missing:
            print("    ↳ 깊이 확인 없이 지나간 박스: " + ", ".join(missing)
                  + "\n      DDS 송수신 버퍼를 키우면 올라간다 "
                    "(FASTRTPS_DEFAULT_PROFILES_FILE · net.core.rmem_max)")
        if used:
            agree = sum(1 for c in used if c.agree)
            chk("두 경로 일치율 100 %", agree == len(used),
                f"{agree}/{len(used)} · 불일치 "
                + (", ".join(sorted({c.verdict for c in used if not c.agree})) or "없음"))
            # ★ 이게 RGB 와 **독립인** 유일한 숫자다. 여기가 맞아야 교차검증에 뜻이 있다.
            # payload 는 box_id 그 자체다 (label.make_payload 는 항등 함수)
            id_h = {p[3]: p[0]["h_mm"] for p in plan}
            bad = [(c.box_id, c.depth_h_mm) for c in used
                   if c.box_id in id_h and abs(c.depth_h_mm - id_h[c.box_id]) > 15.0]
            chk("깊이가 잰 높이가 사양과 맞다", not bad,
                ", ".join(f"{b}={h:.0f}mm" for b, h in bad) or f"{len(used)} 건 전부")
            worst = max(used, key=lambda c: abs(c.dz_mm))
            print(f"  · 최대 dz {worst.dz_mm:+.1f} mm · 최대 dxy "
                  f"{max(c.dxy_mm for c in used):.1f} mm · 윗면 유효비율 최소 "
                  f"{min(c.depth_valid_ratio for c in used):.2f}")
    if dropped:
        print(f"  · 폐기 {len(dropped)} 개 — "
              + ", ".join(r[0] for r in dropped)
              + " (NO_FEASIBLE_POSE 는 정상 경로다)")
    print("\n  적재 결과")
    for r in rows:
        print(f"    {r[0]}  {r[1]:8s}  ({r[2]}, {r[3]}, {r[4]}) mm")
    print("\n" + ("전부 통과" if ok else "★ 실패 있음"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
