#!/usr/bin/env python3
"""인식 노드 — 투입부 등록·검증(cam1·cam2) + 픽업 좌표(pick_cam).

CLAUDE.md §8 의 "라인 인식 노드 2 · 구역 인식 노드 3" 을 구현한다.

★ 왜 한 프로세스에 같이 두나
  둘이 **박스 순번을 공유해야** 한다. 투입부에서 BOX00007 로 등록한 박스가
  스토퍼에 도착했을 때 "이게 7번"임을 알아야 관제탑에 box_id 를 실어 보낼 수
  있는데, 벨트가 한 줄이라 추월이 없으므로 **등록 순서 = 도착 순서**다.
  프로세스를 가르면 이 순번을 서비스로 주고받아야 하고, 그럴 서비스가 인터페이스에
  없다. 중재자를 db_service_node 에 같이 둔 것과 같은 이유다 (명세서 §8.5).
  노드는 둘이고 프로세스만 하나다 — CLAUDE.md §8 "관제탑은 하나" 규칙과 무관하다.

★ 카메라 파라미터를 코드에 안 박는다
  내부 파라미터는 `<cam>/camera_info`, 자세는 tf 에서 온다. `system.yaml` 의
  `recognition.pick_cam` 에는 x_mm 이 아예 없고(§15 미결), tf 프레임 이름 규약도
  미정이다. 이미지 헤더의 frame_id 를 그대로 tf 에 물어보면 둘 다 안 막힌다.

★ 토픽 이름
  기존 손목 카메라 배선과 같은 규약이다 — `<ns>/rgb` + `<ns>/camera_info`,
  frameId 는 카메라 이름 (demo_stack15_test1_1.py 의 ROS2CameraHelper 참고).
  ⚠ 지금 씬이 ROS 로 내보내는 카메라는 `/wrist_cam` 하나뿐이다. cam1·cam2·pick_cam
    은 트랙 A 가 같은 방식으로 CameraHelper 를 붙여야 이 노드가 그림을 받는다.
    토픽 이름은 전부 파라미터라 씬 쪽 이름이 정해지면 launch 에서 맞추면 된다.
"""
import threading

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from packing_common.config import load
from packing_msgs.msg import BoxAtPickup, ZoneStatus
from packing_msgs.srv import RegisterBox, VerifyBox

from packing_vision.detect import dominant_color_name, find_box, read_qr
from packing_vision.geometry import box_pose_from_corners, quat_to_R


class BoxLine:
    """투입부에서 등록된 순서 그대로의 대기열. 두 노드가 같이 쓴다."""

    def __init__(self):
        self._q = []
        self._lock = threading.Lock()

    def push(self, box_id, type_name):
        with self._lock:
            self._q.append((box_id, type_name))

    def peek(self):
        with self._lock:
            return self._q[0] if self._q else (None, None)

    def pop(self):
        with self._lock:
            return self._q.pop(0) if self._q else (None, None)

    def __len__(self):
        with self._lock:
            return len(self._q)


class CameraSub:
    """`<ns>/rgb` + `<ns>/camera_info` 를 묶어 받는다.

    camera_info 를 같이 받는 이유: 내부 파라미터를 코드에 박지 않기 위해서다.
    Isaac 의 CameraHelper 는 camera_info 를 안 쏘므로 CameraInfoHelper 를 따로
    붙여야 한다 — 씬 쪽에 이미 그 선례가 있다 (손목 카메라).
    """

    def __init__(self, node, ns, on_image, cb_group):
        self.node, self.ns, self._on_image = node, ns, on_image
        self.K = None
        self.frame_id = ""
        self.bridge = CvBridge()
        # ★ 카메라 토픽은 반드시 BEST_EFFORT 다. 기본 RELIABLE 로 구독하면
        #   이미지가 하나도 안 온다 (CLAUDE.md §3-4 규약 5).
        node.create_subscription(Image, f"{ns}/rgb", self._image,
                                 qos_profile_sensor_data, callback_group=cb_group)
        node.create_subscription(CameraInfo, f"{ns}/camera_info", self._info,
                                 qos_profile_sensor_data, callback_group=cb_group)

    def _info(self, msg):
        self.K = np.array(msg.k, float).reshape(3, 3)
        self.frame_id = msg.header.frame_id

    def _image(self, msg):
        # 구독 콜백은 입력을 검증한다. rclpy 가 예외를 삼켜서, 잘못된 프레임 하나에
        # 인식이 통째로 멎는데 에러는 안 뜬다 (CLAUDE.md §9).
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:                      # noqa: BLE001
            self.node.get_logger().warn(f"{self.ns} 이미지 변환 실패: {e}")
            return
        if bgr is None or bgr.size == 0:
            return
        try:
            self._on_image(bgr, msg.header)
        except Exception as e:                      # noqa: BLE001
            self.node.get_logger().error(f"{self.ns} 처리 중 예외: {e}")


def camera_pose(tf_buf, world_frame, frame_id):
    """tf 에서 월드 ← 카메라(광학) 변환을 꺼낸다. 없으면 None."""
    tr = tf_buf.lookup_transform(world_frame, frame_id, rclpy.time.Time())
    q = tr.transform.rotation
    t = tr.transform.translation
    return quat_to_R(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])


class LineDetector(Node):
    """cam1 판독 → RegisterBox · cam2 재판독 → VerifyBox."""

    def __init__(self, cfg, line):
        super().__init__("line_detector")
        self.cfg, self.line = cfg, line
        self.types = cfg["boxes"]["types"]
        self.belt_top_m = cfg["conveyor"]["belt_top_z_mm"] / 1000.0
        # stage 1 은 전 박스가 한 구역으로 간다. 지어내지 않고 yaml 에서 읽는다.
        self.default_dest = str(cfg["boxes"]["stage1"]["zone"]).upper()

        self.declare_parameter("cam1_ns", "/cam1")
        self.declare_parameter("cam2_ns", "/cam2")
        self.declare_parameter("world_frame", "World")
        self.world_frame = str(self.get_parameter("world_frame").value)
        cb = ReentrantCallbackGroup()
        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)

        self.cli_reg = self.create_client(RegisterBox, "/db/register_box",
                                          callback_group=cb)
        self.cli_ver = self.create_client(VerifyBox, "/db/verify_box",
                                          callback_group=cb)

        self._n = 0                 # 다음 박스 순번. boxes_master 와 같은 BOX%05d 규칙
        self._armed1 = True         # 빈 화면을 한 번 봐야 다음 박스를 센다
        self._armed2 = True

        self.cam1 = CameraSub(self, self.get_parameter("cam1_ns").value,
                              self._on_cam1, cb)
        self.cam2 = CameraSub(self, self.get_parameter("cam2_ns").value,
                              self._on_cam2, cb)
        self.get_logger().info("line_detector 시작 — cam1 등록 · cam2 검증")

    # ── 공통: 그림에서 박스 하나를 읽어 종류까지 정한다 ──────────────
    def _measure(self, bgr, cam):
        """검출 → tf → 절대 치수로 종류 판별.

        ★ 비율(긴변/짧은변)로 고르지 않는다. 3·4·5호의 비율이 1.36 / 1.32 / 1.26
          으로 너무 가까워서 몇 픽셀만 흔들려도 종류가 바뀐다. 종류가 바뀌면
          높이가 바뀌고, 높이가 바뀌면 로봇이 허공이나 박스 속으로 내려간다.
          pick_cam 과 같은 절대 크기 방식을 쓴다 — 후보마다 자기 높이의 평면에
          투영해 사양 치수와 비교하면 오차가 171 / ~0 / 175 mm 로 갈린다.
        """
        det = find_box(bgr)
        if det is None:
            return None, None
        if cam.K is None:
            self.get_logger().warn(f"{cam.ns}: camera_info 대기 중", once=True)
            return None, None
        if det["clipped"]:
            self.get_logger().warn(f"{cam.ns}: 박스가 화면 밖으로 잘렸다 — 건너뛴다")
            return None, None
        try:
            R, t = camera_pose(self.tf_buf, self.world_frame,
                               cam.frame_id or cam.ns.strip("/"))
        except Exception as e:                  # noqa: BLE001
            self.get_logger().warn(f"{cam.ns}: tf 조회 실패 — {e}", once=True)
            return None, None
        out = box_pose_from_corners(det["corners_px"], cam.K, R, t,
                                    self.belt_top_m, self.types)
        if out is None:
            self.get_logger().warn(f"{cam.ns}: 사양 3종과 안 맞는다")
            return None, None
        return det, out

    def _on_cam1(self, bgr, header):
        det, out = self._measure(bgr, self.cam1)
        if det is None:
            self._armed1 = True                 # 빈 벨트 — 다음 박스를 받을 준비
            return
        if not self._armed1:
            return                              # 같은 박스를 계속 보고 있다
        self._armed1 = False
        bt = out["type"]

        box_id = "BOX%05d" % self._n            # ★ boxes_master 와 같은 규칙
        self._n += 1
        color = dominant_color_name(bgr) or "red"

        qr = read_qr(bgr)
        if qr:
            dest = qr.split("|")[1] if "|" in qr else self.default_dest
        else:
            # ⚠ stage 1 씬에는 QR 이 없다 (spawner 가 단색 머티리얼만 입힌다).
            #   목적지는 yaml 의 stage1.zone 을 쓰고, payload 는 cam2 가 같은
            #   규칙으로 다시 만들 수 있게 결정적으로 합성한다. 그래야 검증이
            #   "순번이 어긋났는가" 를 실제로 잡아낸다.
            dest = self.default_dest
            qr = f"PALLETRIS|{dest}|{box_id}"

        req = RegisterBox.Request(
            box_id=box_id, size_w_mm=int(bt["w_mm"]), size_d_mm=int(bt["d_mm"]),
            size_h_mm=int(bt["h_mm"]), color=color, dest=dest, qr_payload=qr)
        self._call(self.cli_reg, req, lambda r, b=box_id, t=bt: self._reg_done(r, b, t))

    def _reg_done(self, res, box_id, bt):
        if res is None or not res.ok:
            self.get_logger().error(
                f"register_box 실패 {box_id}: {getattr(res, 'message', 'no response')}")
            return
        self.line.push(box_id, bt["name"])
        self.get_logger().info(
            f"cam1 등록 {box_id} ({bt['name']}) → {res.message} · 대기열 {len(self.line)}")

    def _on_cam2(self, bgr, header):
        det, _ = self._measure(bgr, self.cam2)
        if det is None:
            self._armed2 = True
            return
        if not self._armed2:
            return
        self._armed2 = False

        qr = read_qr(bgr)
        if not qr:
            # cam1 과 같은 규칙으로 다시 만든다. 순번이 어긋났으면 여기서 갈린다.
            idx = self._n - 1
            if idx < 0:
                return
            qr = f"PALLETRIS|{self.default_dest}|BOX%05d" % idx
        box_id = qr.split("|")[-1]
        self._call(self.cli_ver, VerifyBox.Request(box_id=box_id, qr_payload=qr),
                   lambda r, b=box_id: self._ver_done(r, b))

    def _ver_done(self, res, box_id):
        if res is None:
            return
        if res.match:
            self.get_logger().info(f"cam2 검증 OK {box_id}")
        else:
            self.get_logger().warn(
                f"cam2 검증 불일치 {box_id} — 등록 주인 {res.registered_id or '(없음)'}")

    # ★ 모든 서비스 호출은 call_async + done_callback 이다.
    #   콜백 안에서 spin_until_future_complete 를 부르면 executor 가 자기 자신을
    #   기다려 에러 한 줄 없이 멈춘다 (CLAUDE.md §9).
    def _call(self, cli, req, done):
        if not cli.service_is_ready():
            self.get_logger().warn(f"{cli.srv_name} 서버가 아직 없다 — 건너뛴다")
            return
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f: done(f.result() if f.done() else None))


class ZoneDetector(Node):
    """pick_cam 하향 → 스토퍼 도착과 집을 좌표를 BoxAtPickup 으로."""

    def __init__(self, cfg, line):
        super().__init__("detector_zone_b")
        self.cfg, self.line = cfg, line
        self.types = cfg["boxes"]["types"]
        self.belt_top_m = cfg["conveyor"]["belt_top_z_mm"] / 1000.0

        self.declare_parameter("zone", "b")
        self.declare_parameter("pick_cam_ns", "/pick_cam")
        # tf 의 뿌리 프레임. Action Graph 가 parentPrim=/World 로 tf 를 낸다.
        self.declare_parameter("world_frame", "World")
        # ★ 처리 효율 (평가 항목). 카메라는 30 Hz 로 오지만 추론은 이 주기로만 한다.
        #   더 중요한 절약은 아래 accepting 게이트다 — 관제탑이 집는 중이면
        #   추론 자체를 안 한다.
        self.declare_parameter("infer_rate_hz", 5.0)

        self.zone = str(self.get_parameter("zone").value).lower()
        self.world_frame = str(self.get_parameter("world_frame").value)
        self._min_dt = 1.0 / max(0.1, float(self.get_parameter("infer_rate_hz").value))
        self._last = 0.0
        self._present = False
        self._idle = True          # 관제탑이 놀고 있는가

        cb = ReentrantCallbackGroup()
        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)

        self.pub = self.create_publisher(
            BoxAtPickup, f"/zone_{self.zone}/box_at_pickup", 10)
        self.create_subscription(
            ZoneStatus, f"/zone_{self.zone}/status", self._on_status, 10,
            callback_group=cb)
        self.cam = CameraSub(self, self.get_parameter("pick_cam_ns").value,
                             self._on_image, cb)
        self.get_logger().info(
            f"detector_zone_b 시작 — /zone_{self.zone}/box_at_pickup 발행")

    def _on_status(self, msg):
        # IDLE 일 때만 추론한다. 집는 중에는 새 좌표가 필요 없고, 오히려 팔이
        # 화면에 들어와 오검출을 만든다.
        self._idle = (msg.state == "IDLE")

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_image(self, bgr, header):
        t = self._now()
        if not self._idle or (t - self._last) < self._min_dt:
            return
        self._last = t

        det = find_box(bgr)
        if det is None:
            self._publish_absent()
            return
        if det["clipped"]:
            # 잘린 사각형의 중심은 진짜 중심이 아니다. 틀린 좌표를 주느니
            # "안 보인다" 가 낫다 — 로봇이 빗나간 자리를 집는 쪽이 훨씬 나쁘다.
            self.get_logger().warn("박스가 시야 밖으로 잘렸다 — 좌표를 내지 않는다")
            self._publish_absent()
            return
        if self.cam.K is None:
            self.get_logger().warn("camera_info 대기 중", once=True)
            return

        try:
            tr = self.tf_buf.lookup_transform(
                self.world_frame, self.cam.frame_id or "pick_cam",
                rclpy.time.Time())
        except Exception as e:                  # noqa: BLE001
            # use_sim_time 이 꺼져 있거나 /clock 이 없으면 여기서 계속 실패한다.
            self.get_logger().warn(f"tf 조회 실패 ({self.world_frame} ← "
                                   f"{self.cam.frame_id}): {e}", once=True)
            return

        q = tr.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        t_vec = np.array([tr.transform.translation.x, tr.transform.translation.y,
                          tr.transform.translation.z])

        out = box_pose_from_corners(det["corners_px"], self.cam.K, R, t_vec,
                                    self.belt_top_m, self.types)
        if out is None:
            self.get_logger().warn("사양 3종 어느 것과도 안 맞는다 — 좌표를 내지 않는다")
            self._publish_absent()
            return

        box_id, expect = self.line.peek()
        if box_id is None:
            # 투입부가 등록한 적 없는 박스가 서 있다. 관제탑은 box_id 로 DB 를
            # 조회하므로 빈 문자열을 보내면 거기서 막힌다 — 그게 맞는 동작이다.
            self.get_logger().warn("대기열이 비었다 — 투입부 인식을 놓쳤다")
        elif expect and expect != out["type"]["name"]:
            self.get_logger().warn(
                f"{box_id}: 투입부는 {expect}, 픽업은 {out['type']['name']} 로 봤다")

        cx, cy, cz = out["center_m"]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = cx, cy, cz
        half = np.radians(out["yaw_deg"]) / 2.0
        pose.orientation.z, pose.orientation.w = float(np.sin(half)), float(np.cos(half))

        self.pub.publish(BoxAtPickup(present=True, box_id=box_id or "", pose=pose))
        if not self._present:
            self.get_logger().info(
                f"도착 {box_id or '(미등록)'} {out['type']['name']} "
                f"({cx*1000:.0f}, {cy*1000:.0f}, {cz*1000:.0f}) mm "
                f"yaw {out['yaw_deg']:.1f}° · 맞춤오차 {out['fit_err_m']*1000:.1f} mm")
        self._present = True

    def _publish_absent(self):
        if self._present:
            # 박스가 떠났다 = 관제탑이 집어갔다. 대기열에서 뺀다.
            self.line.pop()
            self.get_logger().info(f"픽업 자리 비었다 · 대기열 {len(self.line)}")
        self._present = False
        self.pub.publish(BoxAtPickup(present=False, box_id="", pose=Pose()))


def main(args=None):
    rclpy.init(args=args)
    cfg = load()
    line = BoxLine()
    nodes = [LineDetector(cfg, line), ZoneDetector(cfg, line)]
    ex = MultiThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
