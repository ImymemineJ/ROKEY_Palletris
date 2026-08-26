#!/usr/bin/env python3
"""인식 노드 — 투입부 등록·검증(cam1·cam2) + 픽업 좌표(pick_cam).

CLAUDE.md §8 의 "라인 인식 노드 2 · 구역 인식 노드 3" 을 구현한다.

★ 왜 한 프로세스에 같이 두나
  둘이 **박스 순번을 공유해야** 한다. 투입부에서 BOX007 로 등록한 박스가
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

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from packing_common.config import load
from packing_common.units import m_to_mm   # ★ 직접 *1000 금지 (CLAUDE.md §4)
from packing_msgs.msg import BoxAtPickup, ZoneStatus
from packing_msgs.srv import GetBoxInfo, RegisterBox, ReportMiss, VerifyBox

from packing_vision.detect import dominant_color_name, find_box, read_qr
from packing_vision.label import parse_payload
from packing_vision.geometry import box_pose_from_corners, quat_to_R


# ★ 예전에 여기 있던 BoxLine(등록 순서 대기열)은 없앴다 (2026-08-25).
#   투입부와 픽업이 **다른 프로세스**가 됐고(구역별로 따로 띄운다), 구역이 셋이면
#   단일 FIFO 자체가 틀린다 — 분기기가 박스를 A/B/C 로 갈라 보내므로 도착 순서가
#   등록 순서와 다르다. 지금은 픽업 카메라가 박스 윗면 QR 을 직접 읽는다.


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

    def __init__(self, cfg):
        super().__init__("line_detector")
        self.cfg = cfg
        self._n_reg = 0             # 등록 누계 (로그·지표용)
        self.types = cfg["boxes"]["types"]
        self.belt_top_m = cfg["conveyor"]["belt_top_z_mm"] / 1000.0

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
        # ★ 인식률의 분모를 만드는 통로다. 박스를 봤는데 식별 못 하면 여기로 알린다 —
        #   안 알리면 그 박스는 통계에서 사라져 인식률이 언제나 100 % 로 나온다.
        self.cli_miss = self.create_client(ReportMiss, "/db/report_miss",
                                           callback_group=cb)

        # ★ 박스 순번 카운터를 두지 않는다. box_id 는 **QR 이 정본**이다.
        #   세어서 만들면 cam1 이 한 개만 놓쳐도 그 뒤 전부가 한 칸씩 밀려
        #   DB 의 다른 박스 기록을 덮어쓴다 — 에러 없이 전부 틀리는 쪽이다.
        self._miss1 = 0             # QR 판독 실패 누계 (지표·로그용)
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
            self._report_miss(cam.ns.strip("/"), "SIZE_MISMATCH")
            return None, None
        # ★ 원시 측정값을 남긴다. 위에서 사양 3종 중 하나로 **스냅**하기 때문에
        #   DB 에 들어가는 것은 사양 치수(340x250 …)지 잰 값이 아니다.
        #   카메라 교정·인식 정확도를 볼 때 필요한 건 스냅 전 숫자다.
        #   info 가 아니라 debug 인 이유: 프레임마다 나오면 다른 로그가 안 보인다.
        #     보려면  --ros-args --log-level debug
        lo, sh = out["measured_m"]
        bt = out["type"]
        self.get_logger().debug(
            f"{cam.ns}: 실측 {lo * 1000:.1f} x {sh * 1000:.1f} mm "
            f"→ {bt['name']} ({bt['w_mm']}x{bt['d_mm']}) "
            f"오차 {out['fit_err_m'] * 1000:.1f} mm")
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

        qr = read_qr(bgr, det["corners_px"])
        box_id = parse_payload(qr)
        if box_id is None:
            # ★ QR 을 못 읽었다 = 이 박스는 등록하지 않고 흘려보낸다.
            #   운송장번호를 지어내면 안 된다 (위 self._miss1 주석 참조).
            #   등록 안 된 박스는 중재자가 미등록으로 보고 RETURN 시킨다 —
            #   설계결정서 §2.2 가 요구한 "리젝트 라인이 실제로 발동" 이 이 경로다.
            #   yaml 의 recognition.cam1_box_miss_rate 로 인위적 드롭을 주입할
            #   필요도 없다. 라벨이 흐리거나 잘리면 여기가 자연히 걸린다.
            self._miss1 += 1
            self.get_logger().warn(
                f"cam1 QR 판독 실패 (누계 {self._miss1}) — 등록 건너뜀"
                + (f" · 읽힌 값 {qr!r}" if qr else " · QR 이 안 보인다"))
            self._report_miss("cam1", "QR_UNREADABLE")
            return
        color = dominant_color_name(bgr) or "red"

        # ★ dest 를 비워 보낸다. QR 에는 운송장번호만 있고 구역 글자가 없다 —
        #   구역은 db_service_node 가 주소(boxes_master.address)의 동으로 정한다.
        #   여기서 추측해 채우면 인식이 구역까지 아는 척하게 되고, 실제 분류
        #   근거(주소)와 어긋나도 아무도 모른다 (설계결정서 §5-3).
        req = RegisterBox.Request(
            box_id=box_id, size_w_mm=int(bt["w_mm"]), size_d_mm=int(bt["d_mm"]),
            size_h_mm=int(bt["h_mm"]), color=color, dest="", qr_payload=qr)
        self._call(self.cli_reg, req, lambda r, b=box_id, t=bt: self._reg_done(r, b, t))

    def _report_miss(self, where_at, reason):
        """박스를 봤는데 식별 못 했다 — DB 에 알린다 (인식률의 분모).

        ★ 응답을 기다리지 않는다. 이건 통계용이라 라인을 세울 이유가 없다.
        """
        if not self.cli_miss.service_is_ready():
            self.get_logger().warn("/db/report_miss 가 아직 없다 — 통계에서 빠진다",
                                   once=True)
            return
        self.cli_miss.call_async(ReportMiss.Request(where_at=where_at, reason=reason))

    def _reg_done(self, res, box_id, bt):
        if res is None or not res.ok:
            self.get_logger().error(
                f"register_box 실패 {box_id}: {getattr(res, 'message', 'no response')}")
            return
        self._n_reg += 1
        self.get_logger().info(
            f"cam1 등록 {box_id} ({bt['name']}) → {res.message} · 누계 {self._n_reg}")

    def _on_cam2(self, bgr, header):
        det, _ = self._measure(bgr, self.cam2)
        if det is None:
            self._armed2 = True
            return
        if not self._armed2:
            return
        self._armed2 = False

        qr = read_qr(bgr, det["corners_px"])
        box_id = parse_payload(qr)
        if box_id is None:
            # cam1 이 등록한 박스인데 cam2 가 못 읽었다 = 검증 불가.
            # 여기서 box_id 를 추측해 VerifyBox 를 부르면 **엉뚱한 박스를
            # 통과시킨다.** 부르지 않는 쪽이 안전하다.
            self.get_logger().warn(
                "cam2 QR 판독 실패 — 검증 건너뜀"
                + (f" · 읽힌 값 {qr!r}" if qr else " · QR 이 안 보인다"))
            return
        self._call(self.cli_ver, VerifyBox.Request(box_id=box_id, qr_payload=qr),
                   lambda r, b=box_id: self._ver_done(r, b))

    def _ver_done(self, res, box_id):
        if res is None:
            return
        if res.match:
            self.get_logger().info(f"cam2 검증 OK {box_id}")
        else:
            # ★ 경고만 남기고 끝내면 순서가 꼬인 박스가 그대로 지나간다.
            #   그렇다고 여기서 버리면 멀쩡한 박스를 죽일 수 있다 — cam2 가 잘못
            #   읽었을 수도 있기 때문이다. 그래서 **기록만** 남기고 판단은 미룬다.
            #   불일치가 늘어나는지는 지표(scan_misses)로 보인다.
            #   ⚠ 미결: 불일치를 REJECTED 로 할지 재순환으로 돌릴지는 팀이 정할 값이다.
            self.get_logger().warn(
                f"cam2 검증 불일치 {box_id} — 등록 주인 {res.registered_id or '(없음)'}")
            self._report_miss("cam2", "VERIFY_MISMATCH")

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
    """구역 하나의 하향 카메라 → 스토퍼 도착과 집을 좌표를 BoxAtPickup 으로.

    ★ **구역마다 따로 띄운다** (CLAUDE.md §8 "구역 인식 노드 3"). 한 프로세스에
      몰면 파이썬 GIL 때문에 세 구역 추론이 줄을 서서 돈다 — 구역을 늘린 의미가
      없어진다. 프로세스를 나눠야 코어를 실제로 나눠 쓴다.

    ★ box_id 는 **자기 카메라가 읽은 QR** 에서 나온다 (2026-08-25).
      예전에는 투입부 노드가 등록한 순서를 in-process 큐로 넘겨받았는데,
      그 방식은 세 가지가 깨진다:
        1. 프로세스를 나누면 큐를 공유할 수 없다
        2. 구역이 셋이면 **단일 FIFO 자체가 틀린다** — 분기기가 박스를 A/B/C 로
           갈라 보내므로 도착 순서가 등록 순서와 다르다
        3. 투입부가 한 개만 놓쳐도 그 뒤 전부가 한 칸씩 밀린다
      픽업 카메라는 박스 윗면을 바로 위에서 본다. 라벨이 거기 있으므로
      **읽으면 그만이다.** 구역끼리 아무것도 공유하지 않는다.
    """

    def __init__(self, cfg):
        # ★ 노드 이름은 launch 가 `name="detector_zone_a"` 로 붙인다.
        #   구역마다 **프로세스가 따로**라 name= 을 줘도 안전하다
        #   (한 프로세스에 노드가 둘일 때만 위험하다 — bringup.launch.py 주석 참조).
        super().__init__("zone_detector")
        self.cfg = cfg
        self.types = cfg["boxes"]["types"]
        self.belt_top_m = cfg["conveyor"]["belt_top_z_mm"] / 1000.0

        self.declare_parameter("zone", "b")
        self.declare_parameter("pick_cam_ns", "/pick_cam")
        # tf 의 뿌리 프레임. Action Graph 가 parentPrim=/World 로 tf 를 낸다.
        self.declare_parameter("world_frame", "World")
        # ★ 처리 효율 (평가 항목). 카메라는 30 Hz 로 오지만 추론은 이 주기로만 한다.
        #   더 중요한 절약은 아래 IDLE 게이트다 — 관제탑이 집는 중이면 추론 자체를 안 한다.
        self.declare_parameter("infer_rate_hz", 5.0)

        self.zone = str(self.get_parameter("zone").value).lower()
        ns = str(self.get_parameter("pick_cam_ns").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self._min_dt = 1.0 / max(0.1, float(self.get_parameter("infer_rate_hz").value))
        self._last = 0.0
        self._present = False
        self._idle = True          # 관제탑이 놀고 있는가
        self._seen = None          # 직전에 발행한 box_id (중복 로그 억제)

        cb = ReentrantCallbackGroup()
        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)

        self.cli_info = self.create_client(GetBoxInfo, "/db/get_box_info",
                                           callback_group=cb)
        self.pub = self.create_publisher(
            BoxAtPickup, f"/zone_{self.zone}/box_at_pickup", 10)
        self.create_subscription(
            ZoneStatus, f"/zone_{self.zone}/status", self._on_status, 10,
            callback_group=cb)
        self.cam = CameraSub(self, ns, self._on_image, cb)
        self.get_logger().info(
            f"detector_zone_{self.zone} 시작 — {ns} -> /zone_{self.zone}/box_at_pickup")

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
                self.world_frame, self.cam.frame_id or self.cam.ns.strip("/"),
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

        # ★ 원시 측정값 (스냅 전). LineDetector._measure 의 같은 로그와 짝이다.
        #   여기 값이 로봇이 실제로 집으러 갈 때 쓰는 판정이다.
        _lo, _sh = out["measured_m"]
        self.get_logger().debug(
            f"실측 {_lo * 1000:.1f} x {_sh * 1000:.1f} mm "
            f"→ {out['type']['name']} "
            f"({out['type']['w_mm']}x{out['type']['d_mm']}) "
            f"오차 {out['fit_err_m'] * 1000:.1f} mm")

        # ★ box_id 는 여기서 읽는다. 다른 노드에서 받아오지 않는다.
        #
        # ★ 한 번 읽으면 박스가 떠날 때까지 **기억한다.** 매 프레임 다시 읽으려
        #   들면 안 된다 — 해독 실패는 무작위가 아니라 **자세에 따라 결정적**이라,
        #   나쁜 자세로 멈춘 박스는 몇 번을 다시 봐도 계속 실패한다.
        #   실측(투영 48 회): 자세를 흔들면 98 % 지만, 한 자세로 굳으면 0 % 아니면 100 % 다.
        #   박스는 스토퍼에서 멈춰 있으므로 한 번만 맞히면 된다.
        box_id = parse_payload(read_qr(bgr, det["corners_px"])) or ""
        if not box_id and self._present and self._seen:
            # 같은 박스를 계속 보고 있다 — 아까 읽은 값을 그대로 쓴다.
            box_id = self._seen
        elif not box_id:
            # 라벨을 아직 한 번도 못 읽었다. 관제탑은 box_id 로 DB 를 조회하므로
            # 빈 문자열이면 거기서 막힌다 — 그게 맞는 동작이다.
            # 지어내면 남의 박스 기록을 덮는다.
            self.get_logger().warn("픽업 QR 판독 실패 — box_id 없이 발행한다",
                                   throttle_duration_sec=2.0)

        # ★ 계약은 mm 정수다 (CLAUDE.md §3-2). 내부 계산은 m 로 하지만
        #   토픽에 실을 때 여기서 한 번만 mm 로 바꾼다.
        #   z 는 싣지 않는다 — 벨트 상면(800mm)은 고정이라 관제탑이 안다.
        cx, cy, cz = out["center_m"]
        self.pub.publish(BoxAtPickup(
            present=True, box_id=box_id,
            x_mm=m_to_mm(cx), y_mm=m_to_mm(cy),
            yaw_deg=float(out["yaw_deg"])))
        if box_id and box_id != self._seen:
            self.get_logger().info(
                f"도착 {box_id or '(미상)'} {out['type']['name']} "
                f"({m_to_mm(cx)}, {m_to_mm(cy)}, {m_to_mm(cz)}) mm "
                f"yaw {out['yaw_deg']:.1f}° · 맞춤오차 {out['fit_err_m']*1000:.1f} mm")
            self._crosscheck(box_id, out["type"])
        self._present = True
        if box_id:
            self._seen = box_id

    def _crosscheck(self, box_id, btype):
        """투입부가 등록한 치수와 픽업에서 잰 치수가 같은가 — 비동기로 확인만.

        ★ 발행을 여기에 매달지 않는다. 서비스 응답을 기다렸다가 좌표를 내면
          로봇이 그만큼 늦게 움직인다. 어긋나면 경고만 남기고 지표로 쓴다.
        """
        if not self.cli_info.service_is_ready():
            return
        fut = self.cli_info.call_async(GetBoxInfo.Request(box_id=box_id))

        def done(f):
            r = f.result() if f.done() else None
            if r is None or not r.found:
                self.get_logger().warn(f"{box_id}: 투입부 등록 기록이 없다")
                return
            want = (r.info.size_w_mm, r.info.size_d_mm, r.info.size_h_mm)
            got = (btype["w_mm"], btype["d_mm"], btype["h_mm"])
            if want != got:
                self.get_logger().warn(
                    f"{box_id}: 투입부는 {want}, 픽업은 {got} 로 봤다")

        fut.add_done_callback(done)

    def _publish_absent(self):
        if self._present:
            self.get_logger().info("픽업 자리 비었다")
        self._present, self._seen = False, None
        self.pub.publish(BoxAtPickup(present=False, box_id=""))


def _spin(node):
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """투입부 인식 — cam1 등록 · cam2 검증. 라인이 하나라 이 노드도 하나다."""
    rclpy.init(args=args)
    _spin(LineDetector(load()))


def zone_main(args=None):
    """구역 인식 — 구역마다 **따로 띄운다** (CLAUDE.md §8 "구역 인식 노드 3").

    한 프로세스에 셋을 몰면 파이썬 GIL 때문에 추론이 줄을 서서 돈다.
    """
    rclpy.init(args=args)
    _spin(ZoneDetector(load()))


if __name__ == "__main__":
    main()
