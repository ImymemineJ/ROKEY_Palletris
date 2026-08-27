#!/usr/bin/env python3
"""인식 노드 — 투입부 등록·검증(cam1·cam2) + 픽업 좌표(pick_cam).

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : ② 투입부 인식(cam1·cam2) · ④ 픽업 좌표(pick_cam)
    ║     불러 주는 쪽        : launch
    ║     이 파일이 부르는 쪽 : db 서비스 · 관제탑 토픽
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

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

★ 깊이 교차검증 (2026-08-26 추가)
  RGB 만으로는 박스 높이를 **추측**한다 — 사양 3종 평면에 각각 투영해 제일 잘
  맞는 것을 고른다. 종류를 틀리면 평면이 60~130 mm 어긋나고 **중심 x, y 까지
  같이 틀어진다.** 그래서 같은 카메라의 `<ns>/depth` 를 같이 받아 윗면 z 를
  **직접 재고**, 두 결과를 대조한다 (`depth.py`). 결과는 `VisionCheck` 토픽으로 나간다.
  ⚠ 깊이가 없으면 **지금까지의 동작 그대로** 돈다. 붙였다가 라인이 서면 안 되므로
    없을 때가 기본 경로이고, 있을 때만 확인이 하나 더 붙는 구조다.

★ 토픽 이름
  기존 손목 카메라 배선과 같은 규약이다 — `<ns>/rgb` + `<ns>/camera_info`,
  frameId 는 카메라 이름 (demo_stack15_test1_1.py 의 ROS2CameraHelper 참고).
  ⚠ 지금 씬이 ROS 로 내보내는 카메라는 `/wrist_cam` 하나뿐이다. cam1·cam2·pick_cam
    은 트랙 A 가 같은 방식으로 CameraHelper 를 붙여야 이 노드가 그림을 받는다.
    토픽 이름은 전부 파라미터라 씬 쪽 이름이 정해지면 launch 에서 맞추면 된다.
"""

import collections
import time

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
from packing_msgs.msg import BoxAtPickup, VisionCheck, ZoneStatus
from packing_msgs.srv import GetBoxInfo, RegisterBox, ReportMiss, VerifyBox

from packing_vision import depth as depthlib
from packing_vision.detect import dominant_color_name, find_box, read_qr
from packing_vision.label import parse_payload
from packing_vision import geometry as _geo
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

    def __init__(self, node, ns, on_image, cb_group, depth_suffix=""):
        self.node, self.ns, self._on_image = node, ns, on_image
        self.K = None
        self.frame_id = ""
        self.bridge = CvBridge()
        # ★ 깊이는 **최근 몇 장**을 stamp 와 함께 들고, 호출부가 처리 중인 RGB
        #   프레임에 가장 가까운 것을 고른다 (`depth_at`).
        #
        #   왜 "최신 한 장" 이면 안 되나 (실측): RGB 콜백이 QR 디코드로 100 ms 가량
        #   붙잡혀 있는 동안 다음 깊이 프레임이 들어와 최신 자리를 덮는다. 그러면
        #   짝이 한 프레임 어긋나 그 프레임의 교차검증이 통째로 버려진다 —
        #   8 박스 실행에서 **깊이 사용률이 38 %** 까지 떨어졌다. 최근 몇 장을
        #   들고 있으면 같은 실행이 100 % 가 된다. 유실이 아니라 **지연**이었다.
        #
        #   왜 오래된 것을 그냥 쓰면 안 되나: 벨트가 0.5 m/s 다. 한 틱(0.1 초)이면
        #   50 mm 다. 그래서 고르되 **허용 시차 밖이면 안 쓴다** (max_stamp_skew_s).
        self._depth_buf = collections.deque(maxlen=6)   # [(stamp, img), ...]
        self.rgb_stamp = None
        # ★ 카메라 토픽은 반드시 BEST_EFFORT 다. 기본 RELIABLE 로 구독하면
        #   이미지가 하나도 안 온다 (CLAUDE.md §3-4 규약 5).
        node.create_subscription(Image, f"{ns}/rgb", self._image,
                                 qos_profile_sensor_data, callback_group=cb_group)
        node.create_subscription(CameraInfo, f"{ns}/camera_info", self._info,
                                 qos_profile_sensor_data, callback_group=cb_group)
        if depth_suffix:
            # ★ message_filters 로 시각 동기를 하지 않는다. 동기화기를 쓰면 짝이
            #   안 맞을 때 **RGB 까지 같이 버린다** — 깊이 토픽이 아예 없는 씬에서
            #   인식이 통째로 멎는다는 뜻이다. 여기서는 깊이가 부록이라
            #   "최신 것 + 시차 검사" 가 맞는 구조다. 시차가 크면 그 프레임만
            #   깊이를 안 쓰고 RGB 로 간다.
            # ★ RGB 와 **같은 QoS** 로 받는다 (BEST_EFFORT · 큐 5).
            #   한때 "최신 한 장만 필요하다"고 큐를 1 로 좁혔는데, 그게 오히려
            #   깊이만 24 % 를 떨어뜨렸다 (실측: rgb 5.1 Hz 수신 vs depth 3.9 Hz).
            #   콜백이 늦게 도는 동안 두 장이 겹치면 큐 1 은 한 장을 버린다.
            #   지금은 아래 링 버퍼가 stamp 로 짝을 고르므로 **여러 장 받는 편이 낫다** —
            #   버릴지 말지는 짝 맞추기가 정하지, 큐가 정할 일이 아니다.
            #   ⚠ RELIABLE 로 바꾸면 안 된다. 씬이 BEST_EFFORT 로 내므로 QoS 가
            #     안 맞아 **한 장도 안 온다** (CLAUDE.md §3-4 규약 5).
            node.create_subscription(Image, f"{ns}/{depth_suffix}", self._depth,
                                     qos_profile_sensor_data, callback_group=cb_group)

    @staticmethod
    def _stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _info(self, msg):
        """`camera_info` — 내부 파라미터 K 와 **프레임 이름**을 받는다.

        ★ `frame_id` 를 여기서 챙기는 게 핵심이다. 이 이름으로 tf 를 조회하므로
          씬 prim 이름이 바뀌어도 코드를 안 고친다. 카메라 자세를 코드에 박지 않는
          유일한 방법이다.
        """
        self.K = np.array(msg.k, float).reshape(3, 3)
        self.frame_id = msg.header.frame_id

    def _depth(self, msg):
        """32FC1(m) 또는 16UC1(mm) 을 받아 float32 · m 로 통일해 둔다.

        ★ 16UC1 을 같이 받는 이유: Isaac 은 32FC1 을 내지만 실기 RealSense 드라이버
          기본값은 16UC1(mm) 이다. 여기서 안 받아 두면 실기로 옮길 때 깊이가
          **에러 없이 0** 이 된다 — uint16 을 m 로 읽으면 전부 범위 밖이라
          "TOO_FEW_PIXELS" 로만 보이고 원인이 안 드러난다.
        """
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:                      # noqa: BLE001
            self.node.get_logger().warn(f"{self.ns} 깊이 변환 실패: {e}", once=True)
            return
        if img is None or img.size == 0:
            return
        img = np.asarray(img)
        if img.ndim == 3 and img.shape[2] == 1:
            img = img[:, :, 0]
        if img.dtype == np.uint16:
            img = img.astype(np.float32) / 1000.0
        self._depth_buf.append((self._stamp(msg.header),
                                img.astype(np.float32, copy=False)))

    def depth_at(self, stamp, tol_s):
        """`stamp` 에 가장 가까운 깊이 프레임. 허용 시차 밖이면 (None, skew).

        반환: (img, skew_s) — img 가 None 이면 못 찾은 것이고, skew 는 그때
        **가장 가까웠던 값**이다 (진단용. 얼마나 빗나갔는지 알아야 고칠 수 있다).
        """
        if not self._depth_buf:
            return None, None
        best_img, best = None, None
        for st, img in self._depth_buf:
            d = abs(st - stamp)
            if best is None or d < best:
                best, best_img = d, img
        return (best_img if best <= tol_s else None), best

    @property
    def has_depth(self):
        return bool(self._depth_buf)

    def _image(self, msg):
        """`<ns>/rgb` 구독 콜백. BGR 로 바꿔 상위 핸들러에 넘긴다.

        ★ **예외를 여기서 다 잡는다.** rclpy 는 콜백 예외를 삼키므로, 잘못된 프레임
          하나에 인식이 통째로 멎는데 에러는 한 줄도 안 뜬다 (CLAUDE.md §9).
        """
        # 구독 콜백은 입력을 검증한다. rclpy 가 예외를 삼켜서, 잘못된 프레임 하나에
        # 인식이 통째로 멎는데 에러는 안 뜬다 (CLAUDE.md §9).
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:                      # noqa: BLE001
            self.node.get_logger().warn(f"{self.ns} 이미지 변환 실패: {e}")
            return
        if bgr is None or bgr.size == 0:
            return
        self.rgb_stamp = self._stamp(msg.header)
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


class DepthCrossCheck:
    """깊이 교차검증 한 벌 — 설정 · 판정 · 발행을 한 곳에 모은다.

    ★ 왜 클래스 하나로 묶나
      라인 인식(cam1·cam2)과 구역 인식(pick_cam)이 **같은 확인을 해야** 한다.
      함수로 흩어 두면 한쪽만 고쳐지고, 그 상태로도 에러가 안 난다 —
      이 저장소가 반복해서 데인 종류의 버그다 (CLAUDE.md §10-3 "돌아갈 때만 낮았다").
      노드마다 이 객체를 하나씩 갖고, 바뀌는 것은 생성 인자뿐이다.

    ★ 설정 3층: DEFAULTS(코드) ← recognition.depth(yaml) ← ROS 파라미터(런치).
      런치에서 `-p depth.mode:=off` 한 줄로 되돌릴 수 있다. 통합 중에 깊이가
      말썽이면 코드를 안 고치고 끄는 길이 있어야 한다.
    """

    # 런타임에 덮어쓸 수 있는 것들. 자료형이 곧 ROS 파라미터 자료형이다.
    PARAMS = ("enabled", "mode", "topic_suffix", "is_range", "shrink_ratio",
              "min_valid_ratio", "min_px", "plane_gate_mm", "mad_k",
              "max_spread_mm", "h_weight", "z_tol_mm", "xy_tol_mm", "yaw_tol_deg",
              "max_stamp_skew_s", "reject_on_disagree")

    def __init__(self, node, cfg, topic, zone=""):
        self.node, self.zone = node, zone
        base = depthlib.merged_cfg(cfg.get("recognition"))
        # ★ 파라미터 기본값을 **yaml 이 이긴 값**으로 선언한다. 그래야 아무것도
        #   안 주면 yaml 이, 주면 런치가 이긴다 — 3층 우선순위가 이 한 줄에 있다.
        over = {}
        for k in self.PARAMS:
            node.declare_parameter(f"depth.{k}", base[k])
            over[k] = node.get_parameter(f"depth.{k}").value
        self.cfg = depthlib.merged_cfg(cfg.get("recognition"), over)
        self.types = cfg["boxes"]["types"]
        self.belt_top_m = cfg["conveyor"]["belt_top_z_mm"] / 1000.0
        self.pub = node.create_publisher(VisionCheck, topic, 10)
        # 지표용 누계. 로그만 남기면 발표에서 셀 수가 없다.
        self.n_total = self.n_agree = self.n_nodepth = 0
        node.get_logger().info(
            f"깊이 교차검증: {'켬' if self.on else '끔'} · mode={self.cfg['mode']} "
            f"· 허용 z {self.cfg['z_tol_mm']}mm / xy {self.cfg['xy_tol_mm']}mm "
            f"/ yaw {self.cfg['yaw_tol_deg']}도 → {topic}")

    @property
    def on(self):
        return bool(self.cfg["enabled"]) and self.cfg["mode"] != "off"

    @property
    def depth_suffix(self):
        """CameraSub 에 넘길 깊이 토픽 꼬리. 꺼져 있으면 "" — 구독조차 안 만든다."""
        return str(self.cfg["topic_suffix"]) if self.on else ""

    def measure(self, cam, corners_px, R, t, rgb_stamp=None, source_hint=""):
        """깊이로 윗면을 재고 자세를 뽑는다. (dep, plane, reason)."""
        if not self.on:
            return None, None, "DISABLED"
        if not cam.has_depth:
            return None, None, "NO_DEPTH_TOPIC"
        # ★ 짝 맞추기. **지금 처리 중인 프레임**의 stamp 로 고른다 —
        #   `cam.rgb_stamp`(마지막으로 **받은** 프레임)가 아니다. 둘은 다를 수 있다:
        #   MultiThreadedExecutor + ReentrantCallbackGroup 이라 이 함수가 도는 동안
        #   다음 RGB 가 들어와 `cam.rgb_stamp` 를 앞으로 민다. 실측에서 관측됐다.
        ref = rgb_stamp if rgb_stamp is not None else cam.rgb_stamp
        if ref is None:
            return None, None, "STALE"
        dimg, skew = cam.depth_at(ref, float(self.cfg["max_stamp_skew_s"]))
        if dimg is None:
            # 허용 시차 밖이다. 그 프레임만 건너뛴다 — 다음 프레임에 또 온다.
            # 자주 뜨면 씬의 깊이 발행이 느리거나 stamp 규약이 다른 것이다.
            self.node.get_logger().debug(
                f"{source_hint}: 깊이 짝이 없다 (최소 시차 "
                f"{(skew or 0.0) * 1000:.0f} ms)", throttle_duration_sec=5.0)
            return None, None, "STALE"
        plane, reason = depthlib.top_plane_z(dimg, corners_px,
                                             cam.K, R, t, self.cfg)
        if plane is None or reason:
            # NOT_FLAT 은 plane 값이 같이 온다 — 진단 토픽에 실어야 원인이 보인다.
            return None, plane, reason or "TOO_FEW_PIXELS"
        dep = depthlib.pose_from_plane(corners_px, cam.K, R, t, plane["z_m"],
                                       self.types, self.belt_top_m,
                                       self.cfg["h_weight"])
        # ★ None = 잰 높이가 최저 사양의 절반 미만 — **벨트를 본 것**이다
        #   (pose_from_plane 의 벨트 게이트 주석). 박스 실측이 아니므로 깊이
        #   없음으로 강등한다 — RGB 가 있으면 그쪽이, 없으면 그 프레임은 보류.
        return dep, plane, "" if dep is not None else "BELT"

    def evaluate(self, cam, corners_px, rgb_out, R, t, source, box_id="",
                 rgb_stamp=None):
        """확인하고 발행한다. **최종 채택할 pose** 와 판정을 돌려준다.

        반환: (out, res). out 이 None 이면 좌표를 내지 말라는 뜻이다
        (reject_on_disagree 가 켜져 있고 어긋난 경우뿐).
        """
        if not self.on:
            return rgb_out, {"verdict": "NO_DEPTH", "used": "RGB", "agree": False}

        dep, plane, reason = self.measure(cam, corners_px, R, t, rgb_stamp, source)
        res = depthlib.cross_check(rgb_out, dep, self.cfg)
        if dep is None:
            res["verdict"] = reason or res["verdict"]
        self._publish(source, box_id, rgb_out, dep, plane, reason, res)

        self.n_total += 1
        if dep is None:
            self.n_nodepth += 1
        elif res["agree"]:
            self.n_agree += 1
        else:
            # ★ 경고는 여기 한 곳에서만 낸다. 호출부마다 쓰면 문구가 갈린다.
            self.node.get_logger().warn(
                f"{source} 교차검증 불일치 [{res['verdict']}] {box_id or '(미상)'} — "
                f"RGB {rgb_out['type']['name'] if rgb_out else '(없음)'} vs "
                f"깊이 {dep['type']['name']} · "
                f"dz {res['dz_mm']:+.1f}mm · dxy {res['dxy_mm']:.1f}mm · "
                f"dyaw {res['dyaw_deg']:.1f}도")

        if res["used"] == "DEPTH" and dep is not None:
            return depthlib.as_rgb_shape(dep), res
        if res["used"] == "NONE":
            return None, res
        return rgb_out, res

    def _publish(self, source, box_id, rgb, dep, plane, reason, res):
        m = VisionCheck()
        m.stamp = self.node.get_clock().now().to_msg()
        m.source, m.zone, m.box_id = source, self.zone, box_id or ""
        if rgb is not None:
            cx, cy, cz = rgb["center_m"]
            m.rgb_ok = True
            m.rgb_type = str(rgb["type"]["name"])
            m.rgb_x_mm, m.rgb_y_mm = m_to_mm(cx), m_to_mm(cy)
            # ★ 윗면으로 바꿔 싣는다. 깊이가 재는 것이 윗면이라 그래야 나란히 놓인다.
            m.rgb_top_z_mm = m_to_mm(cz + rgb["type"]["h_mm"] / 2000.0)
            m.rgb_yaw_deg = float(rgb["yaw_deg"])
            m.rgb_fit_err_mm = float(rgb["fit_err_m"] * 1000.0)
        if plane is not None:
            m.depth_spread_mm = float(plane["spread_mm"])
            m.depth_px_used = int(plane["px_used"])
            m.depth_valid_ratio = float(plane["valid_ratio"])
        if dep is not None:
            dx, dy, dz = dep["center_m"]
            m.depth_ok = True
            m.depth_type = str(dep["type"]["name"])
            m.depth_x_mm, m.depth_y_mm = m_to_mm(dx), m_to_mm(dy)
            m.depth_top_z_mm = m_to_mm(dz)
            m.depth_yaw_deg = float(dep["yaw_deg"])
            m.depth_h_mm = float(dep["h_m"] * 1000.0)
        else:
            m.depth_reason = str(reason or "")
        m.agree = bool(res["agree"])
        m.dz_mm, m.dxy_mm = float(res["dz_mm"]), float(res["dxy_mm"])
        m.dyaw_deg = float(res["dyaw_deg"])
        m.verdict, m.used = str(res["verdict"]), str(res["used"])
        self.pub.publish(m)

    def summary(self):
        """지표 한 줄. 종료 로그와 하네스가 같이 쓴다."""
        return (f"교차검증 {self.n_total} 건 · 일치 {self.n_agree} · "
                f"깊이없음 {self.n_nodepth} · 불일치 "
                f"{self.n_total - self.n_agree - self.n_nodepth}")


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

        # ★ 깊이 교차검증. 라인 두 대가 **같은 판정기**를 쓴다 — 카메라마다 따로
        #   두면 임계값이 갈리고, 갈린 줄도 모른다.
        self.xcheck = DepthCrossCheck(self, cfg, "/line/vision_check")
        sfx = self.xcheck.depth_suffix
        self.cam1 = CameraSub(self, self.get_parameter("cam1_ns").value,
                              self._on_cam1, cb, depth_suffix=sfx)
        self.cam2 = CameraSub(self, self.get_parameter("cam2_ns").value,
                              self._on_cam2, cb, depth_suffix=sfx)
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
            return None, None, None
        if cam.K is None:
            self.get_logger().warn(f"{cam.ns}: camera_info 대기 중", once=True)
            return None, None, None
        if det["clipped"]:
            self.get_logger().warn(f"{cam.ns}: 박스가 화면 밖으로 잘렸다 — 건너뛴다")
            return None, None, None
        # ★ 중앙 게이트 (2026-08-27 김병직 실측). 박스가 카메라 축에서 벗어날수록
        #   **옆면이 실루엣에 껴서** 측정이 부푼다 — 이동 중 실측이 343.8 -> 368.6
        #   -> 397.7 로 자라는 스트림이 그 증거이고, 가장자리에서 3호(340)가
        #   400을 넘는 순간 **4호(410)로 오등록**됐다 (BOX018: 실물 3호 -> DB 4호
        #   -> 로봇이 70mm 위에서 흡착 3연속 실패 -> 폐기). 프레임 중앙 1/3 에
        #   있을 때만 판정한다 — 벨트 위라 중앙 프레임은 반드시 온다.
        _c = np.mean(np.asarray(det["corners_px"], float), axis=0)
        _H, _W = bgr.shape[0], bgr.shape[1]
        # ★ 2차: 2/3 -> **1/3 폭** (같은 날). 2/3 안에서도 최대 부풀기 +39mm 라
        #   3호가 375mm 까지 밀리는데 3/4호 분류 경계가 정확히 375 다 — 시드 고정
        #   지터가 그 구간인 BOX015/018 이 재현성 있게 4호로 넘어갔다 (실측 2판).
        #   1/3 폭이면 최대 +19mm, 경계까지 16mm 여유.
        if abs(_c[0] - _W / 2.0) > _W / 6.0 or abs(_c[1] - _H / 2.0) > _H / 6.0:
            return None, None, None         # 중앙 아님 — 다음 프레임에서 판정
        try:
            R, t = camera_pose(self.tf_buf, self.world_frame,
                               cam.frame_id or cam.ns.strip("/"))
        except Exception as e:                  # noqa: BLE001
            self.get_logger().warn(f"{cam.ns}: tf 조회 실패 — {e}", once=True)
            return None, None, None
        out = box_pose_from_corners(det["corners_px"], cam.K, R, t,
                                    self.belt_top_m, self.types)
        if out is None:
            # ★ 여기서 `_report_miss` 를 부르지 않는다 (2026-08-26).
            #   `rescue` 모드면 이 박스를 깊이가 살릴 수 있는데, 여기서 미리
            #   실패로 세면 **같은 박스가 실패로도 세지고 등록으로도 세어**
            #   인식률의 분모가 부푼다 (분모 = 등록 성공 + 여기로 온 실패).
            #   보고는 **살리기를 시도한 뒤** 호출부가 한다.
            # ★ 실측 치수를 같이 남긴다 (2026-08-27). 숫자가 없으면 "인식이 안
            #   된다" 로만 보여서, 병합인지 축소인지 완전 오측인지 못 가른다 —
            #   존 인식은 이미 이렇게 하고 있다(`geometry.LAST_REJECT`).
            _lr = getattr(_geo, "LAST_REJECT", None)
            self.get_logger().warn(
                f"{cam.ns}: 사양 3종과 안 맞는다"
                + (f" [{_lr}]" if _lr else ""))
            # 카메라 자세는 그래도 돌려준다 — RGB 가 포기한 자리야말로 깊이가
            # 답을 갖고 있을 수 있다. 호출부가 깊이 단독으로 한 번 더 본다.
            return det, None, (R, t)
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
        return det, out, (R, t)

    def _on_cam1(self, bgr, header):
        """cam1 — 크기 측정 + QR 운송장번호 판독 → `/db/register_box`.

        ★ **`box_id` 를 세어서 만들지 않는다.** QR 이 정본이다. 순번으로 만들면
          한 개만 놓쳐도 그 뒤 전부가 한 칸씩 밀려 DB 의 다른 박스 기록을 덮어쓴다 —
          에러 없이 전부 틀리는 쪽이다.
        ★ QR 을 못 읽으면 **등록하지 않고 흘려보낸다.** 등록 안 된 박스는 중재자가
          미등록으로 보고 `RETURN` 시킨다. 설계결정서가 요구한 "리젝트 라인이 실제로
          발동" 이 이 경로다 — 인위적 드롭률을 주입할 필요가 없다.
        ★ `dest` 는 비워 보낸다. 구역은 DB 가 **배송지 주소의 동**으로 정한다.
          여기서 추측해 채우면 실제 분류 근거와 어긋나도 아무도 모른다.

        `_armed1` 는 **박스 하나당 한 번만** 등록하기 위한 빗장이다. 30 Hz 로 들어오는
        같은 박스를 매 프레임 등록하면 안 된다.
        """
        det, out, ctx = self._measure(bgr, self.cam1)
        if det is None or (out is None and not self.xcheck.on):
            # ★ arming 을 되돌린다 = **다음 프레임에 다시 본다.** 깊이를 붙이기
            #   전과 같은 뜻이다. 여기서 arming 을 소비해 버리면 RGB 가 한 번
            #   실패한 박스가 영원히 등록되지 않는다 — 박스는 계속 지나가는데
            #   에러는 안 뜬다.
            if det is not None:
                # 박스는 봤는데 사양 3종과 안 맞았다. 깊이가 꺼져 있어 살릴 길도 없다.
                self._report_miss("cam1", "SIZE_MISMATCH")
            self._armed1 = True
            return
        if not self._armed1:
            return                              # 같은 박스를 계속 보고 있다
        self._armed1 = False
        # ★ 교차검증은 arming 뒤에 한다. `_measure` 안에 넣으면 30 Hz 로 발행돼
        #   진단 토픽이 쓰레기가 된다 — 박스 하나에 판정 하나면 충분하다.
        had_rgb = out is not None
        out, _res = self.xcheck.evaluate(self.cam1, det["corners_px"], out,
                                         ctx[0], ctx[1], "cam1",
                                         rgb_stamp=CameraSub._stamp(header))
        if out is None:
            # 채택할 자세가 없다. 다음 프레임에 다시 본다 (위와 같은 이유).
            self._armed1 = True
            if had_rgb:
                # RGB 는 답을 냈는데 깊이와 갈렸다 = reject_on_disagree 가 켜져 있다.
                self.get_logger().warn("cam1: 교차검증이 막았다 — 등록 건너뜀")
                self._report_miss("cam1", "DEPTH_DISAGREE")
            else:
                # RGB 가 포기했고 깊이도 못 살렸다. **이제서야** 실패로 센다.
                self._report_miss("cam1", "SIZE_MISMATCH")
            return
        # ★★ 깊이 프레임 대기 (2026-08-27 정정인 실측). 등록은 박스당 한 번이라
        #   **첫 armed 프레임의 품질이 전부**다. 이 PC 에서 그 프레임의 RGB 가
        #   절반쯤 한 등급 작게 봤고(4호->3호), 깊이가 실린 프레임은 정확했다
        #   (vision_check 실측: depth 4호 · h 279 · spread 0.1). 틀린 크기가
        #   DB 에 들어가면 로봇이 낮은 z 로 내려가다 박스 윗면에 얹혀
        #   IK_FAIL 3연발 -> 박스 포기가 난다.
        #   prefer 인데 이 프레임에 깊이가 안 실렸으면(_res.used != DEPTH)
        #   arming 을 유지한 채 최대 6프레임 더 본다 — 깊이 실린 프레임에서
        #   등록하기 위해서다. 그래도 안 오면 RGB 로라도 등록한다
        #   (안 하면 박스가 통째로 RETURN 된다).
        if (self.xcheck.on and str(self.xcheck.cfg.get("mode")) == "prefer"
                and _res.get("used") != "DEPTH"):
            self._wait1 = getattr(self, "_wait1", 0) + 1
            if self._wait1 <= 6:
                self._armed1 = True
                return
            self.get_logger().warn(
                "cam1: 깊이 실린 프레임을 6번 기다렸지만 안 왔다 — RGB 로 등록")
        self._wait1 = 0
        if not had_rgb:
            # ★ 깊이가 살린 박스다. RGB 단독이었으면 여기까지 못 왔다.
            self.get_logger().info(
                f"cam1: RGB 는 포기했는데 깊이가 살렸다 — {out['type']['name']} "
                f"(윗면 실측 기준)")
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
        # ★ 실측치를 등록 로그까지 들고 간다 (2026-08-27). DB 에 들어가는 건 사양
        #   치수로 **스냅된** 값이라, 종류를 잘못 고르면 로그만 봐서는 알 수 없다.
        #   10.10.0.2 에서 4호가 3호로 등록돼 로봇이 70 mm 낮게 내려간 사고가
        #   있었는데, 그때 "몇 mm 로 쟀길래" 를 아무도 몰라 원인을 못 좁혔다.
        _m = out.get("measured_m") if isinstance(out, dict) else None
        _ms = ("%.0fx%.0f" % (_m[0] * 1000, _m[1] * 1000)) if _m else None
        self._call(self.cli_reg, req,
                   lambda r, b=box_id, t=bt, s=_ms: self._reg_done(r, b, t, s))

    def _report_miss(self, where_at, reason):
        """박스를 봤는데 식별 못 했다 — DB 에 알린다 (인식률의 분모).

        ★ 응답을 기다리지 않는다. 이건 통계용이라 라인을 세울 이유가 없다.
        ★ 쿨다운 2초 (2026-08-27 김병직). 프레임마다 부르니 같은 박스 하나가
          지나가는 동안 수십 번 쌓였다 — 실측 10분에 1200건, 인식률 분모가
          프레임 수로 왜곡(11%)되고 DB 만 두들긴다 (db_service 사망 사고의
          방아쇠였다). 같은 (지점, 사유)는 2초에 한 번만 보낸다 — 벨트 0.55 m/s
          에서 박스 하나가 시야를 지나는 시간보다 길어 사실상 박스당 1건이 된다.
        """
        import time as _t
        k = (where_at, reason)
        now = _t.monotonic()
        last = getattr(self, "_miss_last", {}).get(k, 0.0)
        if now - last < 2.0:
            return
        if not hasattr(self, "_miss_last"):
            self._miss_last = {}
        self._miss_last[k] = now
        if not self.cli_miss.service_is_ready():
            self.get_logger().warn("/db/report_miss 가 아직 없다 — 통계에서 빠진다",
                                   once=True)
            return
        self.cli_miss.call_async(ReportMiss.Request(where_at=where_at, reason=reason))

    def _reg_done(self, res, box_id, bt, measured=None):
        """`RegisterBox` 응답. 실패해도 **라인을 세우지 않는다** — 로그만 남긴다.

        `measured` 는 cam1 이 실제로 잰 치수다 (2026-08-27). 로그에 같이 실어
        사양 기각이 났을 때 "무엇을 얼마로 쟀길래" 를 바로 볼 수 있게 한다.
        """
        if res is None or not res.ok:
            self.get_logger().error(
                f"register_box 실패 {box_id}: {getattr(res, 'message', 'no response')}")
            return
        self._n_reg += 1
        self.get_logger().info(
            f"cam1 등록 {box_id} ({bt['name']}"
            + (f" · 실측 {measured} mm" if measured else "")
            + f") → {res.message} · 누계 {self._n_reg}")

    def _on_cam2(self, bgr, header):
        """cam2 — 재판독해 `/db/verify_box` 로 교차 검증한다.

        ★ QR 을 못 읽으면 **`VerifyBox` 를 아예 안 부른다.** `box_id` 를 추측해서
          부르면 엉뚱한 박스를 통과시킨다. 안 부르는 쪽이 안전하다.
        """
        det, out, ctx = self._measure(bgr, self.cam2)
        if det is None or (out is None and not self.xcheck.on):
            if det is not None:
                self._report_miss("cam2", "SIZE_MISMATCH")
            self._armed2 = True
            return
        if not self._armed2:
            return
        self._armed2 = False
        # cam2 는 검증 카메라라 좌표를 안 쓴다. 그래도 교차검증은 돌린다 —
        # 여기서 어긋나면 cam1 이 등록한 치수가 틀렸다는 두 번째 증거가 된다.
        fixed, _res = self.xcheck.evaluate(self.cam2, det["corners_px"], out,
                                           ctx[0], ctx[1], "cam2",
                                           rgb_stamp=CameraSub._stamp(header))
        if out is None and fixed is None:
            # RGB 가 포기했고 깊이도 못 살렸다 (cam1 과 같은 회계 규칙).
            self._report_miss("cam2", "SIZE_MISMATCH")

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
        """`VerifyBox` 응답. 불일치는 **기록만** 하고 판단은 미룬다.

        여기서 버리면 멀쩡한 박스를 죽일 수 있다 — cam2 쪽이 잘못 읽었을 수도 있다.
        불일치가 늘어나는지는 `scan_misses` 지표로 보인다.
        """
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
        """서비스 호출 한 번. **`call_async` + `done_callback` 만 쓴다.**

        콜백 안에서 `spin_until_future_complete` 를 부르면 executor 가 자기 자신을
        기다려 **에러 한 줄 없이** 멈춘다 (CLAUDE.md §9).
        """
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
        # ★ 스토퍼 창·자리잡음 필터 (2026-08-27 김병직 — gtpick 대역과 같은 규칙).
        #   실측: 도착 전 박스를 보고했다 — y 3405(스토퍼 -586mm)·yaw 179→88.7 로
        #   널뛰는 좌표에 로봇이 출발해 흡착·IK 실패. ① 가장 큰 박스가 스토퍼에
        #   닿았을 때 중심보다 60mm 여유 밖이면 아직 도착 전, ② 직전 추론과
        #   5mm·1.5도 안으로 같아야 자리잡은 것. ③ 같은 박스가 15초째 걸려
        #   있으면(뒤집힘 등) 강제 보고 — 관제탑 재시도 상한이 치워 준다.
        _ax = str(cfg["conveyor"].get("pickup_axis", "x")).lower()
        self._axis_i = 1 if _ax == "y" else 0
        _half = max(max(t["w_mm"], t["d_mm"]) for t in self.types) / 2000.0
        self._win_m = cfg["conveyor"]["stopper_x_mm"] / 1000.0 - _half - 0.060
        self._stab_prev = None       # (x_mm, y_mm, yaw)
        self._stab_first = None      # 처음 필터에 걸린 시각 (monotonic)
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
        # ★ 여기가 깊이가 실제로 값을 하는 자리다. 이 노드의 좌표는 관제탑을 거쳐
        #   **로봇이 집으러 가는 자리**가 된다. 종류를 틀려 평면이 60~130 mm
        #   어긋나면 x, y 가 같이 틀어지고, 그건 흡착 실패로 나타난다.
        self.xcheck = DepthCrossCheck(self, cfg, f"/zone_{self.zone}/vision_check",
                                      zone=self.zone)
        self.cam = CameraSub(self, ns, self._on_image, cb,
                             depth_suffix=self.xcheck.depth_suffix)
        self.get_logger().info(
            f"detector_zone_{self.zone} 시작 — {ns} -> /zone_{self.zone}/box_at_pickup")

    def _on_status(self, msg):
        """관제탑 `ZoneStatus` 로 **추론을 켜고 끈다** — 평가 항목 "처리 효율" 이 이것이다.

        IDLE 일 때만 추론한다. 집는 중에는 새 좌표가 필요 없고, 오히려 팔이 화면에
        들어와 오검출을 만든다.
        """
        # IDLE 일 때만 추론한다. 집는 중에는 새 좌표가 필요 없고, 오히려 팔이
        # 화면에 들어와 오검출을 만든다.
        self._idle = (msg.state == "IDLE")

    def _now(self):
        """★ 벽시계가 아니라 **ROS 시계**다. `use_sim_time` 이면 Isaac 의 sim time 을 따른다.

        헤드리스는 시뮬이 실시간의 몇 배로 도는데, 추론 간격(`_min_dt`)을 벽시계로
        재면 시뮬 시간으로는 훨씬 촘촘해진다.
        """
        return self.get_clock().now().nanoseconds / 1e9

    def _pickup_roi(self, header, shape):
        """픽업 창의 픽셀 ROI. 한 번 계산해 캐시한다 (카메라·스토퍼 둘 다 정적).

        ★ 왜 (2026-08-27 김병직·라이브 캡처 실측): 픽업 카메라 시야에 **팔레트**가
          들어온다. 방금 쌓은 박스가 같은 분홍이라 검출이 그쪽(더 높아 더 크게
          보임 — 팔레트 950+340=상면 1290, x1.15 확대)을 잡았다 — C존 기각
          541x445 = 정확히 팔레트 위 5호였다. 스포너의 pickup_range 상한과
          같은 원리로, 검출 **전에** 픽업 창 밖을 지운다.
        """
        if getattr(self, "_roi", None) is not None:
            return self._roi
        if self.cam.K is None:
            return None
        try:
            R, t = camera_pose(self.tf_buf, self.world_frame,
                               header.frame_id or self.cam.frame_id)
        except Exception:                                       # noqa: BLE001
            return None                     # tf 가 아직 — 다음 프레임에 다시
        cw = (self.cfg["zones"].get("pickup_cross_mm") or {}).get(self.zone.upper())
        if not cw:
            return None
        stop = self.cfg["conveyor"]["stopper_x_mm"] / 1000.0
        along = (self._win_m - 0.35, stop + 0.05)   # 도착 직전 여유 ~ 스토퍼 앞면
        cross = (cw[0] / 1000.0, cw[1] / 1000.0)
        maxh = max(bt["h_mm"] for bt in self.types) / 1000.0
        us, vs = [], []
        for a in along:
            for c in cross:
                for Z in (self.belt_top_m, self.belt_top_m + maxh):
                    w = (np.array([c, a, Z]) if self._axis_i == 1
                         else np.array([a, c, Z]))
                    pc = R.T @ (w - t)
                    if pc[2] <= 1e-6:
                        continue
                    uv = self.cam.K @ (pc / pc[2])
                    us.append(uv[0]); vs.append(uv[1])
        if len(us) < 4:
            return None
        H, W = shape[0], shape[1]
        M = 25                              # 픽셀 여유
        self._roi = (max(0, int(min(us)) - M), max(0, int(min(vs)) - M),
                     min(W, int(max(us)) + M), min(H, int(max(vs)) + M))
        self.get_logger().info("픽업 ROI 확정 %s (팔레트 등 시야 밖 배제)"
                               % (self._roi,))
        return self._roi

    def _on_image(self, bgr, header):
        """pick_cam — 스토퍼에 선 박스의 **픽업 좌표를 실측**해 `BoxAtPickup` 으로 낸다.

        이 노드의 전부다. 순서:

            ① 게이트   IDLE 이 아니거나 최소 간격 전이면 **추론 자체를 안 한다**
            ② find_box 빨간 윤곽 → minAreaRect. 시야에 **잘렸으면 좌표를 안 낸다**
            ③ tf       이미지 헤더의 frame_id 로 카메라 자세를 조회
            ④ 역투영    코너 4점 + K + 벨트 상면 높이(800) → 월드 중심 · yaw
            ⑤ QR       운송장번호. **한 번 읽으면 박스가 떠날 때까지 기억한다**
            ⑥ 발행      mm 정수로 변환해 publish. z 는 안 싣는다 (벨트 높이는 고정)

        ★ 좌표를 못 믿을 때는 **"안 보인다"(present=false) 를 낸다.** 잘린 사각형의
          중심은 진짜 중심이 아니다 — 틀린 좌표를 주면 로봇이 빗나간 자리를 집는다.
        ★ QR 해독 실패는 무작위가 아니라 **자세에 따라 결정적**이다. 나쁜 자세로 멈춘
          박스는 몇 번을 다시 봐도 계속 실패한다(실측: 한 자세로 굳으면 0% 아니면 100%).
          그래서 재시도가 아니라 **기억**으로 덮는다.
        """
        t = self._now()
        if not self._idle or (t - self._last) < self._min_dt:
            return
        self._last = t

        # ── 픽업 창 밖(팔레트 쪽)을 지우고 검출한다 (_pickup_roi 주석) ──
        roi = self._pickup_roi(header, bgr.shape)
        if roi is not None:
            x0, y0, x1, y1 = roi
            m = np.zeros_like(bgr)
            m[y0:y1, x0:x1] = bgr[y0:y1, x0:x1]
            bgr = m

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
        if out is None and not self.xcheck.on:
            self.get_logger().warn(
                "사양 3종 어느 것과도 안 맞는다 — 좌표를 내지 않는다 [실측 %s]"
                % (_geo.LAST_REJECT or "?"))
            self._publish_absent()
            return

        # ★ 원시 측정값 (스냅 전). LineDetector._measure 의 같은 로그와 짝이다.
        #   여기 값이 로봇이 실제로 집으러 갈 때 쓰는 판정이다.
        if out is not None:
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

        # ★ 발행 **직전**에 교차검증한다. box_id 를 읽은 뒤라 진단 토픽에 어느
        #   박스인지가 실리고, 앞이라 mode:prefer 면 깊이가 잰 평면의 좌표가
        #   그대로 로봇에게 간다. 순서를 바꾸면 둘 중 하나를 잃는다.
        out, res = self.xcheck.evaluate(self.cam, det["corners_px"], out,
                                        R, t_vec, "pick_cam", box_id,
                                        rgb_stamp=CameraSub._stamp(header))
        if out is None:
            self._hold_coordinates(res["verdict"])
            return

        # ★ 계약은 mm 정수다 (CLAUDE.md §3-2). 내부 계산은 m 로 하지만
        #   토픽에 실을 때 여기서 한 번만 mm 로 바꾼다.
        #   z 는 싣지 않는다 — 벨트 상면(800mm)은 고정이라 관제탑이 안다.
        cx, cy, cz = out["center_m"]

        # ── 스토퍼 창·자리잡음 필터 (__init__ 주석 참조) ──
        _now = time.monotonic()
        _cur = (m_to_mm(cx), m_to_mm(cy), round(float(out["yaw_deg"]), 1))
        if self._stab_first is None:
            self._stab_first = _now
        _force = (_now - self._stab_first) > 15.0
        _prev, self._stab_prev = self._stab_prev, _cur
        if not _force:
            if out["center_m"][self._axis_i] < self._win_m:
                return                   # 아직 스토퍼에 도착 전 — 보고하지 않는다
            if (_prev is None or abs(_prev[0] - _cur[0]) > 5
                    or abs(_prev[1] - _cur[1]) > 5
                    or abs(_prev[2] - _cur[2]) > 1.5):
                return                   # 아직 자리잡는 중 — 다음 추론에서 다시
        elif not self._present:
            self.get_logger().warn(
                "15초째 창밖/불안정 — 강제 보고 (집기 실패하면 관제탑 상한이 치운다)")

        self.pub.publish(BoxAtPickup(
            present=True, box_id=box_id,
            x_mm=m_to_mm(cx), y_mm=m_to_mm(cy),
            yaw_deg=float(out["yaw_deg"])))
        if box_id and box_id != self._seen:
            self.get_logger().info(
                f"도착 {box_id or '(미상)'} {out['type']['name']} "
                f"({m_to_mm(cx)}, {m_to_mm(cy)}, {m_to_mm(cz)}) mm "
                f"yaw {out['yaw_deg']:.1f}° · 맞춤오차 {out['fit_err_m']*1000:.1f} mm")
            self._check_against_db(box_id, out["type"])
        self._present = True
        if box_id:
            self._seen = box_id

    def _check_against_db(self, box_id, btype):
        """투입부가 등록한 치수와 픽업에서 잰 치수가 같은가 — 비동기로 확인만.

        ★ 이것도 교차검증이지만 **축이 다르다.** 깊이(`self.xcheck`)는 같은 순간
          같은 카메라를 두 방식으로 보고, 이쪽은 **다른 시각 다른 카메라**(cam1)의
          기록과 대조한다. 둘을 한 이름으로 부르면 어느 쪽이 울렸는지 못 가린다.

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

    def _hold_coordinates(self, verdict):
        """믿을 수 없는 좌표를 "없다" 로 낸다. 침묵하면 관제탑이 영원히 기다린다.

        ★ present=False 는 거짓말이 아니다 — 계약상 "무시하고 사이클을 시작하지
          말 것" 이라는 뜻이고(§3-2), 믿을 수 없는 좌표에 대해 정확히 그 뜻이다.
          로봇이 빗나간 자리를 집는 쪽이 훨씬 나쁘다 (`clipped` 처리와 같은 판단).
        """
        self.get_logger().warn(f"픽업 좌표 보류 [{verdict}] — 좌표를 내지 않는다",
                               throttle_duration_sec=2.0)
        self._publish_absent()

    def _publish_absent(self):
        """"픽업 자리가 비었다" 를 알린다.

        ★ 이 신호가 관제탑의 **빗장을 푼다.** 방금 적재한 박스·방금 버린 박스가
          `present=false` 를 한 번 봐야 다음 박스를 받는다 — 안 그러면 큐에 남아 있던
          이전 프레임 때문에 **같은 박스를 두 번 쌓는다** (실측으로 확인된 버그다).
        """
        if self._present:
            self.get_logger().info("픽업 자리 비었다")
        self._present, self._seen = False, None
        # ★ 필터 상태도 함께 리셋 — 다음 박스를 새 박스로 본다 (창·안정 필터)
        self._stab_prev = self._stab_first = None
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
        # ★ `rclpy.ok()` 로 감싼다. SIGTERM 으로 내리면 컨텍스트가 이미 닫혀 있어
        #   `rcl_shutdown already called` 트레이스백이 남는다 — 노드가 정상 종료해도
        #   로그 끝에 예외가 찍혀 **뭔가 죽은 것처럼 보인다.** (검증 중 실측)
        if rclpy.ok():
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
