#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_node — 카메라 토픽에서 박스의 '색'과 '크기'를 재서 토픽으로 발행한다.

구독:
    /<ns>/rgb           sensor_msgs/Image       rgb8
    /<ns>/depth         sensor_msgs/Image       32FC1, 단위 m
    /<ns>/camera_info   sensor_msgs/CameraInfo  fx, fy, cx, cy

발행:
    /vision/detections  std_msgs/String (JSON)

동작 순서 (한 프레임마다):
    ① RGB 를 HSV 로 바꿔 빨강/초록/파랑 마스크를 만든다  → 색 감지
    ② 마스크 안의 depth 중앙값을 구한다                  → 그 물체까지 거리
    ③ 픽셀 크기 × 거리 ÷ 초점거리 = 실제 크기            → 크기 측정
    ④ JSON 으로 묶어 발행한다

★ 왜 배경 차분이 아니라 색 기준인가:
  배경 차분은 카메라가 바닥을 수직으로 내려다볼 때만 성립한다. camera.usd 에 저장된
  rsd455 는 Y축으로 52.5° 기울어 있어서 '높이 = 바닥까지 거리 차이' 가 성립하지 않는다.
  색은 카메라 각도와 무관하게 잡히므로 확인 단계에서는 색이 맞다.

★ 여기서 재는 크기는 '이미지 평면에서 보이는 크기' 다. 카메라가 물체를 비스듬히 보면
  윗면뿐 아니라 옆면 일부까지 실루엣에 들어와 실제 치수보다 크게 나온다.
  광축에서 멀리 있는 물체일수록 그렇다. 3호/4호/5호 스냅 판정을 기본으로 끄는 이유다.
  (--ros-args -p snap_enabled:=true 로 켤 수 있고, 이때 치수는 system.yaml 에서 읽는다)
"""

import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# 단위 변환은 반드시 packing_common 함수만 쓴다 — 직접 나눗셈 금지 (작업명세서 §1.3)
from packing_common.units import m_to_mm


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # ── 파라미터 ─────────────────────────────────────────────────────────
        # 물리 수치(박스 치수 등)는 system.yaml 이 정본이고, 아래는 노드 동작 설정이다.
        self.declare_parameter('camera_ns', 'cam_scan')
        self.declare_parameter('min_area_px', 300)     # 이보다 작은 얼룩은 노이즈로 버린다
        self.declare_parameter('sat_min', 70)          # 채도 하한 — 회색 배경을 걸러낸다
        self.declare_parameter('val_min', 40)          # 명도 하한 — 초록 박스가 어두워서 낮게 잡았다
        self.declare_parameter('log_period_s', 1.0)
        self.declare_parameter('snap_enabled', False)
        self.declare_parameter('config_path',
                               '~/palletris_ws/src/packing_bringup/config/system.yaml')

        ns = self.get_parameter('camera_ns').value
        self.min_area = int(self.get_parameter('min_area_px').value)
        self.s_min = int(self.get_parameter('sat_min').value)
        self.v_min = int(self.get_parameter('val_min').value)
        self.log_period = float(self.get_parameter('log_period_s').value)
        self.snap_enabled = bool(self.get_parameter('snap_enabled').value)

        # ── 상태 ────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.info_logged = False
        self.depth = None          # 최신 depth 프레임 (numpy, m)
        self.last_log = 0.0
        self.frames = 0

        self.box_types = self._load_box_types() if self.snap_enabled else []

        # ── 통신 ────────────────────────────────────────────────────────────
        # 센서 QoS(best effort)로 구독한다. Isaac 이 reliable 로 쏘든 best effort 로 쏘든
        # 이쪽이 best effort 면 양쪽 다 붙는다. 반대로 잡으면 조용히 아무것도 안 온다.
        self.create_subscription(CameraInfo, f'/{ns}/camera_info',
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, f'/{ns}/depth',
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, f'/{ns}/rgb',
                                 self.on_rgb, qos_profile_sensor_data)

        self.pub = self.create_publisher(String, '/vision/detections', 10)

        self.ns = ns
        self.get_logger().info(
            f"vision_node 시작. 색 기준 감지 모드. 구독 /{ns}/rgb · /{ns}/depth · /{ns}/camera_info")
        if not self.snap_enabled:
            self.get_logger().info(
                "snap_enabled=false — 3호/4호/5호 판정은 하지 않는다. "
                "켜려면 --ros-args -p snap_enabled:=true")

    # ── config ──────────────────────────────────────────────────────────────
    def _load_box_types(self):
        """system.yaml 의 박스 3종 치수를 읽는다. 숫자를 코드에 박지 않는다 (철칙 3)."""
        try:
            from packing_common import config
            cfg = config.load(self.get_parameter('config_path').value)
            return cfg['boxes']['types']
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().warn(f"system.yaml 을 못 읽어 스냅을 끈다: {e}")
            self.snap_enabled = False
            return []

    # ── 콜백 ────────────────────────────────────────────────────────────────
    def on_info(self, msg: CameraInfo):
        # K = [fx 0 cx / 0 fy cy / 0 0 1]
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        if not self.info_logged:
            self.info_logged = True
            self.get_logger().info(
                f"{self.ns} 내부 파라미터 수신: fx={self.fx:.1f}px fy={self.fy:.1f}px "
                f"해상도 {msg.width}x{msg.height}")

    def on_depth(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def on_rgb(self, msg: Image):
        if self.fx is None:
            return                      # camera_info 가 아직 안 왔으면 환산을 못 한다
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        dets = self.detect(bgr)
        self.frames += 1

        out = {
            'camera': self.ns,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'measure': 'image-plane apparent size',
            'detections': dets,
        }
        m = String()
        m.data = json.dumps(out, ensure_ascii=False)
        self.pub.publish(m)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log >= self.log_period:
            self.last_log = now
            if not dets:
                self.get_logger().info(f"[{self.ns}] 검출 없음 (프레임 {self.frames})")
            for d in dets:
                extra = f" → {d['snap']}" if d.get('snap') else ""
                self.get_logger().info(
                    f"[{self.ns}] color={d['color']:<5} "
                    f"크기 {d['size_mm'][0]}x{d['size_mm'][1]}mm  "
                    f"거리 {d['depth_m']:.3f}m  면적 {d['area_px']}px{extra}")

    # ── 핵심 ────────────────────────────────────────────────────────────────
    def color_masks(self, hsv):
        """색마다 HSV 범위로 마스크를 만든다. OpenCV 의 H 는 0~179 다 (0~359 아님)."""
        s, v = self.s_min, self.v_min
        table = {
            # 빨강은 H=0 을 사이에 두고 갈라져 있어 두 구간을 합쳐야 한다
            'red': [((0, s, v), (10, 255, 255)), ((170, s, v), (179, 255, 255))],
            'green': [((35, s, v), (85, 255, 255))],
            'blue': [((95, s, v), (130, 255, 255))],
        }
        out = {}
        for name, ranges in table.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            # 열기 → 닫기: 점 노이즈를 지우고 구멍을 메운다
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            out[name] = mask
        return out

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        dets = []

        for color, mask in self.color_masks(hsv).items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            c = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(c))
            if area < self.min_area:
                continue

            # 기울어진 면도 제대로 재려면 축정렬 사각형보다 최소회전사각형이 낫다
            (u, v), (w_px, h_px), angle = cv2.minAreaRect(c)

            blob = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(blob, [c], -1, 255, -1)
            depth_m = self.median_depth(blob)
            if depth_m is None:
                continue

            long_px, short_px = max(w_px, h_px), min(w_px, h_px)
            # 핀홀 모델: 실제 크기 = 픽셀 크기 × 거리 ÷ 초점거리
            long_mm = m_to_mm(long_px * depth_m / self.fx)
            short_mm = m_to_mm(short_px * depth_m / self.fy)

            det = {
                'color': color,
                'size_mm': [long_mm, short_mm],
                'depth_m': round(depth_m, 4),
                'area_px': int(area),
                'uv': [round(u, 1), round(v, 1)],
                'angle_deg': round(float(angle), 1),
                # 카메라 광학 좌표계 기준 위치 (X 오른쪽, Y 아래, Z 앞)
                'pos_m': [round((u - self.cx) * depth_m / self.fx, 4),
                          round((v - self.cy) * depth_m / self.fy, 4),
                          round(depth_m, 4)],
            }
            if self.snap_enabled:
                det['snap'] = self.snap(long_mm, short_mm)
            dets.append(det)

        return dets

    def median_depth(self, blob):
        """마스크 안의 유효한 depth 중앙값(m). 평균이 아니라 중앙값인 이유는
        가장자리 픽셀이 배경 거리로 튀어서 평균을 통째로 끌어당기기 때문."""
        if self.depth is None:
            return None
        d = self.depth
        if d.shape[:2] != blob.shape[:2]:
            blob = cv2.resize(blob, (d.shape[1], d.shape[0]), interpolation=cv2.INTER_NEAREST)
        vals = d[blob > 0]
        vals = vals[np.isfinite(vals) & (vals > 0.0)]
        if vals.size < 10:
            return None
        return float(np.median(vals))

    def snap(self, long_mm, short_mm):
        """잰 치수를 3호/4호/5호 중 가장 가까운 것에 붙인다."""
        best, best_err = None, None
        for t in self.box_types:
            dims = sorted([t['w_mm'], t['d_mm'], t['h_mm']], reverse=True)
            err = min(math.hypot(long_mm - dims[i], short_mm - dims[j])
                      for i in range(3) for j in range(3) if i != j)
            if best_err is None or err < best_err:
                best, best_err = t['name'], err
        return f"{best} (오차 {int(best_err)}mm)"


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Ctrl+C / SIGTERM 으로 끌 때 rclpy 가 이걸 던진다.
        # 안 잡으면 정상 종료인데도 traceback 이 쏟아져 진짜 에러처럼 보인다.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
