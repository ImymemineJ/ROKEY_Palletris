#!/usr/bin/env python3
"""인식 노드 전 구간 검증 — Isaac 없이 돈다.

    ros2 run packing_db gen_master_db
    ros2 run packing_db db_service_node &
    ros2 run packing_vision vision_node &
    python3 scripts/verify_vision.py

씬 USD 가 이 저장소에 없어서(sync_scene.sh 배포) 진짜 카메라 영상을 못 쓴다.
그래서 이 스크립트가 **Isaac 의 대역**을 선다 — 알려진 카메라·알려진 박스를
직접 투영해 이미지를 만들고, `<ns>/rgb` · `<ns>/camera_info` · `/tf_static` 을
씬과 같은 규약으로 낸다 (손목 카메라 배선과 같은 이름 규칙).

확인하는 것
  cam1 그림  → RegisterBox → palletris.db 에 행이 생기고 종류가 맞는가
  pick_cam   → BoxAtPickup 에 실린 좌표가 진짜 박스 자리와 몇 mm 차이인가
  빈 벨트    → present=False 로 떨어지는가
"""
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                "..", "src", "packing_common"))
from packing_common.config import load                                # noqa: E402
from packing_msgs.msg import BoxAtPickup, ZoneStatus                  # noqa: E402

CFG = load()
W, H = CFG["recognition"]["resolution"]
HFOV = CFG["recognition"]["hfov_deg"]
BELT = CFG["conveyor"]["belt_top_z_mm"] / 1000.0
TYPES = CFG["boxes"]["types"]

# 카메라 자세 (테스트 전용). x 는 yaml 에 없어 진행보고의 씬 프림 값을 쓴다 —
# 노드는 이 값을 tf 로만 받으므로 코드에는 안 박혀 있다.
CAMS = {"cam1":     np.array([CFG["recognition"]["cam1_x_mm"] / 1000.0, 0.0, 2.300]),
        "pick_cam": np.array([9.795, 0.0, 2.300])}
# 하향 광학 프레임: x 오른쪽 · y 아래 · z 앞(= 월드 -Z)
R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])


def K_mat():
    fx = (W / 2.0) / math.tan(math.radians(HFOV) / 2.0)
    return np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])


def render(box_type, center_xy, yaw_deg, t_cam):
    """박스 윗면을 빨간 사각형으로. 없으면 빈 벨트."""
    img = np.full((H, W, 3), (90, 90, 90), np.uint8)
    if box_type is None:
        return img
    w, d = box_type["w_mm"] / 1000.0, box_type["d_mm"] / 1000.0
    z = BELT + box_type["h_mm"] / 1000.0
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    loc = np.array([[w / 2, d / 2], [-w / 2, d / 2], [-w / 2, -d / 2], [w / 2, -d / 2]])
    corners = (loc @ np.array([[c, -s], [s, c]]).T) + np.asarray(center_xy, float)
    P = np.hstack([corners, np.full((4, 1), z)])
    p = (P - t_cam) @ R_DOWN
    uv = (K_mat() @ p.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    cv2.fillPoly(img, [np.round(uv).astype(np.int32)], (40, 40, 235))
    return img


class FakeScene(Node):
    def __init__(self):
        super().__init__("fake_scene")
        self.pub_img, self.pub_info = {}, {}
        for name in CAMS:
            self.pub_img[name] = self.create_publisher(
                Image, f"/{name}/rgb", qos_profile_sensor_data)
            self.pub_info[name] = self.create_publisher(
                CameraInfo, f"/{name}/camera_info", qos_profile_sensor_data)
        self.pub_status = self.create_publisher(ZoneStatus, "/zone_b/status", 10)
        self.seen = []
        self.create_subscription(BoxAtPickup, "/zone_b/box_at_pickup",
                                 lambda m: self.seen.append(m), 10)

        # tf_static — 노드는 여기서만 카메라 자세를 얻는다
        self.tfb = StaticTransformBroadcaster(self)
        tfs = []
        for name, t in CAMS.items():
            m = TransformStamped()
            m.header.frame_id, m.child_frame_id = "World", name
            m.header.stamp = self.get_clock().now().to_msg()
            m.transform.translation.x = float(t[0])
            m.transform.translation.y = float(t[1])
            m.transform.translation.z = float(t[2])
            # R_DOWN = x축 180° 회전 → q = (1,0,0,0)
            m.transform.rotation.x, m.transform.rotation.w = 1.0, 0.0
            tfs.append(m)
        self.tfb.sendTransform(tfs)

    def send(self, name, box_type, center_xy, yaw):
        img = render(box_type, center_xy, yaw, CAMS[name])
        msg = Image()
        msg.header.frame_id, msg.height, msg.width = name, H, W
        msg.encoding, msg.step = "bgr8", W * 3
        msg.data = img.tobytes()
        info = CameraInfo()
        info.header.frame_id, info.height, info.width = name, H, W
        K = K_mat()
        info.k = K.reshape(-1).tolist()
        info.distortion_model = "plumb_bob"
        self.pub_info[name].publish(info)
        self.pub_img[name].publish(msg)

    def idle(self):
        self.pub_status.publish(ZoneStatus(zone="b", state="IDLE", accepting=True))


def pump(node, seconds, fn=None):
    end = time.time() + seconds
    while time.time() < end:
        if fn:
            fn()
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    ok_all = True

    def chk(label, cond, detail=""):
        nonlocal ok_all
        ok_all = ok_all and cond
        print(f"  {'✓' if cond else '✗ 실패'} {label}{(' — ' + detail) if detail else ''}")

    rclpy.init()
    n = FakeScene()
    BT, CENTER, YAW = TYPES[1], (9.78, 0.05), 25.0      # 4호

    print("① 빈 벨트를 먼저 보여 준다 (검출기 arming)")
    pump(n, 2.0, lambda: (n.send("cam1", None, (0, 0), 0),
                          n.send("pick_cam", None, (0, 0), 0), n.idle()))
    chk("빈 벨트에 present=False", all(not m.present for m in n.seen) if n.seen else True,
        f"{len(n.seen)} 건 수신")

    print("② cam1 에 4호 박스 — 등록되는가")
    x1 = CFG["recognition"]["cam1_x_mm"] / 1000.0
    pump(n, 3.0, lambda: (n.send("cam1", BT, (x1, 0.02), 10.0), n.idle()))
    import sqlite3
    db = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "palletris.db")
    row = sqlite3.connect(f"file:{db}?mode=ro", uri=True).execute(
        "SELECT box_id, size_w_mm, size_d_mm, size_h_mm, status FROM boxes_sorted "
        "ORDER BY db_seq DESC LIMIT 1").fetchone()
    chk("DB 에 행이 생겼다", row is not None, str(row))
    if row:
        chk("종류를 맞혔다 (4호)",
            (row[1], row[2], row[3]) == (BT["w_mm"], BT["d_mm"], BT["h_mm"]),
            f"{row[1]}x{row[2]}x{row[3]}")

    print("③ pick_cam 에 같은 박스 — 좌표가 나오는가")
    n.seen.clear()
    pump(n, 3.0, lambda: (n.send("cam1", None, (0, 0), 0),
                          n.send("pick_cam", BT, CENTER, YAW), n.idle()))
    hits = [m for m in n.seen if m.present]
    chk("BoxAtPickup(present=True) 수신", bool(hits), f"{len(hits)} 건")
    if hits:
        # ★ BoxAtPickup 은 mm 정수다 (CLAUDE.md §3-2). 쿼터니언도 z 도 없다.
        #   z 를 안 싣는 건 벨트 상면(800mm)이 고정이라 관제탑이 이미 알기 때문.
        m = hits[-1]
        yaw = m.yaw_deg % 180.0
        dx, dy = m.x_mm - CENTER[0] * 1000, m.y_mm - CENTER[1] * 1000
        dyaw = min(abs(yaw - YAW % 180), 180 - abs(yaw - YAW % 180))
        chk("x 오차 < 10 mm", abs(dx) < 10, f"{dx:+.2f} mm")
        chk("y 오차 < 10 mm", abs(dy) < 10, f"{dy:+.2f} mm")
        chk("yaw 오차 < 3°", dyaw < 3, f"{dyaw:.2f}°")
        chk("box_id 가 실렸다", bool(hits[-1].box_id), hits[-1].box_id)

    print("④ 박스를 치운다 — present=False 로 떨어지는가")
    n.seen.clear()
    pump(n, 2.0, lambda: (n.send("pick_cam", None, (0, 0), 0), n.idle()))
    chk("present=False", any(not m.present for m in n.seen), f"{len(n.seen)} 건")

    n.destroy_node()
    rclpy.shutdown()
    print("\n" + ("전부 통과" if ok_all else "★ 실패 있음"))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
