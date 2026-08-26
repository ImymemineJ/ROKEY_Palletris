#!/usr/bin/env python3
"""인식 노드 전 구간 검증 — Isaac 없이 돈다.

    ros2 run packing_db gen_master_db
    ros2 run packing_db db_service_node &
    ros2 run packing_vision vision_node &
    ros2 run packing_vision zone_detector --ros-args -p zone:=b &
    python3 scripts/verify_vision.py

씬 USD 가 이 저장소에 없어서(sync_scene.sh 배포) 진짜 카메라 영상을 못 쓴다.
그래서 이 스크립트가 **Isaac 의 대역**을 선다 — 알려진 카메라·알려진 박스를
직접 투영해 이미지를 만들고, `<ns>/rgb` · `<ns>/depth` · `<ns>/camera_info` ·
`/tf_static` 을 씬과 같은 규약으로 낸다 (손목 카메라 배선 + setup_camera_graph.py 의
깊이 헬퍼와 같은 이름 규칙 — 깊이는 32FC1 · m).

확인하는 것
  cam1 그림  → RegisterBox → palletris.db 에 행이 생기고 종류가 맞는가
  pick_cam   → BoxAtPickup 에 실린 좌표가 진짜 박스 자리와 몇 mm 차이인가
  빈 벨트    → present=False 로 떨어지는가
  ★ 깊이 교차검증 → VisionCheck 가 나오는가 · 멀쩡한 장면에서 agree 인가 ·
    **일부러 어긋나게 만든 장면**을 잡아내는가 · 깊이를 끊으면 강등되는가 ·
    RGB 가 포기한 박스를 rescue 가 건지는가

rescue 판을 재려면 인식 노드를 그 모드로 띄우고 인자를 준다:
    ros2 run packing_vision zone_detector --ros-args -p zone:=b -p depth.mode:=rescue &
    python3 scripts/verify_vision.py rescue
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
from packing_common.ids import box_id as make_box_id                  # noqa: E402
from packing_vision.label import LABEL_MM, make_payload, render_qr    # noqa: E402
from packing_msgs.msg import BoxAtPickup, VisionCheck, ZoneStatus    # noqa: E402

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


def _project(loc_xy, z, center_xy, yaw_deg, t_cam):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    P = np.hstack([(np.asarray(loc_xy) @ np.array([[c, -s], [s, c]]).T)
                   + np.asarray(center_xy, float), np.full((len(loc_xy), 1), z)])
    uv = (K_mat() @ ((P - t_cam) @ R_DOWN).T).T
    return uv[:, :2] / uv[:, 2:3]


def render(box_type, center_xy, yaw_deg, t_cam, payload=None, inflate=0.0):
    """박스 윗면을 빨간 사각형으로 + (payload 가 있으면) 원근 투영된 QR 라벨.

    ★ 라벨을 안 그리면 cam1 이 QR 을 못 읽어 **한 개도 등록되지 않는다.**
      box_id 는 QR 이 정본이기 때문이다 (vision_node 주석 참조).
      ⚠ 예전 판은 라벨을 안 그렸는데, ② "DB 에 행이 생겼다" 가 **통과했다** —
        이전 실행이 남긴 boxes_sorted 행을 보고 있었기 때문이다.
        `gen_master_db --reset` 을 넣자 드러났다. 없는 기능이 통과로 위장한 경우다.
    """
    img = np.full((H, W, 3), (90, 90, 90), np.uint8)
    if box_type is None:
        return img
    w, d = box_type["w_mm"] / 1000.0, box_type["d_mm"] / 1000.0
    z = BELT + box_type["h_mm"] / 1000.0
    loc = np.array([[w / 2, d / 2], [-w / 2, d / 2], [-w / 2, -d / 2], [w / 2, -d / 2]])
    uv = _project(loc, z, center_xy, yaw_deg, t_cam)
    if inflate:
        # ★ 빨간 실루엣만 부풀린다 (QR 라벨은 그대로). 현실에서 이런 일이 나는 경우:
        #   윗면 가장자리에 하이라이트나 그림자가 얹혀 검출된 사각형이 실제보다
        #   커진다. 그러면 RGB 는 "사양 3종 어느 것과도 안 맞다" 며 **포기하고**,
        #   그 박스는 좌표가 안 나가 통째로 버려진다 — 에러 없이.
        uv = uv.mean(axis=0) + (uv - uv.mean(axis=0)) * (1.0 + inflate)
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


def render_depth(box_type, center_xy, yaw_deg, t_cam):
    """같은 장면의 깊이 이미지 (32FC1 · m).

    ★ 씬(Isaac ROS2CameraHelper type:"depth")과 같은 규약이다 — **광축 방향 거리**.
      시선 거리로 내면 화면 가장자리에서만 어긋나므로 눈으로는 안 보인다.
      값을 여기서 정확히 흉내 내야 노드 쪽 `is_range` 설정을 진짜로 검증한다.
    """
    K = K_mat()
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    d_cam = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)
    d_w = d_cam @ R_DOWN.T
    s_belt = (BELT - t_cam[2]) / d_w[..., 2]
    if box_type is None:
        return s_belt.astype(np.float32)

    z_top = BELT + box_type["h_mm"] / 1000.0
    s_top = (z_top - t_cam[2]) / d_w[..., 2]
    P = t_cam.reshape(1, 1, 3) + s_top[..., None] * d_w
    c, sn = math.cos(math.radians(-yaw_deg)), math.sin(math.radians(-yaw_deg))
    dx, dy = P[..., 0] - center_xy[0], P[..., 1] - center_xy[1]
    lx, ly = dx * c - dy * sn, dx * sn + dy * c
    inside = ((np.abs(lx) <= box_type["w_mm"] / 2000.0)
              & (np.abs(ly) <= box_type["d_mm"] / 2000.0))
    return np.where(inside, s_top, s_belt).astype(np.float32)


_DEPTH_CACHE = {}


def render_depth_cached(box_type, center_xy, yaw_deg, t_cam, name):
    """★ 캐시하는 이유: 깊이 렌더가 매 프레임 1280x720 메시그리드를 만든다.

    그대로 두면 하네스가 프레임당 수십 ms 를 태우고, 그동안 BEST_EFFORT 깊이가
    실제로 유실돼 노드가 STALE 을 낸다 — **하네스가 느린 것을 노드 결함으로 오독한다.**
    (첫 실행에서 실제로 그랬다.) 장면이 몇 개 안 되므로 캐시가 맞다.
    """
    key = (name, None if box_type is None else box_type["name"],
           round(center_xy[0], 4), round(center_xy[1], 4), round(yaw_deg, 3))
    if key not in _DEPTH_CACHE:
        _DEPTH_CACHE[key] = render_depth(box_type, center_xy, yaw_deg, t_cam).tobytes()
    return _DEPTH_CACHE[key]


class FakeScene(Node):
    def __init__(self):
        super().__init__("fake_scene")
        self.pub_img, self.pub_info, self.pub_depth = {}, {}, {}
        for name in CAMS:
            self.pub_img[name] = self.create_publisher(
                Image, f"/{name}/rgb", qos_profile_sensor_data)
            self.pub_info[name] = self.create_publisher(
                CameraInfo, f"/{name}/camera_info", qos_profile_sensor_data)
            self.pub_depth[name] = self.create_publisher(
                Image, f"/{name}/depth", qos_profile_sensor_data)
        self.pub_status = self.create_publisher(ZoneStatus, "/zone_b/status", 10)
        self.seen = []
        self.checks = []
        self.create_subscription(BoxAtPickup, "/zone_b/box_at_pickup",
                                 lambda m: self.seen.append(m), 10)
        # ★ 교차검증 진단 채널. 아무도 안 구독해도 되는 토픽이지만, 하네스가
        #   여기를 보면 **인식이 무엇을 근거로 그 좌표를 냈는지**까지 검증된다.
        self.create_subscription(VisionCheck, "/zone_b/vision_check",
                                 lambda m: self.checks.append(m), 10)
        self.create_subscription(VisionCheck, "/line/vision_check",
                                 lambda m: self.checks.append(m), 10)

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

    def send(self, name, box_type, center_xy, yaw, depth_type=-1, depth=True,
             payload=None, inflate=0.0):
        """RGB 와 깊이를 같이 낸다.

        `depth_type` 을 따로 주면 **깊이만 다른 박스로** 낸다 — 두 경로를 일부러
        갈라 놓고 교차검증이 잡는지 보기 위한 것이다 (-1 이면 RGB 와 같게).
        `depth=False` 면 깊이를 아예 안 낸다 (강등 경로 확인).
        """
        img = render(box_type, center_xy, yaw, CAMS[name], payload, inflate)
        msg = Image()
        msg.header.frame_id, msg.height, msg.width = name, H, W
        msg.encoding, msg.step = "bgr8", W * 3
        # ★ stamp 를 넣는다. 깊이와 시차를 재려면 둘 다 있어야 한다.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = img.tobytes()
        info = CameraInfo()
        info.header.frame_id, info.height, info.width = name, H, W
        K = K_mat()
        info.k = K.reshape(-1).tolist()
        info.distortion_model = "plumb_bob"
        self.pub_info[name].publish(info)
        self.pub_img[name].publish(msg)
        if not depth:
            return
        dt = box_type if depth_type == -1 else depth_type
        dm = Image()
        dm.header.frame_id, dm.height, dm.width = name, H, W
        dm.encoding, dm.step = "32FC1", W * 4
        # ★ stamp 를 RGB 와 같게 맞춘다. 노드가 시차로 STALE 을 거르므로
        #   여기서 안 맞추면 깊이가 통째로 버려지고 원인이 안 보인다.
        dm.header.stamp = msg.header.stamp
        dm.data = render_depth_cached(dt, center_xy, yaw, CAMS[name], name)
        self.pub_depth[name].publish(dm)

    def idle(self):
        self.pub_status.publish(ZoneStatus(zone="b", state="IDLE", accepting=True))


# ★ 씬 대역을 10 Hz 로 묶는다. 예전에는 spin 루프가 도는 대로 (약 15 Hz) 쐈는데,
#   깊이가 붙으면서 프레임당 3.7 MB 가 더해져 초당 190 MB 가 됐다 — 루프백 DDS 가
#   못 버텨 깊이가 절반 넘게 유실됐다. 실기 카메라도 30 Hz 이고 인식은 5 Hz 로만
#   추론하므로, 하네스가 그보다 빨리 쏠 이유가 없다.
SEND_PERIOD_S = 0.1


def pump(node, seconds, fn=None):
    end = time.time() + seconds
    nxt = 0.0
    while time.time() < end:
        if fn and time.time() >= nxt:
            nxt = time.time() + SEND_PERIOD_S
            fn()
        rclpy.spin_once(node, timeout_sec=0.02)


# 인식 노드를 `depth.mode:=rescue` 로 띄워 놓고 이 인자를 주면 ⑧ 의 기대가 뒤집힌다.
EXPECT_RESCUE = "rescue" in sys.argv


def main():
    ok_all = True

    def chk(label, cond, detail=""):
        nonlocal ok_all
        ok_all = ok_all and cond
        print(f"  {'✓' if cond else '✗ 실패'} {label}{(' — ' + detail) if detail else ''}")

    rclpy.init()
    n = FakeScene()
    BT, CENTER, YAW = TYPES[1], (9.78, 0.05), 25.0      # 4호
    # ★ box_id 는 boxes_master 와 같은 규칙이어야 한다 — QR 이 정본이므로
    #   여기서 다른 규칙을 쓰면 DB 와 하네스가 서로 다른 박스를 말하게 된다.
    BOX_ID = make_box_id(0)
    PAY = make_payload(BOX_ID)

    print("① 빈 벨트를 먼저 보여 준다 (검출기 arming)")
    pump(n, 2.0, lambda: (n.send("cam1", None, (0, 0), 0),
                          n.send("pick_cam", None, (0, 0), 0), n.idle()))
    chk("빈 벨트에 present=False", all(not m.present for m in n.seen) if n.seen else True,
        f"{len(n.seen)} 건 수신")

    print("② cam1 에 4호 박스 — 등록되는가")
    x1 = CFG["recognition"]["cam1_x_mm"] / 1000.0
    pump(n, 3.0, lambda: (n.send("cam1", BT, (x1, 0.02), 10.0, payload=PAY), n.idle()))
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
                          n.send("pick_cam", BT, CENTER, YAW, payload=PAY), n.idle()))
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
        chk("box_id 가 실렸다", hits[-1].box_id == BOX_ID,
            hits[-1].box_id or "(비어 있다)")

    print("④ 박스를 치운다 — present=False 로 떨어지는가")
    n.seen.clear()
    pump(n, 2.0, lambda: (n.send("pick_cam", None, (0, 0), 0), n.idle()))
    chk("present=False", any(not m.present for m in n.seen), f"{len(n.seen)} 건")

    # ── ⑤~⑦ 깊이 교차검증 ────────────────────────────────────────────────
    def pick_checks():
        return [c for c in n.checks if c.source == "pick_cam"]

    def last_used_depth():
        """★ 마지막 한 건이 아니라 **깊이를 실제로 쓴 마지막 건**을 본다.

        BEST_EFFORT 라 깊이 프레임은 가끔 유실된다. 유실된 프레임에서 깊이를
        건너뛰는 것은 **설계대로 맞는 동작**이다 (오래된 깊이를 쓰느니 안 쓴다).
        그러니 판정 내용은 깊이를 쓴 건에서 재고, 유실은 아래 사용률로 따로 센다.
        """
        got = [c for c in pick_checks() if c.depth_ok]
        return got[-1] if got else None

    def depth_use_rate():
        got = pick_checks()
        return (sum(1 for c in got if c.depth_ok) / len(got)) if got else 0.0

    print("⑤ 깊이 교차검증 — 멀쩡한 장면에서 두 경로가 일치하는가")
    # ★ 깊이 절은 다른 절보다 **오래 본다.** 깊이는 큰 이미지라 프레임이 가끔
    #   유실되고(문서 §4 참조), 짝이 안 맞은 프레임은 설계대로 건너뛴다.
    #   3 초로는 판정이 2 건밖에 안 잡히는 실행이 있었다 — 표본이 그것뿐이면
    #   **유실 한 번에 절 전체가 건너뛰어져** 뒤 검사가 통째로 안 돈다.
    #   기능의 문제가 아니라 표본 수의 문제다.
    n.checks.clear()
    n.seen.clear()
    pump(n, 6.0, lambda: (n.send("pick_cam", BT, CENTER, YAW, payload=PAY), n.idle()))
    c = last_used_depth()
    chk("VisionCheck 수신", bool(pick_checks()), f"{len(pick_checks())} 건")
    # ★ **프레임 비율이 아니라 "이 박스가 깊이로 확인됐는가"** 를 본다.
    #   박스는 스토퍼에 몇 초 서 있고 추론은 5 Hz 라, 프레임 몇 장을 건너뛰는 것은
    #   아무 문제가 아니다. 프레임 비율은 호스트 대역폭의 성질이지 정확성이 아니다.
    #   (실측: 같은 코드가 대역이 넉넉하면 75 %, 빠듯하면 33 % 로 나온다.)
    chk("이 박스가 깊이로 확인됐다", c is not None,
        f"깊이를 쓴 판정 {len([x for x in pick_checks() if x.depth_ok])} 건 "
        f"/ 전체 {len(pick_checks())} 건 ({depth_use_rate():.0%})")
    if c is None:
        print("   ↳ 깊이가 꺼져 있거나 토픽이 안 온다. 아래 ⑥⑦ 는 건너뛴다.")
    else:
        chk("깊이를 실제로 썼다", c.depth_ok, c.depth_reason or "ok")
        chk("판정 OK", c.verdict == "OK", f"{c.verdict} · used={c.used}")
        chk("두 경로 일치", c.agree,
            f"dz {c.dz_mm:+.1f}mm · dxy {c.dxy_mm:.1f}mm · dyaw {c.dyaw_deg:.1f}도")
        # ★ 깊이가 잰 높이가 사양과 맞는가 — 이게 RGB 와 **독립인** 유일한 숫자다
        chk("깊이 높이 실측이 사양과 맞다", abs(c.depth_h_mm - BT["h_mm"]) < 15.0,
            f"{c.depth_h_mm:.1f} mm (사양 {BT['h_mm']})")
        chk("윗면 픽셀이 충분하다", c.depth_valid_ratio > 0.8 and c.depth_px_used > 500,
            f"비율 {c.depth_valid_ratio:.2f} · {c.depth_px_used} px")

        print("⑥ ★ 일부러 어긋나게 — RGB 는 4호, 깊이는 5호 높이로 낸다")
        # 실제로 이런 일이 나는 경우: 박스가 벨트에 안 붙어 있거나(이음매에서 튐)
        # 다른 것 위에 얹혀 왔다. RGB 실루엣은 그대로인데 높이만 다르다.
        n.checks.clear()
        pump(n, 5.0, lambda: (n.send("pick_cam", BT, CENTER, YAW,
                                     depth_type=TYPES[2], payload=PAY), n.idle()))
        c2 = last_used_depth()
        chk("어긋남을 잡아냈다", c2 is not None and not c2.agree,
            c2.verdict if c2 else "깊이를 쓴 건이 없다")
        if c2:
            chk("사유가 높이 계열이다",
                c2.verdict in ("HEIGHT_MISMATCH", "TYPE_MISMATCH"), c2.verdict)
            chk("dz 가 두 사양의 높이 차만큼 난다",
                abs(abs(c2.dz_mm) - (TYPES[2]["h_mm"] - BT["h_mm"])) < 20.0,
                f"{c2.dz_mm:+.1f} mm (예상 {BT['h_mm'] - TYPES[2]['h_mm']:+d})")
            # ★ 기본값은 **막지 않는다.** 데모가 서는 쪽이 더 나쁘다는 판단이다.
            chk("기본 설정에서는 좌표를 계속 낸다 (막지 않는다)", c2.used == "RGB",
                f"used={c2.used}")

        print("⑦ 깊이를 끊는다 — 강등되지만 **라인은 계속 도는가**")
        n.checks.clear()
        n.seen.clear()
        pump(n, 3.0, lambda: (n.send("pick_cam", BT, CENTER, YAW, depth=False, payload=PAY),
                              n.idle()))
        c3 = pick_checks()[-1] if pick_checks() else None
        hits3 = [m for m in n.seen if m.present]
        # 이게 이 기능의 안전 요건이다 — 깊이가 죽어도 좌표는 나와야 한다
        chk("깊이가 없어도 BoxAtPickup 은 계속 나온다", bool(hits3), f"{len(hits3)} 건")
        chk("강등이 기록된다",
            c3 is not None and c3.used == "RGB" and not c3.depth_ok,
            (c3.verdict + " / " + c3.used) if c3 else "수신 없음")

        print("⑧ ★ 모서리가 12 % 부풀어 RGB 가 포기하는 장면")
        # ★ 깊이 없이는 이 박스가 통째로 버려진다 (에러도 없이). 깊이가 그걸 건지는가.
        #   기본(crosscheck)은 **안 건지는 것이 맞는 동작**이다 — 재현성 때문에
        #   RGB 가 답을 낸 경우와 규칙을 같게 두고, 건지려면 rescue 로 켠다.
        n.checks.clear()
        n.seen.clear()
        pump(n, 4.0, lambda: (n.send("pick_cam", BT, CENTER, YAW,
                                     payload=PAY, inflate=0.12), n.idle()))
        c4 = last_used_depth()
        hits4 = [m for m in n.seen if m.present]
        chk("RGB 가 포기했다 (전제 확인)", c4 is not None and not c4.rgb_ok,
            f"rgb_ok={c4.rgb_ok if c4 else '판정 없음'}")
        if c4:
            chk("깊이는 같은 장면에서 답을 냈다", c4.depth_ok,
                f"{c4.depth_type} · 높이 {c4.depth_h_mm:.1f} mm")
            chk("깊이가 잰 높이가 사양과 맞다", abs(c4.depth_h_mm - BT["h_mm"]) < 15.0,
                f"{c4.depth_h_mm:.1f} mm (사양 {BT['h_mm']})")
        if EXPECT_RESCUE:
            chk("rescue: 버려질 박스를 건졌다", bool(hits4), f"{len(hits4)} 건")
            chk("rescue: 채택 경로가 깊이다", c4 is not None and c4.used == "DEPTH",
                c4.used if c4 else "-")
            if hits4:
                m = hits4[-1]
                dx, dy = m.x_mm - CENTER[0] * 1000, m.y_mm - CENTER[1] * 1000
                chk("rescue: 건진 좌표가 10 mm 안", abs(dx) < 10 and abs(dy) < 10,
                    f"{dx:+.1f}, {dy:+.1f} mm")
        else:
            chk("기본(crosscheck): 건지지 않는다 (설계대로)", not hits4,
                f"{len(hits4)} 건 · used={c4.used if c4 else '-'}")

    n.destroy_node()
    rclpy.shutdown()
    print("\n" + ("전부 통과" if ok_all else "★ 실패 있음"))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
