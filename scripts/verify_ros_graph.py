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
      /conveyor/set_velocity     벨트
      /sim/despawn_box 외 3종     씬 서비스
      /robot/cmd → /robot/event  로봇 팔

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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                "..", "src", "packing_common"))
from packing_common.config import load                                  # noqa: E402
from packing_msgs.msg import RobotCmd, RobotEvent                       # noqa: E402
from packing_msgs.srv import (DespawnBox, SetSpawnEnabled, SetVelocity,  # noqa: E402
                              SwapPallet)

CFG = load()
W, H = CFG["recognition"]["resolution"]
HFOV = CFG["recognition"]["hfov_deg"]
BELT = CFG["conveyor"]["belt_top_z_mm"] / 1000.0
TYPES = CFG["boxes"]["types"]
STOPPER = CFG["conveyor"]["stopper_x_mm"] / 1000.0

CAMS = {"cam1": np.array([CFG["recognition"]["cam1_x_mm"] / 1000.0, 0.0, 2.300]),
        "pick_cam": np.array([9.795, 0.0, 2.300])}
R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])

# ★ 실측 오차. 로봇이 계획대로 정확히 놓지 않는다는 것을 재현한다.
#   has_actual=true 로 이 값을 돌려줘야 heightmap 이 실제 더미를 따라간다.
#   계획값을 되보내면 오차가 누적돼 알고리즘은 1300 안이라 믿는데 실제는 넘는다.
PLACE_ERR_MM = (3, -2, 1)


def K_mat():
    fx = (W / 2.0) / math.tan(math.radians(HFOV) / 2.0)
    return np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])


def render(box_type, center_xy, yaw_deg, t_cam):
    img = np.full((H, W, 3), (90, 90, 90), np.uint8)
    if box_type is None:
        return img
    w, d = box_type["w_mm"] / 1000.0, box_type["d_mm"] / 1000.0
    z = BELT + box_type["h_mm"] / 1000.0
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    loc = np.array([[w / 2, d / 2], [-w / 2, d / 2], [-w / 2, -d / 2], [w / 2, -d / 2]])
    P = np.hstack([(loc @ np.array([[c, -s], [s, c]]).T) + np.asarray(center_xy, float),
                   np.full((4, 1), z)])
    uv = (K_mat() @ ((P - t_cam) @ R_DOWN).T).T
    cv2.fillPoly(img, [np.round(uv[:, :2] / uv[:, 2:3]).astype(np.int32)], (40, 40, 235))
    return img


class IsaacStandIn(Node):
    """Isaac 이 ROS 2 로 내는 것 전부를 대신한다."""

    def __init__(self, plan):
        super().__init__("isaac_stand_in")
        cb = ReentrantCallbackGroup()
        self.plan = list(plan)          # [(type, center_xy, yaw), ...]
        self.log = self.get_logger()

        # ── /clock — 규약 1 ──────────────────────────────────────
        # use_sim_time 인 노드들의 시계가 여기서 온다. 없으면 0 에 멈춘다.
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.t0 = time.time()
        self.create_timer(0.02, self._tick_clock, callback_group=cb)

        # ── 카메라 ────────────────────────────────────────────────
        self.pub_img, self.pub_info = {}, {}
        for nm in CAMS:
            self.pub_img[nm] = self.create_publisher(Image, f"/{nm}/rgb",
                                                     qos_profile_sensor_data)
            self.pub_info[nm] = self.create_publisher(CameraInfo, f"/{nm}/camera_info",
                                                      qos_profile_sensor_data)
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

        # ── 로봇 ──────────────────────────────────────────────────
        self.pub_ev = self.create_publisher(RobotEvent, "/robot/event", 10)
        self.create_subscription(RobotCmd, "/robot/cmd", self._on_cmd, 10,
                                 callback_group=cb)

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
        self.create_timer(0.1, self._pump_cameras, callback_group=cb)
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
    def _send(self, nm, bt, center, yaw):
        img = render(bt, center, yaw, CAMS[nm])
        m = Image()
        m.header.frame_id, m.height, m.width = nm, H, W
        m.encoding, m.step, m.data = "bgr8", W * 3, img.tobytes()
        info = CameraInfo()
        info.header.frame_id, info.height, info.width = nm, H, W
        info.k = K_mat().reshape(-1).tolist()
        info.distortion_model = "plumb_bob"
        self.pub_info[nm].publish(info)
        self.pub_img[nm].publish(m)

    def _pump_cameras(self):
        bt, center, yaw = self.cur if self.cur else (None, (0, 0), 0.0)
        x1 = CFG["recognition"]["cam1_x_mm"] / 1000.0
        self._send("cam1", bt if self.stage == "cam1" else None, (x1, center[1]), yaw)
        self._send("pick_cam", bt if self.stage == "pick" else None, center, yaw)

    def _advance(self):
        el = time.time() - self._t_cam1
        if self.stage == "gap" and self.cur and el > 1.2:
            # 빈 벨트를 충분히 보여 줬다 — 이제 투입부에 박스를 올린다
            self.stage = "cam1"
            self._t_cam1 = time.time()
        elif self.stage == "cam1" and self.cur and el > 2.0:
            # cam1 에서 잠깐 보여 준 뒤 스토퍼로 옮긴다 (벨트 이송을 압축한 것)
            self.stage = "pick"
            self.log.info(f"[씬] 박스가 스토퍼에 도착 — {self.cur[0]['name']}")

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
    def _on_cmd(self, cmd):
        """PICK/PLACE 를 받아 성공을 돌려준다. ★ seq 를 반드시 되돌려준다."""
        ev = RobotEvent()
        ev.seq = cmd.seq                 # ← 이게 없으면 관제탑이 결과를 못 짝짓는다
        ev.action = cmd.action
        ev.box_id = cmd.box_id
        ev.ok = True
        ev.reason = ""
        ev.duration_s = 1.0

        if cmd.action == RobotCmd.PICK:
            self.log.info(f"[로봇] PICK {cmd.box_id} "
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
            self.log.info(f"[로봇] PLACE {cmd.box_id} → 팔레트 "
                          f"({cmd.x_mm},{cmd.y_mm},{cmd.z_mm})")
            self.placed.append(cmd.box_id)
            # 다음 박스를 투입한다
            self.cur = self.plan.pop(0) if self.plan else None
            self.stage = "gap" if self.cur else "gone"      # ★ 빈 벨트를 먼저 보여 준다
            self._t_cam1 = time.time()
        self.pub_ev.publish(ev)


def main():
    n_box = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rng = np.random.default_rng(CFG["boxes"]["random_seed"])
    plan = []
    for i in range(n_box):
        bt = TYPES[int(rng.integers(0, len(TYPES)))]
        y = float(rng.uniform(-0.10, 0.10))
        plan.append((bt, (STOPPER - 0.22, y), float(rng.uniform(-15, 15))))

    rclpy.init()
    scene = IsaacStandIn(plan)
    ex = MultiThreadedExecutor()
    ex.add_node(scene)

    print(f"\n박스 {n_box} 개를 그래프에 흘려보낸다 "
          f"({', '.join(b[0]['name'] for b in plan)})\n")
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

    print()
    chk(f"로봇이 {n_box} 개를 놓았다", len(placed) == n_box, f"{len(placed)} 개")
    chk("DB 에 행이 다 생겼다", len(rows) == n_box, f"{len(rows)} 행")
    chk("전부 PLACED", all(r[1] == "PLACED" for r in rows),
        ", ".join(f"{r[0]}={r[1]}" for r in rows))
    chk("실측 pose 가 기록됐다", all(r[2] is not None for r in rows))
    print("\n  적재 결과")
    for r in rows:
        print(f"    {r[0]}  {r[1]:8s}  ({r[2]}, {r[3]}, {r[4]}) mm")
    print("\n" + ("전부 통과" if ok else "★ 실패 있음"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
