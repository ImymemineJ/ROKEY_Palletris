# packing_common/frames.py — 팔레트 좌표계 변환 (작업명세서 §3.3)
# ★ 트랙 B(김병직)·C(정정인)가 반드시 이 파일을 같이 import 해야 좌표가 맞는다.
#   계약: x_mm, y_mm = 박스 중심 / z_mm = 박스 밑면 / yaw_deg=0 → size_w 가 Px 방향
from dataclasses import dataclass, asdict

from packing_common.units import m_to_mm, mm_to_m


@dataclass
class PlacePose:
    """packing_msgs/PlacePose.msg 와 필드가 1:1 대응하는 순수 파이썬 판.

    Isaac·ROS 없이 노트북에서 돌려야 하므로(트랙 C 규약: 코어는 ROS 의존성 0)
    여기서 packing_msgs 를 import 하지 않는다. ROS 메시지와의 왕복은
    Day 3 서비스 래퍼에서 from_msg / to_msg 로만 한다.
    """

    x_mm: int        # 박스 중심
    y_mm: int        # 박스 중심
    z_mm: int        # ★ 박스 밑면 (중심 아님)
    yaw_deg: float   # 계획값 0.0 / 90.0 · 실측값은 임의 실수

    # ---------- ROS 메시지 어댑터 (packing_msgs 를 import 하지 않기 위한 우회) ----------
    @classmethod
    def from_msg(cls, msg):
        """packing_msgs.msg.PlacePose → PlacePose. 같은 필드를 가진 객체면 무엇이든 받는다."""
        return cls(int(msg.x_mm), int(msg.y_mm), int(msg.z_mm), float(msg.yaw_deg))

    def to_msg(self, msg_cls):
        """PlacePose → packing_msgs.msg.PlacePose. 메시지 '클래스'를 인자로 받아 의존성을 끊는다."""
        return msg_cls(x_mm=int(self.x_mm), y_mm=int(self.y_mm),
                       z_mm=int(self.z_mm), yaw_deg=float(self.yaw_deg))

    # ---------- dict 어댑터 (heightmap_core 내부는 dict 로 pose 를 주고받는다) ----------
    @classmethod
    def from_dict(cls, d):
        return cls(int(d["x_mm"]), int(d["y_mm"]), int(d["z_mm"]), float(d["yaw_deg"]))

    def as_dict(self):
        return asdict(self)


def world_to_pallet_place(center_xyz_m, yaw_deg, h_mm, origin_mm) -> PlacePose:
    """Isaac에서 읽은 박스 pose(중심) → PlacePose(밑면)"""
    return PlacePose(
        x_mm=m_to_mm(center_xyz_m[0]) - origin_mm[0],
        y_mm=m_to_mm(center_xyz_m[1]) - origin_mm[1],
        z_mm=m_to_mm(center_xyz_m[2]) - origin_mm[2] - h_mm // 2,   # ★ 중심→밑면
        yaw_deg=yaw_deg)


def pallet_place_to_world(place, h_mm, origin_mm):
    """PlacePose(밑면) → 월드 박스 중심"""
    return (mm_to_m(place.x_mm + origin_mm[0]),
            mm_to_m(place.y_mm + origin_mm[1]),
            mm_to_m(place.z_mm + origin_mm[2] + h_mm // 2))          # ★ 밑면→중심
