"""frames.py — 팔레트 좌표계 변환. **트랙 B·C 가 같이 쓰는 좌표 규약의 정본.**

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : ⑥⑦⑧⑨ 좌표 규약 — 팔레트↔월드, 중심↔밑면
    ║     불러 주는 쪽        : 트랙 B(cycle) · 트랙 C(algo) · 검증 스크립트
    ║     이 파일이 부르는 쪽 : units (mm↔m 변환)
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

## ★ 계약 한 줄

    x_mm, y_mm = 박스 **중심**  /  z_mm = 박스 **밑면**  /  yaw_deg=0 → size_w 가 Px 방향

**z 를 중심으로 쓰면 박스가 팔레트에 절반 파묻힌다.** 반대로 밑면을 중심으로 쓰면
공중에 뜬다. 이 규약이 어긋나면 알고리즘은 정상인데 화면에서만 이상해지고,
로그에는 아무 단서도 안 남는다.

## ★ 왜 복사하지 말고 import 해야 하나

트랙 B(로봇)와 트랙 C(알고리즘)가 **같은 변환을 양쪽에서 한다.**
계획할 때는 월드 → 팔레트, 놓고 나서 실측할 때는 팔레트 → 월드다.
한쪽만 `h_mm // 2` 를 빼먹으면 계획과 실측이 박스 높이의 절반만큼 어긋나는데,
그 오차가 heightmap 에 누적돼 **알고리즘은 1300 안이라 믿고 실물은 넘는다.**
그래서 이 파일은 두 트랙이 **같은 함수를 부르는** 자리다.

## 두 방향

    world_to_pallet_place()   Isaac 에서 읽은 박스 pose(중심) → PlacePose(밑면)
                              ⑨ 실측 갱신에서 쓴다
    pallet_place_to_world()   PlacePose(밑면) → 월드 박스 중심
                              ⑧ 놓을 자리를 로봇 좌표로 바꿀 때 쓴다

## ROS 의존성이 0 인 이유

`PlacePose` 를 여기서 순수 파이썬 dataclass 로 다시 정의한다. 트랙 C 의 코어는
노트북에서 `packing_msgs` 없이 돌아야 하기 때문이다. ROS 메시지와의 왕복은
`from_msg`/`to_msg` 어댑터 두 개로만 한다 — **메시지 클래스를 인자로 받아**
import 자체를 피한다.
"""
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
