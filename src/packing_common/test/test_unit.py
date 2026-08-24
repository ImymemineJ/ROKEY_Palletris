from packing_common.units import *
from packing_common.frames import *

def test_roundtrip():
    assert mm_to_cell(340) == 34 and mm_to_cell(480) == 48
    assert m_to_mm(mm_to_m(11200)) == 11200

def test_frames_center_bottom():
    origin = [11200, -600, 150]
    # 3호(h=210) 박스 중심이 월드 (11.37, -0.475, 0.255+0.105)에 있으면
    # PlacePose는 x=170, y=125, z=105(밑면)여야 한다... 손계산 케이스 3개
    p = world_to_pallet_place([11.370, -0.475, 0.360], 0.0, 210, origin)
    assert (p.x_mm, p.y_mm, p.z_mm) == (170, 125, 105)
    back = pallet_place_to_world(p, 210, origin)
    assert abs(back[2] - 0.360) < 1e-9