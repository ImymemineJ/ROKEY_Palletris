"""인식 코어 검증 — Isaac 없이 합성 이미지로 돈다.

씬 USD 가 이 저장소에 없으므로(sync_scene.sh 배포) 진짜 카메라 영상은 못 쓴다.
대신 **알려진 카메라·알려진 박스**를 직접 투영해 이미지를 만들고, 그 이미지를
파이프라인에 넣어 원래 값이 되돌아오는지 본다. 기하가 틀리면 여기서 잡힌다.
"""
import numpy as np
import cv2
import pytest

from packing_common.config import load
from packing_vision.detect import find_box, dominant_color_name
from packing_vision.geometry import box_pose_from_corners, quat_to_R

CFG = load()
TYPES = CFG["boxes"]["types"]
BELT_TOP_M = CFG["conveyor"]["belt_top_z_mm"] / 1000.0
W, H = CFG["recognition"]["resolution"]
HFOV = CFG["recognition"]["hfov_deg"]

# 하향 카메라. 광학 규약(x 오른쪽·y 아래·z 앞)에서 z 를 월드 -Z 로 향하게.
R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])


def make_K():
    fx = (W / 2.0) / np.tan(np.radians(HFOV) / 2.0)
    return np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])


def project(P_w, K, R, t):
    """월드 점 → 픽셀 (테스트가 이미지를 만들 때만 쓴다)."""
    p = (np.asarray(P_w, float) - t) @ R          # R^T @ v  == v @ R
    uv = (K @ p.T).T
    return uv[:, :2] / uv[:, 2:3]


def render(box_type, center_xy, yaw_deg, K, R, t):
    """박스 윗면을 빨간 사각형으로 그린 합성 이미지."""
    w = box_type["w_mm"] / 1000.0
    d = box_type["d_mm"] / 1000.0
    z_top = BELT_TOP_M + box_type["h_mm"] / 1000.0
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    local = np.array([[+w / 2, +d / 2], [-w / 2, +d / 2],
                      [-w / 2, -d / 2], [+w / 2, -d / 2]])
    rot = np.array([[c, -s], [s, c]])
    corners = (local @ rot.T) + np.asarray(center_xy, float)
    P = np.hstack([corners, np.full((4, 1), z_top)])

    img = np.full((H, W, 3), (90, 90, 90), np.uint8)      # 회색 벨트
    cv2.fillPoly(img, [project(P, K, R, t).astype(np.int32)], (40, 40, 235))  # BGR 빨강
    return img


CASES = [
    (TYPES[0], (9.80, 0.00), 0.0),
    (TYPES[1], (9.78, 0.12), 30.0),
    (TYPES[2], (9.82, -0.15), 90.0),
    (TYPES[1], (9.75, -0.10), 155.0),
]


@pytest.mark.parametrize("btype,center,yaw", CASES)
def test_pose_roundtrip(btype, center, yaw):
    K = make_K()
    t = np.array([9.795, 0.0, 2.300])          # 진행보고의 씬 프림 위치 (테스트 전용)
    img = render(btype, center, yaw, K, R_DOWN, t)

    det = find_box(img)
    assert det is not None, "합성 이미지에서 박스를 못 찾았다"
    assert dominant_color_name(img) == "red"

    out = box_pose_from_corners(det["corners_px"], K, R_DOWN, t, BELT_TOP_M, TYPES)
    assert out is not None, "종류 매칭 실패"

    # ① 종류를 맞혔나 — 틀리면 높이가 틀리고 로봇이 허공/박스 속으로 간다
    assert out["type"]["name"] == btype["name"], \
        f'{btype["name"]} 을 {out["type"]["name"]} 으로 봤다'

    # ② 중심 — 흡착 여유를 생각하면 10 mm 안이어야 한다
    cx, cy, cz = out["center_m"]
    assert abs(cx - center[0]) < 0.010, f"x 오차 {abs(cx-center[0])*1000:.1f} mm"
    assert abs(cy - center[1]) < 0.010, f"y 오차 {abs(cy-center[1])*1000:.1f} mm"

    # ③ z 는 박스 중심(밑면 아님). 벨트 상면 + h/2
    assert abs(cz - (BELT_TOP_M + btype["h_mm"] / 2000.0)) < 1e-6

    # ④ yaw — 180° 대칭이라 접어서 비교한다
    d = abs(out["yaw_deg"] - (yaw % 180.0))
    assert min(d, 180.0 - d) < 3.0, f"yaw 오차 {min(d, 180.0-d):.1f}°"


def test_empty_belt_returns_none():
    """박스가 없으면 None 이어야 한다. 여기서 아무거나 돌려주면
    관제탑이 빈 벨트를 집으러 간다."""
    img = np.full((H, W, 3), (90, 90, 90), np.uint8)
    assert find_box(img) is None


def test_quat_to_R_identity():
    assert np.allclose(quat_to_R(0, 0, 0, 1), np.eye(3))


def test_clipping_is_flagged():
    """시야 밖으로 잘린 박스는 clipped 로 표시돼야 한다.

    5호가 지터 최대(±150)에서 90° 돌면 필요 ±390 mm 인데 pick_cam 의 좁은 축
    커버가 ±377 mm 다. 13 mm 모자라 실제로 잘린다. 잘린 사각형의 중심은 진짜
    중심이 아니라서(실측 7.7 mm 어긋남) 그대로 쓰면 로봇이 빗나간다.
    """
    K = make_K()
    t = np.array([9.795, 0.0, 2.300])
    big = max(TYPES, key=lambda b: b["h_mm"])          # 5호
    img = render(big, (9.82, -0.15), 90.0, K, R_DOWN, t)
    det = find_box(img)
    assert det is not None
    assert det["clipped"] is True, "잘렸는데 표시가 안 됐다"


def test_not_clipped_when_centered():
    K = make_K()
    t = np.array([9.795, 0.0, 2.300])
    img = render(TYPES[0], (9.80, 0.0), 0.0, K, R_DOWN, t)
    det = find_box(img)
    assert det is not None and det["clipped"] is False


def test_pick_cam_fov_covers_worst_case():
    """★ 현재 씬 배치로는 5호가 안 들어간다 — 알고도 두는 것과 모르고 두는 것은 다르다.

    트랙 A 가 카메라를 광축 기준 90° 돌리면(넓은 축을 Y 로) 통과한다.
    고쳐지면 이 테스트가 xfail 을 벗어나 실패하므로 그때 마크를 지우면 된다.
    """
    W_px, H_px = CFG["recognition"]["resolution"]
    fx = (W_px / 2.0) / np.tan(np.radians(CFG["recognition"]["hfov_deg"]) / 2.0)
    z_cam = CFG["recognition"]["pick_cam"]["z_mm"]
    belt = CFG["conveyor"]["belt_top_z_mm"]
    jitter = 150                       # CLAUDE.md §4. yaml 에는 아직 키가 없다

    worst = min((H_px / 2.0) / fx * (z_cam - belt - b["h_mm"])
                - (jitter + max(b["w_mm"], b["d_mm"]) / 2.0) for b in TYPES)
    if worst < 0:
        pytest.xfail(f"pick_cam 시야가 {abs(worst):.0f} mm 모자란다 — "
                     "카메라를 광축 기준 90° 회전 필요 (트랙 A)")
    assert worst >= 0
