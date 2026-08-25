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


# ══════════════════════════════════════════════════════════════════════
#  QR 라벨 — 페이로드 규약 · 크기 · 해독
# ══════════════════════════════════════════════════════════════════════
from packing_vision.detect import read_qr                      # noqa: E402
from packing_vision.label import (                             # noqa: E402
    LABEL_MM, MIN_MODULE_PX, MODULES, QR_MM, make_payload, modules_for,
    parse_payload, render_qr, required_qr_mm,
)


def render_with_label(box_type, center_xy, yaw_deg, K, R, t, payload,
                      label_mm=LABEL_MM):
    """render() 와 같되 윗면 한가운데에 **원근 투영된** QR 라벨을 얹는다.

    라벨을 정사각으로 붙여 놓고 끝내면 안 된다. 실제 카메라는 박스가 화면
    가장자리로 갈수록 라벨을 사다리꼴로 본다 — 그 상태에서도 읽히는지가
    확인하고 싶은 것이다. 그래서 라벨 네 귀퉁이를 월드에 놓고 같이 투영한다.
    """
    img = render(box_type, center_xy, yaw_deg, K, R, t)
    z_top = BELT_TOP_M + box_type["h_mm"] / 1000.0
    half = label_mm / 2000.0                       # mm -> m, 반변
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    rot = np.array([[c, -s], [s, c]])
    # 이미지 좌표계와 감기 방향을 맞춘다 — 뒤집히면 QR 이 거울상이 되어 안 읽힌다.
    local = np.array([[-half, +half], [+half, +half], [+half, -half], [-half, -half]])
    quad = (local @ rot.T) + np.asarray(center_xy, float)
    dst = project(np.hstack([quad, np.full((4, 1), z_top)]), K, R, t).astype(np.float32)

    lab = cv2.cvtColor(render_qr(payload, module_px=16), cv2.COLOR_GRAY2BGR)
    n = lab.shape[0]
    src = np.array([[0, 0], [n, 0], [n, n], [0, n]], np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(lab, M, (W, H))
    mask = cv2.warpPerspective(np.full((n, n), 255, np.uint8), M, (W, H))
    img[mask > 0] = warped[mask > 0]
    return img


def test_payload_is_waybill_only():
    """★ QR 에 구역 글자를 넣지 않는다 — 구역은 주소로 정한다 (설계결정서 §5-3)."""
    assert make_payload("BOX00007") == "BOX00007"
    assert parse_payload("BOX00007") == "BOX00007"


@pytest.mark.parametrize("bad", ["", "   ", "BOX 1", None])
def test_payload_rejects_garbage(bad):
    """★ 예외가 아니라 None 이다 — 구독 콜백에서 raise 하면 rclpy 가 삼킨다."""
    assert parse_payload(bad) is None


def test_payload_rejects_old_format():
    """옛 규약 라벨이 씬에 남아 있으면 조용히 통과시키지 않고 잡아낸다."""
    assert parse_payload("B|BOX00007") is None


def test_payload_stays_25_modules():
    """글자가 늘면 29 모듈로 커지고 같은 크기에서 해독 여유가 14% 깎인다."""
    assert modules_for(make_payload("BOX00014")) == MODULES == 25


def test_label_fits_every_box_top():
    for b in TYPES:
        narrow = min(b["w_mm"], b["d_mm"])
        assert LABEL_MM <= narrow, f"{b['name']} 윗면({narrow})보다 라벨({LABEL_MM})이 크다"


def test_label_big_enough_for_pick_cam():
    """pick_cam 에서 모듈당 5 px 이상인가 — 이게 무너지면 흐림에 간헐 실패한다."""
    fx = (W / 2.0) / np.tan(np.radians(HFOV) / 2.0)
    z_cam = CFG["recognition"]["pick_cam"]["z_mm"] / 1000.0
    worst_h = max(b["h_mm"] for b in TYPES) / 1000.0      # 제일 높은 = 제일 가까운
    lowest_h = min(b["h_mm"] for b in TYPES) / 1000.0     # 제일 낮은 = 제일 먼 = 최악
    assert worst_h >= lowest_h
    mm_per_px = (z_cam - BELT_TOP_M - lowest_h) * 1000.0 / fx
    assert QR_MM >= required_qr_mm(mm_per_px), (
        f"QR {QR_MM} mm 로는 부족하다 — {required_qr_mm(mm_per_px):.0f} mm 필요 "
        f"({mm_per_px:.2f} mm/px)")
    assert QR_MM / MODULES / mm_per_px >= MIN_MODULE_PX


@pytest.mark.parametrize("btype,center,yaw", CASES)
def test_qr_decodes_from_projected_image(btype, center, yaw):
    """★ 지금까지 read_qr 은 테스트가 한 건도 없었다. 여기가 첫 검증이다."""
    K = make_K()
    t = np.array([9.9, 0.0, CFG["recognition"]["pick_cam"]["z_mm"] / 1000.0])
    payload = make_payload("BOX00007")
    img = render_with_label(btype, center, yaw, K, R_DOWN, t, payload)
    assert parse_payload(read_qr(img)) == "BOX00007"


@pytest.mark.parametrize("btype,center,yaw", CASES)
def test_label_does_not_break_box_detection(btype, center, yaw):
    """라벨은 윗면 한가운데의 **빨갛지 않은 구멍**이다.

    find_box 가 RETR_EXTERNAL 로 바깥 윤곽만 보므로 구멍은 무시돼야 한다.
    그게 깨지면 로봇이 라벨 크기만큼 작은 박스를 집으려 든다.
    """
    K = make_K()
    t = np.array([9.9, 0.0, CFG["recognition"]["pick_cam"]["z_mm"] / 1000.0])
    plain = find_box(render(btype, center, yaw, K, R_DOWN, t))
    with_lab = find_box(render_with_label(btype, center, yaw, K, R_DOWN, t,
                                          make_payload("BOX00007")))
    assert plain is not None and with_lab is not None
    a, b = sorted(plain["size_px"]), sorted(with_lab["size_px"])
    assert max(abs(x - y) for x, y in zip(a, b)) <= 2.0, "라벨이 치수를 바꿨다"
    assert np.linalg.norm(np.subtract(plain["center_px"], with_lab["center_px"])) <= 2.0


def test_label_constants_come_from_yaml():
    """label.py 의 기본값과 yaml 이 갈리면 씬과 인식이 다른 크기를 믿게 된다."""
    from packing_vision import label as L
    q = CFG["recognition"]["qr"]
    assert L._FROM_YAML, "yaml 을 못 읽고 기본값으로 떨어졌다"
    assert (L.MODULE_MM, L.QUIET_MODULES, L.LABEL_MM) == (
        q["module_mm"], q["quiet_modules"], q["label_mm"])
    assert L._DEF["label_mm"] == q["label_mm"], "yaml 을 바꿨으면 label.py 기본값도 맞출 것"
