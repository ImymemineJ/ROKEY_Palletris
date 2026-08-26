"""깊이 교차검증 코어 검증 — Isaac 없이 합성 이미지로 돈다.

`test_vision_core.py` 와 같은 방식이다. 알려진 카메라·알려진 박스를 직접 투영해
**RGB 와 깊이를 같은 장면에서** 만들고, 파이프라인에 넣어 원래 값이 되돌아오는지 본다.

★ 여기서 확인하려는 것은 "깊이가 맞다" 가 아니라 **"두 경로가 독립이다"** 이다.
  독립이 아니면 교차검증이 아무것도 못 잡는다. 그래서 테스트의 절반은
  **RGB 를 일부러 틀리게 만들고 깊이가 그걸 잡아내는지** 본다.
"""
import numpy as np
import cv2
import pytest

from packing_common.config import load
from packing_vision.detect import find_box
from packing_vision.geometry import box_pose_from_corners
from packing_vision import depth as D

CFG = load()
TYPES = CFG["boxes"]["types"]
BELT_M = CFG["conveyor"]["belt_top_z_mm"] / 1000.0
W, H = CFG["recognition"]["resolution"]
HFOV = CFG["recognition"]["hfov_deg"]

# 하향 카메라. 광학 규약(x 오른쪽·y 아래·z 앞)에서 z 를 월드 -Z 로.
R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
T_CAM = np.array([9.795, 0.0, 2.300])       # 진행보고의 씬 프림 위치 (테스트 전용)
CFG_D = D.merged_cfg(CFG.get("recognition"))


def make_K():
    fx = (W / 2.0) / np.tan(np.radians(HFOV) / 2.0)
    return np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])


def top_corners(bt, center_xy, yaw_deg):
    """박스 윗면 네 모서리의 월드 좌표 (4,3)."""
    w, d = bt["w_mm"] / 1000.0, bt["d_mm"] / 1000.0
    z = BELT_M + bt["h_mm"] / 1000.0
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    loc = np.array([[w / 2, d / 2], [-w / 2, d / 2], [-w / 2, -d / 2], [w / 2, -d / 2]])
    xy = (loc @ np.array([[c, -s], [s, c]]).T) + np.asarray(center_xy, float)
    return np.hstack([xy, np.full((4, 1), z)])


def project(P_w, K, R=R_DOWN, t=T_CAM):
    p = (np.asarray(P_w, float) - t) @ R
    uv = (K @ p.T).T
    return uv[:, :2] / uv[:, 2:3]


def render_rgb(bt, center_xy, yaw, K):
    img = np.full((H, W, 3), (90, 90, 90), np.uint8)
    if bt is None:
        return img
    uv = project(top_corners(bt, center_xy, yaw), K)
    cv2.fillPoly(img, [np.round(uv).astype(np.int32)], (40, 40, 235))
    return img


def render_depth(bt, center_xy, yaw, K, is_range=False, tilt=0.0):
    """같은 장면의 깊이 이미지 (32FC1 · m).

    박스 윗면 안이면 윗면까지, 밖이면 벨트까지의 거리를 넣는다.
    `tilt` 를 주면 윗면을 기울인다 (평면성 검사용).
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    d_cam = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)
    d_w = d_cam @ R_DOWN.T                        # 월드 방향

    def s_for(plane_z):
        return (plane_z - T_CAM[2]) / d_w[..., 2]

    s = s_for(BELT_M)                             # 기본은 벨트
    if bt is not None:
        z_top = BELT_M + bt["h_mm"] / 1000.0
        s_top = s_for(z_top)
        P = T_CAM.reshape(1, 1, 3) + s_top[..., None] * d_w
        # 윗면 사각형 안인가 — 박스 로컬 좌표로 돌려 판정한다
        c, sn = np.cos(np.radians(-yaw)), np.sin(np.radians(-yaw))
        dx = P[..., 0] - center_xy[0]
        dy = P[..., 1] - center_xy[1]
        lx, ly = dx * c - dy * sn, dx * sn + dy * c
        inside = ((np.abs(lx) <= bt["w_mm"] / 2000.0)
                  & (np.abs(ly) <= bt["d_mm"] / 2000.0))
        if tilt:
            # 윗면을 x 방향으로 기울인다. 평면이 아니면 NOT_FLAT 이 떠야 한다.
            s_top = s_for(z_top) * (1.0 + tilt * lx)
        s = np.where(inside, s_top, s)
    if is_range:
        s = s * np.linalg.norm(d_cam, axis=-1)
    return s.astype(np.float32)


def rgb_pose(bt, center_xy, yaw, K):
    """RGB 단독 경로 한 번. (det, out)."""
    img = render_rgb(bt, center_xy, yaw, K)
    det = find_box(img)
    assert det is not None
    return det, box_pose_from_corners(det["corners_px"], K, R_DOWN, T_CAM,
                                      BELT_M, TYPES)


CASES = [
    (TYPES[0], (9.80, 0.00), 0.0),
    (TYPES[1], (9.78, 0.12), 30.0),
    (TYPES[2], (9.82, -0.15), 90.0),
    (TYPES[1], (9.75, -0.10), 155.0),
]


# ── ① 깊이 단독으로 윗면 높이를 맞히는가 ────────────────────────────────────
@pytest.mark.parametrize("bt,center,yaw", CASES)
def test_top_plane_z(bt, center, yaw):
    K = make_K()
    det, _ = rgb_pose(bt, center, yaw, K)
    dimg = render_depth(bt, center, yaw, K)

    plane, reason = D.top_plane_z(dimg, det["corners_px"], K, R_DOWN, T_CAM, CFG_D)
    assert reason == "", f"깊이를 못 읽었다: {reason}"
    truth = BELT_M + bt["h_mm"] / 1000.0
    err_mm = abs(plane["z_m"] - truth) * 1000.0
    assert err_mm < 3.0, f"윗면 z 오차 {err_mm:.2f} mm"
    assert plane["spread_mm"] < 5.0, f"산포 {plane['spread_mm']:.2f} mm"


# ── ② 깊이 평면 위에서 자세·종류가 나오는가 ─────────────────────────────────
@pytest.mark.parametrize("bt,center,yaw", CASES)
def test_pose_from_plane(bt, center, yaw):
    K = make_K()
    det, _ = rgb_pose(bt, center, yaw, K)
    dimg = render_depth(bt, center, yaw, K)
    plane, _ = D.top_plane_z(dimg, det["corners_px"], K, R_DOWN, T_CAM, CFG_D)
    dep = D.pose_from_plane(det["corners_px"], K, R_DOWN, T_CAM, plane["z_m"],
                            TYPES, BELT_M)

    assert dep["type"]["name"] == bt["name"], \
        f'{bt["name"]} 을 {dep["type"]["name"]} 으로 봤다'
    dx = (dep["center_m"][0] - center[0]) * 1000.0
    dy = (dep["center_m"][1] - center[1]) * 1000.0
    assert abs(dx) < 8.0 and abs(dy) < 8.0, f"중심 오차 {dx:+.2f}, {dy:+.2f} mm"
    assert D.yaw_delta(dep["yaw_deg"], yaw) < 3.0


# ── ③ 두 경로가 같은 장면에서 일치하는가 (정상 케이스) ───────────────────────
@pytest.mark.parametrize("bt,center,yaw", CASES)
def test_agree_on_good_scene(bt, center, yaw):
    K = make_K()
    det, rgb = rgb_pose(bt, center, yaw, K)
    dimg = render_depth(bt, center, yaw, K)
    plane, _ = D.top_plane_z(dimg, det["corners_px"], K, R_DOWN, T_CAM, CFG_D)
    dep = D.pose_from_plane(det["corners_px"], K, R_DOWN, T_CAM, plane["z_m"],
                            TYPES, BELT_M)
    res = D.cross_check(rgb, dep, CFG_D)
    assert res["agree"], (f"멀쩡한 장면인데 갈렸다 [{res['verdict']}] "
                          f"dz {res['dz_mm']:+.1f} dxy {res['dxy_mm']:.1f} "
                          f"dyaw {res['dyaw_deg']:.1f}")
    assert res["used"] == "RGB"


# ── ④ 깊이가 없으면 **강등**이지 실패가 아니다 ──────────────────────────────
def test_no_depth_degrades():
    K = make_K()
    _det, rgb = rgb_pose(TYPES[1], (9.78, 0.12), 30.0, K)
    res = D.cross_check(rgb, None, CFG_D)
    assert res["used"] == "RGB", "깊이가 없다고 좌표를 막으면 라인이 선다"
    assert res["verdict"] == "NO_DEPTH"


def test_no_depth_image_reason():
    K = make_K()
    det, _ = rgb_pose(TYPES[0], (9.80, 0.0), 0.0, K)
    plane, reason = D.top_plane_z(None, det["corners_px"], K, R_DOWN, T_CAM, CFG_D)
    assert plane is None and reason == "NO_DEPTH_TOPIC"


# ── ⑤ ★ RGB 를 일부러 틀리게 만들고 깊이가 잡아내는가 ───────────────────────
@pytest.mark.parametrize("truth,wrong", [(TYPES[1], TYPES[2]), (TYPES[2], TYPES[0]),
                                         (TYPES[0], TYPES[1])])
def test_catches_wrong_type(truth, wrong):
    """RGB 가 종류를 한 칸 틀렸다고 하자 — 깊이는 같은 장면에서 진짜 높이를 잰다.

    ★ 이 상황이 실제로 어떻게 생기나: 윗면 모서리 검출이 몇 % 부풀거나(그림자·하이라이트)
      박스가 사양과 다르면 RGB 는 **가장 잘 맞는 다른 종류**로 스냅한다.
      스냅 자체는 에러를 안 내므로 지금까지는 아무도 몰랐다.
    """
    K = make_K()
    center, yaw = (9.78, 0.10), 20.0
    det, _ = rgb_pose(truth, center, yaw, K)
    dimg = render_depth(truth, center, yaw, K)      # 장면은 진짜 박스다

    # RGB 가 wrong 으로 봤다고 가정 — 그 평면에서 다시 계산한 결과를 만든다
    fake = box_pose_from_corners(det["corners_px"], K, R_DOWN, T_CAM, BELT_M, [wrong],
                                 tol_m=10.0)
    plane, _ = D.top_plane_z(dimg, det["corners_px"], K, R_DOWN, T_CAM, CFG_D)
    dep = D.pose_from_plane(det["corners_px"], K, R_DOWN, T_CAM, plane["z_m"],
                            TYPES, BELT_M)
    res = D.cross_check(fake, dep, CFG_D)

    assert not res["agree"], "종류가 틀렸는데 통과시켰다"
    assert res["verdict"] in ("HEIGHT_MISMATCH", "TYPE_MISMATCH"), res["verdict"]
    assert dep["type"]["name"] == truth["name"], "깊이도 같이 틀렸다 = 독립이 아니다"


# ── ⑥ 결측·이상치가 섞여도 중앙값이 버티는가 ────────────────────────────────
def test_outliers_and_holes():
    K = make_K()
    bt, center, yaw = TYPES[1], (9.78, 0.05), 15.0
    det, _ = rgb_pose(bt, center, yaw, K)
    dimg = render_depth(bt, center, yaw, K).copy()

    rng = np.random.default_rng(42)          # ★ 시드 고정 — 재현성 (CLAUDE.md §4)
    ys, xs = np.nonzero(np.ones_like(dimg, bool))
    pick = rng.choice(len(xs), size=len(xs) // 4, replace=False)
    dimg[ys[pick], xs[pick]] = 0.0           # 센서 결측
    pick2 = rng.choice(len(xs), size=len(xs) // 50, replace=False)
    dimg[ys[pick2], xs[pick2]] = np.nan      # NaN
    pick3 = rng.choice(len(xs), size=len(xs) // 50, replace=False)
    dimg[ys[pick3], xs[pick3]] = 40.0        # 튄 값

    plane, reason = D.top_plane_z(dimg, det["corners_px"], K, R_DOWN, T_CAM, CFG_D)
    assert reason == "", reason
    truth = BELT_M + bt["h_mm"] / 1000.0
    assert abs(plane["z_m"] - truth) * 1000.0 < 3.0, \
        f"이상치에 끌려갔다: {(plane['z_m'] - truth) * 1000:+.2f} mm"


# ── ⑦ 모서리 검출이 부풀어 벨트가 섞여도 버티는가 (shrink 의 존재 이유) ──────
def test_shrink_rejects_belt_leak():
    K = make_K()
    bt, center, yaw = TYPES[0], (9.80, 0.0), 0.0      # 3호 = 가장 낮다 = 가장 위험
    det, _ = rgb_pose(bt, center, yaw, K)
    dimg = render_depth(bt, center, yaw, K)
    fat = D.shrink_polygon(det["corners_px"], -0.12)   # 12 % 부풀린 모서리

    loose = dict(CFG_D, shrink_ratio=0.0)
    bad, _ = D.top_plane_z(dimg, fat, K, R_DOWN, T_CAM, loose)
    good, reason = D.top_plane_z(dimg, fat, K, R_DOWN, T_CAM, CFG_D)

    truth = BELT_M + bt["h_mm"] / 1000.0
    assert reason == "", reason
    err_good = abs(good["z_m"] - truth) * 1000.0
    err_bad = abs(bad["z_m"] - truth) * 1000.0
    # 중앙값 + 절대 게이트가 있으므로 **z 자체는 둘 다 맞는다.** 중앙값은 절반이
    # 오염되기 전까지 안 흔들리고, 게이트가 벨트 픽셀을 잘라내기 때문이다.
    assert err_good < 3.0, f"수축을 해도 벨트에 끌렸다 {err_good:.2f} mm"
    assert err_bad < 3.0, f"게이트가 벨트를 못 잘랐다 {err_bad:.2f} mm"
    # 대신 **얼마나 헛것을 봤는지**가 valid_ratio 에 남는다. 이 숫자가 안 갈리면
    # 수축 파라미터는 장식이고, 진단 토픽의 valid_ratio 도 아무 뜻이 없다.
    assert good["valid_ratio"] > bad["valid_ratio"] + 0.10, \
        f"수축 전후 유효비율이 같다 ({bad['valid_ratio']:.2f} vs {good['valid_ratio']:.2f})"
    assert good["valid_ratio"] > 0.95, f"수축 후에도 벨트가 섞였다 {good['valid_ratio']:.2f}"


# ── ⑧ 평면이 아니면 NOT_FLAT ────────────────────────────────────────────────
def test_not_flat():
    K = make_K()
    bt, center, yaw = TYPES[2], (9.80, 0.0), 0.0
    det, _ = rgb_pose(bt, center, yaw, K)
    # tilt 0.25 = 480 mm 면 위에서 약 95 mm 단차. 박스 윗면이 이렇게 기울 리 없다.
    dimg = render_depth(bt, center, yaw, K, tilt=0.25)
    plane, reason = D.top_plane_z(dimg, det["corners_px"], K, R_DOWN, T_CAM, CFG_D)
    assert reason == "NOT_FLAT", \
        f"기울어진 면을 통과시켰다 (산포 {plane['spread_mm']:.2f} mm)"

    # 반대로 **평평하면 안 울려야 한다.** 안 그러면 이 검사는 켤 수가 없다.
    flat = render_depth(bt, center, yaw, K)
    _p2, r2 = D.top_plane_z(flat, det["corners_px"], K, R_DOWN, T_CAM, CFG_D)
    assert r2 == "", f"평평한 면에 NOT_FLAT 이 떴다: {r2}"


# ── ⑨ 광축거리/시선거리를 헷갈리면 교차검증이 잡아내는가 ────────────────────
def test_range_vs_zdepth_confusion_is_caught():
    """`is_range` 를 틀리게 두면 **화면 가장자리에서만** 틀린다.

    ★ 이게 이 채널의 값이다. 설정 하나를 잘못 둬도 조용히 넘어가지 않는다.
      가장자리 박스로 재야 보인다 — 중앙에서는 두 규약이 같다.
    """
    K = make_K()
    bt, center, yaw = TYPES[0], (9.30, 0.28), 0.0      # 화면 구석
    det, _ = rgb_pose(bt, center, yaw, K)
    dimg = render_depth(bt, center, yaw, K, is_range=True)   # 씬은 시선거리로 낸다

    # 설정은 광축거리라고 믿고 읽는다 = 틀린 설정
    plane, _ = D.top_plane_z(dimg, det["corners_px"], K, R_DOWN, T_CAM,
                             dict(CFG_D, is_range=False))
    truth = BELT_M + bt["h_mm"] / 1000.0
    off_mm = abs(plane["z_m"] - truth) * 1000.0
    assert off_mm > CFG_D["z_tol_mm"], \
        f"규약을 틀렸는데 {off_mm:.1f} mm 밖에 안 어긋난다 — 검사가 못 잡는다"

    # 설정을 맞추면 돌아온다
    ok, _ = D.top_plane_z(dimg, det["corners_px"], K, R_DOWN, T_CAM,
                          dict(CFG_D, is_range=True))
    assert abs(ok["z_m"] - truth) * 1000.0 < 3.0


# ── ⑩ 설정 3층이 실제로 겹치는가 ────────────────────────────────────────────
def test_cfg_layers():
    base = D.merged_cfg(None)
    assert base["mode"] == "crosscheck"
    y = D.merged_cfg({"depth": {"mode": "prefer", "z_tol_mm": 10.0}})
    assert y["mode"] == "prefer" and y["z_tol_mm"] == 10.0
    r = D.merged_cfg({"depth": {"mode": "prefer"}}, {"mode": "off", "z_tol_mm": None})
    assert r["mode"] == "off", "런타임 오버라이드가 yaml 을 못 이긴다"
    assert r["z_tol_mm"] == base["z_tol_mm"], "None 오버라이드가 값을 지웠다"


def test_yaw_delta_folds():
    assert D.yaw_delta(1.0, 179.0) < 3.0        # 180 도 대칭
    assert D.yaw_delta(0.0, 90.0) < 1e-9        # 긴변/짧은변 뒤바뀜
    assert D.yaw_delta(0.0, 20.0) == pytest.approx(20.0)


# ── ⑪ ★ 이득의 정량 — RGB 가 포기한 자리를 깊이가 살리는가 ──────────────────
@pytest.mark.parametrize("infl", [0.08, 0.10, 0.12, 0.15])
def test_depth_rescues_what_rgb_gives_up(infl):
    """윗면 모서리 검출이 부풀면 RGB 는 **사양 3종 어느 것과도 안 맞다고 포기한다.**

    ★ 실측(이 테스트): 8 % 부풀림에서 RGB 는 이미 포기한다 (허용 오차 60 mm 를 넘는다).
      포기 = `BoxAtPickup` 이 안 나간다 = **그 박스는 통째로 버려진다.** 에러는 안 난다.
      같은 장면에서 깊이는 윗면 z 를 직접 재므로 평면이 확정되고, 중심이 0.5 mm 안에
      들어온다. 이게 깊이를 붙이는 이유다 — 확인만 하는 게 아니라 살리기도 한다.

    ★ 종류 판정에 높이 무게(h_weight)가 왜 필요한지도 여기서 드러난다.
      무게가 1.0 이면 15 % 부풀림에서 깊이도 4호를 5호로 본다 — 치수가 RGB 와 같은
      모서리에서 나오기 때문이다. 3.0 이면 잰 높이가 이긴다.
    """
    K = make_K()
    bt, center, yaw = TYPES[1], (9.55, 0.22), 20.0      # 4호 · 화면 한쪽
    det, _ = rgb_pose(bt, center, yaw, K)
    dimg = render_depth(bt, center, yaw, K)
    fat = D.shrink_polygon(det["corners_px"], -infl)     # 모서리가 부풀어 검출됐다

    rgb = box_pose_from_corners(fat, K, R_DOWN, T_CAM, BELT_M, TYPES)
    assert rgb is None, f"RGB 가 {infl:.0%} 부풀림을 통과시켰다 — 전제가 바뀌었다"

    plane, reason = D.top_plane_z(dimg, fat, K, R_DOWN, T_CAM, CFG_D)
    assert reason == "", f"깊이도 못 읽었다: {reason}"
    dep = D.pose_from_plane(fat, K, R_DOWN, T_CAM, plane["z_m"], TYPES, BELT_M,
                            CFG_D["h_weight"])

    err_mm = np.hypot(dep["center_m"][0] - center[0],
                      dep["center_m"][1] - center[1]) * 1000.0
    assert err_mm < 5.0, f"깊이 중심 오차 {err_mm:.2f} mm"
    assert dep["type"]["name"] == bt["name"], \
        f'깊이가 {bt["name"]} 을 {dep["type"]["name"]} 으로 봤다 (h_weight 확인)'

    # mode:prefer 면 이 자세가 그대로 채택된다 — RGB 단독이면 못 냈을 좌표다
    res = D.cross_check(rgb, dep, dict(CFG_D, mode="prefer"))
    assert res["used"] == "DEPTH"
    assert D.as_rgb_shape(dep)["type"]["name"] == bt["name"]


def test_h_weight_1_would_mistype():
    """위 테스트가 지키는 것이 진짜 지켜지는지 — 무게를 1.0 으로 낮추면 틀려야 한다.

    ★ 이걸 안 재면 h_weight 3.0 이 그냥 통과하는 값인지 실제로 일하는 값인지 모른다.
    """
    K = make_K()
    bt, center, yaw = TYPES[1], (9.55, 0.22), 20.0
    det, _ = rgb_pose(bt, center, yaw, K)
    dimg = render_depth(bt, center, yaw, K)
    fat = D.shrink_polygon(det["corners_px"], -0.15)
    plane, _ = D.top_plane_z(dimg, fat, K, R_DOWN, T_CAM, CFG_D)

    naive = D.pose_from_plane(fat, K, R_DOWN, T_CAM, plane["z_m"], TYPES, BELT_M, 1.0)
    weighted = D.pose_from_plane(fat, K, R_DOWN, T_CAM, plane["z_m"], TYPES, BELT_M, 3.0)
    assert naive["type"]["name"] != bt["name"], "무게 1.0 이 이미 맞힌다 — 이 값은 장식이다"
    assert weighted["type"]["name"] == bt["name"]
