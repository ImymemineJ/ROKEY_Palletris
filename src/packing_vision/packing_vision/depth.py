"""깊이 교차검증 코어 — ROS 없이 도는 순수 기하. `detect.py`·`geometry.py` 와 같은 층.

★ 무엇을 교차검증하는가
  RGB 경로(`geometry.box_pose_from_corners`)는 박스 **높이를 추측한다.**
  사양 3종의 평면에 각각 투영해 치수가 가장 잘 맞는 것을 고르는 방식이다.
  잘 도는 방법이지만 가정이 두 개 깔려 있다 — (1) 박스는 반드시 사양 3종 중
  하나다, (2) 윗면 네 모서리 검출이 정확하다. 둘 중 하나가 깨지면 종류를 틀리고,
  **종류를 틀리면 투영 평면이 60~130 mm 어긋나 중심 x, y 까지 같이 틀어진다.**
  광선이 다른 높이에서 끊기기 때문이고, 화면 가장자리일수록 오차가 커진다.

  깊이는 그 높이를 **직접 잰다.** 사양 표를 안 보고, 모서리 검출에도 안 기댄다
  (윗면 안쪽 픽셀만 쓴다). 그래서 두 경로가 **독립**이고, 독립이라야 교차검증이
  성립한다. 같은 가정을 공유하는 두 계산은 나란히 틀린다.

★ 이 모듈이 하는 일 세 가지
    1. `top_plane_z`   윗면 안쪽 깊이 픽셀 → 월드 z (중앙값 · 이상치 제거)
    2. `pose_from_plane`  그 평면에 모서리를 재투영 → 중심·yaw·치수·종류
    3. `cross_check`   두 결과를 대조해 판정 하나로 접는다

★ 왜 임계값이 여기 DEFAULTS 에 있나
  정본은 `system.yaml` 의 `recognition.depth` 다 (CLAUDE.md §4 "코드에 숫자를 박지
  않는다"). 다만 `system.yaml` 은 공용 파일이라 승인 없이 못 고친다 (§11).
  그래서 **키가 있으면 그쪽이 이긴다** — `merged_cfg()` 가 yaml 을 위에 겹친다.
  팀이 승인해서 아래 블록을 그대로 붙이면 코드는 한 줄도 안 바뀐다:

      recognition:
        depth:
          enabled: true
          mode: crosscheck        # off | crosscheck | rescue | prefer
          topic_suffix: depth
          is_range: false
          shrink_ratio: 0.30
          min_valid_ratio: 0.35
          min_px: 60
          range_mm: [200.0, 6000.0]
          plane_gate_mm: 60.0
          mad_k: 3.0
          h_weight: 3.0
          max_spread_mm: 25.0
          z_tol_mm: 45.0
          xy_tol_mm: 25.0
          yaw_tol_deg: 8.0
          max_stamp_skew_s: 0.02
          reject_on_disagree: false
"""
import cv2
import numpy as np

from packing_vision.geometry import pixels_to_plane, rect_from_corners

# ── 기본값. 정본은 yaml 의 recognition.depth 다 (위 주석 참조) ────────────────
DEFAULTS = {
    # 끄면 이 모듈이 통째로 비켜선다. RGB 단독 동작이 **한 줄도 안 바뀐다** —
    # 교차검증을 붙였다가 라인이 서면 안 되므로 되돌릴 스위치를 먼저 만든다.
    "enabled": True,
    # off        : 깊이를 안 쓴다 (enabled:false 와 같다. 런치에서 끄기 편하라고 둘 다 둔다)
    # crosscheck : RGB 가 정하고 깊이는 확인만 한다 ← 기본
    # rescue     : crosscheck 와 같되, **RGB 가 포기했을 때만** 깊이가 대신 낸다
    # prefer     : 깊이가 있으면 언제나 깊이가 정한다. RGB 는 확인 쪽으로 돈다
    #
    # ★★ 왜 기본이 prefer 가 아닌가 (2026-08-26 결정. 뒤집으려면 아래 조건을 먼저 볼 것)
    #   1. **아직 진짜 깊이로 재 본 적이 없다.** 지금까지의 검증은 정답에서 만든
    #      합성 깊이라 두 경로가 당연히 일치한다. "깊이가 더 정확하다" 는 아직
    #      기대이지 실측이 아니다. 이 저장소는 데스크 지표와 Isaac 실측이 **부호까지**
    #      어긋난 적이 있다 (CLAUDE.md §10-1).
    #   2. **계통 오차에 무방비다.** tf 가 조금 틀렸거나 `is_range` 가 반대면 모든
    #      박스가 같은 방향으로 어긋난다. crosscheck 는 그걸 불일치로 **드러내지만**,
    #      prefer 는 그 값을 그대로 **따라간다.**
    #   3. ★ **재현성이 깨진다.** prefer 는 깊이 프레임이 도착한 박스만 깊이를 쓴다.
    #      실측 도착률이 8 개 중 5~7 개라, 같은 시드로 돌려도 박스마다 어느 경로가
    #      쓰였는지가 네트워크 타이밍에 좌우된다. 평가 항목 "재현성(Play/Stop)" 과
    #      정면으로 부딪친다 (§13).
    #
    #   → prefer 로 승격할 조건 3 개:
    #      (a) 씬이 깊이를 실제로 발행한다 (트랙 A)
    #      (b) 진짜 Isaac 깊이로 crosscheck 를 돌려 불일치 기록이 충분히 쌓인다
    #      (c) 깊이 도착률이 100 % 에 가깝다 (rgb 와 같은 render product 면 자연히 된다)
    #      **그 기록을 만들어 주는 것이 지금의 crosscheck 다.** 지금 prefer 로 넘기면
    #      대조군이 사라져 승격 근거 자체를 못 만든다.
    #
    #   ★ `rescue` 는 그 사이에 위험 없이 이득만 가져가는 자리다.
    #     RGB 가 답을 내는 한 결과가 **항상 같으므로 재현성이 안 깨지고**, 평소에는
    #     깊이가 좌표를 안 정하므로 계통 오차도 안 탄다. 대신 RGB 가 포기해서
    #     **어차피 버려질 박스**만 깊이가 건진다 — 손해 볼 것이 없는 교환이다.
    #     실측: 윗면 모서리가 8 % 부풀면 RGB 는 포기하는데(사양 3종 허용 오차 밖),
    #     깊이는 같은 장면에서 중심을 5 mm 안으로 잡는다.
    "mode": "crosscheck",
    "topic_suffix": "depth",      # <ns>/depth. setup_camera_graph.py 가 내는 이름
    # ★ Isaac ROS2CameraHelper 의 type:"depth" 는 **광축 방향 거리**(distance_to_
    #   image_plane)를 32FC1 · m 로 낸다. ROS 표준(depth_image_proc)과 같은 규약이다.
    #   type:"depth_pcl" 계열의 시선거리(range)를 물리면 여기를 true 로 바꾼다.
    #   ⚠ 이걸 틀리면 화면 중앙은 맞고 **가장자리만** 틀리는 종류의 버그다 —
    #     교차검증이 바로 잡아낸다 (dz 가 가장자리에서만 커진다). 그게 이 채널의 값이다.
    "is_range": False,
    # 윗면 폴리곤을 중심 쪽으로 얼마나 줄여서 볼 것인가 (0.30 = 30 % 축소).
    # ★ 줄이지 않으면 모서리 픽셀이 **벨트를 본다.** 박스 옆으로 한 픽셀만 나가도
    #   깊이가 200~340 mm 뛰므로 중앙값이 통째로 끌려간다.
    "shrink_ratio": 0.30,
    "min_valid_ratio": 0.35,      # 폴리곤 안에서 쓸 수 있는 픽셀 비율의 하한
    "min_px": 60,                 # 그 절대 개수 하한. 비율만 보면 작은 박스가 통과한다
    "range_mm": [200.0, 6000.0],  # 이 밖의 깊이는 센서 결측으로 본다 (0 · inf · NaN 포함)
    # ★ 중앙값에서 이만큼 벗어난 픽셀은 **윗면이 아니다**. 절대 기준이다.
    #   왜 절대값이 필요한가: 합성 평면이나 잡음이 아주 적은 깊이에서는 MAD 가
    #   **정확히 0** 이 된다 (절반 이상이 같은 값). 그러면 MAD 배수 기준이
    #   0 배가 되어 이상치 제거가 통째로 건너뛰어지고, 벨트 픽셀이 30 % 섞여
    #   있어도 산포가 0.00 으로 나온다 — "완벽히 평평하다" 는 거짓 보고다.
    #   값의 근거: 가장 낮은 박스(3호)도 벨트에서 210 mm 위다. 윗면의 실제
    #   기울기·잡음은 수십 mm 를 넘지 않는다. 60 은 그 사이에 있다.
    "plane_gate_mm": 60.0,
    "mad_k": 3.0,                 # 그 안에서 다시 중앙값절대편차 몇 배까지 남길 것인가
    "max_spread_mm": 25.0,        # 이 이상 흩어지면 평면이 아니다 → NOT_FLAT
    # ★ 종류를 고를 때 **높이에 몇 배의 무게**를 줄 것인가. 1.0 이면 치수와 동등.
    #   왜 높이가 더 무거워야 하는가: 높이는 깊이가 **직접 잰 값**(실측 ±3 mm)이고,
    #   치수는 RGB 와 **같은 모서리**에서 나온다. 같은 무게로 두면 모서리가 틀렸을 때
    #   깊이도 같이 틀려서 **독립성이 희석된다** — 그러면 교차검증이 아무것도 못 잡는다.
    #   실측: 모서리를 15 % 부풀리면 무게 1.0 에서 4호를 5호로 봤다. 3.0 에서는 안 그런다.
    #   사양 높이 간격이 60~70 mm 라 잰 값 ±3 mm 로 이미 종류가 갈린다.
    "h_weight": 3.0,
    "z_tol_mm": 45.0,             # 높이 일치 허용. 사양 간 최소 간격 70 mm 의 절반보다 작게
    "xy_tol_mm": 25.0,
    "yaw_tol_deg": 8.0,
    # ★ rgb 와 depth 의 stamp 차이 상한. **한 프레임 안**을 뜻하는 값이어야 한다.
    #   같은 렌더 프로덕트에서 나온 rgb/depth 는 stamp 가 **정확히 같다**
    #   (ROS2CameraHelper 두 개가 한 render product 를 공유한다).
    #   0.20 으로 뒀다가 실측에서 깨졌다 — 깊이가 한 틱 늦게 도착해도 0.2 초 안이라,
    #   박스가 아직 도착하지 않았던 **직전 프레임의 벨트 깊이**를 현재 RGB 와
    #   짝지었다. 결과는 dz +280 mm(= 박스 높이만큼)의 가짜 불일치다.
    #   짝이 안 맞으면 그 프레임은 건너뛰는 게 맞다 — 다음 프레임에 또 온다.
    "max_stamp_skew_s": 0.02,
    # 어긋났을 때 좌표를 아예 안 낼 것인가. 기본은 **낸다** — 데모가 서는 쪽이
    # 더 나쁘다. 어긋남은 토픽(VisionCheck)과 경고로 남고, 켤지는 팀이 정한다.
    "reject_on_disagree": False,
}


def merged_cfg(recognition_cfg=None, overrides=None):
    """DEFAULTS ← yaml(recognition.depth) ← 런타임 오버라이드(ROS 파라미터) 순으로 겹친다.

    ★ 세 층인 이유: 기본값은 코드가, 정본은 yaml 이, 현장 조정은 런치가 갖는다.
      런치에서 `-p depth.mode:=off` 한 줄로 되돌릴 수 있어야 통합 중에 안 막힌다.
    """
    out = dict(DEFAULTS)
    for layer in ((recognition_cfg or {}).get("depth"), overrides):
        for k, v in (layer or {}).items():
            if v is not None:
                out[k] = v
    return out


def shrink_polygon(corners_px, ratio):
    """폴리곤을 무게중심 쪽으로 당긴다. ratio 0.30 이면 변 길이가 70 % 가 된다."""
    p = np.asarray(corners_px, float)
    c = p.mean(axis=0)
    return c + (p - c) * (1.0 - float(ratio))


def deproject(uv, depth_vals, K, R, t, is_range=False):
    """픽셀 + 깊이 → 월드 점 (N,3).

    광학 규약(x 오른쪽 · y 아래 · z 앞)에서 z=1 로 정규화한 광선에 깊이를 곱한다.
    `is_range` 면 광선을 단위벡터로 만든 뒤 곱한다 — 광축 거리와 시선 거리는
    화면 중앙에서만 같고 가장자리로 갈수록 갈린다 (1280x720 · HFOV 60 도 에서 모서리 15 %).
    """
    uv = np.asarray(uv, float).reshape(-1, 2)
    z = np.asarray(depth_vals, float).reshape(-1)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = np.stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy, np.ones(len(uv))], axis=1)
    if is_range:
        d = d / np.linalg.norm(d, axis=1, keepdims=True)
    return np.asarray(t, float).reshape(1, 3) + (d * z.reshape(-1, 1)) @ np.asarray(R, float).T


def top_plane_z(depth_img, corners_px, K, R, t, cfg):
    """윗면 안쪽 깊이 픽셀 → 월드 z. 못 재면 (None, 사유).

    반환: ({"z_m", "spread_mm", "px_used", "valid_ratio"}, "") 또는 (None, "사유")

    ★ 중앙값 + MAD 로 자른다. 평균이 아니다.
      깊이 이미지에는 결측(0 · inf · NaN)과 모서리 튐이 섞인다. 평균은 그런 값
      **하나**에 통째로 끌려가고, 그렇게 틀린 z 는 크게 틀려서 종류를 두 칸 건너뛴다.
      중앙값은 절반이 오염되기 전까지 안 흔들린다.
    """
    if depth_img is None:
        return None, "NO_DEPTH_TOPIC"
    dimg = np.asarray(depth_img)
    if dimg.ndim != 2 or dimg.size == 0:
        return None, "SHAPE_MISMATCH"

    h, w = dimg.shape
    poly = shrink_polygon(corners_px, cfg["shrink_ratio"])
    mask = np.zeros((h, w), np.uint8)
    cv2.fillPoly(mask, [np.round(poly).astype(np.int32)], 255)
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None, "TOO_FEW_PIXELS"

    vals = dimg[ys, xs].astype(float)
    lo_mm, hi_mm = cfg["range_mm"]
    good = np.isfinite(vals) & (vals * 1000.0 >= float(lo_mm)) & (vals * 1000.0 <= float(hi_mm))
    # 여기 valid_ratio 는 **센서 유효성**만 본다. 윗면 여부는 아래 게이트가 가른다.
    if good.sum() < int(cfg["min_px"]):
        return None, "TOO_FEW_PIXELS"
    if float(good.sum()) / float(xs.size) < float(cfg["min_valid_ratio"]):
        return None, "OUT_OF_RANGE"

    P = deproject(np.stack([xs[good], ys[good]], axis=1), vals[good], K, R, t,
                  bool(cfg["is_range"]))
    z = P[:, 2]
    med = float(np.median(z))

    # ① 절대 게이트 — 중앙값에서 plane_gate_mm 밖은 윗면이 아니다 (벨트·이웃 박스).
    #    MAD 배수보다 **먼저** 건다. MAD 가 0 으로 퇴화해도 이건 항상 작동한다.
    keep = np.abs(z - med) <= float(cfg["plane_gate_mm"]) / 1000.0
    if keep.sum() < int(cfg["min_px"]):
        return None, "TOO_FEW_PIXELS"
    z = z[keep]
    med = float(np.median(z))

    # ② 그 안에서 잡음 이상치 한 번 더. 1.4826 은 MAD 를 표준편차로 환산하는 상수다.
    sigma = 1.4826 * float(np.median(np.abs(z - med)))
    if sigma > 0.0:
        keep2 = np.abs(z - med) <= float(cfg["mad_k"]) * sigma
        if keep2.sum() >= int(cfg["min_px"]):
            z = z[keep2]
            med = float(np.median(z))
            sigma = 1.4826 * float(np.median(np.abs(z - med)))

    # ★ valid_ratio 는 "폴리곤 안에서 **윗면으로 인정된** 비율" 이다.
    #   센서 결측과 벨트 혼입을 숫자 하나로 담는다 — 낮으면 둘 중 하나이고,
    #   어느 쪽이든 그 프레임의 깊이를 덜 믿어야 한다는 뜻은 같다.
    spread_mm = sigma * 1000.0
    out = {"z_m": med, "spread_mm": spread_mm, "px_used": int(z.size),
           "valid_ratio": float(z.size) / float(xs.size)}
    if spread_mm > float(cfg["max_spread_mm"]):
        # 평면이 아니다. 박스가 아니거나 두 물체를 한 덩어리로 잡았다.
        # ★ 사유만 돌려주고 값도 같이 준다 — 진단 토픽에 실어야 원인을 본다.
        return out, "NOT_FLAT"
    return out, ""


def pose_from_plane(corners_px, K, R, t, z_top_m, types, belt_top_m,
                    h_weight=None):
    """깊이가 정한 평면에 모서리를 재투영 → 중심·yaw·치수·종류.

    ★ 여기서는 후보 평면을 훑지 않는다. 평면이 **이미 실측으로 정해졌기** 때문이다.
      RGB 경로의 닭-달걀(높이를 모르면 크기를 못 재고, 크기를 모르면 종류를 못 고른다)
      이 깊이 한 방에 끊긴다. 종류는 그 뒤에 치수와 높이로 고르기만 하면 된다.
    """
    if h_weight is None:
        h_weight = DEFAULTS["h_weight"]
    pts = pixels_to_plane(corners_px, K, R, t, z_top_m)
    center_xy, long_m, short_m, yaw = rect_from_corners(pts)
    h_m = z_top_m - belt_top_m

    # ★★ 벨트 게이트 (2026-08-27 정정인 실측). 박스가 화면에 걸치는 프레임에서
    #   RGB 실루엣이 벨트를 덮으면 깊이는 **벨트면**을 잰다 — h 가 -16~-9 mm 로
    #   나오는데도 아래 최근접 스냅이 그걸 3호로 분류했고, prefer 가 채택해
    #   4·5호가 전부 3호로 등록됐다 (한 판에서 103건). 그 등록 높이로 로봇이
    #   내려가다 진짜 윗면에 얹혀 IK_FAIL 3연발 -> 다음 박스 포기가 났다.
    #   사양에서 가장 낮은 박스(3호 210)의 절반도 안 되는 높이는 박스가 아니다.
    if h_m < min(bt["h_mm"] for bt in types) / 2000.0:
        return None

    best = None
    for bt in types:
        w, d = bt["w_mm"] / 1000.0, bt["d_mm"] / 1000.0
        size_err = abs(long_m - max(w, d)) + abs(short_m - min(w, d))
        h_err = abs(h_m - bt["h_mm"] / 1000.0)
        # ★ 치수와 높이를 **같이** 보되 높이를 무겁게 둔다 (h_weight 주석 참조).
        #   치수만 보면 RGB 경로와 같은 실수를 반복하고, 높이만 보면 사양에 없는
        #   물건이 지나가도 가장 가까운 높이로 스냅해 버린다. 무게로 가른다.
        score = size_err + float(h_weight) * h_err
        if best is None or score < best["score"]:
            best = {"type": bt, "score": score, "size_err_m": size_err, "h_err_m": h_err}

    return {"center_m": (float(center_xy[0]), float(center_xy[1]), float(z_top_m)),
            "yaw_deg": yaw, "type": best["type"], "h_m": h_m,
            "measured_m": (long_m, short_m),
            "size_err_m": best["size_err_m"], "h_err_m": best["h_err_m"]}


def yaw_delta(a, b):
    """두 yaw 의 차이를 0~90 으로 접는다.

    사각형은 180 도 대칭이고, 게다가 정사각형에 가까운 박스는 긴변/짧은변이
    바뀌며 90 도 튄다. 접지 않으면 멀쩡한 일치가 172 도 불일치로 보인다.
    """
    d = abs(float(a) - float(b)) % 180.0
    d = min(d, 180.0 - d)
    return min(d, abs(90.0 - d)) if d > 45.0 else d


def cross_check(rgb, dep, cfg):
    """RGB 결과와 깊이 결과를 대조해 판정 하나로 접는다.

    rgb : geometry.box_pose_from_corners 의 반환 (또는 None)
    dep : pose_from_plane 의 반환 (또는 None)
    반환: dict(agree, verdict, used, dz_mm, dxy_mm, dyaw_deg)

    ★ 판정 순서에 뜻이 있다. 높이 → 종류 → 중심 → yaw 순으로 본다.
      **위쪽이 아래쪽의 원인**이기 때문이다. 높이가 어긋나면 중심도 자동으로
      어긋나는데, 중심 불일치를 먼저 보고하면 진짜 원인이 가려진다.
    """
    if rgb is None and dep is None:
        return {"agree": False, "verdict": "NO_DEPTH", "used": "NONE",
                "dz_mm": 0.0, "dxy_mm": 0.0, "dyaw_deg": 0.0}
    if dep is None:
        # 깊이가 없다 = 지금까지의 동작 그대로. 실패가 아니라 **강등**이다.
        return {"agree": False, "verdict": "NO_DEPTH", "used": "RGB",
                "dz_mm": 0.0, "dxy_mm": 0.0, "dyaw_deg": 0.0}
    if rgb is None:
        # ★ RGB 가 사양 3종과 안 맞다고 **포기한** 경우다. 지금까지는 여기서 끝이었다 —
        #   좌표가 안 나가고 그 박스는 통째로 버려졌다. 에러도 안 났다.
        #   깊이는 평면을 실측했으므로 이쪽만으로도 자세가 나온다.
        #   `rescue`·`prefer` 가 그걸 채택한다. RGB 단독보다 강한 유일한 지점이다.
        return {"agree": False, "verdict": "TYPE_MISMATCH",
                "used": "DEPTH" if cfg["mode"] in ("rescue", "prefer") else "NONE",
                "dz_mm": 0.0, "dxy_mm": 0.0, "dyaw_deg": 0.0}

    rz = rgb["center_m"][2] + rgb["type"]["h_mm"] / 2000.0    # 중심 → 윗면
    dz_mm = (rz - dep["center_m"][2]) * 1000.0
    dxy_mm = float(np.hypot(rgb["center_m"][0] - dep["center_m"][0],
                            rgb["center_m"][1] - dep["center_m"][1])) * 1000.0
    dyaw = yaw_delta(rgb["yaw_deg"], dep["yaw_deg"])

    verdict = "OK"
    if abs(dz_mm) > float(cfg["z_tol_mm"]):
        verdict = "HEIGHT_MISMATCH"
    elif rgb["type"]["name"] != dep["type"]["name"]:
        verdict = "TYPE_MISMATCH"
    elif dxy_mm > float(cfg["xy_tol_mm"]):
        verdict = "XY_MISMATCH"
    elif dyaw > float(cfg["yaw_tol_deg"]):
        verdict = "YAW_MISMATCH"

    agree = verdict == "OK"
    # ★ 여기(= RGB 가 답을 낸 경우) rescue 는 crosscheck 와 **한 글자도 다르지 않다.**
    #   그게 재현성이 안 깨지는 근거다 — 깊이가 왔든 안 왔든 좌표가 같다.
    if cfg["mode"] == "prefer":
        used = "DEPTH"
    elif agree or not cfg["reject_on_disagree"]:
        used = "RGB"
    else:
        used = "NONE"
    return {"agree": agree, "verdict": verdict, "used": used,
            "dz_mm": float(dz_mm), "dxy_mm": float(dxy_mm), "dyaw_deg": float(dyaw)}


def as_rgb_shape(dep):
    """깊이 결과를 RGB 경로와 **같은 모양**으로 바꾼다 (mode: prefer 에서 쓴다).

    ★ 호출부가 두 경로를 분기하지 않게 하려는 것이다. 노드가
      `if 깊이: ... else: ...` 로 갈리면 한쪽 경로만 고쳐지는 버그가 생긴다.
      모양을 맞춰 두면 채택만 바뀌고 그 뒤 코드는 하나다.

    ★ 중심 z 는 `윗면 − 사양높이/2` 다. RGB 경로처럼 `벨트 + 높이/2` 로 두지 않는다 —
      깊이는 윗면을 실측했으므로, 박스가 벨트에 딱 붙어 있지 않아도(이음매에서 튀거나
      다른 박스 위에 얹혔거나) 실제 자리를 따라간다. 그게 실측을 쓰는 이유다.
    """
    x, y, z_top = dep["center_m"]
    h_spec = dep["type"]["h_mm"] / 1000.0
    return {"center_m": (x, y, z_top - h_spec / 2.0),
            "yaw_deg": dep["yaw_deg"], "type": dep["type"],
            "measured_m": dep["measured_m"], "fit_err_m": dep["size_err_m"]}
