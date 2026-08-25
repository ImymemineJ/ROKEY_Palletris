"""픽셀 → 월드 좌표. ROS 없이 도는 순수 기하 코어.

★ 왜 카메라 위치를 코드에 안 박는가
  `system.yaml` 의 `recognition.pick_cam` 에는 y·z 만 있고 **x_mm 이 없다.**
  진행보고에는 씬 프림이 (9795, 0, 2300) 이라 적혀 있지만 yaml 에 없는 숫자를
  코드에 박는 것은 금지다 (CLAUDE.md §11). 그래서 카메라 자세는 **tf** 에서,
  내부 파라미터는 **camera_info** 에서 받는다. 둘 다 표준 경로라 tf 프레임 이름
  규약(§15 미결)이 정해져도 코드가 안 바뀐다 — 프레임 이름은 이미지 헤더의
  frame_id 를 그대로 쓰기 때문이다.

★ 좌표 규약
  camera_info 를 내는 카메라 프레임은 ROS 광학 규약이다 — x 오른쪽, y 아래, z 앞.
  Isaac 의 ROS2CameraHelper 도 이 규약으로 낸다.
"""
import numpy as np


def quat_to_R(x, y, z, w):
    """쿼터니언 → 3x3 회전행렬."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def pixels_to_plane(uv, K, R, t, plane_z):
    """픽셀들을 월드 평면 z=plane_z 위로 투영한다.

    uv       : (N,2) 픽셀
    K        : 3x3 내부 파라미터 (camera_info.k)
    R, t     : 월드 ← 카메라(광학) 변환
    plane_z  : 월드 z (m)
    반환     : (N,3) 월드 점. 광선이 평면과 안 만나면 ValueError.
    """
    uv = np.asarray(uv, float).reshape(-1, 2)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    # 광학 프레임 광선
    d_cam = np.stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy,
                      np.ones(len(uv))], axis=1)
    d_w = d_cam @ R.T                      # 월드 방향
    dz = d_w[:, 2]
    if np.any(np.abs(dz) < 1e-9):
        raise ValueError("광선이 평면과 평행하다 — 카메라 자세를 확인할 것")
    s = (plane_z - t[2]) / dz
    if np.any(s <= 0):
        # 평면이 카메라 뒤에 있다. 대개 tf 방향(월드←카메라)이 뒤집힌 경우다.
        raise ValueError("평면이 카메라 뒤에 있다 — tf 방향을 확인할 것")
    return t.reshape(1, 3) + s.reshape(-1, 1) * d_w


def rect_from_corners(pts_w):
    """월드 위 4점 → (중심xy, 긴변, 짧은변, yaw_deg).

    yaw 는 **긴 변**의 방향이다. 0~180 으로 접는다 — 사각형은 180° 대칭이라
    yaw 와 yaw+180 이 같은 자세인데, 접지 않으면 같은 박스가 프레임마다
    0°/180° 를 오가며 로봇이 손목을 반 바퀴씩 헛돌린다.
    """
    p = np.asarray(pts_w, float)[:, :2]
    e = [np.linalg.norm(p[(i + 1) % 4] - p[i]) for i in range(4)]
    i_long = int(np.argmax(e[:2]))                    # 인접한 두 변만 보면 된다
    v = p[(i_long + 1) % 4] - p[i_long]
    long_mm, short_mm = max(e[0], e[1]), min(e[0], e[1])
    yaw = float(np.degrees(np.arctan2(v[1], v[0])) % 180.0)
    return p.mean(axis=0), long_mm, short_mm, yaw


def box_pose_from_corners(corners_px, K, R, t, belt_top_m, types, tol_m=0.030):
    """박스 윗면 네 모서리(픽셀) → 월드 박스 **중심** + yaw + 종류.

    ★ 닭-달걀 문제: 높이를 모르면 투영 평면을 못 정하고, 평면을 못 정하면 크기를
      못 재고, 크기를 모르면 종류(=높이)를 못 고른다.

      반복으로 풀지 않는다. 벨트 상면에 한 번 투영해 크기를 재는 방식은 높은 박스일수록
      크게 부풀어(5호는 1.29 배) 종류 매칭이 통째로 빗나간다.

      대신 **후보마다 자기 높이의 평면에서 평가한다.** 종류가 맞으면 실측이
      사양 치수와 거의 정확히 일치하고, 틀리면 크게 어긋난다. 실측 분리도:
      4호 박스를 3호/4호/5호 로 놓고 재면 오차가 171 / ~0 / 175 mm 로 갈린다.
      한 번에 정해지고 수렴을 기다릴 필요도 없다.
    """
    best = None
    for bt in types:
        h_m = bt["h_mm"] / 1000.0
        try:
            pts = pixels_to_plane(corners_px, K, R, t, belt_top_m + h_m)
        except ValueError:
            continue
        center_xy, long_m, short_m, yaw = rect_from_corners(pts)
        w, d = bt["w_mm"] / 1000.0, bt["d_mm"] / 1000.0
        err = abs(long_m - max(w, d)) + abs(short_m - min(w, d))
        if best is None or err < best["fit_err_m"]:
            best = {"center_m": (float(center_xy[0]), float(center_xy[1]),
                                 float(belt_top_m + h_m / 2.0)),   # ★ 박스 "중심"
                    "yaw_deg": yaw, "type": bt,
                    "measured_m": (long_m, short_m), "fit_err_m": float(err)}

    if best is None:
        return None
    if best["fit_err_m"] > tol_m * 2:
        # 사양 3종 어느 것과도 안 맞는다. 억지로 고르면 엉뚱한 높이로 집으러 간다.
        return None
    return best
