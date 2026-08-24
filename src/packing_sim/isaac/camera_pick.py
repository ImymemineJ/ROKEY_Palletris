"""픽업 하향 카메라 — depth 로 박스 좌표를 뽑는다 (프로토콜 cmd `camera_detect`).

## 무엇을 하는가

집는 자리 위 2.3 m 에 달린 카메라가 아래를 본다. 매 호출마다

    depth -> 월드 포인트클라우드 -> 벨트면 제거 -> 윗면만 남기기
          -> 최소면적 사각형 피팅 -> 중심 · 치수 · yaw · 윗면 높이

를 돌려준다. 좌표는 **월드 mm**, 흡착 지점은 윗면 중심이다.

## 왜 이렇게

`spawner.pickup_check` 는 씬의 프림에서 자세를 그대로 읽는다 (ground truth).
실물에는 그런 게 없다 — 카메라가 본 것이 전부다. 이 모듈은 **같은 질문에
카메라로 답한다.** 둘을 나란히 찍으면 인식 오차가 그대로 보인다
(`compare` 필드). 사양 §5.5 가 ground truth 를 "검증용" 이라고 못박은 이유가
이것이다.

## 규약

  · 카메라 프림은 씬 빌더(`layout_to_spec_test1_1.py`)가 만든다. 여기서는 붙기만 한다.
  · `Camera.initialize()` 는 **타임라인이 돌기 시작한 뒤**에 불러야 한다.
    부팅 직후(setup)에서 부른다.
  · 깊이는 렌더 결과라 **렌더 틱**이 필요하다. `isaac-sim.sh` 로 띄워야 한다 —
    `python.sh` standalone 은 벨트와 같은 이유로 안 맞는다.
  · `Camera.get_pointcloud` 는 안 쓴다 (Isaac 안에서 NameError 로 죽는다).
    `_pointcloud_world()` 가 깊이 + 내부 파라미터로 직접 투영한다.
  · 소켓 스레드에서 부르면 안 된다. 다른 핸들러와 같이 `MainQueue` 를 거친다.

## 안 하는 것

RGB 로 종류를 분류하지 않는다. stage 1 은 라인이 하나라 분류할 곳이 없고,
박스는 전부 같은 색이다 (`spawner.LINE_COLOR` 주석 참고). 치수는 depth 로
재므로 3·4·5호 구분은 그 치수로 붙는다.
"""
import os
import sys
from pathlib import Path

import numpy as np

_PS = str(Path(__file__).resolve().parent.parent)      # src/packing_sim
if _PS not in sys.path:
    sys.path.insert(0, _PS)

CAM_PRIM = "/World/CamPick/Camera"
PLANE_TOL_M = 0.020        # 벨트 상면에서 이만큼 위부터 "박스"
TOP_BAND_M = 0.030         # 가장 높은 점에서 이만큼 아래까지를 윗면으로 본다
MIN_PTS = 200              # 이보다 적으면 못 봤다고 한다
# ★ 깊이 영상을 이 간격으로 솎아 쓴다. 1280x720 을 전부 투영하면 92만 점이고,
#   실측 2026-08-24 로 한 번에 10.7만 점(ROI 통과분)이 나왔다 — 필요량(MIN_PTS 200)의
#   500배다. 그 계산이 **물리 스텝 안에서** 도는 탓에 MainQueue 예산(1.6 s)을 넘겨
#   3개 중 1개가 "메인 루프가 응답하지 않는다" 로 떨어졌다.
#   2 로 솎으면 점이 1/4 이 되고 정밀도는 그대로다 — 상자 윗면은 평면이라 점 밀도가
#   최소면적 사각형 피팅에 거의 영향을 안 준다.
DEPTH_STRIDE = int(os.environ.get("PL_CAM_STRIDE", "2"))


class PickCamera:
    def __init__(self, stage, cfg, mainq, prim_path=CAM_PRIM, log=print):
        self.stage, self.mainq, self._log = stage, mainq, log
        self.path = prim_path
        self.cam = None

        conv = cfg["conveyor"]
        self.belt_top_m = conv["belt_top_z_mm"] / 1000.0
        # ★ 픽업 축 — spawner.py 와 같은 규약이다 (2026-08-24). 기본 "x" 라
        #   이 키가 없는 기존 씬은 동작이 안 바뀐다.
        _ax = str(conv.get("pickup_axis", "x")).lower()
        if _ax not in ("x", "y"):
            raise ValueError("conveyor.pickup_axis 는 x | y 여야 한다: %r" % _ax)
        self.axis = 0 if _ax == "x" else 1
        self.stopper_pos_m = conv["stopper_x_mm"] / 1000.0
        bt = cfg["boxes"]["types"]
        self.h_max_m = max(t["h_mm"] for t in bt) / 1000.0
        self.w_max_m = max(t["w_mm"] for t in bt) / 1000.0
        self.types = {t["name"]: t for t in bt}
        rec = cfg.get("recognition", {})
        self.res = tuple(int(v) for v in rec.get("resolution", [1280, 720]))

    # ── 수명 ────────────────────────────────────────────────
    def start(self):
        """**타임라인이 돌기 시작한 뒤** 메인 스레드에서 부른다."""
        if not self.stage.GetPrimAtPath(self.path).IsValid():
            self._log("[camera] !! %s 가 씬에 없다 — run_layout_to_spec.sh 를 "
                      "먼저 돌려라. 카메라 없이 진행한다." % self.path)
            return False
        from isaacsim.sensors.camera import Camera
        self.cam = Camera(prim_path=self.path, resolution=self.res)
        self.cam.initialize()
        self.cam.add_distance_to_image_plane_to_frame()
        pos = self.cam.get_world_pose()[0]
        self._log("[camera] %s  %dx%d  위치 %s mm  벨트면까지 %.0f mm"
                  % (self.path, self.res[0], self.res[1],
                     [round(float(v) * 1000) for v in pos],
                     (float(pos[2]) - self.belt_top_m) * 1000.0))
        return True

    def handlers(self):
        return {"camera_detect": self.h_detect}

    # ── 프로토콜 ────────────────────────────────────────────
    def h_detect(self, req=None):
        if self.cam is None:
            return {"ok": False, "error": "카메라가 없다"}
        return self.mainq.run(self.detect)

    # ── 깊이 -> 월드 점군. **메인 스레드 전용** ─────────────
    def _pointcloud_world(self):
        """깊이 영상과 내부 파라미터로 월드 점군을 직접 만든다. 못 만들면 None.

        ★ `Camera.get_pointcloud(world_frame=True)` 를 **쓰지 않는다.** Isaac 안에서
              NameError: name 'LOADER_DIR' is not defined
          로 죽는다 — 우리 코드에는 그 이름이 없다. 포인트클라우드 annotator 쪽
          경로 문제라 우리가 고칠 수 있는 자리가 아니다. 그래서 같은 계산을 여기서
          직접 한다 (`perception.CameraPerception._pointcloud_world` 와 같은 식).
          쓰는 API 는 전부 이미 돌고 있는 것들이다 — `start()` 가
          `add_distance_to_image_plane_to_frame()` 을 이미 불러 두어 깊이는 나온다.

        축 규약이 두 번 꺾인다. depth 로 만든 점은 **광학 규약**(+X 오른쪽 ·
        +Y 아래 · +Z 전방)인데 USD 카메라는 **-Z 를 보고 +Y 가 위**다. 그래서
        광학 -> USD 로 한 번 맞추고, 카메라 월드 자세로 한 번 더 돌린다.
        """
        depth = self.cam.get_depth()
        if depth is None:
            return None
        depth = np.asarray(depth, dtype=np.float64)
        if depth.ndim != 2 or not np.isfinite(depth).any():
            return None

        K = np.asarray(self.cam.get_intrinsics_matrix(), dtype=np.float64)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

        if DEPTH_STRIDE > 1:
            depth = depth[::DEPTH_STRIDE, ::DEPTH_STRIDE]
        h, w = depth.shape
        # 솎아도 화소 좌표는 **원본 격자** 위에 있어야 내부 파라미터(cx, cy, fx, fy)와
        # 맞는다. 그래서 인덱스에 stride 를 곱해 되돌린다.
        uu, vv = np.meshgrid(np.arange(w) * DEPTH_STRIDE,
                             np.arange(h) * DEPTH_STRIDE)
        # 배경(무한대)·0·이상치를 뺀다. 하향 카메라라 바닥까지 4.5 m 쯤이다.
        valid = np.isfinite(depth) & (depth > 1e-3) & (depth < 1e3)
        if not valid.any():
            return None

        z = depth[valid]
        x = (uu[valid] - cx) / fx * z
        y = (vv[valid] - cy) / fy * z
        pts_cam = np.stack([x, y, z], axis=1)

        from perception import _quat_to_matrix
        cam_pos, cam_quat = self.cam.get_world_pose(camera_axes="usd")
        R = _quat_to_matrix(np.asarray(cam_quat, dtype=float))
        optical_to_usd = np.array([[1.0, 0.0, 0.0],
                                   [0.0, -1.0, 0.0],
                                   [0.0, 0.0, -1.0]])
        return pts_cam @ optical_to_usd.T @ R.T + np.asarray(cam_pos, dtype=float)

    # ── 본체. **메인 스레드 전용** ──────────────────────────
    def detect(self):
        pts = self._pointcloud_world()
        if pts is None or len(pts) == 0:
            return {"ok": False, "error": "깊이 영상에서 점을 못 만들었다", "n_pts": 0}
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)

        # ── ROI ────────────────────────────────────────────
        # ★ z 천장이 핵심이다. 이게 없으면 시야를 가로지르는 **로봇 팔**이 가장
        #   높은 점군이 되어 팔을 박스로 인식한다 (perception.py 가 같은 경고를 적어 뒀다).
        z0 = self.belt_top_m + PLANE_TOL_M
        z1 = self.belt_top_m + self.h_max_m + PLANE_TOL_M
        # ★ ROI 의 가로 방향도 픽업 축을 따른다. +X 라인이면 pts[:,0], +Y 지선이면
        #   pts[:,1] 을 잘라야 한다 — 안 그러면 엉뚱한 축으로 잘라 점군이 0이 된다.
        a = self.axis
        p0 = self.stopper_pos_m - self.w_max_m - 0.05
        m = ((pts[:, 2] > z0) & (pts[:, 2] < z1)
             & (pts[:, a] > p0) & (pts[:, a] < self.stopper_pos_m + 0.02))
        roi = pts[m]
        if len(roi) < MIN_PTS:
            return {"ok": False, "error": "박스가 안 보인다",
                    "n_pts": int(len(roi)), "present": False}

        # ── 윗면만 ─────────────────────────────────────────
        zmax = float(roi[:, 2].max())
        top = roi[roi[:, 2] > zmax - TOP_BAND_M]
        if len(top) < MIN_PTS:
            return {"ok": False, "error": "윗면 점이 모자라다",
                    "n_pts": int(len(top)), "present": False}

        from perception import min_area_rect, rect_fill_ratio
        center, extent, yaw = min_area_rect(top[:, :2])
        fill = float(rect_fill_ratio(top[:, :2], center, extent, yaw))
        h_m = zmax - self.belt_top_m

        # 치수로 종류를 붙인다 (색이 아니라). 가로/세로는 yaw 에 따라 뒤집힌다.
        a, b = sorted((float(extent[0]), float(extent[1])), reverse=True)
        best, err = None, 1e9
        for t in self.types.values():
            e = (abs(a - t["w_mm"] / 1000.0) + abs(b - t["d_mm"] / 1000.0)
                 + abs(h_m - t["h_mm"] / 1000.0))
            if e < err:
                best, err = t["name"], e

        return {"ok": True, "present": True,
                "top_center_mm": [round(float(center[0]) * 1000.0, 1),
                                  round(float(center[1]) * 1000.0, 1),
                                  round(zmax * 1000.0, 1)],
                "size_mm": [round(a * 1000.0, 1), round(b * 1000.0, 1),
                            round(h_m * 1000.0, 1)],
                "yaw_deg": round(float(yaw), 2),
                "type": best, "type_err_mm": round(err * 1000.0, 1),
                "fill": round(fill, 3), "n_pts": int(len(top))}
