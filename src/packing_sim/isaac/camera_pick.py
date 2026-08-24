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
  · `get_pointcloud` 는 렌더 결과라 **렌더 틱**이 필요하다. `isaac-sim.sh` 로
    띄워야 한다 — `python.sh` standalone 은 벨트와 같은 이유로 안 맞는다.
  · 소켓 스레드에서 부르면 안 된다. 다른 핸들러와 같이 `MainQueue` 를 거친다.

## 안 하는 것

RGB 로 종류를 분류하지 않는다. stage 1 은 라인이 하나라 분류할 곳이 없고,
박스는 전부 같은 색이다 (`spawner.LINE_COLOR` 주석 참고). 치수는 depth 로
재므로 3·4·5호 구분은 그 치수로 붙는다.
"""
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


class PickCamera:
    def __init__(self, stage, cfg, mainq, prim_path=CAM_PRIM, log=print):
        self.stage, self.mainq, self._log = stage, mainq, log
        self.path = prim_path
        self.cam = None

        conv = cfg["conveyor"]
        self.belt_top_m = conv["belt_top_z_mm"] / 1000.0
        self.stopper_x_m = conv["stopper_x_mm"] / 1000.0
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

    # ── 본체. **메인 스레드 전용** ──────────────────────────
    def detect(self):
        pts = self.cam.get_pointcloud(world_frame=True)
        if pts is None or len(pts) == 0:
            return {"ok": False, "error": "포인트클라우드가 비었다", "n_pts": 0}
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)

        # ── ROI ────────────────────────────────────────────
        # ★ z 천장이 핵심이다. 이게 없으면 시야를 가로지르는 **로봇 팔**이 가장
        #   높은 점군이 되어 팔을 박스로 인식한다 (perception.py 가 같은 경고를 적어 뒀다).
        z0 = self.belt_top_m + PLANE_TOL_M
        z1 = self.belt_top_m + self.h_max_m + PLANE_TOL_M
        x0 = self.stopper_x_m - self.w_max_m - 0.05
        m = ((pts[:, 2] > z0) & (pts[:, 2] < z1)
             & (pts[:, 0] > x0) & (pts[:, 0] < self.stopper_x_m + 0.02))
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
