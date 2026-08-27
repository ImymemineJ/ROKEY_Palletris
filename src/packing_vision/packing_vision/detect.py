"""박스 검출 — ROS 없이 도는 순수 CV 코어.

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : ②④ 인식의 **코어** — 상자 윤곽·yaw·QR 판독
    ║     불러 주는 쪽        : vision_node
    ║     이 파일이 부르는 쪽 : (없음 — OpenCV 만. ROS 의존성 0)
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

왜 분리했나: Isaac 씬 USD 가 git 이 아니라 `sync_scene.sh` 로 배포돼서 이 PC 에는
없다. ROS 노드 안에 CV 를 섞어 두면 씬이 올 때까지 한 줄도 검증할 수 없다.
합성 이미지로 여기까지는 지금 검증한다 (트랙 C 가 heightmap_core 를 분리한 것과 같은 이유).

stage 1 의 박스는 **전부 같은 빨강**이다 (`spawner.LINE_COLOR = (1.0, 0.15, 0.15)`).
라인이 하나라 색으로 나눠 보낼 곳이 없기 때문이다. 그래서 종류 판별은 색이 아니라
**치수**로 한다. 색은 stage 2 에서 존 분류로 되살아난다.
"""
import cv2
import numpy as np

# 빨강은 HSV 색상환의 0 과 180 양쪽에 걸친다. 한 구간만 잡으면 절반을 놓친다.
RED_RANGES = (((0, 90, 60), (10, 255, 255)),
              ((170, 90, 60), (180, 255, 255)))


def red_mask(bgr):
    """②④ 빨간 상자 윗면만 남기는 HSV 마스크.

    ★ **HSV 로 간다.** BGR 임계값은 조명이 조금만 바뀌어도 무너진다.
    ★ 빨강은 색상환(Hue)의 **양 끝에 걸쳐 있어** 구간을 둘로 나눠 OR 한다.
      한 구간만 쓰면 절반을 놓친다.
    ★ open → close 로 잡음 점과 구멍을 정리한다. 안 하면 `findContours` 가
      먼지 한 점을 상자로 잡는다.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = None
    for lo, hi in RED_RANGES:
        part = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = part if m is None else cv2.bitwise_or(m, part)
    # 열기로 점노이즈를 털고, 닫기로 박스 윗면의 하이라이트 구멍을 메운다.
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)


def find_box(bgr, min_area_px=400, roi=None):
    """가장 큰 빨강 덩어리의 최소면적 사각형.

    반환: dict(center_px, corners_px(4x2), size_px, angle_deg, area_px) 또는 None.
    roi 를 주면 (x0, y0, x1, y1) 밖은 안 본다 — 컨베이어 밖의 빨강을 배제한다.
    """
    if bgr is None or bgr.size == 0:
        return None
    m = red_mask(bgr)
    if roi:
        x0, y0, x1, y1 = roi
        keep = np.zeros_like(m)
        keep[max(0, y0):y1, max(0, x0):x1] = 255
        m = cv2.bitwise_and(m, keep)

    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    if area < min_area_px:
        return None

    rect = cv2.minAreaRect(c)             # ((cx,cy), (w,h), angle)
    (cx, cy), (rw, rh), ang = rect

    # ★ 화면 가장자리에 닿았는가 = 박스가 시야 밖으로 잘렸는가.
    #   잘린 사각형의 중심은 진짜 중심이 아니다. 실측: 5호를 y 방향으로 13 mm
    #   잘리게 두면 중심이 7.7 mm 어긋나고 치수가 13 mm 작게 나온다.
    #   그대로 발행하면 로봇이 그만큼 빗나간 자리를 집는다 — 조용히 틀리는 쪽이라
    #   여기서 표시해 두고, 호출부가 "안 보인다" 로 처리하게 한다.
    #
    #   ⚠ 이건 가상의 걱정이 아니다. pick_cam 은 지금 넓은 축(1280 px)이 벨트
    #     진행(X)을, 좁은 축(720 px, VFOV 36°)이 지터 방향(Y)을 덮는다.
    #     5호(480) 가 90° 돌아 지터 최대(±150)에 오면 필요 ±390 mm 인데
    #     커버가 ±377 mm 다 — 13 mm 모자란다. 카메라를 광축 기준 90° 돌리면
    #     커버가 ±670 mm 이 되어 280 mm 여유가 생긴다 (트랙 A 확인 필요).
    h_img, w_img = m.shape[:2]
    xs, ys = c[:, 0, 0], c[:, 0, 1]
    clipped = bool(xs.min() <= 0 or ys.min() <= 0
                   or xs.max() >= w_img - 1 or ys.max() >= h_img - 1)

    return {"center_px": (float(cx), float(cy)),
            "corners_px": cv2.boxPoints(rect).astype(float),
            "size_px": (float(rw), float(rh)),
            "angle_deg": float(ang),
            "area_px": area,
            "clipped": clipped}


def read_qr(bgr, corners_px=None):
    """QR 이 보이면 문자열, 아니면 "".

    ★ corners_px(박스 윗면 네 모서리)를 주면 **먼저 그 면을 정사각으로 펴서** 읽는다.

      왜 필요한가: 하향 카메라라도 박스가 화면 중앙에서 벗어나면 윗면이
      사다리꼴로 보인다. 그 상태로 그냥 디코더에 넣으면 자세에 따라 **결정적으로**
      실패한다 — 무작위가 아니라서 같은 자리에 선 박스는 몇 번을 다시 봐도 계속
      실패한다. 실측(하네스 투영): 5호가 y +39 mm · yaw −12도 에서 라벨이
      207x207 px(모듈 6.7 px)로 충분히 큰데도 판독이 0 % 였다.
      같은 그림을 펴서 넣으면 읽힌다.

      이게 QR 을 키우는 것보다 낫다 — 카메라 위치가 바뀌어도 그대로 통한다.

    ★ 펴서 실패하면 원본으로 한 번 더 본다. 모서리 추정이 틀렸을 수 있어서다.
    """
    def _decode(img):
        """QR 한 장 해독. 실패하면 빈 문자열 — **예외를 밖으로 내보내지 않는다.**

        ★ 해독 실패는 정상 상황이다(자세가 나쁘면 못 읽는다). 여기서 예외가
          올라가면 인식 콜백이 통째로 죽는데, rclpy 가 그 예외를 삼켜서
          **에러 한 줄 없이 상자가 영영 안 온다.**
        """
        try:
            data, _pts, _ = cv2.QRCodeDetector().detectAndDecode(img)
            return data or ""
        except cv2.error:
            return ""

    if corners_px is not None:
        try:
            src = _order_corners(np.asarray(corners_px, np.float32))
            side = int(max(np.linalg.norm(src[0] - src[1]),
                           np.linalg.norm(src[1] - src[2])))
            # ★ 2 배로 키워 편다. 원래 크기로 펴면 보간에서 모듈 경계가 뭉개진다.
            side = max(64, min(1024, side * 2))
            dst = np.array([[0, 0], [side, 0], [side, side], [0, side]], np.float32)
            flat = cv2.warpPerspective(bgr, cv2.getPerspectiveTransform(src, dst),
                                       (side, side))
            got = _decode(flat)
            if got:
                return got
        except (cv2.error, ValueError):
            pass
    return _decode(bgr)


def _order_corners(pts):
    """네 점을 좌상 → 우상 → 우하 → 좌하 순으로. 순서가 섞이면 편 그림이 뒤집힌다."""
    c = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    return pts[np.argsort(ang)].astype(np.float32)


def dominant_color_name(bgr, mask=None):
    """덩어리의 대표 색 이름. stage 2 존 분류에서 되살아난다.

    이름은 spawner 의 stage 2 색 주석과 맞춰 둔다 — 빨강/초록/파랑 세 가지.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if mask is None:
        mask = red_mask(bgr)
    if not mask.any():
        return ""
    h = float(np.median(hsv[:, :, 0][mask > 0]))
    if h < 15 or h >= 170:
        return "red"
    if 35 <= h < 85:
        return "green"
    if 85 <= h < 140:
        return "blue"
    return ""
