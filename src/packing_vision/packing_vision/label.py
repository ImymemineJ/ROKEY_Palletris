"""QR 라벨 — 페이로드 규약 · 생성 · 파싱. ROS 도 Isaac 도 없이 도는 순수 코어.

## 왜 별도 파일인가

QR 을 **만드는 쪽**(씬에 붙일 PNG 생성)과 **읽는 쪽**(인식 노드)이 같은 규약을
써야 한다. 규약을 양쪽에 복사하면 한쪽만 고쳐졌을 때 씬은 멀쩡한데 인식만 조용히
실패한다 — 에러도 안 난다. 그래서 규약을 여기 한 곳에 두고 양쪽이 import 한다.

## 페이로드 규약 — **운송장번호 하나뿐이다**

    "<운송장번호>"      예: "BOX001"

- `boxes_master.box_id` 와 **같은 값**이다 (`BOX%03d`, BOX001 부터)
- **구역(A/B/C)은 QR 에 넣지 않는다.** 실제 송장 바코드가 그렇다 — 번호만 있고,
  주소는 사람이 읽는 글자로 인쇄돼 있다. 분류기는 번호로 주소를 조회한다.
  우리도 같다: `db_service_node._zone_by_address` 가
  운송장번호 -> `boxes_master.address` -> 동 -> `zone_map` -> 구역 순으로 정한다.
  (구역 배정은 **배송지 기준** — 설계결정서 §5-3 확정, 작업명세서 미결 #9)
- 접두어("PALLETRIS|" 같은 것)를 붙이지 않는다. 글자가 늘면 QR 모듈이 25 -> 29 로
  커지고, 같은 실제 크기에서 모듈당 픽셀이 14% 줄어 해독 여유가 그만큼 깎인다.

## ★ 운송장번호를 QR 에 넣는 이유 — 번호 밀림을 막는다

QR 이 없으면 인식 노드가 박스를 **세어서** id 를 만들 수밖에 없다
(`BOX%03d % n`). 그러면 cam1 이 한 개만 놓쳐도 그 뒤 전부가 한 칸씩 밀려
DB 의 다른 박스 기록을 덮어쓴다. 에러 없이 전부 틀리는 쪽이다.
`recognition.cam1_box_miss_rate` 가 0 이 아닌 이상 이건 가정이 아니라 예정된 일이다.

## ★ 크기 규약 — 모듈당 5 px 이상 (실측 근거)

25x25 모듈 QR 을 pick_cam 스케일(1.16 mm/px)로 합성해 서브픽셀 정렬을 무작위로
흔들며(±0.5 px, yaw ±15도) 24 회씩 해독한 결과:

    QR 실제   모듈당 px   깨끗   블러5px   노이즈 σ8
     110 mm      3.8      100%     12%       58%
     130 mm      4.5       96%     96%       29%
    ★150 mm      5.2      100%    100%       79%
     190 mm      6.6      100%    100%       83%

110 mm 은 **깨끗한 이미지에서만** 100% 다. 한 픽셀 흐려지면 12% 로 무너진다.
"한 장은 읽히더라" 로 크기를 정하면 씬에서 간헐 실패가 나고, 그게 제일 잡기 어렵다.
그래서 5.2 px/모듈 = **모듈 6 mm** 를 정본으로 잡는다.
"""
import numpy as np

try:                                   # cv2 는 읽기·생성 양쪽에 필요하다
    import cv2
except ImportError:                    # 문서 생성 등 cv2 없는 환경 방어
    cv2 = None

DESTS = ("A", "B", "C")                # 구역. QR 에는 안 들어간다 (위 규약 참조)
# ★ 운송장번호 규칙은 packing_common.ids 가 정본이다 — 복사하지 않는다.
#   DB 를 만드는 쪽 · QR 을 굽는 쪽 · 읽는 쪽 셋이 같아야 한다 (그 파일 주석 참조).
from packing_common.ids import FMT as BOX_ID_FMT   # noqa: E402  "BOX%03d"
from packing_common.ids import START as BOX_ID_START
from packing_common.ids import box_id  # noqa: F401  — 재수출

MODULES = 25                           # "BOX001" 6 글자 -> version 2 (25x25)
#   ★ 8 글자("BOX00000")에서 6 글자로 줄여도 모듈 수는 25 그대로다 —
#     cv2 인코더가 21 모듈(version 1)로 내려가지 않는다. 라벨 크기도 그대로다.

# ── 크기는 system.yaml 이 정본이다 (CLAUDE.md §4 "코드에 숫자를 박지 않는다") ──
# ★ 못 읽어도 죽지 않는다. 이 모듈은 Isaac 내장 파이썬에서도 import 될 수 있고,
#   거기서는 packing_common 이 경로에 없을 수 있다. 그때는 아래 실측 기본값을
#   쓰되 _FROM_YAML 로 표시해 둔다 — 테스트가 둘의 일치를 검사한다.
_DEF = {"module_mm": 7, "quiet_modules": 3, "label_mm": 217}
try:
    from packing_common.config import load as _load
    _QR = dict(_DEF, **(_load()["recognition"].get("qr") or {}))
    _FROM_YAML = True
except Exception:                      # noqa: BLE001 — yaml 도 ROS 도 없는 환경
    _QR = dict(_DEF)
    _FROM_YAML = False

MODULE_MM = int(_QR["module_mm"])       # 모듈 한 변. 위 실측표 근거
QUIET_MODULES = int(_QR["quiet_modules"])   # QR 표준 여백. 아래 QUIET 주석 참조
QR_MM = MODULE_MM * MODULES             # 150
LABEL_MM = MODULE_MM * (MODULES + 2 * QUIET_MODULES)   # 198 — 흰 여백 포함 전체

MIN_MODULE_PX = 6.0                    # 이보다 작으면 비스듬한 자세에서 무너진다
#   ★ 5.0 이 아니다. 평면에서는 5.2 px 로 100 % 였지만, pick_cam 이 실제로
#     보는 **기울어진** 라벨에서는 3호가 94 % 로 떨어졌다 (yaml qr 주석 참조).


def make_payload(box_id):
    """"BOX001" -> "BOX001". 규약이 바뀌면 packing_common.ids 한 곳만 고친다."""
    box_id = str(box_id).strip()
    if not box_id or any(c.isspace() for c in box_id):
        raise ValueError(f"운송장번호가 비었거나 공백이 들어 있다: {box_id!r}")
    return box_id


def parse_payload(s):
    """QR 원문 -> 운송장번호. 규약에 안 맞으면 None.

    ★ None 을 예외 대신 돌려주는 이유: 이 함수를 부르는 곳이 구독 콜백이다.
      rclpy 는 콜백 예외를 삼키므로, 여기서 raise 하면 QR 이 한 번 깨졌을 때
      "박스가 영원히 안 온다. 근데 에러는 없다" 가 된다 (CLAUDE.md §9).

    ★ 옛 규약("B|BOX00000")도 거른다. 구분자가 보이면 씬의 라벨이 옛 판이라는
      뜻이라 조용히 앞부분을 버리면 안 된다 — 라벨을 다시 구워야 한다.
    """
    if not s:
        return None
    v = str(s).strip()
    if not v or any(c.isspace() for c in v) or "|" in v:
        return None
    return v


def render_qr(payload, module_px=8, quiet_modules=QUIET_MODULES):
    """페이로드 -> 흑백(0/255) 정사각 이미지. 흰 여백 포함.

    ★ `cv2.QRCodeEncoder.encode` 는 **이미 0/255 uint8** 을 돌려준다.
      0/1 인 줄 알고 255 를 곱하면 uint8 이 넘쳐 무늬가 뭉개지고, 그 뒤로
      아무것도 해독이 안 된다 (실제로 한 번 걸렸다).

    ★ 확대는 반드시 INTER_NEAREST. 기본 보간을 쓰면 모듈 경계가 회색으로
      번져 작은 QR 에서 해독률이 떨어진다.
    """
    if cv2 is None:
        raise RuntimeError("render_qr 에는 cv2 가 필요하다")
    q = cv2.QRCodeEncoder.create().encode(payload)
    n = q.shape[0]
    big = cv2.resize(q, (n * module_px, n * module_px),
                     interpolation=cv2.INTER_NEAREST)
    # ★ 흰 여백(quiet zone). 박스 윗면이 **빨강**이라 여백이 없으면 디텍터가
    #   QR 바깥 경계를 못 잡는다. 종이 라벨을 붙인 것과 같은 모양이 된다.
    pad = quiet_modules * module_px
    out = np.full((big.shape[0] + 2 * pad, big.shape[1] + 2 * pad), 255, np.uint8)
    out[pad:pad + big.shape[0], pad:pad + big.shape[1]] = big
    return out


def modules_for(payload):
    """이 페이로드가 몇 x 몇 모듈이 되는지. 크기 계산 검산용."""
    if cv2 is None:
        raise RuntimeError("modules_for 에는 cv2 가 필요하다")
    return int(cv2.QRCodeEncoder.create().encode(payload).shape[0])


def required_qr_mm(mm_per_px, modules=MODULES, min_module_px=MIN_MODULE_PX):
    """이 스케일의 카메라가 읽으려면 QR 이 실제로 몇 mm 여야 하나."""
    return modules * min_module_px * float(mm_per_px)
