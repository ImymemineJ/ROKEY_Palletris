"""운송장번호 규칙 — **한 곳에만 둔다.**

## 왜 공용인가

이 규칙을 쓰는 곳이 셋이다:
    · `packing_db.gen_master_db`  정답지(boxes_master)를 만들 때
    · `scripts/gen_qr_labels`     QR 그림을 구울 때
    · `packing_vision.label`      카메라가 읽은 값을 검사할 때
셋 중 하나만 바뀌면 **씬이 붙인 QR 과 DB 가 아는 번호가 갈린다** — 등록이 통째로
"master 에 없는 박스" 로 거부되는데, 원인이 형식 불일치라는 건 안 보인다.
그래서 복사하지 않고 여기서 import 한다 (CLAUDE.md §12).

## 규칙

    BOX001 ~ BOX150      (BOX + 3자리, **1 부터**)

★ 0 이 아니라 1 부터다. 운송장번호는 사람이 읽는 번호다.
  코드에서 인덱스로 만들 때는 `box_id(i)` 를 쓴다 — `FMT % i` 를 직접 쓰면
  한 칸씩 어긋난다.
"""

FMT = "BOX%03d"
START = 1


def box_id(index):
    """0 부터 세는 인덱스 -> 운송장번호. box_id(0) == "BOX001"."""
    return FMT % (int(index) + START)


def index_of(bid):
    """운송장번호 -> 0 부터 세는 인덱스. 형식이 아니면 None."""
    s = str(bid or "").strip()
    if not s.startswith("BOX") or not s[3:].isdigit():
        return None
    return int(s[3:]) - START
