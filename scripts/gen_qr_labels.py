#!/usr/bin/env python3
"""master DB 를 읽어 박스별 QR 라벨 PNG 를 굽는다.

    python3 scripts/gen_qr_labels.py                 # 기본 경로로
    python3 scripts/gen_qr_labels.py --out /tmp/qr   # 다른 곳에

## 왜 Isaac 밖에서 미리 굽는가

Isaac Sim 의 내장 파이썬(3.11)에 cv2 가 있다는 보장이 없다. 씬 안에서 QR 을
만들려다 실패하면 **박스에 라벨이 없는 채로 데모가 돈다** — 인식이 통째로 죽는다.
PNG 로 먼저 구워 두면 spawner 는 텍스처를 읽어 붙이기만 하면 되고, 그건 USD
기본 기능이라 의존성이 없다.

## ★ QR 에는 운송장번호만 들어간다

구역(A/B/C)은 QR 에 없다. 실제 송장 바코드가 그렇다 — 번호만 있고 주소는
사람이 읽는 글자다. 분류기가 번호로 주소를 조회해 담당 구역을 정한다.
우리도 `db_service_node._zone_by_address` 가 같은 순서로 한다:

    운송장번호 -> boxes_master.address -> 동 -> zone_map -> 구역

그래서 여기서는 `boxes_master.box_id` 만 읽으면 된다. `zone` 은 라벨에 안 들어가고,
분포를 보여 주려고 출력에만 쓴다. DB 는 **읽기 전용**으로 연다.
"""
import argparse
import pathlib
import sqlite3
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src" / "packing_vision"))

import cv2  # noqa: E402

from packing_vision.label import (  # noqa: E402
    LABEL_MM, MODULE_MM, MODULES, QR_MM, QUIET_MODULES, make_payload, render_qr,
)

DEF_DB = pathlib.Path.home() / "palletris_ws" / "palletris.db"
DEF_OUT = pathlib.Path(__file__).resolve().parents[1] / "qr_labels"
# 굽는 해상도. 실제 크기는 씬이 정하고(198 mm), 여기서는 픽셀만 넉넉히 준다.
# 모듈당 16 px 이면 라벨 한 변 528 px — 텍스처로 충분하고 파일도 작다.
MODULE_PX = 16


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", type=pathlib.Path, default=DEF_DB)
    ap.add_argument("--out", type=pathlib.Path, default=DEF_OUT)
    ap.add_argument("--module-px", type=int, default=MODULE_PX)
    a = ap.parse_args()

    if not a.db.exists():
        sys.exit(f"master DB 가 없다: {a.db}\n"
                 f"  먼저: python3 src/packing_db/packing_db/gen_master_db.py")

    # ★ 읽기 전용으로 연다. db_service_node 가 유일 writer 라는 규약을 이 도구가
    #   깨면 'database is locked' 가 데모 중에 터진다 (CLAUDE.md §10).
    con = sqlite3.connect(f"file:{a.db}?mode=ro", uri=True)
    rows = con.execute("SELECT box_id, zone, address FROM boxes_master "
                       "ORDER BY delivery_seq").fetchall()
    con.close()
    if not rows:
        sys.exit("boxes_master 가 비어 있다")

    a.out.mkdir(parents=True, exist_ok=True)
    for box_id, _zone, _addr in rows:
        img = render_qr(make_payload(box_id), module_px=a.module_px)
        cv2.imwrite(str(a.out / f"{box_id}.png"), img)

    side = (MODULES + 2 * QUIET_MODULES) * a.module_px
    print(f"라벨 {len(rows)} 장 -> {a.out}")
    print(f"  페이로드   {make_payload(rows[0][0])!r} — 운송장번호만 "
          f"({MODULES}x{MODULES} 모듈)")
    print(f"  이미지     {side}x{side} px")
    print(f"  실제 크기  QR {QR_MM} mm + 여백 {QUIET_MODULES * MODULE_MM} mm "
          f"-> 라벨 한 변 {LABEL_MM} mm")
    zc = {}
    for _b, z, _a in rows:
        zc[z] = zc.get(z, 0) + 1
    print(f"  주소 예시  {rows[0][2]!r} -> 구역 {rows[0][1]}  (QR 에는 안 들어간다)")
    print(f"  구역 분포  {dict(sorted(zc.items()))}")


if __name__ == "__main__":
    main()
