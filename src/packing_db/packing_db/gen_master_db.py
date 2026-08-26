#!/usr/bin/env python3
"""boxes_master 생성 — 작업명세서 §8.3.

실행 전에 "정답지"를 미리 깔아 둔다. 실행 중 흘러간 박스(boxes_sorted)와 대조해야
인식률·누락을 잴 수 있다. 등록된 박스만 세면 인식률이 언제나 100 % 로 나온다
(명세서 §8.6 의 함정).

★ stage 1 에서 zone 을 편향 랜덤으로 뽑으면 안 된다.
  15 개 중 40 % 가 A/C 로 배정되면 B 관제탑이 없는 팔레트에 plan_placement 를
  호출하게 되고 게이트를 통과할 수 없다. stage 1 은 전부 "B" 다.
"""
import argparse
import os
import random
import sqlite3

from packing_common.config import load
from packing_common.ids import box_id as make_box_id
from packing_common.paths import default_db_path, find_data
from packing_db.delivery_points import APARTMENTS, address_of

# ★ 파일명은 palletris.db 다 (CLAUDE.md §10). 트랙 D 가이드에는 packing.db 로
#   적혀 있는데 정본은 CLAUDE.md 라 그쪽을 따른다. 경로 자체는 하드코딩하지 않고
#   _default_db_path() 가 config 위치에서 역산한다.

# ── 동(洞) → 구역 배정표 (작업명세서 미결 #9 "배송 주소 생성 규칙", 민범진) ──
#
# 구역 A/B/C 는 **배송지 기준**이다 (설계결정서 §5-3). 실제 택배 분류와 같은 순서다:
#     송장 -> 운송장번호 -> (조회) 주소 -> 동 -> 담당 구역
#
# ★ 주소에 "(B구역)" 같은 정답을 박아 넣지 않는다. 예전 판이 그랬는데, 그러면
#   주소를 읽는 것이 곧 정답을 읽는 것이라 배정 로직이 아무 일도 안 하게 된다.
#   동 이름만 주고 구역은 zone_map 이 정하게 해야 표를 바꿨을 때 실제로 달라진다.
ZONE_DONG = {
    # 담당 지역은 **서울 구로구** 한 곳이다. 구로구는 동서로 길쭉해서
    # 경인로를 따라 동 -> 중 -> 서 세 덩어리로 가르면 배송 동선이 안 겹친다.
    "A": [("서울 구로구", "신도림동"), ("서울 구로구", "구로동"),
          ("서울 구로구", "가리봉동")],                       # 동부
    "B": [("서울 구로구", "고척동"), ("서울 구로구", "개봉동")],   # 중부
    "C": [("서울 구로구", "오류동"), ("서울 구로구", "궁동"),
          ("서울 구로구", "온수동"), ("서울 구로구", "천왕동"),
          ("서울 구로구", "항동")],                            # 서부
}


# 구역별 배송지 후보. delivery_points 의 실제 아파트를 동 기준으로 갈라 담는다.
_BY_ZONE = {z: [(d, n, la, lo) for d, n, la, lo in APARTMENTS
                if d in {dong for _gu, dong in lst}]
            for z, lst in ZONE_DONG.items()}


#: 법정동 -> 구역. ZONE_DONG 을 뒤집은 것이다.
ZONE_OF = {d: z for z, lst in ZONE_DONG.items() for _gu, d in lst}


def _delivery_point(rng, zone):
    """배송지 하나를 뽑는다 -> (주소, 위도, 경도).

    ★ 번지를 랜덤으로 지어내지 않는다. 실존하는 아파트만 쓴다 —
      지어낸 주소는 지오코딩도 경로 계산도 안 되고, 발표에서 물으면 답이 없다.
    """
    d, n, la, lo = rng.choice(_BY_ZONE[zone])
    return address_of(d, n), la, lo


def dong_of(address):
    """주소에서 동을 뽑는다. 없으면 None.

    ★ 규칙은 하나뿐이다 — "동" 으로 끝나는 어절. 정규식으로 더 정교하게 만들
      이유가 없다. 주소를 만드는 쪽도 읽는 쪽도 이 파일 하나다.
    """
    for tok in str(address or "").split():
        if len(tok) > 1 and tok.endswith("동"):
            return tok
    return None


def build(cfg, db_path, schema_path):
    """boxes_master 를 채운다. 이미 있으면 지우고 다시 만든다 (재현성)."""
    conn = sqlite3.connect(db_path)
    with open(schema_path, encoding="utf-8") as f:
        conn.executescript(f.read())

    # ★ 스포너와 같은 시드를 쓴다. 다르면 "master 에 없는 박스" 만 나온다.
    rng = random.Random(cfg["boxes"]["random_seed"])

    if int(cfg["stage"]) == 1:
        n = int(cfg["boxes"]["stage1"]["total_count"])
        zones = [cfg["boxes"]["stage1"]["zone"]] * n        # ★ 전부 B. 랜덤 금지
        plan = None
    else:
        # ★ stage 2 는 박스 수를 랜덤으로 뽑지 않는다. **배송지 구조에서 나온다** —
        #   구역 3 x 아파트 5 x 세대 N. 랜덤으로 뽑으면 어떤 아파트는 박스가 0 개라
        #   방문지가 5 곳이 안 되고, 배송 순서 계산의 전제가 깨진다.
        per = int(cfg["boxes"]["stage2"]["households_per_apt"])
        plan = [(z, d, nm) for z in cfg["zones"]["list"]
                for d, nm, _la, _lo in APARTMENTS if ZONE_OF[d] == z
                for _ in range(per)]
        rng.shuffle(plan)          # 컨베이어에는 섞여서 온다 — 구역별로 몰리지 않게
        zones = [z for z, _d, _n in plan]
        n = len(plan)

    # ★ 배정표를 먼저 깐다. 이게 없으면 중재자가 모든 박스를 RETURN 시킨다.
    conn.execute("DELETE FROM zone_map")
    conn.executemany("INSERT INTO zone_map(dong, zone) VALUES(?,?)",
                     [(d, z) for z, lst in ZONE_DONG.items() for _, d in lst])

    conn.execute("DELETE FROM boxes_master")
    rows = []
    coord = {(d, nm): (la, lo) for d, nm, la, lo in APARTMENTS}
    for i in range(n):
        if plan is None:
            addr, la, lo = _delivery_point(rng, zones[i])
        else:
            _z, d, nm = plan[i]
            addr, (la, lo) = address_of(d, nm), coord[(d, nm)]
        rows.append((make_box_id(i), addr, i, zones[i], la, lo))
    conn.executemany(
        "INSERT INTO boxes_master(box_id, address, delivery_seq, zone, lat, lon) "
        "VALUES(?,?,?,?,?,?)", rows)
    conn.commit()
    # ★ 검산: 만든 주소가 정말 원래 구역으로 되돌아오는지 확인한다.
    #   동 이름 오타 하나로 전 박스가 RETURN 되는 것을 여기서 잡는다.
    m = dict(conn.execute("SELECT dong, zone FROM zone_map").fetchall())
    for box_id, addr, _seq, zone, _la, _lo in rows:
        d = dong_of(addr)
        if d is None or m.get(d) != zone:
            conn.close()
            raise SystemExit(f"주소 -> 구역 검산 실패: {box_id} {addr!r} "
                             f"-> 동 {d!r} -> {m.get(d)!r} (기대 {zone!r})")
    conn.close()
    return rows


def main():
    ap = argparse.ArgumentParser(description="boxes_master 생성 (작업명세서 §8.3)")
    ap.add_argument("--db-path", default=None, help="기본: 워크스페이스 루트의 palletris.db")
    ap.add_argument("--config", default=None, help="system.yaml 경로")
    a = ap.parse_args()

    cfg = load(a.config)
    db_path = default_db_path(a.db_path)
    # ★ 옛 코드는 `<here>/schema.sql` **하나만** 봤다. 설치본에는 그 자리에 파일이
    #   없어서 일반 colcon build 로는 FileNotFoundError 로 죽었다.
    schema = find_data("packing_db", "schema.sql", __file__)

    rows = build(cfg, db_path, schema)
    zc = {}
    for _, _, _, z, _la, _lo in rows:
        zc[z] = zc.get(z, 0) + 1
    print(f"boxes_master {len(rows)} 행 생성 → {db_path}")
    print(f"  stage {cfg['stage']} · 구역 분포 {zc}")
    print(f"  {rows[0][0]} ~ {rows[-1][0]}")
    print(f"  주소 예시 {rows[0][1]!r} -> 동 {dong_of(rows[0][1])!r} -> 구역 {rows[0][3]}")
    pts = sorted({(r[1], r[4], r[5]) for r in rows})
    print(f"  배송지 {len(pts)} 곳 (실제 아파트 · 좌표 포함)")
    for a, la, lo in pts:
        print(f"    {a:34} {la},{lo}")
    print(f"  zone_map {sum(len(v) for v in ZONE_DONG.values())} 행 "
          f"({' · '.join(z + ':' + ','.join(d for _, d in v) for z, v in ZONE_DONG.items())})")


# ★ _default_db_path 는 packing_common.paths.default_db_path 로 옮겼다 (2026-08-26).
#   db_service_node 에도 똑같은 "config 경로 −4 단계" 산술이 있어 두 벌이었다.


if __name__ == "__main__":
    main()
