#!/usr/bin/env python3
"""boxes_master 적재 — 정답지 CSV 를 palletris.db 에 넣는다 (작업명세서 §8.3).

    ros2 run packing_db gen_master_db              # 정답지만 적재 (안전)
    ros2 run packing_db gen_master_db --reset      # 이전 실행 기록까지 지우고 적재
    ros2 run packing_db gen_master_db --regen      # ★ 정답지 CSV 자체를 다시 만든다

★ 이름이 "gen" 이지만 **만들어 내지 않는다.** 만드는 곳은 `--regen` 하나뿐이다.
  이름은 방법이 아니라 결과를 가리킨다 — palletris.db 의 master 표를 갖춘다.
  정답지 자체는 `packing_db/data/*.csv` 이고 git 에 박혀 있다 (master_data.py 참조).

실행 전에 "정답지"를 미리 깔아 둔다. 실행 중 흘러간 박스(boxes_sorted)와 대조해야
인식률·누락을 잴 수 있다. 등록된 박스만 세면 인식률이 언제나 100 % 로 나온다
(명세서 §8.6 의 함정).

★ `--reset` 은 기본이 아니다.
  이전 실행 기록이 섞이면 지표가 통째로 망가진다 (metrics.py 의 elapsed_s 가
  `MAX(ts) - MIN(ts)` 라 어제 실행까지 포함된다 — 실측으로 robot_idle_rate 가
  100 % 로 굳는 것을 봤다). 그렇다고 기본을 파괴적으로 두면 "정답지 있나 확인차"
  한 번 돌렸다가 데모 기록이 날아간다. **경고는 항상 하고, 지우는 건 시킬 때만** 한다.

★ stage 1 은 예전 코드 그대로 둔다 (`rng.choice` 로 그때그때 생성).
  최종 구현은 stage 2 이고 stage 1 은 의도적 복귀 경로다. 동결할 이유가 없다.
"""
import argparse
import random
import sqlite3
import sys

from packing_common.config import load
from packing_common.ids import box_id as make_box_id
from packing_common.paths import default_db_path, find_data
from packing_db.delivery_points import APARTMENTS, address_of
from packing_db.master_data import (dong_of, load_master,  # noqa: F401 — dong_of 재수출
                                    load_zone_map, reset_run_tables,
                                    run_table_counts, verify)

# ★ dong_of 는 master_data 로 옮겼고 여기서 재수출한다.
#   db_service_node 와 기존 테스트가 `from packing_db.gen_master_db import dong_of`
#   로 가져가고 있어, 그쪽을 안 건드리려고 이름만 남겼다.

# ★ 파일명은 palletris.db 다 (CLAUDE.md §10). 경로는 하드코딩하지 않고
#   packing_common.paths.default_db_path() 가 워크스페이스 표식으로 찾는다.


def _delivery_point(rng, by_zone, zone):
    """배송지 하나를 뽑는다 -> (주소, 위도, 경도). **stage 1 전용.**

    ★ 번지를 랜덤으로 지어내지 않는다. 실존하는 아파트만 쓴다 —
      지어낸 주소는 지오코딩도 경로 계산도 안 되고, 발표에서 물으면 답이 없다.
    """
    d, n, la, lo = rng.choice(by_zone[zone])
    return address_of(d, n), la, lo


def derive(cfg, zone_rows):
    """정답지 150 행을 **처음부터 만든다.** `--regen` 전용.

    ★ 평소에는 절대 안 불린다. 정본은 data/boxes_master.csv 다.
      APARTMENTS 나 zone_map.csv 를 고쳤을 때만 여기를 돌려 CSV 를 새로 뽑는다.

    ★ 박스 수를 랜덤으로 뽑지 않는다. **배송지 구조에서 나온다** —
      구역 3 x 아파트 5 x 세대 N. 랜덤으로 뽑으면 어떤 아파트는 박스가 0 개라
      방문지가 5 곳이 안 되고, 배송 순서 계산의 전제가 깨진다.
    """
    zone_of = dict(zone_rows)
    rng = random.Random(cfg["boxes"]["random_seed"])
    per = int(cfg["boxes"]["stage2"]["households_per_apt"])
    coord = {(d, nm): (la, lo) for d, nm, la, lo in APARTMENTS}

    plan = [(z, d, nm) for z in cfg["zones"]["list"]
            for d, nm, _la, _lo in APARTMENTS if zone_of.get(d) == z
            for _ in range(per)]
    rng.shuffle(plan)          # 컨베이어에는 섞여서 온다 — 구역별로 몰리지 않게

    rows = []
    for i, (z, d, nm) in enumerate(plan):
        la, lo = coord[(d, nm)]
        rows.append((make_box_id(i), address_of(d, nm), i, z, la, lo))
    return rows


def _stage1_rows(cfg, zone_rows):
    """stage 1 (B 구역 단독) 정답지. 예전 방식 그대로 그때그때 만든다.

    ★ stage 1 에서 zone 을 편향 랜덤으로 뽑으면 안 된다. 15 개 중 40 % 가 A/C 로
      배정되면 B 관제탑이 없는 팔레트에 plan_placement 를 호출하게 되고 게이트를
      통과할 수 없다. stage 1 은 전부 "B" 다.
    """
    zone_of = dict(zone_rows)
    by_zone = {}
    for d, n, la, lo in APARTMENTS:
        by_zone.setdefault(zone_of.get(d), []).append((d, n, la, lo))

    rng = random.Random(cfg["boxes"]["random_seed"])
    n = int(cfg["boxes"]["stage1"]["total_count"])
    zone = str(cfg["boxes"]["stage1"]["zone"])
    rows = []
    for i in range(n):
        addr, la, lo = _delivery_point(rng, by_zone, zone)
        rows.append((make_box_id(i), addr, i, zone, la, lo))
    return rows


def build(cfg, db_path, schema_path, reset=False):
    """정답지를 palletris.db 에 넣는다. 이미 있으면 지우고 다시 넣는다 (재현성).

    반환값은 넣은 행 목록 — 기존 테스트가 이걸 그대로 쓴다.
    """
    # ★ timeout 을 짧게 준다. db_service_node 가 떠 있으면 빨리 실패해야 한다 —
    #   기본 5 초를 기다렸다가 SQLite 기본 메시지를 뱉으면 원인을 못 알아본다.
    conn = sqlite3.connect(db_path, timeout=2.0)
    try:
        with open(schema_path, encoding="utf-8") as f:
            conn.executescript(f.read())

        zone_rows = load_zone_map()
        if int(cfg["stage"]) == 1:
            rows = _stage1_rows(cfg, zone_rows)
        else:
            rows = load_master()
            verify(cfg, rows, zone_rows)        # ★ 매 적재마다 검산한다

        # ★ BEGIN IMMEDIATE 로 쓰기 잠금을 앞에서 잡는다. 우리는 두 번째 writer 라
        #   (CLAUDE.md §12: 유일 writer 는 db_service_node) 반쯤 쓰다 막히면
        #   정답지가 반만 들어간 DB 가 남는다. 통째로 잡거나 통째로 실패해야 한다.
        conn.execute("BEGIN IMMEDIATE")
        if reset:
            for t in ("boxes_sorted", "box_events", "scan_misses", "recirculation"):
                conn.execute(f"DELETE FROM {t}")
            conn.execute("DELETE FROM sqlite_sequence WHERE name IN "
                         "('boxes_sorted','box_events','scan_misses','recirculation')")
        # ★ 배정표를 먼저 깐다. 이게 없으면 중재자가 모든 박스를 RETURN 시킨다.
        conn.execute("DELETE FROM zone_map")
        conn.executemany("INSERT INTO zone_map(dong, zone) VALUES(?,?)", zone_rows)
        conn.execute("DELETE FROM boxes_master")
        conn.executemany(
            "INSERT INTO boxes_master(box_id, address, delivery_seq, zone, lat, lon) "
            "VALUES(?,?,?,?,?,?)", rows)
        conn.commit()
    except sqlite3.OperationalError as e:
        conn.close()
        if "locked" in str(e).lower():
            raise SystemExit(
                f"DB 가 잠겨 있다 — db_service_node 가 떠 있는 것 같다.\n"
                f"  gen_master_db 는 두 번째 writer 다 "
                f"(CLAUDE.md §12: 유일 writer 는 db_service_node).\n"
                f"  노드를 먼저 내리고 다시 실행할 것.\n  DB: {db_path}")
        raise
    conn.close()
    return rows


def _warn_leftovers(db_path):
    """이전 실행 기록이 남아 있으면 알린다. **지우지는 않는다.**"""
    try:
        c = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
        n = run_table_counts(c)
        c.close()
    except sqlite3.Error:
        return
    if not any(n.values()):
        return
    left = " · ".join(f"{k} {v} 행" for k, v in n.items() if v)
    print(f"\n⚠ 이전 실행 기록이 남아 있다 — {left}")
    print("  지표가 섞인다: metrics 의 elapsed_s 는 MAX(ts)-MIN(ts) 라 어제 실행까지")
    print("  포함되고, 그러면 robot_idle_rate 가 100 % 로 굳는다.")
    print("  새로 재려면:  ros2 run packing_db gen_master_db --reset")


def _regen(cfg, db_path):
    """정답지 CSV 를 다시 만들어 소스 트리에 쓴다. 그리고 곧바로 되읽어 검산한다."""
    from packing_common.paths import workspace_root
    ws = workspace_root()
    if not ws:
        raise SystemExit("--regen 은 소스 트리에서만 된다 (워크스페이스를 못 찾았다)")
    out = f"{ws}/src/packing_db/packing_db/data/boxes_master.csv"

    zone_rows = load_zone_map()
    rows = derive(cfg, zone_rows)
    with open(out, encoding="utf-8") as f:
        head = [ln for ln in f if ln.lstrip().startswith("#")]   # 머리 주석 보존
    with open(out, "w", encoding="utf-8") as f:
        f.writelines(head)
        f.write("box_id,address,zone,lat,lon\n")
        for bid, addr, _seq, zone, la, lo in rows:
            if "," in addr or '"' in addr:
                raise SystemExit(f"주소에 쉼표·따옴표가 있다 — CSV 를 못 쓴다: {addr!r}")
            f.write(f"{bid},{addr},{zone},{la:.6f},{lo:.6f}\n")

    verify(cfg, load_master(), zone_rows)        # ★ 쓴 것을 되읽어 확인
    print(f"정답지 {len(rows)} 행을 다시 만들었다 → {out}")
    print("  ★ git diff 로 무엇이 바뀌었는지 확인하고 커밋할 것.")


def main():
    ap = argparse.ArgumentParser(
        description="정답지 CSV 를 palletris.db 에 적재한다 (작업명세서 §8.3)")
    ap.add_argument("--db-path", default=None, help="기본: 워크스페이스 루트의 palletris.db")
    ap.add_argument("--config", default=None, help="system.yaml 경로")
    ap.add_argument("--reset", action="store_true",
                    help="이전 실행 기록(boxes_sorted·box_events 등)까지 지운다")
    ap.add_argument("--regen", action="store_true",
                    help="★ 정답지 CSV 자체를 다시 만든다 (APARTMENTS 를 고쳤을 때만)")
    a = ap.parse_args()

    cfg = load(a.config)
    if a.regen:
        _regen(cfg, None)
        return

    db_path = default_db_path(a.db_path)
    # ★ 옛 코드는 `<here>/schema.sql` **하나만** 봤다. 설치본에는 그 자리에 파일이
    #   없어서 일반 colcon build 로는 FileNotFoundError 로 죽었다.
    schema = find_data("packing_db", "schema.sql", __file__)

    if a.reset:
        # 지우기 **전에** 몇 행을 지우는지 보여 준다. 파괴적 동작은 눈에 보여야 한다.
        try:
            c = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
            n = run_table_counts(c)
            c.close()
            if any(n.values()):
                print("--reset : " + " · ".join(f"{k} {v} 행" for k, v in n.items() if v)
                      + " 를 지운다")
        except sqlite3.Error:
            pass

    rows = build(cfg, db_path, schema, reset=a.reset)

    zc = {}
    for _, _, _, z, _la, _lo in rows:
        zc[z] = zc.get(z, 0) + 1
    zone_rows = load_zone_map()
    src = "생성 (stage 1)" if int(cfg["stage"]) == 1 else "정답지 CSV"
    print(f"boxes_master {len(rows)} 행 적재 → {db_path}   [{src}]")
    print(f"  stage {cfg['stage']} · 구역 분포 {zc}")
    print(f"  {rows[0][0]} ~ {rows[-1][0]}")
    print(f"  주소 예시 {rows[0][1]!r} -> 동 {dong_of(rows[0][1])!r} -> 구역 {rows[0][3]}")
    pts = sorted({(r[1], r[4], r[5]) for r in rows})
    print(f"  배송지 {len(pts)} 곳 (실제 아파트 · 좌표 포함)")
    for addr, la, lo in pts:
        print(f"    {addr:34} {la},{lo}")
    by_zone = {}
    for d, z in zone_rows:
        by_zone.setdefault(z, []).append(d)
    print(f"  zone_map {len(zone_rows)} 행 "
          f"({' · '.join(z + ':' + ','.join(v) for z, v in by_zone.items())})")

    if not a.reset:
        _warn_leftovers(db_path)


if __name__ == "__main__":
    sys.exit(main())
