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

# ★ 파일명은 palletris.db 다 (CLAUDE.md §10). 트랙 D 가이드에는 packing.db 로
#   적혀 있는데 정본은 CLAUDE.md 라 그쪽을 따른다. 경로 자체는 하드코딩하지 않고
#   _default_db_path() 가 config 위치에서 역산한다.

_SIDO = ["서울 강남구", "부산 해운대구", "대전 유성구", "광주 서구", "경기 성남시"]


def _address(rng, zone):
    """배송지. 실제 주소일 필요는 없고 zone 과 1:1 로 묶이기만 하면 된다."""
    return f"{rng.choice(_SIDO)} {rng.randint(1, 300)}-{rng.randint(1, 99)} ({zone}구역)"


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
    else:
        n = int(cfg["boxes"]["stage2"]["total_count"])
        bias = cfg["profiles"][cfg["run_profile"]]["zone_bias"]
        zones = rng.choices(["A", "B", "C"], weights=[bias[z] for z in "ABC"], k=n)

    conn.execute("DELETE FROM boxes_master")
    rows = [("BOX%05d" % i, _address(rng, zones[i]), i, zones[i]) for i in range(n)]
    conn.executemany(
        "INSERT INTO boxes_master(box_id, address, delivery_seq, zone) VALUES(?,?,?,?)",
        rows)
    conn.commit()
    conn.close()
    return rows


def main():
    ap = argparse.ArgumentParser(description="boxes_master 생성 (작업명세서 §8.3)")
    ap.add_argument("--db-path", default=None, help="기본: 워크스페이스 루트의 palletris.db")
    ap.add_argument("--config", default=None, help="system.yaml 경로")
    a = ap.parse_args()

    cfg = load(a.config)
    db_path = os.path.expanduser(a.db_path) if a.db_path else _default_db_path()
    schema = os.path.join(os.path.dirname(os.path.realpath(__file__)), "schema.sql")

    rows = build(cfg, db_path, schema)
    zc = {}
    for _, _, _, z in rows:
        zc[z] = zc.get(z, 0) + 1
    print(f"boxes_master {len(rows)} 행 생성 → {db_path}")
    print(f"  stage {cfg['stage']} · 구역 분포 {zc}")
    print(f"  {rows[0][0]} ~ {rows[-1][0]}")


def _default_db_path():
    """워크스페이스 루트를 config 위치에서 역산한다.

    system.yaml 은 <ws>/src/packing_bringup/config/system.yaml 이므로 4 단계 위가 루트다.
    실행 위치에 기대지 않는 이유: 상대경로면 실행 디렉터리마다 다른 빈 DB 가 생겨
    "master 에 없는 박스" 만 찍힌다 (트랙 D 가이드 D1 경고).
    """
    from packing_common.config import find_config
    cfg_path = os.path.realpath(find_config())
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(cfg_path))))
    return os.path.join(ws, "palletris.db")


if __name__ == "__main__":
    main()
