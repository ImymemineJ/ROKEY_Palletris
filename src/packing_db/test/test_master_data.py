"""정답지 CSV — 로더와 검산.

★ 이 파일의 테스트는 **드리프트 한 종류당 하나씩**이다.
  정답지를 파일로 얼렸으니, 이제 위험은 "파일과 코드가 따로 논다" 는 것이다.
  APARTMENTS 를 고치고 --regen 을 잊는다든가, CSV 줄을 하나 지운다든가.
  그럴 때 **조용히 틀린 정답지로 데모가 도는 것**을 막는 게 여기 목적이다.

★ 시드 42 로 다시 굴려 byte 비교하는 테스트는 일부러 없다 (master_data.py 머리말).
"""
import copy
import os
import sqlite3
import tempfile

import pytest

from packing_common.config import load
from packing_db import master_data as md
from packing_db.gen_master_db import build

CFG = load()
SCHEMA = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
                      "packing_db", "schema.sql")


@pytest.fixture
def rows():
    return md.load_master()


@pytest.fixture
def zone_rows():
    return md.load_zone_map()


# ── 로더 ──────────────────────────────────────────────────────────
def test_master_loads(rows):
    assert len(rows) == 150
    assert rows[0][0] == "BOX001" and rows[-1][0] == "BOX150"


def test_delivery_seq_is_the_line_number(rows):
    """★ delivery_seq 는 파일에 없다. 줄 순서가 곧 투입 순서다."""
    assert [r[2] for r in rows] == list(range(len(rows)))


def test_zone_map_loads_raw_not_dict(zone_rows):
    """★ dict 로 주면 중복 동을 조용히 삼킨다. raw 여야 verify 가 잡는다."""
    assert isinstance(zone_rows, list)
    assert ("개봉동", "B") in zone_rows


def test_comments_are_stripped(rows, zone_rows):
    """`#` 주석 줄이 데이터로 새어 들어오면 안 된다."""
    assert not any(r[0].startswith("#") for r in rows)
    assert not any(d.startswith("#") for d, _z in zone_rows)


def test_clean_file_verifies(rows, zone_rows):
    md.verify(CFG, rows, zone_rows)          # 안 죽으면 통과


# ── 검산: 드리프트 한 종류당 하나 ────────────────────────────────
def test_catches_zone_typo(rows, zone_rows):
    """★ 살아남은 그 검산 — 동 이름이 틀어지면 전 박스가 RETURN 된다."""
    bad = list(rows)
    bid, addr, seq, zone, la, lo = bad[7]
    bad[7] = (bid, addr, seq, "A" if zone != "A" else "C", la, lo)
    with pytest.raises(SystemExit, match="zone_map"):
        md.verify(CFG, bad, zone_rows)


def test_catches_coordinate_edit(rows, zone_rows):
    """★ delivery_points.py 를 고치고 --regen 을 잊은 경우."""
    bad = list(rows)
    bid, addr, seq, zone, la, lo = bad[3]
    bad[3] = (bid, addr, seq, zone, la + 0.01, lo)
    with pytest.raises(SystemExit, match="APARTMENTS"):
        md.verify(CFG, bad, zone_rows)


def test_catches_deleted_line(rows, zone_rows):
    """★ 줄을 지우면 행 수 검사에 먼저 걸린다 (box_id 검사보다 앞이다)."""
    with pytest.raises(SystemExit, match="행 수"):
        md.verify(CFG, rows[:5] + rows[6:], zone_rows)


def test_catches_renumbered_box_id(rows, zone_rows):
    """★ 행 수는 맞는데 번호만 틀어진 경우 — 손으로 고치다 흔한 실수다."""
    bad = list(rows)
    bid, addr, seq, zone, la, lo = bad[42]
    bad[42] = ("BOX999", addr, seq, zone, la, lo)
    with pytest.raises(SystemExit, match="box_id"):
        md.verify(CFG, bad, zone_rows)


def test_catches_reordered_lines(rows, zone_rows):
    """★ 정렬해 버린 경우도 같은 검사에 걸린다."""
    bad = list(rows)
    bad[10], bad[20] = bad[20], bad[10]
    with pytest.raises(SystemExit, match="box_id"):
        md.verify(CFG, bad, zone_rows)


def test_catches_config_change(rows, zone_rows):
    """★ households_per_apt 를 바꾸고 --regen 을 잊은 경우."""
    cfg = copy.deepcopy(CFG)
    cfg["boxes"]["stage2"]["households_per_apt"] = 20
    with pytest.raises(SystemExit, match="행 수"):
        md.verify(cfg, rows, zone_rows)


def test_catches_duplicate_dong(rows, zone_rows):
    """★ 한 동이 두 구역에 걸치면 배정이 뒤죽박죽이 된다."""
    with pytest.raises(SystemExit, match="중복"):
        md.verify(CFG, rows, zone_rows + [("개봉동", "A")])


def test_catches_unknown_address(rows, zone_rows):
    """★ APARTMENTS 에 없는 주소는 지오코딩도 경로 계산도 안 된다."""
    bad = list(rows)
    bid, _addr, seq, zone, la, lo = bad[0]
    bad[0] = (bid, "서울 구로구 개봉동 없는아파트", seq, zone, la, lo)
    with pytest.raises(SystemExit, match="APARTMENTS"):
        md.verify(CFG, bad, zone_rows)


def test_error_message_names_the_file_and_line(rows, zone_rows):
    """★ 어느 파일 어느 줄인지 안 찍으면 150 줄에서 눈으로 찾아야 한다."""
    bad = list(rows)
    bad[10], bad[20] = bad[20], bad[10]          # 행 수는 그대로 → 줄 번호가 찍힌다
    with pytest.raises(SystemExit) as e:
        md.verify(CFG, bad, zone_rows)
    msg = str(e.value)
    assert "boxes_master.csv:" in msg
    assert "--regen" in msg, "고치는 방법을 안 알려 준다"


# ── 파일 = DB 내용 ────────────────────────────────────────────────
def test_file_is_what_lands_in_the_db():
    """★ CSV 가 정답지다. DB 에 다른 게 들어가면 파일을 보는 의미가 없다."""
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "t.db")
        build(CFG, path, SCHEMA)
        con = sqlite3.connect(path)
        got = con.execute("SELECT box_id, address, delivery_seq, zone, lat, lon "
                          "FROM boxes_master ORDER BY delivery_seq").fetchall()
        con.close()
    assert got == md.load_master()


# ── --reset ───────────────────────────────────────────────────────
def _seed_run_tables(con):
    con.execute("INSERT INTO boxes_sorted(box_id, zone, size_w_mm, size_d_mm, "
                "size_h_mm, status) VALUES('BOX001','B',340,250,210,'PLACED')")
    con.execute("INSERT INTO box_events(box_id, status, ts) VALUES('BOX001','PLACED',1.0)")
    con.execute("INSERT INTO scan_misses(where_at, reason, ts) "
                "VALUES('cam1','QR_UNREADABLE',1.0)")
    con.commit()


def test_default_keeps_run_tables_and_reset_clears_them():
    """★ 기본은 안전. 지우는 건 시킬 때만 — 모르고 데모 기록을 날리면 안 된다."""
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "t.db")
        build(CFG, path, SCHEMA)
        con = sqlite3.connect(path)
        _seed_run_tables(con)
        con.close()

        build(CFG, path, SCHEMA)                       # 기본 — 남아 있어야 한다
        con = sqlite3.connect(path)
        assert md.run_table_counts(con)["boxes_sorted"] == 1
        con.close()

        build(CFG, path, SCHEMA, reset=True)           # --reset — 비어야 한다
        con = sqlite3.connect(path)
        assert all(v == 0 for v in md.run_table_counts(con).values())
        # 정답지는 그대로여야 한다
        assert con.execute("SELECT COUNT(*) FROM boxes_master").fetchone()[0] == 150
        con.close()


def test_reset_restarts_db_seq():
    """★ db_seq 가 이어지면 plan_route·pallet_access 의 투입 순서가 어긋난다."""
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "t.db")
        build(CFG, path, SCHEMA)
        con = sqlite3.connect(path)
        _seed_run_tables(con)
        md.reset_run_tables(con)
        con.execute("INSERT INTO boxes_sorted(box_id, zone, size_w_mm, size_d_mm, "
                    "size_h_mm, status) VALUES('BOX002','B',340,250,210,'SCANNED')")
        con.commit()
        assert con.execute("SELECT db_seq FROM boxes_sorted").fetchone()[0] == 1
        con.close()
