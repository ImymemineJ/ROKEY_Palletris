"""배송지 -> 구역 배정 검증 (작업명세서 미결 #9).

ROS 없이 sqlite 만으로 돈다. 구역 배정은 데모의 분기 판단 전체가 걸린 로직이라
노드를 띄우지 않고도 여기서 갈라지는 지점을 잡아야 한다.
"""
import sqlite3
import tempfile
import os

import pytest

from packing_common.config import find_config, load
from packing_db.delivery_points import APARTMENTS, DEPOT
from packing_db.gen_master_db import ZONE_DONG, build, dong_of

CFG = load()
SCHEMA = os.path.join(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))), "packing_db", "schema.sql")


@pytest.fixture(scope="module")
def db():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "t.db")
        rows = build(CFG, path, SCHEMA)
        con = sqlite3.connect(path)
        yield con, rows
        con.close()


def zone_of(con, box_id):
    """db_service_node._zone_by_address 와 같은 순서로 따라간다."""
    r = con.execute("SELECT address FROM boxes_master WHERE box_id=?", (box_id,)).fetchone()
    if not r:
        return None
    d = dong_of(r[0])
    if d is None:
        return None
    z = con.execute("SELECT zone FROM zone_map WHERE dong=?", (d,)).fetchone()
    return z[0] if z else None


@pytest.mark.parametrize("addr,want", [
    ("서울 서초구 반포동 12-3", "반포동"),
    ("서울 강남구 역삼동 300-99", "역삼동"),
    ("서울 송파구 석촌동 1-1", "석촌동"),
    ("서울 강남구 12-3", None),          # 동이 없다
    ("", None),
    (None, None),
])
def test_dong_extraction(addr, want):
    assert dong_of(addr) == want


def test_every_master_row_routes_to_its_zone(db):
    """★ 주소만 보고 따라갔을 때 정답지의 zone 이 나와야 한다.

    어긋나면 인식은 멀쩡한데 박스가 통째로 엉뚱한 구역으로 간다.
    """
    con, rows = db
    for box_id, addr, _seq, zone, _la, _lo in rows:
        assert zone_of(con, box_id) == zone, f"{box_id} {addr!r} -> {zone_of(con, box_id)}"


def test_zone_map_has_no_duplicate_dong(db):
    """한 동이 두 구역에 걸치면 배정이 뒤죽박죽이 된다."""
    con, _ = db
    n = con.execute("SELECT COUNT(*) FROM zone_map").fetchone()[0]
    uniq = con.execute("SELECT COUNT(DISTINCT dong) FROM zone_map").fetchone()[0]
    assert n == uniq == sum(len(v) for v in ZONE_DONG.values())


def test_unknown_dong_gets_no_zone(db):
    """★ 표에 없는 동은 None 이어야 한다 — 아무 구역에나 넣으면 오배송이다."""
    con, _ = db
    con.execute("INSERT INTO boxes_master(box_id, address, delivery_seq, zone) "
                "VALUES('BOX99999', '서울 마포구 연남동 1-1', 999, 'B')")
    assert zone_of(con, "BOX99999") is None
    con.execute("DELETE FROM boxes_master WHERE box_id='BOX99999'")


def test_address_does_not_leak_the_answer(db):
    """★ 주소에 '(B구역)' 같은 정답을 박아 두면 배정 로직이 아무 일도 안 한다."""
    con, rows = db
    for _bid, addr, _s, _z, _la, _lo in rows:
        assert "구역" not in addr, f"주소에 정답이 들어 있다: {addr!r}"


# ══════════════════════════════════════════════════════════════════════
#  배송지 좌표 — 경로 API 가 이 값을 그대로 쓴다
# ══════════════════════════════════════════════════════════════════════

def test_every_master_row_has_coordinates(db):
    """좌표가 비면 경로 계산이 통째로 안 된다."""
    con, _ = db
    n = con.execute("SELECT COUNT(*) FROM boxes_master "
                    "WHERE lat IS NULL OR lon IS NULL").fetchone()[0]
    assert n == 0


def test_coordinates_are_inside_guro(db):
    """★ 좌표가 구로구 밖이면 경로가 엉뚱하게 나오는데 에러는 안 난다.

    구로구 대략 경계: 위도 37.47~37.52 · 경도 126.81~126.90.
    """
    con, _ = db
    for addr, la, lo in con.execute(
            "SELECT DISTINCT address, lat, lon FROM boxes_master"):
        assert 37.46 <= la <= 37.53, f"{addr} 위도 {la}"
        assert 126.80 <= lo <= 126.91, f"{addr} 경도 {lo}"


def test_depot_is_inside_guro():
    _name, la, lo = DEPOT
    assert 37.46 <= la <= 37.53 and 126.80 <= lo <= 126.91


def test_apartment_dongs_are_all_mapped():
    """★ 표에 있는 아파트의 동이 zone_map 에 다 있어야 한다.

    하나라도 빠지면 그 아파트로 가는 박스가 전부 RETURN 된다.
    """
    mapped = {d for lst in ZONE_DONG.values() for _gu, d in lst}
    missing = {d for d, _n, _la, _lo in APARTMENTS} - mapped
    assert not missing, f"zone_map 에 없는 동: {missing}"


def test_no_duplicate_apartment_coordinates():
    """같은 좌표가 둘이면 경유지가 겹쳐 경로가 이상해진다."""
    coords = [(la, lo) for _d, _n, la, lo in APARTMENTS]
    assert len(coords) == len(set(coords))
