"""팔레트 접근성 검증 — 쌓인 더미에서 꺼내는 순서가 맞는가.

ROS 없이 순수 계산이라 여기서 전부 잡는다.
"""
import pytest

from packing_db.accessibility import (best_visit_order, blocking_moves,
                                      footprint, is_above, unload_order,
                                      visit_moves)


def box(bid, x, y, z, w=340, d=250, h=210, yaw=0.0):
    return {"box_id": bid, "x": x, "y": y, "z": z, "w": w, "d": d, "h": h, "yaw": yaw}


def test_footprint_swaps_on_yaw90():
    """yaw 90 이면 가로세로가 바뀐다. 안 바꾸면 겹침 판정이 통째로 틀린다."""
    a = footprint(box("A", 0, 0, 0, w=400, d=200, yaw=0))
    b = footprint(box("B", 0, 0, 0, w=400, d=200, yaw=90))
    assert (a[2] - a[0], a[3] - a[1]) == (400, 200)
    assert (b[2] - b[0], b[3] - b[1]) == (200, 400)


def test_stacked_box_blocks_the_one_below():
    low = box("low", 500, 400, 0)
    high = box("high", 500, 400, 210)
    assert is_above(high, low)
    assert not is_above(low, high)


def test_side_by_side_do_not_block():
    """★ 같은 층에 나란히 선 박스는 서로를 막지 않는다.

    바닥 높이만 비교하면 z 가 같아서 서로 막는 것으로 나온다 — 그러면
    순서 계산이 통째로 무너진다.
    """
    a = box("a", 200, 400, 0)
    b = box("b", 600, 400, 0)
    assert not is_above(a, b)
    assert not is_above(b, a)


def test_offset_stack_still_blocks():
    """살짝 어긋나게 얹혀도 막는 것은 막는 것이다."""
    low = box("low", 500, 400, 0)
    high = box("high", 620, 430, 210)      # 발자국이 크게 겹친다
    assert is_above(high, low)


def test_barely_touching_corner_does_not_block():
    """모서리만 스치는 것은 안 막는 것으로 본다 (계산 오차 구분)."""
    low = box("low", 500, 400, 0, w=340, d=250)
    high = box("high", 500 + 340, 400 + 250, 210, w=340, d=250)
    assert not is_above(high, low)


def test_unload_order_peels_top_down():
    """3층 탑 -> 위에서부터 나와야 한다."""
    b = [box("bottom", 500, 400, 0), box("middle", 500, 400, 210),
         box("top", 500, 400, 420)]
    got = [x["box_id"] for x in unload_order(b)]
    assert got == ["top", "middle", "bottom"]


def test_unload_order_layers():
    """같은 층 둘은 같은 겹(layer)이고, 그 위 박스보다 뒤에 나온다."""
    b = [box("l1", 200, 400, 0), box("l2", 600, 400, 0),
         box("up", 200, 400, 210)]
    o = unload_order(b)
    lay = {x["box_id"]: x["layer"] for x in o}
    assert lay["up"] == 1
    assert lay["l1"] > lay["up"]
    assert lay["l2"] == 1          # up 이 안 막으므로 l2 도 바로 꺼낼 수 있다


def test_unload_order_prefers_higher_then_outer():
    """한 겹 안에서는 높은 것 -> 바깥쪽 순이다."""
    b = [box("low_out", 100, 100, 0), box("high_in", 500, 400, 0, h=340)]
    got = [x["box_id"] for x in unload_order(b)]
    assert got[0] == "high_in"     # 더 높다


def test_unload_order_covers_every_box():
    b = [box(f"B{i}", 200 + (i % 3) * 350, 300 + (i // 3) * 260, (i // 6) * 210)
         for i in range(12)]
    o = unload_order(b)
    assert len(o) == 12
    assert {x["box_id"] for x in o} == {x["box_id"] for x in b}
    assert [x["seq"] for x in o] == list(range(1, 13))


def test_blocking_moves_zero_for_unload_order():
    """★ 접근성 순서로 배송하면 들어내는 일이 0 이다. 이게 이 모듈의 존재 이유다."""
    b = [box("bottom", 500, 400, 0), box("middle", 500, 400, 210),
         box("top", 500, 400, 420), box("side", 100, 100, 0)]
    order = [x["box_id"] for x in unload_order(b)]
    total, _ = blocking_moves(b, order)
    assert total == 0


def test_blocking_moves_counts_bad_order():
    """거꾸로 배송하면 위에 얹힌 것을 그때마다 들어내야 한다."""
    b = [box("bottom", 500, 400, 0), box("middle", 500, 400, 210),
         box("top", 500, 400, 420)]
    total, detail = blocking_moves(b, ["bottom", "middle", "top"])
    assert total == 3              # bottom 꺼낼 때 2 개 + middle 꺼낼 때 1 개
    assert detail[0]["moves"] == 2


def test_no_infinite_loop_on_weird_geometry():
    """실측 pose 가 흔들려 서로가 서로를 막아도 멈추지 않는다."""
    a = box("a", 500, 400, 100)
    c = box("c", 500, 400, 100)    # 완전히 겹친 같은 높이 — 병리적 입력
    o = unload_order([a, c])
    assert len(o) == 2


# ══════════════════════════════════════════════════════════════════════
#  방문 순서 — 전수 탐색
# ══════════════════════════════════════════════════════════════════════

def _flat(seq):
    """거리를 무시하는 dist_fn — 들어내기만 보고 싶을 때."""
    return 0.0


def test_visit_never_repeats_an_address():
    """★ 한 배송지는 한 번만 간다. 같은 아파트로 가는 박스가 여러 층에 흩어져도."""
    b = [box("a1", 500, 400, 0), box("mid", 500, 400, 210),
         box("a2", 500, 400, 420), box("other", 100, 100, 0)]
    addr = {"a1": "A동", "a2": "A동", "mid": "B동", "other": "C동"}
    v = best_visit_order(b, lambda x: addr[x], _flat)
    names = [g["address"] for g in v]
    assert len(names) == len(set(names)) == 3
    a = next(g for g in v if g["address"] == "A동")
    assert set(a["box_ids"]) == {"a1", "a2"}


def test_visit_finds_zero_lift_order_when_possible():
    b = [box("bottom", 500, 400, 0), box("top", 500, 400, 210),
         box("side", 100, 100, 0)]
    addr = {"bottom": "A", "top": "B", "side": "C"}
    v = best_visit_order(b, lambda x: addr[x], _flat)
    assert visit_moves(b, v) == 0
    assert [g["address"] for g in v].index("B") < [g["address"] for g in v].index("A")


def test_visit_minimises_lifts_when_zero_impossible():
    """★ 0 이 불가능한 배치도 있다 — 그때는 최소값을 찾아야 한다.

    A동 박스가 맨 아래와 맨 위에 있고 그 사이에 B동 박스가 끼면,
    A동을 한 번에 처리하려면 B동 박스를 반드시 한 번 들어내야 한다.
    """
    b = [box("a_low", 500, 400, 0), box("b_mid", 500, 400, 210),
         box("a_top", 500, 400, 420)]
    addr = {"a_low": "A", "a_top": "A", "b_mid": "B"}
    v = best_visit_order(b, lambda x: addr[x], _flat)
    assert visit_moves(b, v) == 1          # 0 은 불가능하다
    assert len(v) == 2


def test_visit_breaks_ties_by_distance():
    """들어내기가 같으면 거리가 짧은 쪽을 고른다."""
    b = [box("p", 100, 100, 0), box("q", 600, 100, 0), box("r", 1100, 100, 0)]
    addr = {"p": "P", "q": "Q", "r": "R"}
    pos = {"P": 0.0, "Q": 1.0, "R": 5.0}   # 1 차원 거리

    def d(seq):
        cur, acc = 0.0, 0.0
        for a in seq:
            acc += abs(pos[a] - cur)
            cur = pos[a]
        return acc

    v = best_visit_order(b, lambda x: addr[x], d)
    assert [g["address"] for g in v] == ["P", "Q", "R"]


def test_visit_refuses_too_many_addresses():
    """★ 상한을 넘으면 조용히 어림짐작으로 갈아타지 않고 에러를 낸다."""
    b = [box(f"B{i}", 100 + i * 300, 100, 0) for i in range(6)]
    addr = {f"B{i}": f"A{i}" for i in range(6)}
    with pytest.raises(ValueError, match="전수 탐색"):
        best_visit_order(b, lambda x: addr[x], _flat, max_addrs=5)
