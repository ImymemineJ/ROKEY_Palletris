"""팔레트 **접근성** — 쌓인 더미에서 꺼내기 쉬운 순서를 뽑는다.

## 개념

적재는 알고리즘이 충전율 최대로 한다. 배송 순서를 적재에 끼워 넣지 않는다 —
박스가 배송 순서대로 컨베이어에 오지 않으므로 그건 애초에 불가능하고,
끼워 넣으면 충전율(평가 항목)이 떨어진다.

대신 **다 쌓고 난 뒤** 더미를 보고 "이건 지금 꺼낼 수 있다 / 이건 위에 뭐가 있다"
를 계산해서, **꺼내기 쉬운 것부터 배송**한다. 순서를 적재에 맞추는 게 아니라
적재에 순서를 맞추는 것이다.

    적재  : 충전율 최대 (알고리즘 마음대로)
              ↓
    접근성 : 위에 아무것도 없는 박스 = 지금 꺼낼 수 있다
              ↓ 꺼내면 그 아래가 드러난다 (반복)
    배송  : 그 순서대로

## 왜 이게 맞는가

기사가 트럭 뒤에서 박스를 꺼낼 때, 위에 뭐가 올라가 있으면 그걸 먼저 치워야 한다.
접근성 순서를 무시하고 배송하면 **한 집 배달하려고 박스 세 개를 들어냈다 다시
쌓는다.** 그 횟수가 아래 `blocking_moves()` 가 재는 값이다.

## 입력

`boxes_sorted` 의 실측 적재 위치다 (place_x_mm · place_y_mm · place_z_mm · 치수).
★ `place_z_mm` 은 **밑면** 기준이다 (CLAUDE.md §3-1). 중심으로 쓰면 층 판정이 어긋난다.
★ `place_x/y_mm` 은 박스 **중심**이다.

## 좌표

팔레트 로컬 mm. yaw 는 0 또는 90 뿐이라(PlacePose 규약) 90 이면 w/d 를 맞바꾼다 —
회전 행렬이 필요 없다.
"""

#: 두 박스가 "겹친다" 고 볼 최소 면적 (mm^2). 모서리만 스치는 것은 안 막는 것으로 본다.
#: 셀이 10 mm 라 100 mm^2 = 한 셀. 그보다 작은 겹침은 계산 오차다.
MIN_OVERLAP_MM2 = 100

#: 높이 판정 여유 (mm). 실측 pose 는 몇 mm 씩 흔들리므로 딱 붙여 비교하면
#: 바로 위에 얹힌 박스를 "안 겹친다" 로 놓친다.
Z_TOL_MM = 15


def footprint(b):
    """박스의 바닥 사각형 (x0, y0, x1, y1). 중심 좌표 + yaw 로 만든다."""
    w, d = b["w"], b["d"]
    if abs(float(b.get("yaw", 0.0)) % 180.0 - 90.0) < 45.0:
        w, d = d, w                       # yaw 90 -> 가로세로 맞바꿈
    return (b["x"] - w / 2.0, b["y"] - d / 2.0,
            b["x"] + w / 2.0, b["y"] + d / 2.0)


def _overlap_mm2(a, b):
    ax0, ay0, ax1, ay1 = footprint(a)
    bx0, by0, bx1, by1 = footprint(b)
    ox = min(ax1, bx1) - max(ax0, bx0)
    oy = min(ay1, by1) - max(ay0, by0)
    return ox * oy if ox > 0 and oy > 0 else 0.0


def is_above(upper, lower):
    """upper 가 lower **위에** 있는가 = lower 를 꺼내려면 upper 를 먼저 치워야 하는가.

    ★ 바닥 높이만 비교하면 안 된다. 옆에 나란히 선 같은 층 박스도 z 가 같아서
      서로를 막는 것으로 나온다. 발자국이 겹치는지를 같이 봐야 한다.
    """
    if upper is lower:
        return False
    if upper["z"] + Z_TOL_MM < lower["z"] + lower["h"]:
        return False                      # 위가 아니다 (같은 층이거나 아래다)
    return _overlap_mm2(upper, lower) >= MIN_OVERLAP_MM2


def unload_order(boxes):
    """꺼내기 쉬운 순서. 위에 아무것도 없는 것부터 한 겹씩 벗긴다.

    boxes: [{"box_id", "x", "y", "z", "w", "d", "h", "yaw"}]  — mm 정수 / z 는 밑면
    반환 : 같은 dict 목록을 **꺼낼 순서대로**. 각 원소에 다음을 채워 준다:
             "layer"   몇 번째로 벗겨지는 겹인가 (1 = 맨 위, 지금 바로 꺼낼 수 있다)
             "seq"     전체 순번 (1 부터)

    ★ 한 겹 안에서는 **높은 것 → 팔레트 바깥쪽** 순으로 둔다.
      높은 쪽이 허리를 덜 굽히고, 바깥쪽이 팔을 덜 뻗는다.
    """
    rest = [dict(b) for b in boxes]
    if not rest:
        return []
    cx = sum(b["x"] for b in rest) / len(rest)
    cy = sum(b["y"] for b in rest) / len(rest)

    out, layer = [], 0
    while rest:
        layer += 1
        free = [b for b in rest if not any(is_above(o, b) for o in rest)]
        if not free:
            # ★ 서로가 서로를 막는 고리. 실측 pose 가 흔들리면 생길 수 있다.
            #   무한 루프로 두면 지표 계산이 멈춘다 — 남은 것 중 제일 높은 것을
            #   강제로 뽑고 넘어간다.
            free = [max(rest, key=lambda b: b["z"] + b["h"])]
        free.sort(key=lambda b: (-(b["z"] + b["h"]),
                                 -((b["x"] - cx) ** 2 + (b["y"] - cy) ** 2)))
        for b in free:
            b["layer"] = layer
            b["seq"] = len(out) + 1
            out.append(b)
            rest.remove(b)
    return out


def blocking_moves(boxes, order):
    """주어진 배송 순서로 꺼낼 때 **몇 번을 들어냈다 다시 쌓아야** 하는가.

    order: box_id 목록 (배송 순서)
    반환 : (총 들어낸 횟수, [{"box_id", "moves"}])

    ★ 이게 접근성의 정량 지표다. `unload_order` 로 배송하면 0 이고,
      아무 순서로 배송하면 커진다. 그 차이가 곧 개선 폭이다.
    """
    left = {b["box_id"]: dict(b) for b in boxes}
    total, detail = 0, []
    for bid in order:
        tgt = left.get(bid)
        if tgt is None:
            continue
        n = sum(1 for o in left.values() if is_above(o, tgt))
        total += n
        detail.append({"box_id": bid, "moves": n})
        del left[bid]
    return total, detail


# ══════════════════════════════════════════════════════════════════════
#  방문 순서 — 전수 탐색  (2026-08-25 확정)
# ══════════════════════════════════════════════════════════════════════
#
# 배송지 순서를 **전부 해 보고** 제일 좋은 것을 고른다.
#   1순위 들어냈다 다시 쌓는 횟수 최소
#   2순위 그중 최단 거리
#
# ★ 왜 어림짐작(구석 점수 · 겹 단위)을 버렸나 — 구역 B 실측 (배송지 9곳):
#       구석 점수 + 위상정렬   13.81 km · 58.6 분 · 0 회
#     ★ 전수 탐색(0회 고정)    11.39 km · 48.4 분 · 0 회
#       전수 탐색(거리 최소)    9.80 km · 41.7 분 · 3 회
#   같은 0 회인데 어림짐작이 10 분을 더 썼다. 규칙 하나로 밀어붙이면 이만큼 손해다.
#
# ★ 경우의 수는 **배송지 수**로 정해진다. 박스 수가 아니다 —
#   한 아파트에 박스가 8 개 가도 "한 번 가서 8 개 내려놓기" 라 한 곳이다.
#       배송지 5 곳 =     120 가지 -> 즉시      ← 최종 구현이 여기다
#       배송지 8 곳 =  40,320 가지 -> 0.3 초
#       배송지 9 곳 = 362,880 가지 -> 2.6 초
#       배송지 11 곳 = 4,000 만 가지 -> 5 분    ← 여기부터는 못 쓴다
#   그래서 상한을 둔다. 넘으면 **조용히 어림짐작으로 갈아타지 않고 에러를 낸다** —
#   갈아타면 결과가 나빠진 줄도 모르고 넘어간다.
MAX_BRUTE_ADDRS = 9


def best_visit_order(boxes, address_of, dist_fn, max_addrs=MAX_BRUTE_ADDRS):
    """배송지 방문 순서를 전수 탐색으로 고른다. **한 배송지는 한 번만** 방문한다.

    boxes      : {"box_id","x","y","z","w","d","h","yaw"} 목록 (mm · z 는 밑면)
    address_of : box_id -> 배송지 문자열 (같은 문자열 = 같은 곳)
    dist_fn    : (배송지 목록) -> 총 이동거리. 물류센터 왕복까지 포함해 넘긴다.

    반환: [{"address","box_ids","seq"}]  — 방문 순서대로
    """
    from itertools import permutations

    group = {}
    for b in boxes:
        group.setdefault(address_of(b["box_id"]), []).append(b["box_id"])
    addrs = list(group)
    if not addrs:
        return []
    if len(addrs) > max_addrs:
        raise ValueError(
            f"배송지가 {len(addrs)} 곳이라 전수 탐색을 못 한다 (상한 {max_addrs}). "
            "아파트 수를 줄이거나 상한을 올릴 것 — 한 곳 늘 때마다 시간이 곱으로 는다.")

    # 어떤 박스 위에 무엇이 얹혀 있는가. 순서마다 다시 계산하면 느리다.
    above = {b["box_id"]: {o["box_id"] for o in boxes if is_above(o, b)} for b in boxes}

    def lifts(order):
        left = {b["box_id"] for b in boxes}
        n = 0
        for a in order:
            n += _take_group(group[a], above, left)
        return n

    best = min(permutations(addrs), key=lambda o: (lifts(o), dist_fn(o)))
    return [{"address": a, "box_ids": group[a], "seq": i}
            for i, a in enumerate(best, 1)]


def _take_group(box_ids, above, left):
    """한 배송지에서 그 집 박스를 전부 내린다. 들어낸 횟수를 돌려준다.

    ★ **위에 있는 것부터** 꺼낸다. 순서를 안 따지면 실제보다 많이 센다 —
      같은 집 박스가 위아래로 있으면 위엣것을 먼저 집어야 아래것이 그만큼 덜 막힌다.
      (실측: 3층 탑에서 순서를 안 따지면 2 회, 따지면 1 회)
    """
    todo = [b for b in box_ids if b in left]
    n = 0
    while todo:
        pick = min(todo, key=lambda b: len(above[b] & left))
        n += len(above[pick] & left)
        left.discard(pick)
        todo.remove(pick)
    return n


def visit_moves(boxes, visits):
    """이 방문 순서로 돌 때 들어냈다 다시 쌓는 총 횟수.

    한 배송지에서 그 집 박스를 **전부** 내린다. 그때 위에 얹혀 있는 박스만 센다.

    ★ 0 이 항상 가능한 것은 아니다. 한 아파트 박스가 여러 층에 흩어져 있고 그 사이에
      남의 박스가 끼면, "한 번만 방문" 과 "안 들어내기" 가 부딪힌다.
      구역 B 실측에서 실제로 그런 팔레트가 나왔다 — 그때는 전수 탐색이 최소값을 찾아 준다.
    """
    above = {b["box_id"]: {o["box_id"] for o in boxes if is_above(o, b)} for b in boxes}
    left = {b["box_id"] for b in boxes}
    return sum(_take_group(g["box_ids"], above, left) for g in visits)
