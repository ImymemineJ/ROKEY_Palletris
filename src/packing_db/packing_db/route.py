"""배송 경로 — 네이버 Directions 로 **실제 도로 거리**를 잰다.

## 이 모듈이 하는 일 — **거리를 재는 것뿐이다**

배송 순서는 **우리가 정한다** — 팔레트에서 꺼내기 쉬운 순서를 지켜야 하기 때문이다
(`accessibility.py`). 그러니 "순서를 대신 짜 주는" 기능은 필요 없고, 오히려 있으면
우리 순서를 덮어써서 방해가 된다. 필요한 건 "이 순서로 가면 몇 km 인가" 하나다.

## ★ 순서는 **도로거리로** 고른다 — 직선거리로 고르면 안 된다

실측 (2026-08-25, 구역 B · 배송지 9곳 · 들어내기 0 회 조건):
      직선거리로 고른 순서 -> 실제 도로 18.98 km · 83.5 분
    ★ 도로거리로 고른 순서 -> 실제 도로 17.56 km · 77.3 분   (7.5 % 짧다)

같은 실측에서 **45 쌍 중 41 쌍이 방향에 따라 거리가 달랐다.** 제일 심한 구간은
창동아파트 ↔ 고척파크푸르지오로 3,595 m vs 2,251 m (1.3 km 차이) 였다.
일방통행·중앙분리대 때문이다. 직선거리는 이걸 전혀 모른다.

## ★ 후보를 먼저 줄이고 나서 부른다

순서 후보를 전부 API 로 재면 배송지 5 곳도 120 회다. 그럴 필요가 없다 —
**들어내기 횟수는 API 없이 계산되므로**, 최소 횟수를 내는 후보만 남기고
그것들만 도로거리로 잰다. 보통 몇 개에서 몇십 개로 준다.

그래도 많으면 직선거리로 상위 몇 개만 추려 잰다. 그때는 **몇 개를 추렸는지 알린다** —
조용히 자르면 최적을 놓친 줄도 모른다.

## 기본정보 (문서 원문)

    GET https://maps.apigw.ntruss.com/map-direction-15/v1/driving
    헤더 x-ncp-apigw-api-key-id / x-ncp-apigw-api-key
    경유지 최대 15 개 (실측: 16 개는 HTTP 400)

★ 호스트가 문서 링크와 다르다 — `naveropenapi.apigw.ntruss.com` 은 전부 401 이다.

## ★ 키는 코드에도 저장소에도 넣지 않는다

    export NCP_API_KEY_ID=...
    export NCP_API_KEY=...
"""
import json
import os
import urllib.error
import urllib.parse
import urllib.request

HOST = "https://maps.apigw.ntruss.com"
PATH = "/map-direction-15/v1/driving"
MAX_WAYPOINTS = 15                     # 실측 상한. 16 을 넣으면 HTTP 400 이다
STOPS_PER_CALL = MAX_WAYPOINTS + 1     # 경유지 15 + 도착지 1

# 탐색 옵션. 문서 기본값이 traoptimal("실시간 최적")이라 그대로 쓴다.
# 실측(구로구 4곳): traoptimal 10.40 km / trafast 10.40 / tracomfort 10.64.
DEFAULT_OPTION = "traoptimal"

# 차량 정보는 문서 기본값 그대로 둔다 (2026-08-25 팀 결정).
# 우리 구간(구로구 시내)은 통행료가 0 원이라 cartype 을 바꿔도 거의 안 변한다.
# ⚠ 미결: 실제 배송 트럭의 차종·연비는 팀이 정할 값이다. 지어내지 않는다.
DEFAULT_CARTYPE = 1
DEFAULT_FUELTYPE = "gasoline"
DEFAULT_MILEAGE = 14


class RouteError(RuntimeError):
    pass


def _keys():
    kid = os.environ.get("NCP_API_KEY_ID", "").strip()
    key = os.environ.get("NCP_API_KEY", "").strip()
    if not kid or not key:
        raise RouteError(
            "NCP_API_KEY_ID / NCP_API_KEY 환경변수가 없다.\n"
            "  export NCP_API_KEY_ID=...\n"
            "  export NCP_API_KEY=...\n"
            "★ 키를 소스나 yaml 에 적지 말 것 — 저장소에 올라간다.")
    return kid, key


def _fmt(p):
    """(위도, 경도) -> 'lon,lat'. ★ 네이버는 경도가 먼저다 — 뒤집으면 바다로 간다."""
    return f"{float(p[1]):.7f},{float(p[0]):.7f}"


def directions(start, goal, waypoints=(), option=DEFAULT_OPTION, timeout=20.0,
               cartype=DEFAULT_CARTYPE, fueltype=DEFAULT_FUELTYPE,
               mileage=DEFAULT_MILEAGE):
    """실제 도로 경로 한 구간. start/goal/waypoints 는 전부 (위도, 경도).

    반환: dict(distance_m, duration_s, toll_won, taxi_won, fuel_won, path)
    """
    kid, key = _keys()
    wp = list(waypoints)
    if len(wp) > MAX_WAYPOINTS:
        raise RouteError(f"경유지가 {len(wp)} 개라 상한 {MAX_WAYPOINTS} 을 넘는다")

    q = {"start": _fmt(start), "goal": _fmt(goal), "option": option,
         "cartype": int(cartype), "fueltype": str(fueltype), "mileage": mileage}
    if wp:
        q["waypoints"] = "|".join(_fmt(p) for p in wp)
    url = f"{HOST}{PATH}?{urllib.parse.urlencode(q, safe='|,')}"
    req = urllib.request.Request(url, headers={
        "x-ncp-apigw-api-key-id": kid, "x-ncp-apigw-api-key": key})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            d = json.load(r)
    except urllib.error.HTTPError as e:
        raise RouteError(f"Directions HTTP {e.code}: "
                         f"{e.read().decode('utf-8', 'replace')[:300]}") from e
    except Exception as e:                      # noqa: BLE001 — 네트워크 전반
        raise RouteError(f"Directions 호출 실패: {e}") from e

    if d.get("code") != 0:
        raise RouteError(f"Directions code={d.get('code')} {d.get('message')}")
    routes = d.get("route", {})
    if option not in routes or not routes[option]:
        raise RouteError(f"경로가 비었다 — 응답 옵션 {list(routes)}")
    r0 = routes[option][0]
    s = r0["summary"]
    return {
        "distance_m": int(s["distance"]),
        "duration_s": int(s["duration"]) / 1000.0,   # ★ 응답은 밀리초다
        "toll_won": int(s.get("tollFare", 0)),
        "taxi_won": int(s.get("taxiFare", 0)),
        "fuel_won": int(s.get("fuelPrice", 0)),
        "path": r0.get("path", []),
    }


#: 도로거리로 재 볼 후보 순서의 최대 개수. 넘으면 직선거리로 추려 낸다.
#: 배송지 5 곳이면 순서가 120 가지뿐이라 전부 재도 된다. 그래서 상한을 그 위로 둔다.
MAX_ROAD_CANDIDATES = 120


def best_order(candidates, depot, points, prefilter=None,
               max_candidates=MAX_ROAD_CANDIDATES, on_try=None, **kw):
    """후보 순서들을 **실제 도로거리로** 재서 가장 짧은 것을 고른다.

    candidates : 순서 후보 목록. 각각 배송지 이름의 나열
    depot      : (위도, 경도) 물류센터 — 출발이자 도착
    points     : {이름: (위도, 경도)}
    prefilter  : 후보가 너무 많을 때 쓰는 정렬 키 (보통 직선거리 함수)

    반환: (최적 순서, 그 순서의 measure() 결과, 실제로 재 본 후보 수)
    """
    cands = list(candidates)
    if not cands:
        raise RouteError("후보 순서가 없다")
    trimmed = 0
    if len(cands) > max_candidates:
        if prefilter is None:
            raise RouteError(
                f"후보가 {len(cands)} 개인데 상한이 {max_candidates} 다. "
                "prefilter(직선거리 함수)를 주거나 상한을 올릴 것 — "
                "조용히 자르지 않는다.")
        cands.sort(key=prefilter)
        trimmed = len(cands) - max_candidates
        cands = cands[:max_candidates]

    best, best_r = None, None
    for i, o in enumerate(cands, 1):
        r = measure(o, depot, points, **kw)
        if on_try:
            on_try(o, r, i, len(cands))
        if best_r is None or r["distance_m"] < best_r["distance_m"]:
            best, best_r = o, r
    best_r["tried"] = len(cands)
    best_r["trimmed"] = trimmed        # ★ 몇 개를 못 재 봤는지 알린다
    return best, best_r


def measure(order, depot, points, **kw):
    """정해진 순서를 **한 번의 호출**로 실제 측정한다 (경로 좌표까지 받는다).

    order  : 배송지 이름 목록 (방문 순서)
    depot  : (위도, 경도) 물류센터
    points : {이름: (위도, 경도)}

    ★ 배송지가 16 곳을 넘으면 나눠 부르고 더한다. 조용히 잘라 내지 않는다.
    """
    stops = [points[a] for a in order]
    if not stops:
        raise RouteError("배송지가 없다")
    total = {"distance_m": 0, "duration_s": 0.0, "toll_won": 0,
             "fuel_won": 0, "path": [], "legs": 0}
    cur = depot
    i = 0
    while i < len(stops):
        chunk = stops[i:i + STOPS_PER_CALL]
        r = directions(cur, chunk[-1], chunk[:-1], **kw)
        for k in ("distance_m", "toll_won", "fuel_won"):
            total[k] += r[k]
        total["duration_s"] += r["duration_s"]
        total["path"] += r["path"]
        total["legs"] += 1
        cur = chunk[-1]                 # ★ 다음 구간은 여기서 이어서 출발한다
        i += STOPS_PER_CALL
    # 물류센터 복귀
    r = directions(cur, depot, **kw)
    for k in ("distance_m", "toll_won", "fuel_won"):
        total[k] += r[k]
    total["duration_s"] += r["duration_s"]
    total["path"] += r["path"]
    total["legs"] += 1
    return total
