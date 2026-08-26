"""정답지 로더 — `data/*.csv` 를 읽고 검산한다. ROS 의존성 0.

★ 왜 파일로 얼렸나 (2026-08-26)
  예전에는 실행할 때마다 `random.Random(42).shuffle` 로 150 행을 만들었다. 시드가
  고정이라 결과는 늘 같았지만 (두 번 돌려 byte 동일 확인), 세 가지가 아쉬웠다:
    · 주소 목록을 눈으로 볼 수도 손으로 고칠 수도 없다
    · git diff 에 "무엇이 어디로 갔는지" 가 안 남는다
    · `shuffle` 내부는 CPython 판 사이에서 보장되지 않는다 — 팀원 PC 마다
      다른 정답지가 나올 여지가 있다

  이제 `data/boxes_master.csv` 가 정본이다. 만드는 곳은 `--regen` 하나뿐이다.

★ 검산(`verify`)이 예전보다 세졌다.
  옛 검산은 방금 `ZONE_DONG` 으로 만든 걸 `ZONE_DONG` 으로 대조해 거의 동어반복이었다.
  이제 **따로 편집되는 두 파일**(`boxes_master.csv` · `zone_map.csv`)과 `APARTMENTS`
  를 서로 대조하므로 실질이 생긴다. 어느 하나만 고치고 나머지를 잊으면 그 자리에서 죽는다.

★ 시드 42 로 다시 굴려 byte 비교하는 테스트는 **일부러 안 만든다.**
  아래 검산 1·3·4 가 실제 드리프트를 이미 다 덮는다. 반면 CPython 이 `shuffle`
  내부를 바꾸면 **아무 의미 없는 차이로 팀원 PC 에서 데모 직전에** 터진다.
  이제 정본은 CSV 다. 검토 수단은 `--regen` + `git diff`.
"""
import csv
import os

from packing_common.ids import box_id as make_box_id
from packing_common.paths import find_data
from packing_db.delivery_points import APARTMENTS, address_of

MASTER_CSV = os.path.join("data", "boxes_master.csv")
ZONE_MAP_CSV = os.path.join("data", "zone_map.csv")

#: 실행 기록 표 — `--reset` 이 비우는 대상. 정답지(boxes_master·zone_map)는 아니다.
RUN_TABLES = ("boxes_sorted", "box_events", "scan_misses", "recirculation")


def dong_of(address):
    """주소에서 동을 뽑는다. 없으면 None.

    ★ 규칙은 하나뿐이다 — "동" 으로 끝나는 어절. 정규식으로 더 정교하게 만들
      이유가 없다. 주소를 만드는 쪽도 읽는 쪽도 이 파일 하나다.

    ★ gen_master_db 에서 여기로 옮겼다. 주소 *파싱*이지 생성이 아니다.
      gen_master_db 가 재수출하므로 기존 import 는 그대로 산다.
    """
    for tok in str(address or "").split():
        if len(tok) > 1 and tok.endswith("동"):
            return tok
    return None


def _rows(path):
    """`#` 주석과 빈 줄을 걷어낸 CSV 행. (줄번호, dict) 로 준다.

    ★ 줄번호를 들고 다니는 이유: 검산이 실패했을 때 **파일 어느 줄**인지 못 찍으면
      150 줄에서 눈으로 찾아야 한다.
    """
    with open(path, encoding="utf-8") as f:
        lines = [(i, ln) for i, ln in enumerate(f, 1)
                 if ln.strip() and not ln.lstrip().startswith("#")]
    if not lines:
        raise SystemExit(f"{path} 이 비어 있다")
    header = lines[0][1]
    body = "".join(ln for _i, ln in lines)
    rdr = csv.DictReader(body.splitlines())
    return header, [(lines[i + 1][0], r) for i, r in enumerate(rdr)]


def load_zone_map(path=None):
    """동 → 구역 배정표. **dict 가 아니라 raw 리스트**로 준다.

    ★ dict 로 주면 중복 동을 조용히 삼킨다. 중복은 "배정이 뒤죽박죽" 이라는 뜻이라
      반드시 걸려야 한다 — verify() 가 raw 를 받아 검사한 뒤에 dict 로 만든다.
    """
    p = path or find_data("packing_db", ZONE_MAP_CSV, __file__)
    _h, rows = _rows(p)
    return [(r["dong"].strip(), r["zone"].strip()) for _ln, r in rows]


def load_master(path=None):
    """정답지 150 행 → `(box_id, address, delivery_seq, zone, lat, lon)`.

    ★ delivery_seq 는 파일에 없다. **줄 순서**가 곧 투입 순서라 여기서 채운다.
      컬럼으로 두면 줄 순서와 어긋날 수 있는 자리가 하나 더 생긴다.
    """
    p = path or find_data("packing_db", MASTER_CSV, __file__)
    _h, rows = _rows(p)
    out = []
    for seq, (ln, r) in enumerate(rows):
        try:
            out.append((r["box_id"].strip(), r["address"].strip(), seq,
                        r["zone"].strip(), float(r["lat"]), float(r["lon"])))
        except (KeyError, TypeError, ValueError) as e:
            raise SystemExit(f"{p}:{ln} 을 읽지 못했다 ({e}) — 줄: {r!r}")
    return out


def verify(cfg, rows, zone_rows, path=None):
    """정답지가 config · zone_map · APARTMENTS 와 앞뒤가 맞는지. 어긋나면 SystemExit.

    ★ 매 로드마다 돈다. 옛 코드는 만들 때 한 번만 했다.
    """
    p = path or find_data("packing_db", MASTER_CSV, __file__)

    def die(msg, line=None):
        where = f"{p}:{line}" if line else p
        raise SystemExit(f"정답지 검산 실패 — {where}\n  {msg}\n"
                         f"  고치려면: ros2 run packing_db gen_master_db --regen")

    # ── 5. zone_map 에 중복 동 (dict 로 만들기 **전에** 본다) ──
    seen = {}
    for d, z in zone_rows:
        if d in seen:
            die(f"zone_map 에 동이 중복이다: {d!r} → {seen[d]!r} · {z!r}")
        seen[d] = z

    # ── 1. 행 수가 config 와 맞나 ──
    #    households_per_apt 를 바꾸고 --regen 을 잊으면 여기서 걸린다.
    per = int(cfg["boxes"]["stage2"]["households_per_apt"])
    apt_of_zone = {}
    for d, _n, _la, _lo in APARTMENTS:
        apt_of_zone.setdefault(seen.get(d), 0)
        apt_of_zone[seen.get(d)] += 1
    want = sum(apt_of_zone.get(z, 0) for z in cfg["zones"]["list"]) * per
    if len(rows) != want:
        die(f"행 수가 {len(rows)} 인데 config 상으로는 {want} 이어야 한다 "
            f"(아파트 {sum(apt_of_zone.get(z, 0) for z in cfg['zones']['list'])} 곳 "
            f"x households_per_apt {per})")

    # ── 3 준비: APARTMENTS 를 주소로 색인. 좌표는 6 자리로 비교한다 ──
    #    CSV 에 %.6f 로 적으므로 리터럴과 정확히 맞아야 한다.
    apt = {address_of(d, n): (round(la, 6), round(lo, 6))
           for d, n, la, lo in APARTMENTS}
    zones_ok = set(str(z) for z in cfg["zones"]["list"])

    for i, (bid, addr, _seq, zone, la, lo) in enumerate(rows):
        ln = i + 1                                  # 사람이 셀 때의 데이터 줄 번호
        # ── 2. box_id 가 줄 번호와 맞나 ──
        #    줄을 지우거나 중복하거나 정렬하면 조용히 번호가 밀리는 대신 여기서 죽는다.
        if bid != make_box_id(i):
            die(f"{i} 번째 줄의 box_id 가 {bid!r} 인데 {make_box_id(i)!r} 이어야 한다 "
                f"— 줄이 지워졌거나 순서가 바뀌었다", ln)
        # ── 8. 좌표가 있나 ──
        if la is None or lo is None:
            die(f"{bid} 에 좌표가 없다", ln)
        # ── 3. 주소·좌표가 APARTMENTS 와 같나 (드리프트 차단선) ──
        if addr not in apt:
            die(f"{bid} 의 주소 {addr!r} 가 delivery_points.APARTMENTS 에 없다", ln)
        if apt[addr] != (round(la, 6), round(lo, 6)):
            die(f"{bid} 의 좌표가 {(la, lo)} 인데 APARTMENTS 는 {apt[addr]} 다 "
                f"— delivery_points.py 를 고치고 --regen 을 잊었다", ln)
        # ── 6. zone 이 config 의 구역 목록 안에 있나 ──
        if zone not in zones_ok:
            die(f"{bid} 의 구역 {zone!r} 이 zones.list {sorted(zones_ok)} 에 없다", ln)
        # ── 4. 주소 → 동 → 구역이 되돌아오나 (살아남은 그 검산) ──
        d = dong_of(addr)
        if d is None:
            die(f"{bid} 의 주소 {addr!r} 에서 동을 못 뽑았다", ln)
        if seen.get(d) != zone:
            die(f"{bid}: 동 {d!r} 은 zone_map 상 {seen.get(d)!r} 인데 "
                f"정답지는 {zone!r} 이다 — 동 이름 오타면 전 박스가 RETURN 된다", ln)


def reset_run_tables(conn):
    """실행 기록 표를 비운다 → `{표이름: 지운 행수}`.

    ★ 정답지(boxes_master · zone_map)는 안 건드린다.
    ★ recirculation 도 비운다 — 재순환 횟수는 "이번 실행" 의 값이라야 한다.
    ★ sqlite_sequence 까지 지워 db_seq 를 1 부터 되돌린다.
      plan_route.py · pallet_access.py 가 `ORDER BY s.db_seq` 로 투입 순서를 본다.
    """
    out = {}
    for t in RUN_TABLES:
        out[t] = conn.execute(f"SELECT COUNT(*) FROM {t}").fetchone()[0]
        conn.execute(f"DELETE FROM {t}")
    conn.execute("DELETE FROM sqlite_sequence WHERE name IN "
                 "('boxes_sorted','box_events','scan_misses','recirculation')")
    conn.commit()
    return out


def run_table_counts(conn):
    """실행 기록이 얼마나 남아 있나 → `{표이름: 행수}`. 경고 문구용."""
    return {t: conn.execute(f"SELECT COUNT(*) FROM {t}").fetchone()[0]
            for t in RUN_TABLES}
