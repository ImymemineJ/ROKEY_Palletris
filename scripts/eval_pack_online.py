#!/usr/bin/env python3
"""온라인 패킹 결과를 **Isaac 없이** 재현한다 (트랙 A).

    python3 scripts/eval_pack_online.py                     # 사양 그대로
    python3 scripts/eval_pack_online.py --sweep             # 파라미터 훑기
    python3 scripts/eval_pack_online.py --clearance 30 --halo 20 -v

## 왜 필요한가

`demo_stack15_test1_1.py` 한 번이 Isaac 부팅 포함 4~5 분이다. 패킹 자리 선택을
고칠 때마다 그걸 돌리면 하루가 간다. 그런데 **자리 선택 자체는 물리가 필요 없다** —
`packing.HeightmapPacker` 는 하이트맵 하나만 보고 정한다. 그래서 도착 순서만 같으면
여기서 몇 초에 같은 답이 나온다.

물리가 필요한 것(흡착·이송·knock·실측 오차)은 여전히 Isaac 이 봐야 한다.
**여기서 통과했다고 실물이 통과하는 게 아니다.** 여기서 떨어지면 실물도 떨어진다는
쪽으로만 쓴다 — 빠른 기각용이다.

## 실물과 다른 점 하나

실제 온라인 경로는 매 박스마다 **비전이 본 실측 자세**로 하이트맵을 다시 세운다
(`demo_stack15.PalletVision.scan` -> `_obs_to_placement`). 여기서는 계획 좌표를
그대로 commit 한다 — 즉 **배치 오차와 knock 이 0 인 이상적인 경우**다.
실측(2026-08-23)에서 배치 오차는 최대 4.8 mm 라 자리 선택을 바꿀 정도는 아니지만,
knock 70 mm 가 난 뒤부터는 갈릴 수 있다. `--knock` 으로 그 상황을 흉내낼 수 있다.
"""
import argparse
import importlib.util
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
SPEC = ROOT / "src/packing_bringup/config/system.yaml"


def load_packer():
    """`src/packing_sim/packing.py` 를 sys.path 오염 없이 읽는다.

    `src/packing_sim` 을 통째로 넣으면 거기 `config.py` 가 다른 config 를 가린다.
    (`demo_stack15._load_packer_module` 과 같은 방법이다.)
    """
    if "_packing_hm" in sys.modules:
        return sys.modules["_packing_hm"]
    spec = importlib.util.spec_from_file_location(
        "_packing_hm", ROOT / "src/packing_sim/packing.py")
    mod = importlib.util.module_from_spec(spec)
    # ★ exec_module 전에 등록해야 한다. @dataclass 가 cls.__module__ 로
    #   sys.modules 를 되짚어 보기 때문이다.
    sys.modules["_packing_hm"] = mod
    spec.loader.exec_module(mod)
    return mod


def manifest(cfg, n, seed=None):
    """`demo_stack15.manifest` 와 **같은 난수열**이어야 한다 — numpy 쪽을 쓴다."""
    types = cfg["boxes"]["types"]
    rng = np.random.default_rng(cfg["boxes"]["random_seed"] if seed is None else seed)
    p = np.array(cfg["boxes"]["size_distribution"], dtype=float)
    return [types[int(i)] for i in rng.choice(len(types), int(n), p=p / p.sum())]


def make_packer(PK, cfg, clearance_mm, halo_mm):
    """인자는 `demo_stack15_test1_1.factory()` 와 같다. **클래스는 다르다.**

    ★ 그쪽은 팔 간섭 술어를 얹은 `ArmAwarePacker` 라 여기보다 자리를 **더 거부한다**.
      즉 여기 결과는 상한(낙관치)이다 — 파일 머리말대로 "여기서 떨어지면 실물도
      떨어진다" 쪽으로만 쓰고, 그 반대로는 쓰지 마라.
    """
    hm, pal = cfg["heightmap"], cfg["pallet"]
    return PK.HeightmapPacker(
        size_xy=(pal["width_mm"] / 1000.0, pal["depth_mm"] / 1000.0),
        max_height=pal["max_stack_mm"] / 1000.0,
        cell=hm["cell_mm"] / 1000.0,
        yaws_deg=tuple(float(v) for v in hm["allowed_yaw_deg"]),
        min_support_ratio=float(hm["min_support_ratio"]),
        support_tol=hm["support_tol_mm"] / 1000.0,
        wall_margin=0.0, enforce_load_order=True,
        descent_halo=halo_mm / 1000.0)


def run(cfg, PK, n, clearance_mm, halo_mm, order="arrival", verbose=False, seed=None):
    """도착 순서대로 한 개씩 자리를 찾아 commit 한다 = 온라인.

    Returns: dict(placed, dropped, layers, top_mm, rows)
    """
    boxes = manifest(cfg, n, seed)
    if order == "volume":
        # B5 오프라인이 쓰는 순서. 라인에서는 못 하지만 상한을 보려고 둔다.
        boxes = sorted(boxes, key=lambda t: -(t["w_mm"] * t["d_mm"] * t["h_mm"]))
    c = clearance_mm / 1000.0
    pk = make_packer(PK, cfg, clearance_mm, halo_mm)
    rows, dropped = [], []
    pal = cfg["pallet"]
    packed_m3 = 0.0                      # ★ clearance 를 뺀 **실제** 박스 부피
    for i, t in enumerate(boxes):
        size = (t["w_mm"] / 1000.0 + c, t["d_mm"] / 1000.0 + c, t["h_mm"] / 1000.0)
        p = pk.find_placement(size, t["mass_g"] / 1000.0)
        if p is None:
            dropped.append((i + 1, t["name"]))
            rows.append({"i": i + 1, "name": t["name"], "ok": False})
            if verbose:
                print("  [%2d] %-3s  자리 없음 (최상단 %.0f mm)"
                      % (i + 1, t["name"], pk.current_height * 1000))
            continue
        pk.commit(p)
        packed_m3 += t["w_mm"] * t["d_mm"] * t["h_mm"] / 1e9
        cx, cy, _ = p.center_local
        rows.append({"i": i + 1, "name": t["name"], "ok": True,
                     "x": cx * 1000, "y": cy * 1000, "z": p.z * 1000,
                     "yaw": p.yaw_deg, "sup": p.support_ratio})
        if verbose:
            print("  [%2d] %-3s  P(%4.0f,%4.0f) z%-4.0f yaw%-3.0f 지지 %.2f  최상단 %.0f"
                  % (i + 1, t["name"], cx * 1000, cy * 1000, p.z * 1000,
                     p.yaw_deg, p.support_ratio, pk.current_height * 1000))
    zs = sorted({round(r["z"]) for r in rows if r["ok"]})
    return {"placed": sum(1 for r in rows if r["ok"]), "n": len(boxes),
            "dropped": dropped, "layers": len(zs), "z_list": zs,
            "top_mm": pk.current_height * 1000,
            # ★ `pk.volume_utilization()` 을 쓰면 안 된다. 그 함수는 Placement.w/d 를
            #   적분하는데, 여기서는 clearance 30 mm 를 더한 치수로 배치를 잡으므로
            #   **부풀린 부피**가 들어간다 (실측 17~18 % 상대 과대: n=11 은 33.5 vs
            #   실제 28.5 %, n=15 는 44.3 vs 37.7 %). 적재율은 협의 안건 숫자라
            #   부풀면 곧장 결론이 틀어진다. 원 치수로 따로 적산한다.
            "util": packed_m3 / (pal["width_mm"] * pal["depth_mm"]
                                 * pal["max_stack_mm"] / 1e9),
            "rows": rows}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=0, help="0 = boxes.stage1.total_count")
    ap.add_argument("--clearance", type=float, default=30.0, help="이웃 간격 [mm]")
    ap.add_argument("--halo", type=float, default=40.0, help="하강 통로 [mm]")
    ap.add_argument("--order", choices=("arrival", "volume"), default="arrival")
    ap.add_argument("--sweep", action="store_true", help="clearance x halo 훑기")
    ap.add_argument("--seed", type=int, default=None,
                    help="boxes.random_seed 를 이 실행에서만 덮어쓴다 (사양 파일은 안 건드린다)")
    ap.add_argument("--scan-seeds", type=int, default=0, metavar="N",
                    help="seed 0..N-1 중 도착 순서로 전부 놓이는 것을 찾는다")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    cfg = yaml.safe_load(SPEC.read_text(encoding="utf-8"))
    PK = load_packer()
    n = a.n or int(cfg["boxes"]["stage1"]["total_count"])
    pal = cfg["pallet"]

    print("=" * 72)
    print("온라인 패킹 오프라인 재현 — 물리 없음")
    print("=" * 72)
    print("팔레트   %d x %d mm · 최대 적재 %d mm"
          % (pal["width_mm"], pal["depth_mm"], pal["max_stack_mm"]))
    print("박스     seed %d · %s · %d 개"
          % (cfg["boxes"]["random_seed"], cfg["boxes"]["size_distribution"], n))
    print("-" * 72)

    if a.sweep:
        print("%-10s %-8s %-8s %-7s %-9s %s"
              % ("clearance", "halo", "순서", "놓음", "단/최상단", "떨어진 것"))
        for order in ("arrival", "volume"):
            for cl in (30.0, 25.0, 20.0):
                for ha in (40.0, 30.0, 20.0, 10.0, 0.0):
                    r = run(cfg, PK, n, cl, ha, order)
                    print("%-10.0f %-8.0f %-8s %d/%-5d %d단/%-5.0f %s"
                          % (cl, ha, order, r["placed"], r["n"], r["layers"],
                             r["top_mm"], [f"{i}:{t}" for i, t in r["dropped"]] or "-"))
        return 0

    if a.scan_seeds:
        # ★ 사양의 seed 42 는 "고정 — 베이스라인 비교 필수" 다. 여기서 찾는 seed 는
        #   그 값을 바꾸자는 게 아니라, **도착 순서만으로 전부 들어가는 상자 조합이
        #   존재하는가**를 확인하고 시연용 실행에 쓰기 위한 것이다.
        print("seed 0~%d 중 도착 순서로 %d/%d 가 되는 것을 찾는다 "
              "(clearance %.0f · halo %.0f)\n"
              % (a.scan_seeds - 1, n, n, a.clearance, a.halo))
        hits = []
        for sd in range(a.scan_seeds):
            r = run(cfg, PK, n, a.clearance, a.halo, "arrival", seed=sd)
            if r["placed"] == r["n"]:
                mix = [t["name"] for t in manifest(cfg, n, sd)]
                hits.append((sd, r, mix))
                print("  seed %-4d  %d/%d  %d단 / 최상단 %.0f mm  적재율 %.1f%%  %s"
                      % (sd, r["placed"], r["n"], r["layers"], r["top_mm"],
                         r["util"] * 100, "".join(m[0] for m in mix)))
        print("\n찾은 것 %d 개 / %d" % (len(hits), a.scan_seeds))
        if hits:
            best = max(hits, key=lambda h: -h[1]["top_mm"])
            print("가장 낮게 쌓이는 것: seed %d (최상단 %.0f mm, %d단)"
                  % (best[0], best[1]["top_mm"], best[1]["layers"]))
            print("구성: %s" % best[2])
        return 0 if hits else 1

    r = run(cfg, PK, n, a.clearance, a.halo, a.order, verbose=True, seed=a.seed)
    print("-" * 72)
    print("놓은 것  %d / %d        떨어진 것 %s"
          % (r["placed"], r["n"], [f"{i}번 {t}" for i, t in r["dropped"]] or "없음"))
    print("단       %d 단  밑면 z %s mm" % (r["layers"], r["z_list"]))
    print("최상단   %.0f mm  (한계 %d)" % (r["top_mm"], pal["max_stack_mm"]))
    print("적재율   %.1f %%  (clearance 제외한 순 박스 부피 / 팔레트 가용 부피)"
          % (r["util"] * 100))
    print("=" * 72)
    return 0 if r["placed"] == r["n"] else 1


if __name__ == "__main__":
    sys.exit(main())
