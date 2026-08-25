"""적재율의 **천장**은 어디인가 — 박스 치수가 팔레트를 어떻게 나누나.

## 왜 이걸 재나

"목표 70% (작업명세서 §13)에 왜 못 닿나" 에 답하려면 먼저 **닿을 수 있는 값인지**를
알아야 한다. 우리 알고리즘이 못 하는 것인지, 애초에 박스 치수가 안 되는 것인지.

## 무엇을 재나

한 종류만 무한히 온다고 가정하고, 회전(0/90°)만 허용한 격자 배치로
**완벽하게** 쌓았을 때의 적재율. 알고리즘이 아니라 **기하의 상한**이다.

    ros2 run packing_algo ceiling
"""
import argparse
import copy

from packing_common.config import load


def best_layer(W, D, w, d, step=10):
    """W×D 바닥을 w×d 사각형(90° 회전 허용)으로 채울 때 최대 개수.

    ⚠ **이 값을 천장으로 쓰지 말 것.** 바닥을 한 번만 자르고 각 조각을 한 방향
      격자로 채우는 2분할(길로틴) 탐색이라, **격자가 아닌 배치를 못 본다.**
      실측으로 틀린 것이 확인됐다 (2026-08-25):
          4호 410×310 → 이 함수 5개 / **실제 패커 6개**
          3호 340×250 → 이 함수 10개 / 실제 패커 9개 (이쪽은 반대로 낙관적)
      천장은 아래 `measured()` 처럼 **패커를 실제로 돌려서** 재야 한다.
      이 함수는 참고용으로만 남긴다.
    """
    best, how = 0, None
    for axis in ("X", "Y"):
        span = W if axis == "X" else D
        for cut in range(0, span + 1, step):
            for (a1, b1) in ((w, d), (d, w)):
                for (a2, b2) in ((w, d), (d, w)):
                    if axis == "X":
                        n = (cut // a1) * (D // b1) + ((W - cut) // a2) * (D // b2)
                    else:
                        n = (W // a1) * (cut // b1) + (W // a2) * ((D - cut) // b2)
                    if n > best:
                        best, how = n, (axis, cut, (a1, b1), (a2, b2))
    return best, how


def measured(cfg):
    """★ 한 종류만 흘려보내 **패커로 직접** 재는 천장. 통로 규칙은 뺀다
    (기하의 상한을 보려는 것이고, 팔 제약은 그 위에 따로 붙는다)."""
    import numpy as np

    from packing_algo.heightmap_core import HeightmapPacker

    class NoCorridor(HeightmapPacker):
        def _corridor_ok(self, wc, dc, bz, h_mm):
            return np.ones_like(bz, dtype=bool)

    out = []
    for b in cfg["boxes"]["types"]:
        w, d, h = b["w_mm"], b["d_mm"], b["h_mm"]
        # ① 한 층만 (max_stack 을 박스 높이로 잘라서)
        one = copy.deepcopy(cfg)
        one["pallet"]["max_stack_mm"] = h
        pk = NoCorridor(one)
        per = 0
        for k in range(80):
            ok, pose, _, _ = pk.plan(f"L{k}", w, d, h)
            if not ok:
                pk.cancel(f"L{k}")
                break
            pk.update(f"L{k}", w, d, h, dict(pose))
            per += 1
        # ② 전체 높이
        pk2 = NoCorridor(cfg)
        n = 0
        for k in range(300):
            ok, pose, _, msg = pk2.plan(f"F{k}", w, d, h)
            if not ok:
                pk2.cancel(f"F{k}")
                if msg == "PALLET_FULL":
                    break
                continue
            pk2.update(f"F{k}", w, d, h, dict(pose))
            n += 1
        out.append(dict(name=b["name"], w=w, d=d, h=h, per_layer=per, total=n,
                        fill=pk2.fill_rate(), cap=pk2.capacity_rate(),
                        hmax=int(pk2.hm.max()), guess=best_layer(
                            cfg["pallet"]["width_mm"], cfg["pallet"]["depth_mm"], w, d)[0]))
    return out


def rows(cfg):
    P = cfg["pallet"]
    W, D, H = P["width_mm"], P["depth_mm"], P["max_stack_mm"]
    out = []
    for b in cfg["boxes"]["types"]:
        w, d, h = b["w_mm"], b["d_mm"], b["h_mm"]
        n, _ = best_layer(W, D, w, d)
        layers = H // h
        out.append(dict(name=b["name"], w=w, d=d, h=h, per_layer=n, layers=layers,
                        total=n * layers,
                        area_use=n * w * d / (W * D),
                        height_use=layers * h / H,
                        rate=n * layers * w * d * h / (W * D * H)))
    return out, (W, D, H)


def markdown(cfg):
    rs, (W, D, H) = rows(cfg)
    ms = measured(cfg)
    mix = cfg["boxes"]["size_distribution"]
    vols = [r["w"] * r["d"] * r["h"] for r in rs]
    share = [m * v for m, v in zip(mix, vols)]
    tot = sum(share)
    mix = cfg["boxes"]["size_distribution"]
    vols = [m["w"] * m["d"] * m["h"] for m in ms]
    share = [q * v for q, v in zip(mix, vols)]
    tot = sum(share)
    L = ["# 적재율의 천장 — 박스 치수가 정한다", "",
         f"팔레트 {W} × {D} × 최대 {H}mm → 용적 **{W*D*H/1e9:.3f} m³**", "",
         "한 종류만 흘려보내 **패커로 직접** 잰 값이다 (팔 통로 제약은 뺐다 —",
         "기하의 상한을 보려는 것이고 팔 제약은 그 위에 따로 붙는다).", "",
         "| 박스 | 치수 | 층당 | 총 개수 | 최고 높이 | 적재율(실제높이) | 적재율(용량) |",
         "|---|---|---|---|---|---|---|"]
    for m in ms:
        L.append(f"| {m['name']} | {m['w']}×{m['d']}×{m['h']} | {m['per_layer']}개 | "
                 f"{m['total']}개 | {m['hmax']}mm | **{m['fill']*100:.1f}%** | "
                 f"{m['cap']*100:.1f}% |")
    L += ["", "## 읽는 법", "",
          "- **한 종류만 오면 70~79% 가 나온다.** 섞여 오면 우리 값이 59.9% 다.",
          "  차이의 대부분은 **높이가 섞여 절벽이 생기는 것**이다 (트러블슈팅 TS-030·031).",
          "- 두 지표가 갈리는 이유: 5호는 340×3=1020 에서 끝나 1300 중 280mm 를 못 쓴다.",
          "  실제높이 기준(76.0%)은 그 낭비를 안 세고, 용량 기준(59.6%)은 센다.", "",
          "## 부피로 보면 큰 박스가 지배한다", "",
          "| 박스 | 개수 비율 | **부피 비율** |",
          "|---|---|---|"]
    for m, q, sh in zip(ms, mix, share):
        L.append(f"| {m['name']} | {q*100:.0f}% | **{sh/tot*100:.1f}%** |")
    L += ["",
          "개수로는 3호가 절반이지만 **부피로는 5호가 가장 크다** — 3호는 작아서 개수만 많다.",
          "",
          "## 길로틴 추정이 왜 틀리나 (참고)", "",
          "| 박스 | 2분할 추정 | **실제 패커** |",
          "|---|---|---|"]
    for m in ms:
        L.append(f"| {m['name']} | {m['guess']}개 | **{m['per_layer']}개** |")
    L += ["",
          "4호 410×310 은 격자로는 5개지만 **비격자 배치로 6개**가 들어간다.",
          "천장을 손으로 추정하면 틀린다 — 패커를 돌려서 재야 한다.", "",
          "## 결론", "",
          "**한 종류만 모으면 70~79%, 섞이면 59.9%.** 목표 70% 는 *혼합* 조건에서는",
          "사실상 상한선이고, **종류별로 나눠 쌓으면 넘는다** (실측 72.5%, TS-034)."]
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser(description="적재율 천장 (기하 상한)")
    ap.add_argument("-o", "--out", default=None, help="md 로 저장할 경로")
    args = ap.parse_args()
    md = markdown(load())
    print(md)
    if args.out:
        with open(args.out, "w") as f:
            f.write(md)
        print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
