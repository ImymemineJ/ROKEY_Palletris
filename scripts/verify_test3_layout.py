"""test_3.usd 배치 게이트 — 씬을 **실측**해서 설정과 맞는지 본다.

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/verify_test3_layout.py

★ 왜 설정이 아니라 씬을 재나. 좌표의 정본은 `config/test_3.yaml` 이지만, 씬은
  사람이 GUI 로도 만진다. 둘이 어긋나면 로봇이 허공을 집거나 팔레트 밖에 놓는데,
  둘 다 **에러 없이** 일어난다. 그래서 bbox 로 직접 재서 대조한다.

★ 왜 `isaac-sim.sh` 인가. `python.sh` 헤드리스는 OmniGraph 를 안 돌려 GUI 에서만
  나는 문제를 숨긴다 (CLAUDE.md 와 같은 이유).
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "packing_sim", "isaac"))

from pxr import Usd, UsdGeom                                   # noqa: E402

STAGE = os.environ.get("PL_STAGE",
                       "/home/rokey/isaac_scenes/Collected_test_3/test_3.usd")
OVERLAY = os.environ.get("PL_CONFIG_OVERLAY",
                         os.path.join(ROOT, "src", "packing_bringup",
                                      "config", "test_3.yaml"))
REACH_MM = 1700.0                      # H2017 도달거리
PURPOSES = None                        # 아래에서 채운다

_fail = []


def chk(name, ok, detail=""):
    print("     %-30s %s %s" % (name, "OK" if ok else "X", detail))
    if not ok:
        _fail.append(name)


def main():
    global PURPOSES
    import yaml
    cfg = yaml.safe_load(open(OVERLAY, encoding="utf-8"))
    st = Usd.Stage.Open(STAGE)
    if st is None:
        raise SystemExit("씬을 못 열었다: %s" % STAGE)
    PURPOSES = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                UsdGeom.Tokens.proxy]

    def box(path):
        # ★ 캐시를 매번 새로 만든다. 재사용하면 값을 바꿔도 옛 bbox 가 나온다.
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), PURPOSES)
        p = st.GetPrimAtPath(path)
        if not p or not p.IsValid():
            return None
        r = bb.ComputeWorldBound(p).ComputeAlignedRange()
        if r.IsEmpty():
            return None
        a, b = r.GetMin(), r.GetMax()
        return [v * 1000.0 for v in (a[0], a[1], a[2], b[0], b[1], b[2])]

    pal = cfg["pallet"]
    ped = float(cfg["robot"]["pedestal_height_mm"])
    origins = cfg["zones"]["pallet_origin_world_mm"]
    bases = cfg["zones"]["robot_base_world_mm"]
    cross = cfg["zones"]["pickup_cross_mm"]
    max_stack = float((cfg.get("pallet") or {}).get("max_stack_mm", 1300))
    cells = {"A": "/ConveyorTrack_08", "B": "/ConveyorTrack_07",
             "C": "/World/ConveyorTrack_07"}
    merges = {"A": "/World/MergeLine_A", "B": "/World/MergeLine_B",
              "C": "/World/MergeLine_C"}

    print("씬     %s" % STAGE)
    print("설정   %s" % OVERLAY)
    print("페데스탈 %.0f · 팔레트 %.0f(X) x %.0f(Y) · 최대적재 %.0f"
          % (ped, pal["width_mm"], pal["depth_mm"], max_stack))

    for z in "ABC":
        o, b = origins[z], bases[z]
        print("")
        print("=== 존 %s   로봇 (%.0f, %.0f, %.0f) ===" % (z, b[0], b[1], b[2]))
        pb, cb, mb = box("/World/Pallet_%s" % z), box(cells[z]), box(merges[z])
        chk("팔레트 prim 있음", pb is not None)
        chk("셀 컨베이어 prim 있음", cb is not None)
        chk("합류 라인 prim 있음", mb is not None)
        if not (pb and cb and mb):
            continue

        # 설정 <-> 씬
        chk("팔레트 원점 = 설정", abs(pb[0] - o[0]) < 2 and abs(pb[1] - o[1]) < 2,
            "씬 (%.0f, %.0f) · 설정 (%.0f, %.0f)" % (pb[0], pb[1], o[0], o[1]))
        chk("팔레트 상면 = 설정", abs(pb[5] - o[2]) < 2,
            "씬 %.0f · 설정 %.0f" % (pb[5], o[2]))
        chk("팔레트 치수 = 설정",
            abs(pb[3] - pb[0] - pal["width_mm"]) < 2
            and abs(pb[4] - pb[1] - pal["depth_mm"]) < 2,
            "%.0f x %.0f" % (pb[3] - pb[0], pb[4] - pb[1]))

        rb = box("/World/robot_%s" % z)
        chk("로봇 베이스 z = 페데스탈", rb is not None and abs(rb[2] - ped) < 5,
            "" if rb is None else "씬 %.0f" % rb[2])

        # 도달
        far = math.dist((b[0], b[1], ped),
                        (pb[0], pb[1], pb[5] + max_stack))
        chk("최원점 <= 도달", far <= REACH_MM, "%.1f / %.0f" % (far, REACH_MM))
        pick_p = cfg["conveyor"]["stopper_x_mm"]
        h_max = max(t["h_mm"] for t in
                    (cfg.get("boxes", {}).get("types")
                     or [{"h_mm": 340}]))
        pick = math.dist((b[0], b[1], ped),
                         (b[0], pick_p, cfg["conveyor"]["belt_top_z_mm"] + h_max))
        chk("픽업점 <= 도달", pick <= REACH_MM, "%.1f" % pick)

        # 팔레트가 셀 컨베이어 위에 온전히
        chk("팔레트가 셀 컨베이어 위",
            cb[0] - 1 <= pb[0] and pb[3] <= cb[3] + 1,
            "셀 X %.0f~%.0f · 팔레트 %.0f~%.0f" % (cb[0], cb[3], pb[0], pb[3]))
        # 페데스탈 침범
        pedb = box("/World/Pedestal_%s" % z)
        chk("셀 컨베이어가 페데스탈 안 침범",
            pedb is None or cb[3] <= pedb[0] + 1,
            "" if pedb is None else "셀 끝 %.0f · 페데스탈 %.0f" % (cb[3], pedb[0]))
        # ── 셀 -> **커브** -> 합류 ────────────────────────────────
        # ★ 2026-08-25: 셀과 합류 사이에 코너 커브(ConveyorBelt_A02)가 들어왔다.
        #   예전처럼 "셀 끝 = 합류 시작" 을 보면 안 된다 — 둘은 이제 안 붙는다.
        #   봐야 하는 것은 **셀 -> 커브 -> 합류** 가 끊기지 않는가다.
        cvp = (cfg["conveyor"].get("curve_prim") or {}).get(z, "")
        vb = box(cvp + "/Rollers") if cvp else None
        if vb is None and cvp:
            vb = box(cvp)
        chk("커브 %s prim" % z, vb is not None, cvp or "설정 없음")
        if vb is not None:
            chk("셀 -> 커브 이어짐 (Y)", vb[1] <= cb[4] + 3,
                "셀 끝 %.0f · 커브 시작 %.0f" % (cb[4], vb[1]))
            chk("커브 -> 합류 이어짐 (X)", mb[0] <= vb[3] + 3,
                "커브 끝 %.0f · 합류 시작 %.0f" % (vb[3], mb[0]))
            chk("커브가 셀 X 를 덮는다", vb[0] <= cb[0] + 3 or vb[0] <= pb[0] + 3,
                "커브 X %.0f~%.0f · 셀 %.0f~%.0f" % (vb[0], vb[3], cb[0], cb[3]))
        # ★ 팔레트는 커브를 돌아 합류에 올라탄다. 합류가 직접 팔레트를 받을 필요는
        #   없고, **합류가 트럭까지 닿는지**만 보면 된다 (아래 트럭 절에서 본다).
        # 지선 <-> pickup_cross_mm
        spur = box({"A": "/World/ConveyorTrack_09/Rollers_01",
                    "B": "/World/ConveyorTrack_08/Rollers_01",
                    "C": "/World/ConveyorTrack_03/Rollers_01"}[z])
        chk("지선 X = pickup_cross_mm",
            spur is not None and abs(spur[0] - min(cross[z])) < 3
            and abs(spur[3] - max(cross[z])) < 3,
            "" if spur is None else "씬 %.0f~%.0f · 설정 %s"
            % (spur[0], spur[3], cross[z]))
        chk("스토퍼 Y = stopper_x_mm",
            spur is not None
            and abs(spur[4] - cfg["conveyor"]["stopper_x_mm"]) < 3,
            "" if spur is None else "씬 %.0f" % spur[4])

        # 분기점 지오메트리
        br = box("/World/Bridge_%s" % z)
        chk("브리지 판이 롤러면보다 낮다",
            br is not None and 795.0 <= br[5] < 800.0,
            "" if br is None else "상면 %.1f" % br[5])

    # 트럭
    print("")
    print("=== 트럭 ===")
    bed = box("/World/Truck_Bed")
    chk("적재함 prim 있음", bed is not None)
    if bed:
        # ★ 코너 턴테이블에서 90° 돌리면 적재함을 **가로지르는** 치수가 팔레트의
        #   depth(1200)가 아니라 width(800)로 바뀐다. 그래야 3 장이 1톤 규격
        #   2800 에 들어간다 (3 x 800 = 2400). 안 돌리면 3600 이라 못 들어간다.
        turn = bool(pal.get("turntable_before_truck"))
        cross = pal["width_mm"] if turn else pal["depth_mm"]
        print("     팔레트 회전 %s -> 적재함 가로 치수 %.0f mm"
              % ("함(턴테이블)" if turn else "안 함", cross))
        ys = []
        for z in "ABC":
            mb = box(merges[z])
            ys.append(((mb[1] + mb[4]) / 2.0 - cross / 2.0,
                       (mb[1] + mb[4]) / 2.0 + cross / 2.0))
        span = max(y[1] for y in ys) - min(y[0] for y in ys)
        chk("팔레트 3 장이 적재함 안",
            bed[1] - 1 <= min(y[0] for y in ys)
            and max(y[1] for y in ys) <= bed[4] + 1,
            "3 장 %.0f · 적재함 %.0f" % (span, bed[4] - bed[1]))
        # ★ 1톤 트럭 규격. 작업명세서 §3.2 의 2800 이 정본이다.
        want_bed = float((cfg.get("truck") or {}).get("bed_len_mm", 2800))
        chk("적재함 = 1톤 규격", abs((bed[4] - bed[1]) - want_bed) < 3,
            "%.0f (규격 %.0f)" % (bed[4] - bed[1], want_bed))
        ys.sort()
        overlap = [i for i in range(len(ys) - 1) if ys[i][1] > ys[i + 1][0] + 1]
        chk("팔레트끼리 안 겹친다", not overlap)
        for z in "ABC":
            mb = box(merges[z])
            chk("합류 %s 가 적재함까지" % z, abs(mb[3] - (-2400.0)) < 5,
                "%.0f" % mb[3])

    # ── 소터 ────────────────────────────────────────────────────
    # ★ 왜 게이트에 넣나. A47 의 본선 롤러에는 소터 자리만큼 **구멍**이 있다.
    #   소터 높이가 어긋나거나 속도가 0 이면 라인이 거기서 막히는데, 씬만 보면
    #   멀쩡해 보인다 (실측 2026-08-25: 세 번 연속 이걸로 막혔다).
    srt = (cfg["conveyor"].get("sorters") or {})
    rt = cfg.get("routing") or {}
    if srt:
        print("")
        print("=== 소터 ===")
        want_x = rt.get("sorter_x_mm") or {}
        for z in "ABC":
            sp = srt.get(z)
            if not sp:
                chk("소터 %s 설정" % z, False, "없다")
                continue
            sb = box(sp)
            jb = box(sp.rsplit("/", 1)[0] + "/Rollers")
            chk("소터 %s prim" % z, sb is not None)
            if sb is None or jb is None:
                continue
            # 롤러면과 같은 높이여야 한다. 7 mm 만 높아도 박스가 턱에 걸린다.
            chk("소터 %s 윗면 = 본선 롤러" % z, abs(sb[5] - jb[5]) < 0.5,
                "%.1f (롤러 %.1f)" % (sb[5], jb[5]))
            # 설정의 소터 구간이 실제 bbox 와 맞는지 — 어긋나면 라우터가
            # 엉뚱한 X 에서 꺾어 박스가 소터 밖에서 옆으로 밀린다.
            w = want_x.get(z)
            chk("소터 %s 구간 = 설정" % z,
                w is not None and abs(sb[0] - min(w)) < 3
                and abs(sb[3] - max(w)) < 3,
                "실측 %.0f~%.0f · 설정 %s" % (sb[0], sb[3], w))
            g = sp + "/ActionGraph"
            nd = st.GetPrimAtPath(g + "/conveyor_belt")
            chk("소터 %s IsaacConveyor" % z, nd.IsValid())
            if not nd.IsValid():
                continue
            tg = [str(x) for x in
                  nd.GetRelationship("inputs:conveyorPrim").GetTargets()]
            chk("소터 %s conveyorPrim" % z, tg == [sp + "/Sorter_physics"], "%s" % tg)
            # ★ 애셋 기본은 로컬 (1,0,0) = 월드 +X 인데 이 라인은 -X 로 흐른다.
            #   그대로면 소터가 본선과 맞서 밀어 박스가 소터 위에서 멎는다.
            d = nd.GetAttribute("inputs:direction").Get()
            chk("소터 %s direction = 본선 진행" % z,
                d is not None and tuple(d) == (-1.0, 0.0, 0.0), "%s" % (d,))
            sv = st.GetPrimAtPath(g + "/SorterSpeed").GetAttribute("inputs:value")
            chk("소터 %s SorterSpeed != 0" % z,
                sv is not None and abs(sv.Get() or 0.0) > 1e-6,
                "%.4f" % (sv.Get() if sv else 0.0))
        chk("라우팅이 소터를 쓴다", bool(rt.get("use_sorter")),
            "use_sorter=%r" % rt.get("use_sorter"))

    print("")
    print("=" * 70)
    if _fail:
        print("RESULT: 실패 — %d 항목  (%s)" % (len(_fail), ", ".join(_fail)))
    else:
        print("RESULT: 통과 — 전 항목")
    print("=" * 70)
    sys.stdout.flush()
    os._exit(1 if _fail else 0)


main()
