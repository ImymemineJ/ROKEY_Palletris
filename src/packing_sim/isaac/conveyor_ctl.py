"""벨트 속도 제어 — `set_velocity` 핸들러 (공통가이드 §3, 작업명세서 §2.2). **김학민 소유.**

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : ③ 소터 분기 구동 · ⑤ 벨트 정지 · ⑩ 재가동
    ║     불러 주는 쪽        : 관제탑(/conveyor/set_velocity) · sim_main
    ║     이 파일이 부르는 쪽 : OmniGraph 벨트 변수 (mainq 경유)
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

    cc = ConveyorCtl(stage, cfg)
    server.register(cc.handlers())

seg 는 프로토콜 표 그대로 `"main" | "b_pickup"` 이다 (stage 1 은 이 둘뿐이다).
seg -> 그래프 경로는 `layout_to_spec_test1_1.py` 의 TRACKS/SEG_KEYS 와 짝이다.
한쪽만 고치면 벨트가 조용히 안 선다.

## ★ 월드 속도 = `graph:variable:Velocity` x 그 조각의 `xformOp:scale.x`

가이드(§A6~A8)의 예시는 요청 mps 를 변수에 그대로 쓴다. 그러면 조각이
스케일된 만큼 빨라진다 — 실측 2026-08-22: scale.x 2.5161 · Velocity 0.5 에서
월드 1.2588 m/s (이득 2.518 = scale.x). 0.5 를 요청했는데 1.26 으로 도는 것이다.
그래서 여기서는 **조각마다 자기 scale.x 로 나눠서** 써넣는다.

조각별 scale.x 가 다르다는 점이 중요하다. 사양 §3.2 대로 main 7500 · b_pickup 2500
으로 나누면 두 조각의 scale.x 가 3 배 차이 나므로, 한 값을 양쪽에 쓰면 픽업 벨트가
본선의 3 배로 돈다.

## 왜 `Velocity` 변수 **하나만** 쓰나

`IsaacConveyor` 노드가 이 그래프 변수를 읽어 `physxSurfaceVelocity` 를 authoring 한다.
변수를 0 으로 두면 노드가 표면속도를 **아예 안 쓰므로** 벨트가 선다.

⚠ 이 정지는 **박스를 세우는 수단이 아니다.** 박스는 씬의 스토퍼 벽이 세운다
(`layout_to_spec_test1_1.py`, 설계결정서 §4.4). 정지는 흡착 직전 떨림을 없애는
용도다 — `cycle.py:_run` 이 present 를 받은 뒤 b_pickup 을 끊는다. 한때 벽 없이
정지만으로 세우려던 판이 있었는데, `pickup_check` 는 박스가 *이미 멎어야* present 를
주고 `cycle.py` 는 present 를 받아야 벨트를 세우므로 서로를 기다려 아무도 못 세웠다.

**표면속도를 여기서 직접 쓰지 않는다.** 한때 "그래프가 안 도는 헤드리스 대비" 로
`physxSurfaceVelocity` 를 같이 써넣었는데, 진리원이 둘이 되기만 하고 이득이 없었다.
그래프가 도는 환경에서는 노드가 매 틱 덮어쓰므로 무의미하고, 안 도는 환경에서는
아래 caveat 대로 어차피 못 믿는다.

## ★★ 벨트 속도는 **`isaac-sim.sh` 로만** 검증한다 (실측 2026-08-22)

같은 씬·같은 `Velocity` 값(main 0.132482 · b_pickup 0.397447)인데 런처에 따라 다르다:

    isaac-sim.sh --no-window          main 0.500 · b_pickup 0.501 m/s   ✓ 사양
    python.sh (render=True)           약 1.50 ~ 1.54 m/s  (약 3 배, 실행마다 다름)
    python.sh (render=False)          약 12.7 m/s  (그래프가 아예 안 돔)

`IsaacConveyor` 는 `OnTick` 의 **프레임 delta** 로 이송량을 적분한다. standalone 은
매 스텝 렌더까지 하느라 프레임이 실시간보다 느려서 delta 가 부풀고, 그만큼 벨트가
더 나간다. **실행마다 값이 달라지는 것 자체가 프레임률 의존의 증거다.**

  · 이송속도가 중요한 시험(투입 간격·버퍼·사이클 타임)은 `isaac-sim.sh` 로 할 것.
    게이트: `scripts/verify_belt_speed.py`
  · `sim_main.py` standalone 은 **정지(0)가 정확하다** — 실측으로 박스가 x=0.501 에
    완전히 멎었다. B4 사이클이 실제로 의존하는 것은 "벨트를 세워 브레이크로 쓴다"
    쪽이라 픽앤플레이스 검증에는 지장이 없다.
"""
# ★ test1-1 기본값. 값은 그래프 경로 하나 또는 **여러 개의 리스트**다.
#   씬이 다르면 `conveyor.belt_graphs` 로 통째로 갈아끼운다 (아래 __init__ 참조).
#   한 seg 가 여러 조각인 경우가 있다 — test_3 의 본선은 4 조각을 이어 붙였고,
#   4 개를 한꺼번에 세워야 박스가 조각 경계에서 끌려가지 않는다.
BELT_GRAPH = {
    "main": "/World/ConveyorTrack/ConveyorBeltGraph",
    "b_pickup": "/World/ConveyorTrack1/ConveyorBeltGraph",
}
VEL_VAR = "graph:variable:Velocity"

# ★ 소터(ConveyorBelt_A47)의 속도는 그래프 **변수**가 아니라 `SorterSpeed`
#   ConstantFloat 노드에 들어 있다. 본선(`graph:variable:Velocity`)과 경로가
#   다르므로 따로 다룬다. 값은 `conveyor.sorters` 로 씬마다 준다 (test1-1 은 없다).
SORTER_GRAPH = "ActionGraph"          # <sorter>/ActionGraph
SORTER_NODE = "conveyor_belt"         # IsaacConveyor 노드
SORTER_SPEED = "SorterSpeed"          # ConstantFloat — inputs:value 가 속도
SORTER_REROUTE = "reroute"            # ConstantBool — 애셋의 분류 스위치
SORTER_PHYSICS = "Sorter_physics"     # conveyorPrim

# ConveyorBelt_A05 롤러 1 스케일당 길이 (m). layout_to_spec_test1_1.py 와 같은 상수다.
ROLLER_LEN_PER_SCALE = 1.98724



class ConveyorCtl:
    """시나리오 ③(소터 분기) · ⑤(벨트 정지) · ⑩(재가동)을 맡는 씬 핸들러.

    ★ **월드 속도 = `graph:variable:Velocity` × 그 조각의 `xformOp:scale.x`** 다.
      요청 m/s 를 변수에 그대로 쓰면 조각이 스케일된 만큼 빨라진다 —
      실측: scale.x 2.5161 · Velocity 0.5 에서 월드 **1.2588 m/s**.
      그래서 조각마다 **자기 scale.x 로 나눠서** 써넣는다.
      조각별 scale.x 가 3배까지 다르므로, 한 값을 양쪽에 쓰면 픽업 벨트가 본선의
      3배로 돈다.

    ★ 한 `seg` 에 조각이 여러 개면 **동시에** 켜고 끈다. 본선 4조각 중 하나만
      안 서면 박스가 조각 경계에서 계속 끌려간다.

    ★ USD 쓰기는 전부 `mainq` 를 거친다 — 소켓/워커 스레드에서 직접 쓰면
      메인의 `world.step()` 과 겹쳐 **행이 걸리거나 스테이지가 깨진다.**
    """

    def __init__(self, stage, cfg, mainq, log=print):
        self.stage = stage
        self.cfg = cfg
        # ★ USD 쓰기는 반드시 메인 스레드에서. 소켓 스레드에서 바로 Set() 하면
        #   world.step() 과 겹쳐 응답이 영영 안 오거나 프로세스가 죽는다
        #   (실측 2026-08-22, mainq.py docstring 참조).
        self.mainq = mainq
        self._log = log
        self.nominal = float(cfg["conveyor"]["belt_velocity_mps"])
        self._logged = set()          # 데크 배율은 seg 당 한 번만 찍는다

        # ── seg -> 그래프 경로 목록 ────────────────────────
        # ★ 왜 설정으로 뺐나. test_3 은 라인이 3 갈래(지선·셀·합류)라 seg 가 10 개다.
        #   여기에 하드코딩하면 씬이 하나 늘 때마다 이 파일을 고쳐야 하고, 그러면
        #   김학민 소유 파일을 남이 계속 건드리게 된다. 기본값은 test1-1 그대로다.
        table = cfg["conveyor"].get("belt_graphs") or BELT_GRAPH
        self.graphs = {}
        # 마지막으로 지시한 seg 별 속도 [m/s]. `speed_of()` 참조.
        self._cur = {}
        for seg, v in table.items():
            self.graphs[seg] = [v] if isinstance(v, str) else list(v)

        # ── 조각별 이득(gain) ──────────────────────────────
        # ★ 이전에는 그래프 부모 Xform 의 scale.x 를 썼다. test_3 에서 깨진다.
        #   지선(Rollers_01)은 부모 Xform 을 형제 조각과 **공유**하면서 자기만
        #   90도 돌아 있어, 부모 scale.x(1.7387)가 이 조각의 진행방향 배율
        #   (실측 1.6621)과 4.6% 어긋난다.
        #   그래서 **롤러 bbox 를 진행방향으로 재서** 배율을 구한다.
        #   test1-1 본선으로 검산: 4648mm / 1987.24 = 2.3390 = 옛 scale.x. 일치한다.
        self.gain = {}
        for seg, paths in self.graphs.items():
            ok = []
            for gpath in paths:
                g = stage.GetPrimAtPath(gpath)
                if not g or not g.IsValid():
                    self._log("[conveyor_ctl] !! 그래프 %s 가 씬에 없다 — seg %r"
                              % (gpath, seg))
                    continue
                gain = self._measure_gain(gpath)
                if gain is None:
                    continue
                ok.append(gpath)
                self.gain[gpath] = gain
                self._log("[conveyor_ctl] seg %-9s %-46s 진행 %s 배율 %.4f "
                          "(Velocity %.4f = %.2f m/s)"
                          % (seg, gpath, self._axis.get(gpath, "?"), gain,
                             self.nominal / gain, self.nominal))
            self.graphs[seg] = ok
        # 조각이 하나도 안 잡힌 seg 는 지운다 — set_velocity 가 조용히 성공하면 안 된다.
        for seg in [s for s, v in self.graphs.items() if not v]:
            self._log("[conveyor_ctl] !! seg %r 은 쓸 수 있는 조각이 없다" % seg)
            del self.graphs[seg]

        # ── 소터 (ConveyorBelt_A47) ────────────────────────
        # ★ 왜 본선과 따로인가. 소터는 분기 조각 안의 **독립 컨베이어**다.
        #   본선 롤러에는 로컬 X 1651~2555 (월드 약 1571 mm) 구멍이 있고 그
        #   자리를 소터가 받친다 — 그래서 소터를 안 돌리면 라인이 막힌다.
        #   대신 꺾이는 범위가 본선 조각(6925 mm)이 아니라 소터 구간뿐이라,
        #   분류하는 동안 뒤차가 본선을 그대로 흘러온다.
        # ── 배율을 안 먹이는 벨트 (원시값) ────────────────
        # ★ 왜 필요한가. 배율 환산(`_measure_gain`)이 안 맞는 조각이 있다.
        #   실측 2026-08-25: 순환 루프 `/World/ConveyorTrack_05` 는 배율 8.9995 로
        #   나오는데 그 값으로 환산한 0.0778 을 넣으면 박스가 2 초에 4 mm 밖에
        #   안 간다. 씬에 원래 있던 0.2 로 두면 약 1.4 m/s 로 잘 흐른다.
        #   그런 조각은 **환산하지 말고 원시값을 그대로** 써넣는다.
        #   여기서 한 번 써넣고, 그 뒤로는 아무도 안 건드린다 (루프는 여닫을 일이 없다).
        for gp, v in (cfg["conveyor"].get("belt_velocity_raw") or {}).items():
            g = stage.GetPrimAtPath(gp)
            a2 = g.GetAttribute(VEL_VAR) if (g and g.IsValid()) else None
            if not a2:
                self._log("[conveyor_ctl] !! 원시속도: %s 에 %s 가 없다" % (gp, VEL_VAR))
                continue
            a2.Set(float(v))
            self._log("[conveyor_ctl] 원시   %-46s Velocity %.4f (배율 환산 안 함)"
                      % (gp, float(v)))

        # ── 순환 루프 커브/직선 방향 보정 ─────────────────
        loop_cfg = (cfg.get("conveyor") or {}).get("loop") or {}
        for key in ("curve_dir", "straight_dir"):
            for tp, d in (loop_cfg.get(key) or {}).items():
                nd = stage.GetPrimAtPath(tp + "/ConveyorBeltGraph/ConveyorNode") or stage.GetPrimAtPath(tp + "/ConveyorNode")
                if nd and nd.IsValid():
                    from pxr import Gf
                    nd.GetAttribute("inputs:direction").Set(Gf.Vec3f(*[float(x) for x in d]))
                    self._log("[conveyor_ctl] 루프 방향   %-46s direction %s" % (tp, tuple(d)))

        # 기본 진행/분류 방향 — 부르는 쪽이 world 를 안 주면 이걸 쓴다.
        _rt = cfg.get("routing") or {}
        self.thru_world = list(_rt.get("thru_world") or [-1, 0, 0])
        self.divert_world = list(_rt.get("divert_world") or [0, 1, 0])
        # ★ 소터는 **본선의 일부**다 (본선 롤러에 소터 자리만큼 구멍이 있다).
        #   그래서 이 seg 의 속도를 바꿀 때 소터 속도도 같이 따라간다.
        #   방향은 안 건드린다 — 분류 중에 본선을 잠깐 세웠다 켜도 하던 분류가
        #   그대로 이어져야 하기 때문이다.
        self.sorter_follow_seg = str(cfg["conveyor"].get("sorter_follow_seg")
                                     or "main")
        self.sorters = {}
        self.sorter_gain = {}
        for zn, spath in (cfg["conveyor"].get("sorters") or {}).items():
            g = spath + "/" + SORTER_GRAPH
            node = self.stage.GetPrimAtPath(g + "/" + SORTER_NODE)
            conv = self.stage.GetPrimAtPath(spath + "/" + SORTER_PHYSICS)
            if not node or not node.IsValid() or not conv or not conv.IsValid():
                self._log("[conveyor_ctl] !! 소터 %s (%s) 가 씬에 없다" % (zn, spath))
                continue
            gain = self._measure_gain(g, SORTER_NODE)
            if gain is None:
                continue
            self.sorters[str(zn)] = spath
            self.sorter_gain[str(zn)] = gain
            self._log("[conveyor_ctl] 소터 %-2s %-44s 진행 %s 배율 %.4f "
                      "(SorterSpeed %.4f = %.2f m/s)"
                      % (zn, spath, self._axis.get(g, "?"), gain,
                         self.nominal / gain, self.nominal))

    def _measure_gain(self, gpath, node_name="ConveyorNode"):
        """그래프가 미는 롤러의 **진행방향 배율**.

        `IsaacConveyor` 는 direction 을 conveyorPrim 의 **로컬 축**으로 해석하고,
        표면속도는 그 조각의 월드 변환만큼 배로 커진다. 그러니 배율은
        `로컬 진행방향 벡터 x 월드행렬` 의 **길이**다. 방향축은 그 벡터의 최대성분.

        ⚠ 롤러 bbox 길이로 재면 틀린다. test_3 의 본선 조각은 애셋 2 개 길이를
          한 Xform 에 담고 있어 bbox 를 1 스케일당 길이로 나누면 배율이 2 배로
          나온다. 배율은 "몇 개를 이어 붙였나" 가 아니라 "얼마나 늘렸나" 다.
        검산 — test1-1 본선: |(1,0,0) x M| = 2.3390 = 옛 scale.x. 일치한다.
        """
        from pxr import UsdGeom
        node = self.stage.GetPrimAtPath(gpath + "/" + node_name)
        if not node or not node.IsValid():
            self._log("[conveyor_ctl] !! %s 에 %s 가 없다" % (gpath, node_name))
            return None
        tg = node.GetRelationship("inputs:conveyorPrim").GetTargets()
        if not tg:
            self._log("[conveyor_ctl] !! %s 의 conveyorPrim 이 비었다" % gpath)
            return None
        roll = self.stage.GetPrimAtPath(tg[0])
        if not roll or not roll.IsValid():
            # ★ 실제로 났던 사고다. GUI 복제 후 원본을 지우면 관계가 죽은 경로를
            #   가리킨 채 남는다. 벨트는 **에러 없이** 안 돈다 — 크게 알린다.
            self._log("[conveyor_ctl] !! %s 의 conveyorPrim %s 가 씬에 없다 "
                      "(복제 후 관계 미수정)" % (gpath, tg[0]))
            return None
        # ★★ 커브 벨트(`inputs:curved = True`)는 배율을 먹이면 안 된다.
        #   커브에서 `direction` 은 진행 방향이 아니라 **회전축**이다 (A02/A11 은
        #   (0,0,-37) 로 들어 있다). 그대로 이 식에 넣으면 배율이 38.5 로 나와
        #   0.7 m/s 를 요청했는데 0.018 이 써진다 — 벨트가 사실상 선다.
        #   커브는 velocity 를 **그대로** 쓴다 (씬에 이미 들어 있는 값과 같은 단위).
        cv = node.GetAttribute("inputs:curved")
        if cv and cv.Get() is True:
            self._axis = getattr(self, "_axis", {})
            self._axis[gpath] = "커브"
            return 1.0
        d = node.GetAttribute("inputs:direction").Get()
        if d is None:
            self._log("[conveyor_ctl] !! %s 에 direction 이 없다" % gpath)
            return None
        m = UsdGeom.XformCache().GetLocalToWorldTransform(roll)
        M = m.ExtractRotationMatrix()      # 좌상단 3x3 — 스케일이 들어 있다
        wv = [sum(float(d[k]) * float(M[k][j]) for k in range(3)) for j in range(3)]
        gain = (wv[0] ** 2 + wv[1] ** 2 + wv[2] ** 2) ** 0.5
        if gain <= 0.0:
            raise ValueError("%s 의 진행방향 배율이 %r 이다" % (gpath, gain))
        ax = "XYZ"[max(range(3), key=lambda k: abs(wv[k]))]
        sgn = "+" if wv[max(range(3), key=lambda k: abs(wv[k]))] > 0 else "-"
        self._axis = getattr(self, "_axis", {})
        self._axis[gpath] = sgn + ax
        return gain

    # ── 핸들러 ──────────────────────────────────────────────
    def speed_of(self, seg):
        """마지막으로 지시한 속도 [m/s]. 지시한 적 없으면 None.

        ★ 라우터가 "그 구역 지선이 서 있나" 를 보는 데 쓴다. 서 있는데 꺾어 넣으면
          박스가 지선 입구(Y 0.7 언저리)에서 멎었다가 45 s 뒤 본선으로 되돌아온다
          (실측 2026-08-26: `BOX004 가 45 s 안에 지선 C 에 못 올라갔다 (Y 0.68)`).
          지선을 세우는 것은 관제탑이다 — 집기 전에 반드시 세운다 (§2.5).
        """
        return self._cur.get(seg)

    def handlers(self):
        """이 모듈이 제공하는 씬 명령표. 소켓과 ROS 서비스가 같은 함수를 공유한다."""
        return {"set_velocity": self.h_set_velocity,
                "set_direction": self.h_set_direction,
                "set_sorter": self.h_set_sorter}

    def h_set_velocity(self, req):
        """{"cmd":"set_velocity","seg":"b_pickup","mps":0.0} -> {"ok":..,"actual":..}"""
        seg = req.get("seg")
        
        # ★ mps 가 None 이거나 전달되지 않았을 때 0.0 로 예외 처리
        raw_mps = req.get("mps")
        if raw_mps is None:
            raw_mps = 0.0

        try:
            mps = float(raw_mps)
        except (TypeError, ValueError):
            return {"ok": False, "error": "bad mps", "actual": 0.0}

        # 인자 검사는 소켓 스레드에서 끝내고, USD 쓰기만 메인 스레드로 넘긴다.
        if seg not in self.graphs:
            return {"ok": False, "error": "unknown seg %r (있는 것: %s)"
                    % (seg, sorted(self.graphs)), "actual": 0.0}
        return self.mainq.run(lambda: self.set_velocity(seg, mps))

    def h_set_direction(self, req):
        """{"cmd":"set_direction","seg":"main_a","world":[0,1,0]} -> {"ok":..}

        ★ 위치변환 레일. 그 구간 롤러의 **표면속도 방향**을 바꾼다.
        분기점마다 짧은 전환 벨트를 새로 넣는 대신 이렇게 하는 이유:
        test_3 의 본선 조각은 지선과 프레임 메시(`SM_ConveyorBelt_A23`)를
        **하나로 공유**한다. L 자 한 덩어리라 분기점 부분만 떼어낼 수 없고,
        떼어내려면 지선의 측면 레일까지 같이 사라진다.
        다행히 본선 조각 하나에 분기점이 **정확히 하나씩** 있어서, 그 조각을
        통째로 꺾으면 그게 곧 위치변환 레일이다.

        ⚠ 꺾는 동안 그 조각 위의 **모든** 박스가 같이 옆으로 간다.
          부르는 쪽이 그 조각에 목표 박스 하나만 있도록 보장해야 한다.
        """
        seg = req.get("seg")
        w = req.get("world")
        if seg not in self.graphs:
            return {"ok": False, "error": "unknown seg %r (있는 것: %s)"
                    % (seg, sorted(self.graphs))}
        try:
            wv = [float(w[0]), float(w[1]), float(w[2])]
        except (TypeError, ValueError, IndexError, KeyError):
            return {"ok": False, "error": "bad world %r" % (w,)}
        if wv == [0.0, 0.0, 0.0]:
            return {"ok": False, "error": "world 가 영벡터다"}
        return self.mainq.run(lambda: self.set_direction(seg, wv))

    def set_direction(self, seg, wv):
        """월드 방향을 각 조각의 **로컬 축**으로 환산해 direction 에 써넣는다."""
        from pxr import UsdGeom
        paths = self.graphs.get(seg)
        if not paths:
            return {"ok": False, "error": "unknown seg %r" % (seg,)}
        for gpath in paths:
            node = self.stage.GetPrimAtPath(gpath + "/ConveyorNode")
            if not node or not node.IsValid():
                return {"ok": False, "error": "%s 에 ConveyorNode 가 없다" % gpath}
            tg = node.GetRelationship("inputs:conveyorPrim").GetTargets()
            roll = self.stage.GetPrimAtPath(tg[0]) if tg else None
            if not roll or not roll.IsValid():
                return {"ok": False, "error": "%s 의 conveyorPrim 이 없다" % gpath}
            from pxr import Gf
            node.GetAttribute("inputs:direction").Set(
                Gf.Vec3f(*self._local_axis(roll, wv)))
        return {"ok": True, "world": wv}

    @staticmethod
    def _local_axis(roll, wv):
        """월드 방향 `wv` 를 `roll` 의 **로컬 단위축**으로 환산한다.

        `IsaacConveyor.inputs:direction` 은 conveyorPrim 의 로컬축이다.
        월드행렬에 스케일이 섞여 있으므로 각 축을 정규화한 뒤 투영하고,
        가장 큰 성분 하나만 남긴다 (벨트는 축 방향으로만 민다).
        """
        from pxr import UsdGeom
        M = UsdGeom.XformCache().GetLocalToWorldTransform(roll) \
            .ExtractRotationMatrix()
        loc = []
        for k in range(3):
            row = [float(M[k][j]) for j in range(3)]
            n = (row[0] ** 2 + row[1] ** 2 + row[2] ** 2) ** 0.5 or 1.0
            loc.append(sum(wv[j] * row[j] for j in range(3)) / n)
        i = max(range(3), key=lambda k: abs(loc[k]))
        out = [0.0, 0.0, 0.0]
        out[i] = 1.0 if loc[i] > 0 else -1.0
        return out

    def h_set_sorter(self, req):
        """{"cmd":"set_sorter","zone":"A","divert":true} -> {"ok":..,"world":[..]}

        ★ 소터로 분류한다. `set_direction` 과 무엇이 다른가:

          `set_direction` 은 **본선 조각 전체(6925 mm)** 의 표면속도를 꺾는다.
          그래서 그 조각 위에 박스가 둘이면 못 꺾고, 못 꺾인 박스는 순환 루프를
          돌아 다시 와서 더 몰린다. 투입 간격을 20 s 까지 벌려야 했던 이유다.

          소터는 분기 조각 안의 **독립 컨베이어**라 꺾이는 범위가 소터 구간
          (약 1492 mm) 뿐이다. 분류하는 동안 뒤차는 본선을 그대로 흘러온다.

        ★ 왜 애셋의 `reroute` 스위치를 안 쓰나. A47 안에 분류 배선이
          반쯤 들어 있는데(`reroute`->`to_float`->x90°->`rotate_vector`)
          **`rotate_vector` 의 출력이 어디에도 연결돼 있지 않다.** 그래서
          `reroute` 를 켜도 아무 일이 안 난다. 끊긴 사슬을 잇는 대신
          `inputs:direction` 을 직접 쓴다 — 본선에서 이미 검증된 경로다.
          `reroute` 는 애셋 상태가 어긋나 보이지 않게 같이 맞춰만 둔다.

        `mps` 를 주면 속도도 같이 바꾼다 (안 주면 건드리지 않는다).
        """
        zn = str(req.get("zone"))
        if zn not in self.sorters:
            return {"ok": False, "error": "unknown zone %r (있는 것: %s)"
                    % (zn, sorted(self.sorters))}
        div = bool(req.get("divert"))
        w = req.get("world")
        if w is None:
            w = self.divert_world if div else self.thru_world
        try:
            wv = [float(w[0]), float(w[1]), float(w[2])]
        except (TypeError, ValueError, IndexError, KeyError):
            return {"ok": False, "error": "bad world %r" % (w,)}
        if wv == [0.0, 0.0, 0.0]:
            return {"ok": False, "error": "world 가 영벡터다"}
        mps = req.get("mps")
        if mps is not None:
            try:
                mps = float(mps)
            except (TypeError, ValueError):
                return {"ok": False, "error": "bad mps %r" % (mps,)}
        return self.mainq.run(lambda: self.set_sorter(zn, div, wv, mps))

    def set_sorter(self, zn, div, wv, mps=None):
        """**메인 스레드 전용.** USD 쓰기는 여기서만 한다."""
        from pxr import Gf
        spath = self.sorters.get(zn)
        if not spath:
            return {"ok": False, "error": "unknown zone %r" % (zn,)}
        g = spath + "/" + SORTER_GRAPH
        node = self.stage.GetPrimAtPath(g + "/" + SORTER_NODE)
        conv = self.stage.GetPrimAtPath(spath + "/" + SORTER_PHYSICS)
        if not node or not node.IsValid() or not conv or not conv.IsValid():
            return {"ok": False, "error": "소터 %s 의 노드/컨베이어가 없다" % zn}
        node.GetAttribute("inputs:direction").Set(
            Gf.Vec3f(*self._local_axis(conv, wv)))
        rr = self.stage.GetPrimAtPath(g + "/" + SORTER_REROUTE)
        if rr and rr.IsValid():
            a = rr.GetAttribute("inputs:value")
            if a:
                a.Set(bool(div))
        if mps is not None:
            sp = self.stage.GetPrimAtPath(g + "/" + SORTER_SPEED)
            if not sp or not sp.IsValid():
                return {"ok": False, "error": "소터 %s 에 %s 가 없다"
                        % (zn, SORTER_SPEED)}
            sp.GetAttribute("inputs:value").Set(mps / self.sorter_gain[zn])
        return {"ok": True, "zone": zn, "divert": div, "world": wv}

    def set_sorter_velocity(self, zn, mps):
        """**메인 스레드 전용.** 속도만 바꾼다 (방향은 그대로)."""
        spath = self.sorters.get(zn)
        if not spath:
            return {"ok": False, "error": "unknown zone %r" % (zn,)}
        sp = self.stage.GetPrimAtPath(
            spath + "/" + SORTER_GRAPH + "/" + SORTER_SPEED)
        if not sp or not sp.IsValid():
            return {"ok": False, "error": "소터 %s 에 %s 가 없다" % (zn, SORTER_SPEED)}
        sp.GetAttribute("inputs:value").Set(mps / self.sorter_gain[zn])
        return {"ok": True, "actual": mps}

    # ── 내부. **메인 스레드 전용** ──────────────────────────
    def set_velocity(self, seg, mps):
        """⑤⑩ 벨트 한 구간의 속도를 바꾼다. **`0.0` 이 곧 정지다.**

        관제탑이 ⑤에서 `0.0`, ⑩에서 원래 속도로 두 번 부른다.
        ★ 흡착 전에 **반드시 서야 한다.** 안 세우고 흡착하면 박스가 끌려가면서
          패드가 미끄러진다.
        """
        paths = self.graphs.get(seg)
        if not paths:
            return {"ok": False, "error": "unknown seg %r (있는 것: %s)"
                    % (seg, sorted(self.graphs)), "actual": 0.0}
        # ★ 한 조각이라도 못 쓰면 ok=false 다. 본선 4 조각 중 1 개가 안 서면
        #   박스가 그 경계에서 계속 끌려가는데, ok 를 주면 그 상태로 흡착한다.
        for gpath in paths:
            prim = self.stage.GetPrimAtPath(gpath)
            if not prim or not prim.IsValid():
                return {"ok": False, "error": "그래프 %s 가 없다" % gpath,
                        "actual": 0.0}
            attr = prim.GetAttribute(VEL_VAR)
            if not attr:
                return {"ok": False, "error": "%s 에 %s 가 없다" % (gpath, VEL_VAR),
                        "actual": 0.0}
            # ★ 그래프 변수 하나만 쓴다. 실측으로 이게 정확하다 (위 주석).
            attr.Set(mps / self.gain[gpath])
        # ★ 지금 속도를 기억해 둔다. USD 에서 되읽으면 배율(gain)을 되돌려야 하고
        #   조각마다 값이 달라 정확한 원값을 못 얻는다. **누가 마지막으로 뭘 시켰나**
        #   가 필요한 전부다 — 라우터가 "이 지선이 서 있나" 를 이걸로 본다.
        # ★ 값이 바뀔 때만 찍는다 (2026-08-26 밤). 재가동(0.5)이 왕복은 되는데
        #   라우터가 계속 0 을 보는 미스터리 — 실제 도착값을 눈으로 확정하기 위함.
        if self._cur.get(seg) != float(mps):
            self._log("[conveyor_ctl] seg %-9s -> %.2f m/s" % (seg, float(mps)))
        self._cur[seg] = float(mps)
        # ★★ 본선을 켜면 소터도 켠다. 안 그러면 소터가 0 으로 서 있고, 소터
        #   자리에는 본선 롤러가 **없어서** 박스가 거기서 그냥 멎는다
        #   (실측 2026-08-25: BOX00000 이 X -6.89 에서 90 s 정지, 뒤로 2 개가
        #   줄섰다. `run_all` 을 고쳤지만 데모는 seg 별로 켜서 안 불렸다).
        if seg == self.sorter_follow_seg:
            for zn in self.sorters:
                self.set_sorter_velocity(zn, mps)
        return {"ok": True, "actual": mps}

    def stop_all(self):
        """모든 벨트 정지. **소터도 같이 끈다** — 소터가 서 있으면 본선 롤러의
        구멍 때문에 라인이 막힌다."""
        for seg in self.graphs:
            self.set_velocity(seg, 0.0)
        # ★ 소터도 같이 세운다. 안 세우면 본선이 멎은 뒤에도 소터 구간의 박스만
        #   혼자 흘러가 분기점을 지나쳐 버린다.
        for zn in self.sorters:
            self.set_sorter_velocity(zn, 0.0)

    def run_all(self):
        """모든 벨트를 사양 속도로 기동. 소터도 같이 켠다 (`stop_all` 의 짝).

        본선과 지선(`*_pickup`)의 속도가 다르다 — 아래 주석 참고.
        """
        # ★ 지선(T자의 I 부분, `*_pickup` = ConveyorBeltGraph_01)만 따로 느리게
        #   (2026-08-27 김병직). 풀 비전에서 뒤차가 스토퍼의 앞차에 붙으면 두
        #   실루엣이 병합돼 치수 맞춤이 터진다 (실측 — A존 3호 2개 병합).
        #   지선이 느리면 뒤차가 닿기 전에 인식+픽이 끝난다. 부수 이득: 스토퍼
        #   충돌 기울어짐 감소 (0.35 시절 흡착 간극 2.1~2.4mm 실측).
        #   키가 없으면 본선과 같은 속도 — 기존 동작 그대로.
        spur = float(self.cfg["conveyor"].get("spur_velocity_mps", self.nominal))
        for seg in self.graphs:
            self.set_velocity(seg, spur if seg.endswith("_pickup") else self.nominal)
        # ★ 소터를 반드시 같이 돌린다. 본선 롤러에는 소터 자리에 구멍이 있어
        #   소터가 서 있으면 **라인이 거기서 막힌다** (실측 2026-08-25: 박스가
        #   X -6.62 에서 165 s 정지).
        # ★★ 방향도 같이 세운다. A47 애셋의 기본 direction 은 로컬 (1,0,0) =
        #   월드 **+X** 인데 이 라인은 -X 로 흐른다. 속도만 켜면 소터가 본선과
        #   맞서 밀어 박스가 소터 위에서 멎는다 (실측 2026-08-25: BOX00000 이
        #   X -7.11 에서 75 s 정지, 뒤로 3 개가 줄섰다).
        for zn in self.sorters:
            self.set_sorter(zn, False, self.thru_world, self.nominal)
