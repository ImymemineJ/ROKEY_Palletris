"""벨트 속도 제어 — `set_velocity` 핸들러 (공통가이드 §3, 작업명세서 §2.2). **김학민 소유.**

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

# ConveyorBelt_A05 롤러 1 스케일당 길이 (m). layout_to_spec_test1_1.py 와 같은 상수다.
ROLLER_LEN_PER_SCALE = 1.98724



class ConveyorCtl:
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

    def _measure_gain(self, gpath):
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
        node = self.stage.GetPrimAtPath(gpath + "/ConveyorNode")
        if not node or not node.IsValid():
            self._log("[conveyor_ctl] !! %s 에 ConveyorNode 가 없다" % gpath)
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
    def handlers(self):
        return {"set_velocity": self.h_set_velocity,
                "set_direction": self.h_set_direction}

    def h_set_velocity(self, req):
        """{"cmd":"set_velocity","seg":"b_pickup","mps":0.0} -> {"ok":..,"actual":..}

        오케스트레이터는 **ok 를 확인한 뒤** 흡착한다 (§2.5). 안 멈춘 채 집으면
        박스가 끌려가므로 여기서 거짓말을 하면 안 된다 — 못 쓰면 ok=false 다.
        """
        seg = req.get("seg")
        try:
            mps = float(req.get("mps"))
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
            M = UsdGeom.XformCache().GetLocalToWorldTransform(roll) \
                .ExtractRotationMatrix()
            # 월드 -> 로컬. M 에 스케일이 섞여 있어 각 축을 정규화한 뒤 투영한다.
            loc = []
            for k in range(3):
                row = [float(M[k][j]) for j in range(3)]
                n = (row[0] ** 2 + row[1] ** 2 + row[2] ** 2) ** 0.5 or 1.0
                loc.append(sum(wv[j] * row[j] for j in range(3)) / n)
            i = max(range(3), key=lambda k: abs(loc[k]))
            out = [0.0, 0.0, 0.0]
            out[i] = 1.0 if loc[i] > 0 else -1.0
            from pxr import Gf
            node.GetAttribute("inputs:direction").Set(Gf.Vec3f(*out))
        return {"ok": True, "world": wv}

    # ── 내부. **메인 스레드 전용** ──────────────────────────
    def set_velocity(self, seg, mps):
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
        return {"ok": True, "actual": mps}

    def stop_all(self):
        for seg in self.graphs:
            self.set_velocity(seg, 0.0)

    def run_all(self):
        for seg in self.graphs:
            self.set_velocity(seg, self.nominal)
