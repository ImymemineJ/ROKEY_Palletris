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
BELT_GRAPH = {
    "main": "/World/ConveyorTrack/ConveyorBeltGraph",
    "b_pickup": "/World/ConveyorTrack1/ConveyorBeltGraph",
}
VEL_VAR = "graph:variable:Velocity"



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
        # 조각마다 scale.x 를 한 번 읽어 둔다. 씬이 도는 동안 안 바뀐다.
        self.gain = {}
        for seg, gpath in BELT_GRAPH.items():
            track = gpath.rsplit("/", 1)[0]
            prim = stage.GetPrimAtPath(track)
            if not prim or not prim.IsValid():
                self._log("[conveyor_ctl] !! %s 가 씬에 없다 — seg %r 은 안 먹는다"
                          % (track, seg))
                continue
            a = prim.GetAttribute("xformOp:scale")
            sx = float(a.Get()[0]) if a and a.Get() is not None else 1.0
            if sx <= 0.0:
                raise ValueError("%s 의 scale.x 가 %r 이다" % (track, sx))
            self.gain[seg] = sx
            self._log("[conveyor_ctl] seg %-9s %s  scale.x %.4f "
                      "(Velocity %.4f = %.2f m/s)"
                      % (seg, gpath, sx, self.nominal / sx, self.nominal))

    # ── 핸들러 ──────────────────────────────────────────────
    def handlers(self):
        return {"set_velocity": self.h_set_velocity}

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
        if seg not in BELT_GRAPH:
            return {"ok": False, "error": "unknown seg %r (있는 것: %s)"
                    % (seg, sorted(BELT_GRAPH)), "actual": 0.0}
        return self.mainq.run(lambda: self.set_velocity(seg, mps))

    # ── 내부. **메인 스레드 전용** ──────────────────────────
    def set_velocity(self, seg, mps):
        gpath = BELT_GRAPH.get(seg)
        if gpath is None:
            return {"ok": False, "error": "unknown seg %r (있는 것: %s)"
                    % (seg, sorted(BELT_GRAPH)), "actual": 0.0}
        if seg not in self.gain:
            return {"ok": False, "error": "seg %r 의 벨트 조각이 씬에 없다" % seg,
                    "actual": 0.0}
        prim = self.stage.GetPrimAtPath(gpath)
        if not prim or not prim.IsValid():
            return {"ok": False, "error": "그래프 %s 가 없다" % gpath, "actual": 0.0}
        attr = prim.GetAttribute(VEL_VAR)
        if not attr:
            return {"ok": False, "error": "%s 에 %s 가 없다" % (gpath, VEL_VAR),
                    "actual": 0.0}
        # ★ 그래프 변수 하나만 쓴다. 실측으로 이게 정확하다 (아래 주석).
        attr.Set(mps / self.gain[seg])
        return {"ok": True, "actual": mps}

    def stop_all(self):
        for seg in self.gain:
            self.set_velocity(seg, 0.0)

    def run_all(self):
        for seg in self.gain:
            self.set_velocity(seg, self.nominal)
