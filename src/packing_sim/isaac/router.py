#!/usr/bin/env python3
"""배송지로 목적지 존을 정하고 **소터**로 박스를 지선에 밀어넣는다. (트랙 B, 2026-08-26)

## 왜 이 파일이 생겼나

`sim_main.py --ros` 경로에는 **박스를 꺾는 주체가 없었다.** 소터를 돌리는 것
(`conveyor_ctl.run_all` -> `set_sorter(..., mps)`)과 **꺾으라고 지시하는 것**은
별개인데, 지시는 `demo_stack15_test1_1.py:router_thread` 안에만 있었다.
그래서 ROS 로 띄우면 박스가 존 옆을 그냥 지나쳐 순환 루프만 돌았다
(실측 2026-08-26: 12 개 중 존에 들어간 것 0 개).

§3-3 계약에는 중재자(`DiverterRequest`)가 있지만 **부르는 쪽이 아무도 없다**
(클라이언트 0 개). 양쪽을 다 새로 짜는 대신, 이미 검증된 데모 라우터를
**같은 프로세스 안으로** 옮겼다. 나중에 중재자 답을 `zone_of()` 에 꽂으면
그대로 ROS 판단으로 바뀐다 — 그 한 군데만 갈아끼우면 된다.

## 데모판과 다른 것

  · 소켓(`link.call`)이 아니라 **핸들러를 직접** 부른다 (CLAUDE.md §2 · §9)
  · 박스 pose 를 `spawner._snap` 에서 읽는다 (`get_box_pose` 를 N 번 안 부른다)
  · 스레드가 아니라 `step(now)` 다 — 메인 루프가 부른다. **블로킹 금지** (§9)

## 옮겨 온 실측 규칙 (하나씩 다 이유가 있다 — 지우지 말 것)

  1. 박스가 지선 폭 **안에 완전히** 들어왔을 때만 꺾는다 (여유 0.25 m).
     경계에서 바로 꺾으면 몸통 일부가 레일 밑으로 들어가 걸린다 (X -18.89).
  2. **본선 띠 안**에 있을 때만. 순환 루프(Y -5.1)의 박스는 X 가 맞아도
     그 조각 위에 없다 — 꺾어 봐야 안 움직인다.
  3. 배타 구간(소터 + **상류** 여유)에 다른 박스가 있으면 기다린다.
     본선이 -X 로 흐르므로 뒤차는 X 가 큰 쪽에서 온다.
  4. 존 계량 — 그 존 지선이 차 있으면 안 꺾는다. 안 꺾인 박스는 루프를 돈다.
     **본선을 세우지 않는다** (세우면 로봇 3 대가 서로 굶는다).
  5. 지선에 올라타면 **본선으로 복귀**시킨다. 안 하면 뒤차가 전부 옆으로
     밀린다 (실측 2026-08-24: 박스 6 개가 분기점에 뭉쳐 라인이 막혔다).
  6. 타임아웃(45 s) 안에 못 올라타면 복귀. 5 번과 같은 이유다.
  7. 꺾을 때 **속도를 같이 준다.** A47 의 `SorterSpeed` 기본값이 0.0 이고
     `h_set_sorter` 는 "mps 를 안 주면 안 건드린다" 라, 안 주면 소터가 0 인 채
     남는다. 본선 롤러에는 소터 자리에 구멍이 있어 **선 소터는 라인을 막는다.**
"""
import os
import time

MAIN_LINE_Y_DEFAULT = [-500, 500]


class Router:
    """`sim_main` 메인 루프가 매 프레임 `step(time.time())` 을 부른다."""

    #: 폴링 주기 [s]. 데모판과 같다 — 이보다 잦으면 물리 스텝만 갉아먹는다.
    PERIOD_S = 0.15
    #: 지선 폭 안쪽 여유 [m]. 가장 큰 5호가 480 mm 라 240 을 물린다.
    EDGE_M = 0.25

    def __init__(self, cfg, spawner, conveyor, log=print):
        self.sp, self.cc, self.log = spawner, conveyor, log
        rt = cfg.get("routing") or {}
        self.rt = rt
        self.kind_of, self._key = self._route_map(rt)

        self.jx = {k: (min(v) / 1000.0, max(v) / 1000.0)
                   for k, v in (rt.get("junction_x_mm") or {}).items()}
        self.mseg = rt.get("main_seg") or {}
        self.segx = {k: (min(v) / 1000.0, max(v) / 1000.0)
                     for k, v in (rt.get("main_seg_x_mm") or {}).items()}
        self.y_done = float(rt.get("divert_done_y_mm", 1200)) / 1000.0
        _mly = rt.get("main_line_y_mm") or MAIN_LINE_Y_DEFAULT
        self.y_lo, self.y_hi = min(_mly) / 1000.0, max(_mly) / 1000.0
        self.thru = list(rt.get("thru_world", [-1, 0, 0]))
        self.div = list(rt.get("divert_world", [0, 1, 0]))

        _sx = rt.get("sorter_x_mm") or {}
        self.use_sorter = bool(rt.get("use_sorter")) and bool(_sx)
        guard = float(rt.get("sorter_guard_mm", 1200)) / 1000.0
        # 규칙 3 — 배타 구간은 소터 구간 + **상류(+X)** 여유다.
        self.sortx = {k: (min(v) / 1000.0, max(v) / 1000.0 + guard)
                      for k, v in _sx.items()}

        # 규칙 4 — 존 계량용 지선 창
        _cross = (cfg["zones"].get("pickup_cross_mm") or {})
        _pr = cfg["conveyor"].get("pickup_range_mm") or [0, 0]
        self.pk_y = (min(_pr) / 1000.0, max(_pr) / 1000.0)
        self.pk_x = {k: (min(v) / 1000.0, max(v) / 1000.0)
                     for k, v in _cross.items()}
        self.meter_max = int(os.environ.get("PL_METER")
                             or rt.get("meter_max", 1) or 0)

        self.max_laps = int(os.environ.get("PL_MAX_LAPS")
                            or rt.get("max_laps", 0) or 0)
        self.loop_y = float(rt.get("loop_y_mm", -1500)) / 1000.0
        self.div_timeout_s = float(os.environ.get("PL_DIV_TIMEOUT", "45"))
        # ⚠ 키 이름은 `belt_velocity_mps` 다 (`belt_speed_mps` 가 아니다 — 2026-08-26 실측)
        self.belt_mps = float(cfg["conveyor"]["belt_velocity_mps"])

        self.state, self.laps, self.in_loop = {}, {}, {}
        self.t_div, self.t_log = {}, {}
        self._next = 0.0
        self._h_sorter = conveyor.handlers().get("set_sorter")
        self._h_dir = conveyor.handlers().get("set_direction")
        self._h_vel = conveyor.handlers().get("set_velocity")

        self.log("[router] 켬 (%s) — %s"
                 % ("소터" if self.use_sorter else "조각 꺾기",
                    ", ".join("%s->%s" % kv for kv in sorted(self.kind_of.items()))
                    or "표 없음"))
        if not self.kind_of or not self.jx:
            self.log("[router] !! routing 설정이 비어 있다 — 아무것도 안 꺾는다")

    # ── 분류 기준 ────────────────────────────────────────────────────────
    @staticmethod
    def _route_map(rt):
        """우선순위 by_dest > by_color > by_type (데모판과 같다).

        ★ 여기가 **중재자를 꽂을 자리**다. `/diverter/request` 를 부르는 쪽이
          생기면 `zone_of()` 가 그 답을 쓰도록 바꾸면 된다 — 나머지는 그대로다.
        """
        for key, pick in (("by_dest", lambda r: r.get("dest")),
                          ("by_color", lambda r: r.get("color")),
                          ("by_type", lambda r: r.get("type"))):
            tbl = rt.get(key)
            if tbl:
                return {str(k): str(v) for k, v in tbl.items()}, pick
        return {}, (lambda r: None)

    def zone_of(self, box_id):
        rec = self.sp.boxes.get(box_id)
        return None if rec is None else self.kind_of.get(self._key(rec))

    # ── 소터 명령 ────────────────────────────────────────────────────────
    def _turn(self, zone, seg, want_div):
        """규칙 7 — 소터면 **항상 mps 를 같이 준다.**"""
        if self.use_sorter and self._h_sorter:
            return self._h_sorter({"zone": zone, "divert": bool(want_div),
                                   "mps": self.belt_mps}) or {}
        if self._h_dir:
            return self._h_dir({"seg": seg,
                                "world": self.div if want_div else self.thru}) or {}
        return {"ok": False, "error": "set_sorter/set_direction 핸들러가 없다"}

    def _zone_load(self, zn, allpos):
        r = self.pk_x.get(zn)
        if r is None:
            return 0
        return sum(1 for q in allpos.values()
                   if r[0] <= q[0] <= r[1] and self.pk_y[0] <= q[1] <= self.pk_y[1])

    # ── 메인 루프가 부른다 ───────────────────────────────────────────────
    def step(self, now):
        if now < self._next:
            return
        self._next = now + self.PERIOD_S
        if not self.kind_of or not self.jx:
            return

        # ★ **done 박스도** 읽는다. 지선에 올라탄 박스가 곧 그 존의 부하다.
        #   건너뛰면 존이 꽉 차 있어도 계속 꺾어 보낸다.
        allpos = {b: e["pos_m"] for b, e in self.sp._snap.items()}
        poses = {b: q for b, q in allpos.items() if self.state.get(b) != "done"}

        self._count_laps(poses)

        for bid, p in list(poses.items()):
            z = self.zone_of(bid)
            if z is None or z not in self.jx:
                continue
            seg = self.mseg.get(z)

            if p[1] >= self.y_done:                     # 지선에 올라탔다
                if self.state.get(bid) == "divert":     # 규칙 5 — 본선 복귀
                    self._turn(z, seg, False)
                    self.log("[router] %s 지선 %s 진입 (Y %.2f) — 본선 복귀"
                             % (bid, z, p[1]))
                self.state[bid] = "done"
                continue

            if self.state.get(bid) == "divert":
                if now - self.t_div.get(bid, 0.0) > self.div_timeout_s:
                    self._turn(z, seg, False)           # 규칙 6
                    self.state[bid] = "done"
                    self.log("[router] !! %s 가 %.0f s 안에 지선 %s 에 못 올라탔다"
                             " (Y %.2f) — 본선 복귀"
                             % (bid, self.div_timeout_s, z, p[1]))
                continue

            lo, hi = self.jx[z]
            if not (lo + self.EDGE_M <= p[0] <= hi - self.EDGE_M):
                continue                                # 규칙 1
            if not (self.y_lo <= p[1] <= self.y_hi):
                continue                                # 규칙 2

            slo, shi = (self.sortx.get(z) if self.use_sorter else None) \
                or self.segx.get(z, (lo - 3.5, hi + 3.5))
            busy = [b2 for b2, q in allpos.items()
                    if b2 != bid and self.y_lo <= q[1] <= self.y_hi
                    and slo <= q[0] <= shi]
            if busy:
                continue                                # 규칙 3

            if self.meter_max > 0 and self._zone_load(z, allpos) >= self.meter_max:
                if now - self.t_log.get("z" + z, 0.0) >= 10.0:
                    self.t_log["z" + z] = now
                    self.log("[router] 존 %s 가 차 있다 (%d 대) — %s 는 한 바퀴 돈다"
                             % (z, self._zone_load(z, allpos), bid))
                continue                                # 규칙 4

            r = self._turn(z, seg, True)
            if r.get("ok"):
                self.state[bid] = "divert"
                self.t_div[bid] = now
                if seg and self._h_vel:                 # 계량이 세워 뒀을 수 있다
                    self._h_vel({"seg": seg, "mps": self.belt_mps})
                self.log("[router] %s -> 존 %s   X %.2f m 에서 꺾는다"
                         % (bid, z, p[0]))
            else:
                self.log("[router] !! 분류 명령 실패 (존 %s) -> %r" % (z, r))

    def _count_laps(self, poses):
        """리턴 라인을 `max_laps` 바퀴 돌면 치운다.

        ★ 존이 영영 안 비면 박스가 **영원히 돈다.** prim 이 쌓여 프레임이 떨어진다
          (실측 2026-08-25: 12 개 중 8 개가 동시에 루프를 돌았다).
          한 바퀴 = 루프로 내려갔다가 본선 띠로 돌아오는 것.
        """
        if self.max_laps <= 0:
            return
        despawn = self.sp.handlers().get("despawn_box")
        for b, q in poses.items():
            if q[1] < self.loop_y:
                self.in_loop[b] = True
            elif self.in_loop.get(b) and self.y_lo <= q[1] <= self.y_hi:
                self.in_loop[b] = False
                self.laps[b] = self.laps.get(b, 0) + 1
                self.log("[router] %s 리턴 %d 바퀴째" % (b, self.laps[b]))
                if self.laps[b] >= self.max_laps and despawn:
                    rr = despawn({"box_id": b}) or {}
                    self.state[b] = "done"
                    self.log("[router] %s %d 바퀴 -> 소멸 (%s)"
                             % (b, self.laps[b],
                                "ok" if rr.get("ok") else repr(rr)))
