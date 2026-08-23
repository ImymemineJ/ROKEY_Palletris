"""15박스 연속 적재 — **실제 씬 `test1-1.usd` 위에서** (트랙 A 통합 시험).

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec /home/rokey/palletris_ws/scripts/demo_stack15_test1_1.py

`scripts/demo_stack15.py`(B5) 는 받침대만 있는 합성 씬에서 박스를 픽 지점으로
**급송**해 15개를 쌓는다. 이 스크립트는 같은 15개를 **A 가 만든 진짜 라인**으로
보낸다 — 컨베이어를 타고 와서 스토퍼에 서고, 거기서 집힌다. 새로 만드는 것은
글루뿐이고 판단하는 코드는 전부 실물이다:

    packing_robot/cycle.py                상태머신 (§2.5)
    packing_sim/isaac/robot_motion.py     B 명령 6개
    packing_sim/isaac/{sim_server,spawner,conveyor_ctl,mainq}.py   <- A
    demo_stack15.{manifest,PalletVision,OnlineServices}            <- B5 그대로 import
    src/packing_sim/packing.HeightmapPacker                        배치 계획

## B5 데모와 다른 점 셋 — 전부 "진짜 라인이라서"

  1) **박스가 벨트를 타고 온다.** 급송(`set_world_pose`)이 없다. 스폰점 x=500 에서
     스토퍼 x=10000 까지 9.5 m 를 0.5 m/s 로 온다 (약 19 s).
  2) **도착 순서 그대로 집는다.** B5 는 오프라인 계획이라 부피 내림차순으로 급송해
     15/15 를 넣었다. 라인에는 그 자유가 없다 — 그래서 온라인 패커
     (`OnlineServices`, 매 박스 비전 재계산)를 쓴다. 못 넣으면 PALLET_FULL 이
     그대로 결과에 남는다. 그게 이 시험이 답해야 할 질문이다.
  3) **벨트를 진짜로 세운다.** `OnlineServices.set_velocity` 는 기록만 하는 mock
     이다(B5 씬에 컨베이어가 없다). 여기서는 소켓으로 실제 구간을 세운다
     — 안 세운 채 흡착하면 박스가 끌려간다 (§2.5).

## 스폰은 사양 스케줄로 돈다

`boxes.stage1.spawn_interval_s`(12 s) 마다 하나씩, `total_count`(15) 개.
사이클이 그보다 오래 걸리면 픽업 구간(7500~10000, 2500 mm)에 줄이 선다 —
설계결정서 §4.4 가 말한 "백압 버퍼"가 실제로 생기는지 보는 것이 이 시험의 절반이다.
`zones.buffer_capacity` 는 4 다.

## 왜 `isaac-sim.sh` 인가

벨트가 `IsaacConveyor` OmniGraph 노드로 도는데 그 노드는 렌더 틱에 물려 있다.
`python.sh` standalone 은 속도를 틀리게 돌린다. 근거는 `isaac/conveyor_ctl.py`
docstring. 여기서는 이송 시간이 결과의 일부라 `isaac-sim.sh` 를 쓴다.
"""
import math
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from omni.physx import get_physx_interface

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src/packing_sim/isaac"))
sys.path.insert(0, str(ROOT / "src/packing_robot/packing_robot"))

import _paths                                            # noqa: E402
import import_h2017_usd as IMP                           # noqa: E402
import demo_stack15 as D                                 # B5 산출물 그대로  # noqa: E402

STAGE = os.environ.get("PL_STAGE", "/home/rokey/isaac_scenes/project_1/test1-1.usd")
PORT = int(os.environ.get("PL_PORT", "8797"))            # 8765=sim_main · 8798=B5 · 8799=B4
ROBOT_PRIM = "/World/robot"
GRIPPER_PRIM = ROBOT_PRIM + "/SurfaceGripper"
EE_FRAME = "tool0"
ZONE = "B"
BOX_ID_FMT = "BOX%05d"
BOOT_STEPS = 20
SETTLE_STEPS = 90                                        # READY 수렴 (1.5 s)
N_BOXES = int(os.environ.get("PL_N", "0")) or None        # 0/미지정이면 사양값
# ★ 상자 조합을 이 실행에서만 바꾼다. `system.yaml` 의 random_seed 42 는
#   "고정 — 베이스라인 비교 필수" 라 파일을 고치지 않는다.
#   seed 42 는 5호가 4개 뽑힌 빡센 패라(사양 분포로는 3개) 도착 순서로는 14/15 가 상한이다.
#   시연용으로 전부 들어가는 조합을 쓰려면 PL_SEED 를 준다 — 어떤 seed 가 되는지는
#   `python3 scripts/eval_pack_online.py --scan-seeds 60` 이 답한다 (60개 중 6개).
#   ⚠ PL_SEED 를 쓴 실행은 **베이스라인이 아니다.** 결과에 그렇게 찍는다.
SEED = os.environ.get("PL_SEED")
# ★ 투입 순서. stage 1 은 **스포너가 곧 투입 순서**라 여기서 정할 수 있다.
#   arrival = 뽑힌 그대로 (기본). 무엇이 올지 못 고르는 실제 라인의 조건이다.
#   volume  = 부피 내림차순. **B5 데모(`demo_stack15.py`)와 같은 조건**이다 —
#             그쪽 docstring: "15개를 도착 순서 그대로 넣으면 패커가 14/15 밖에
#             못 넣는다. 부피 내림차순으로 정렬하면 15/15 가 들어간다."
#   seed 42 로 실측(`scripts/eval_pack_online.py --sweep`): arrival 은 clearance ·
#   halo 를 어떻게 줘도 14/15 가 상한이고, volume 은 어떤 설정에서도 15/15 다.
#   즉 이 차이는 튜닝이 아니라 **순서**다.
ORDER = os.environ.get("PL_ORDER", "arrival")
if ORDER not in ("arrival", "volume"):
    raise SystemExit("PL_ORDER 는 arrival | volume: %r" % ORDER)
ARRIVE_TIMEOUT_S = 180.0                                 # 다음 박스가 스토퍼에 설 때까지
CYCLE_WALL_S = 120.0
SETTLE_S = float(os.environ.get("PL_SETTLE", "0.7"))     # 놓은 뒤 재기 전 정착 (B5 는 0.5)
WATCHDOG_S = float(os.environ.get("PL_WATCHDOG", "900"))

cfg = _paths.system_config()
STOPPER_X = cfg["conveyor"]["stopper_x_mm"] / 1000.0
BELT_MPS = float(cfg["conveyor"]["belt_velocity_mps"])
ORIGIN_MM = cfg["zones"]["pallet_origin_world_mm"][ZONE]
S1 = cfg["boxes"]["stage1"]
BUFFER_CAP = int(cfg["zones"]["buffer_capacity"])        # 픽업 구간에 세울 수 있는 최대
N = N_BOXES or int(S1["total_count"])
SPAWN_DT = float(S1["spawn_interval_s"])

_S = {"phase": "boot", "n": 0, "t0": time.time(), "done": False,
      "rows": [], "result": None}


def log(*a):
    print(*a, flush=True)


# ── 씬 열기 + Play ──────────────────────────────────────────
omni.usd.get_context().open_stage(STAGE)
stage = omni.usd.get_context().get_stage()
log("\n" + "=" * 78)
log("15박스 연속 적재 — 실제 라인 (스폰 %.0f s 간격 · %d 개 · 존 %s)" % (SPAWN_DT, N, ZONE))
log("=" * 78)
log("씬        %s" % STAGE)
log("팔레트    원점 %s mm   최대 적재 %s mm" % (ORIGIN_MM, cfg["pallet"]["max_stack_mm"]))
log("라인      스폰 %s -> 스토퍼 %.3f m   벨트 %.2f m/s"
    % (cfg["conveyor"]["spawn_point_mm"], STOPPER_X, BELT_MPS))
for p in (ROBOT_PRIM, GRIPPER_PRIM, "/World/Stopper"):
    ok = stage.GetPrimAtPath(p).IsValid()
    log("  %-28s %s" % (p, "있음" if ok else "!! 없음"))
    if not ok:
        log("  -> src/packing_sim/run_layout_to_spec.sh 를 먼저 돌려라.")
        omni.kit.app.get_app().post_quit()
        raise SystemExit
log("-" * 78)

omni.timeline.get_timeline_interface().play()


def setup():
    """물리가 돌기 시작한 뒤에 부른다 (아티큘레이션 초기화가 필요하다)."""
    import conveyor_ctl
    import mainq as MQ
    import robot_motion as RM
    import spawner as SP
    import sim_server as SS
    import isaacsim.robot_motion.motion_generation as mg
    from isaacsim.core.prims import SingleArticulation

    art = SingleArticulation(prim_path=ROBOT_PRIM, name="h2017")
    art.initialize()

    tmp = tempfile.mkdtemp(prefix="h2017_lula_")
    urdf, _n, missing = IMP.make_abs_urdf(tmp)
    if missing:
        raise RuntimeError("메시 없음: %s" % missing[:3])
    lula = mg.LulaKinematicsSolver(robot_description_path=str(_paths.DESCRIPTOR),
                                   urdf_path=str(urdf))
    pos, quat = art.get_world_pose()
    lula.set_robot_base_pose(np.array(pos), np.array(quat))
    log("로봇      베이스 %s mm  EE %s"
        % ([round(float(v) * 1000) for v in pos], EE_FRAME))

    q = MQ.MainQueue()
    rm = RM.RobotMotion(art, lula, cfg, ee=EE_FRAME, gripper_path=GRIPPER_PRIM)
    sp = SP.Spawner(stage, cfg, q, zone=ZONE)
    cc = conveyor_ctl.ConveyorCtl(stage, cfg, q)

    server = SS.SimServer(port=PORT)
    server.register({"ping": lambda r: {"ok": True}})
    server.register(sp.handlers())
    server.register(cc.handlers())
    server.register(rm.handlers())
    log("소켓      127.0.0.1:%d   cmd %d 개" % (PORT, len(server.cmds())))

    # ★ READY 로 순간이동. 안 하면 첫 이동이 179 deg 라 구간 예산(20 s)을 넘겨
    #   MOTION_TIMEOUT 이 난다 (verify_b4_test1_1.py 주석의 실측).
    import yaml
    from isaacsim.core.utils.types import ArticulationAction
    ready = np.array(yaml.safe_load(_paths.DESCRIPTOR.read_text())["default_q"], dtype=float)
    art.set_joint_positions(ready)
    art.set_joint_velocities(np.zeros(6))
    _S["ready_action"] = ArticulationAction(joint_positions=ready)
    log("자세      READY 로 순간이동 %s deg" % np.round(np.degrees(ready), 1).tolist())

    _S.update(rm=rm, sp=sp, cc=cc, q=q, server=server, art=art)
    cc.stop_all()


class LinkPrim:
    """`PalletVision` 이 기대하는 최소 인터페이스 — **소켓으로** 읽는다.

    B5 는 `SingleRigidPrim` 을 직접 들고 읽지만, 여기서는 `get_box_pose` 로만 본다.
    (a) 실제 오케스트레이터가 가진 것이 그것뿐이고, (b) 스포너 스냅샷은 메인 스레드가
    갱신하므로 소켓 스레드에서 PhysX 를 건드리는 일이 없다.
    """

    def __init__(self, link, box_id):
        self.link, self.box_id = link, box_id

    def get_world_pose(self):
        r = self.link.call(cmd="get_box_pose", box_id=self.box_id)
        if not r.get("ok"):
            raise RuntimeError("get_box_pose(%s) -> %r" % (self.box_id, r))
        y = math.radians(float(r.get("yaw_deg", 0.0)))
        return (np.asarray(r["pos_m"], dtype=float),
                np.array([math.cos(y / 2.0), 0.0, 0.0, math.sin(y / 2.0)]))


def _line_state(link, ids):
    """라인에 남아 있는 박스들의 위치. 타임아웃 진단용."""
    st = link.call(cmd="spawn_status")
    out = []
    for bid in ids:
        r = link.call(cmd="get_box_pose", box_id=bid)
        if not r.get("ok"):
            continue                      # 놓였거나 despawn 된 것
        x, y, z = r["pos_m"]
        out.append("%s(%.2f,%.2f,%.2f)" % (bid[-3:], x, y, z))
    return "라인 %s개 · %s" % (st.get("on_belt", "?"), " ".join(out) if out else "비었다")


class LineServices(D.OnlineServices):
    """B5 의 온라인 서비스 그대로 + **벨트만 진짜로 세운다.**

    부모의 `set_velocity` 는 기록만 하는 mock 이다 (B5 씬에는 컨베이어가 없다).
    실제 라인에서 그러면 `cycle.py` 가 "멈췄다" 고 믿고 흡착해 박스를 끌고 간다.
    """

    def __init__(self, link, *a, **kw):
        super().__init__(*a, **kw)
        self.link = link

    def set_velocity(self, seg, mps):
        self._rec("/conveyor/set_velocity", seg, mps)
        return bool(self.link.call(cmd="set_velocity", seg=seg, mps=mps).get("ok"))

    def reset_pallet(self):
        """`/algo/reset` 대역 — 하이트맵을 비운다.

        `plan_placement` 이 매번 `vision.scan()` 으로 하이트맵을 다시 세우므로,
        비전이 본 목록만 비우면 된다. **`swap_pallet` 과 반드시 짝으로** 부른다 —
        안 비우면 패커가 이미 내보낸 박스 위에 다음 자리를 잡는다.
        """
        self._rec("/algo/reset", "B")
        self.vision.placed.clear()
        self.last = None


def spawner_thread(specs, ids):
    """사양 스케줄대로 박스를 라인에 올린다. **자기 소켓을 따로 쓴다** (스레드 안전)."""
    import sim_link as SL
    link = SL.SimLink(port=PORT)
    _S["spawn_link"] = link
    try:
        for i, bid in enumerate(ids):
            if _S["done"]:
                return
            if i:
                # 사양 간격이 **하한**이다.
                t_end = time.time() + SPAWN_DT
                while time.time() < t_end and not _S["done"]:
                    time.sleep(0.05)
                if _S["done"]:
                    return
                # ★ 백압 — 버퍼가 차 있으면 넣지 않는다 (설계결정서 §7:
                #   "목표 구역 버퍼에 여유 있음 -> 통과"). 사이클 14.5 s 인데 간격이
                #   12 s 라 무조건 넣으면 큐가 계속 길어진다. 그러면 벨트가 뒤에서
                #   계속 밀어 박스들이 서로 눌리고, 결국 솔버가 터져 박스가 초속
                #   수 km 로 날아간다 (2026-08-23 BOX00014 실측). 사양대로 막는다.
                t_wait = time.time()
                while not _S["done"]:
                    st = link.call(cmd="spawn_status")
                    if st.get("queue", 0) < BUFFER_CAP:
                        break
                    if time.time() - t_wait > 120.0:
                        log("   [투입] 버퍼가 120 s 째 안 빈다 — 그냥 넣는다")
                        break
                    time.sleep(0.5)
                held = time.time() - t_wait
                if held > 1.0:
                    log("   [투입] 백압 대기 %.0f s (버퍼 %d 대기 중)"
                        % (held, BUFFER_CAP))
            t = specs[bid]
            r = link.call(cmd="spawn_box", box_id=bid, type=t["name"], yaw_deg=0)
            st = link.call(cmd="spawn_status")
            log("   [투입] %-9s %-3s  (라인 위 %s 개)"
                % (bid, t["name"], st.get("on_belt", "?")))
            if not r.get("ok"):
                log("   [투입] !! spawn_box %s -> %r" % (bid, r))
    except Exception as e:
        log("   [투입] !! %r" % (e,))


def worker(specs, ids):
    """머리 쪽. 소켓으로만 말한다."""
    import cycle as CY
    import sim_link as SL

    try:
        link = SL.SimLink(port=PORT)
        _S["link"] = link

        by_id = {bid: LinkPrim(link, bid) for bid in ids}
        hm, pal = cfg["heightmap"], cfg["pallet"]
        PKM = D._load_packer_module()

        def factory():
            return PKM.HeightmapPacker(
                size_xy=(pal["width_mm"] / 1000.0, pal["depth_mm"] / 1000.0),
                max_height=pal["max_stack_mm"] / 1000.0,
                cell=hm["cell_mm"] / 1000.0,
                yaws_deg=tuple(float(v) for v in hm["allowed_yaw_deg"]),
                min_support_ratio=float(hm["min_support_ratio"]),
                support_tol=hm["support_tol_mm"] / 1000.0,
                wall_margin=0.0, enforce_load_order=True,
                descent_halo=D.DESCENT_HALO_MM / 1000.0)

        vision = D.PalletVision(by_id, specs, ORIGIN_MM)
        svc = LineServices(link, specs, factory, vision, D.CLEARANCE_MM,
                           log=lambda *a: log("           ", *a))
        cyc = CY.Cycle(link, svc, cfg, zone=ZONE, log=lambda *a: log("   [cycle]", *a))

        # 라인을 돌린다. 이후 정지/재가동은 `cycle.py` 가 b_pickup 만 만진다.
        link.call(cmd="set_velocity", seg="main", mps=BELT_MPS)
        link.call(cmd="set_velocity", seg="b_pickup", mps=BELT_MPS)
        threading.Thread(target=spawner_thread, args=(specs, ids),
                         daemon=True, name="spawn").start()

        rest = {}                                        # 놓인 뒤의 기준 자세 (knock 용)
        swapped_for = set()                              # 이 박스 때문에 이미 교체했다
        pallets = 1
        t_all = time.time()
        i = 0
        while i < len(ids) and not _S["done"]:
            # ── 다음 박스가 스토퍼에 설 때까지 ──────────────
            pick, t0, nag = None, time.time(), 0.0
            while time.time() - t0 < ARRIVE_TIMEOUT_S and not _S["done"]:
                c = link.call(cmd="pickup_check", zone=ZONE)
                if c.get("present"):
                    pick = c
                    break
                # 안 오면 어디에 있는지 주기적으로 찍는다. 이게 없으면 타임아웃
                # 났을 때 "안 왔다" 는 사실만 남고 원인을 못 찾는다
                # (2026-08-23 15번째가 180 s 동안 안 와서 겪었다).
                if time.time() - t0 > nag + 15.0:
                    nag = time.time() - t0
                    log("   ...대기 %.0f s — %s" % (nag, _line_state(link, ids)))
                time.sleep(0.05)
            if pick is None:
                log("   !! %d 번째 박스가 %.0f s 안에 스토퍼에 안 왔다 — 중단"
                    % (i + 1, ARRIVE_TIMEOUT_S))
                log("      마지막 라인 상태: %s" % _line_state(link, ids))
                _S["stalled"] = True
                break
            bid = pick["box_id"]
            t = specs[bid]
            wait_s = time.time() - t0

            log("\n[%2d/%d] %-9s %-3s %s mm %.0f kg   대기 %.1f s   앞면 %.4f m"
                % (i + 1, len(ids), bid, t["name"],
                   [t["w_mm"], t["d_mm"], t["h_mm"]], t["mass_g"] / 1000.0, wait_s,
                   pick["pose"]["pos_m"][0] + t["w_mm"] / 2000.0))

            # ── 사이클 한 바퀴 — 실물 cycle.py 그대로 ───────
            tc = time.time()
            res = cyc.run(bid, pick["pose"])
            wall = time.time() - tc

            # ── 놓은 뒤 정착 ────────────────────────────────
            # ★ B5 견본(`demo_stack15.main`)은 사이클이 끝난 뒤 SETTLE_S 만큼 물리를
            #   더 돌린 **다음에** 잰다. 여기서는 그게 빠져 있어서 흔들리는 도중을
            #   찍고 있었다. 흡착을 떼는 순간 매달린 박스가 진동하는데(패드 rotXY
            #   드라이브 강성 100 · 감쇠 0), TCP 정지 판정은 **툴** 속도라 그걸 못 본다
            #   — `gripper.py` 의 RELEASE_MIN_HOLD_S 주석이 같은 얘기다.
            #   실측 2026-08-23: 3번 5호가 |dq| 177 deg 로 크게 돌아온 뒤 놓였고,
            #   즉시 재니 y 오차 -76.9 mm 였다. 사양 판정선(30 mm)을 넘는 값이라
            #   무엇을 재는지가 결과를 가른다. 물리는 메인 스레드가 계속 돌리므로
            #   여기서는 기다리기만 하면 된다.
            time.sleep(SETTLE_S)

            # ── 결과 · knock ────────────────────────────────
            # ★ `svc.last` 는 **성공한 계획**만 갱신된다. PALLET_FULL 이면 직전 박스의
            #   계획이 그대로 남아 있어서, 그걸 목표로 쓰면 오차가 -1130 mm 같은
            #   헛것이 찍힌다 (2026-08-23 첫 실행에서 실제로 그랬다).
            place = dict(svc.last) if (res == "PLACED" and svc.last) else None
            err = np.array([float("nan")] * 3)
            if place is not None:
                want = np.asarray(CY.pallet_place_to_world(place, t["h_mm"], ORIGIN_MM))
                got = by_id[bid].get_world_pose()[0]
                err = (got - want) * 1000.0
            knock, who = 0.0, "-"
            for pid, ref in rest.items():
                dmm = float(np.linalg.norm(by_id[pid].get_world_pose()[0] - ref) * 1000.0)
                if dmm > knock:
                    knock, who = dmm, pid
            if res == "PLACED":
                rest[bid] = by_id[bid].get_world_pose()[0].copy()

            _S["rows"].append({"id": bid, "type": t["name"], "res": res, "err": err,
                               "wall": wall, "wait": wait_s, "knock": knock, "who": who,
                               "place": place})
            log("       %-11s 오차 [%+.1f %+.1f %+.1f] mm   %.1f s   knock %.1f mm (%s)"
                % (res, err[0], err[1], err[2], wall, knock, who))

            # ── PALLET_FULL 은 **팔레트를 갈아서** 푼다 (사양 경로) ──
            # 트랙B 가이드 상태도: "PALLET_FULL: /sim/set_spawn_enabled(false)
            #   -> swap_pallet -> /algo/reset -> 재가동".
            # 이 박스는 아직 라인 위에 그대로 있다(스토퍼가 잡고 있다). 빈 팔레트를
            # 받은 뒤 **같은 박스를 다시** 집는다. 한 박스당 한 번만 교체한다 —
            # 빈 팔레트에도 안 들어가면 그건 팔레트 문제가 아니라 박스 문제다.
            if res == "PALLET_FULL" and bid not in swapped_for:
                swapped_for.add(bid)
                _S["rows"].pop()                     # 이건 최종 결과가 아니다
                log("       팔레트가 찼다 — 사양 경로로 교체한다 "
                    "(스폰 정지 -> swap_pallet -> algo reset -> 재가동)")
                link.call(cmd="set_spawn_enabled", enabled=False)
                sw = link.call(cmd="swap_pallet", zone=ZONE)
                svc.reset_pallet()
                link.call(cmd="set_spawn_enabled", enabled=True)
                if not sw.get("ok"):
                    log("       !! swap_pallet 실패: %r" % sw)
                else:
                    pallets += 1
                    log("       팔레트 #%d — 박스 %d 개를 내보냈다. %s 를 다시 집는다."
                        % (pallets, sw.get("removed", 0), bid))
                rest.clear()                          # 팔레트가 비었으니 knock 기준도 리셋
                continue                              # i 를 안 올린다 = 같은 박스 재시도
            i += 1
            # ★ PALLET_FULL 에서 멈추지 않는다. 큰 박스가 못 들어갔다고 작은 박스도
            #   못 들어가는 게 아니다 — seed 42 에서 14번 5호는 자리가 없지만
            #   15번 3호는 들어간다 (scripts/eval_pack_online.py 로 확인). 예전 판은
            #   여기서 break 해서 멀쩡한 박스를 안 돌렸다.
            #   사양상으로도 이건 실패가 아니다: "버퍼 우회는 정상적인 흐름 제어이지
            #   실패가 아님" (설계결정서 §7 · bypass_count 를 retry_count 와 분리하는 이유).
            #   stage 2 에서는 Return Line 으로 재순환한다. stage 1 은 그 설비가 없어
            #   여기서 라인 밖으로 뺀다.
            if res in ("PALLET_FULL", "BLOCKED", "IK_FAIL", "PICK_FAIL"):
                if res == "PALLET_FULL":
                    log("       이 팔레트에 자리가 없다 — 우회 처리하고 다음 박스로 간다.")
                else:
                    log("       !! 실패. 라인에서 빼고 다음 박스로 넘어간다.")
                # 스토퍼를 막고 있으면 다음 박스가 못 온다. 반드시 치운다.
                link.call(cmd="despawn_box", box_id=bid)

        _S["result"] = {"wall": time.time() - t_all, "pallets": pallets}
    except BaseException:
        import traceback
        _S["result"] = {"trace": traceback.format_exc()}
    finally:
        _S["done"] = True


def report():
    r = _S.get("result") or {}
    rows = _S["rows"]
    log("\n" + "=" * 78)
    if SEED is not None:
        log("★ 이 실행은 seed %s 다 — 사양 seed 42 의 베이스라인이 아니다." % SEED)
    log("투입 순서     %s%s" % (ORDER,
        "  (부피 내림차순 — B5 데모와 같은 조건)" if ORDER == "volume" else
        "  (뽑힌 그대로 — 실제 라인 조건)"))
    if "trace" in r:
        log("!! 예외\n" + r["trace"])
    log("  #  박스       종류  결과         오차 XY / Z [mm]   대기 s  사이클 s  knock mm")
    log("-" * 78)
    for i, w in enumerate(rows, 1):
        exy = float(np.linalg.norm(w["err"][:2]))
        cell = ("%6.1f / %+6.1f" % (exy, w["err"][2])) if np.isfinite(exy) else "     -  /     - "
        log("%3d  %-10s %-4s  %-11s %s   %6.1f   %6.1f   %7.1f"
            % (i, w["id"], w["type"], w["res"], cell,
               w["wait"], w["wall"], w["knock"]))
    log("-" * 78)

    placed = [w for w in rows if w["res"] == "PLACED"]
    bypassed = [w for w in rows if w["res"] == "PALLET_FULL"]
    failed = [w for w in rows if w["res"] not in ("PLACED", "PALLET_FULL")]
    ok = bool(placed)
    log("처리한 것     %d / %d   (놓음 %d · 우회 %d · 실패 %d)"
        % (len(rows), N, len(placed), len(bypassed), len(failed)))
    if bypassed:
        log("우회          %s — 이 팔레트에 자리가 없는 박스다. 설계결정서 §7 상"
            % ", ".join("%s(%s)" % (w["id"], w["type"]) for w in bypassed))
        log("              버퍼 우회는 흐름 제어이지 실패가 아니다(bypass_count).")
    if failed:
        log("!! 실패       %s"
            % ", ".join("%s(%s)=%s" % (w["id"], w["type"], w["res"]) for w in failed))
    if placed:
        zs = sorted({int(w["place"]["z_mm"]) for w in placed if w["place"]})
        tops = [w["place"]["z_mm"] + next(t["h_mm"] for t in cfg["boxes"]["types"]
                                          if t["name"] == w["type"])
                for w in placed if w["place"]]
        log("단            %d 단  (밑면 z %s mm)" % (len(zs), zs))
        log("최상단        %d mm  (한계 %d)" % (max(tops), cfg["pallet"]["max_stack_mm"]))
        exy = [float(np.linalg.norm(w["err"][:2])) for w in placed]
        ez = [abs(float(w["err"][2])) for w in placed]
        kn = max(w["knock"] for w in rows)
        cyc_s = [w["wall"] for w in placed]
        log("배치 오차     XY 최대 %.1f mm · Z 최대 %.1f mm  (허용 %d / %d)"
            % (max(exy), max(ez), D.PLACE_TOL_XY_MM, D.PLACE_TOL_Z_MM))
        log("최대 knock    %.1f mm  (경고선 %.0f)" % (kn, D.KNOCK_WARN_MM))
        log("사이클        평균 %.1f s · 최대 %.1f s   (목표 %.0f s · 스폰 간격 %.0f s)"
            % (sum(cyc_s) / len(cyc_s), max(cyc_s),
               cfg["robot"]["cycle_time_target_s"], SPAWN_DT))
        log("팔레트        %d 판" % r.get("pallets", 1))
        log("벽시계        %.0f s" % r.get("wall", 0.0))
        # 합격 기준. ★ "N 개 전부 놓임" 이 아니다 — 자리가 없어 우회한 것은 사양상
        #   실패가 아니고(설계결정서 §7), 도착 순서로 15 개가 다 들어가는지는 **패커의
        #   상한** 문제라 이 게이트가 물을 것이 아니다 (eval_pack_online.py 가 답한다).
        #   여기가 물어야 하는 것은 "라인이 15 개를 끝까지 처리했는가" 다.
        all_handled = len(rows) == N and not failed
        ok = (all_handled
              and max(exy) < D.PLACE_TOL_XY_MM and max(ez) < D.PLACE_TOL_Z_MM
              and kn < D.KNOCK_WARN_MM
              and max(tops) <= cfg["pallet"]["max_stack_mm"])
        for cond, msg in ((all_handled,
                           "%d 개를 끝까지 처리해야 한다 (놓거나 우회. BLOCKED 는 실패)" % N),
                          (max(exy) < D.PLACE_TOL_XY_MM, "XY 오차 %d mm 안" % D.PLACE_TOL_XY_MM),
                          (max(ez) < D.PLACE_TOL_Z_MM, "Z 오차 %d mm 안" % D.PLACE_TOL_Z_MM),
                          (kn < D.KNOCK_WARN_MM, "knock %d mm 안" % D.KNOCK_WARN_MM),
                          (max(tops) <= cfg["pallet"]["max_stack_mm"], "최대 적재 높이 안")):
            if not cond:
                log("  !! %s" % msg)
    log("-" * 78)
    log("RESULT: %s" % ("통과" if ok else "실패"))
    log("=" * 78 + "\n")
    for k in ("link", "spawn_link"):
        try:
            if _S.get(k):
                _S[k].close()
        except Exception:
            pass
    try:
        if _S.get("server"):
            _S["server"].close()
    except Exception:
        pass
    omni.timeline.get_timeline_interface().stop()
    omni.kit.app.get_app().post_quit()


def on_step(dt):
    if _S["phase"] == "done":
        return
    if _S["phase"] == "boot":
        _S["n"] += 1
        if _S["n"] < BOOT_STEPS:
            return
        try:
            setup()
        except BaseException:
            import traceback
            log("!! setup 실패\n" + traceback.format_exc())
            _S["phase"] = "done"
            report()
            return
        _S["phase"] = "settle"
        _S["n"] = 0
        return
    if _S["phase"] == "settle":
        _S["n"] += 1
        _S["art"].apply_action(_S["ready_action"])       # 안 걸면 옛 목표로 튄다
        if _S["n"] >= SETTLE_STEPS:
            log("자세      수렴 %s deg"
                % np.round(np.degrees(_S["art"].get_joint_positions()[:6]), 1).tolist())
            if SEED is not None:
                _c = dict(cfg)
                _c["boxes"] = dict(cfg["boxes"], random_seed=int(SEED))
                fed = D.manifest(_c, N)
                log("★ seed %s 로 덮어썼다 — 사양(42)이 아니다. 베이스라인 아님." % SEED)
            else:
                fed = D.manifest(cfg, N)
            if ORDER == "volume":
                fed = sorted(fed, key=lambda t: -(t["w_mm"] * t["d_mm"] * t["h_mm"]))
                log("★ 투입 순서 = 부피 내림차순 (B5 데모와 같은 조건). "
                    "상자 구성은 사양 seed 그대로다.")
            ids = [BOX_ID_FMT % i for i in range(N)]
            specs = {bid: dict(t) for bid, t in zip(ids, fed)}
            log("박스      %d 개  %s" % (N, [t["name"] for t in fed]))
            log("-" * 78)
            _S["phase"] = "run"
            _S["t0"] = time.time()
            threading.Thread(target=worker, args=(specs, ids),
                             daemon=True, name="stack15").start()
        return
    _S["q"].pump()
    _S["rm"].step()                                      # ★ world.step() 앞, 메인 스레드
    _S["sp"].step()
    if _S["done"] or time.time() - _S["t0"] > WATCHDOG_S:
        _S["phase"] = "done"
        report()


_sub = get_physx_interface().subscribe_physics_step_events(on_step)
log("Play — 부팅 대기...\n")
