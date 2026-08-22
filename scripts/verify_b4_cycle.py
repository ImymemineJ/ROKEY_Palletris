#!/usr/bin/env python3
"""B4 — pick & place **단일 사이클**. 소켓 프로토콜 위에서 끝까지 돈다.

    ~/isaac-sim-5.1.0/python.sh scripts/verify_b4_cycle.py
    ~/isaac-sim-5.1.0/python.sh scripts/verify_b4_cycle.py --gui --hold

## 무엇을 검증하나

명세서 §2.5 정상 시퀀스를 **실제 산출물 그대로** 돌린다.

    packing_robot/cycle.py      상태머신   ← 이 스크립트가 그대로 돌린다
    packing_sim/isaac/robot_motion.py  B 명령 핸들러 6개
    packing_robot/gripper.py    흡착
    packing_robot/sim_link.py   개행 JSON 소켓

테스트 전용 사본이 하나라도 끼면 이 통과가 실물의 통과를 보장하지 못한다.
그래서 여기서 새로 만드는 것은 **둘뿐**이다.

  1. **소켓 서버 대역** — 원래 `packing_sim/isaac/sim_server.py`(김학민)다. 아직 없어서
     같은 프로토콜의 최소 서버를 여기 둔다. A 것이 들어오면 이 클래스만 지운다.
  2. **§2.7 Mock 서비스** — `/db/*` `/algo/*` `/conveyor/*`. 명세서 표의 값 그대로다.
     (`get_box_info` 는 항상 4호, `plan_placement` 는 (205,155,0,0.0))

## 스레드 배치 — 규칙 1 이 실제로 지켜지는지 보는 것이 절반이다

    메인 스레드   : rm.step() -> world.step()          물리
    소켓 스레드   : 명령 핸들러                          연결마다 하나
    사이클 스레드 : Cycle.run()                          SimLink 클라이언트

`grasp` / `release` 는 소켓 스레드에서 결과를 기다린다. **물리가 그동안 멈추면 영원히
결과가 안 나온다** — 이 스크립트가 끝까지 돌면 안 멈춘 것이다. 스텝 수도 같이 찍는다.
"""
import argparse
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                          # noqa: E402
import verify_lula_ik as V                             # noqa: E402
import verify_suction as VS                            # 씬 만들기 재사용  # noqa: E402

sys.path.insert(0, str(_paths.ROOT / "src/packing_sim/isaac"))

BOX_ID = "BOX00000"
BOX_PRIM = "/World/boxes/%s" % BOX_ID                  # 공통가이드 §3 규칙 5
GRIPPER = "/World/h2017/SurfaceGripper"
PORT = 8799                                            # 8765 는 A 것. 충돌 피한다


# ─────────────────────────────────────────────────────────────
class StandInServer:
    """`sim_server.py`(김학민) 대역. 개행 JSON, 요청-응답, **연결 여러 개**.

    규칙 3 대로 클라이언트를 동시에 여러 개 받는다 — 단일 연결로 짜면 브릿지와
    오케스트레이터 중 하나가 조용히 멈춘다.
    """

    def __init__(self, handlers, port=PORT, log=print):
        self.handlers = dict(handlers)
        self._log = log
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(8)
        self.srv.settimeout(0.2)
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._accept, daemon=True)
        self._t.start()

    def _accept(self):
        while not self._stop.is_set():
            try:
                c, _ = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def _serve(self, conn):
        f = conn.makefile("rw")
        while not self._stop.is_set():
            line = f.readline()
            if not line:
                break
            try:
                req = json.loads(line)
                fn = self.handlers.get(req.get("cmd"))
                # 규칙 2 — 모르는 cmd 에 크래시하지 않는다
                res = fn(req) if fn else {"ok": False, "error": "unknown_cmd"}
            except Exception as e:
                res = {"ok": False, "error": repr(e)}
            f.write(json.dumps(res) + "\n")
            f.flush()
        conn.close()

    def close(self):
        self._stop.set()
        self.srv.close()


class MockServices:
    """§2.7 Mock 규약 그대로. 값은 명세서 표에서 왔다."""

    def __init__(self, log=print, place=None):
        self._log = log
        self.calls = []
        self.place = place or {"x_mm": 205, "y_mm": 155, "z_mm": 0, "yaw_deg": 0.0}

    def _rec(self, name, *a):
        self.calls.append((name,) + a)
        self._log("             [mock] %s%s" % (name, a if a else ""))

    def set_velocity(self, seg, mps):
        self._rec("/conveyor/set_velocity", seg, mps)
        return True

    def get_box_info(self, box_id):
        self._rec("/db/get_box_info", box_id)
        return {"w_mm": 410, "d_mm": 310, "h_mm": 280, "zone": "B"}   # 항상 4호

    def plan_placement(self, zone, box_id, w, d, h):
        self._rec("/algo/plan_placement", zone, box_id)
        return True, dict(self.place), "OK"

    def set_status(self, box_id, status):
        self._rec("/db/set_status", box_id, status)
        return True

    def mark_placed(self, box_id, pose):
        self._rec("/db/mark_placed", box_id)
        return True

    def update_heightmap(self, zone, box_id, w, d, h, actual):
        self._rec("/algo/update_heightmap", box_id)
        return True, 0.5

    def cancel_plan(self, zone, box_id):
        self._rec("/algo/cancel_plan", box_id)
        return True


# ─────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zone", default="B", choices=["A", "B", "C"])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--place", type=int, nargs=3, default=None, metavar=("PX", "PY", "PZ"),
                    help="Mock plan_placement 을 덮어쓴다 [mm]. 기본은 명세서 §2.7 의 (205,155,0)")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--hold", action="store_true")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    _paths.check()
    if not V.ASSET_SUCTION.exists():
        sys.exit("no asset: %s\n  -> scripts/build_gripper_usd.py 를 먼저 돌릴 것"
                 % V.ASSET_SUCTION)

    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": not a.gui})
    tee = V.start_log(a.log or _paths.ROOT / "results" / "verify_b4_cycle.txt")

    import robot_motion as RM                          # packing_sim/isaac
    import cycle as CY                                 # packing_robot
    import sim_link as SL

    ok_all = True
    server = link = None
    try:
        S = V.bring_up(zone=a.zone, render=a.gui, asset=V.ASSET_SUCTION)
        stage, cfg = S.world.stage, S.cfg
        spec = next(t for t in cfg["boxes"]["types"] if t["name"] == "4호")   # Mock 과 동일
        dims = [spec["w_mm"] / 1000.0, spec["d_mm"] / 1000.0, spec["h_mm"] / 1000.0]
        mass = spec["mass_g"] / 1000.0
        half_h = dims[2] / 2.0
        belt_z = cfg["conveyor"]["belt_top_z_mm"] / 1000.0
        pick_x = cfg["conveyor"]["stopper_x_mm"] / 1000.0
        origin = cfg["zones"]["pallet_origin_world_mm"][a.zone]

        VS.BOX = BOX_PRIM                              # 규칙 5 경로로 스폰한다
        # ★ 벨트가 팔레트를 덮으면 놓으러 내려갈 때 팔이 벨트에 막힌다.
        #   스토퍼에서 끝나게 잘라 둔다 — 팔레트는 x 10.3 부터다.
        VS.make_belt(stage, belt_z, pick_x - 0.35, 0.0, size=(1.2, 1.0))
        VS.make_pallet(stage, origin, cfg)
        VS.make_pedestal(stage, S.base, cfg)      # 없으면 로봇이 공중에 떠 보인다
        box_pos = (pick_x, 0.0, belt_z + half_h + 0.001)
        VS.make_box(stage, "mesh", dims, box_pos, mass)

        from isaacsim.core.prims import SingleRigidPrim
        S.world.reset()
        S.art.initialize()
        box_rp = SingleRigidPrim(prim_path=BOX_PRIM, name="box",
                                 reset_xform_properties=False)
        box_rp.initialize()
        V.teleport(S, S.ready, steps=int(0.3 * S.hz))
        for _ in range(int(0.7 * S.hz)):
            S.world.step(render=S.render)

        print("ASSET   %s" % V.ASSET_SUCTION)
        print("ZONE    %s   base %s mm   팔레트 원점 %s mm"
              % (a.zone, list(np.round(S.base * 1000).astype(int)), origin))
        print("박스    %s  4호 %s mm  %.0f kg   프림 %s"
              % (BOX_ID, [int(v * 1000) for v in dims], mass, BOX_PRIM))

        # ── 몸: robot_motion + 소켓 서버 대역 ───────────────
        rm = RM.RobotMotion(S.art, S.lula, cfg, ee=S.ee, gripper_path=GRIPPER,
                            gap_fn=lambda: VS.measure(S, box_rp, half_h)[0])
        handlers = dict(rm.handlers())

        def h_get_box_pose(req):                       # A 소유 명령. 대역이라 여기 둔다
            pos, quat = box_rp.get_world_pose()
            w, x, y, z = [float(v) for v in quat]
            yaw = math.degrees(math.atan2(2 * (w * z + x * y),
                                          1 - 2 * (y * y + z * z)))
            return {"ok": True, "pos_m": [float(v) for v in pos], "yaw_deg": yaw}

        handlers["get_box_pose"] = h_get_box_pose
        handlers["ping"] = lambda req: {"ok": True}
        server = StandInServer(handlers, port=a.port)
        print("소켓    127.0.0.1:%d  cmd %s" % (a.port, sorted(handlers)))

        # ── 머리: Cycle 을 별도 스레드에서 ─────────────────
        link = SL.SimLink(port=a.port)
        place = ({"x_mm": a.place[0], "y_mm": a.place[1], "z_mm": a.place[2],
                  "yaw_deg": 0.0} if a.place else None)
        svc = MockServices(place=place)
        cyc = CY.Cycle(link, svc, cfg, zone=a.zone)
        pick_pose = {"pos_m": list(box_rp.get_world_pose()[0]), "yaw_deg": 0.0}
        print("\n[사이클 시작] 박스 중심 %s" % np.round(pick_pose["pos_m"], 4))

        box = {}
        th = threading.Thread(target=lambda: box.update(r=cyc.run(BOX_ID, pick_pose)),
                              daemon=True)
        th.start()

        # ── 메인 스레드는 물리만 돌린다 ────────────────────
        steps = 0
        t0 = time.time()
        while th.is_alive() and time.time() - t0 < 120.0:
            rm.step()
            S.world.step(render=S.render)
            steps += 1
        th.join(timeout=5.0)
        wall = time.time() - t0
        res = box.get("r", "TIMEOUT")

        pos = np.asarray(box_rp.get_world_pose()[0], dtype=float)
        want = CY.pallet_place_to_world(svc.place, spec["h_mm"], origin)
        err_mm = (pos - np.asarray(want)) * 1000.0

        print("\n[결과]  %s   물리 %d 스텝 (%.1f s 벽시계, %.1f s 시뮬)"
              % (res, steps, wall, steps / S.hz))
        print("        박스 최종 %s m" % np.round(pos, 4))
        print("        목표     %s m" % np.round(want, 4))
        print("        오차     [%+.1f %+.1f %+.1f] mm" % tuple(err_mm))
        print("        흡착 상태 %s   is_holding %s" % (rm.grip.status(), rm.grip.is_holding()))

        placed = res == "PLACED"
        on_target = float(np.linalg.norm(err_mm[:2])) < 30.0 and abs(err_mm[2]) < 20.0
        spun = steps > 100                     # 물리가 실제로 돌았는가 (규칙 1)
        for cond, msg in ((placed, "사이클이 PLACED 로 끝나야 한다 — %s" % res),
                          (on_target, "배치 오차가 XY 30 mm · Z 20 mm 안이어야 한다"),
                          (spun, "물리가 돌아야 한다 — 핸들러가 메인 루프를 세웠다")):
            if not cond:
                print("        !! %s" % msg)
            ok_all &= cond

        need = ["/conveyor/set_velocity", "/db/get_box_info", "/algo/plan_placement",
                "/db/set_status", "/algo/update_heightmap", "/db/mark_placed"]
        got = [c[0] for c in svc.calls]
        missing = [n for n in need if n not in got]
        print("        서비스 호출 %d 회%s" % (len(got), "" if not missing
                                            else "   !! 빠진 것 %s" % missing))
        ok_all &= not missing

        print("\nRESULT: %s" % ("통과" if ok_all else "실패"))
        if a.hold and a.gui:
            V.hold_open(S)
    except BaseException:
        import traceback
        print("\n!! 예외\n" + traceback.format_exc())
        raise
    finally:
        if link is not None:
            link.close()
        if server is not None:
            server.close()
        tee.close()
        kit.close()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
