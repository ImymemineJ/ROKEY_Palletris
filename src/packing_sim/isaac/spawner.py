"""A 소유 소켓 핸들러 — 박스 스폰 · 스토퍼 감지 · pose 조회 (작업명세서 §5.5~§5.6).

    q  = mainq.MainQueue()
    sp = Spawner(stage, cfg, q, zone="B")
    server.register(sp.handlers())
    while running:
        q.pump()                  # ★ 매 물리 스텝, 메인 스레드
        sp.step()
        world.step(render=True)

프로토콜 표(공통가이드 §3) 중 여기서 맡는 것:
    spawn_box · spawn_status · set_spawn_enabled · despawn_box
    pickup_check · get_box_pose

## ★ 박스 프림 경로는 `/World/boxes/<box_id>` 다 (규칙 5)

`robot_motion.py` 의 `BOX_PRIM_FMT` 와 **글자 하나까지** 같아야 한다. USD 경로는
대소문자를 구분한다 — 기존 `spawn_feeder.py` 는 `/World/Boxes/Box_00_No3` 에
만드는데, 그건 규칙 5 와 다르므로 B4 의 `grasp(box_id)` 가 프림을 못 찾는다.
**이 파일이 인터페이스 경로이고, `spawn_feeder.py` 는 단독 데모 경로다.** 섞지 않는다.

## ★ 스폰 z 는 밑면 기준이다 (§5.5, 함정 #7)

`conveyor.spawn_point_mm` 은 **박스 밑면 중심**이다. prim 의 translate 는 중심이므로
`+ h/2` 를 반드시 더한다. 810 을 그대로 쓰면 5호 밑면이 640 이 되어 벨트 상면(800)
보다 160 mm 아래 — 즉시 관통해서 물리가 터진다.

## ★ 물리 속성은 **박스마다** 건다 (§5.4)

`solverPositionIterationCount` 를 PhysicsScene 에 걸면 무시되고 스택이 무너진다.
리지드바디별 속성이라 여기 스포너에서 박스마다 걸어야 한다.

## 스레드 규약

핸들러는 **소켓 스레드**에서, `step()` 은 **메인(물리) 스레드**에서 돈다.
USD 를 고치는 일(스폰·디스폰)은 소켓 스레드에서 하면 안 되므로 `MainQueue` 에
넣고 메인 스레드가 처리한다 (`mainq.py` — 왜 그런지 실측 근거가 거기 있다).
읽기(pickup_check·get_box_pose)는 `step()` 이 갱신해 둔 스냅샷을 보므로
락 없이 안전하다.
"""
import math

from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

BOX_ROOT = "/World/boxes"                 # ★ 규칙 5. 소문자다
BOX_PRIM_FMT = BOX_ROOT + "/%s"
LOOKS_ROOT = "/World/Looks"

BOX_SLUG = {"3호": "No3", "4호": "No4", "5호": "No5"}
BOX_COLOR = {"3호": (1.0, 0.15, 0.15),
             "4호": (0.15, 0.9, 0.25),
             "5호": (0.2, 0.35, 1.0)}
# 스토퍼 앞 이 거리 안에 박스 **앞면**이 오고 거의 멎으면 픽업 대기로 본다.
PICKUP_MARGIN_M = 0.060                   # 트랙A 가이드 §A6~A8 의 60 mm
PICKUP_VEL_MPS = 0.01                     # |vx| < 1 cm/s
# 벨트 상면 기준 높이 허용오차. 흡착으로 들린 박스를 걸러 내는 것이 목적이라
# 넉넉해도 된다 — 가장 낮은 박스(3호 210)의 절반보다는 작아야 한다.
PICKUP_Z_TOL_M = 0.05
# despawn 한 박스를 옮겨 둘 자리. 라인·팔레트·로봇 어디에도 안 걸린다.
DESPAWN_XYZ_M = (-50.0, -50.0, -10.0)


def _yaw_deg(quat):
    """(w, x, y, z) -> yaw [deg]."""
    w, x, y, z = [float(v) for v in quat]
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


class Spawner:
    def __init__(self, stage, cfg, mainq, zone="B", log=print):
        self.stage = stage
        self.cfg = cfg
        # ★ USD 쓰기(스폰·디스폰)는 메인 스레드에서. mainq.py docstring 참조.
        self.mainq = mainq
        self.zone = zone
        self._log = log

        self.dt = 1.0 / float(cfg["physics"]["scene"]["time_steps_per_second"])
        self.types = {t["name"]: t for t in cfg["boxes"]["types"]}
        sp = cfg["conveyor"]["spawn_point_mm"]
        self.spawn_xyz_m = [v / 1000.0 for v in sp]
        self.stopper_x_m = cfg["conveyor"]["stopper_x_mm"] / 1000.0
        # 픽업 판정 구간. **위쪽 경계가 없으면 팔레트에 놓인 박스가 다시 잡힌다** —
        # 아래 `_refresh` 주석의 실측 참조.
        self.pickup_x0_m = cfg["conveyor"]["pickup_range_mm"][0] / 1000.0
        self.belt_top_m = cfg["conveyor"]["belt_top_z_mm"] / 1000.0
        self.default_yaw = float(cfg["boxes"].get("spawn_yaw_deg", 0.0))

        phys = cfg["physics"]
        self.rb_pos_iter = int(phys["rigid_body"]["solver_position_iterations"])
        self.rb_vel_iter = int(phys["rigid_body"]["solver_velocity_iterations"])
        self.rest_offset_m = phys["collider"]["rest_offset_mm"] / 1000.0
        self.contact_offset_m = phys["collider"]["contact_offset_mm"] / 1000.0

        self.enabled = True
        self.spawned = 0
        self.boxes = {}                    # box_id -> {"path", "dims", "type"}
        self._snap = {}                    # box_id -> {"pos_m", "yaw_deg", "vel_mps"}
        self._pickup = {"present": False, "box_id": "", "pose": None}

        self._ensure_roots()
        self._log("[spawner] 스폰점 %s m (밑면) · 픽업 판정 x %.3f~%.3f · 벨트 상면 %.3f "
                  "· 경로 %s"
                  % (self.spawn_xyz_m, self.pickup_x0_m, self.stopper_x_m,
                     self.belt_top_m, BOX_ROOT))

    # ── 등록 ────────────────────────────────────────────────
    def handlers(self):
        return {"spawn_box": self.h_spawn_box,
                "spawn_status": self.h_spawn_status,
                "set_spawn_enabled": self.h_set_spawn_enabled,
                "despawn_box": self.h_despawn_box,
                "pickup_check": self.h_pickup_check,
                "get_box_pose": self.h_get_box_pose}

    # ── 메인 스레드 ─────────────────────────────────────────
    def step(self):
        """매 물리 스텝 한 번. **메인 스레드 전용.**

        큐 처리는 `sim_main` 이 `mainq.pump()` 로 한다 — 여기서는 스냅샷만 갱신한다.
        """
        self._refresh()

    def _refresh(self):
        """모든 박스의 pose/속도를 한 번 읽어 스냅샷과 픽업 상태를 갱신한다.

        ★ **USD 에서 읽는다. `SingleRigidPrim` 을 박스마다 만들지 않는다.**

        예전엔 박스마다 `SingleRigidPrim` 을 만들어 `get_world_pose()` 를 불렀다.
        두 가지가 터졌다 (실측 2026-08-22):

          1) 시뮬레이션이 도는 중에 만든 뷰가 **엉뚱한 값**을 돌려줬다 —
             x=0.5 에 스폰한 박스를 x=10.3 으로 보고했다.
          2) 그 뷰가 물고 있는 프림을 지우면 **시뮬레이션 뷰 전체가 무효화**된다:
               "prim was deleted while being used by a shape in a tensor view class"
               "Simulation view object is invalidated ... getDofPositions"
             즉 박스 하나 despawn 에 **로봇 아티큘레이션까지 같이 죽는다.**
             실제로 프로세스가 그대로 내려갔다.

        USD 는 물리 스텝마다 갱신되므로 pose 는 그대로 읽으면 되고, 속도는
        프레임 간 차분으로 낸다. 스토퍼 판정(|vx| < 1 cm/s)에는 이 정밀도로 충분하다.
        """
        from pxr import UsdGeom as _UG
        cache = _UG.XformCache()
        snap = {}
        best = None
        for box_id, rec in list(self.boxes.items()):
            prim = self.stage.GetPrimAtPath(rec["path"])
            if not prim or not prim.IsValid():
                continue
            m = cache.GetLocalToWorldTransform(prim)
            t = m.ExtractTranslation()
            pos = [float(t[0]), float(t[1]), float(t[2])]
            q = m.ExtractRotationQuat()
            im = q.GetImaginary()
            yaw = _yaw_deg((q.GetReal(), im[0], im[1], im[2]))
            prev = self._snap.get(box_id)
            if prev is None:
                vel = [0.0, 0.0, 0.0]
            else:
                vel = [(pos[i] - prev["pos_m"][i]) / self.dt for i in range(3)]
            entry = {"pos_m": pos, "yaw_deg": yaw, "vel_mps": vel}
            snap[box_id] = entry
            # 스토퍼 판정 — 박스 **앞면**이 기준이다 (§5.6)
            #
            # ★★ 조건이 셋이다. 예전에는 "앞면이 스토퍼 근처보다 앞" + "거의 정지"
            #    둘뿐이었는데, **위쪽 경계가 없어서 팔레트에 놓인 박스가 계속
            #    present 로 잡혔다.** 팔레트는 x 10300~11500 이라 스토퍼(10000)보다
            #    앞이고, 놓인 박스는 당연히 멎어 있다. 그래서 `best` 가 늘 팔레트 위
            #    박스가 되어 로봇이 **방금 놓은 것을 다시 집었다**
            #    (2026-08-23 실측: 15박스 연속 중 2번째 사이클부터 BOX00000 을 계속
            #     재차 집어 knock 338 mm, 새 박스는 영영 안 집혔다).
            #
            #    1) 중심이 픽업 구간 [pickup_range[0], stopper_x] 안 — 팔레트를 제외한다
            #    2) 앞면이 스토퍼 마진 안 — 실제로 벽에 닿아 섰다
            #    3) 밑면이 벨트 상면 — 집혀서 들린 것과 라인 밖으로 떨어진 것을 제외한다
            #    여기에 원래의 "거의 정지" 가 더해진다.
            w, h = rec["dims"][0], rec["dims"][2]
            front = pos[0] + w / 2.0
            in_zone = self.pickup_x0_m <= pos[0] <= self.stopper_x_m
            at_stopper = front > self.stopper_x_m - w / 2.0 - PICKUP_MARGIN_M
            on_belt = abs(pos[2] - (self.belt_top_m + h / 2.0)) < PICKUP_Z_TOL_M
            if (in_zone and at_stopper and on_belt
                    and abs(entry["vel_mps"][0]) < PICKUP_VEL_MPS):
                # 여러 개가 줄 서면 **가장 앞선 것**이 집힐 차례다
                if best is None or pos[0] > best[1]["pos_m"][0]:
                    best = (box_id, entry)
        self._snap = snap
        if best is None:
            self._pickup = {"present": False, "box_id": "", "pose": None}
        else:
            box_id, e = best
            self._pickup = {"present": True, "box_id": box_id,
                            "pose": {"pos_m": e["pos_m"], "yaw_deg": e["yaw_deg"]}}

    # ── 소켓 핸들러 ─────────────────────────────────────────
    def h_spawn_box(self, req):
        """{"cmd":"spawn_box","box_id":"BOX00000","type":"4호","yaw_deg":0}"""
        box_id = req.get("box_id")
        tname = req.get("type")
        if not box_id:
            return {"ok": False, "error": "box_id 가 없다"}
        if tname not in self.types:
            return {"ok": False, "error": "모르는 type %r (있는 것: %s)"
                    % (tname, sorted(self.types))}
        if not self.enabled:
            return {"ok": False, "error": "spawn disabled"}
        if box_id in self.boxes:
            return {"ok": False, "error": "이미 있는 box_id %s" % box_id}
        yaw = float(req.get("yaw_deg", self.default_yaw))
        return self.mainq.run(lambda: self._spawn(box_id, tname, yaw))

    def h_despawn_box(self, req):
        box_id = req.get("box_id")
        if box_id not in self.boxes:
            return {"ok": False, "error": "없는 box_id %s" % box_id}
        return self.mainq.run(lambda: self._despawn(box_id))

    def h_spawn_status(self, req=None):
        return {"spawned": self.spawned, "enabled": bool(self.enabled),
                "on_belt": len(self.boxes)}

    def h_set_spawn_enabled(self, req):
        self.enabled = bool(req.get("enabled", True))
        self._log("[spawner] 스폰 %s" % ("ON" if self.enabled else "OFF"))
        return {"ok": True}

    def h_pickup_check(self, req=None):
        """스토퍼에 도착해 멎은 박스를 보고한다 (§5.6).

        zone 은 프로토콜 표에 있지만 stage 1 은 B 하나뿐이라 받기만 한다.
        """
        return dict(self._pickup)

    def h_get_box_pose(self, req):
        box_id = req.get("box_id")
        e = self._snap.get(box_id)
        if e is None:
            return {"ok": False, "error": "없는 box_id %s" % box_id}
        return {"ok": True, "pos_m": e["pos_m"], "yaw_deg": e["yaw_deg"]}

    # ── 내부 ────────────────────────────────────────────────
    def _ensure_roots(self):
        if not self.stage.GetPrimAtPath(BOX_ROOT):
            UsdGeom.Xform.Define(self.stage, BOX_ROOT)
        if not self.stage.GetPrimAtPath(LOOKS_ROOT):
            UsdGeom.Scope.Define(self.stage, LOOKS_ROOT)
        for name, rgb in BOX_COLOR.items():
            mp = "%s/Box%s" % (LOOKS_ROOT, BOX_SLUG[name])
            if self.stage.GetPrimAtPath(mp):
                continue
            mat = UsdShade.Material.Define(self.stage, mp)
            sh = UsdShade.Shader.Define(self.stage, mp + "/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")

    def _spawn(self, box_id, tname, yaw_deg):
        t = self.types[tname]
        dims = (t["w_mm"] / 1000.0, t["d_mm"] / 1000.0, t["h_mm"] / 1000.0)
        mass = t["mass_g"] / 1000.0
        x, y, z_bottom = self.spawn_xyz_m
        pos = (x, y, z_bottom + dims[2] / 2.0)          # ★ 밑면 -> 중심 (§5.5)
        path = BOX_PRIM_FMT % box_id
        self._make_box(path, tname, dims, pos, yaw_deg, mass, box_id)
        self.boxes[box_id] = {"path": path, "dims": dims, "type": tname}
        self.spawned += 1
        self._log("[spawner] %s %s %dx%dx%d mm %.1f kg yaw %+.0f -> %s"
                  % (box_id, tname, t["w_mm"], t["d_mm"], t["h_mm"], mass,
                     yaw_deg, path))
        return {"ok": True}

    def _despawn(self, box_id):
        """박스를 작업 영역에서 치운다.

        ⚠ **프림을 지우지 않는다.** 시뮬레이션이 도는 중에 물리 프림을 지우면
          PhysX 텐서 뷰가 통째로 무효화돼 **로봇 아티큘레이션까지 죽는다**
          (실측 2026-08-22, `_refresh` docstring 참조). 그래서 대신
          작업 영역 밖으로 옮기고 목록에서 뺀다. 겉보기·계측 모두 사라진 것과
          같고, 프림은 다음 `world.reset()` 때 정리하면 된다.

        Day 4 재순환(`/sim/despawn_box`, §2.6)에는 이 동작으로 충분하다.
        """
        rec = self.boxes.pop(box_id, None)
        if rec is None:
            return {"ok": False, "error": "없는 box_id %s" % box_id}
        prim = self.stage.GetPrimAtPath(rec["path"])
        if prim and prim.IsValid():
            # 물리에서 떼어내고(키네마틱) 라인 밖으로 옮긴다.
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
            xf = UsdGeom.Xformable(prim)
            for op in xf.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    op.Set(Gf.Vec3d(DESPAWN_XYZ_M))
                    break
        self._snap.pop(box_id, None)
        self._log("[spawner] despawn %s (라인 밖으로 이동)" % box_id)
        return {"ok": True}

    def _make_box(self, path, tname, dims, pos, yaw_deg, mass, box_id):
        """치수를 지오메트리에 굽는다 — `xformOp:scale` 을 안 쓴다.

        `spawn_feeder._make_box` 와 같은 형상이다. 거기 주석대로 scale 자체는
        흡착과 무관하지만(2026-08-22 정정), 치수를 구운 Mesh 가 어차피 깔끔하고
        `belt_cycle._box_dims` 도 extent 를 먼저 본다.

        여기에만 있는 것: **§5.4 의 박스별 물리 속성**과 `box_id` 커스텀 어트리뷰트.
        """
        hx, hy, hz = dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        prim = mesh.GetPrim()

        pts = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
               (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
        mesh.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
        mesh.CreateFaceVertexCountsAttr([4] * 6)
        mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4,
                                          1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
        mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
        mesh.CreateSubdivisionSchemeAttr("none")     # 없으면 모서리가 둥글어진다
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*BOX_COLOR[tname])])

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        half = math.radians(yaw_deg) / 2.0
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half))))
        # ★ AddScaleOp 을 넣지 않는다.

        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateRestOffsetAttr(self.rest_offset_m)         # ★ 단위 m (§5.4)
        col.CreateContactOffsetAttr(self.contact_offset_m)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        # ★ 박스마다. PhysicsScene 에 걸면 무시되고 스택이 무너진다 (§5.4)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(self.rb_pos_iter)
        rb.CreateSolverVelocityIterationCountAttr(self.rb_vel_iter)

        # ground truth 조회용 (§5.5). DB 가 없어도 씬만으로 id 를 되찾을 수 있다.
        prim.CreateAttribute("box_id", Sdf.ValueTypeNames.String).Set(box_id)
        prim.CreateAttribute("box_type", Sdf.ValueTypeNames.String).Set(tname)

        mat = UsdShade.Material(
            self.stage.GetPrimAtPath("%s/Box%s" % (LOOKS_ROOT, BOX_SLUG[tname])))
        if mat:
            UsdShade.MaterialBindingAPI.Apply(prim)
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
