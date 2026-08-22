"""벨트 간헐 구동 — 박스를 픽 지점에 세우고, 로봇 사이클이 끝나면 다시 보낸다.

스토퍼로 박스를 막아 세우는 대신 **벨트를 멈춰서** 세운다. 픽 지점이 항상
같으므로 로봇은 고정된 자세 하나로 집으면 된다.

    투입 -> 주행 -> 선두 박스가 픽 지점 도달 -> 벨트 정지
         -> 로봇 집기 -> 팔레트 적재 -> 원위치
         -> 벨트 재기동 -> 반복

로봇 붙이는 법
    아직 H2017 이 씬에 없어서 기본값은 **스텁**이다 (사양의 cycle_time_target_s
    동안 잡고 있다가 박스를 팔레트로 옮긴다). 진짜 로봇이 들어오면 스텁을
    갈아 끼운다 — 상태를 이 모듈이 들고 있으므로 로봇 쪽은 두 함수만 주면 된다.

        import belt_cycle
        belt_cycle.set_robot(on_pick=my_start_cycle, is_idle=my_is_home)
        belt_cycle.start()

    on_pick(box_path, pose)  픽 지점에 박스가 섰을 때 한 번 불린다. 사이클 시작.
    is_idle()               로봇이 적재를 끝내고 원위치했으면 True.
                            True 가 되는 순간 벨트가 다시 돈다.

벨트를 세우는 레버 (2026-08-22 전체 앱에서 재실측)
    ★ 한때 데크의 `physxSurfaceVelocity:surfaceVelocity` 를 직접 0 으로 썼다.
      **GUI 에서는 안 듣는다.** IsaacConveyor 노드가 매 물리 틱마다
      `direction x velocity` 를 그 어트리뷰트에 도로 써 넣기 때문이다.
      우리가 쓴 0 은 다음 틱에 지워진다.

      이걸 오래 못 잡은 이유는 **python.sh 헤드리스에서는 그래프가 안 돌아서
      잘 됐기 때문**이다. 헤드리스로만 검증하면 이 버그를 못 본다.
      재현하려면 창만 끄고 전체 앱으로 돌려야 한다:

          ./isaac-sim.sh --no-window --exec <스크립트>

      isaacsim.exp.full.kit 에서 레버 5종을 각각 23 스텝 유지 검사한 결과:

          surfaceVelocity = 0            0/23  실패 (노드가 다시 쓴다)
          inputs:enabled = False        23/23  성공
          graph:variable:Velocity = 0   23/23  성공   <- 이걸 쓴다
          og 런타임 변수 set            23/23  성공
          og inputs:enabled = False     23/23  성공

    지금은 **그래프 변수 `Velocity` 를 0 으로** 만든다. 노드가 직접 표면속도를
    0 으로 써 주므로 방향·커브(각속도) 처리를 노드에 그대로 맡길 수 있고,
    USD 어트리뷰트 하나만 건드리면 된다. og 임포트도 필요 없다.

    다시 돌릴 때는 속도를 되돌리는 것만으로는 안 될 수 있다 — 멈춰 있던 박스가
    잠들면(sleep) 표면속도를 다시 안 받는다. surfaceVelocityEnabled 를
    False -> True 로 토글해 깨운다.

벨트 속도
    노드가 쓰는 표면속도는 **데크 로컬** 값이라 프림 스케일이 곱해진다.
    이 씬은 x 스케일이 2.516 이라 변수 0.5 는 월드에서 1.26 m/s 가 된다.
    사양 belt_velocity_mps 가 월드에서 그대로 나오도록 데크의 월드 변환으로
    환산해 변수에 넣는다 (0.5 m/s -> 변수 0.19872). STOP_LEAD 는 0.5 m/s 에서
    잰 값이라, 속도를 안 맞추면 정지 선행이 통째로 틀린다.

환경 변수
    PL_PICK_X      픽 지점 [mm]. 기본 9760 = stopper_x_mm - 5호 폭/2.
                   도달 검증이 쓰는 값과 같다. 함부로 줄이면 리치를 넘는다.
    PL_STOP_LEAD   정지 명령을 얼마나 앞당길지 [mm]. 벨트를 세워도 박스가
                   관성으로 더 간다 — 0.5m/s 에서 실측 40mm 였다. 기본 40 은
                   그 값이다. 벨트 속도를 바꾸면 다시 재라.
    PL_CYCLE_S     스텁 로봇의 1 사이클 시간 [초]. 기본 10.0.
    PL_BELT_MPS    벨트 속도 [m/s]. 기본 0.5.
"""

import os

import omni.timeline
import omni.usd
from omni.physx import get_physx_interface
from pxr import Gf, Usd, UsdGeom, UsdPhysics

# 픽 지점 [mm]. 사양이 도달 검증(scripts/verify_reach.py)에서 쓰는 값과
# 같아야 한다 — 거기서는 박스 중심을 stopper_x_mm - w/2 로 잡는다.
# 가장 큰 5호(480) 기준 10000 - 240 = 9760. 이보다 앞(작은 x)에 세우면
# 로봇에서 멀어져 5호 픽업이 실효 리치를 넘는다 (9700 이면 1578mm).
PICK_X = float(os.environ.get("PL_PICK_X", "9760")) / 1000.0
STOP_LEAD = float(os.environ.get("PL_STOP_LEAD", "40")) / 1000.0
CYCLE_S = float(os.environ.get("PL_CYCLE_S", "10.0"))
def _spec_belt_mps():
    """사양 conveyor.belt_velocity_mps. 못 읽으면 0.5."""
    try:
        import yaml
        with open(os.path.expanduser(
                "~/palletris_ws/src/packing_bringup/config/system.yaml")) as fh:
            return float(yaml.safe_load(fh)["conveyor"]["belt_velocity_mps"])
    except Exception:
        return 0.5


BELT_MPS = float(os.environ.get("PL_BELT_MPS", _spec_belt_mps()))
SETTLE_MAX_S = 1.5      # 이 시간이 지나면 안 멎어도 진행한다 (교착 방지)

BOX_ROOT = "/World/Boxes"
PALLET = "/World/Pallet"

# 픽 지점을 이만큼 넘어가면 그 박스는 놓친 것으로 본다 [mm].
# 벨트 끝이 x=10000, 픽 지점이 9760 이라 여유가 240mm 뿐이다. 넘어간 박스는
# 바닥으로 떨어지므로 로봇에 넘기면 안 된다 — 도달범위 밖 좌표다.
RUNAWAY = float(os.environ.get("PL_RUNAWAY_MM", "150")) / 1000.0

# IsaacConveyor 노드가 데크에 써 넣는 PhysX 표면속도. **레버가 아니다** —
# 우리가 0 을 써도 노드가 다음 틱에 도로 쓴다 (docstring 의 실측표 참고).
# 재기동 때 잠든 박스를 깨우는 용도로만 남겨 둔다.
SURF_VEL = "physxSurfaceVelocity:surfaceVelocity"
SURF_ON = "physxSurfaceVelocity:surfaceVelocityEnabled"

# 진짜 레버. 그래프 변수 하나가 그 그래프에 딸린 컨베이어의 속도를 정한다.
VEL_VAR = "graph:variable:Velocity"
CONVEYOR_NODE = "isaacsim.asset.gen.conveyor.IsaacConveyor"

# 벨트 상면. start() 에서 실측값으로 덮어쓴다.
BELT_TOP_Z = 0.800

_state = {}
_robot = {"on_pick": None, "is_idle": None}


# ─────────────────────────────────────────────────────────────
def set_robot(on_pick, is_idle):
    """로봇 사이클을 꽂는다. 스텁 대신 진짜 로봇을 쓸 때 부른다."""
    _robot["on_pick"] = on_pick
    _robot["is_idle"] = is_idle


def start():
    stop()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("[cycle] 열린 스테이지가 없다.")

    global BELT_TOP_Z
    top = _belt_top(stage)
    if top is not None:
        BELT_TOP_Z = top

    # 구동부는 USD 만 보면 알 수 있다 — Play 를 기다릴 필요가 없다.
    drives = _find_drives(stage)
    if not drives:
        raise RuntimeError(
            "[cycle] IsaacConveyor 노드를 못 찾았다. 컨베이어 그래프가 있는 "
            "씬인지 확인해라 (test1-1.usd 면 /World/ConveyorTrack*/ConveyorBeltGraph).")

    _state.update(
        stage=stage, drives=drives, wake=0,
        phase="RUN", timer=0.0, box=None, placed=0, lost=0,
    )
    _set_running(True)

    _state["physics_sub"] = get_physx_interface().subscribe_physics_step_events(_on_step)
    _state["timeline_sub"] = (
        omni.timeline.get_timeline_interface()
        .get_timeline_event_stream()
        .create_subscription_to_pop(_on_timeline)
    )

    mode = "실제 로봇" if _robot["on_pick"] else f"스텁 ({CYCLE_S:.1f}초/사이클)"
    for d in drives:
        print(f"[cycle] 구동 {d['deck'].GetPath()}  "
              f"변수 {d['run']:.5f} x 스케일 {d['world']:.3f} = {BELT_MPS:.3f} m/s")
    print(f"[cycle] 픽 지점 x={PICK_X * 1000:.0f}mm  "
          f"정지 선행 {STOP_LEAD * 1000:.0f}mm  벨트면 {BELT_TOP_Z * 1000:.0f}mm  "
          f"벨트 {BELT_MPS:.2f} m/s  놓침 판정 +{RUNAWAY * 1000:.0f}mm")
    print(f"[cycle] 로봇: {mode}. Play 하면 박스가 픽 지점에 서고 벨트가 멈춘다.")


def stop():
    if _state.get("physics_sub") is not None:
        _state["physics_sub"] = None
    if _state.get("timeline_sub") is not None:
        _state["timeline_sub"] = None
    if _state.get("drives"):
        _restore_belt()         # 씬을 굴러가는 상태로 되돌려 놓고 나간다
    _state.clear()


def status():
    return {k: _state.get(k) for k in ("phase", "box", "placed", "timer", "lost")}


# ─────────────────────────────────────────────────────────────
def _find_drives(stage):
    """컨베이어 구동부를 찾는다. **Play 전에도 된다** — USD 만 보면 되니까.

    예전 _ensure_decks 는 ConveyorNode 가 데크에 표면속도를 써 놓기를 첫 물리
    스텝까지 기다렸다. 이제 노드 프림에서 데크와 방향을 직접 읽으므로 기다릴
    일이 없다 (그래서 "Stop 뒤 다시 못 잡는" 문제도 같이 없어진다).

    반환 원소: {"var": 속도변수 Attr, "deck": 데크 프림,
                "run": 굴릴 때 변수에 넣을 로컬 속도, "world": 로컬->월드 배율}
    """
    out = []
    for prim in stage.Traverse():
        a = prim.GetAttribute("node:type")
        if not a or a.Get() != CONVEYOR_NODE:
            continue
        rel = prim.GetRelationship("inputs:conveyorPrim")
        tgts = list(rel.GetTargets()) if rel else []
        deck = stage.GetPrimAtPath(tgts[0]) if tgts else None
        if deck is None or not deck.IsValid():
            print(f"[cycle] ※ {prim.GetPath()} 의 conveyorPrim 이 비었다 — 건너뛴다.")
            continue
        var = prim.GetParent().GetAttribute(VEL_VAR)
        if not var or var.Get() is None:
            print(f"[cycle] ※ {prim.GetParent().GetPath()} 에 {VEL_VAR} 가 없다 — "
                  f"이 컨베이어는 못 세운다.")
            continue

        d = prim.GetAttribute("inputs:direction")
        d = d.Get() if d else None
        d = Gf.Vec3d(d[0], d[1], d[2]) if d is not None else Gf.Vec3d(1, 0, 0)
        n = d.GetLength()
        d = d / n if n > 1e-9 else Gf.Vec3d(1, 0, 0)
        # 노드가 쓰는 표면속도는 데크 로컬이라 프림 스케일이 곱해진다.
        w = UsdGeom.Xformable(deck).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).TransformDir(d).GetLength()

        c = prim.GetAttribute("inputs:curved")
        curved = bool(c.Get()) if c and c.Get() is not None else False
        if curved or w < 1e-9:
            # 커브는 각속도라 환산이 다르다. 씬에 적힌 값을 그대로 쓴다.
            run = float(var.Get())
        else:
            run = BELT_MPS / w
        out.append({"var": var, "deck": deck, "run": run, "world": w,
                    "node": str(prim.GetPath())})
    return out


def _belt_top(stage):
    """벨트 데크 상면 z 를 잰다. 못 재면 None."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    tops = []
    for prim in stage.Traverse():
        if prim.GetName() not in ("Rollers", "Belt"):
            continue
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not r.IsEmpty():
            tops.append(r.GetMax()[2])
    return max(tops) if tops else None


def _set_running(on):
    """벨트 on/off. 그래프 변수 Velocity 가 레버다 (docstring 의 실측표 참고).

    표면속도를 직접 쓰지 않는다 — ConveyorNode 가 매 틱 도로 쓰기 때문에
    GUI 에서는 안 먹는다. 변수를 0 으로 두면 노드가 스스로 0 을 써 준다.
    """
    for d in _state.get("drives", []):
        d["var"].Set(0.0 if not on else float(d["run"]))
        if on:
            # 멈춰 있던 박스가 잠들어 있으면 속도만 되돌려선 안 깨어난다.
            # enabled 를 껐다가 다음 스텝에 켜서 접촉을 다시 만든다.
            e = d["deck"].GetAttribute(SURF_ON)
            if e:
                e.Set(False)
    if on:
        _state["wake"] = 1


def _restore_belt():
    """벨트를 굴러가는 상태로 되돌려 놓는다 (깨우기 절차 없이 즉시).

    Stop 할 때 반드시 부른다. 우리가 0 으로 둔 변수를 그대로 두면 다음 Play
    에서 벨트가 아예 안 돈다.
    """
    for d in _state.get("drives", []):
        d["var"].Set(float(d["run"]))
        e = d["deck"].GetAttribute(SURF_ON)
        if e:
            e.Set(True)


def _wake_step():
    """재기동 다음 스텝 — surfaceVelocityEnabled 를 도로 켠다."""
    if not _state.get("wake"):
        return
    _state["wake"] -= 1
    if _state["wake"] > 0:
        return
    for d in _state.get("drives", []):
        e = d["deck"].GetAttribute(SURF_ON)
        if e:
            e.Set(True)


def _boxes(stage):
    root = stage.GetPrimAtPath(BOX_ROOT)
    if not root or not root.IsValid():
        return []
    return [c for c in root.GetChildren() if c.IsValid()]


def _lead_box(stage):
    """벨트 위에 있는 박스 중 가장 앞선 것. (프림, 중심 x) 또는 (None, None).

    이미 집어 간 박스와 팔레트에 쌓인 박스는 빼야 한다. 벨트면 근처 높이에
    있고 픽 지점을 아직 안 지난 것만 센다.
    """
    best, best_x = None, None
    for prim in _boxes(stage):
        a = prim.GetAttribute("xformOp:translate")
        t = a.Get() if a else None
        if t is None:
            continue
        if not (BELT_TOP_Z - 0.10 <= t[2] <= BELT_TOP_Z + 0.60):
            continue        # 집혀서 떠 있거나 팔레트에 쌓인 것
        if t[0] > PICK_X + 0.30:
            continue        # 픽 지점을 지나쳐 버린 것 (사이클 실패분)
        if best_x is None or t[0] > best_x:
            best, best_x = prim, t[0]
    return best, best_x


# ─────────────────────────────────────────────────────────────
def _on_step(dt):
    stage = _state.get("stage")
    if stage is None:
        return
    _wake_step()
    phase = _state["phase"]
    _state["timer"] += dt

    if phase == "RUN":
        prim, x = _lead_box(stage)
        if prim is None:
            return
        if x >= PICK_X - STOP_LEAD:
            _set_running(False)
            _state.update(phase="SETTLE", timer=0.0,
                          box=str(prim.GetPath()), last_x=None)
        return

    if phase == "SETTLE":
        # 벨트를 세워도 박스가 관성으로 조금 더 간다. 시간으로 대충 기다리지
        # 말고 정말 멎을 때까지 본다 — 로봇에 넘길 좌표가 그 사이에 바뀌면
        # 픽 지점이 고정이 아니게 된다.
        prim = stage.GetPrimAtPath(_state["box"])
        t = prim.GetAttribute("xformOp:translate").Get() if prim else None
        if t is None:
            _state.update(phase="RUN", timer=0.0, box=None)
            _set_running(True)
            return

        # ★ 벨트가 안 서면 박스가 벨트 끝(x=10000)을 넘어 바닥으로 떨어진다.
        #   그 박스를 로봇에 넘기면 도달범위 밖 좌표로 IK 를 풀다 시간만 버린다
        #   (2026-08-22 실측: 픽 지점 +622mm, 바닥 z=155mm 인 박스를 잡으라고
        #   넘겨서 '이동' 웨이포인트가 10초 시간초과, 잔여 89.9도).
        #   놓친 건 놓친 대로 버리고 다음 박스를 받는다.
        if t[0] > PICK_X + RUNAWAY or t[2] < BELT_TOP_Z - 0.20:
            _state["lost"] = _state.get("lost", 0) + 1
            print(f"[cycle] ★ 놓쳤다 {_state['lost']}개 — "
                  f"{_state['box'].split('/')[-1]} "
                  f"x={t[0] * 1000:.0f}mm z={t[2] * 1000:.0f}mm "
                  f"(픽 지점 {PICK_X * 1000:.0f}mm). 벨트가 제때 안 섰다. "
                  f"이 박스는 버리고 다음 박스를 받는다.")
            _state.update(phase="RUN", timer=0.0, box=None)
            _set_running(True)
            return

        last = _state.get("last_x")
        _state["last_x"] = float(t[0])
        moving = last is None or abs(t[0] - last) > 2e-4      # 0.2mm/step
        if moving and _state["timer"] < SETTLE_MAX_S:
            return
        err = (t[0] - PICK_X) * 1000.0
        print(f"[cycle] 정지 — {_state['box'].split('/')[-1]} "
              f"x={t[0] * 1000:.0f}mm (픽 지점 대비 {err:+.0f}mm)")
        pose = (float(t[0]), float(t[1]), float(t[2]))
        if _robot["on_pick"] is not None:
            _robot["on_pick"](_state["box"], pose)
        else:
            _stub_begin(stage, prim, pose)
        _state.update(phase="PICK", timer=0.0)
        return

    if phase == "PICK":
        done = (_robot["is_idle"]() if _robot["is_idle"] is not None
                else _stub_step(stage, dt))
        if not done:
            return
        _state["placed"] += 1
        print(f"[cycle] 적재 완료 {_state['placed']}개 — 벨트 재기동")
        _set_running(True)
        _state.update(phase="RUN", timer=0.0, box=None)


def _on_timeline(event):
    if event.type == int(omni.timeline.TimelineEventType.STOP):
        # ★ 순서가 중요하다. 캐시를 비우기 **전에** 표면속도를 되돌려야 한다.
        _restore_belt()
        _state.update(phase="RUN", timer=0.0, box=None, placed=0, lost=0,
                      wake=0)
        _state.pop("stub", None)
        _state.pop("last_x", None)
        print("[cycle] Stop — 사이클 리셋, 벨트 원상복구")


# ─────────────────────────────────────────────────────────────
# 스텁 로봇 — 진짜 H2017 이 들어오기 전까지 사이클을 흉내 낸다.
#
# 관절을 푸는 게 아니라 박스를 키네마틱으로 바꿔 옮긴다. 목적은 "벨트가
# 언제 멈추고 언제 다시 도는지" 를 눈으로 확인하는 것 하나다.
# set_robot() 으로 갈아 끼우면 이 코드는 안 불린다.
def pallet_slot(stage, idx, dims):
    """팔레트 위 적재 자리. 단순 격자다 — 진짜 패킹은 packing.py 쪽 몫이다."""
    prim = stage.GetPrimAtPath(PALLET)
    if not prim or not prim.IsValid():
        return Gf.Vec3d(11.6, 0.0, BELT_TOP_Z)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    cols, rows = 2, 2
    per_layer = cols * rows
    layer, rem = divmod(idx, per_layer)
    cx, cy = divmod(rem, rows)
    w, d = (mx[0] - mn[0]) / cols, (mx[1] - mn[1]) / rows
    return Gf.Vec3d(mn[0] + w * (cx + 0.5),
                    mn[1] + d * (cy + 0.5),
                    mx[2] + dims[2] * (layer + 0.5))


def _box_dims(prim):
    """박스 로컬 치수 (w, d, h) [m].

<<<<<<< HEAD
    예전엔 xformOp:scale 만 읽었다. 스폰 박스가 Cube(size=1) + scale 이었기
    때문인데, 흡착이 스케일 걸린 프림을 못 다뤄서 scale 없는 Mesh 로 바뀌었다
    (spawn_feeder._make_box). 그래서 **extent 를 먼저 보고 scale 은 있으면 곱한다**
    — 두 방식 다 받는다. scale 만 읽으면 새 박스가 전부 (0.4, 0.3, 0.3) 로 잡혀
    팔레트 슬롯이 어긋난다.
=======
    예전엔 xformOp:scale 만 읽었다. 스폰 박스가 Cube(size=1) + scale 이었기 때문인데,
    치수를 지오메트리에 굽는 Mesh 로 바뀌면서 scale 이 없어졌다 (spawn_feeder._make_box).
    그래서 **extent 를 먼저 보고 scale 은 있으면 곱한다** — 두 방식 다 받는다.
    scale 만 읽으면 새 박스가 전부 (0.4, 0.3, 0.3) 로 잡혀 팔레트 슬롯이 어긋난다.
>>>>>>> origin/main
    """
    ext = UsdGeom.Boundable(prim).GetExtentAttr().Get()
    d = ([float(ext[1][i] - ext[0][i]) for i in range(3)] if ext
         else [0.4, 0.3, 0.3])
    a = prim.GetAttribute("xformOp:scale")
    s = a.Get() if a else None
    if s:
        d = [d[i] * float(s[i]) for i in range(3)]
    return (d[0], d[1], d[2])


def _stub_begin(stage, prim, pose):
    # 물리를 끄고(키네마틱) 손으로 옮긴다. 안 그러면 중력에 그냥 떨어진다.
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    dims = _box_dims(prim)
    t1 = pallet_slot(stage, _state["placed"], dims)
    # 이송 높이는 출발지와 목적지 **둘 다** 위여야 한다. 팔레트가 트럭 바닥
    # 높이로 올라가면서 적재 상단이 벨트면보다 훨씬 높아졌다 — 벨트 기준으로
    # 고정하면 높은 층에 놓을 때 박스를 아래로 끌고 가 팔레트를 들이받는다.
    lift = max(pose[2], float(t1[2])) + 0.30
    _state["stub"] = {
        "path": str(prim.GetPath()),
        "t0": Gf.Vec3d(*pose),
        "t1": t1,
        "lift": lift,
        "t": 0.0,
    }


def _stub_step(stage, dt):
    """0 -> CYCLE_S 동안 집기(25%) · 이송(50%) · 놓기(25%) 를 흉내 낸다."""
    st = _state.get("stub")
    if st is None:
        return True
    st["t"] += dt
    u = min(1.0, st["t"] / CYCLE_S)
    prim = stage.GetPrimAtPath(st["path"])
    if prim and prim.IsValid():
        a, b, lift = st["t0"], st["t1"], st["lift"]
        if u < 0.25:                       # 수직으로 든다
            k = u / 0.25
            p = Gf.Vec3d(a[0], a[1], a[2] + (lift - a[2]) * k)
        elif u < 0.75:                     # 팔레트 위로 옮긴다
            k = (u - 0.25) / 0.5
            p = Gf.Vec3d(a[0] + (b[0] - a[0]) * k,
                         a[1] + (b[1] - a[1]) * k, lift)
        else:                              # 내려놓는다
            k = (u - 0.75) / 0.25
            p = Gf.Vec3d(b[0], b[1], lift + (b[2] - lift) * k)
        prim.GetAttribute("xformOp:translate").Set(p)
    if u >= 1.0:
        _state.pop("stub", None)
        return True
    return False
