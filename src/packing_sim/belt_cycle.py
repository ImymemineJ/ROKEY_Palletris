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

벨트를 세우는 레버 (2026-08-22 실측으로 정정)
    예전에는 `graph:variable:Velocity` 를 0 으로 썼다. **실행 중에는 안 듣는다.**
    OmniGraph 변수의 런타임 값은 그래프 인스턴스에 있어서 USD 어트리뷰트를
    고쳐도, og 변수 API 로 set 해도 ConveyorNode 가 다시 안 읽는다. 실측:

        graph:variable:Velocity = 0 적용 후에도 박스가 1.26 m/s 로 계속 갔다.

    그래서 박스가 픽 지점을 지나 벨트 끝(x=10000)에서 바닥으로 떨어졌다.
    지금은 ConveyorNode 가 데크에 써 놓는 **physxSurfaceVelocity:surfaceVelocity
    를 직접 0 으로** 만든다 (실측: 1초 안에 선다). 노드가 매 틱 다시 쓰지
    않으므로 우리가 쓴 값이 유지된다.

    다시 돌릴 때는 속도를 되돌리는 것만으로는 안 된다 — 멈춰 있던 박스가
    잠들어서(sleep) 표면속도를 다시 안 받는다. surfaceVelocityEnabled 를
    False -> True 로 토글해 깨운다.

벨트 속도
    surfaceVelocity 는 **데크 로컬** 값이라 프림 스케일이 곱해진다. 씬에 적힌
    0.5 가 월드에서는 1.26 m/s 였다 (스케일 2.52). 사양 belt_velocity_mps 가
    월드에서 그대로 나오도록 start() 에서 환산해 다시 쓴다. STOP_LEAD 40mm 는
    0.5 m/s 에서 잰 값이라, 속도를 안 맞추면 정지 선행이 통째로 틀린다.

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

# ConveyorNode 가 데크 프림에 써 놓는 PhysX 표면속도. 이게 진짜 레버다.
SURF_VEL = "physxSurfaceVelocity:surfaceVelocity"
SURF_ON = "physxSurfaceVelocity:surfaceVelocityEnabled"

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

    if not _find_velocity_attrs(stage):
        raise RuntimeError("[cycle] graph:variable:Velocity 를 못 찾았다. "
                           "컨베이어 그래프가 있는 씬인지 확인해라.")

    global BELT_TOP_Z
    top = _belt_top(stage)
    if top is not None:
        BELT_TOP_Z = top

    # 데크의 표면속도는 ConveyorNode 가 **처음 틱을 돌 때** 쓴다. Play 전에는
    # 어트리뷰트 자체가 없어서 여기서 잡을 수 없다 — 첫 물리 스텝에 잡는다.
    _state.update(
        stage=stage, decks=None, run_sv={}, wake=0,
        deck_try=0, deck_warned=False,
        phase="RUN", timer=0.0, box=None, placed=0,
    )

    _state["physics_sub"] = get_physx_interface().subscribe_physics_step_events(_on_step)
    _state["timeline_sub"] = (
        omni.timeline.get_timeline_interface()
        .get_timeline_event_stream()
        .create_subscription_to_pop(_on_timeline)
    )

    mode = "실제 로봇" if _robot["on_pick"] else f"스텁 ({CYCLE_S:.1f}초/사이클)"
    print(f"[cycle] 픽 지점 x={PICK_X * 1000:.0f}mm  "
          f"정지 선행 {STOP_LEAD * 1000:.0f}mm  벨트면 {BELT_TOP_Z * 1000:.0f}mm  "
          f"벨트 {BELT_MPS:.2f} m/s")
    print(f"[cycle] 로봇: {mode}. Play 하면 박스가 픽 지점에 서고 벨트가 멈춘다.")


def stop():
    if _state.get("physics_sub") is not None:
        _state["physics_sub"] = None
    if _state.get("timeline_sub") is not None:
        _state["timeline_sub"] = None
    if _state.get("decks"):
        _restore_belt()         # 씬을 굴러가는 상태로 되돌려 놓고 나간다
    _state.clear()


def status():
    return {k: _state.get(k) for k in ("phase", "box", "placed", "timer")}


# ─────────────────────────────────────────────────────────────
def _find_velocity_attrs(stage):
    out = []
    for prim in stage.Traverse():
        a = prim.GetAttribute("graph:variable:Velocity")
        if a and a.Get() is not None:
            out.append(a)
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


def _ensure_decks(stage):
    """표면속도가 걸린 데크를 잡고, 사양 속도가 나오도록 다시 쓴다.

    Play 전에는 ConveyorNode 가 아직 아무것도 안 써서 못 잡는다. 첫 물리
    스텝에 한 번 잡으면 그 뒤로는 우리가 들고 있는다.
    """
    if _state.get("decks") is not None:
        return bool(_state["decks"])

    decks, run_sv = [], {}
    for prim in stage.Traverse():
        a = prim.GetAttribute(SURF_VEL)
        v = a.Get() if a else None
        if v is None or Gf.Vec3d(v[0], v[1], v[2]).GetLength() < 1e-9:
            continue
        local = Gf.Vec3d(v[0], v[1], v[2])
        # surfaceVelocity 는 데크 로컬이다. 월드 속도로 환산해 사양에 맞춘다.
        world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).TransformDir(local)
        w = world.GetLength()
        if w < 1e-9:
            continue
        run = local * (BELT_MPS / w)
        decks.append(prim)
        run_sv[str(prim.GetPath())] = Gf.Vec3f(run[0], run[1], run[2])
        print(f"[cycle] 데크 {prim.GetPath()}  "
              f"로컬 {local[0]:+.3f} -> 월드 {w:.3f} m/s  "
              f"=> 사양 {BELT_MPS:.3f} m/s 로 재설정")

    if not decks:
        # ★ 여기서 빈 결과를 캐시하면 안 된다. Play 직후 몇 스텝은 ConveyorNode 가
        #   아직 표면속도를 안 써서 못 찾는 게 정상이다. 캐시해 버리면 Stop 후
        #   다시 Play 했을 때 영영 못 잡는다 (2026-08-22 Stop->Play 시험에서 발견).
        _state["deck_try"] = _state.get("deck_try", 0) + 1
        if _state["deck_try"] == 180 and not _state.get("deck_warned"):
            _state["deck_warned"] = True
            print("[cycle] ★ 3초가 지나도 표면속도가 걸린 데크를 못 찾았다 — "
                  "벨트를 못 세운다. 컨베이어 그래프가 도는지 확인해라.")
        return False

    _state["decks"] = decks
    _state["run_sv"] = run_sv
    _state["deck_try"] = 0
    _set_running(True)
    return True


def _set_running(on):
    """벨트 on/off. 데크의 PhysX 표면속도를 직접 건드린다 (docstring 참고)."""
    for prim in _state.get("decks", []):
        a = prim.GetAttribute(SURF_VEL)
        if not a:
            continue
        a.Set(_state["run_sv"][str(prim.GetPath())] if on else Gf.Vec3f(0, 0, 0))
        if on:
            # 멈춰 있던 박스가 잠들어 있으면 속도만 되돌려선 안 깨어난다.
            # enabled 를 껐다가 다음 스텝에 켜서 접촉을 다시 만든다.
            e = prim.GetAttribute(SURF_ON)
            if e:
                e.Set(False)
    if on:
        _state["wake"] = 1


def _restore_belt():
    """벨트를 굴러가는 상태로 되돌려 놓는다 (깨우기 절차 없이 즉시).

    Stop 할 때 반드시 부른다. 우리가 써 놓은 (0,0,0) 을 그대로 두면 다음
    Play 에서 _ensure_decks 가 "표면속도가 0인 프림"으로 보고 데크를 아예
    못 찾는다 — ConveyorNode 는 Stop 뒤에 값을 다시 안 쓴다 (실측).
    """
    for prim in _state.get("decks", []):
        a = prim.GetAttribute(SURF_VEL)
        if a:
            a.Set(_state["run_sv"][str(prim.GetPath())])
        e = prim.GetAttribute(SURF_ON)
        if e:
            e.Set(True)


def _wake_step():
    """재기동 다음 스텝 — surfaceVelocityEnabled 를 도로 켠다."""
    if not _state.get("wake"):
        return
    _state["wake"] -= 1
    if _state["wake"] > 0:
        return
    for prim in _state.get("decks", []):
        e = prim.GetAttribute(SURF_ON)
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
    if not _ensure_decks(stage):
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
        _state.update(phase="RUN", timer=0.0, box=None, placed=0,
                      decks=None, run_sv={}, wake=0,
                      deck_try=0, deck_warned=False)
        _state.pop("stub", None)
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

    예전엔 xformOp:scale 만 읽었다. 스폰 박스가 Cube(size=1) + scale 이었기
    때문인데, 흡착이 스케일 걸린 프림을 못 다뤄서 scale 없는 Mesh 로 바뀌었다
    (spawn_feeder._make_box). 그래서 **extent 를 먼저 보고 scale 은 있으면 곱한다**
    — 두 방식 다 받는다. scale 만 읽으면 새 박스가 전부 (0.4, 0.3, 0.3) 로 잡혀
    팔레트 슬롯이 어긋난다.
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
