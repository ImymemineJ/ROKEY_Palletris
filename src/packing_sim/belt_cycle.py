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
BELT_MPS = float(os.environ.get("PL_BELT_MPS", "0.5"))
SETTLE_MAX_S = 1.5      # 이 시간이 지나면 안 멎어도 진행한다 (교착 방지)

BOX_ROOT = "/World/Boxes"
PALLET = "/World/Pallet"

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

    vels = _find_velocity_attrs(stage)
    if not vels:
        raise RuntimeError("[cycle] graph:variable:Velocity 를 못 찾았다. "
                           "컨베이어 그래프가 있는 씬인지 확인해라.")
    # 씬에 적힌 값이 사양 속도를 내는 로컬 값이다. 재기동할 때 이 값으로 돌린다.
    run_vals = {a: a.Get() for a in vels}

    global BELT_TOP_Z
    top = _belt_top(stage)
    if top is not None:
        BELT_TOP_Z = top

    _state.update(
        stage=stage, vels=vels, run_vals=run_vals,
        phase="RUN", timer=0.0, box=None, placed=0,
    )
    _set_running(True)

    _state["physics_sub"] = get_physx_interface().subscribe_physics_step_events(_on_step)
    _state["timeline_sub"] = (
        omni.timeline.get_timeline_interface()
        .get_timeline_event_stream()
        .create_subscription_to_pop(_on_timeline)
    )

    mode = "실제 로봇" if _robot["on_pick"] else f"스텁 ({CYCLE_S:.1f}초/사이클)"
    print(f"[cycle] 픽 지점 x={PICK_X * 1000:.0f}mm  "
          f"정지 선행 {STOP_LEAD * 1000:.0f}mm  벨트면 {BELT_TOP_Z * 1000:.0f}mm")
    print(f"[cycle] 로봇: {mode}. Play 하면 박스가 픽 지점에 서고 벨트가 멈춘다.")


def stop():
    if _state.get("physics_sub") is not None:
        _state["physics_sub"] = None
    if _state.get("timeline_sub") is not None:
        _state["timeline_sub"] = None
    if _state.get("vels"):
        _set_running(True)      # 씬을 굴러가는 상태로 되돌려 놓고 나간다
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


def _set_running(on):
    for a in _state.get("vels", []):
        a.Set(_state["run_vals"][a] if on else 0.0)


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
        _state.update(phase="RUN", timer=0.0, box=None, placed=0)
        _state.pop("stub", None)
        _set_running(True)
        print("[cycle] Stop — 사이클 리셋, 벨트 재기동")


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
    a = prim.GetAttribute("xformOp:scale")
    s = a.Get() if a else None
    return (float(s[0]), float(s[1]), float(s[2])) if s else (0.4, 0.3, 0.3)


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
