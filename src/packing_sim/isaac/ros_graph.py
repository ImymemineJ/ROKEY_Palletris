"""Isaac 안 ROS 2 통로 — Action Graph 하나로 끝낸다. **소켓 없음** (CLAUDE.md §2).

이 파일이 대체하는 것
    `sim_server.py` 의 JSON 소켓(localhost:8765). §2 가 폐기했다.
    Isaac 안팎을 잇는 통로는 ROS 2 하나뿐이다 — Action Graph 의 C/rcl 백엔드가
    Python 버전 차이(3.11 vs 3.12)를 우회하므로 rclpy 가 필요 없다.

세우는 것 (`/World/PalletrisROS`)
    OnPlaybackTick ─┬─> ROS2Subscriber  /zone_<z>/robot/cmd   (packing_msgs/RobotCmd)
                    ├─> [서비스 4종]     Req -> Script -> Res
                    └─> EventGate ─────> ROS2Publisher /zone_<z>/robot/event (RobotEvent)

    `/clock` · `/tf` · 카메라는 **여기서 안 만든다.** 김학민(A)의 기존 그래프 소관이다.
    `/zone_*/box_at_pickup` 도 아니다 — §3-2 에서 **인식 노드(D)** 가 발행한다.

★★ 실측으로 알아낸 제약 4가지 (TS-022). 순서를 지키지 않으면 조용히 안 된다.

  1. 서비스 배선은 `outputs:execOut` 이 아니라 **`outputs:onReceived`** 다.
  2. **메시지 필드는 Play 후에야 생긴다.** 노드가 타입을 푼 다음에 동적 속성이
     만들어진다. 그래서 `build()` → (Play + 몇 프레임) → `wire()` 두 단계다.
  3. **바깥 파이썬 폴링으로는 서비스 응답을 못 채운다.** 요청 도착과 응답 송신이
     같은 틱에 끝나서 항상 한 프레임 늦는다. 그래서 핸들러가 그래프 **안**
     (`ScriptNode`)으로 들어간다. Isaac 메인 스레드라 **블로킹 금지** (§9).
  4. `Res` 의 `Response:*` 는 첫 요청이 와야 생긴다. `wire()` 에서 .srv 를 읽어
     **미리 만들어 둔다** — 안 그러면 첫 호출만 기본값이 나간다. (실측: 미리 만들면
     1회차부터 `ok=true`, 안 만들면 1회차만 `ok=false`)
  5. 요청 값은 반드시 `db.inputs.<이름>` 으로 읽는다. `og.Controller.attribute().get()`
     은 **연결을 안 따라가** 로컬 기본값(빈 문자열/0)을 준다.

⚠️ 알려진 결함 — **서비스 응답의 float 이 안 실린다** (`SetVelocity.actual_mps`).
   브리지가 `Unable to write float value (nil)` 을 뱉고 0.0 이 나간다.
   string·bool 은 정상이고, **토픽의 float32 는 정상**이다(RobotCmd.yaw_deg 3.5 수신 확인).
   즉 서비스 응답 경로 한정이다. 우리 계약에서 걸리는 필드는 이 하나뿐이라
   관제탑은 `ok` 만 보고 판단한다. §15 미결.

쓰는 법 (sim_main)
    import ros_graph
    ros_graph.set_handlers(set_velocity=..., despawn_box=..., ...)
    ros_graph.build(zone="B")
    world.play()                       # ← Play 가 먼저
    for _ in range(60): world.step()
    ros_graph.wire()                   # ← 그다음 데이터 배선
    ...
    ros_graph.send_event(seq=7, ok=True, ...)     # 매 틱 EventGate 가 비워 준다

혼자 확인 (씬 없이 빈 스테이지로)
    ~/isaacsim/python.sh scripts/selftest_ros_graph.py
"""
import os

import omni.graph.core as og

GRAPH = "/World/PalletrisROS"
PKG = "packing_msgs"

# ── 서비스 4종. (핸들러 키, srv 이름, 서비스 경로) ─────────────────────────
#   §3-3 이 srv 타입만 정하고 **경로를 안 적었다.** 아래는 잠정값이고 §15 미결이다.
SERVICES = [
    ("set_velocity",      "SetVelocity",     "/conveyor/set_velocity"),
    ("despawn_box",       "DespawnBox",      "/sim/despawn_box"),
    ("set_spawn_enabled", "SetSpawnEnabled", "/sim/set_spawn_enabled"),
    ("swap_pallet",       "SwapPallet",      "/sim/swap_pallet"),
]
# ★ 로봇 토픽은 **구역별**이다 (2026-08-26 정합).
#   CLAUDE.md §3-2 는 `/robot/cmd` 단수로 적혀 있지만 로봇이 3 대라 관제탑이
#   `/zone_{a|b|c}/robot/cmd` 로 나눠 쓴다 (`orchestrator_node.py:161-171`).
#   여기가 단수로 남아 있어서 **관제탑과 Isaac 이 서로 못 만났다** — 에러도 안 난다.
#   구역 문자는 관제탑과 같게 **소문자**로 맞춘다 (`self.zone = str(zone).lower()`).
CMD_TOPIC_FMT = "/zone_%s/robot/cmd"
EVT_TOPIC_FMT = "/zone_%s/robot/event"


def cmd_topic(zone):
    return CMD_TOPIC_FMT % str(zone).lower()


def evt_topic(zone):
    return EVT_TOPIC_FMT % str(zone).lower()

# ROS 원시형 -> OmniGraph 타입. TS-022 에서 string->token, float32->float 실측 확인.
_OG_TYPE = {
    "bool": og.BaseDataType.BOOL,
    "int8": og.BaseDataType.INT, "uint8": og.BaseDataType.UCHAR,
    "int16": og.BaseDataType.INT, "uint16": og.BaseDataType.UINT,
    "int32": og.BaseDataType.INT, "uint32": og.BaseDataType.UINT,
    "int64": og.BaseDataType.INT64, "uint64": og.BaseDataType.UINT64,
    "float32": og.BaseDataType.FLOAT, "float64": og.BaseDataType.DOUBLE,
    "string": og.BaseDataType.TOKEN,
}

# 핸들러 등록소. ScriptNode 가 여기를 통해 sim_main 의 함수를 부른다.
#   시그니처: handler(**요청필드) -> dict(응답필드)  ·  즉시 반환할 것 (§9)
HANDLERS = {}
# ★★ 구역별로 갈린다 (2026-08-26 김병직). 로봇이 3 대라 하나로 두면 안 된다:
#   · `_EVENTS` 가 하나면 A 구역 결과가 B 구역 토픽으로 나간다
#   · `seen_seq` 가 하나면 A 구역이 집어간 seq 를 B 구역이 "이미 봤다"고 버린다
#   둘 다 **에러 없이 엉뚱하게 도는** 종류다.
# 보낼 RobotEvent 대기열 {zone: [ev, ...]}. send_event() 가 넣고 EventGate 가 비운다.
_EVENTS = {}
_state = {"wired": False, "zones": [], "seen_seq": {}}
# 디버그 출력 (환경변수 PL_ROSGRAPH_DEBUG=1)
DEBUG = os.environ.get("PL_ROSGRAPH_DEBUG") == "1"


def set_handlers(**fns):
    """sim_main 이 실제 동작(spawner·conveyor_ctl·robot_motion)을 꽂는다."""
    HANDLERS.update(fns)


def _one_zone(zone):
    """zone 을 안 주면 구역이 하나일 때만 그걸 쓴다. 여럿이면 안 넘어간다.

    ★ 구역이 여럿인데 zone 을 생략하면 **조용히 엉뚱한 구역에 실린다.**
      기존 호출부(selftest_ros_graph.py)는 구역이 하나라 그대로 돈다.
    """
    if zone is not None:
        return str(zone).upper()
    zs = _state["zones"]
    if len(zs) == 1:
        return zs[0]
    raise ValueError("구역이 %d 개다 — zone 을 명시할 것 (%s)" % (len(zs), zs))


def send_event(zone=None, **fields):
    """RobotEvent 를 한 건 예약한다. 실제 발행은 다음 틱의 EventGate 가 한다.

    ★ 여기서 바로 발행하지 않는 이유: 발행은 그래프 평가 안에서 일어나야 한다.
      바깥에서 Publisher 입력만 써 두면 언제 나갈지가 틱 순서에 좌우된다.
    """
    _EVENTS.setdefault(_one_zone(zone), []).append(dict(fields))


def take_cmd(zone=None):
    """그 구역에 새로 들어온 RobotCmd 를 dict 로 준다. 없으면 None.

    ★ 같은 메시지를 두 번 집지 않도록 `seq` 로 거른다 (§3-4 규약 2번).
      Subscriber 의 `outputs:*` 는 마지막 수신값을 계속 들고 있어서, 안 거르면
      매 틱 같은 명령을 새 명령으로 오인한다.
      **seq 기억은 구역별이다** — 하나로 두면 A 가 본 seq 를 B 가 버린다.
    """
    if not _state["wired"]:
        return None
    z = _one_zone(zone)
    node = og.Controller.node(f"{GRAPH}/CmdSub_{z}")
    out = {}
    for a in node.get_attributes():
        n = a.get_name()
        if n.startswith("outputs:") and n != "outputs:execOut":
            try:
                out[n.split(":", 1)[1]] = og.Controller.attribute(n, node).get()
            except Exception:                                   # noqa: BLE001
                pass
    seq = out.get("seq")
    if seq is None or seq == _state["seen_seq"].get(z) or seq == 0:
        return None
    _state["seen_seq"][z] = seq
    return out


# ── 인터페이스 정의 읽기 ────────────────────────────────────────────────────
def _iface_path(subfolder, name, ext):
    """설치된 packing_msgs 의 .msg/.srv 실제 경로. AMENT_PREFIX_PATH 를 뒤진다.

    ament_index_python 을 못 쓴다 — Jazzy 쪽은 python3.12 용이라 Isaac(3.11)에서
    import 가 안 된다. 환경변수를 직접 훑는 게 확실하다.
    """
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        p = os.path.join(prefix, "share", PKG, subfolder, f"{name}.{ext}")
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(
        f"{PKG}/{subfolder}/{name}.{ext} 를 못 찾았다 — "
        "source install/setup.bash 를 하고 Isaac 을 띄웠는지 확인할 것")


def _parse_fields(text):
    """.msg/.srv 한 덩어리 -> [(필드명, ros타입)]. 상수(`=`)와 주석은 버린다."""
    out = []
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or "=" in line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            out.append((parts[1], parts[0]))
    return out


def srv_fields(name):
    """(요청필드, 응답필드). `---` 로 가른다."""
    with open(_iface_path("srv", name, "srv")) as f:
        a, _, b = f.read().partition("---")
    return _parse_fields(a), _parse_fields(b)


# ── 1단계: 뼈대. Play 전에 부른다 ───────────────────────────────────────────
def build(zone="B", zones=None):
    """노드를 만들고 **정적 배선만** 한다. 데이터 배선은 wire() 에서.

    Args:
        zone:  구역 하나 (기존 호출부 호환)
        zones: 구역 여럿 — `["A","B","C"]`. 주면 이쪽이 이긴다.

    ★★ **그래프는 하나, 서비스는 한 벌, 로봇 노드만 구역별**이다.
      구역마다 그래프를 따로 만들면 `/conveyor/set_velocity` 같은 **전역 서비스에
      서버가 3 개** 붙는다 (srv 에 구역 칸이 없다) — 어느 것이 답할지 정해지지 않는다.
      로봇 토픽만 구역이 갈리므로 CmdSub/EvtGate/EvtPub 만 구역 수만큼 만든다.
    """
    K, c = og.Controller.Keys, og.Controller
    zs = [str(z).upper() for z in (zones or [zone])]
    seen, zlist = set(), []
    for z in zs:                       # 중복 제거 — 같은 구역에 노드 둘이면 겹친다
        if z not in seen:
            seen.add(z)
            zlist.append(z)
    _state["zones"] = zlist
    for z in zlist:
        _EVENTS.setdefault(z, [])
        _state["seen_seq"].setdefault(z, None)

    nodes = [("Tick", "omni.graph.action.OnPlaybackTick"),
             ("Ctx", "isaacsim.ros2.bridge.ROS2Context")]
    values = [
        # ROS_DOMAIN_ID 를 씬에 박지 않는다 — 팀이 값을 바꾸면 씬도 고쳐야 한다
        ("Ctx.inputs:useDomainIDEnvVar", True),
    ]
    conn = []

    # ── 로봇 토픽: 구독·발행은 구역별, **게이트는 하나** ──
    #
    # ★★ 게이트를 구역마다 두면 안 된다 (2026-08-26 김병직 실측).
    #   `Tick.outputs:tick` 에 `ScriptNode` 를 여럿 매달면 **첫 틱에만 다 돌고
    #   그 뒤로는 하나만 남는다.** 어느 것이 살아남는지도 실행마다 다르다:
    #       실행 1: A 만 계속  ·  실행 2: B 만 계속
    #       (gate setup 은 A·B·C 셋 다 뜬다 — 노드는 만들어졌는데 평가만 멎는다)
    #   그래서 B·C 의 RobotEvent 가 영영 안 나갔다. 어댑터는 ok 로 답했는데도라
    #   **관제탑 쪽에서는 20 초 타임아웃으로만 보인다.**
    #   1 구역일 때 안 보였던 이유: 그때 Tick 에 달린 ScriptNode 가 하나였다
    #   (서비스 4 개는 Tick 이 아니라 `_Req.outputs:onReceived` 뒤에 달린다).
    #   -> 게이트 하나가 구역 전부를 훑고 각 EvtPub 입력에 써 넣는다.
    nodes += [("EvtGate", "omni.graph.scriptnode.ScriptNode")]
    values += [("EvtGate.inputs:usePath", False),
               ("EvtGate.inputs:script", _event_gate_script(zlist))]
    conn += [("Tick.outputs:tick", "EvtGate.inputs:execIn")]

    for z in zlist:
        sub, pub = f"CmdSub_{z}", f"EvtPub_{z}"
        nodes += [(sub, "isaacsim.ros2.bridge.ROS2Subscriber"),
                  (pub, "isaacsim.ros2.bridge.ROS2Publisher")]
        values += [
            (f"{sub}.inputs:topicName", cmd_topic(z)),
            (f"{sub}.inputs:messagePackage", PKG),
            (f"{sub}.inputs:messageSubfolder", "msg"),
            (f"{sub}.inputs:messageName", "RobotCmd"),
            (f"{pub}.inputs:topicName", evt_topic(z)),
            (f"{pub}.inputs:messagePackage", PKG),
            (f"{pub}.inputs:messageSubfolder", "msg"),
            (f"{pub}.inputs:messageName", "RobotEvent"),
        ]
        conn += [("Tick.outputs:tick", f"{sub}.inputs:execIn"),
                 ("Ctx.outputs:context", f"{sub}.inputs:context"),
                 ("Ctx.outputs:context", f"{pub}.inputs:context"),
                 # ★ EvtPub 은 Tick 이 아니라 게이트가 친다.
                 ("EvtGate.outputs:execOut", f"{pub}.inputs:execIn")]

    for key, srv, path in SERVICES:
        r, s, g = f"{key}_Req", f"{key}_Res", f"{key}_Fn"
        nodes += [(r, "isaacsim.ros2.bridge.OgnROS2ServiceServerRequest"),
                  (g, "omni.graph.scriptnode.ScriptNode"),
                  (s, "isaacsim.ros2.bridge.OgnROS2ServiceServerResponse")]
        values += [(f"{r}.inputs:serviceName", path),
                   (f"{r}.inputs:messagePackage", PKG),
                   (f"{r}.inputs:messageSubfolder", "srv"),
                   (f"{r}.inputs:messageName", srv),
                   (f"{s}.inputs:messagePackage", PKG),
                   (f"{s}.inputs:messageSubfolder", "srv"),
                   (f"{s}.inputs:messageName", srv),
                   (f"{g}.inputs:usePath", False),
                   (f"{g}.inputs:script", _service_script(key, srv))]
        conn += [("Tick.outputs:tick", f"{r}.inputs:execIn"),
                 # ★ execOut 이 아니라 onReceived (TS-022 1번)
                 (f"{r}.outputs:onReceived", f"{g}.inputs:execIn"),
                 (f"{g}.outputs:execOut", f"{s}.inputs:onReceived"),
                 (f"{r}.outputs:serverHandle", f"{s}.inputs:serverHandle")]

    og.Controller.edit({"graph_path": GRAPH, "evaluator_name": "execution"},
                       {K.CREATE_NODES: nodes, K.SET_VALUES: values, K.CONNECT: conn})
    return GRAPH


# ── 2단계: 데이터 배선. **Play 하고 몇 프레임 돈 뒤에** 부른다 ───────────────
def wire():
    """동적 필드(요청 인자)를 ScriptNode 에 잇고, 응답 속성을 미리 만든다.

    ★ 이걸 build() 안에서 하면 안 된다 — 그 시점엔 `outputs:Request:*` 가 아직
      없어서 `Failed to connect` 로 죽는다 (TS-022 2번).
    """
    c = og.Controller
    for key, srv, _path in SERVICES:
        req_fields, res_fields = srv_fields(srv)
        rn, gn, sn = (c.node(f"{GRAPH}/{key}_Req"), c.node(f"{GRAPH}/{key}_Fn"),
                      c.node(f"{GRAPH}/{key}_Res"))
        pairs = []
        for fname, ftype in req_fields:
            t = _OG_TYPE.get(ftype)
            if t is None:                     # 배열·중첩 타입은 아직 안 쓴다
                continue
            c.create_attribute(gn, fname, og.Type(t), og.AttributePortType.INPUT)
            pairs.append((f"{GRAPH}/{key}_Req.outputs:Request:{fname}",
                          f"{GRAPH}/{key}_Fn.inputs:{fname}"))
        # ★ 응답 속성을 여기서 미리 만든다. 안 만들면 **첫 호출만** 기본값이
        #   나간다 (TS-022 4번 — Res 의 Response:* 는 첫 요청 때 생긴다).
        for fname, ftype in res_fields:
            t = _OG_TYPE.get(ftype)
            if t is None or os.environ.get("PL_NO_PRECREATE") == "1":
                continue
            try:
                c.create_attribute(sn, f"Response:{fname}", og.Type(t),
                                   og.AttributePortType.INPUT)
            except Exception:                                   # noqa: BLE001
                pass                          # 이미 있으면 그대로 쓴다
        if pairs:
            og.Controller.edit(GRAPH, {og.Controller.Keys.CONNECT: pairs})
    _state["wired"] = True


# ── ScriptNode 스크립트 (그래프 안에서 도는 파이썬) ─────────────────────────
def _service_script(key, srv):
    """요청 필드를 읽어 핸들러를 부르고 응답을 써 넣는 스크립트.

    ★ 필드 이름을 **생성 시점에 .srv 에서 읽어 박는다.** 런타임에 속성을 훑으면
      ScriptNode 자체 입력(`script`·`usePath`·`scriptPath`)까지 섞여 들어온다.
    ★ 값은 반드시 `db.inputs.<이름>` 으로 읽는다. `og.Controller.attribute(...).get()`
      은 **연결을 안 따라가** 로컬 기본값(빈 문자열)을 준다 — 실측으로 확인했다.
    """
    req_fields, _ = srv_fields(srv)
    names = [f for f, t in req_fields if t in _OG_TYPE]
    return f'''
import importlib
import omni.graph.core as _og

_M = importlib.import_module("{__name__}")
_FIELDS = {names!r}

def setup(db): pass

def compute(db):
    # ★ Isaac 메인 스레드다. 여기서 기다리면 물리가 안 돌아 판정이 영영 안 난다 (§9).
    #   핸들러는 **즉시 반환**해야 한다 — 오래 걸리는 일은 큐에 넣고 돌아온다.
    req = {{}}
    for f in _FIELDS:
        try:
            v = getattr(db.inputs, f)
        except Exception:
            continue
        req[f] = str(v) if isinstance(v, (bytes, bytearray)) else v

    fn = _M.HANDLERS.get("{key}")
    if fn is None:
        print("[ros_graph] {key} 핸들러가 등록 안 됨")
        res = {{"ok": False}}
    else:
        try:
            res = fn(**req) or {{}}
        except Exception as e:
            print(f"[ros_graph] {key} 핸들러 예외: {{e!r}} req={{req}}")
            res = {{"ok": False}}

    node = _og.Controller.node(_M.GRAPH + "/{key}_Res")
    if _M.DEBUG:
        print("[ros_graph:dbg] Res 속성 =", [a.get_name() for a in node.get_attributes()])
    for k, v in res.items():
        try:
            at = _og.Controller.attribute("inputs:Response:" + k, node)
            at.set(v)
            if _M.DEBUG:
                print(f"[ros_graph:dbg] {{k}} set({{v!r}}) -> readback {{at.get()!r}} "
                      f"type={{at.get_resolved_type()}}")
        except Exception as e:
            print(f"[ros_graph] {key} 응답 '{{k}}' 쓰기 실패: {{e!r}}")
    db.outputs.execOut = _og.ExecutionAttributeState.ENABLED
    return True
'''


def _event_gate_script(zones):
    """구역 전부의 대기열을 훑어 각 Publisher 입력을 채우고 execOut 을 친다.

    ★★ **게이트는 하나다** (2026-08-26). `Tick` 에 ScriptNode 를 여럿 달면 첫 틱
      뒤로 하나만 살아남는다 — `build()` 주석에 실측을 적어 뒀다.

    ★ execOut 은 하나뿐이라 켜면 **구역 Publisher 셋이 다 발행한다.** 새 이벤트가
      없는 구역은 직전 값을 다시 쏘는데, 관제탑이 `seq` 로 거르므로(§3-4 규약 2)
      문제가 되지 않는다 — 애초에 EvtPub 이 매 틱 다시 쏘고 있어서 달라지는 것도 없다.
    """
    zl = ", ".join('"%s"' % str(z).upper() for z in zones)
    return f'''
import importlib
import omni.graph.core as _og

_M = importlib.import_module("{__name__}")
_ZONES = [{zl}]

def setup(db):
    if _M.DEBUG:
        print("[ros_graph] gate setup — 구역 %s" % _ZONES)

def compute(db):
    _any = False
    for _z in _ZONES:
        q = _M._EVENTS.get(_z) or []
        if not q:
            continue
        ev = q.pop(0)          # 구역당 한 틱에 하나. 뒤집히면 seq 가 꼬인다
        if _M.DEBUG:
            print("[ros_graph] gate %s pop seq=%s (큐 %d 남음)"
                  % (_z, ev.get("seq"), len(q)))
        node = _og.Controller.node(_M.GRAPH + "/EvtPub_" + _z)
        for k, v in ev.items():
            try:
                _og.Controller.attribute("inputs:" + k, node).set(v)
            except Exception:
                print("[ros_graph] %s RobotEvent 필드 %r 를 못 씀 — 계약 확인" % (_z, k))
        _any = True
    # ⚠ 새 이벤트가 없어도 EvtPub 은 어차피 매 틱 마지막 값을 다시 쏜다 (미해결).
    #   관제탑은 seq 로 거른다. 여기서 DISABLED 를 걸어도 안 막혔다 — 실측.
    #   ★ False 를 돌려주면 **게이트가 다음 틱부터 안 돈다.** 절대 그러지 말 것.
    db.outputs.execOut = (_og.ExecutionAttributeState.ENABLED if _any
                          else _og.ExecutionAttributeState.DISABLED)
    return True
'''



# 혼자 확인은 `scripts/selftest_ros_graph.py` 가 한다.
# ★ 여기에 __main__ 을 두면 안 된다 — 이 파일을 직접 실행하면 최상단
#   `import omni.graph.core` 가 SimulationApp 보다 먼저 돌아 ModuleNotFoundError 다.
#   이 모듈은 **Isaac 이 이미 떠 있다**는 전제의 라이브러리다.
