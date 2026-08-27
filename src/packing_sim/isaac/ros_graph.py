"""Isaac 안 ROS 2 통로 — Action Graph 하나로 끝낸다. **소켓 없음** (CLAUDE.md §2).

이 파일이 대체하는 것
    `sim_server.py` 의 JSON 소켓(localhost:8765). §2 가 폐기했다.
    Isaac 안팎을 잇는 통로는 ROS 2 하나뿐이다 — Action Graph 의 C/rcl 백엔드가
    Python 버전 차이(3.11 vs 3.12)를 우회하므로 rclpy 가 필요 없다.

세우는 것 (`/World/PalletrisROS`)
    OnPlaybackTick ─┬─> ROS2Subscriber  /zone_<z>/robot/cmd   (packing_msgs/RobotCmd)
                    ├─> [서비스 4종]     Req -> Script -> Res
                    └─> EventGate ─────> ROS2Publisher /zone_<z>/robot/event (RobotEvent)

    `/clock` · `/tf` 는 원래 김학민(A)의 씬 그래프 소관이지만, **씬에 없으면**
    `build_clock_tf()` 가 런타임에 세운다 (씬에 있으면 비켜선다). 카메라는 안 만든다.
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
# ★★ **임시 대역** — 씬에 카메라 prim 이 아직 하나도 없다 (2026-08-26, 89 개 삭제).
#   §3-2 정본은 이 토픽을 **인식 노드(트랙 D)** 가 낸다. 여기서 내는 것은 계약 위반이
#   아니라 **대역**이다 — 카메라가 들어오면 `PL_GTPICK=0` 으로 끄고 인식 노드로 돌린다.
#   끄지 않고 둘 다 켜면 발행자가 둘이라 관제탑이 두 좌표를 번갈아 받는다.
#
#   왜 필요한가: 이 한 토픽이 없으면 관제탑이 **영원히 IDLE** 이라 그 뒤 사슬
#   (algo · db · 로봇 · 반출)이 통째로 안 돈다. 카메라를 기다리는 동안 나머지를
#   못 재는 것이 더 큰 손해다.
PICK_TOPIC_FMT = "/zone_%s/box_at_pickup"
# ★ PL_GTPICK=2 — **신원 대역 모드** (2026-08-27 김병직 설계).
#   풀 비전에서 좌표는 존 인식 카메라가 내지만, 픽업 카메라로 박스 윗면 QR 을
#   읽는 건 사양(§5) 밖이고 실측도 실패한다 (3/202). 신원은 씬 실측이 계속
#   내되 **다른 토픽**으로 낸다 — 관제탑이 "좌표=카메라 · 신원=이 토픽" 으로
#   합친다. 같은 토픽에 발행자 둘이 서는 사고(§ gtpick 주석)가 원천 차단된다.
if os.environ.get("PL_GTPICK") == "2":
    PICK_TOPIC_FMT = "/zone_%s/box_ident"


def cmd_topic(zone):
    return CMD_TOPIC_FMT % str(zone).lower()


def evt_topic(zone):
    return EVT_TOPIC_FMT % str(zone).lower()


def pick_topic(zone):
    return PICK_TOPIC_FMT % str(zone).lower()

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
# 보낼 BoxAtPickup {zone: fields}. **큐가 아니라 최신값 한 벌**이다 —
#   도착은 상태이지 사건이 아니라서, 밀리면 낡은 좌표를 뒤늦게 내보내게 된다.
#   관제탑도 같은 박스를 여러 번 받는 것을 전제로 만들어져 있다
#   (orchestrator_node.on_box_arrived: "detector 는 5Hz 폴링이라 여러 번 알린다").
_PICKUPS = {}
# ★★ 서비스별 요청 필드 {key: [이름, ...]}. **모듈에 둬야 한다** (2026-08-26 실측).
#   예전에는 생성한 스크립트의 **최상단**에 `_FIELDS = [...]` 로 박았는데,
#   Isaac 의 ScriptNode 들은 최상단 네임스페이스를 **공유**한다 — 나중에 실행된
#   스크립트의 `_FIELDS` 가 앞의 것을 통째로 덮었다. 증상:
#       [ros_graph] !! set_velocity 요청 필드를 못 읽었다: box_id(AttributeError...)
#       [ros_side]  h_set_velocity 실패: bad mps   받은 요청={}
#   `box_id` 는 DespawnBox 필드다. 이름(`{key}`)은 f-string 으로 박혀 맞게 찍히는데
#   필드만 남의 것이라, 로그만 봐서는 "왜 mps 가 나쁘지" 로만 보인다.
#   -> 키로 찾으면 네임스페이스가 겹쳐도 안 섞인다.
_SRV_FIELDS = {}
_state = {"wired": False, "zones": [], "seen_seq": {}}
# 디버그 출력 (환경변수 PL_ROSGRAPH_DEBUG=1)
DEBUG = os.environ.get("PL_ROSGRAPH_DEBUG") == "1"
# 픽업 좌표를 씬 실측(ground truth)으로 대신 낼지. 카메라가 들어오면 0 으로 끈다.
GT_PICK = os.environ.get("PL_GTPICK", "1") != "0"


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


def send_pickup(zone=None, **fields):
    """BoxAtPickup 최신값을 갈아 끼운다. 실제 발행은 다음 틱의 EventGate 가 한다.

    ★ `send_event` 와 달리 **덮어쓴다.** 도착은 사건이 아니라 상태다 — 큐로 쌓으면
      박스가 이미 떠난 뒤에 낡은 좌표가 뒤늦게 나간다.
    """
    _PICKUPS[_one_zone(zone)] = dict(fields)


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


# ── /clock · /tf — 씬에 없으면 우리가 낸다 ─────────────────────────────────
def build_clock_tf(cfg, log=print):
    """`/clock` 과 `/tf` 발행 그래프를 **씬에 없을 때만** 세운다.

    ★ 왜 여기 있나 — 원래는 씬(트랙 A) 소관이다. `setup_test3_scene.py [11]` 이
      같은 그래프를 USD 에 써 넣는다. 그런데 2026-08-26 실측에서 씬의 OmniGraph
      43 개가 **전부 컨베이어**였다: `/clock` 발행자 0, `/tf` 발행자 0.

      `/clock` 이 없으면 `use_sim_time:=true` 인 노드의 시계가 0 에 멎어
      **타이머가 하나도 안 뜬다** — 관제탑의 상태 발행도, 로봇 20 초 타임아웃
      감시도 같이 죽는다. 노드는 살아 있고 에러도 안 난다 (CLAUDE.md §10).
      씬이 고쳐질 때까지 전 구간이 못 돈다는 뜻이라 런타임에서 메운다.

    ★★ **씬이 이미 내고 있으면 손대지 않는다.** `/clock` 발행자가 둘이면 두 시계가
      번갈아 나가 tf 조회가 "extrapolation into the future" 로 실패한다. 그래서
      그래프 경로에 Clock 노드가 있으면 그냥 비켜선다 — 트랙 A 가 씬을 고치는
      순간 이 함수는 저절로 아무것도 안 하게 된다. 지울 필요가 없다.

    Returns: "scene" (씬이 이미 있다) · "built" (우리가 세웠다) · "skip" (설정 없음)
    """
    import omni.usd
    from pxr import Sdf

    ros = (cfg.get("ros") or {})
    if not ros:
        log("[ros_graph] 설정에 ros 가 없다 — /clock·/tf 를 안 만든다")
        return "skip"

    g = str(ros.get("graph_path", "/World/ROS2Bridge"))
    st = omni.usd.get_context().get_stage()

    # ★★ PL_CLOCKTF=2 — 씬 노드와 무관하게 **우리가 낸다** (2026-08-27).
    #   PL_CAMROS=2 와 같은 사연이다: 씬에 /World/ROS2Bridge/Clock 노드가
    #   생겼는데(setup_test3_scene [11] 적용본) 실측은 발행자 0 이다 —
    #   ros2 topic info /clock → Publisher 0 · Subscription 3, /tf 는 토픽조차
    #   없음 (2026-08-27 새벽). 노드만 보고 비켜서면 **아무도 안 낸다.**
    #   씬 그래프와 경로가 겹치지 않게 별도 경로에 세운다.
    _force = os.environ.get("PL_CLOCKTF") == "2"
    if _force:
        g = GRAPH + "_ClockTf"
        log("[ros_graph] PL_CLOCKTF=2 — 씬 노드와 무관하게 /clock·/tf 를 우리가 낸다")

    # 씬이 이미 내고 있나. 그래프 경로가 다를 수 있으니 **노드 타입으로** 훑는다.
    scene_clock = scene_tf = None
    if not _force:
        for prim in st.Traverse():
            if prim.GetTypeName() == "OmniGraphNode":
                t = str((prim.GetAttribute("node:type").Get()
                         if prim.GetAttribute("node:type") else "") or "")
                if t.endswith("ROS2PublishClock"):
                    scene_clock = prim
                elif t.endswith("ROS2PublishTransformTree"):
                    scene_tf = prim
    if scene_clock is not None:
        log("[ros_graph] /clock 은 씬이 낸다 (%s) — 건너뛴다" % scene_clock.GetPath())
        # ★★ 비켜서기 **전에** 광학 프레임을 씬의 tf 에 붙인다 (2026-08-26).
        #   예전에는 여기서 바로 돌아가서, 씬이 /clock 을 내는 순간 광학 프레임
        #   생성까지 같이 사라졌다 — 그러면 이미지 frameId 를 아무리 맞게 줘도
        #   tf 에 그 프레임이 없어 인식이 전부 "평면이 카메라 뒤" 로 죽는다.
        if scene_tf is not None:
            try:
                opts = _ensure_optical_frames(cfg, st, log)
                rel = scene_tf.GetRelationship("inputs:targetPrims")
                tg = list(rel.GetTargets())
                added = [o for o in opts if Sdf.Path(o) not in tg]
                if added:
                    rel.SetTargets(tg + [Sdf.Path(o) for o in added])
                    log("[ros_graph] 씬 tf 에 광학 프레임 %d 개 추가: %s"
                        % (len(added), ", ".join(a.rsplit("/", 1)[-1] for a in added)))
            except Exception as e:                              # noqa: BLE001
                log("[ros_graph] !! 씬 tf 에 광학 프레임을 못 붙였다: %r" % (e,))
        else:
            log("[ros_graph] ⚠ 씬에 tf 발행 노드가 없다 — 광학 프레임을 못 붙인다")
        return "scene"

    K, c = og.Controller.Keys, og.Controller
    NS = "isaacsim.ros2.bridge."
    tfc = ros.get("tf") or {}
    try:
        c.edit(
            {"graph_path": g, "evaluator_name": "execution"},
            {K.CREATE_NODES: [("Tick", "omni.graph.action.OnPlaybackTick"),
                              ("Ctx", NS + "ROS2Context"),
                              # ★★ 시간 소스. 이게 없으면 `/clock` 이 **계속 0** 을
                              #   낸다 (2026-08-26 실측: sec 0 nanosec 0 고정).
                              #   토픽은 멀쩡히 보이고 발행자 수도 1 이라 "됐다"고
                              #   착각하기 딱 좋은데, 구독하는 노드들의 시계가
                              #   0 에 멎어 **타이머가 하나도 안 뜬다** — /clock 이
                              #   아예 없을 때와 증상이 똑같다 (§10).
                              ("SimTime", "isaacsim.core.nodes."
                                          "IsaacReadSimulationTime"),
                              ("Clock", NS + "ROS2PublishClock"),
                              ("Tf", NS + "ROS2PublishTransformTree")],
             K.CONNECT: [("Tick.outputs:tick", "Clock.inputs:execIn"),
                         ("Tick.outputs:tick", "Tf.inputs:execIn"),
                         ("SimTime.outputs:simulationTime",
                          "Clock.inputs:timeStamp"),
                         ("SimTime.outputs:simulationTime",
                          "Tf.inputs:timeStamp"),
                         ("Ctx.outputs:context", "Clock.inputs:context"),
                         ("Ctx.outputs:context", "Tf.inputs:context")],
             # ROS_DOMAIN_ID 를 박지 않는다 — 팀이 값을 바꾸면 씬도 고쳐야 한다
             K.SET_VALUES: [("Ctx.inputs:useDomainIDEnvVar", True),
                            ("Clock.inputs:topicName",
                             str(ros.get("clock_topic", "/clock"))),
                            ("Tf.inputs:topicName",
                             str(tfc.get("topic", "/tf")))]})
    except Exception as e:                                      # noqa: BLE001
        log("[ros_graph] !! /clock·/tf 그래프 생성 실패: %r" % (e,))
        return "skip"

    # ⚠ 확장이 안 켜져 있으면 노드가 안 만들어진다. 그대로 만지면
    #   `Accessed invalid null prim` 으로 **죽는다** (setup_test3_scene [11] 실측).
    tf_node = st.GetPrimAtPath(g + "/Tf")
    if not tf_node.IsValid() or not st.GetPrimAtPath(g + "/Clock").IsValid():
        log("[ros_graph] !! ROS2 브리지 노드가 안 만들어졌다 — 확장 미기동.\n"
            "   LD_LIBRARY_PATH 를 확인할 것 (scripts/run_test3_ros.sh 참고)")
        return "skip"

    par = str(tfc.get("parent_prim", "/World"))
    if st.GetPrimAtPath(par).IsValid():
        tf_node.GetRelationship("inputs:parentPrim").SetTargets([Sdf.Path(par)])

    # ★ 프레임 이름은 **prim 이름**이 된다. 씬에 없는 prim 은 조용히 버린다 —
    #   넣으면 그래프가 통째로 죽어 /clock 까지 같이 멎는다.
    tgts, miss = [], []
    for tp in (tfc.get("target_prims") or []):
        (tgts if st.GetPrimAtPath(str(tp)).IsValid() else miss).append(str(tp))

    # ★★ 카메라 **광학 프레임**도 여기서 만든다 — 이미지를 누가 쏘든 필요하다.
    #   tf 는 이미지 발행자와 **독립**이므로 여기가 맞는 자리다.
    #   2026-08-26: `cameras.pick_prim` 설정 기준으로 바꿨다 (rsd455 리그 대응) —
    #   옛 `/CamPick_*` 하드코딩은 씬에서 지워진 이름이다.
    for opt in _ensure_optical_frames(cfg, st, log):
        if opt not in tgts:
            tgts.append(opt)

    tf_node.GetRelationship("inputs:targetPrims").SetTargets(
        [Sdf.Path(t) for t in tgts])

    log("[ros_graph] /clock·/tf 를 **우리가** 세웠다 (%s) — 씬에 없어서다.\n"
        "   프레임 %d 개: %s"
        % (g, len(tgts), ", ".join(t.rsplit("/", 1)[-1] for t in tgts) or "없음"))
    if miss:
        log("   ⚠ 씬에 없어 뺀 prim: %s" % ", ".join(miss))
    return "built"


def _ensure_optical_frames(cfg, stage, log=print):
    """설정의 카메라 렌즈마다 **광학 프레임** 자식을 만들고 경로 목록을 돌려준다.

    ★ 2026-08-26 rsd455 리그 대응. 두 가지가 옛 방식과 다르다:
      · 프레임 이름이 **리그 이름**에서 온다 (`rsd455_a_optical`). 렌즈 prim 이름
        (`RSD455`)으로 지으면 A·B·C 가 **전부 같은 프레임명**이 되어 tf 가 겹친다
        — 프레임 이름은 prim 이름이기 때문이다.
      · 광학 프레임을 **렌즈 prim 의 자식**으로 둔다. 그러면 렌즈의 자세·오프셋을
        그대로 상속해 dz 보정이 필요 없다 (옛 `_make_optical_frame` 의 14.5 mm
        수동 보정은 형제 관계였을 때 이야기다).
    """
    from pxr import Gf, Sdf, UsdGeom
    cams = (cfg.get("cameras") or {})
    lens_paths = list((cams.get("pick_prim") or {}).values())
    for k in ("spawn_prim", "line2_prim"):
        v = cams.get(k)
        if v:
            lens_paths.append(str(v))
    out = []
    for lens in lens_paths:
        prim = stage.GetPrimAtPath(str(lens))
        if not prim or not prim.IsValid():
            log("[ros_graph] !! 카메라 %s 가 씬에 없다 — 광학 프레임 생략" % lens)
            continue
        # 리그 이름 = /World 바로 아래 요소 (rsd455_a · CamPick_A ...)
        parts = str(lens).strip("/").split("/")
        rig = parts[1] if len(parts) > 1 else parts[-1]
        opt = "%s/%s_optical" % (lens, rig)
        if not stage.GetPrimAtPath(opt).IsValid():
            xf = UsdGeom.Xform.Define(stage, opt)
            xf.ClearXformOpOrder()
            # X 축 180도: USD 카메라(-Z 시선) -> ROS 광학(+z 전방)
            xf.AddOrientOp().Set(Gf.Quatf(0.0, Gf.Vec3f(1.0, 0.0, 0.0)))
        out.append(opt)
    return out


def _make_optical_frame(stage, path, cam_path):
    """`path` 에 ROS 광학 규약 자식 Xform 을 만든다 (USD 카메라 기준 X축 180도).

    카메라 프림의 로컬 오프셋도 같이 넣는다 — `CamPick_*/Camera` 가 부모보다
    14.5 mm 아래 있어서, 무시하면 1.4 m 높이에서 약 1 % 배율 오차가 난다.
    """
    from pxr import Gf, UsdGeom
    cam = stage.GetPrimAtPath(cam_path)
    dz = 0.0
    if cam.IsValid():
        for op in UsdGeom.Xformable(cam).GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                v = op.Get()
                if v is not None:
                    dz = float(v[2])
                break
    xf = UsdGeom.Xform.Define(stage, path)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, dz))
    # X 축 180도: (w, x, y, z) = (0, 1, 0, 0)
    xf.AddOrientOp().Set(Gf.Quatf(0.0, Gf.Vec3f(1.0, 0.0, 0.0)))
    return path


# ── 픽업 카메라 — 씬에 발행자가 없으면 우리가 낸다 ─────────────────────────
def _mute_scene_camera_graphs(st, log=print):
    """씬의 카메라 ROS 그래프를 **런타임에만** 끈다 (PL_CAMROS=2 일 때).

    ★ 왜 (2026-08-27 새 씬 `test_3.usd` 09:14 실측): 씬에 `ROS_Camera` 그래프가
      **7 개** 들어왔는데 전부 기본값 그대로라 토픽이 다 같다 —
      `/rgb` · `/depth` · `camera_info` (frameId `sim_camera`). 문제가 둘이다:
        ① 한 토픽에 발행자 7 개 → 서로 다른 카메라 이미지가 섞여 나온다.
           우리 인식은 `/cam1` · `/pick_cam_a` 를 보므로 쓰이지도 않는다.
        ② **렌더 프로덕트가 7 개 더 생긴다.** 렌더는 이미 병목이고(프레임당
           30~60 ms) 1280x720 5 대가 33 ms 를 먹은 실측이 있다 — 배속이 무너진다.
           `_03`·`_04` 는 `_01`·`_02` 와 **같은 카메라**를 가리키는 중복이다.
    ★ 파일을 고치지 않는다. `SetActive(False)` 는 열린 스테이지에만 적용되고
      디스크의 USD 는 그대로다 (씬은 김학민 소유 — CLAUDE.md §11).
      되살리려면 `PL_SCENE_CAMS=1`.
    ★ `/World/ROS2Bridge` 의 clock·tf 는 **안 건드린다.** 여기서는 그래프 안에
      `ROS2CameraHelper` 가 있는 것만 고른다.
    """
    if os.environ.get("PL_SCENE_CAMS") == "1":
        log("[ros_graph] PL_SCENE_CAMS=1 — 씬 카메라 그래프를 그대로 둔다")
        return 0
    from pxr import Usd            # 이 파일은 Isaac 안에서만 도므로 지역 import
    muted = []
    for prim in st.Traverse():
        if prim.GetTypeName() != "OmniGraph":
            continue
        has_cam = False
        for n in Usd.PrimRange(prim):
            t = n.GetAttribute("node:type")
            if t and str(t.Get() or "").endswith("ROS2CameraHelper"):
                has_cam = True
                break
        if has_cam and prim.IsActive():
            prim.SetActive(False)
            muted.append(prim.GetName())
    if muted:
        log("[ros_graph] 씬 카메라 그래프 %d 개를 껐다 (%s) — 토픽이 전부 /rgb 로 "
            "겹치고 렌더 프로덕트만 늘린다. 되살리려면 PL_SCENE_CAMS=1"
            % (len(muted), ", ".join(muted)))
    return len(muted)


def build_cameras(cfg, zones, log=print):
    """`/pick_cam_{a|b|c}/rgb` + `/camera_info` 를 런타임에 발행한다.

    ★ 왜 런타임인가 — `ROS2CameraHelper` 는 `renderProductPath` 를 요구하는데
      그건 **런타임에 만들어지는 물건**이라 USD 에 그냥 적어 둘 수 없다.
      `setup_test3_scene.py [11]` 이 카메라를 못 넣은 이유가 이것이다.
      우리는 이미 런타임이므로 여기서 만들면 된다 (`/clock`·`/tf` 와 같은 방식).

    ★ 왜 필요한가 — 이게 없으면 사슬이 끊긴다:
          pick_cam -> (발행자 0) -> vision -> box_at_pickup(안 옴) -> 관제탑 IDLE
      Isaac 은 픽업 좌표를 이미 알고 있는데(`pickup_check` present=true) 그게
      ROS 로 안 나가서 관제탑이 영원히 기다렸다 (실측 2026-08-26).

    ★★ **helper 가 두 개다.** `ROS2CameraHelper` 는 `camera_info` 를 안 쏜다 —
      `ROS2CameraInfoHelper` 를 따로 붙여야 한다. `vision_node` 는 내부
      파라미터를 코드에 안 박고 `camera_info` 에서 읽으므로, 없으면
      "camera_info 대기 중" 으로 멈춘다 (`vision_node.py:57` 주석).

    ★ `frameId` 는 **씬 prim 이름**이어야 한다 (`CamPick_A`). `vision_node` 가
      이미지 헤더의 `frame_id` 로 tf 를 조회하는데, `ROS2PublishTransformTree` 는
      프레임 이름을 prim 이름으로 정하기 때문이다. 둘이 다르면 tf 조회가 실패한다.

    ⚠ 오프스크린 렌더는 **GUI 조명 리그를 못 받는다.** 돔 라이트가 꺼져 있으면
      눈으로는 멀쩡한데 이미지가 캄캄하다 (CLAUDE.md §5). 아래에서 검사한다.

    Returns: 만든 카메라 수
    """
    import omni.usd

    # ★ PL_CAMROS=0 — 카메라 발행을 **아예 안 만든다** (2026-08-27).
    #   렌더 프로덕트는 발행 주기와 무관하게 **렌더 프레임마다** GPU 로 그려져서,
    #   1280x720 5 대가 프레임당 약 33 ms 를 먹었다 (실측 — 렌더 스킵 2 로도
    #   벽시계 1.0 배속에 묶인 원인). vision:=false + gtpick 대역으로 도는 동안은
    #   이미지 소비자가 없으니 끄는 게 순이득이다. 카메라 붙일 때 2 로 되돌릴 것.
    if os.environ.get("PL_CAMROS") == "0":
        log("[ros_graph] PL_CAMROS=0 — 카메라 발행 없음 (렌더 비용 절감)")
        return 0

    rec = (cfg.get("recognition") or {})
    res = list(rec.get("resolution") or [1280, 720])
    st = omni.usd.get_context().get_stage()

    # 씬이 이미 카메라를 쏘고 있으면 비켜선다 (/clock 과 같은 이유 — 발행자 2 개면
    # vision 이 두 이미지를 번갈아 받아 tf 시각이 어긋난다).
    # ★★ PL_CAMROS=2 는 **강제**다 (2026-08-26). 씬에 `RGBPublish` 노드가 있는데
    #   실제로는 토픽이 하나도 안 나오는 상태를 실측했다 — 노드만 보고 비켜서면
    #   **둘 다 안 쏘는** 상태가 된다. 씬 노드가 진짜로 쏘는지는 빌드 시점에
    #   알 수 없으므로(rclpy 없음), 사람이 `ros2 topic list` 로 확인하고 강제한다.
    if os.environ.get("PL_CAMROS") != "2":
        for prim in st.Traverse():
            if prim.GetTypeName() == "OmniGraphNode":
                t = prim.GetAttribute("node:type")
                if t and str(t.Get() or "").endswith("ROS2CameraHelper"):
                    log("[ros_graph] 픽업 카메라는 씬이 쏜다 (%s) — 건너뛴다.\n"
                        "   ⚠ 토픽이 실제로 안 나오면 PL_CAMROS=2 로 강제할 것" % prim.GetPath())
                    return 0
    else:
        log("[ros_graph] PL_CAMROS=2 — 씬 노드와 무관하게 우리가 쏜다")
        _mute_scene_camera_graphs(st, log)

    try:
        import omni.replicator.core as rep
    except ImportError as e:                                    # noqa: BLE001
        log("[ros_graph] !! omni.replicator.core 를 못 쓴다 (%r) — 카메라 생략" % (e,))
        return 0

    # 돔 라이트 확인. 없으면 이미지가 캄캄하게 나가는데 **에러는 안 난다.**
    if not any(p.GetTypeName() == "DomeLight" for p in st.Traverse()):
        log("[ros_graph] ⚠ 씬에 DomeLight 가 없다 — 카메라 이미지가 어두울 수 있다"
            " (오프스크린 렌더는 GUI 조명 리그를 안 받는다)")

    K, c = og.Controller.Keys, og.Controller
    NS = "isaacsim.ros2.bridge."
    made = 0
    # ★ 2026-08-26 — prim 경로는 **설정이 정본**이다. 옛 `/World/CamPick_A/Camera`
    #   는 씬에서 지워졌다 (카메라 89 개 삭제 후 rsd455 리그 5 대 설치, PR #33).
    #   `cameras.pick_prim.{A,B,C}` 가 그 리그의 Color 렌즈를 가리킨다.
    cams_cfg = (cfg.get("cameras") or {})
    pick_prims = (cams_cfg.get("pick_prim") or {})

    # ★ 픽업 리그 정렬 (2026-08-27 김병직 — 풀 비전 5호 실패의 처방).
    #   카메라가 스토퍼를 비껴 보면 키 큰 박스(5호 340mm)는 옆면까지 실루엣에
    #   잡혀 치수 맞춤(허용 60mm)이 터진다 — 실측: 3·4호 통과(6.4~9.5mm),
    #   5호 성공 0/443. 리그를 **스토퍼 박스 중심 바로 위**로 옮기면 옆면이
    #   사라진다 (오프셋 60mm/높이 1.16m ≈ 3도). USD 는 안 건드린다 —
    #   런타임 이동이라 되돌리기는 오버레이 키를 빼면 끝이다.
    #   ⚠ 회전은 안 만진다 — 리그 내부 자세(하향)를 그대로 쓴다.
    _align = cams_cfg.get("align_pick_z_mm")
    if _align:
        from pxr import Gf, UsdGeom
        _stop_m = cfg["conveyor"]["stopper_x_mm"] / 1000.0
        _half = max(max(t["w_mm"], t["d_mm"]) for t in cfg["boxes"]["types"]) / 2000.0
        _cross = (cfg["zones"].get("pickup_cross_mm") or {})
        _ax_y = str(cfg["conveyor"].get("pickup_axis", "x")).lower() == "y"
        for z in zones:
            cw = _cross.get(str(z).upper())
            lens = pick_prims.get(str(z).upper())
            if not cw or not lens:
                continue
            rig_path = "/" + "/".join(lens.strip("/").split("/")[:2])
            rig = st.GetPrimAtPath(rig_path)
            if not rig.IsValid():
                continue
            cm = (cw[0] + cw[1]) / 2000.0
            tgt_along = _stop_m - _half          # 5호가 스토퍼에 닿았을 때 중심
            pos = (Gf.Vec3d(cm, tgt_along, _align / 1000.0) if _ax_y
                   else Gf.Vec3d(tgt_along, cm, _align / 1000.0))
            xf = UsdGeom.Xformable(rig)
            _ops = {o.GetOpName(): o for o in xf.GetOrderedXformOps()}
            _op = _ops.get("xformOp:translate") or xf.AddTranslateOp()
            _op.Set(pos)
            log("[ros_graph] 픽업 리그 정렬 %s -> (%.0f, %.0f, %.0f) mm"
                % (rig_path, pos[0] * 1000, pos[1] * 1000, pos[2] * 1000))

    targets = []
    for z in zones:
        zu = str(z).upper()
        targets.append(("Cam_%s" % zu, "/pick_cam_%s" % zu.lower(),
                        pick_prims.get(zu) or "/World/CamPick_%s/Camera" % zu))
    # ★ 라인 카메라 2대 (2026-08-27, 김병직 지시 — vision:=true 전환 준비).
    #   cam1(투입부 등록)·cam2(검증). vision_node 의 cam1_ns/cam2_ns 기본값이
    #   /cam1·/cam2 라 네임스페이스를 거기 맞춘다. 렌즈 경로는 설정이 정본
    #   (rsd455_1/_2 리그) — 광학 프레임은 build_clock_tf 가 이미 tf 에 넣었다.
    if cams_cfg.get("spawn_prim"):
        targets.append(("Cam_L1", "/cam1", str(cams_cfg["spawn_prim"])))
    if cams_cfg.get("line2_prim"):
        targets.append(("Cam_L2", "/cam2", str(cams_cfg["line2_prim"])))
    for gname, ns, cam_path in targets:
        if not st.GetPrimAtPath(cam_path).IsValid():
            log("[ros_graph] !! %s 가 씬에 없다 — %s 카메라 생략" % (cam_path, ns))
            continue
        try:
            rp = rep.create.render_product(cam_path, tuple(int(v) for v in res))
            rp_path = rp.path if hasattr(rp, "path") else str(rp)
        except Exception as e:                                  # noqa: BLE001
            log("[ros_graph] !! %s 렌더프로덕트 실패: %r" % (cam_path, e))
            continue

        # ★★ **광학 프레임을 따로 만든다.** 이게 없으면 인식이 통째로 실패한다.
        #
        #   `geometry.pixels_to_plane` 는 **ROS 광학 규약**으로 광선을 만든다 —
        #   x 오른쪽 · y 아래 · **z 전방**. 그런데 `ROS2PublishTransformTree` 가
        #   내는 tf 는 **USD 프림 자세** 그대로다. `CamPick_*` 는 회전이 없어
        #   (+Z 가 월드 위) 광선이 하늘로 뻗고, `s <= 0` 이 되어
        #   "평면이 카메라 뒤에 있다" 로 **3 종 전부** 예외가 난다.
        #   그러면 로그에는 "사양 3종 어느 것과도 안 맞는다" 로 찍혀서
        #   **크기 문제처럼 보인다** — 실제로는 투영이 한 번도 성공 못 한 것이다
        #   (실측 2026-08-26).
        #
        #   USD 카메라는 -Z 를 본다. 광학 프레임은 거기서 **X 축 180도** 돌린 것이다:
        #     X -> X · Y -> -Y · Z -> -Z(=시선 방향)
        #   자식 Xform 을 하나 만들어 tf 대상에 넣으면 `PublishTransformTree` 가
        #   프림 이름 그대로 프레임을 내 준다 — 노드를 새로 안 붙여도 된다.
        #   ★ 프레임 자체는 `build_clock_tf` 가 이미 만들어 tf 에 넣어 뒀다
        #     (누가 이미지를 쏘든 필요해서 그쪽으로 옮겼다). 여기서는 이름만 쓴다.
        # ★ 광학 프레임 이름은 `_ensure_optical_frames` 와 같은 규칙이다 —
        #   **리그 이름**(=/World 바로 아래)에서 온다. 렌즈 prim 이름(`RSD455`)으로
        #   지으면 A·B·C 가 전부 같은 프레임명이 되어 tf 가 겹친다.
        _parts = cam_path.strip("/").split("/")
        _rig = _parts[1] if len(_parts) > 1 else _parts[-1]
        opt_path = "%s/%s_optical" % (cam_path, _rig)
        if st.GetPrimAtPath(opt_path).IsValid():
            frame = opt_path.rsplit("/", 1)[-1]
        else:
            log("[ros_graph] !! 광학 프레임 %s 이 없다 — prim 프레임을 쓴다."
                " 인식이 실패할 것이다 (build_clock_tf 를 먼저 불렀나?)" % opt_path)
            frame = _rig

        # ── 깊이 (2026-08-27) ────────────────────────────────────────────
        # ★★ 왜 붙이는가: RGB 만으로는 **종류를 못 가른다.** `geometry` 는 사양 3 종
        #   평면에 각각 투영해 가장 잘 맞는 것을 고르는 **추측**인데, 실측 정확도가
        #   46 % 였다 (2026-08-27, 두 판 모두). 한 등급 작게 보면 로봇이 명령받은
        #   z 로 내려가다 **진짜 박스 윗면에 얹혀** 멈추고 IK_FAIL 이 난다 —
        #   실측 도달 z 가 실제 윗면 +5~9 mm 였다. 4호·5호가 한 개도 안 집혔다.
        #   깊이는 윗면 z 를 **직접 재므로** 이 추측을 없앤다 (CLAUDE.md §5).
        #
        # ★ **rgb 와 같은 render product** 를 쓴다. 그래야 stamp 가 정확히 같아
        #   짝 맞추기가 100 % 가 된다 — 한 프레임만 어긋나도 깊이 교차검증이
        #   `HEIGHT_MISMATCH` 만 낸다 (CLAUDE.md §10 의 증상표).
        #
        # ★ `type:"depth"` 는 `distance_to_image_plane`(광축거리)를 낸다.
        #   그래서 `recognition.depth.is_range` 는 **false**(기본) 가 맞다.
        #   시선거리로 바꾸는 헬퍼를 쓰게 되면 그때 true 로 올릴 것.
        #
        #   끄기: PL_CAMDEPTH=0 (렌더 비용이 부담될 때). 끄면 예전과 완전히 같다.
        _depth_on = os.environ.get("PL_CAMDEPTH", "1") != "0"
        _nodes = [("Tick", "omni.graph.action.OnPlaybackTick"),
                  ("Ctx", NS + "ROS2Context"),
                  ("Rgb", NS + "ROS2CameraHelper"),
                  ("Info", NS + "ROS2CameraInfoHelper")]
        _conn = [("Tick.outputs:tick", "Rgb.inputs:execIn"),
                 ("Tick.outputs:tick", "Info.inputs:execIn"),
                 ("Ctx.outputs:context", "Rgb.inputs:context"),
                 ("Ctx.outputs:context", "Info.inputs:context")]
        _vals = [("Ctx.inputs:useDomainIDEnvVar", True),
                 ("Rgb.inputs:renderProductPath", rp_path),
                 ("Rgb.inputs:frameId", frame),
                 ("Rgb.inputs:nodeNamespace", ns),
                 ("Rgb.inputs:topicName", "rgb"),
                 ("Rgb.inputs:type", "rgb"),
                 ("Info.inputs:renderProductPath", rp_path),
                 ("Info.inputs:frameId", frame),
                 ("Info.inputs:nodeNamespace", ns),
                 ("Info.inputs:topicName", "camera_info")]
        if _depth_on:
            _nodes.append(("Depth", NS + "ROS2CameraHelper"))
            _conn += [("Tick.outputs:tick", "Depth.inputs:execIn"),
                      ("Ctx.outputs:context", "Depth.inputs:context")]
            _vals += [("Depth.inputs:renderProductPath", rp_path),
                      ("Depth.inputs:frameId", frame),
                      ("Depth.inputs:nodeNamespace", ns),
                      ("Depth.inputs:topicName", "depth"),
                      ("Depth.inputs:type", "depth")]

        g = "%s/%s" % (GRAPH, gname)
        try:
            c.edit(
                {"graph_path": g, "evaluator_name": "execution"},
                {K.CREATE_NODES: _nodes,
                 K.CONNECT: _conn,
                 K.SET_VALUES: _vals})
        except Exception as e:                                  # noqa: BLE001
            log("[ros_graph] !! %s 카메라 그래프 실패: %r" % (ns, e))
            continue
        made += 1
        log("[ros_graph] 카메라 %s -> %s/rgb%s + /camera_info  (frameId %s · %dx%d)"
            % (cam_path, ns, " + /depth" if _depth_on else "", frame, res[0], res[1]))

    return made


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
        _PICKUPS.setdefault(z, None)
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

        # ── 임시 대역: 픽업 좌표 발행 ────────────────────────
        # ★★ **새 ScriptNode 를 Tick 에 달지 않는다.** 위 주석의 실측대로 Tick 에
        #   ScriptNode 를 여럿 매달면 첫 틱 뒤로 하나만 살아남는다. 그래서 이 발행도
        #   기존 EvtGate 하나가 같이 맡는다 — 노드는 Publisher 만 는다.
        if GT_PICK:
            ppub = f"PickPub_{z}"
            nodes += [(ppub, "isaacsim.ros2.bridge.ROS2Publisher")]
            values += [
                (f"{ppub}.inputs:topicName", pick_topic(z)),
                (f"{ppub}.inputs:messagePackage", PKG),
                (f"{ppub}.inputs:messageSubfolder", "msg"),
                (f"{ppub}.inputs:messageName", "BoxAtPickup"),
            ]
            conn += [("Ctx.outputs:context", f"{ppub}.inputs:context"),
                     ("EvtGate.outputs:execOut", f"{ppub}.inputs:execIn")]

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
    # ★ 스크립트 최상단이 아니라 **모듈 표**에 넣는다 (`_SRV_FIELDS` 주석 참조).
    _SRV_FIELDS[key] = names
    return f'''
import importlib
import omni.graph.core as _og

_M = importlib.import_module("{__name__}")

def setup(db): pass

def compute(db):
    # ★ Isaac 메인 스레드다. 여기서 기다리면 물리가 안 돌아 판정이 영영 안 난다 (§9).
    #   핸들러는 **즉시 반환**해야 한다 — 오래 걸리는 일은 큐에 넣고 돌아온다.
    req = {{}}
    _miss = []
    # ★★ 값은 **Req 노드의 outputs:Request:<필드>** 에서 직접 읽는다 (2026-08-27).
    #   `db.inputs.<필드>` 는 string 은 전달되는데 **float32 는 연결값이 안 오고
    #   기본값 0.0 이 나온다** — 실측: 재가동 set_velocity(0.5) 가 왕복은 되는데
    #   항상 0.0 으로 처리됐다. 정지(0.0)만 있던 동안은 증상이 안 보였다
    #   ("bad mps 수정 이후 성공" 도 전부 0.0 이었다). Req 출력을 직접 읽으면
    #   연결 전달에 의존하지 않는다.
    _rq = _og.Controller.node(_M.GRAPH + "/{key}_Req")
    # ★ 모듈 표에서 **내 것만** 꺼낸다. 최상단 변수로 두면 다른 ScriptNode 가 덮는다.
    for f in _M._SRV_FIELDS.get("{key}", []):
        try:
            _at = _og.Controller.attribute("outputs:Request:" + f, _rq)
            v = _at.get() if _at is not None else getattr(db.inputs, f)
        except Exception as e:
            # ★ 조용히 넘기면 안 된다. 필드 하나가 빠지면 핸들러가
            #   "bad mps" 같은 **엉뚱한 사유**로 실패하고, 진짜 원인(그 필드를
            #   못 읽었다)은 아무 데도 안 남는다 (2026-08-26 실측: 재현에 실패해
            #   한참 헤맸다).
            _miss.append("%s(%r)" % (f, e))
            continue
        req[f] = str(v) if isinstance(v, (bytes, bytearray)) else v
    if _miss:
        print("[ros_graph] !! {key} 요청 필드를 못 읽었다: " + ", ".join(_miss))

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
    return f'''
import importlib
import omni.graph.core as _og

_M = importlib.import_module("{__name__}")
# ★ 최상단에 `_ZONES = [...]` 같은 이름을 두지 않는다 — ScriptNode 들이 최상단 네임스페이스를
#   공유해서 서로 덮는다 (`_SRV_FIELDS` 주석의 set_velocity 사고와 같은 부류).
#   지금은 게이트가 하나뿐이라 안 겹치지만 같은 함정을 남겨 두지 않는다.

def setup(db):
    if _M.DEBUG:
        print("[ros_graph] gate setup — 구역 %s" % _M._state["zones"])

def compute(db):
    _any = False
    for _z in _M._state["zones"]:
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

    # ── 임시 대역: 픽업 좌표 ────────────────────────────────
    # ★ 이벤트와 달리 **큐가 아니라 최신값**이다. 매 틱 같은 값을 다시 써도 되고,
    #   그래야 관제탑이 늦게 떠도 도착을 놓치지 않는다 (5Hz 폴링과 같은 모양).
    # ★ 여기서 _any 를 같이 올린다 — 안 올리면 로봇 이벤트가 없는 동안
    #   execOut 이 DISABLED 라 **픽업이 한 번도 안 나간다.**
    if _M.GT_PICK:
        for _z in _M._state["zones"]:
            pk = _M._PICKUPS.get(_z)
            if not pk:
                continue
            node = _og.Controller.node(_M.GRAPH + "/PickPub_" + _z)
            for k, v in pk.items():
                try:
                    _og.Controller.attribute("inputs:" + k, node).set(v)
                except Exception:
                    print("[ros_graph] %s BoxAtPickup 필드 %r 를 못 씀 — 계약 확인"
                          % (_z, k))
            # ★★ 쓰고 나면 **비운다** (2026-08-26). 안 비우면 값이 계속 남아
            #   게이트가 **매 틱** ENABLED 가 되고, 그러면 EvtPub 까지 같이 켜져
            #   낡은 RobotEvent 가 매 틱 재방송된다. 관제탑이 seq 로 거르긴 하지만
            #   `지난 명령의 결과 무시` 경고가 초당 수십 줄씩 쏟아져 로그를 덮는다.
            #   `_GtPick` 이 0.2 초마다 다시 채우므로 발행은 의도대로 5 Hz 다.
            _M._PICKUPS[_z] = None
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
