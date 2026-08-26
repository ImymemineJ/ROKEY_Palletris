"""소터 셀 direction 스위칭 — 정사각 벨트 조각 하나를 소터로 쓴다. (2026-08-25)

`IsaacConveyor.inputs:direction` 은 `conveyorPrim` 의 **로컬 축**이다. 정사각 조각
하나에 이 값을 런타임으로 갈아끼우면 그 셀이 직진/좌배출/우배출을 하는 소터가 된다.

## 무엇을 바꿀 수 있고 무엇은 못 바꾸나

  소터가 꺾는 것은 **진행 방향**이지 물체의 **자세(yaw)** 가 아니다.
  박스를 지선으로 빼는 데는 자세가 필요 없으므로 소터로 충분하다.
  팔레트 반출처럼 자세를 90° 돌려야 하는 것은 **커브 벨트**여야 한다
  (안 돌리면 적재함을 가로지르는 치수가 1200x3=3600 > 2800 이라 상차가 안 된다).

## 규약

  - `rclpy` import 금지 — Isaac 내장 Python 3.11 프로세스에서 돈다
  - 숫자는 전부 `system.yaml`/씬 설정에서 읽어 생성자로 넘긴다 (CLAUDE.md §4)
  - 씬 로직이므로 소켓/토픽을 새로 파지 않는다

## 실측으로 정한 것 (2026-08-25, 빈 스테이지 샌드박스)

  1. **`inputs:direction` 은 `float3` 하나뿐이다.** float2/float3 를 런타임에
     가르는 분기는 필요 없다 — 타입이 다르면 씬이 잘못된 것이므로 **죽어야** 한다.
  2. **월드 방향을 그대로 쓰면 안 된다.** 셀 Xform 에 yaw 가 걸려 있으면
     로컬 +X 가 월드 +X 가 아니다. `ConveyorCtl._local_axis` 로 환산한다 —
     "셀 yaw 를 0 으로 두세요" 라는 배치 제약이 사라진다. 본선에서 이미
     검증된 경로다 (`conveyor_ctl.set_direction`).
  3. **latch 에 타임아웃이 필요하다.** 박스가 셀 경계에 걸쳐 멎으면 `_inside`
     가 계속 참이라 direction 이 DIVERT 에 물린 채 뒤 박스까지 딸려 나간다.
"""


def _conveyor_prim(stage, node):
    """`inputs:conveyorPrim` 관계가 가리키는 실제 벨트 prim.

    direction 을 이 prim 의 로컬축으로 환산해야 하므로 반드시 있어야 한다.
    """
    tg = node.GetRelationship("inputs:conveyorPrim").GetTargets()
    prim = stage.GetPrimAtPath(tg[0]) if tg else None
    if not prim or not prim.IsValid():
        raise RuntimeError(
            "%s 의 inputs:conveyorPrim 이 비었다. Stage 에서 벨트 prim 을 "
            "연결할 것 (상위 Xform 이 아니라 Belt/Rollers 여야 한다)" % node.GetPath())
    return prim


def _set_direction(node, conveyor, wx, wy):
    """월드 방향 (wx, wy) 를 벨트 로컬축으로 환산해 써넣는다."""
    from pxr import Gf

    # ★ 이 디렉터리는 `__init__.py` 가 없다. `sim_main.py` 가 HERE 를 sys.path 에
    #   넣고 평평하게 import 하는 것이 이 트랙의 관례다 (sim_main.py:62).
    try:
        from conveyor_ctl import ConveyorCtl
    except ImportError:                       # 패키지로 import 한 경우
        from .conveyor_ctl import ConveyorCtl

    attr = node.GetAttribute("inputs:direction")
    if not attr.IsValid():
        raise RuntimeError(
            "%s 에 inputs:direction 이 없다. ConveyorNode(OGN) prim 경로가 맞는지 "
            "Stage 에서 확인할 것 — 벨트 메시 경로가 아니다" % node.GetPath())
    # ★ float3 가 정본이다 (실측). 다른 타입이면 씬이 잘못됐으니 조용히 넘기지 않는다.
    if "3" not in str(attr.GetTypeName()):
        raise RuntimeError("%s 의 direction 타입이 %s 다 (float3 여야 한다)"
                           % (node.GetPath(), attr.GetTypeName()))
    attr.Set(Gf.Vec3f(*ConveyorCtl._local_axis(conveyor, (wx, wy, 0.0))))


def _set_velocity(node, value):
    attr = node.GetAttribute("inputs:velocity")
    if attr.IsValid():
        attr.Set(float(value))


class SorterCell:
    """벨트 조각 하나 = 소터 셀 하나.

    node_path   : ConveyorNode(OGN) 의 prim 경로. 벨트 메시 경로가 아니다.
    center_x/y  : 셀 중심의 월드 좌표 [m]
    half_len    : 배출 판정을 하는 반폭 [m]. 박스 최대 변(0.48)보다 넉넉히.
    thru_world  : 직진의 **월드** 방향 (x, y). 이 라인은 -X 로 흐른다.
    divert_world: 배출의 **월드** 방향 (x, y).
    divert_zone : 이 셀이 빼낼 zone 문자열. None 이면 항상 직진.
    latch_timeout_s : 잡은 박스가 이 시간 안에 셀을 못 벗어나면 강제로 푼다.
                      0 이면 타임아웃 없음.
    """

    THROUGH = "through"
    DIVERT = "divert"

    def __init__(self, stage, node_path, center_x, center_y, half_len,
                 divert_zone, thru_world=(-1.0, 0.0), divert_world=(0.0, 1.0),
                 velocity=0.5, latch_timeout_s=5.0):
        self.stage = stage
        self.node = stage.GetPrimAtPath(node_path)
        if not self.node or not self.node.IsValid():
            raise RuntimeError("prim 없음: %s" % node_path)
        self.conveyor = _conveyor_prim(stage, self.node)

        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.half_len = float(half_len)
        self.divert_zone = divert_zone
        self.thru_world = (float(thru_world[0]), float(thru_world[1]))
        self.divert_world = (float(divert_world[0]), float(divert_world[1]))
        self.latch_timeout_s = float(latch_timeout_s)

        self.state = self.THROUGH
        self.latched_box = None
        self._latched_for = 0.0
        self.stats = {"divert": 0, "timeout": 0}

        _set_velocity(self.node, velocity)
        self.apply(self.THROUGH)

    # ----------------------------------------------------------------
    def apply(self, state):
        w = self.divert_world if state == self.DIVERT else self.thru_world
        _set_direction(self.node, self.conveyor, w[0], w[1])
        self.state = state

    def inside(self, x, y):
        return (abs(x - self.center_x) <= self.half_len
                and abs(y - self.center_y) <= self.half_len)

    # ----------------------------------------------------------------
    def update(self, boxes, dt=0.0):
        """매 물리 스텝마다 호출.

        boxes : {box_id: (x, y, zone)}  월드 좌표 [m]
        dt    : 지난 스텝의 시간 [s]. latch 타임아웃에만 쓴다.
        """
        if self.divert_zone is None:
            return

        # 배출 중인 박스가 셀을 벗어났으면 직진으로 되돌린다
        if self.latched_box is not None:
            self._latched_for += float(dt)
            entry = boxes.get(self.latched_box)
            if entry is None or not self.inside(entry[0], entry[1]):
                self.latched_box = None
                self._latched_for = 0.0
                self.apply(self.THROUGH)
            elif (self.latch_timeout_s > 0.0
                  and self._latched_for > self.latch_timeout_s):
                # ★ 박스가 셀 경계에 걸려 멎었다. 안 풀면 direction 이 DIVERT 에
                #   물린 채 뒤 박스까지 딸려 나간다.
                self.stats["timeout"] += 1
                self.latched_box = None
                self._latched_for = 0.0
                self.apply(self.THROUGH)
            return

        # 셀 안에 있고 zone 이 맞는 박스를 하나만 잡는다
        for box_id, entry in boxes.items():
            x, y, zone = entry[0], entry[1], entry[2]
            if zone != self.divert_zone or not self.inside(x, y):
                continue
            self.latched_box = box_id
            self._latched_for = 0.0
            self.stats["divert"] += 1
            self.apply(self.DIVERT)
            return


class SorterLine:
    """직렬로 놓인 소터 셀들을 한 번에 돌린다."""

    def __init__(self, cells):
        self.cells = list(cells)

    def update(self, boxes, dt=0.0):
        for cell in self.cells:
            cell.update(boxes, dt)


def build_from_config(stage, cfg):
    """설정에서 생성한다. 좌표를 코드에 박지 않는다 (CLAUDE.md §4).

    sorter:
      velocity: 0.7
      thru_world:   [-1, 0]      # 본선 진행 (스폰 -> 존 C)
      divert_world: [0, 1]       # 지선 (+Y)
      latch_timeout_s: 5.0
      cells:
        - node_path: /World/sorter_a/ConveyorBeltGraph/ConveyorNode
          center_mm: [-7637, 0]
          half_len_mm: 350
          divert_zone: "A"
    """
    vel = float(cfg.get("velocity", 0.5))
    thru = cfg.get("thru_world", (-1.0, 0.0))
    div = cfg.get("divert_world", (0.0, 1.0))
    tmo = float(cfg.get("latch_timeout_s", 5.0))
    cells = []
    for item in cfg["cells"]:
        cx_mm, cy_mm = item["center_mm"]
        cells.append(SorterCell(
            stage=stage,
            node_path=item["node_path"],
            center_x=cx_mm / 1000.0,
            center_y=cy_mm / 1000.0,
            half_len=item["half_len_mm"] / 1000.0,
            divert_zone=item.get("divert_zone"),
            thru_world=item.get("thru_world", thru),
            divert_world=item.get("divert_world", div),
            velocity=float(item.get("velocity", vel)),
            latch_timeout_s=float(item.get("latch_timeout_s", tmo)),
        ))
    return SorterLine(cells)
