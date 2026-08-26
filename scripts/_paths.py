"""저장소 안의 경로를 한 군데서 푼다. `scripts/` 의 검증 스크립트가 전부 이걸 쓴다.

경로를 스크립트마다 적으면 반드시 어긋난다 — 실제로 구 저장소에서
`../../../M0609/...` 같은 상대경로가 파일마다 흩어져 있었다.

`packing_robot` 이 colcon 으로 빌드된 뒤에는 `import packing_robot.fk` 가 되지만,
게이팅 스크립트는 **빌드 전에도 돌아야 하므로** sys.path 로 직접 붙인다.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent          # ~/palletris_ws
ROBOT = ROOT / "src/packing_robot"

URDF = ROBOT / "urdf/h2017.urdf"
MESH_COLLISION = ROBOT / "urdf/meshes/h2017_collision"
MESH_VISUAL = ROBOT / "urdf/meshes/h2017_white"
DESCRIPTOR = ROBOT / "config/h2017_descriptor.yaml"
RMPFLOW = ROBOT / "config/h2017_rmpflow_common.yaml"
SYSTEM_YAML = ROOT / "src/packing_bringup/config/system.yaml"

# fk.py 는 packing_robot 패키지 안에 있다. 빌드 전에도 import 되게 붙인다.
if str(ROBOT / "packing_robot") not in sys.path:
    sys.path.insert(0, str(ROBOT / "packing_robot"))
if str(ROOT / "src/packing_algo") not in sys.path:
    sys.path.insert(0, str(ROOT / "src/packing_algo"))
if str(ROOT / "src/packing_common") not in sys.path:
    sys.path.insert(0, str(ROOT / "src/packing_common"))


def system_config():
    """system.yaml 을 읽는다. **모든 숫자의 단일 출처** (작업명세서 4, 철칙 3)."""
    import yaml
    return yaml.safe_load(SYSTEM_YAML.read_text())


def check():
    missing = [p for p in (URDF, MESH_COLLISION, DESCRIPTOR, SYSTEM_YAML) if not p.exists()]
    if missing:
        sys.exit("없는 경로:\n  " + "\n  ".join(str(m) for m in missing))
