"""저장소 안의 경로를 한 군데서 푼다. `scripts/` 의 검증 스크립트가 전부 이걸 쓴다.

경로를 스크립트마다 적으면 반드시 어긋난다 — 실제로 구 저장소에서
`../../../M0609/...` 같은 상대경로가 파일마다 흩어져 있었다.

`packing_robot` 이 colcon 으로 빌드된 뒤에는 `import packing_robot.fk` 가 되지만,
게이팅 스크립트는 **빌드 전에도 돌아야 하므로** sys.path 로 직접 붙인다.
"""
import os
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


def deep_merge(base, over):
    """`over` 를 `base` 위에 겹친다. dict 는 파고들고 나머지는 덮어쓴다."""
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def system_config():
    """system.yaml 을 읽는다. **모든 숫자의 단일 출처** (작업명세서 4, 철칙 3).

    환경변수 둘 (2026-08-26 김병직):

      `PL_CONFIG_OVERLAY`  system.yaml **위에 겹친다.** 3 구역(test_3)이 이 방식이다.
      `PALLETRIS_CONFIG`   파일을 **통째로 바꾼다.** 완전한 설정일 때만 쓸 것.

    ★★ 왜 겹치기여야 하나 — `config/test_3.yaml` 은 **오버레이**다.
      최상위 키가 9 개 없다 (`physics` · `service` · `heightmap` · `recognition` ·
      `lighting` · `profiles` · `run_profile` · `arbiter` · `stage`).
      통째로 바꾸면 `cfg["physics"]["scene"]` 에서 KeyError 가 난다.
      ⚠ 그런데 **트레이스백이 안 보인다** — `sim_main` 의 `finally: kit.close()` 가
        프로세스를 먼저 끝내서 종료 코드 0 에 로그도 깨끗하다. 실측 2026-08-26:
        "Simulation App Shutting Down" 한 줄만 남고 6 초 만에 조용히 끝났다.
      이름은 `setup_test3_scene.py` 가 이미 쓰는 `PL_CONFIG_OVERLAY` 를 그대로 쓴다.

    ★ 겹치기가 맞는 또 하나의 이유: `zones` 하위에 `buffer_capacity` · `list` 는
      오버레이에 없다. 겹치면 그건 system.yaml 것이 살고 좌표만 test_3 것이 된다.
    """
    import yaml
    cfg = yaml.safe_load(_resolve(os.environ.get("PALLETRIS_CONFIG")
                                  or SYSTEM_YAML).read_text())
    over = os.environ.get("PL_CONFIG_OVERLAY")
    if over:
        cfg = deep_merge(cfg, yaml.safe_load(_resolve(over).read_text()))
    return cfg


def _resolve(path):
    """상대경로를 **워크스페이스 루트 기준**으로도 풀어 본다.

    ★ 2026-08-26 실측 버그: `run_demo.sh` 는 `cd $ISAAC_DIR` 를 한 뒤 실행한다.
      그래서 `PL_CONFIG_OVERLAY=src/packing_bringup/config/test_3.yaml` 처럼
      워크스페이스 기준 상대경로를 주면 **현재 디렉터리가 Isaac 폴더**라
      FileNotFoundError 가 난다 (`demo_stack15_test1_1.py` 는 자기 오버레이
      처리에서 이미 ROOT 기준으로 풀고 있었는데 여기만 안 그랬다).
      cwd -> ROOT 순으로 찾는다.
    """
    p = Path(os.path.expanduser(str(path)))
    if p.is_absolute() or p.exists():
        return p
    cand = ROOT / p
    if cand.exists():
        return cand
    raise FileNotFoundError(
        "설정 파일을 못 찾았다: %s\n  찾아본 곳: %s · %s" % (path, p.resolve(), cand))


def check():
    missing = [p for p in (URDF, MESH_COLLISION, DESCRIPTOR, SYSTEM_YAML) if not p.exists()]
    if missing:
        sys.exit("없는 경로:\n  " + "\n  ".join(str(m) for m in missing))
