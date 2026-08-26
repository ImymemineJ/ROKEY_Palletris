"""config 로더 — `system.yaml` 하나가 모든 숫자의 출처다 (작업명세서 §4, 철칙 3).

경로를 하드코딩하지 않는다. `~/palletris_ws/...` 를 기본값으로 박아 뒀더니 CI 에서
`FileNotFoundError: /github/home/palletris_ws/...` 로 테스트가 통째로 깨졌다 —
워크스페이스가 어디에 체크아웃되는지는 환경마다 다르다.

찾는 순서:
  1. 인자로 준 경로
  2. 환경변수 `PALLETRIS_CONFIG`
  3. 이 파일 위치에서 위로 올라가며 `src/packing_bringup/config/system.yaml`
     (심볼릭 링크를 풀어 원본 소스 트리를 따라간다 — `--symlink-install` 대응)
  4. 현재 작업 디렉터리에서 위로 올라가며 같은 것 (CI 는 저장소 루트에서 돈다)
  5. ★ ROS 패키지 인덱스의 `share/packing_bringup/config/system.yaml`
     (2026-08-26 추가) — 소스 트리 없이 install 만 있는 경우다.

★ 5 번을 왜 이제야 넣나
  `packing_bringup/setup.py` 는 이 파일을 share 에 **계속 설치해 오고 있었다.**
  주석까지 "소스 없이 install 만 배포할 때의 대비책" 이라고 적혀 있는데, 정작
  여기서 그 자리를 안 봐서 **죽은 폴백**이었다. 워크스페이스 밖에서 노드를 띄우면
  `PALLETRIS_CONFIG` 없이는 못 찾았다.

★ 1~4 의 순서는 **한 글자도 안 바꿨다.** 소스 트리가 정본이라는 규약(편집이 재빌드
  없이 먹혀야 한다)이 그대로 유지돼야 하고, 순서를 건드리면 CI 가 조용히 다른
  yaml 을 집을 수 있다. 5 번은 **맨 뒤에 덧붙이기만** 했다.
"""
import os

import yaml

from packing_common.paths import _share_dir, _up

REL = os.path.join("src", "packing_bringup", "config", "system.yaml")
REL_INNER = os.path.join("packing_bringup", "config", "system.yaml")


def _candidates(path=None):
    if path:
        yield os.path.expanduser(path)
    env = os.environ.get("PALLETRIS_CONFIG")
    if env:
        yield os.path.expanduser(env)
    for start in (os.path.dirname(os.path.realpath(__file__)), os.getcwd()):
        # ★ 위로 올라가는 구현은 paths._up 하나뿐이다. 두 벌이면 한쪽만 고쳐진다.
        for d in _up(start):
            yield os.path.join(d, REL)
            yield os.path.join(d, REL_INNER)
    # ★ 마지막 폴백. 소스 트리가 아예 없는 설치본에서만 여기까지 온다.
    share = _share_dir("packing_bringup")
    if share:
        yield os.path.join(share, "config", "system.yaml")


def find_config(path=None):
    """system.yaml 의 실제 경로. 못 찾으면 FileNotFoundError."""
    tried = []
    for c in _candidates(path):
        if os.path.isfile(c):
            return c
        tried.append(c)
    raise FileNotFoundError(
        "system.yaml 을 찾지 못했다. 워크스페이스 루트에서 실행하거나 "
        "PALLETRIS_CONFIG 로 경로를 지정할 것.\n  찾아본 곳:\n    "
        + "\n    ".join(dict.fromkeys(tried)))


def load(path=None):
    with open(find_config(path), encoding="utf-8") as f:
        return yaml.safe_load(f)
