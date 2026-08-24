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
"""
import os

import yaml

REL = os.path.join("src", "packing_bringup", "config", "system.yaml")
REL_INNER = os.path.join("packing_bringup", "config", "system.yaml")


def _candidates(path=None):
    if path:
        yield os.path.expanduser(path)
    env = os.environ.get("PALLETRIS_CONFIG")
    if env:
        yield os.path.expanduser(env)
    for start in (os.path.dirname(os.path.realpath(__file__)), os.getcwd()):
        d = start
        for _ in range(8):                      # 8단계면 어떤 배치에서도 루트에 닿는다
            yield os.path.join(d, REL)
            yield os.path.join(d, REL_INNER)
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent


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
