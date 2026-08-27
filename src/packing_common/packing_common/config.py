"""config 로더 — `system.yaml` 하나가 모든 숫자의 출처다 (작업명세서 §4, 철칙 3).

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : (전 구간) 모든 숫자의 단일 출처를 읽는다
    ║     불러 주는 쪽        : 모든 노드 · 스크립트
    ║     이 파일이 부르는 쪽 : system.yaml (+ test_3.yaml 오버레이)
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

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


def deep_merge(base, over):
    """`over` 를 `base` 위에 겹친다. dict 는 파고들고 나머지는 덮어쓴다."""
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load(path=None):
    """설정을 읽는다. `PL_CONFIG_OVERLAY` 가 있으면 그 위에 겹친다.

    ★★ 오버레이가 필요한 이유 (2026-08-26 김병직):
      3 구역 씬 `test_3` 은 좌표계가 통째로 다르다 — 팔레트가 셀 컨베이어
      위(z 950)고 페데스탈이 1950 이며 이격이 X 방향이다. 그 값들이
      `config/test_3.yaml` 에 있는데 그건 **오버레이지 완전한 설정이 아니다.**
      최상위 키가 9 개 없다 (`physics` · `service` · `heightmap` · `recognition` ·
      `lighting` · `profiles` · `run_profile` · `arbiter` · `stage`).
      통째로 바꾸면 `cfg["physics"]` 에서 KeyError 가 난다 (실측).

    ★ **Isaac 과 ROS 노드가 같은 값을 봐야 한다.** 한쪽만 오버레이를 읽으면
      관제탑은 test1-1 좌표로 계획하고 Isaac 은 test_3 좌표로 움직인다 —
      에러 없이 엉뚱한 자리에 놓는다. `scripts/_paths.py` 도 같은 규약이다.

    환경변수:
      `PL_CONFIG_OVERLAY`  기본 설정 **위에 겹친다** (부분 설정이어도 된다)
      `PALLETRIS_CONFIG`   기본 설정 파일 자체를 바꾼다 (완전한 설정일 것)
    """
    with open(find_config(path), encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    over = os.environ.get("PL_CONFIG_OVERLAY")
    if over:
        with open(_resolve_overlay(over), encoding="utf-8") as f:
            cfg = deep_merge(cfg, yaml.safe_load(f))
    return cfg


def _resolve_overlay(path):
    """오버레이 경로. 상대경로면 **cwd 다음에 저장소 루트**로도 찾는다.

    ★ 2026-08-26 실측: `run_demo.sh` 는 `cd $ISAAC_DIR` 뒤에 실행한다. 그래서
      `PL_CONFIG_OVERLAY=src/packing_bringup/config/test_3.yaml` 같은 저장소 기준
      상대경로가 **현재 디렉터리 기준으로 풀려 FileNotFoundError** 가 났다.
      launch 에서 넘길 때도 노드의 cwd 를 보장할 수 없다.
    """
    p = os.path.expanduser(str(path))
    if os.path.isabs(p) or os.path.exists(p):
        return p
    # find_config 가 찾아낸 system.yaml 에서 저장소 루트를 역산한다
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.realpath(find_config())))))
    cand = os.path.join(root, p)
    if os.path.exists(cand):
        return cand
    raise FileNotFoundError(
        "PL_CONFIG_OVERLAY 를 못 찾았다: %s\n  찾아본 곳: %s · %s"
        % (path, os.path.abspath(p), cand))
