"""자산 경로 찾기 — 전원이 이것만 쓴다. 손으로 짜지 말 것.

★ 왜 만들었나 (2026-08-26 실측)
  `schema.sql` · `system.yaml` 을 찾는 코드가 **세 군데에 따로** 있었고 셋 다
  `share` 를 안 봤다. 그런데 `setup.py` 들은 자산을 share 에 **제대로 설치하고**
  있었다 — `packing_bringup/setup.py` 는 주석으로 "소스 없이 install 만 배포할 때의
  대비책" 이라고까지 적어 뒀다. 만들어 놓고 찾는 쪽이 그 자리를 안 봤다.

  팀 전원이 `--symlink-install` 을 쓰는 덕에 `realpath(__file__)` 이 소스 트리로
  되돌아가 우연히 돌고 있었다. 그 옵션 없이 `colcon build` 하면:

      gen_master_db    FileNotFoundError: .../site-packages/packing_db/schema.sql
      db_service_node  SystemExit: schema.sql 을 찾지 못했다

  ★ 설치본에서 `share` 는 파이썬 모듈 기준 **4 단계 위**다.
      <install>/<pkg>/lib/python3.12/site-packages/<pkg>/   ← 모듈이 여기
      <install>/<pkg>/share/<pkg>/                          ← 자산은 여기
    옛 코드는 1 단계만 올라갔다. 그게 그 버그다.

★ 탐색은 **소스 트리가 먼저**다.
  `system.yaml` 은 "소스가 정본" 이 규약이라 편집이 재빌드 없이 먹혀야 한다
  (config.py 머리말). 정답지 CSV 도 같다 — `--regen` 직후 다음 실행이 새 파일을
  바로 써야 한다. share 는 **소스가 없을 때의 폴백**이지 우선 순위가 아니다.

★ `ament_index_python` 을 모듈 최상단에서 import 하지 않는다.
  package.xml 이 "ROS 의존성이 없어 Isaac 쪽 프로세스에서도 그대로 import 할 수
  있다" 고 약속한다. Isaac 내장 파이썬(3.11)이나 ROS 를 source 안 한 `scripts/`
  에서도 이 모듈은 살아 있어야 한다. 그래서 함수 안에서 try/except 로만 쓴다.
"""
import os

#: 워크스페이스를 알아보는 표식. 이 상대 경로가 있으면 거기가 루트다.
_MARKER = os.path.join("src", "packing_bringup", "config", "system.yaml")

#: 위로 몇 단계까지 올라가 볼까. 8 이면 어떤 배치에서도 파일시스템 루트에 닿는다.
_UP_LEVELS = 8


def _up(start, levels=_UP_LEVELS):
    """`start` 에서 부모로 올라가며 디렉터리를 내놓는다. 루트에 닿으면 멈춘다.

    ★ config.py 도 이 함수를 쓴다. 위로 올라가는 구현이 두 벌이면 한쪽만 고쳐진다.
    """
    d = os.path.abspath(start)
    for _ in range(levels):
        yield d
        parent = os.path.dirname(d)
        if parent == d:                 # 파일시스템 루트
            return
        d = parent


def workspace_root(start=None):
    """소스 트리 루트(`~/palletris_ws`). 못 찾으면 None.

    ★ 홈 디렉터리를 박지 않는다. 워크스페이스가 어디에 체크아웃되는지는 환경마다
      다르다 — CI 에서 `/github/home/...` 로 깨진 전례가 있다 (config.py 머리말).
    """
    for base in (start, os.path.dirname(os.path.realpath(__file__)), os.getcwd()):
        if not base:
            continue
        for d in _up(base):
            if os.path.isfile(os.path.join(d, _MARKER)):
                return d
    return None


def _share_dir(pkg):
    """ROS 패키지 인덱스가 아는 share 경로. 못 쓰면 None.

    ★ import 를 함수 안에 둔다 (머리말 참조). ament 가 없거나 패키지가 아직
      설치 안 됐으면 조용히 None — 이 함수는 여러 후보 중 하나일 뿐이다.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:                                   # noqa: BLE001
        return None


def _candidates(pkg, rel, module_file=None):
    """`find_data` 가 순서대로 볼 곳. 소스 먼저, share 나중 (머리말 참조)."""
    ws = workspace_root()
    if ws:
        # 파이썬 패키지 안 (packing_db/packing_db/schema.sql · data/*.csv)
        yield os.path.join(ws, "src", pkg, pkg, rel)
        # 패키지 루트 바로 아래 (packing_bringup/config/system.yaml)
        yield os.path.join(ws, "src", pkg, rel)

    if module_file:
        # --symlink-install 이면 realpath 가 소스 트리로 되돌아간다
        here = os.path.dirname(os.path.realpath(module_file))
        yield os.path.join(here, rel)
        # ★ 일반 설치본. share 는 4 단계 위다 (머리말 참조).
        #   ROS 를 source 안 했을 때도 되는 유일한 후보라 아래 share 후보와 별개다.
        yield os.path.normpath(
            os.path.join(here, "..", "..", "..", "..", "share", pkg, rel))

    share = _share_dir(pkg)
    if share:
        yield os.path.join(share, rel)


def find_data(pkg, rel, module_file=None):
    """패키지 자산의 실제 경로.

        find_data("packing_db", "schema.sql", __file__)
        find_data("packing_db", "data/boxes_master.csv", __file__)

    못 찾으면 FileNotFoundError — **찾아본 곳을 전부 적는다.** 경로 문제는
    "어디를 봤는지" 를 모르면 고칠 수가 없다.
    """
    tried = []
    for c in _candidates(pkg, rel, module_file):
        if os.path.isfile(c):
            return c
        tried.append(c)
    raise FileNotFoundError(
        f"{pkg} 의 {rel} 을 찾지 못했다.\n  찾아본 곳:\n    "
        + "\n    ".join(dict.fromkeys(tried)))


def default_db_path(path=None):
    """`palletris.db` 경로.

    인자 > 환경변수 `PALLETRIS_DB` > `<ws>/palletris.db` > `<cwd>/palletris.db`

    ★ 예전에는 db_service_node 와 gen_master_db 가 각각 "config 경로에서 4 단계 위"
      로 구했다. `find_config()` 가 share 를 돌려주는 순간 그 산술이 엉뚱한 곳을
      가리키는데, DB 는 없으면 **그냥 새로 만들어지므로** 빈 DB 를 조용히 쓰게 된다
      ("master 에 없는 박스" 증상). 그래서 워크스페이스 표식으로 찾는다.
    """
    if path:
        return os.path.abspath(os.path.expanduser(path))
    env = os.environ.get("PALLETRIS_DB")
    if env:
        return os.path.abspath(os.path.expanduser(env))
    ws = workspace_root()
    if ws:
        return os.path.join(ws, "palletris.db")
    return os.path.abspath("palletris.db")
