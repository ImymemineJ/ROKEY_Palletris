"""paths.py — 자산 경로 찾기.

★ 이 테스트가 막는 것: "개발 PC 에서만 되는 경로".
  2026-08-26 이전에는 schema.sql 을 찾는 코드가 `--symlink-install` 아래서만
  돌았다. 일반 colcon build 로 설치하면 db_service_node 와 gen_master_db 가
  **둘 다 죽었는데**, 팀 전원이 그 옵션을 쓰는 바람에 아무도 몰랐다.
  경로 회귀의 최종 방어선은 scripts/verify_fresh_install.sh 이고,
  여기서는 그 앞단(후보 생성 규칙)을 잡는다.
"""
import os

import pytest

from packing_common import paths


def test_workspace_root_finds_the_repo():
    """워크스페이스 표식(src/packing_bringup/config/system.yaml)으로 루트를 찾는다."""
    ws = paths.workspace_root()
    assert ws is not None, "워크스페이스 루트를 못 찾았다"
    assert os.path.isfile(os.path.join(ws, "src", "packing_bringup",
                                       "config", "system.yaml"))


def test_find_data_locates_schema():
    """packing_db 의 schema.sql — 파이썬 패키지 안에 있는 자산."""
    p = paths.find_data("packing_db", "schema.sql")
    assert os.path.isfile(p)
    assert p.endswith("schema.sql")


def test_find_data_locates_config_under_package_root():
    """system.yaml — 패키지 루트 바로 아래에 있는 자산 (packing_bringup/config/)."""
    p = paths.find_data("packing_bringup", os.path.join("config", "system.yaml"))
    assert os.path.isfile(p)


def test_find_data_missing_lists_what_it_tried():
    """★ 못 찾으면 **찾아본 곳을 전부** 적어야 한다.

    경로 문제는 "어디를 봤는지" 를 모르면 고칠 수가 없다. 옛 코드는
    `SystemExit("schema.sql 을 찾지 못했다")` 한 줄이라 후보를 손으로 재현해야 했다.
    """
    with pytest.raises(FileNotFoundError) as e:
        paths.find_data("packing_db", "없는파일.csv", __file__)
    msg = str(e.value)
    assert "찾아본 곳" in msg
    assert msg.count("없는파일.csv") >= 2, "후보가 하나도 안 적혔다"


def test_install_layout_candidate_goes_up_four_levels():
    """★ 설치본에서 share 는 모듈 기준 **4 단계 위**다.

    <install>/<pkg>/lib/python3.12/site-packages/<pkg>/모듈.py
    <install>/<pkg>/share/<pkg>/자산

    옛 코드는 1 단계만 올라가 `site-packages/share/...` 를 봤다. 그게 그 버그다.
    """
    fake = os.path.join("/opt/ws/install/packing_db/lib/python3.12",
                        "site-packages/packing_db/mod.py")
    cands = list(paths._candidates("packing_db", "schema.sql", fake))
    assert "/opt/ws/install/packing_db/share/packing_db/schema.sql" in cands


def test_source_tree_comes_before_share():
    """★ 소스가 먼저다. system.yaml 은 "소스가 정본" 이고 정답지 CSV 도 같다 —
    --regen 직후 다음 실행이 새 파일을 바로 써야 한다."""
    cands = list(paths._candidates("packing_db", "schema.sql", __file__))
    src = [i for i, c in enumerate(cands) if os.sep + "src" + os.sep in c]
    shr = [i for i, c in enumerate(cands) if os.sep + "share" + os.sep in c]
    assert src and shr, f"소스·share 후보가 둘 다 있어야 한다: {cands}"
    assert min(src) < min(shr), "share 후보가 소스보다 먼저 나온다"


def test_default_db_path_prefers_argument(tmp_path):
    p = paths.default_db_path(str(tmp_path / "x.db"))
    assert p == str(tmp_path / "x.db")


def test_default_db_path_honours_env(monkeypatch, tmp_path):
    monkeypatch.setenv("PALLETRIS_DB", str(tmp_path / "env.db"))
    assert paths.default_db_path() == str(tmp_path / "env.db")


def test_default_db_path_falls_back_to_workspace(monkeypatch):
    monkeypatch.delenv("PALLETRIS_DB", raising=False)
    p = paths.default_db_path()
    assert p.endswith("palletris.db")
    assert p == os.path.join(paths.workspace_root(), "palletris.db")


def test_paths_has_no_ros_import_at_module_level():
    """★ packing_common 은 "ROS 의존성이 없다" 고 package.xml 이 약속한다.

    ament_index_python 을 최상단에서 import 하면 Isaac 내장 파이썬이나
    ROS 를 source 안 한 scripts/ 에서 이 모듈이 통째로 죽는다.
    """
    with open(paths.__file__, encoding="utf-8") as f:
        lines = f.read().split("\n")
    # 들여쓰기 없는 import 줄 = 모듈 최상단. 주석·독스트링은 세지 않는다.
    top = [ln for ln in lines
           if ln.startswith("import ") or ln.startswith("from ")]
    assert not [ln for ln in top if "ament" in ln], \
        f"ament 를 최상단에서 import 하고 있다: {top}"
