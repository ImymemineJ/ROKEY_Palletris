"""bringup.launch.py 검증 — 구역 목록과 카메라 목록이 실제로 짝이 맞는가.

★ launch 는 실행해 봐야 알 수 있는 실수가 유난히 많은 곳이다:
  · zones 와 pick_cam_ns 의 개수가 다르면 **다른 구역 카메라로 좌표를 낸다**
  · 노드 이름이 겹치면 파라미터가 섞인다
  둘 다 로그에 에러가 안 뜨고 조용히 틀린다. 그래서 여기서 잡는다.
"""
import importlib.util
import pathlib

import pytest

_LAUNCH = (pathlib.Path(__file__).resolve().parents[1]
           / "launch" / "bringup.launch.py")


def _mod():
    spec = importlib.util.spec_from_file_location("bringup_launch", _LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_launch_file_exists():
    assert _LAUNCH.is_file()


@pytest.mark.parametrize("text,want", [
    ("['a','b','c']", ["a", "b", "c"]),
    ('["a", "b"]', ["a", "b"]),
    ("b", ["b"]),
    ("a,b,c", ["a", "b", "c"]),
    ("a b c", ["a", "b", "c"]),
    ("[]", []),
])
def test_as_list_parses_launch_argument(text, want):
    """launch 인자는 전부 문자열로 들어온다. 목록으로 못 펴면 구역이 하나로 붙는다."""
    assert _mod()._as_list(text) == want


def test_launch_description_builds():
    """선언 단계에서 터지지 않는가 — 여기서 죽으면 노드가 하나도 안 뜬다."""
    ld = _mod().generate_launch_description()
    assert ld is not None
    assert len(ld.entities) > 0


def test_declared_arguments_cover_multizone():
    """★ 3구역을 띄우려면 이 인자들이 있어야 한다. 하나라도 빠지면 1구역으로 뜬다."""
    ld = _mod().generate_launch_description()
    names = {getattr(e, "name", None) for e in ld.entities}
    for need in ("zone", "zones", "pick_cam_ns", "use_sim_time"):
        assert need in names, f"launch 인자 {need!r} 가 없다"


def test_default_is_single_zone():
    """기본값은 1구역이어야 한다 — 씬에 픽업 카메라가 아직 한 대뿐이다."""
    m = _mod()
    ld = m.generate_launch_description()
    dflt = {getattr(e, "name", None): e for e in ld.entities
            if getattr(e, "name", None) in ("zones", "pick_cam_ns")}
    for k, e in dflt.items():
        txt = "".join(getattr(s, "text", "") for s in e.default_value)
        assert len(m._as_list(txt)) == 1, f"{k} 기본값이 1개가 아니다: {txt!r}"
