"""`config.py` 를 **이름이 아니라 경로로** 불러온다.

## 왜 이게 필요한가

`import config as C` 는 Isaac 안에서 **다른 파일을 잡는다.**

    /home/rokey/isaacsim/exts/omni.pip.compute/pip_prebundle/cv2/config.py

번들 OpenCV 가 자기 디렉터리를 `sys.path` **맨 앞**에 꽂는데, 그 안에도
`config.py` 가 있다. 우리 `src/packing_sim` 을 sys.path 에 넣어 둬도 cv2 가
나중에 0번 자리를 가져가면 그쪽이 이긴다. 그리고 cv2 의 `config.py` 는 단독으로
실행되면 안 되는 파일이라 (`LOADER_DIR` 이 로더의 지역변수다) 이렇게 죽는다:

    NameError: name 'LOADER_DIR' is not defined

실측 2026-08-24: 픽업 카메라 인식(`camera_pick.detect`)이 이 에러로 죽고 있었다.
메시지에 우리 이름이 하나도 안 나와서 **Isaac 의 포인트클라우드 API 버그로 오해했다.**
`Camera.get_pointcloud` 를 안 쓰는 경로로 갈아끼워도 그대로 죽었고, 그제서야
트레이스백을 남겨 보고 `perception.py:22 import config as C` 가 범인인 걸 알았다.

## 어떻게 푸는가

이름 조회를 안 한다. `config.py` 의 **파일 경로**를 직접 지정해 읽는다.
sys.path 에 뭐가 끼어들든 상관없어진다.

`_cfgload` 라는 이름 자체는 겹칠 일이 없으므로 이 모듈은 평범하게 import 해도 된다.

    from _cfgload import C
"""
import importlib.util as _ilu
from pathlib import Path as _Path

_PATH = _Path(__file__).resolve().parent / "config.py"
_spec = _ilu.spec_from_file_location("packing_sim_config", _PATH)
if _spec is None or _spec.loader is None:      # 파일이 없으면 여기서 분명히 죽는다
    raise ImportError(f"config.py 를 못 찾았다: {_PATH}")
C = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(C)
