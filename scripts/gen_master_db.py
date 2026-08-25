#!/usr/bin/env python3
"""boxes_master 생성 — 문서·.gitignore 가 이 경로를 부른다 (트랙 D 가이드 D1).

실제 구현은 packing_db 패키지 안에 있다. 여기 두면 ROS 를 source 하지 않고도
저장소 루트에서 바로 돌릴 수 있다:

    python3 scripts/gen_master_db.py
    ros2 run packing_db gen_master_db      # 같은 것
"""
import os
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))
for p in (os.path.join(_HERE, "..", "src", "packing_db"),
          os.path.join(_HERE, "..", "src", "packing_common")):
    sys.path.insert(0, os.path.realpath(p))

from packing_db.gen_master_db import main   # noqa: E402

if __name__ == "__main__":
    main()
