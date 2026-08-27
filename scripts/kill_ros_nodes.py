#!/usr/bin/env python3
"""bringup 이 남긴 ROS 노드 프로세스를 정리한다. (2026-08-26 김병직)

## 왜 `pkill` 을 쓰지 않나

`pkill -f orchestrator_node` 는 **자기를 부른 셸까지 죽인다.** `-f` 가 전체 명령줄을
보는데, 그 셸의 명령줄에도 방금 친 문자열이 그대로 들어 있기 때문이다.
`[o]rchestrator_node` 같은 우회도 소용없다 — 명령줄 **다른 자리**에 맨 단어가
있으면 정규식이 거기에 걸린다 (실측 2026-08-26: 종료코드 144 로 두 번 끊겼다).

## 왜 필요한가

`ros2 launch` 를 끊어도 **자식 파이썬 노드는 살아남는다.** 관제탑이 둘·셋이 되어
같은 구역에 명령을 겹쳐 쏘는데 **에러는 안 난다** — `ros2 topic info` 의
`Publisher count` 가 하나 더 보이는 것으로만 드러난다 (실측: 관제탑 3 개).
"""
import os
import signal
import subprocess
import sys

TARGETS = ("lib/palletris_orchestrator/", "lib/packing_vision/",
           "lib/packing_algo/", "lib/packing_db/", "bringup.launch.py")


def main():
    me = {os.getpid(), os.getppid()}
    out = subprocess.run(["ps", "-eo", "pid,args"],
                         capture_output=True, text=True).stdout
    killed = []
    for line in out.splitlines()[1:]:
        pid, _, args = line.strip().partition(" ")
        try:
            pid = int(pid)
        except ValueError:
            continue
        # ★ 자기 자신과 부모는 건드리지 않는다 (위 docstring 의 그 사고).
        if pid in me or __file__ in args:
            continue
        if any(t in args for t in TARGETS):
            try:
                os.kill(pid, signal.SIGTERM)
                killed.append(pid)
            except OSError:
                pass
    print("정리한 PID %d 개: %s" % (len(killed), killed or "-"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
