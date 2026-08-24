"""리허설 사전점검 — 통합 전에 30초 만에 "조용히 안 되는" 것들을 잡는다.

오늘 실제로 세 번 당한 것들이 전부 에러 없이 조용히 실패하는 종류였다:
  · ROS_DOMAIN_ID 불일치 (117 vs 115) → 노드는 뜨는데 서로 안 보인다
  · 좀비 노드 → 이전 heightmap 을 든 채로 살아 있어 "박스 1개 놓았는데 적재 27개"
  · packing_msgs 미빌드 → import 에러

실행:
    ros2 run packing_algo rehearsal_check
"""
import os
import shutil
import subprocess
import sys

OK, NG, WARN = "\033[32m✔\033[0m", "\033[31m✘\033[0m", "\033[33m▲\033[0m"
fails = []


def check(name, cond, hint="", warn_only=False):
    mark = OK if cond else (WARN if warn_only else NG)
    print(f" {mark} {name}")
    if not cond:
        if hint:
            print(f"     → {hint}")
        if not warn_only:
            fails.append(name)
    return cond


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15).stdout
    except subprocess.SubprocessError:
        return ""


def main():
    print("\n── 리허설 사전점검 ──────────────────────────────\n")

    # 1. 환경
    print("[환경]")
    dom = os.environ.get("ROS_DOMAIN_ID")
    check(f"ROS_DOMAIN_ID = {dom}", dom == "115",
          "팀 전원 115 여야 한다. 다르면 노드가 에러 없이 서로 안 보인다 (함정 #2)")
    wl = os.path.expanduser("~/.ros/fastdds_whitelist.xml")
    check("fastdds_whitelist.xml", os.path.isfile(wl),
          "cp scripts/fastdds_whitelist.xml ~/.ros/", warn_only=True)
    check("FASTRTPS_DEFAULT_PROFILES_FILE", bool(os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")),
          "~/.bashrc 에 export 하고 source", warn_only=True)

    # 2. 빌드
    print("\n[빌드]")
    ifaces = sh("ros2 interface list 2>/dev/null | grep -c packing_msgs").strip()
    check(f"packing_msgs 인터페이스 {ifaces or 0}개", (ifaces.isdigit() and int(ifaces) >= 20),
          "colcon build --symlink-install (옵션 필수 — 없이 빌드하면 캐시가 깨진다)")
    execs = sh("ros2 pkg executables 2>/dev/null | grep packing")
    for pkg, node in (("packing_algo", "algo_service_node"), ("packing_robot", "orchestrator")):
        check(f"{pkg} {node}", node in execs, f"{pkg} 가 빌드됐는지 확인")

    # 3. 좀비 노드
    print("\n[좀비 노드]")
    # ★ 자기 자신을 세면 안 된다. `ros2 run packing_algo rehearsal_check` 는
    #   install/packing_algo/lib/... 로 실행되므로 이 pgrep 에 그대로 걸린다 —
    #   그러면 아무것도 안 떠 있어도 항상 "좀비 1개" 로 실패한다.
    mine = {str(os.getpid()), str(os.getppid())}
    zombies = [l for l in sh("pgrep -af 'install/packing' ").splitlines()
               if "pgrep" not in l and l.split(" ", 1)[0] not in mine
               and "rehearsal_check" not in l]
    # ★ 경고가 아니라 실패로 다룬다. 좀비가 살아 있으면 서비스 서버가 둘이 되어
    #   **꽉 찬 팔레트를 든 옛 노드가 NO_FEASIBLE_POSE 를 답한다.** 새 노드는 OK 를
    #   로그에 찍는데 클라이언트는 다른 답을 받아, 로그만 보면 원인을 찾을 수 없다.
    #   실제로 두 번 당했다 (박스 1개에 "적재 27개" · 20개 전부 버림).
    check(f"살아 있는 packing 프로세스 {len(zombies)}개", len(zombies) == 0,
          "서비스 서버가 둘이 되어 옛 노드가 엉뚱한 답을 한다. 반드시 정리할 것:\n"
          "       ps -eo pid,args --no-headers | grep packing_ | grep -v grep "
          "| awk '{print $1}' | xargs -r kill -9"
          if zombies else "")

    # 4. config 정합성 (B·C 계약)
    print("\n[config · 좌표 계약]")
    try:
        from packing_common.config import load
        from packing_common.frames import pallet_place_to_world, world_to_pallet_place
        cfg = load()
        pal, z = cfg["pallet"], cfg["zones"]
        check(f"팔레트 {pal['width_mm']}×{pal['depth_mm']} · max_stack {pal['max_stack_mm']}",
              pal["width_mm"] == 1200 and pal["depth_mm"] == 800,
              "2026-08-22 90° 회전 이후 Px=1200(월드+X) · Py=800(월드+Y) 이어야 한다")
        check(f"support_tol_mm = {cfg['heightmap']['support_tol_mm']}",
              cfg["heightmap"]["support_tol_mm"] == cfg["heightmap"]["cell_mm"],
              "cell_mm 과 같아야 한다. 작으면 2층부터 전부 NO_FEASIBLE_POSE (함정 #10)")
        origin = z["pallet_origin_world_mm"]["B"]
        p = world_to_pallet_place([11.370, 0.475, 0.360], 0.0, 210, origin)
        back = pallet_place_to_world(p, 210, origin)
        check("frames.py 왕복 변환 (B·C 공용 계약)", abs(back[2] - 0.360) < 1e-9,
              "밑면↔중심 h/2 변환이 깨졌다 (함정 #11)")
    except Exception as e:                                     # noqa: BLE001
        check(f"packing_common import — {e}", False, "source install/setup.bash 했는지 확인")

    # 5. 노드 (떠 있으면)
    print("\n[실행 중인 노드]")
    svc = sh("ros2 service list 2>/dev/null")
    algo_up = all(f"/algo/{s}" in svc for s in
                  ("plan_placement", "update_heightmap", "cancel_plan", "reset"))
    check("/algo/* 서비스 4개", algo_up,
          "ros2 run packing_algo algo_service_node 를 먼저 띄운다", warn_only=True)
    topics = sh("ros2 topic list 2>/dev/null")
    check("/zone_b/status", "/zone_b/status" in topics,
          "orchestrator 가 떠 있어야 한다", warn_only=True)
    for s in ("/db/get_box_info", "/conveyor/set_velocity"):
        check(f"{s} (없어도 리허설은 진행됨)", s in svc,
              "트랙 D/A 대기 — orchestrator 가 건너뛴다", warn_only=True)

    print("\n────────────────────────────────────────────────")
    if fails:
        print(f" {NG} 막는 문제 {len(fails)}개: " + ", ".join(fails))
        sys.exit(1)
    print(f" {OK} 리허설 진행 가능\n")


if __name__ == "__main__":
    main()
