#!/usr/bin/env python3
"""RobotCmd → `Cycle` → RobotEvent. **ROS 도 소켓도 모른다** (트랙 B 김병직).

왜 따로 뺐나: 이 어댑터가 살 자리가 두 군데다.
  · 지금(과도기)  `robot_node.py` — Isaac 밖 rclpy 노드, 소켓으로 Isaac 에 붙는다
  · 최종          `sim_main.py` — Isaac 안. Action Graph 가 `/robot/cmd` 를 받아
                  `ros_graph.take_cmd()` 로 넘겨주면 여기를 부르고
                  `ros_graph.send_event(**결과)` 로 답한다 (CLAUDE.md §2)
전송이 바뀌어도 **규칙은 안 바뀐다.** 두 번 유도하면 한쪽이 반드시 틀린다.

입출력은 순수 dict 다:
    ev = run_cmd(cycle, cmd_dict, log)      # cmd_dict: RobotCmd 필드 그대로
    # ev: RobotEvent 필드 그대로

## ★ 여기서 지키는 규약 3개 (CLAUDE.md §3-4)

1. `seq` 를 **그대로 복사**한다. 안 하면 타임아웃 뒤 늦게 온 답이 다음 박스의
   결과로 둔갑한다 — 에러도 안 나고 엉뚱하게 도는 버그다.
2. `has_actual` 은 **PLACE 성공 + 씬에서 실제로 읽었을 때만** true.
   `Cycle.place()` 는 못 읽으면 **계획값**을 돌려주므로 그걸 그대로 실으면 안 된다.
   계획값을 되보내면 오차가 누적돼 알고리즘은 1300 안이라 믿는데 실물이 넘는다.
3. **무슨 일이 있어도 답한다.** 예외로 조용히 죽으면 관제탑이 20초를 통째로 버린다.

## ★ Cycle 은 구역당 하나다. 명령마다 새로 만들면 안 된다

`_stack_top_m`(이송 고도의 바닥 — 0 이 되면 팔이 적재물 위를 낮게 지나 스택을
친다, §10-3) · `_obstacles` · `_pick_yaw` · `_pick_c/_pick_top` 이 PICK 과 PLACE
사이를 넘어간다. 부르는 쪽이 `Cycle` 을 들고 있어야 한다.
"""
import time

# RobotCmd.msg 의 상수와 같아야 한다. (Isaac 안에서는 packing_msgs 를 import 할 수
# 없을 수 있어 숫자로 둔다 — .msg 가 바뀌면 여기도 바꾼다.)
PICK, PLACE, HOME, STOP = 0, 1, 2, 3
ACTION_NAME = {PICK: "PICK", PLACE: "PLACE", HOME: "HOME", STOP: "STOP"}


def run_cmd(cycle, cmd, log=print):
    """명령 하나를 수행하고 RobotEvent 필드를 dict 로 돌려준다.

    Args:
        cycle: `Cycle` 인스턴스. **구역당 하나를 계속 재사용한다** (위 주석).
        cmd:   RobotCmd 필드 dict — action · seq · box_id · x_mm · y_mm · z_mm ·
               yaw_deg · size_w_mm · size_d_mm · size_h_mm
        log:   한 줄 로거

    Returns:
        RobotEvent 필드 dict. 예외를 밖으로 내보내지 않는다 (규약 3).
    """
    # ★ import 가 두 갈래다. Isaac 안에서는 colcon 설치본(파이썬 3.12)을 못 보고
    #   `_paths.py` 가 `src/packing_robot/packing_robot` 를 sys.path 에 붙여
    #   **평면 import** 가 된다 (`demo_stack15.py` 도 `import cycle` 로 쓴다).
    try:
        from packing_robot.cycle import MotionError
    except ImportError:                                   # Isaac 안 (평면 경로)
        from cycle import MotionError

    t0 = time.time()
    action = int(cmd.get("action", -1))
    seq = int(cmd.get("seq", 0))
    box_id = str(cmd.get("box_id", ""))
    name = ACTION_NAME.get(action, f"?{action}")

    ev = {"seq": seq, "action": action, "box_id": box_id,      # ★ seq 그대로 복사
          "ok": False, "reason": "ABORTED",
          "has_actual": False,
          "actual_x_mm": 0, "actual_y_mm": 0, "actual_z_mm": 0, "actual_yaw_deg": 0.0,
          "duration_s": 0.0}

    if action not in ACTION_NAME:
        # ★ 조용히 무시하지 않는다. 답이 없으면 관제탑은 20초를 버린 뒤에야 안다.
        log(f"[adapter] 모르는 action={action} seq={seq}")
        ev["duration_s"] = time.time() - t0
        return ev

    log(f"[adapter] ← {name} seq={seq} {box_id} "
        f"({cmd.get('x_mm')}, {cmd.get('y_mm')}, {cmd.get('z_mm')}) "
        f"yaw={float(cmd.get('yaw_deg', 0.0)):.1f}")

    try:
        if action == PICK:
            h = int(cmd["size_h_mm"])
            # ★ RobotCmd.z_mm 은 **박스 밑면**이다 (RobotCmd.msg 정본).
            #   `Cycle.pick` 은 박스 **중심**을 받으므로 h/2 를 더한다.
            #   ⚠ 이 한 줄이 어긋나면 팔이 h/2 만큼 빗나간 높이로 내려간다.
            pose = {"pos_m": [int(cmd["x_mm"]) / 1000.0,
                              int(cmd["y_mm"]) / 1000.0,
                              (int(cmd["z_mm"]) + h / 2.0) / 1000.0],
                    "yaw_deg": float(cmd.get("yaw_deg", 0.0))}
            r = cycle.pick(box_id, pose, h)
            ev["ok"] = (r == "PICKED")
            ev["reason"] = "" if ev["ok"] else "GRASP_FAILED"

        elif action == PLACE:
            # ★ PLACE 의 좌표는 **팔레트 기준**이다 (PlacePose 를 그대로 실어 보낸다).
            #   z 는 밑면. `pallet_place_to_world` 가 그 규약으로 월드 중심을 만든다.
            place = {"x_mm": int(cmd["x_mm"]), "y_mm": int(cmd["y_mm"]),
                     "z_mm": int(cmd["z_mm"]), "yaw_deg": float(cmd.get("yaw_deg", 0.0))}
            r, actual, measured = cycle.place(box_id, place, int(cmd["size_w_mm"]),
                                              int(cmd["size_d_mm"]), int(cmd["size_h_mm"]))
            if r != "PLACED":
                ev["reason"] = "DROPPED"
            else:
                ev["ok"], ev["reason"] = True, ""
                # ★ 규약 2 — 씬에서 **실제로 읽은** 경우에만 싣는다.
                if measured:
                    ev["has_actual"] = True
                    ev["actual_x_mm"] = int(actual["x_mm"])
                    ev["actual_y_mm"] = int(actual["y_mm"])
                    ev["actual_z_mm"] = int(actual["z_mm"])        # 밑면 기준
                    ev["actual_yaw_deg"] = float(actual.get("yaw_deg",
                                                            cmd.get("yaw_deg", 0.0)))
                else:
                    log(f"[adapter] {box_id} 실측 pose 를 못 읽었다 "
                        f"— has_actual=false 로 보낸다 (계획값을 되보내지 않는다)")

        elif action == HOME:
            # ★ 진짜 홈 관절값이 아무 데도 없다 — 적재물 위로 팔만 뺀다.
            #   `Cycle.retreat_home()` 주석에 왜인지 적어 뒀다.
            cycle.retreat_home()
            ev["ok"], ev["reason"] = True, ""

        else:   # STOP
            # `robot_motion.handlers()` 에 stop 이 없다 (move_ee · motion_status ·
            # grasp · release · is_holding · tcp_speed 여섯뿐). 없는 API 를 부르지 않는다.
            # 조용히 ok 를 주면 "멈췄다"고 오해하게 만든다.
            log(f"[adapter] STOP seq={seq} — 아직 구현되지 않았다 "
                f"(robot_motion 에 핸들러 없음)")

    except MotionError as e:
        ev["reason"] = "IK_UNSOLVABLE" if e.state == "IK_FAIL" else "COLLISION"
        log(f"[adapter] {name} seq={seq} 이동 실패 {e.state}")
    except Exception as e:                       # noqa: BLE001 — 규약 3
        ev["reason"] = "ABORTED"
        log(f"[adapter] {name} seq={seq} 예외 {e!r}")

    ev["duration_s"] = float(time.time() - t0)
    log(f"[adapter] → {name} seq={seq} ok={ev['ok']}"
        + (f" {ev['reason']}" if ev["reason"] else "")
        + (f" 실측({ev['actual_x_mm']},{ev['actual_y_mm']},{ev['actual_z_mm']})"
           if ev["has_actual"] else "")
        + f" {ev['duration_s']:.1f}s")
    return ev
