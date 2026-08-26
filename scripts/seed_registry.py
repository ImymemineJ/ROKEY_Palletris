#!/usr/bin/env python3
"""cam1 대신 박스를 **미리 등록**한다 (임시 대역).

    ros2 run 없이 그냥:  python3 scripts/seed_registry.py

## 왜 필요한가

`/db/get_box_info` 는 `boxes_sorted` 를 본다. 그 표를 채우는 것은 `RegisterBox`
하나뿐이고, 그걸 부르는 것은 **cam1**(`line_detector`)이다. 씬에 카메라 prim 이
아직 하나도 없어서 `vision:=false` 로 도는 지금은 **아무도 등록하지 않는다.**

그러면 관제탑이 박스마다 이렇게 버린다 (실측 2026-08-26):

    [orchestrator.c] DB에 없음: BOX002 → 폐기
    [spawner] despawn BOX002 (라인 밖으로 이동)

`boxes_master` 로 대신할 수도 없다 — 거기에는 주소·구역만 있고 **치수가 없다.**

## 무엇을 하는가

`sim_main._Feeder` 와 **똑같은 순서**로 박스 종류를 뽑아 `/db/register_box` 를
부른다. 쓰기는 그대로 `db_service_node` 가 하므로 "유일한 writer" 규약(§8)을
깨지 않는다.

★ 같은 시드·같은 분포를 쓰므로 급송이 실제로 내보내는 종류와 **일치한다.**
  한쪽만 바꾸면 치수가 어긋나 로봇이 엉뚱한 높이에서 흡착한다.

★ 이것은 `_GtPick` 이 픽업 카메라를 대신하는 것과 같은 성격의 **대역**이다.
  cam1 이 들어오면 이 스크립트는 필요 없어진다.
"""
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/packing_common"))
from packing_common.config import load                        # noqa: E402
from packing_msgs.srv import RegisterBox                      # noqa: E402

TIMEOUT_S = 5.0


def manifest(cfg, n):
    """`_Feeder.__init__` 과 **같은 식**으로 종류를 뽑는다. 바꾸면 양쪽 다 바꿀 것."""
    bx = cfg["boxes"]
    rng = np.random.default_rng(int(bx.get("random_seed", 42)))
    pr = np.array(bx["size_distribution"], dtype=float)
    pr = pr / pr.sum()
    idx = rng.choice(len(bx["types"]), n, p=pr)
    return [bx["types"][int(i)] for i in idx]


# ★ `BoxInfo.dest` 는 계약상 **"A"|"B"|"C"** 다 (CLAUDE.md §3-1).
#   `by_dest` 의 도시명(SEOUL/BUSAN/DAEJEON)은 **컨베이어 분기 표식**이라 성격이 다르다.
#   도시명을 넣으면 db_service_node 가 주소 기준 구역과 비교해 150 번 경고한다:
#       "BOX001 인식이 보낸 dest BUSAN 이 주소 기준 구역 B 와 다르다"


def main():
    import sqlite3

    cfg = load()
    ws = Path(__file__).resolve().parents[1]
    db = sqlite3.connect("file:%s?mode=ro" % (ws / "palletris.db"), uri=True)
    master = {b: (z, a) for b, z, a in
              db.execute("SELECT box_id, zone, address FROM boxes_master")}
    db.close()
    if not master:
        raise SystemExit("boxes_master 가 비었다 — ros2 run packing_db gen_master_db 먼저")

    n = len(master)
    types = manifest(cfg, n)
    colors = (cfg.get("routing") or {}).get("by_color") or {}
    zone_color = {str(z).upper(): c for c, z in colors.items()}

    rclpy.init()
    node = Node("seed_registry")
    cli = node.create_client(RegisterBox, "/db/register_box")
    if not cli.wait_for_service(timeout_sec=TIMEOUT_S):
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("/db/register_box 서버가 없다 — bringup 을 먼저 띄울 것")

    ok = fail = 0
    # ★ box_id 순서가 `_Feeder` 의 스폰 순서(BOX001, BOX002, ...)와 같아야
    #   i 번째 종류가 그 박스의 종류가 된다.
    for i, box_id in enumerate(sorted(master)):
        t = types[i]
        zone = master[box_id][0]
        req = RegisterBox.Request()
        req.box_id = box_id
        req.size_w_mm = int(t["w_mm"])
        req.size_d_mm = int(t["d_mm"])
        req.size_h_mm = int(t["h_mm"])
        req.color = zone_color.get(str(zone).upper(), "")
        req.dest = str(zone).upper()          # ★ 구역 문자다 (§3-1)
        req.qr_payload = box_id            # QR 은 운송장번호만 싣는다 (gen_qr_labels 주석)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=TIMEOUT_S)
        # ★ 여기서는 `spin_until_future_complete` 를 써도 된다. 콜백 안이 아니라
        #   **독립 실행 스크립트**의 메인 루프다 (CLAUDE.md §9 의 금지는 콜백 안 얘기).
        res = fut.result()
        if res is not None and res.ok:
            ok += 1
        else:
            fail += 1
            if fail <= 5:
                node.get_logger().warn(
                    "%s 등록 실패: %s" % (box_id, getattr(res, "message", "응답 없음")))

    print("등록 %d 건 · 실패 %d 건" % (ok, fail))
    print("  종류 표본:", ", ".join("%s=%s" % (b, types[i]["name"])
                                  for i, b in enumerate(sorted(master)[:5])))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
