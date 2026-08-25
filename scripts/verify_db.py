#!/usr/bin/env python3
"""db_service_node 기능 검증 — 서비스 7종을 실제 흐름대로 호출한다.

    ros2 run packing_db gen_master_db          # 정답지 깔기
    ros2 run packing_db db_service_node &      # 노드 띄우기
    python3 scripts/verify_db.py               # 이 스크립트

박스 하나가 등록 → 검증 → 분기 → 집기 → 적재까지 가는 동안 각 서비스가
무엇을 돌려주는지 확인한다. 실패 경로(모르는 status · 미등록 박스 · 남의 QR)도
같이 친다 — 성공 경로만 보면 "조용히 잘못 도는" 버그를 못 잡는다.
"""
import sys
import rclpy
from rclpy.node import Node
from packing_msgs.msg import PlacePose
from packing_msgs.srv import (DiverterRequest, GetBoxInfo, MarkPlaced, RegisterBox,
                              SetStatus, VerifyBox, ZoneQueue)

ok_all = True
def chk(label, cond, detail=""):
    global ok_all
    ok_all = ok_all and cond
    print(f"  {'✓' if cond else '✗ 실패'} {label}{(' — ' + detail) if detail else ''}")

rclpy.init()
n = Node("db_smoke")
def call(t, name, req):
    c = n.create_client(t, name)
    if not c.wait_for_service(timeout_sec=10.0):
        print(f"  ✗ 서비스 없음: {name}"); sys.exit(1)
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)   # 테스트 클라이언트라 허용
    return f.result()

BOX = "BOX00000"
# ★ QR 페이로드는 운송장번호 하나뿐이다 (packing_vision/label.py 규약).
QR = "BOX00000"

print("① register_box — 4호(410x310x280) 등록")
# ★ dest 를 비워 보낸다. 인식 노드가 실제로 그렇게 보낸다 — QR 에 구역 글자가
#   없으므로, 구역은 DB 가 주소(boxes_master.address)의 동으로 정해야 한다.
r = call(RegisterBox, "/db/register_box", RegisterBox.Request(
    box_id=BOX, size_w_mm=410, size_d_mm=310, size_h_mm=280,
    color="red", dest="", qr_payload=QR))
chk("ok", r.ok, r.message)

print("①-2 구역이 배송지에서 정해졌나 (설계결정서 §5-3 · 미결 #9)")
chk("응답이 구역과 동을 함께 알려 준다",
    r.message.startswith("zone=") and "동 " in r.message, r.message)

print("② get_box_info — weight_g 가 치수에서 채워졌나")
r = call(GetBoxInfo, "/db/get_box_info", GetBoxInfo.Request(box_id=BOX))
chk("found", r.found)
chk("weight_g=5000 (4호)", r.info.weight_g == 5000, str(r.info.weight_g))
chk("status=SCANNED", r.info.status == "SCANNED", r.info.status)
chk("color/qr 보존", (r.info.color, r.info.qr_payload) == ("red", QR),
    f"color={r.info.color!r} qr={r.info.qr_payload!r}")
# ★ dest 는 "인식이 보내온 값" 칸이다. 인식이 구역을 모르므로 비어 있는 것이 맞다.
#   실제 배정 구역은 boxes_sorted.zone 이고 ⑤ 에서 확인한다.
chk("dest 는 비어 있다 (구역은 주소가 정한다)", r.info.dest == "", repr(r.info.dest))

print("③ verify_box — 같은 QR 이면 match")
r = call(VerifyBox, "/db/verify_box", VerifyBox.Request(box_id=BOX, qr_payload=QR))
chk("match", r.match, f"registered_id={r.registered_id}")

print("④ verify_box — 남의 QR 이면 mismatch")
r = call(VerifyBox, "/db/verify_box", VerifyBox.Request(box_id="BOX00001", qr_payload=QR))
chk("match=False", not r.match, f"registered_id={r.registered_id}")

print("⑤ diverter — 등록된 박스는 B 로")
r = call(DiverterRequest, "/diverter/request", DiverterRequest.Request(box_id=BOX))
chk("target=B", r.target == "B", f"reason={r.reason}")

print("⑥ diverter — 미등록 박스는 RETURN + retry 증가")
r = call(DiverterRequest, "/diverter/request", DiverterRequest.Request(box_id="BOX00009"))
chk("target=RETURN", r.target == "RETURN", f"reason={r.reason}")

print("⑦ set_status — PICKED")
r = call(SetStatus, "/db/set_status", SetStatus.Request(box_id=BOX, status="PICKED", reason=""))
chk("ok", r.ok)

print("⑧ set_status — 모르는 status 는 거부")
r = call(SetStatus, "/db/set_status", SetStatus.Request(box_id=BOX, status="NONSENSE", reason=""))
chk("ok=False", not r.ok)

print("⑨ zone_queue — PICKED 1개가 잡히나")
r = call(ZoneQueue, "/db/zone_queue", ZoneQueue.Request(zone="B"))
chk("count=1", r.count == 1, f"count={r.count} placed={r.placed_count}")

print("⑩ mark_placed — 실측 pose 저장")
r = call(MarkPlaced, "/db/mark_placed", MarkPlaced.Request(
    box_id=BOX, zone="B", actual_pose=PlacePose(x_mm=615, y_mm=402, z_mm=0, yaw_deg=90.0)))
chk("ok", r.ok)

print("⑪ zone_queue — 이제 placed 1")
r = call(ZoneQueue, "/db/zone_queue", ZoneQueue.Request(zone="B"))
chk("count=0 placed=1", r.count == 0 and r.placed_count == 1, f"count={r.count} placed={r.placed_count}")

print("⑫ mark_placed — 미등록 박스는 거부")
r = call(MarkPlaced, "/db/mark_placed", MarkPlaced.Request(
    box_id="BOX00014", zone="B", actual_pose=PlacePose()))
chk("ok=False", not r.ok)

n.destroy_node(); rclpy.shutdown()
print("\n" + ("전부 통과" if ok_all else "★ 실패 있음"))
sys.exit(0 if ok_all else 1)
