"""벨트가 정말 도는지 소켓으로 직접 물어본다. Isaac 이 떠 있는 PC 에서 실행."""
import json, socket, sys, time
for _ in range(20):                       # Isaac 이 뜨는 중일 수 있으니 몇 번 시도한다
    try:
        s = socket.create_connection(("127.0.0.1", 8765), timeout=3); break
    except OSError:
        time.sleep(0.5)
else:
    sys.exit("★ 8765 에 못 붙었다 — sim_main.py 가 떠 있는지 확인하세요")
f = s.makefile("rw")
def call(**r):
    f.write(json.dumps(r) + "\n"); f.flush(); return json.loads(f.readline())
print("ping        :", call(cmd="ping"))
for seg in ("main", "b_pickup"):
    print(f"{seg:<12}:", call(cmd="set_velocity", seg=seg, mps=0.5))
print("spawn_box   :", call(cmd="spawn_box", box_id="TEST00", type="3호", yaw_deg=0.0))
print("\n박스가 벨트를 타는지 Isaac 화면을 5초쯤 보세요. 그다음 Enter.")
input()
print("pickup_check:", call(cmd="pickup_check", zone="B"))
print("get_box_pose:", call(cmd="get_box_pose", box_id="TEST00"))
