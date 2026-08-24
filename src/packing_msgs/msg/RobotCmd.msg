# orchestrator → 로봇.  로봇에게 내리는 단일 명령.
# ★ 이 토픽을 발행하는 노드는 orchestrator 하나뿐이다.
#
# 왜 서비스가 아니라 토픽인가:
#   집기·놓기 모션은 5~10초 걸린다. 서비스로 묶으면 그동안 응답을 붙들고
#   있어야 하고 타임아웃 관리가 복잡해진다.
#   명령은 토픽으로 던지고 결과는 /robot/event 로 받는다.

uint8 PICK   = 0
uint8 PLACE  = 1
uint8 HOME   = 2
uint8 STOP   = 3
uint8 action

string  box_id       # HOME/STOP 이면 ""
uint32  seq          # ★ 명령 일련번호. event 가 이 값을 되돌려줘야
                     #   "지금 온 결과가 방금 그 명령의 것"임을 보장한다.

# 단위는 mm 정수 · 도(deg). packing_common.units 로만 변환할 것.
int32   x_mm         # 박스 중심
int32   y_mm         # 박스 중심
int32   z_mm         # ★ 박스 밑면. 중심으로 쓰면 팔레트에 파묻힌다
float32 yaw_deg      # 0 또는 90

int32   size_w_mm
int32   size_d_mm
int32   size_h_mm

int32   approach_z_mm   # 수직 하강 가능 높이. 0 이면 로봇 재량
