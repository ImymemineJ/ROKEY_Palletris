# 관제탑 → 로봇 (CLAUDE.md §3-2)
#
# 왜 서비스가 아니라 토픽인가: 모션이 5~10초 걸린다. 서비스로 묶으면 그동안
# 응답을 붙들고 있어야 하고 타임아웃 관리가 복잡해진다.
uint8 PICK  = 0
uint8 PLACE = 1
uint8 HOME  = 2
uint8 STOP  = 3
uint8   action

string  box_id
uint32  seq          # ★ 명령 일련번호. event 가 이 값을 되돌려줘야
                     #   "지금 온 결과가 방금 그 명령의 것"임을 보장한다.

int32   x_mm         # 박스 중심
int32   y_mm
int32   z_mm         # ★ 박스 밑면
float32 yaw_deg

int32   size_w_mm
int32   size_d_mm
int32   size_h_mm

int32   approach_z_mm   # 수직 하강 가능 높이. 0 이면 로봇 재량
