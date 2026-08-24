# 로봇 → orchestrator.  명령 처리 결과.
# ★ 반드시 seq 를 되돌려줄 것. 안 그러면 어느 명령의 결과인지 알 수 없다.

uint32  seq          # RobotCmd.seq 를 그대로 복사
uint8   action       # RobotCmd.action 을 그대로 복사
string  box_id

bool    ok
string  reason       # "GRASP_FAILED"|"IK_UNSOLVABLE"|"COLLISION"|
                     # "DROPPED"|"TIMEOUT"|"ABORTED" / 성공이면 ""

# ── 실측 결과 (PLACE 성공 시에만 유효) ──
# ★ heightmap 갱신에 이 값이 들어간다. 계획값을 되보내면 오차가 누적돼
#   알고리즘은 840mm 로 끝났다고 믿는데 실제 더미가 850mm 를 넘는다.
bool    has_actual
int32   actual_x_mm
int32   actual_y_mm
int32   actual_z_mm      # 밑면 기준
float32 actual_yaw_deg

float32 duration_s
