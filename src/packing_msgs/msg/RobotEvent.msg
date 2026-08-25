# 로봇 → 관제탑 (CLAUDE.md §3-2)
uint32  seq          # RobotCmd.seq 를 그대로 복사
uint8   action
string  box_id

bool    ok
string  reason       # "GRASP_FAILED"|"IK_UNSOLVABLE"|"COLLISION"|
                     # "DROPPED"|"TIMEOUT"|"ABORTED" / 성공이면 ""

# ── 실측 결과 (PLACE 성공 시에만 유효) ──
# ★ has_actual == true 일 때만 actual_* 를 UpdateHeightmap 에 넣는다 (§3-4 규약 3번).
#   계획값을 되보내면 오차가 누적돼 알고리즘은 1300 안이라 믿는데 실제 더미가 넘는다.
bool    has_actual
int32   actual_x_mm
int32   actual_y_mm
int32   actual_z_mm      # 밑면 기준
float32 actual_yaw_deg

float32 duration_s
