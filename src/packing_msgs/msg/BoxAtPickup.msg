# 인식 노드 → 관제탑. 픽업 좌표까지 실어 보낸다 (CLAUDE.md §3-2)
#
# ★ geometry_msgs/Pose 를 쓰지 않는다. 단위 규칙이 "설정·DB = mm 정수" 인데
#   Pose 는 m float 라 경계에서 매번 변환이 끼어들고, 쿼터니언에서 yaw 를 빼는
#   코드가 노드마다 복붙된다.
string  box_id
bool    present      # false = 박스 없음. 무시하고 사이클을 시작하지 말 것
int32   x_mm         # 박스 중심 (pick_cam 실측)
int32   y_mm
float32 yaw_deg
