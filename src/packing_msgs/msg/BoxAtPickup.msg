# A → B
bool                 present    # false = 스토퍼 자리가 비었음 (B 는 대기 상태로)
string               box_id
geometry_msgs/Pose   pose       # 월드 좌표, m. ★ 박스 중심
