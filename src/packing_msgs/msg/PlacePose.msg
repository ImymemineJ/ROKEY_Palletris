# 박스 하나의 자세.
# ★ x_mm / y_mm 는 박스 "중심", z_mm 은 박스 "밑면"
#   z 를 중심으로 쓰면 팔레트에 절반 파묻힌다.
int32   x_mm
int32   y_mm
int32   z_mm
float32 yaw_deg      # 0 또는 90. 0이면 size_w 가 월드 +X 방향
