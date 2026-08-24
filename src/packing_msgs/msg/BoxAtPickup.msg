# 스토퍼(x=10000)에 박스가 도착했는지.
# 발행: 트랙 D 의 detector_zone_b (천장 RGB). 구독: 트랙 B 의 orchestrator.
# ★ 구독 콜백에서는 판단만 하고 무거운 일을 하지 않는다 (작업명세서 §1.5).
# 작업명세서 §2.1 · §5.6

bool               present   # false = 스토퍼 자리가 비었음 (B 는 대기 상태로 전환)
string             box_id
geometry_msgs/Pose pose      # 월드 좌표, 단위 m. ★ 박스 "중심"
