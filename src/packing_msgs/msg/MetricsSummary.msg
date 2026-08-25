# db_service_node → 모니터링.  1 Hz.
# ★ JSON 문자열이 아니라 전용 타입이다 (CLAUDE.md §11).
#   트랙 D 가이드 D3 은 std_msgs/String 에 JSON 을 실으라고 했지만 그 방식은
#   §11 에서 폐기됐다. 필드로 내면 rqt_plot 이 바로 그린다.
#
# ★ 지표의 유일한 출처는 palletris.db 다 (db_service_node 가 유일한 reader/writer).
#   fill_rate·max_height_mm 만 예외로 zone status 토픽을 중계한다 —
#   그 둘은 heightmap 을 가진 algo 만 알 수 있다.
#
# ⚠ .msg 주석에 별표+슬래시(C 주석 종료 기호)를 쓰지 말 것.
#   이 주석은 생성된 C 헤더에 블록 주석으로 그대로 복사된다. 그래서
#   "/zone_" + 별표 + "/status" 라고 적었더니 주석이 거기서 끝나 버리고
#   뒤 한글이 C 코드로 파싱돼 packing_msgs 빌드가 통째로 깨졌다.
#   토픽을 예로 들 때는 /zone_b/status 처럼 실제 이름을 쓴다.

builtin_interfaces/Time stamp
float32 elapsed_s          # 첫 이벤트부터 지금까지

# ── 개수 ──
int32   total_boxes        # boxes_master. ★ 인식률의 분모다.
                           #   boxes_sorted 를 분모로 쓰면 등록된 박스만 세게 되어
                           #   인식률이 언제나 100 % 로 나온다 (작업명세서 §8.6 함정)
int32   scanned_count      # SCANNED 에 도달한 박스
int32   placed_count
int32   pick_failed_count
int32   rejected_count
int32   queue_len          # 해당 구역 대기 (DIVERTED·PICKED)

# ── 비율 (분모가 0 이면 -1.0. 0.0 과 "잴 수 없음"은 다르다) ──
float32 recognition_rate     # scanned / total_boxes
float32 place_success_rate   # placed / (placed + pick_failed)
float32 cycle_avg_s          # AVG(placed_at - picked_at)
float32 robot_idle_rate      # 1 − (PICKED→PLACED 시간 합 / elapsed_s)

# ── algo 만 아는 값. zone status 토픽을 중계한다 ──
float32 fill_rate
int32   max_height_mm
string  zone            # 중계해 온 구역. 예: "b"
