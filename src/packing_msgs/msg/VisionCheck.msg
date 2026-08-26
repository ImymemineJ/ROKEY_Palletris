# 인식 노드 → 모니터링·지표.  **RGB 단독 추정과 깊이 실측을 나란히 싣는다.**
#
# ★ 왜 이 타입이 필요한가 — 두 경로가 독립이기 때문이다.
#   RGB 경로는 박스 높이를 **추측**한다. 사양 3종(3호 210 · 4호 280 · 5호 340)의
#   평면에 각각 투영해 치수가 제일 잘 맞는 것을 고른다 (geometry.box_pose_from_corners).
#   종류를 한 칸 틀리면 투영 평면이 60~130 mm 어긋나고, 그러면 **중심 x, y 까지
#   같이 틀어진다** — 광선이 다른 높이에서 끊기기 때문이다. 화면 가장자리일수록 크다.
#   깊이 경로는 그 높이를 **직접 잰다.** 추측이 안 들어간다.
#   둘이 같은 값을 내면 그 자세는 두 번 확인된 것이고, 갈리면 어느 쪽이 틀렸는지가
#   아니라 "믿으면 안 된다"는 것이 먼저 확정된다.
#
# ★ 왜 로그가 아니라 토픽인가
#   로그는 지표가 안 된다. 평가 항목의 "검증 및 분석(정량)"은 숫자를 요구하는데,
#   교차검증 결과가 get_logger().warn 안에만 있으면 발표에서 셀 수가 없다.
#   필드로 내면 rqt_plot 이 바로 그린다 (MetricsSummary 와 같은 이유).
#
# ★ 이 토픽은 **아무도 구독하지 않아도 된다.** 진단 채널이라 로봇 동작 경로에
#   끼어들지 않는다. BoxAtPickup 은 그대로 나가고, 여기는 그 옆에서 관측만 한다.
#
# 주의: .msg 주석에 별표+슬래시(C 주석 종료 기호)를 쓰지 말 것 — 생성된 C 헤더의
#   블록 주석이 거기서 끝나 빌드가 깨진다 (MetricsSummary.msg 참조).

builtin_interfaces/Time stamp

string  source          # "cam1" | "cam2" | "pick_cam"
string  zone            # 구역 인식일 때 "a"|"b"|"c". 라인 인식이면 ""
string  box_id          # 못 읽었으면 ""

# ── ① RGB 단독 (지금까지의 경로) ─────────────────────────────────────────
bool    rgb_ok
string  rgb_type        # "3호" | "4호" | "5호"
int32   rgb_x_mm        # 박스 중심
int32   rgb_y_mm
int32   rgb_top_z_mm    # ★ 윗면 z. 중심 z 가 아니다 — 깊이가 재는 것이 윗면이다
float32 rgb_yaw_deg
float32 rgb_fit_err_mm  # 사양 치수와의 맞춤 오차 (긴변+짧은변)

# ── ② 깊이 실측 (독립 경로) ──────────────────────────────────────────────
bool    depth_ok        # 깊이를 실제로 썼는가. false 면 아래 depth_ 필드는 무의미
string  depth_reason    # depth_ok=false 인 이유. 성공이면 ""
                        # "DISABLED" | "NO_DEPTH_TOPIC" | "STALE" | "SHAPE_MISMATCH"
                        # | "TOO_FEW_PIXELS" | "OUT_OF_RANGE" | "NOT_FLAT"
string  depth_type      # 깊이 평면에서 다시 잰 치수로 고른 종류
int32   depth_x_mm
int32   depth_y_mm
int32   depth_top_z_mm  # 깊이가 잰 윗면 z (월드)
float32 depth_yaw_deg
float32 depth_h_mm      # 잰 윗면 z − 벨트 상면 = 박스 높이 실측
float32 depth_spread_mm # 윗면 깊이 산포. 크면 평면이 아니다 (박스가 아닐 수 있다)
int32   depth_px_used   # 이상치 제거 후 남은 픽셀 수
float32 depth_valid_ratio  # 폴리곤 안에서 **윗면으로 인정된** 픽셀 비율.
                           # 낮으면 센서 결측이거나 벨트가 섞였다 — 덜 믿어야 한다

# ── ③ 대조 결과 ──────────────────────────────────────────────────────────
bool    agree           # 아래 세 축이 전부 허용 안이고 종류도 같은가
float32 dz_mm           # rgb_top_z − depth_top_z
float32 dxy_mm          # 중심 거리
float32 dyaw_deg        # 0~90 으로 접은 차이
string  verdict         # "OK" | "NO_DEPTH" | "HEIGHT_MISMATCH" | "TYPE_MISMATCH"
                        # | "XY_MISMATCH" | "YAW_MISMATCH" | "NOT_FLAT"
string  used            # 최종 채택 경로: "RGB" | "DEPTH" | "NONE"
