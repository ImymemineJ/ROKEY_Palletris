# DB 에 저장되는 박스 한 개의 정보.
string  box_id
int32   size_w_mm
int32   size_d_mm
int32   size_h_mm
int32   weight_g
string  color          # 1차 ID 수단 (HSV 판정 결과, 예: "RED")

# ── 2차용. 1차에서는 빈 문자열 / 0 ──
string  dest           # QR 목적지 "A"|"B"|"C". 1차는 ""
string  qr_payload     # QR 원문. 1차는 ""
int32   retry_count    # Return line 재순환 횟수. 1차는 항상 0
string  status         # SPAWNED|SCANNED|VERIFIED|DIVERTED|PICKED|PLACED|PICK_FAILED|REJECTED
