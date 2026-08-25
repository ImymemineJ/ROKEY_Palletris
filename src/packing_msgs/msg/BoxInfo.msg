# DB 가 들고 있는 박스 1개의 전부 (CLAUDE.md §3-1)
string  box_id
int32   size_w_mm
int32   size_d_mm
int32   size_h_mm
int32   weight_g
string  color
string  dest           # QR 목적지 "A"|"B"|"C"
string  qr_payload
int32   retry_count    # Return line 재순환 횟수
string  status         # SPAWNED|SCANNED|VERIFIED|DIVERTED|PICKED|PLACED|PICK_FAILED|REJECTED
