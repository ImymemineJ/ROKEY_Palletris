# 관제탑 → 중재자·모니터링. 2 Hz (CLAUDE.md §3-2)
string  zone           # "a"|"b"|"c"
string  state          # IDLE|FETCHING|PLANNING|PICKING|PLACING|UPDATING|DONE
int32   queue_len
int32   placed_count
int32   rejected_count
float32 fill_rate      # ★ 분모는 max_stack 이 아니라 실제 최고높이 (함정: 충전율이 낮게 나옴)
int32   max_height_mm
bool    accepting      # ★ false = 더 보내지 마라. 중재자는 이 칸만 본다
builtin_interfaces/Time stamp
