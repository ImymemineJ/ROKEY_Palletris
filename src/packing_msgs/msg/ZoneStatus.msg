# orchestrator → 모니터링 · 중재자(2차).  2 Hz 발행.

string  zone           # "a"|"b"|"c"
string  state          # IDLE|FETCHING|PLANNING|PICKING|PLACING|UPDATING|DONE

int32   queue_len
int32   placed_count
int32   rejected_count

float32 fill_rate      # 분모는 max_stack 이 아니라 실제 최고높이
int32   max_height_mm

bool    accepting      # ★ false = 더 보내지 마라 (PALLET_FULL 등)
                       #   2차 중재자가 이 한 칸만 보면 판단이 끝난다
builtin_interfaces/Time stamp
