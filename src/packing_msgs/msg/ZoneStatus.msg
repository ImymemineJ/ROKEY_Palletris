string  zone
int32   queue_len          # /db/zone_queue 응답 중계 (§6.7)
int32   buffer_cap
float32 est_free_time_s    # 사이클 잔여 추정 — 중재자 동점 처리에 사용
string  state              # "IDLE"|"BUSY"|"BLOCKED"|"PALLET_FULL"
float32 fill_rate
