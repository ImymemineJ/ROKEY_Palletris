# scripts/

| 파일 | 누가 | 언제 |
|---|---|---|
| `fastdds_whitelist.xml` | 전원 | clone 직후 `~/.ros/` 로 복사 (루트 README 참고) |
| `sync_scene.sh` | 김학민 | 씬 변경 시 |
| `_paths.py` | — | 저장소 경로 해결. 아래 스크립트가 전부 import 한다 |
| `import_h2017_usd.py` | 김병직 | **`~/isaac_assets/h2017.usd` 생성.** USD 는 커밋 못 하므로 **만드는 스크립트**를 커밋한다 |
| `verify_drive_hold.py` | 김병직 | 드라이브 게인 실측 — 팔이 중력 아래 자세를 버티는가 |
| `verify_reach.py` | 김병직 | **G1 책상판 — 완료.** `python3 scripts/verify_reach.py` (씬 없이 돈다) |
| `verify_reach_isaac.py` | 김병직 | G1 Isaac 판 (Day 2, A3 이후) — **아직 없음** |
| `check_place_plan.py` | 김병직 | 알고리즘 배치 계획 프리플라이트 (`plan.json`) |
| `verify_spheres.py` | 김병직 | Lula 충돌 구체가 메시를 덮는지 |
| `gen_collision_spheres.py` | 김병직 | 충돌 메시에서 구체 재유도 (`--update`) |
| `read_log.py` | 김병직 | Isaac Script Editor 출력을 터미널에서 읽는다 |
| `gen_master_db.py` | 민범진 | D1 — **직접 추가할 것** |

게이팅 검증 스크립트(G1~G7)는 전부 여기에 커밋한다. (작업명세서 §11)
