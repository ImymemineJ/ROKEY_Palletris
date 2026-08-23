# scripts/

| 파일 | 누가 | 언제 |
|---|---|---|
| `fastdds_whitelist.xml` | 전원 | clone 직후 `~/.ros/` 로 복사 (루트 README 참고) |
| `sync_scene.sh` | 김학민 | 씬 변경 시 |
| `_paths.py` | — | 저장소 경로 해결. 아래 스크립트가 전부 import 한다 |
| `import_h2017_usd.py` | 김병직 | **`~/isaac_assets/h2017.usd` 생성.** USD 는 커밋 못 하므로 **만드는 스크립트**를 커밋한다 |
| `verify_drive_hold.py` | 김병직 | 드라이브 게인 실측 — 팔이 중력 아래 자세를 버티는가 |
| `verify_reach.py` | 김병직 | **G1 책상판 — 완료.** `python3 scripts/verify_reach.py` (씬 없이 돈다). 11점 정의의 출처 — Isaac 판이 이걸 import 한다 |
| `verify_lula_ik.py` | 김병직 | **B1 — Lula 기동.** IK 한 점. Isaac 배선(`bring_up`)이 여기 한 군데에만 있다 |
| `verify_reach_isaac.py` | 김병직 | **G1 Isaac 판 + G5.** 11점을 Lula 로. 씬 전체 충돌은 A3 대기 |
| `check_place_plan.py` | 김병직 | 알고리즘 배치 계획 프리플라이트 (`plan.json`) |
| `verify_spheres.py` | 김병직 | Lula 충돌 구체가 메시를 덮는지 |
| `gen_collision_spheres.py` | 김병직 | 충돌 메시에서 구체 재유도 (`--update`) |
| `gen_reach_diagram.py` | 김병직 | **`docs/가이드/img/reach_side.svg` 를 system.yaml 에서 생성.** 손으로 그린 축척 도면이라 페데스탈·max_stack 이 바뀔 때마다 조용히 틀려졌다. 값이 바뀌면 다시 돌릴 것 |
| `build_gripper_usd.py` | 김병직 | **흡착 그리퍼를 h2017.usd 에 붙여 `h2017_suction.usd` 생성 (B3 전제).** URDF 에 그리퍼가 없고 USD 는 커밋 못 하므로 **만드는 스크립트**를 커밋한다. `--check` 로 scale·흡착점 검증 |
| `verify_suction.py` | 김병직 | **B3 — 흡착 판정.** grasp/release/is_holding 을 실물리로. `measure` 가 흡착면↔상판 간극(mm)을 재는 참조 구현 — 통합 시 `gap_fn` 배선이 이걸 베낀다 |
| `verify_cycle.py` | 김병직 | **시뮬레이션 A — 사이클 예행.** 픽→이송→플레이스를 계획대로 돌리며 사이클타임·툴 기울기·박스↔팔 간섭·이송 여유를 잰다. `--gui` 면 팔레트/벨트/박스를 **시각 전용**으로 띄운다 (물리 없음, 측정 무영향). `--transit line --yaw fixed` 로 옛 방식 재현 |
| `verify_b4_cycle.py` | 김병직 | **B4 — 단일 사이클 판정.** 소켓 프로토콜 위에서 픽→플레이스 1개 완주. 내장 StandInServer 가 A `sim_server` 의 대역이라 **명령 프로토콜의 참조 구현**을 겸한다 |
| `demo_stack15.py` | 김병직 | **B5 예행 — 15박스 다층(7단) 적재 데모.** HeightmapPacker 계획 + 로봇 기하를 아는 끼워넣기 순서. 촬영은 `--gui --hold`, `--kit-log` 로 런타임 로그 억제, `--plan-mode online` 은 비전 모의(현재 오프라인보다 낮음) |
| `script_editor_lula.py` | 김병직 | **Isaac Sim GUI 의 Script Editor 에 붙여넣는 스니펫.** `POINT` 만 바꿔 11점 중 하나로 팔을 보낸다 |
| `read_log.py` | 김병직 | Isaac Script Editor 출력을 터미널에서 읽는다 |
| `gen_master_db.py` | 민범진 | D1 — **직접 추가할 것** |

게이팅 검증 스크립트(G1~G7)는 전부 여기에 커밋한다. (작업명세서 §11)
