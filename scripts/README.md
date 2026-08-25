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
| `verify_link_clearance.py` | 김병직 | **팔 링크 대 적재물 간섭 실측 — 특히 `link_2` (issues/18·25).** `--probe` 는 근거리 기둥 높이를 쓸어 한계를 낸다 (`demo_stack15.py:331` 의 `ped-150` 어림값 대체). `--sweep-dy`/`--sweep-ped` 로 기구학 파라미터 감도. 씬 없이 6초 |
| `verify_link_clearance_isaac.py` | 김병직 | **위 스크립트의 Isaac/Lula 판.** `~/isaac-sim-5.1.0/python.sh` 로 돌린다. 기하는 책상판에서 import 하므로 같은 기둥을 본다. 여유를 셋(`desk*`/`lula`/`scene`)으로 내서 **가지 선택**과 **FK 모델**을 따로 검증한다. `--gui --spawn --hold` 로 눈으로 확인 |
| `verify_spheres.py` | 김병직 | Lula 충돌 구체가 메시를 덮는지 |
| `gen_collision_spheres.py` | 김병직 | 충돌 메시에서 구체 재유도 (`--update`) |
| `gen_reach_diagram.py` | 김병직 | **`docs/가이드/img/reach_side.svg` 를 system.yaml 에서 생성.** 손으로 그린 축척 도면이라 페데스탈·max_stack 이 바뀔 때마다 조용히 틀려졌다. 값이 바뀌면 다시 돌릴 것 |
| `build_gripper_usd.py` | 김병직 | **흡착 그리퍼를 h2017.usd 에 붙여 `h2017_suction.usd` 생성 (B3 전제).** URDF 에 그리퍼가 없고 USD 는 커밋 못 하므로 **만드는 스크립트**를 커밋한다. `--check` 로 scale·흡착점 검증 |
| `verify_suction.py` | 김병직 | **B3 — 흡착 판정.** grasp/release/is_holding 을 실물리로. `measure` 가 흡착면↔상판 간극(mm)을 재는 참조 구현 — 통합 시 `gap_fn` 배선이 이걸 베낀다 |
| `verify_cycle.py` | 김병직 | **시뮬레이션 A — 사이클 예행.** 픽→이송→플레이스를 계획대로 돌리며 사이클타임·툴 기울기·박스↔팔 간섭·이송 여유를 잰다. `--gui` 면 팔레트/벨트/박스를 **시각 전용**으로 띄운다 (물리 없음, 측정 무영향). `--transit line --yaw fixed` 로 옛 방식 재현 |
| `verify_b4_cycle.py` | 김병직 | **B4 — 단일 사이클 판정.** 소켓 프로토콜 위에서 픽→플레이스 1개 완주. 내장 StandInServer 가 A `sim_server` 의 대역이라 **명령 프로토콜의 참조 구현**을 겸한다 |
| `demo_stack15.py` | 김병직 | **B5 예행 — 15박스 다층(7단) 적재 데모.** HeightmapPacker 계획 + 로봇 기하를 아는 끼워넣기 순서. 촬영은 `--gui --hold`, `--kit-log` 로 런타임 로그 억제, `--plan-mode online` 은 비전 모의(현재 오프라인보다 낮음) |
| `verify_belt_speed.py` | 김학민 | **벨트 이송속도 게이트.** `isaac-sim.sh --no-window --exec` **전용** — `IsaacConveyor` 가 렌더 틱에 물려 있어 `python.sh` 로는 값이 틀린다. main·b_pickup 을 따로 잰다 (scale.x 가 3 배 달라 한쪽만 맞을 수 있다) |
| `verify_b4_test1_1.py` | 김학민 | **B4 를 실제 씬 `test1-1.usd` 에서.** `verify_b4_cycle` 은 합성 씬이고 이건 진짜 라인이다 — 박스가 벨트를 타고 와 스토퍼에 서고, A 의 `sim_server`/`spawner`/`conveyor_ctl` 과 B 의 `robot_motion` 이 같은 소켓에 붙는다 |
| `demo_stack15_test1_1.py` | 김학민 | **15박스 연속 — 실제 라인.** `demo_stack15`(B5)를 컨베이어 위로 옮긴 것. 급송이 없고 도착 순서 그대로 집으므로 온라인 패커를 쓴다. 스폰은 사양 스케줄(`boxes.stage1` 12 s x 15). `PL_N` 으로 개수 축소 |
| `script_editor_lula.py` | 김병직 | **Isaac Sim GUI 의 Script Editor 에 붙여넣는 스니펫.** `POINT` 만 바꿔 11점 중 하나로 팔을 보낸다 |
| `read_log.py` | 김병직 | Isaac Script Editor 출력을 터미널에서 읽는다 |
| `script_editor_test_scene.py` | 정정인 | **최소 테스트 씬 — 씬 담당 없이 혼자 확인용.** `system.yaml` 숫자만으로 도크 바닥(z=0) · 벨트 두 조각(상면 z=800, 폭 1546) · 스토퍼 · 팔레트 3장을 그린다. 빈 스테이지에 붙여넣고 Ctrl+Enter 한 뒤 `script_editor_qr_boxes.py` 를 이어서 돌린다. ⚠ **껍데기다** — 컨베이어 구동 · 로봇 · 카메라 · tf · Action Graph 는 없다(트랙 A 소관). `PUSH_MPS` 로 박스에 +X 초기속도만 줄 수 있다 |
| `script_editor_qr_boxes.py` | 정정인 → **김학민** | **QR 텍스처 박스를 사양대로 벨트에 스폰.** Script Editor 에 붙여넣고 Ctrl+Enter. 박스 3종·혼입비·질량·스폰지점(500,0,810)·지터(y ±150 · yaw ±15)를 **전부 `system.yaml` 에서 읽는다**. MDL 비동기 함정을 피해 `UsdPreviewSurface` 를 직접 조립하고, 리지드바디는 Xform 루트에 건다. `qr_payload`·`dest`·치수를 프림의 `palletris:*` 커스텀 속성과 매니페스트로 남긴다(§3-1). ⚠ 쓰기 전 2가지 — `QR_CODES` 의 `payload` 를 실제 QR 내용으로 채우고, 이미지를 저장소로 옮길 것 |
| `gen_master_db.py` | 민범진 | D1 — **직접 추가할 것** |

게이팅 검증 스크립트(G1~G7)는 전부 여기에 커밋한다. (작업명세서 §11)
