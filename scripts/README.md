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
| `gen_master_db.py` | 민범진 | **D1 — boxes_master 생성.** 실행 전에 "정답지"를 깐다. 실행 중 흘러간 박스(`boxes_sorted`)와 대조해야 인식률을 잴 수 있다 — 등록된 것만 세면 언제나 100 % 로 나온다 (명세서 §8.6). stage 1 은 zone 전부 `B` (랜덤 금지). `ros2 run packing_db gen_master_db` 와 같다 |
| `verify_db.py` | 민범진 | **D2 판정 — `/db/*` 6종 + `/diverter/request`.** 박스 하나를 등록→검증→분기→집기→적재까지 태우며 응답을 확인한다. 실패 경로(모르는 status·미등록 박스·남의 QR)도 같이 친다 — 성공 경로만 보면 조용히 잘못 도는 버그를 못 잡는다. `db_service_node` 를 먼저 띄울 것 |
| `db_peek.py` | 민범진 | **palletris.db 를 사람이 읽는 표로.** SQL 도 sqlite3 CLI 도 필요 없다(이 PC 에 CLI 가 없다 — 가이드 D1 의 `sqlite3 ...` 확인 명령은 안 돈다). 읽기 전용(`mode=ro`)이라 `db_service_node` 와 같이 떠 있어도 안전하다. `--box BOX00003` 은 그 박스의 상태 전이 이력, `--csv` 는 발표용 덤프 |
| `report.py` | 민범진 | **D3 — 발표의 "결과 수치".** 처리량·인식률·pick&place 성공률·평균 사이클·로봇 유휴율·최고 높이(실측)를 표로. 쿼리는 `packing_db/metrics.py` 하나를 `db_service_node` 의 `/metrics/summary` 와 **같이 쓴다** — 복붙하면 실시간 화면과 최종 보고서의 숫자가 갈린다. `--csv` 덤프, `--watch` 는 2초마다 다시 그린다. 읽기 전용이라 실행 중에 돌려도 안전하다 |
| `verify_vision.py` | 민범진 | **D 인식 전 구간 — Isaac 없이.** 씬 USD 가 이 저장소에 없어서(sync_scene.sh 배포) 이 스크립트가 **Isaac 의 대역**을 선다 — 알려진 카메라·박스를 직접 투영해 `<ns>/rgb`·`<ns>/camera_info`·`/tf_static` 을 씬과 같은 규약으로 낸다. cam1 등록 → pick_cam 좌표 → 빈 벨트까지 확인. 실측 x 0.14 mm · yaw 0.05°. `db_service_node` 와 `vision_node` 를 먼저 띄울 것 |
| `verify_ros_graph.py` | 민범진 | **★ ROS 2 그래프 전 구간 — Isaac 없이.** 진짜로 도는 것은 인식·DB·알고리즘·관제탑 넷이고, 이 스크립트가 **Isaac 이 하는 일 전부**를 대신한다 — `/clock`·`/tf_static`·카메라 2대·씬 서비스 4종·로봇(`/robot/cmd`→`/robot/event`). `verify_b4_cycle.py` 의 StandInServer 와 같은 자리의 ROS 2 판이다. 박스가 인식→등록→분기→집기→적재→heightmap 갱신까지 실제로 흘러간다. `ros2 launch packing_bringup bringup.launch.py` 를 먼저 띄울 것 |

게이팅 검증 스크립트(G1~G7)는 전부 여기에 커밋한다. (작업명세서 §11)
