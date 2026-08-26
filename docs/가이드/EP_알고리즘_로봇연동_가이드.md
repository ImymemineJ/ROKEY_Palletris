# Extreme Point (EP) 대조군 알고리즘 — 로봇팔 연동 및 시뮬레이션 가이드

**수신**: 트랙 B (로봇 제어 담당자)  
**발신**: 트랙 C (패킹 알고리즘 담당자 - 정정인)  
**날짜**: 2026-08-26  
**관련 브랜치**: `presentation/final-ppt`

---

## 1. 개요

발표 및 알고리즘 검증을 위해 3D Bin Packing 학술 논문의 글로벌 표준 대조군인 **Extreme Point (EP, Crainic et al. 2008)** 알고리즘이 `src/packing_algo/extreme_point.py` 및 `scripts/demo_stack15.py`에 완전 연동되었습니다.

로봇팔 제어 담당자분께서 Isaac Sim 환경에서 명령 한 줄로 EP 알고리즘 기반 적재 플랜 및 로봇팔 구동 모션을 바로 테스트하고 시각 자료를 확보하실 수 있습니다.

---

## 2. 시뮬레이션 실행 커맨드

### ① EP 알고리즘 기반 로봇팔 적재 시뮬레이션 구동 (GUI)
```bash
cd ~/Project_Palletris
python3 scripts/demo_stack15.py --plan-mode ep --seed 42 --gui
```
- **EP 알고리즘**(`ExtremePointPacker`)으로 3D 극점 계획을 세운 뒤, 두산 H2017 로봇팔이 해당 플랜으로 적재를 수행합니다.

### ② 제안 알고리즘 (우리 최종안) 로봇팔 적재 시뮬레이션 구동 (GUI)
```bash
python3 scripts/demo_stack15.py --plan-mode algo --seed 42 --gui
```
- **최종 튜닝된 제안 알고리즘** (Heightmap + Lookahead + `_corridor_ok` + 자유낙하 완화)으로 적재를 수행합니다.

### ③ 오프라인 플랜 JSON 파일만 생성할 경우
```bash
python3 scripts/demo_stack15.py --plan-mode ep --plan-only --boxes 35 --seed 42 --dump-plan /tmp/ep_plan.json
```

---

## 3. 관전 포인트 (트랙 B 관점)

1. **로봇 팔뚝/그리퍼 기하 간섭**:
   - EP 알고리즘은 순수 3D 공간 좌표 모서리만 다루므로 그리퍼(Ø245mm) 및 팔뚝이 내려갈 수 있는 수직 진입 통로(`_corridor_ok`)를 사전 계산하지 않습니다.
   - 로봇팔 하강 시 솟구친 이웃 기둥과 팔뚝이 충돌하거나 `IK_FAIL`이 발생하는 지점을 관찰하실 수 있습니다.

2. **스택 충돌 및 밀림 (knock)**:
   - EP 알고리즘의 지형 비평탄화로 인해 로봇팔 접근 시 이웃 박스를 치고 들어가는 현상(knock)을 측정 비교할 수 있습니다.

---

## 4. 변경 및 구현 파일 목록

- `src/packing_algo/packing_algo/extreme_point.py`: EP 3D 극점 패커 클래스 (`ExtremePointPacker`)
- `scripts/demo_stack15.py`: `--plan-mode ep` 옵션 연동 완료
- `docs/발표/05_compare_ep.mp4`: 애니메이션 비교 동영상 파일
