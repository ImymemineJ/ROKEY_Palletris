# 트랙 C — 패킹 알고리즘 (담당: 정정인)

이 폴더(`src/packing_algo`) 안에서 작업할 때 이 파일이 저장소 루트의 `CLAUDE.md`와 함께
자동으로 로드된다. 루트 파일의 철칙 4개·단위/좌표 규약은 여기서도 그대로 적용된다.

## 정본 문서 — 자동으로 함께 로드된다

@../../docs/가이드/트랙C_정정인.md

## 미션

"이 박스를 어디에 놓을까"에 200ms 안에 답하는 순수 파이썬 알고리즘.
**Isaac Sim이 전혀 필요 없다** — 노트북에서 numpy/scipy로 개발하고 테스트하며,
Day 3에만 ROS2 서비스 래퍼를 씌운다. 발표에서 "최적화했다"를 증명하는 유일한 근거가
이 트랙의 베이스라인 비교 그래프이므로 C4(베이스라인)는 절대 생략하지 않는다.

## 3일 목표

| Day | 목표 | 완료 기준 |
|---|---|---|
| 1 (오후) | C0 `packing_common` · C1 heightmap 코어 골격 | common push, units 테스트 통과 |
| 2 | C1 완성(±3mm 오차 테스트) · C2 배치규칙+통로검사 · C3 적재율+시각화 · **C4 베이스라인** | 가상 15박스 시퀀스로 fill_rate 곡선 |
| 3 | C5 ROS 래퍼 · 통합 | 김병직의 호출에 200ms 내 응답 |

## 파일 구조

- `packing_algo/heightmap_core.py` — 핵심 클래스 `HeightmapPacker` (ROS 의존성 0)
- `packing_algo/algo_service_node.py` — ROS2 서비스 래퍼 (Day 3에만 작성, 코어는 손대지 않음)
- `packing_algo/viz.py` — matplotlib 3종: heightmap 히트맵 / 3D 배치도 / 적재율 곡선
- `test/test_core.py` — 수용 테스트 (아래 참고)
- `scripts/compare_baseline.py` — 베이스라인 비교 (★ 절대 생략 금지)

## 이 트랙이 절대 어기면 안 되는 것

1. `support_tol_mm`는 반드시 10(`= heightmap.cell_mm`)이어야 한다. 5로 낮추면 실측 pose의
   양자화 오차 때문에 2층부터 거의 모든 후보가 `NO_FEASIBLE_POSE`로 탈락한다.
2. `update()`는 **실측 pose**로 heightmap을 갱신한다. 계획(plan) 위치로 갱신하면 4~5단에서
   오차가 누적돼 배치가 어긋난다.
3. 지지율 계산은 상위 K개(`topk_support_check`)만 정밀 계산한다. 전체 후보에 대해 도는
   순진한 구현은 200ms 요구를 못 맞춘다(5호 기준 후보 약 5,878개 × 셀 1,824개 ≈ 1,070만 연산).
   `np.lib.stride_tricks.sliding_window_view` 대신 `scipy.ndimage.maximum_filter1d`
   (축별 separable 1-D 필터)를 쓴다.
4. `max_stack_mm`은 김병직의 G1(도달범위 검증, Day 2 09:00 확정) 결과가 나오기 전까지는
   **config 파라미터로만** 다룬다. 코드에 850을 박지 않는다.
5. 좌표 변환은 `packing_common/frames.py`의 `world_to_pallet_place` /
   `pallet_place_to_world`만 쓴다 — 김병직과 같은 파일을 import해야 좌표가 맞는다.
   `x_mm,y_mm`=중심, `z_mm`=밑면.
6. `/algo/*` 서비스 4개(`plan_placement`, `update_heightmap`, `cancel_plan`, `reset`)는 전부
   **`box_id`로 계획을 관리**한다. 순서로 관리하면 재순환/버퍼 우회가 있을 때 stale plan이
   영원히 남는다. 픽 실패·타임아웃 등 모든 이탈 경로에서 `cancel_plan`이 호출돼야 한다
   (호출은 트랙 B 책임이지만, C는 이게 호출될 것을 전제로 설계한다).

## 수용 기준 (완료 정의)

- **C1**: `test_noise_3mm` 통과 — 박스 pose에 ±3mm 랜덤 오차를 주입한 상태에서 5층까지 정상 배치
- **C1**: `test_speed` 통과 — 40% 찬 팔레트 상태에서 `plan()` 응답 < 200ms
- **C4**: `BaselinePacker`(yaw 고정 + 좌하단 순차 채움) 대비 fill_rate가 더 높음을 그래프로 증명
- **C5**: `algo_service_node`가 김병직의 서비스 호출에 200ms 이내로 응답

## 자주 막히는 곳

| 증상 | 원인 |
|---|---|
| 2층부터 NO_FEASIBLE_POSE만 나옴 | `support_tol_mm` ≠ 10, 또는 갱신 시 셀 격자 양자화 누락 |
| 박스가 팔레트 밖에 놓임 | `frames.py`를 김병직과 다른 파일로 import, 또는 팔레트 원점 좌표 확인 |
| plan이 느림 | `sliding_window_view` 사용 여부 → `maximum_filter1d`로 교체 |
| fill_rate가 이상하게 낮음 | 분모가 `max_stack_mm` 고정 아닌지 확인 — **실제 최고 높이**를 써야 함 |
| 90도 배치가 이상함 | yaw=90이면 (w,d) 스왑 — `plan`과 `update` 양쪽 모두에 적용했는지 |
