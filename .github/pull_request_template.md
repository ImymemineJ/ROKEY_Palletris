## 무엇을 했나

<!-- 한 줄. 예: A7 스토퍼 감지 → /zone_b/box_at_pickup 발행 -->

## 관련 산출물 / 이슈

<!-- 예: A7, closes #12 -->

## 확인한 것

- [ ] `colcon build --symlink-install` 성공
- [ ] 내 노드를 실제로 띄워서 동작 확인 (또는 Mock 상대로 확인)
- [ ] 새로 추가한 숫자를 코드에 박지 않고 `config/system.yaml` 에서 읽는다
- [ ] 콜백(타이머 포함) 안에서 `spin_until_future_complete` 를 쓰지 않았다
- [ ] mm(정수) ↔ m(float) 변환은 `packing_common.units` 만 사용했다
- [ ] `.usd` / `.db` / `build` / `install` / `log` 를 커밋하지 않았다

## 남이 알아야 할 변경 (해당되면 체크하고 팀 채팅에 공지)

- [ ] `packing_msgs` 를 바꿨다 → 머지 후 **전원 pull + 재빌드** 필요
- [ ] `config/system.yaml` 의 값을 바꿨다 → 바꾼 키와 이유:
- [ ] 서비스/토픽 시그니처를 바꿨다 → 영향받는 트랙:
