"""C4 베이스라인 — 단순 순차 적재 (명세서 §7.7).

제안 알고리즘의 대조군이다. **절대 생략 금지** — 발표에서 "최적화했다"를 증명하는
유일한 근거가 이 대조군과의 비교이기 때문이다.

`HeightmapPacker` 를 상속해 `plan()` 만 갈아끼운다. 그러면 `update()` / `fill_rate()` /
heightmap 상태 관리를 전부 그대로 재사용하므로, 비교 실험이 두 패커를 완전히 같은
조건으로 돌릴 수 있다.
"""
from packing_algo.heightmap_core import HeightmapPacker


class BaselinePacker(HeightmapPacker):
    """회전 없이(yaw 0 고정) 좌하단부터 훑어서 '처음 들어가는 곳'에 놓는다.

    점수도 지지율도 없다 — '아무 생각 없이 쌓기'의 대조군이다.

    ★★ 2026-08-24: **통로 검사는 건다.** 전에는 안 걸었다. 그때는 통로 검사가
      "그리퍼가 지나갈 여유" 라는 휴리스틱이었지만, 지금은 궤적 스윕 실측에 근거한
      **물리적 도달 가능성**이다 (`_corridor_ok`, 트랙 B PR #15).

      대조군이 그걸 안 받으면 **팔이 갈 수 없는 자리를 세게 되고**, 비교가
      "전략 대 전략" 이 아니라 "물리를 지킴 대 안 지킴" 이 된다. 우리가 방금
      우리 옛 숫자(66.9%)를 허수라고 버린 것과 같은 이유다.

      ⚠ 이 변경은 **우리에게 유리한 쪽으로 작동한다** (차이 +10.8%p → +17.7%p).
        그래서 근거를 여기 남긴다 — 유리해서 바꾼 게 아니라, 같은 잣대를 양쪽에
        적용하니 그렇게 된 것이다. 베이스라인의 정체성은 '회전 없음 · 좌하단 순차 ·
        점수 없음' 이지 '물리를 무시함' 이 아니다.

        시드 30개 포화 기준:
          제안                     21.2개 · 58.5%
          베이스라인 (통로 없음)     18.0개 · 47.8%   <- 도달 불가 자리 포함
          베이스라인 (통로 적용)     14.2개 · 40.8%   <- 이 값을 쓴다
    """

    def plan(self, box_id, w_mm, d_mm, h_mm):
        wc, dc = w_mm // self.cell, d_mm // self.cell
        if wc > self.W or dc > self.D:
            return False, None, 0.0, "NO_FEASIBLE_POSE"
        bz = self._base_z(wc, dc)                       # 부모의 윈도 최대 계산을 재사용
        corridor = self._corridor_ok(wc, dc, bz, h_mm)  # ★ 물리적 도달 가능성 (위 참조)
        for i in range(bz.shape[0]):
            for j in range(bz.shape[1]):
                if bz[i, j] + h_mm <= self.max_stack and corridor[i, j]:
                    # int() 로 감싼다 — i,j 는 numpy 인덱스라 int64 가 샌다 (TS-017)
                    pose = dict(x_mm=int(i * self.cell + w_mm // 2),
                                y_mm=int(j * self.cell + d_mm // 2),
                                z_mm=int(bz[i, j]), yaw_deg=0.0)
                    self.plans[box_id] = (pose, w_mm, d_mm, h_mm)
                    return True, pose, 0.0, "OK"
        # 어디에도 안 들어간다 — 가장 낮은 박스로도 안 되면 팔레트가 끝난 것
        return False, None, 0.0, "PALLET_FULL" if self._is_pallet_full() else "NO_FEASIBLE_POSE"
