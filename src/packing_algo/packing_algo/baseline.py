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

    점수도 지지율도 통로 검사도 없다 — '아무 생각 없이 쌓기'의 대조군이다.
    높이 제한만 본다: base_z + h ≤ max_stack.
    """

    def plan(self, box_id, w_mm, d_mm, h_mm):
        wc, dc = w_mm // self.cell, d_mm // self.cell
        if wc > self.W or dc > self.D:
            return False, None, 0.0, "NO_FEASIBLE_POSE"
        bz = self._base_z(wc, dc)                       # 부모의 윈도 최대 계산을 재사용
        for i in range(bz.shape[0]):
            for j in range(bz.shape[1]):
                if bz[i, j] + h_mm <= self.max_stack:
                    pose = dict(x_mm=i * self.cell + w_mm // 2,
                                y_mm=j * self.cell + d_mm // 2,
                                z_mm=int(bz[i, j]), yaw_deg=0.0)
                    self.plans[box_id] = (pose, w_mm, d_mm, h_mm)
                    return True, pose, 0.0, "OK"
        # 어디에도 안 들어간다 — 가장 낮은 박스로도 안 되면 팔레트가 끝난 것
        return False, None, 0.0, "PALLET_FULL" if self._is_pallet_full() else "NO_FEASIBLE_POSE"
