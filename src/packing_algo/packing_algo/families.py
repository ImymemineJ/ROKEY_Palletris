"""계열별 대조군 — "왜 heightmap 인가" 의 근거 (명세서 §13 챌린지).

`baseline.py` 는 '아무 생각 없이 쌓기' 하나뿐이라 **"우리가 문헌의 어느 계열보다
낫다"** 를 못 보여준다. 여기서 온라인 3D 적재의 대표 계열을 같은 조건으로 돌린다.

## 실험 설계 — 제약은 같게, 전략만 다르게

모든 계열에 **똑같은 물리 제약**을 건다:

    높이 제한(max_stack) · 통로(_corridor_ok) · 지지율(min_support_ratio) · 계획 간격

계열 차이는 **"놓을 수 있는 칸 중 무엇을 고르나"** 하나뿐이다. 제약을 계열마다
다르게 주면 비교가 '전략 대 전략'이 아니라 '물리를 지킴 대 안 지킴'이 된다 —
`baseline.py` 가 통로 검사를 대조군에도 거는 것과 같은 이유다.

⚠ 그래서 여기 숫자는 `baseline.py` 의 것과 **직접 비교하면 안 된다.**
  `BaselinePacker` 는 지지율을 안 보기 때문이다(그쪽은 그게 의도다).

## 계열

| 계열 | 고르는 기준 | 출처 |
|---|---|---|
| `ShelfPacker` | 현재 층에서만 좌하단. 안 들어가면 층을 올린다 | 층 단위 팔레타이징 (산업 관행) · shelf/NFDH 계열 |
| `DBLFPacker` | 가장 낮은 z → 가장 아래(y) → 가장 왼쪽(x) | Karabulut & Inceoglu (2004) |
| `ExtremePointPacker` | **극점(extreme point)** 후보만 보고 그중 DBLF | Crainic, Perboli & Tadei (2008) |
| `HeightmapPacker` (우리) | 전 격자 점수화: 높이·지지·벽·먼쪽·평면 | Wang & Hauser (2019) 변형 |

★ 어느 것도 **미래 박스를 모른다** (온라인). 오프라인 상한은 `ceiling.py` 가 잰다.
"""
import numpy as np

from packing_algo.heightmap_core import HeightmapPacker


class _FamilyPacker(HeightmapPacker):
    """계열 대조군 공통 뼈대. 자식은 `_rank()` 만 구현한다."""

    def _rank(self, yaw, wc, dc, bz, cand, h_mm):
        """이 yaw 에서 후보 칸의 **우선순위 배열**을 돌려준다 (작을수록 먼저).

        shape 는 bz 와 같다. 후보가 아닌 칸은 값이 무시된다.
        """
        raise NotImplementedError

    def _yaws(self):
        """훑을 yaw. 계열에 따라 회전을 안 쓰기도 한다."""
        return (0, 90)

    def plan(self, box_id, w_mm, d_mm, h_mm):
        why = self._bad_size(w_mm, d_mm, h_mm)
        if why:
            self.last_warning = f"{box_id}: 치수가 잘못됐다 ({why})"
            return False, None, 0.0, "NO_FEASIBLE_POSE"
        # ★ 계획 간격은 코어와 똑같이 건다. 안 걸면 이 계열만 박스를 더 촘촘히 놓아
        #   적재율이 부당하게 높게 나온다.
        w_mm, d_mm = w_mm + self.plan_clr, d_mm + self.plan_clr
        best = None
        for yaw in self._yaws():
            w, d = (w_mm, d_mm) if yaw == 0 else (d_mm, w_mm)
            wc, dc = w // self.cell, d // self.cell
            if wc > self.W or dc > self.D:
                continue
            bz = self._base_z(wc, dc)
            cand = (bz + h_mm <= self.max_stack) & self._corridor_ok(wc, dc, bz, h_mm)
            if not cand.any():
                continue
            rank = self._rank(yaw, wc, dc, bz, cand, h_mm)
            # 후보 아닌 칸을 맨 뒤로 밀어 정렬 자리를 안 뺏기게 한다
            rank = np.where(cand, rank, np.inf)
            for idx in np.argsort(rank, axis=None):
                if not np.isfinite(rank.flat[idx]):
                    break                                   # 남은 건 전부 후보 아님
                i, j = np.unravel_index(idx, bz.shape)
                # ★ 지지율은 물리다. 계열과 무관하게 전부 건다.
                if self._support_ratio(i, j, wc, dc, int(bz[i, j])) < self.min_sup:
                    continue
                key = (float(rank[i, j]), yaw)
                if best is None or key < best[0]:
                    best = (key, int(i), int(j), int(bz[i, j]), yaw, w, d)
                break                                       # 이 yaw 의 최선을 찾았다
        if best is None:
            return False, None, 0.0, (
                "PALLET_FULL" if self._is_pallet_full() else "NO_FEASIBLE_POSE")
        _, i, j, z, yaw, w, d = best
        pose = dict(x_mm=int(i * self.cell + w // 2), y_mm=int(j * self.cell + d // 2),
                    z_mm=int(z), yaw_deg=float(yaw))
        self.plans[box_id] = (pose, w_mm, d_mm, h_mm)
        self._after_pick(z, h_mm)
        return True, pose, 0.0, "OK"

    def _after_pick(self, z_mm, h_mm):
        """층 관리 같은 계열별 뒷정리. 기본은 아무것도 안 한다."""


class DBLFPacker(_FamilyPacker):
    """Deepest-Bottom-Left-Fill — Karabulut & Inceoglu (2004).

    가장 낮은 자리(z) → 그중 로봇에 가까운 쪽(y 작은 쪽) → 그중 왼쪽(x 작은 쪽).
    점수도 지지 가중치도 없다. **"낮은 데부터 채운다"는 원칙 하나뿐**이다.

    ★ 우리 알고리즘의 `w_height` 항만 남기고 나머지(지지·벽·먼쪽·평면)를 껐을 때와
      같다. 그래서 "그 항들이 정말 일을 하는가" 의 직접적인 대조군이다.
    """

    def _rank(self, yaw, wc, dc, bz, cand, h_mm):
        ii, jj = np.indices(bz.shape)
        # z 를 최우선, 다음 j, 다음 i. 계수는 하위 항이 상위 항을 못 넘게 잡는다.
        return bz.astype(np.float64) * (self.W + self.D) * 2 + jj * 2.0 + ii * 1.0 / self.W


class ShelfPacker(_FamilyPacker):
    """층(shelf) 단위 적재 — 산업 팔레타이징의 사실상 관행.

    현재 층 바닥(`layer_z`)에 딱 놓이는 자리만 쓴다. 거기 안 들어가면 **층을 올린다**
    (`layer_z += 그 층의 최고 박스 높이`). 층 안에서는 좌하단부터.

    ★ 실물 팔레타이저가 이렇게 한다 — 같은 규격 박스를 층으로 깔고 층을 반복한다.
      우리 시나리오는 박스가 3종(210/280/340)이라 층 높이가 안 맞는데, **그 손해가
      얼마인지**가 이 대조군의 요점이다.
    """

    def __init__(self, cfg):
        super().__init__(cfg)
        self.layer_z = 0
        self.layer_h = 0

    def reset(self):
        super().reset()
        self.layer_z = self.layer_h = 0

    def _rank(self, yaw, wc, dc, bz, cand, h_mm):
        ii, jj = np.indices(bz.shape)
        on_layer = (bz == self.layer_z)
        # 현재 층에 놓이는 칸을 절대 우선. 그 안에서 좌하단.
        return np.where(on_layer, 0.0, 1e9) + jj * 2.0 + ii * 1.0 / self.W

    def plan(self, box_id, w_mm, d_mm, h_mm):
        ok, pose, s, msg = super().plan(box_id, w_mm, d_mm, h_mm)
        if ok and pose["z_mm"] == self.layer_z:
            return ok, pose, s, msg
        if ok:
            # 현재 층에 자리가 없어 위로 새 층이 시작된 것 — 층 기준을 그리로 옮긴다
            self.layer_z, self.layer_h = pose["z_mm"], h_mm
            return ok, pose, s, msg
        # 층을 못 채웠는데 실패했다면 한 번 더 올려 본다 (다음 층 시도)
        top = int(self.hm.max())
        if msg == "NO_FEASIBLE_POSE" and top > self.layer_z:
            self.layer_z, self.layer_h = top, 0
            return super().plan(box_id, w_mm, d_mm, h_mm)
        return ok, pose, s, msg

    def _after_pick(self, z_mm, h_mm):
        if z_mm == self.layer_z:
            self.layer_h = max(self.layer_h, h_mm)


class ExtremePointPacker(_FamilyPacker):
    """극점(Extreme Point) 기반 — Crainic, Perboli & Tadei (2008).

    격자 전체가 아니라 **놓인 박스가 만들어 낸 모서리**만 후보로 본다:
    빈 팔레트의 (0,0), 그리고 놓인 박스마다 (x+w, y) · (x, y+d).
    그중에서 DBLF 순서로 고른다.

    ★ 후보가 O(놓인 개수) 라 아주 싸다 — 우리 격자 전수 점수화(120x80)의 수백분의 1.
      **"그 계산을 더 쓸 값어치가 있는가"** 를 재는 대조군이다.
    """

    def _extreme_cells(self):
        # ★ placed 는 (actual_pose, w, d, h) 다 (heightmap_core.update).
        #   actual_pose 의 x/y 는 **중심**이라 모서리로 되돌려야 한다 (§3-1).
        pts = {(0, 0)}
        for (pose, w, d, h) in self.placed:
            wc, dc = int(round(w / self.cell)), int(round(d / self.cell))
            i0 = int(round((pose["x_mm"] - w / 2.0) / self.cell))
            j0 = int(round((pose["y_mm"] - d / 2.0) / self.cell))
            pts.add((i0 + wc, j0))
            pts.add((i0, j0 + dc))
            pts.add((i0 + wc, j0 + dc))
        return pts

    def _rank(self, yaw, wc, dc, bz, cand, h_mm):
        ii, jj = np.indices(bz.shape)
        ep = np.zeros(bz.shape, dtype=bool)
        for (i, j) in self._extreme_cells():
            if 0 <= i < bz.shape[0] and 0 <= j < bz.shape[1]:
                ep[i, j] = True
        base = bz.astype(np.float64) * (self.W + self.D) * 2 + jj * 2.0 + ii * 1.0 / self.W
        # 극점이 아닌 칸은 아예 안 본다 — 이게 이 계열의 정체성이다.
        return np.where(ep, base, np.inf)
