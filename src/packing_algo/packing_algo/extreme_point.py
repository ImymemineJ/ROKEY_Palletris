"""C4 비교 대조군 — Extreme Point (EP) 3D Bin Packing 알고리즘.

학술/산업 논문(Crainic et al. 2008, INFORMS JoC)에서 가장 널리 인용되는
글로벌 표준 3D Bin Packing 알고리즘이다.

제안 알고리즘(Heightmap + 가중합 + Lookahead + _corridor_ok)의 대조군으로 사용된다.
HeightmapPacker를 상속하여 plan() 및 EP 관리 로직을 구현하므로
동일한 환경(Isaac Sim / demo_stack15 / compare_baseline)에서 돌려볼 수 있다.
"""

import math
import numpy as np
from packing_algo.heightmap_core import HeightmapPacker


class ExtremePointPacker(HeightmapPacker):
    """Extreme Point (EP) 3D Bin Packing 패커.

    3D 공간에서 박스가 배치될 때 발생하는 3D 모서리 교차점(Extreme Points)을 관리하며,
    우선순위(Bottom-Left-Deepest: z -> y -> x)에 따라 박스를 적재한다.

    옵션:
        use_rotation: True 이면 yaw 0도 및 90도 회전을 둘 다 시도함 (기본 True)
        use_corridor: True 이면 로봇 팔 통로 검사(_corridor_ok)를 적용함 (기본 False)
    """

    def __init__(self, cfg, use_rotation=True, use_corridor=False):
        super().__init__(cfg)
        self.use_rotation = use_rotation
        self.use_corridor = use_corridor
        
        # 팔레트 크기 (mm 정수)
        self.pal_w_mm = self.W * self.cell
        self.pal_d_mm = self.D * self.cell

        # 초기 Extreme Point: (0, 0, 0)
        self.eps = [(0, 0, 0)]
        
        # 배치된 박스 목록: [(x, y, z, w, d, h), ...] (mm 단위)
        self.placed_boxes = []

    def plan(self, box_id, w_mm, d_mm, h_mm):
        """EP 알고리즘으로 다음 박스의 배치 위치를 결정한다."""
        if w_mm > self.pal_w_mm or d_mm > self.pal_d_mm:
            return False, None, 0.0, "NO_FEASIBLE_POSE"

        # 회전 옵션에 따른 (w, d, yaw) 후보
        orientations = [(w_mm, d_mm, 0.0)]
        if self.use_rotation and w_mm != d_mm:
            orientations.append((d_mm, w_mm, 90.0))

        # EP 정렬: Bottom-Left-Deepest (z -> y -> x 오름차순)
        sorted_eps = sorted(self.eps, key=lambda p: (p[2], p[1], p[0]))

        best_plan = None  # (ep, pose, bw, bd, bh, yaw)

        for ep in sorted_eps:
            ep_x, ep_y, ep_z = ep

            for bw, bd, yaw in orientations:
                # 1. 팔레트 범위 검사
                if ep_x + bw > self.pal_w_mm or ep_y + bd > self.pal_d_mm:
                    continue
                if ep_z + h_mm > self.max_stack:
                    continue

                # 2. 기존 배치된 박스들과 3D AABB 겹침 검사
                if self._check_overlap(ep_x, ep_y, ep_z, bw, bd, h_mm):
                    continue

                # 3. 밑면 지지율 (Support Ratio) 검사
                if not self._check_support(ep_x, ep_y, ep_z, bw, bd):
                    continue

                # 4. 로봇 팔 통로 검사 (_corridor_ok, 옵션 선택)
                if self.use_corridor:
                    wc, dc = bw // self.cell, bd // self.cell
                    bz = self._base_z(wc, dc)
                    corridor = self._corridor_ok(wc, dc, bz, h_mm)
                    cell_x = int(ep_x // self.cell)
                    cell_y = int(ep_y // self.cell)
                    if cell_x >= corridor.shape[0] or cell_y >= corridor.shape[1] or not corridor[cell_x, cell_y]:
                        continue

                # 가장 우선순위가 높은(정렬 순서상 빠른) EP를 선택
                best_plan = (ep, bw, bd, h_mm, yaw)
                break

            if best_plan is not None:
                break

        if best_plan is None:
            is_full = self._is_pallet_full()
            return False, None, 0.0, "PALLET_FULL" if is_full else "NO_FEASIBLE_POSE"

        # 배치 결정
        (ep_x, ep_y, ep_z), bw, bd, bh, yaw = best_plan
        center_x = ep_x + bw // 2
        center_y = ep_y + bd // 2
        
        pose = dict(
            x_mm=int(center_x),
            y_mm=int(center_y),
            z_mm=int(ep_z),
            yaw_deg=float(yaw)
        )

        self.plans[box_id] = (pose, bw, bd, bh)
        self.placed_boxes.append((ep_x, ep_y, ep_z, bw, bd, bh))
        self.placed.append((pose, bw, bd, bh))

        # Heightmap 갱신
        wc, dc = bw // self.cell, bd // self.cell
        cx, cy = ep_x // self.cell, ep_y // self.cell
        top_z = ep_z + bh
        self.hm[cx:cx+wc, cy:cy+dc] = np.maximum(self.hm[cx:cx+wc, cy:cy+dc], top_z)

        # Extreme Points 갱신
        self._update_extreme_points(ep_x, ep_y, ep_z, bw, bd, bh)

        return True, pose, 0.0, "OK"

    def _check_overlap(self, x, y, z, w, d, h):
        """기존 배치된 박스들과 3D 공간상 겹침(Overlap)이 있는지 검사."""
        for bx, by, bz, bw, bd, bh in self.placed_boxes:
            if (x < bx + bw and x + w > bx and
                y < by + bd and y + d > by and
                z < bz + bh and z + h > bz):
                return True
        return False

    def _check_support(self, x, y, z, w, d):
        """밑면 지지율(Support Ratio) 검사."""
        if z == 0:
            return True

        support_area = 0
        total_area = w * d

        for bx, by, bz, bw, bd, bh in self.placed_boxes:
            if bz + bh == z:
                ix_min = max(x, bx)
                ix_max = min(x + w, bx + bw)
                iy_min = max(y, by)
                iy_max = min(y + d, by + bd)

                if ix_max > ix_min and iy_max > iy_min:
                    support_area += (ix_max - ix_min) * (iy_max - iy_min)

        return (support_area / total_area) >= self.w_s

    def _update_extreme_points(self, x, y, z, w, d, h):
        """박스 배치 후 Extreme Point 목록 갱신 (Crainic et al. 2008 기준)."""
        new_eps = []
        for ep_x, ep_y, ep_z in self.eps:
            if (x <= ep_x < x + w and y <= ep_y < y + d and z <= ep_z < z + h):
                continue
            new_eps.append((ep_x, ep_y, ep_z))

        candidates = [
            (x + w, y, z),
            (x, y + d, z),
            (x, y, z + h),
            (x + w, y + d, z),
            (x + w, y, z + h),
            (x, y + d, z + h),
        ]

        for bx, by, bz, bw, bd, bh in self.placed_boxes:
            if bz <= z < bz + bh and by <= y < by + bd and bx + bw > x:
                candidates.append((bx + bw, y, z))
            if bz <= z < bz + bh and bx <= x < bx + bw and by + bd > y:
                candidates.append((x, by + bd, z))
            if bx <= x < bx + bw and by <= y < by + bd and bz + bh > z:
                candidates.append((x, y, bz + bh))

        candidates.append((x + w, 0, z))
        candidates.append((0, y + d, z))

        for cx, cy, cz in candidates:
            if 0 <= cx < self.pal_w_mm and 0 <= cy < self.pal_d_mm and 0 <= cz < self.max_stack:
                inside = False
                for bx, by, bz, bw, bd, bh in self.placed_boxes:
                    if (bx <= cx < bx + bw and by <= cy < by + bd and bz <= cz < bz + bh):
                        inside = True
                        break
                if not inside and (cx, cy, cz) not in new_eps:
                    new_eps.append((cx, cy, cz))

        self.eps = new_eps
