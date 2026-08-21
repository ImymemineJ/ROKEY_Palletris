import math

import numpy as np
from scipy.ndimage import maximum_filter1d

class HeightmapPacker:
    """팔레트 1개의 상태 + 배치 결정. 단위: 내부는 전부 mm 정수, 격자 10mm."""

    def __init__(self, cfg):
        hm_cfg = cfg["heightmap"]; pal = cfg["pallet"]
        self.cell = hm_cfg["cell_mm"]                       # 10
        self.W = pal["width_mm"]  // self.cell              # 80  (Px)
        self.D = pal["depth_mm"]  // self.cell              # 120 (Py)
        self.hm = np.zeros((self.W, self.D), dtype=np.int32)
        self.max_stack = pal["max_stack_mm"]                # ★ G1 전까지 이 값만 바꿔가며 실험
        self.w_h, self.w_s = hm_cfg["w_height"], hm_cfg["w_support"]
        self.tol = hm_cfg["support_tol_mm"]                 # 10 (= cell_mm! 5로 줄이지 말 것)
        self.clr = hm_cfg["gripper_clearance_mm"] // self.cell   # 3셀
        self.min_sup = hm_cfg["min_support_ratio"]          # 0.7
        self.topk = hm_cfg["topk_support_check"]            # 50
        self.yaw_tol = hm_cfg["yaw_tol_deg"]                # 5.0 — 이만큼 틀어지면 AABB 기록
        self.box_types = cfg["boxes"]["types"]              # PALLET_FULL 판정에 최소 높이가 필요
        # ★ 동점 벌점 계수 — 아래 plan() 참고. 상수를 박지 않고 config 값에서 유도한다.
        #   "실제로 의미 있는 최소 점수차"(높이 1셀 = w_h*cell/max_stack)의 1% 를
        #   좌표 전체 범위(W+D)에 나눠 준 값이라, 진짜 점수차를 절대 뒤집지 못한다.
        self.tie_eps = 0.01 * self.w_h * self.cell / self.max_stack / (self.W + self.D)
        self.plans = {}                                      # (zone,box_id) -> PlacePose
        self.placed = []                                     # [(x,y,z,w,d,h,yaw), ...]

    # ---------- 핵심 유틸 ----------
    @staticmethod
    def _sliding_max(a, w, axis):
        """길이 w 윈도의 시작 인덱스별 최대. maximum_filter1d는 중심 기준이므로 슬라이스로 보정."""
        mf = maximum_filter1d(a, size=w, axis=axis, mode="constant", cval=0)
        start = w // 2
        return np.take(mf, np.arange(start, start + a.shape[axis] - w + 1), axis=axis)

    def _base_z(self, wc, dc):
        """모든 배치 시작셀 (i,j)에 대한 밑면 높이 = 점유영역 최대. shape (W-wc+1, D-dc+1)"""
        return self._sliding_max(self._sliding_max(self.hm, wc, 0), dc, 1)

    def plan(self, box_id, w_mm, d_mm, h_mm):
        """→ (ok, PlacePose, score, message)"""
        best = None
        for yaw in (0, 90):
            w, d = (w_mm, d_mm) if yaw == 0 else (d_mm, w_mm)
            wc, dc = w // self.cell, d // self.cell
            bz = self._base_z(wc, dc)                       # ① 경계는 shape가 처리
            ok_h = bz + h_mm <= self.max_stack              # ② 높이
            corridor = self._corridor_ok(wc, dc, bz)        # ④ 통로 (전 후보, 필터로 싸다)
            cand = ok_h & corridor
            if not cand.any(): continue
            # 지지율 전 1차 점수. 후보가 아닌 칸은 -inf 로 빼서 topk 자리를 낭비하지 않게 한다.
            base_score = self.w_h * (1 - bz / self.max_stack)
            # ★ 동점이면 좌하단(i+j 작은 쪽) 우선 — 명세서 §7.3 DBLF.
            #   np.argsort 기본값은 불안정 정렬이라, 빈 팔레트처럼 점수가 전부 같을 때
            #   topk 에 뽑히는 후보가 뒤죽박죽이 되어 좌하단이 아예 안 들어온다.
            #   아래 for 문의 key 동점 처리는 "뽑힌 50개 안에서만" 도는 뒷북이므로,
            #   1차 점수에 미리 위치 벌점을 섞어 정렬 자체가 DBLF 를 따르게 한다.
            ii, jj = np.indices(bz.shape)
            score = np.where(cand, base_score - self.tie_eps * (ii + jj), -np.inf)
            # ③ 지지율: 상위 K개만 정밀 계산 (전 후보는 너무 비쌈 — 명세서 §7.9)
            flat = np.argsort(score, axis=None)[::-1][: self.topk]
            for idx in flat:
                i, j = np.unravel_index(idx, score.shape)
                if not cand[i, j]: continue
                sup = self._support_ratio(i, j, wc, dc, int(bz[i, j]))
                if sup < self.min_sup: continue
                s = self.w_h * (1 - bz[i, j] / self.max_stack) + self.w_s * sup
                key = (s, -(i + j))                          # 동점 → 좌하단 우선
                if best is None or key > best[0]:
                    best = (key, i, j, int(bz[i, j]), yaw, w, d)
        if best is None:
            # ③ 이 박스만 못 놓은 건지(NO_FEASIBLE_POSE), 팔레트가 정말 끝난 건지(PALLET_FULL) 구분
            return False, None, 0.0, "PALLET_FULL" if self._is_pallet_full() else "NO_FEASIBLE_POSE"
        _, i, j, z, yaw, w, d = best
        pose = dict(x_mm=i * self.cell + w // 2, y_mm=j * self.cell + d // 2,
                    z_mm=z, yaw_deg=float(yaw))              # ★ x,y=중심 / z=밑면
        self.plans[box_id] = (pose, w_mm, d_mm, h_mm)
        return True, pose, float(best[0][0]), "OK"

    def _is_pallet_full(self):
        """가장 낮은 박스 종류조차 높이 제한에 걸려 어디에도 못 들어가면 팔레트가 끝난 것.
        (코드에 210 같은 상수를 박지 않고 config 의 boxes.types 에서 최소 높이를 읽는다.)"""
        t = min(self.box_types, key=lambda b: b["h_mm"])
        for w, d in ((t["w_mm"], t["d_mm"]), (t["d_mm"], t["w_mm"])):   # 두 방향 다 확인
            bz = self._base_z(w // self.cell, d // self.cell)
            if (bz + t["h_mm"] <= self.max_stack).any():
                return False
        return True

    @staticmethod
    def _aabb(w_mm, d_mm, yaw_deg):
        """yaw 만큼 돌아간 w×d 사각형의 축정렬 외접 사각형(AABB) 크기(mm)."""
        r = math.radians(yaw_deg)
        c, sn = abs(math.cos(r)), abs(math.sin(r))
        return int(round(w_mm * c + d_mm * sn)), int(round(w_mm * sn + d_mm * c))

    def _support_ratio(self, i, j, wc, dc, base_z):
        foot = self.hm[i:i + wc, j:j + dc]
        return float((np.abs(foot - base_z) <= self.tol).mean())   # ★ tol=10 필수

    def _corridor_ok(self, wc, dc, bz):
        """그리퍼 여유 포함 확장 윈도의 최대가 밑면 이하인가 — 패딩으로 경계 처리"""
        c = self.clr
        pad = np.pad(self.hm, c, mode="constant", constant_values=0)
        big = self._sliding_max(self._sliding_max(pad, wc + 2 * c, 0), dc + 2 * c, 1)
        return big <= bz + self.tol

    def update(self, box_id, w_mm, d_mm, h_mm, actual):
        """★ 실측 pose로 갱신 — 이게 자기교정의 핵심 (명세서 §7.6)"""
        planned = self.plans.pop(box_id, None)               # ★ box_id 로 계획을 회수 (없으면 None)
        ok = True
        yaw = float(actual["yaw_deg"])

        # ① yaw 가 계획에서 yaw_tol_deg 이상 벗어났나
        if planned is not None and abs(yaw - planned[0]["yaw_deg"]) > self.yaw_tol:
            # 비스듬히 놓였다 = 축정렬 격자에 그대로 못 넣는다. 외접 사각형으로 부풀려
            # "여기는 뭔가 있다"고 안전하게 기록하고, 실패를 호출자(트랙 B)에게 알린다.
            w, d = self._aabb(w_mm, d_mm, yaw)
            ok = False
        else:
            w, d = (w_mm, d_mm) if round(yaw / 90) % 2 == 0 else (d_mm, w_mm)

        # 셀 수는 올림. 모자라게 기록하면 다음 층 박스가 그 틈에 떠서 놓인다.
        wc, dc = -(-w // self.cell), -(-d // self.cell)
        # ★ 반올림으로 셀을 정한다. // (내림) 을 쓰면 음수 오차에서만 한 칸 밀려 편향이 생긴다.
        #   (참값 x=170 인데 실측 167 → 내림 i=-1 / 반올림 i=0)
        #   한 칸 밀리면 원래 자리에 낡은 높이가 1셀 기둥으로 남고, base_z 는 최댓값이라
        #   다음 박스가 그 기둥 때문에 통째로 떠서 놓인다. (±3mm 에서 8개 → 10개 차이)
        i = int(round((actual["x_mm"] - w / 2) / self.cell))
        j = int(round((actual["y_mm"] - d / 2) / self.cell))
        top_q = max(0, round((actual["z_mm"] + h_mm) / self.cell) * self.cell)   # ★ 셀 격자로 양자화

        # ② 실측 좌표가 배열 밖으로 나갈 때 안전하게 자르기.
        #    자르지 않으면 음수 인덱스 슬라이스가 되어 엉뚱한 칸을 덮어쓴다.
        i0, i1 = max(i, 0), min(i + wc, self.W)
        j0, j1 = max(j, 0), min(j + dc, self.D)
        if i0 >= i1 or j0 >= j1:                             # 통째로 팔레트 밖 — 기록할 것이 없다
            return False, self.fill_rate(), len(self.placed)
        if (i0, i1, j0, j1) != (i, i + wc, j, j + dc):       # 일부만 걸쳐 있다 = 이미 이상 상황
            ok = False

        self.hm[i0:i1, j0:j1] = np.maximum(self.hm[i0:i1, j0:j1], top_q)
        self.placed.append((actual, w_mm, d_mm, h_mm))
        return ok, self.fill_rate(), len(self.placed)

    def cancel(self, box_id): self.plans.pop(box_id, None)
    def reset(self): self.hm[:] = 0; self.plans.clear(); self.placed.clear()

    def fill_rate(self):
        vol = sum(w * d * h for _, w, d, h in self.placed) / 1e9      # m³
        hmax = max(1, int(self.hm.max()))                              # ★ 실제 최고 높이!
        return vol / (0.8 * 1.2 * hmax / 1000.0)