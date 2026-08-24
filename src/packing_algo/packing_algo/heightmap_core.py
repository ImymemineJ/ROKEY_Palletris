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
        self.w_wall = hm_cfg.get("w_wall", 0.0)             # 벽 밀착도 (0 이면 기존 동작)
        self.w_far = hm_cfg.get("w_far", 0.0)               # 로봇에서 먼 쪽 우선 (0 이면 기존 동작)
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
        self.last_warning = ""                              # 마지막 update 경고 (래퍼가 로그로 올린다)
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
            # ★ 벽 밀착도 — 박스 둘레 중 팔레트 가장자리에 닿는 비율 (0~1).
            #   가장자리부터 채우면 가운데에 "큰 박스가 통째로 들어갈 자리"가 남는다.
            #   자투리 공간이 흩어지는 것을 막는 항이라, 이게 없으면 나중에 오는 5호가
            #   들어갈 480x380 짜리 연속 영역이 사라진다. (명세서 §7.3 Max-Contact-Area)
            per = 2 * (wc + dc)
            wall = ((ii == 0) * dc + (ii + wc == self.W) * dc +
                    (jj == 0) * wc + (jj + dc == self.D) * wc) / per
            # ★ 동점일 때 채우는 순서 — 로봇 기하 때문에 축마다 방향이 다르다 (트랙 B 피드백).
            #   로봇 베이스 (10900, 0) · 팔레트 원점 (10300, 300) 기준:
            #     Px 0(월드 X 10300) = 컨베이어 쪽  → 작은 쪽부터. 그대로 둔다.
            #     Py 0(월드 Y  300)  = 로봇에 가장 가까운 변(300mm) → ★ 큰 쪽부터 채운다.
            #   가까운 자리를 먼저 채우면, 그 뒤 먼 자리에 놓을 때 팔뚝이 이미 놓인 박스
            #   위를 넘어가야 한다. 가까운 변을 마지막에 남겨야 팔이 안 걸린다.
            #   동점 후보 중에서만 고르는 것이라 적재율 손해는 0 이다 (측정: 60.0% 동일,
            #   팔이 넘어가는 배치가 11.1회 → 9.7회로 감소).
            order = ii + (bz.shape[1] - 1 - jj)
            # ★ 먼 쪽 우선은 **동점 처리로는 약하다.** tie_eps 는 의도적으로 아주 작아서
            #   (진짜 점수차의 1%) 팔레트에 구조가 생기면 높이·지지·벽 항에 묻힌다.
            #   실측: 동점 처리만으로는 팔 넘김 5.5 → 5.3 회로 거의 안 준다.
            #   제대로 된 가중치가 필요하고, 다행히 적재율 손해가 없다
            #   (w_far 0.2 에서 팔 넘김 2.2 회 · 적재율 66.3% 로 오히려 소폭 상승).
            far = (jj + dc) / self.D                    # 먼 쪽일수록 1 에 가깝다
            score = np.where(cand, base_score + self.w_wall * wall + self.w_far * far
                             - self.tie_eps * order, -np.inf)
            # ③ 지지율: 상위 K개만 정밀 계산 (전 후보는 너무 비쌈 — 명세서 §7.9)
            flat = np.argsort(score, axis=None)[::-1][: self.topk]
            for idx in flat:
                i, j = np.unravel_index(idx, score.shape)
                if not cand[i, j]: continue
                sup = self._support_ratio(i, j, wc, dc, int(bz[i, j]))
                if sup < self.min_sup: continue
                s = (self.w_h * (1 - bz[i, j] / self.max_stack) + self.w_s * sup
                     + self.w_wall * wall[i, j] + self.w_far * far[i, j])
                key = (s, -order[i, j])                      # 동점 → 위 order 기준
                if best is None or key > best[0]:
                    best = (key, i, j, int(bz[i, j]), yaw, w, d)
        if best is None:
            # ③ 이 박스만 못 놓은 건지(NO_FEASIBLE_POSE), 팔레트가 정말 끝난 건지(PALLET_FULL) 구분
            return False, None, 0.0, "PALLET_FULL" if self._is_pallet_full() else "NO_FEASIBLE_POSE"
        _, i, j, z, yaw, w, d = best
        # ★ int()/float() 로 감싸는 이유: i,j,z 는 numpy 배열에서 나온 int64 다.
        #   그대로 내보내면 json.dump 가 "int64 is not JSON serializable" 로 죽고
        #   (스냅샷 저장이 통째로 실패한다), ROS 메시지 대입에서도 타입 경고가 난다.
        pose = dict(x_mm=int(i * self.cell + w // 2), y_mm=int(j * self.cell + d // 2),
                    z_mm=int(z), yaw_deg=float(yaw))         # ★ x,y=중심 / z=밑면
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
        self.last_warning = ""
        yaw = float(actual["yaw_deg"])

        # ① yaw 가 계획에서 yaw_tol_deg 이상 벗어났나
        # ★ 180° 주기로 비교한다. 박스는 회전 대칭이라 yaw 와 yaw±180 은 같은 배치다.
        #   로봇이 손목 회전량을 줄이려고 등가 각을 골라 오는데(cycle.nearest_equiv_yaw),
        #   단순 차이로 재면 그때마다 "yaw 오차 초과" 로 오판해 footprint 를 AABB 로
        #   부풀린다 — 멀쩡한 배치에서 적재율이 떨어진다.
        dyaw = abs((yaw - planned[0]["yaw_deg"] + 90.0) % 180.0 - 90.0) if planned else 0.0
        if planned is not None and dyaw > self.yaw_tol:
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
        # ★ 실측이 한계를 넘으면 조용히 넘어가면 안 된다. 그대로 기록하면 그 위로는
        #   영영 아무것도 못 놓고, 왜 그런지도 안 보인다. 실측 pose 가 잘못 왔다는
        #   신호이므로(get_box_pose 가 박스 중심이 아닌 것을 주는 등) 크게 남긴다.
        over = top_q > self.max_stack
        if over:
            self.last_warning = (f"{box_id}: 실측 상면 {top_q}mm 가 max_stack "
                                 f"{self.max_stack}mm 를 넘었다 — 실측 pose 확인 필요")
            ok = False

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
        """적재율 = 박스 부피 합 / (팔레트 바닥 넓이 × **실제 최고 높이**)  — 명세서 §7.8

        ★ 분모를 max_stack 으로 고정하지 말 것 (함정 #19). 절반만 찼는데 1300 으로
          나누면 부당하게 낮다. 대신 **층이 새로 올라가는 순간 분모가 계단처럼 뛰어**
          적재율이 일시적으로 떨어져 보인다 — 정의상 정상이다.
        ★ 바닥 넓이도 config 에서 온다. 0.8*1.2 로 박아 두면 팔레트 치수가 바뀔 때
          (2026-08-22 90° 회전 같은 일) 조용히 틀린 값이 나온다 (철칙 3).
        """
        vol = sum(w * d * h for _, w, d, h in self.placed) / 1e9       # m³
        hmax = max(1, int(self.hm.max()))                              # ★ 실제 최고 높이
        area_m2 = (self.W * self.cell) * (self.D * self.cell) / 1e6    # 1200×800 → 0.96
        return vol / (area_m2 * hmax / 1000.0)

    def capacity_rate(self):
        """용량 대비 적재율 = 박스 부피 합 / (팔레트 바닥 넓이 × **max_stack**)

        ★ 이건 §7.8 의 `fill_rate` 를 대체하는 게 아니라 **보조 지표**다.
          - `fill_rate` : "쌓은 만큼의 공간을 얼마나 알차게 썼나" (정본, 함정 #19)
          - `capacity_rate` : "트럭 한 칸(1200×800×1300)을 얼마나 채웠나"
          층이 올라갈 때 분모가 안 뛰므로 **단조 증가**한다. 발표 곡선용.
          포화까지 채우면 실제 최고 높이가 max_stack 에 붙어 두 값이 1%p 안쪽으로 만난다.
        """
        vol = sum(w * d * h for _, w, d, h in self.placed) / 1e9       # m³
        area_m2 = (self.W * self.cell) * (self.D * self.cell) / 1e6
        return vol / (area_m2 * self.max_stack / 1000.0)
