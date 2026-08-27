"""heightmap_core.py — 적재 알고리즘의 코어 (트랙 C 정정인).

    ╔══ 시나리오에서의 위치 ═══════════════════════════════════════════════════╗
    ║  ① 스폰 → ② 투입부 인식 → ③ 목적지·분기 → ④ 픽업 좌표 → ⑤ 벨트 정지     ║
    ║  → ⑥ 자리 계산 → ⑦ 집기 → ⑧ 놓기 → ⑨ 갱신·기록 → ⑩ 벨트 재가동 → ①    ║
    ║                                                                        ║
    ║  ★ 이 파일이 맡는 구간 : ⑥ 자리 계산의 **본체** · ⑨ 실측 반영
    ║     불러 주는 쪽        : algo_service_node · 수용 테스트 · 시드 스윕
    ║     이 파일이 부르는 쪽 : (없음 — ROS·Isaac 의존성 0)
    ╚════════════════════════════════════════════════════════════════════════╝
    ※ 전체 지도는 `docs/코드_리딩가이드.md` 에 있다.

**ROS 도 Isaac 도 import 하지 않는다.** 노트북에서 그대로 돌아가고, 수용 테스트
(`test/test_core.py`)와 시드 스윕(`robustness.py`)이 이 파일만 불러 돈다.
ROS 껍데기는 `algo_service_node.py` 다 — 거기서 `plan`/`update`/`cancel`/`reset`
네 개를 서비스로 연다.

## 우리 문제는 교과서 3D bin packing 이 아니다

| | 교과서 (offline) | 우리 문제 |
|---|---|---|
| 박스 목록 | 전부 미리 안다 | **하나씩 도착한다** (online) |
| 되돌리기 | 배치를 바꿔 가며 최적화 | **한 번 놓으면 끝** |
| 배치 주체 | 없음 (기하만 본다) | **6축 로봇 팔** — 기하학적으로 들어가도 팔이 못 가는 자리가 있다 |

세 번째가 이 파일의 성격을 정한다. `_corridor_ok` 가 여기 있는 이유이고,
"들어가긴 하는데 놓을 수는 없는 자리"를 후보에서 빼는 것이 알고리즘의 절반이다.

## 상태 — 팔레트 한 장

    self.hm        (W, D) int32. 셀당 **현재 높이[mm]**. 10mm 격자라 1200x800 → 120x80
    self.plans     box_id → 계획. `update` 가 실측과 대조하려고 들고 있다
    self.applied   box_id → 최종 판정. **재시도 멱등**을 위한 것 (update 주석 참조)
    self.placed    실제로 반영된 (pose, w, d, h) 목록. 적재율의 분자

단위는 **내부 전부 mm 정수**다. m float 는 경계(`packing_common.units`)에서만 쓴다.

## 배치 한 번의 흐름 (`plan`)

    yaw 0 / 90 각각에 대해
      ① _base_z        발자국 안 최고높이 = 놓을 밑면 높이       (슬라이딩 최대)
      ② 높이 필터       밑면 + h <= max_stack(1300)
      ③ _corridor_ok   **팔이 내려올 수 있나** — 주변 벽이 윗면+150 이하인가
      ④ _tower_ok      기둥 억제 (기본 꺼짐)
      ⑤ 가중합 점수     w_height .6 · w_support .4 · w_wall .2 · w_far .1 · w_level .05
      ⑥ 지지율 정밀검사  점수 상위부터 — 전 후보에 돌리기엔 비싸다 (명세서 §7.9)
      ⑦ _lookahead     상위 k개를 놓아 보고 뒤 박스를 굴려 미래 부피가 큰 쪽 (기본 꺼짐)
    → (ok, pose, score, message)   message = "OK" | "NO_FEASIBLE_POSE" | "PALLET_FULL"

★ **`NO_FEASIBLE_POSE` 와 `PALLET_FULL` 은 다르다** (CLAUDE.md §7).
  앞은 *이 박스만* 자리가 없다는 뜻이라 관제탑이 그 박스만 되돌리고 **계속**한다.
  뒤는 `_is_pallet_full()` 이 "가장 작은 박스로도 후보가 없다" 를 확인했을 때만 낸다.

## 속도 예산

`plan()` 평균 **2 ms** (95% 7ms · 최악 22ms). 명세서 §7.9 상한은 200 ms 다.
이 여유는 전부 `scipy.ndimage.maximum_filter1d` 에서 나온다 — 모든 시작셀에 대한
발자국 최댓값을 파이썬 루프 없이 한 번에 낸다. 루프로 바꾸면 100배 느려진다.
(앞보기를 켜면 평균 131 ms 로 올라가 상한에 붙는다. 그래서 기본이 꺼짐이다.)

## 이 파일을 고칠 때

- **숫자를 여기 박지 않는다.** 전부 `cfg`(= `system.yaml`)에서 읽는다.
- **새 항은 기본값 0 으로 넣는다.** 그래야 켜기 전까지 기존 결과가 안 바뀌고,
  A/B 를 같은 코드로 잴 수 있다. w_wall · w_far · w_level · 낙하 완화 · 기둥 억제가
  전부 그렇게 들어왔다.
- **판정은 Isaac 이 한다** (CLAUDE.md §10-1). 데스크 스윕은 후보를 좁히는 용도다 —
  데스크에서는 배치 실패가 박스 1개 손해지만, 실측에서는 실패 2건이 적재 3개를
  깎은 적이 있다(팔이 스택을 63mm 밀고, 무너진 지형이 뒤 박스 자리까지 죽였다).
  부호까지 반대로 나온 항이 실제로 있었다.
"""
import math
import random

import numpy as np
from scipy.ndimage import maximum_filter1d

class HeightmapPacker:
    """팔레트 1개의 상태 + 배치 결정. 단위: 내부는 전부 mm 정수, 격자 10mm."""

    def __init__(self, cfg):
        hm_cfg = cfg["heightmap"]; pal = cfg["pallet"]
        self.cell = hm_cfg["cell_mm"]                       # 10
        self.W = pal["width_mm"]  // self.cell              # 120 (Px, 월드 +X)
        self.D = pal["depth_mm"]  // self.cell              # 80  (Py, 월드 +Y = 로봇 쪽)
        self.hm = np.zeros((self.W, self.D), dtype=np.int32)
        self.max_stack = pal["max_stack_mm"]                # ★ G1 전까지 이 값만 바꿔가며 실험
        self.w_h, self.w_s = hm_cfg["w_height"], hm_cfg["w_support"]
        self.w_wall = hm_cfg.get("w_wall", 0.0)             # 벽 밀착도 (0 이면 기존 동작)
        self.w_far = hm_cfg.get("w_far", 0.0)               # 로봇에서 먼 쪽 우선 (0 이면 기존 동작)
        # ★ 평면 유지 항. 새 박스 **윗면**이 이웃 최고높이와 같으면 1, 벌어질수록 0.
        #   왜 필요한가: 이 팔레트에서 표면은 "조금 울퉁불퉁"해지지 않는다. 박스 높이가
        #   210/280/340 뿐이라 단차는 늘 60mm 이상 — **평평 아니면 절벽**이다.
        #   실측(시드 30): 계획 실패 26.6 회 중 19.6 회가 지지율 미달이고, 그때도
        #   높이·통로로는 셀의 27% 가 아직 남아 있었다. 즉 자리는 있는데 **딛을 수가**
        #   없다. 지지율 하한을 0.7 -> 0.5 로 낮추면 +3.7%p 가 나오지만 그건 공짜가
        #   아니다 — 최악 49% 의 바닥이 1m 깊이 위에 뜬다(외팔보). 그래서 사후에
        #   완화하는 대신 **절벽을 처음부터 안 만드는** 쪽으로 풀었다.
        #   ⚠ 값이 작아야 한다. 0.1 이상은 튜닝 시드에서만 오르고 검증 시드에서 사라진다
        #     (0.15/ring8: 튜닝 61.5% -> 검증 58.7%). 0.05 는 링 2/4/8 셀 어디서나
        #     같이 오른다(검증 61.1/61.3/61.0%) — 과적합이 아니라는 신호다.
        self.w_level = hm_cfg.get("w_level", 0.0)           # 평면 유지 (0 이면 기존 동작)
        self.ring = hm_cfg.get("level_ring_cells", 4)       # 이웃으로 볼 바깥 셀 수
        # ★★ 기둥 억제 3종 (2026-08-25, 김병직 제안 → 실측). **전부 기본 꺼짐**이라
        #   켜야만 동작이 바뀐다. 왜 필요한가: 시드 7 실측에서 가운데 260x350 자리가
        #   **16개 박스 동안 계속 비어 있다가**, 17번째로 (965,615) 기둥을 4단(윗면
        #   1300)까지 올리는 순간 벽이 970mm 가 되어 통로 검사에 막혔다. 그 뒤 11개가
        #   연속 버려졌다. 즉 자리가 없어진 게 아니라 **스스로 봉했다.**
        #   가중치로는 못 고친다 — w_wall/w_far 를 끄면 우물이 오히려 늘어난다
        #   (208 -> 296 L, 적재율 70.8 -> 57.2%).
        #
        #   top_box_max_h: 윗면이 max_stack 에 **정확히 닿는** 자리에 이 높이보다 큰
        #     박스를 금지한다. 큰 박스로 꼭대기를 막지 않는다. 0 이면 꺼짐.
        #     ※ 막힌 박스는 NO_FEASIBLE_POSE 로 되돌아간다 (§7). 리턴 라인 3바퀴 후
        #       삭제가 팀 결정이므로 무한 순환은 retry_count 가 끊는다.
        #   tower_ring_mm: 놓은 뒤 윗면이 **주변 링의 최저**보다 이만큼 넘게 높으면 금지.
        #     ⚠ 링의 **최고**와 비교하면 안 된다 — 박스 높이가 최대 340 이라 그 조건은
        #       항상 참이고 규칙이 한 번도 발동하지 않는다 (측정으로 확인).
        #   tower_ring_cells: 위 링의 크기(셀). 25 = 250mm.
        #   max_span_mm: 놓은 뒤 윗면이 **팔레트 전체 최저점**보다 이만큼 넘게 높으면 금지.
        #     사실상 "층을 채우고 올라가라". ⚠ 적재율 지표에 속기 쉽다 — 600 에서
        #     실제높이 기준 74.9% 가 나오지만 개수는 26 -> 13 이고 560mm 에서 멈춘다.
        #     분모가 실제 최고높이라 낮게 끝나면 뻥튀기된다. **용량 기준으로 볼 것.**
        self.top_box_max_h = int(hm_cfg.get("top_box_max_h", 0))
        self.tower_ring_mm = int(hm_cfg.get("tower_ring_mm", 0))
        self.tower_ring_cells = int(hm_cfg.get("tower_ring_cells", 25))
        self.max_span_mm = int(hm_cfg.get("max_span_mm", 0))
        # ★ 앞보기 — 후보 몇 개를 실제로 놓아 보고, 그 뒤 박스를 굴려 본 뒤 고른다.
        #   왜 지금 되나: `plan()` 이 2.7ms 인데 로봇 모션이 5~10초다. **계산 예산이
        #   1000배 남는데 한 수만 보고 확정하고 있었다.**
        #   la_k <= 1 이면 예전 그리디와 **완전히 같다** (cands[0] = 옛 best).
        la = hm_cfg.get("lookahead", {}) or {}
        self.la_k = int(la.get("k", 1))              # 살펴볼 후보 수
        self.la_rollouts = int(la.get("rollouts", 3))
        self.la_depth = int(la.get("depth", 4))
        self.tol = hm_cfg["support_tol_mm"]                 # 10 (= cell_mm! 5로 줄이지 말 것)
        # ★ 구 등방 halo. 아래 arm_* 로 대체됐고, robot 섹션이 없는 축약 cfg 에서만 쓴다.
        self.clr = hm_cfg["gripper_clearance_mm"] // self.cell   # 3셀
        # ★ 하강 통로는 **팔 굵기**로 봐야 한다 — gripper_clearance_mm(30) 이 아니다.
        #   두 값 다 system.yaml 에 이미 있다. 새 상수를 지어내지 않는다.
        rb = cfg["robot"]
        # ★ 통로 반경. 기본은 그리퍼 몸통 반경(122mm -> 12셀) = 기존 동작.
        #   ⚠ 이 모델은 팔을 **수직 원기둥**으로 본다. 실제 link_3(팔뚝)는 팔꿈치에서
        #     비스듬히 내려와 옆으로 더 차지한다 (2026-08-25 영상 실측: 5호 2단 680mm
        #     기둥이 발자국에서 **145mm** 떨어져 있었는데 반경 122 라 안 보였고,
        #     팔뚝이 그 기둥 옆면에 걸려 46mm 위에서 멈췄다 — 잔차 55mm, IK_FAIL).
        #     `corridor_radius_mm` 로 그 여유를 넓힌다.
        self.arm_r = int(hm_cfg.get("corridor_radius_mm", 0)) // self.cell \
            or rb["gripper_body_diameter_mm"] // 2 // self.cell   # 122//10 = 12셀
        # ★★ 2단 통로 (2026-08-25). 팔은 **위로 갈수록 굵다** — 반경 하나로 사방을 보면
        #   얕은 자리까지 같이 손해를 본다.
        #   descriptor 실측 (link_4 충돌구체, 축이탈 + 반지름 = 축에서의 최대 폭):
        #     손목 중심(z 0)     축이탈 203 + r 117 = **320 mm**
        #     154 뒤            163 + 119 = 282 mm
        #     307 뒤            127 +  98 = 225 mm
        #     442 뒤             45 +  96 = 141 mm
        #     594 뒤              7 + 108 = 115 mm
        #   그리고 손목 중심은 흡착면에서 121(플랜지~손목) + 150(그리퍼) = **271 mm** 위다.
        #   → 1단: 반경 122 안에서는 벽이 "내 윗면 + 150" 까지 (그리퍼 몸통만 지난다)
        #     2단: 반경 320 안에서는 벽이 "내 윗면 + 271" 까지 (손목이 그 높이에 있다)
        #   ⚠ 2단 값이 정본은 아니다 — 팔 자세에 따라 손목이 기울어 실제 폭이 달라진다.
        #     그래서 **기본 꺼짐**으로 두고 Isaac 으로 재서 정한다 (CLAUDE.md §10-1).
        self.arm_r2 = int(hm_cfg.get("corridor_tier2_radius_mm", 0)) // self.cell
        self.arm_gap2 = int(hm_cfg.get("corridor_tier2_gap_mm", 271))
        self.arm_gap = rb["gripper_length_mm"]                          # 150 mm
        # ★★ 낙하 완화 (2026-08-25, 김병직 지적 "자유낙하가 되니 구속이 풀린 부분이 있다").
        #   `_corridor_ok` 가 존재하는 이유는 **팔이 박스를 따라 내려가야** 하기 때문이다.
        #   그런데 가벼운 박스는 협곡 입구에서 놓고 떨어뜨린다 (cycle.py DROP_*) —
        #   팔이 안 내려가므로 벽이 `윗면 + 그리퍼길이` 를 넘어도 된다.
        #   실측: 시드 7 끝 시점에 z=560 · 0.14 m² · 여유 740mm 자리가 **높이로는
        #   3종이 다 들어가는데 통로가 막아** 비어 있었다. 계획이 낙하를 모르는 탓이다.
        #   drop_relief_mm 만큼 그 자리에 한해 벽 높이를 더 봐준다. 0 이면 꺼짐.
        #   ⚠ 로봇의 낙하 상한(PL_DROP_MM)보다 크게 잡으면 안 된다 — 계획이 "놓을 수
        #     있다" 고 한 자리를 로봇이 못 놓는다.
        self.drop_relief = int(hm_cfg.get("drop_relief_mm", 0))
        self.drop_max_kg = float(hm_cfg.get("drop_max_kg", 0.0))
        self._mass_kg = {(b["w_mm"], b["d_mm"], b["h_mm"]): b["mass_g"] / 1000.0
                         for b in cfg["boxes"]["types"]}
        # ★ 통로 검사의 **방향성** — 팔은 로봇 쪽(py 0)에서 들어온다. 1.0 이면 지금까지의
        #   대칭 검사, 0.0 이면 먼 쪽(py 큰 쪽)은 안 본다. 기본 1.0 = 기존 동작 그대로.
        # ★★ 계획 간격 (2026-08-25, 김병직 실측 → 정정인 승인).
        #   발자국을 이만큼 부풀려 잡아 **박스 사이에 실제 틈**을 남긴다.
        #   간격 0 은 슬롯이 박스와 **정확히 같은 크기**라 물리적으로 성립하지 않는다:
        #     · 콜라이더 contact offset 이 면마다 2 mm
        #     · 배치 XY 오차가 실측 최대 7.6 mm · yaw 오차 4.0도 (410 박스 모서리 14 mm)
        #   Isaac 실측(20박스·도착순·이격400): 간격 0 이면 9kg 5호가 338 mm 위에서
        #   막히고 이웃을 **1046 mm 밀어 팔레트 밖으로 떨어뜨렸다** (RESULT 실패).
        #   10 을 넣자 **20/20 · 10단 1300 mm · knock 11.5 mm · 통과**.
        #   ⚠ plan 과 update 에 **똑같이** 걸어야 한다. 한쪽만 부풀리면 update 의
        #     `i = round((actual.x_mm - w/2)/cell)` 이 셀 단위로 어긋나 발자국이
        #     실제와 다른 자리에 찍힌다. 그래서 호출부가 아니라 **코어 안에서** 건다.
        self.plan_clr = int(hm_cfg.get("plan_clearance_mm", 0))
        self.corridor_far = float(hm_cfg.get("corridor_far_ratio", 1.0))
        # ★ 로봇 쪽(py 작은 쪽) 여유는 **따로** 준다. 팔뚝은 이 방향에서 넘어 들어오므로
        #   먼 쪽과 같은 기준(윗면+150)을 주면 넘어야 할 벽을 그대로 통과시킨다.
        #   기본값은 arm_gap = 기존 동작 그대로.
        self.near_gap = float(hm_cfg.get("corridor_near_gap_mm", self.arm_gap))
        self.min_sup = hm_cfg["min_support_ratio"]          # 0.7
        self.topk = hm_cfg["topk_support_check"]            # 50
        self.yaw_tol = hm_cfg["yaw_tol_deg"]                # 5.0 — 이만큼 틀어지면 AABB 기록
        self.box_types = cfg["boxes"]["types"]              # PALLET_FULL 판정에 최소 높이가 필요
        # ★ 앞보기가 "다음에 뭐가 올까" 를 굴릴 때 쓴다. 실제 혼입비를 써야
        #   3호가 절반이라는 사실이 반영된다 (균등으로 굴리면 5호를 과대평가한다).
        self.box_mix = list(cfg["boxes"].get("size_distribution",
                                             [1.0] * len(self.box_types)))
        # ★ 동점 벌점 계수 — 아래 plan() 참고. 상수를 박지 않고 config 값에서 유도한다.
        #   "실제로 의미 있는 최소 점수차"(높이 1셀 = w_h*cell/max_stack)의 1% 를
        #   좌표 전체 범위(W+D)에 나눠 준 값이라, 진짜 점수차를 절대 뒤집지 못한다.
        self.tie_eps = 0.01 * self.w_h * self.cell / self.max_stack / (self.W + self.D)
        self.last_warning = ""                              # 마지막 update 경고 (래퍼가 로그로 올린다)
        self.last_reason = ""                               # "" | YAW | OUTSIDE | CLIPPED | OVER_MAX
        self.plans = {}                                      # (zone,box_id) -> PlacePose
        self.placed = []                                     # [(x,y,z,w,d,h,yaw), ...]
        # ★ 이미 반영한 박스. box_id -> 그때의 ok.
        #   CLAUDE.md §3-4 는 서비스 타임아웃 2.0초 · **재시도 1회** 를 규약으로 못 박았다.
        #   재시도가 오면 update 가 두 번 불린다 — 멱등이 아니면 값이 샌다 (아래 update 주석).
        self.applied = {}

    # ---------- 핵심 유틸 ----------
    def _bad_size(self, w_mm, d_mm, h_mm):
        """치수가 말이 되나. 안 되면 이유 문자열, 되면 "".

        ★ 왜 필요한가: 셀 수가 0 이나 음수가 되면 `maximum_filter1d` 가
          `RuntimeError: incorrect filter size` 로 **예외를 던진다.** 서비스 콜백에서
          예외가 나면 rclpy 는 응답을 안 보내고, 관제탑은 2초 타임아웃 뒤 재시도해
          같은 예외를 또 만난다 — 박스 하나가 라인을 영구히 막는다 (§9·§13).
          "못 놓는다"와 "죽는다"는 다르다. 여기서 걸러 NO_FEASIBLE_POSE 로 답한다.

        실제로 0 이 올 수 있다 — 인식이 실패했거나 `GetBoxInfo` 가 found=false 인데
        관제탑이 기본값 0 을 그대로 보내는 경우다.
        """
        for name, v in (("w", w_mm), ("d", d_mm), ("h", h_mm)):
            if not isinstance(v, (int, float)) or v != v:      # NaN 포함
                return f"{name}={v!r} 가 수가 아니다"
            if v <= 0:
                return f"{name}={v} — 0 이하"
        if w_mm < self.cell or d_mm < self.cell:
            # 셀 격자보다 작으면 점유 셀이 0 개가 되어 필터가 죽는다
            return f"{w_mm}x{d_mm} 가 격자({self.cell}mm)보다 작다"
        return ""

    @staticmethod
    def _sliding_max(a, w, axis):
        """길이 w 윈도의 시작 인덱스별 최대. maximum_filter1d는 중심 기준이므로 슬라이스로 보정."""
        mf = maximum_filter1d(a, size=w, axis=axis, mode="constant", cval=0)
        start = w // 2
        return np.take(mf, np.arange(start, start + a.shape[axis] - w + 1), axis=axis)

    def _ring_max(self, wc, dc):
        """발자국을 ring 셀만큼 부풀린 영역의 최고높이. shape 는 _base_z 와 같다.

        팔레트 **밖**은 0 으로 채운다 — 가장자리 배치에서 "바깥은 낮다"가 되어
        level 항이 벽 쪽을 깎는데, 그건 `w_wall` 이 이미 반대로 밀고 있어 상쇄된다.
        (바깥을 무시하도록 고치면 벽 밀착이 과해져 검증 시드에서 오히려 내려갔다.)
        """
        c = self.ring
        pad = np.pad(self.hm, c, mode="constant", constant_values=0)
        return self._sliding_max(self._sliding_max(pad, wc + 2 * c, 0), dc + 2 * c, 1)

    def _base_z(self, wc, dc):
        """모든 배치 시작셀 (i,j)에 대한 밑면 높이 = 점유영역 최대. shape (W-wc+1, D-dc+1)"""
        return self._sliding_max(self._sliding_max(self.hm, wc, 0), dc, 1)

    def _future_boxes(self, n, salt):
        """굴려 볼 가상의 다음 박스 n개. **결정적이어야 한다.**

        ★ `hash()` 를 쓰면 안 된다 — 파이썬 문자열 해시는 프로세스마다 무작위라
          (PYTHONHASHSEED) 같은 입력에 다른 계획이 나온다. 평가 항목 "재현성"
          (Play/Stop 재현)이 그 자리에서 깨진다. 상태에서 직접 씨앗을 만든다.
        """
        seed = (int(self.hm.sum()) * 1000003 + len(self.placed) * 97 + salt) & 0x7fffffff
        rng = random.Random(seed)
        types = [(b["w_mm"], b["d_mm"], b["h_mm"]) for b in self.box_types]
        return [rng.choices(types, self.box_mix)[0] for _ in range(n)]

    def _rollout_volume(self, hm0, seq):
        """이 heightmap 에서 seq 를 그리디로 채웠을 때 들어간 총 부피(mm³)."""
        sub = HeightmapPacker.__new__(HeightmapPacker)
        sub.__dict__.update(self.__dict__)
        sub.hm = hm0
        sub.plans, sub.placed, sub.applied = {}, [], {}
        sub.la_k = 1                                  # ★ 굴리는 중엔 앞보기를 끈다 (무한재귀 방지)
        vol = 0
        for (w, d, h) in seq:
            ok, pose, _, _ = sub.plan("_LA", w, d, h)
            sub.plans.pop("_LA", None)
            if not ok:
                continue
            ww, dd = (w, d) if pose["yaw_deg"] == 0 else (d, w)
            i = (pose["x_mm"] - ww // 2) // sub.cell
            j = (pose["y_mm"] - dd // 2) // sub.cell
            sub.hm[i:i + ww // sub.cell, j:j + dd // sub.cell] = pose["z_mm"] + h
            vol += w * d * h
        return vol

    def _lookahead(self, cands, h_mm):
        """상위 후보들을 각각 놓아 보고, 뒤에 올 박스를 굴려 **미래 부피**가 큰 쪽을 고른다.

        그리디는 "지금 이 박스에 제일 좋은 자리" 를 고른다. 그런데 그 자리가
        **다음 박스들이 들어올 표면을 망칠 수** 있다 — 기둥이 생기는 이유다 (TS-031).
        실측(시드 30/40 교차검증): 59.9 -> 61.6% · 59.0 -> 61.1%.
        """
        top = cands[: self.la_k]
        if len(top) < 2:
            return cands[0]
        seqs = [self._future_boxes(self.la_depth, r) for r in range(self.la_rollouts)]
        best, best_v = None, None
        for c in top:
            _, i, j, z, yaw, w, d = c
            hm2 = self.hm.copy()
            hm2[i:i + w // self.cell, j:j + d // self.cell] = z + h_mm
            v = sum(self._rollout_volume(hm2.copy(), q) for q in seqs) / len(seqs)
            if best_v is None or v > best_v:
                best, best_v = c, v
        return best

    def _tower_ok(self, cand, bz, h_mm, wc, dc):
        """기둥 억제 3종. 셋 다 꺼져 있으면 `cand` 를 그대로 돌려준다."""
        if not (self.top_box_max_h or self.tower_ring_mm or self.max_span_mm):
            return cand
        top = bz + h_mm
        if self.top_box_max_h and h_mm > self.top_box_max_h:
            cand = cand & (top < self.max_stack)
        if self.tower_ring_mm:
            from scipy.ndimage import minimum_filter
            ring_lo = minimum_filter(self.hm.astype(np.int32), size=self.tower_ring_cells)
            # 발자국 **중심** 칸의 주변 최저와 비교한다 (bz 는 좌상단 기준 배열이다)
            ii, jj = np.indices(bz.shape)
            ci = np.clip(ii + wc // 2, 0, self.W - 1)
            cj = np.clip(jj + dc // 2, 0, self.D - 1)
            cand = cand & (top - ring_lo[ci, cj] <= self.tower_ring_mm)
        if self.max_span_mm:
            cand = cand & (top - int(self.hm.min()) <= self.max_span_mm)
        return cand

    def plan(self, box_id, w_mm, d_mm, h_mm):
        """→ (ok, PlacePose, score, message)"""
        why = self._bad_size(w_mm, d_mm, h_mm)
        if why:
            # ★ 예외로 죽지 않는다. 계약상 message 는 3가지뿐이라(§3-3)
            #   NO_FEASIBLE_POSE 로 답하고 이유는 last_warning 으로 올린다.
            self.last_warning = f"{box_id}: 치수가 잘못됐다 ({why})"
            return False, None, 0.0, "NO_FEASIBLE_POSE"
        # ★ 낙하 대상인지는 **간격을 더하기 전** 실제 치수로 판단한다
        relief = 0
        if self.drop_relief and self.drop_max_kg > 0:
            kg = self._mass_kg.get((w_mm, d_mm, h_mm))
            if kg is not None and kg <= self.drop_max_kg:
                relief = self.drop_relief
        w_mm, d_mm = w_mm + self.plan_clr, d_mm + self.plan_clr   # ★ 계획 간격
        # ★ 예전엔 최선 1개만 남겼다. 지금은 상위 몇 개를 모은다 — 앞보기(_lookahead)가
        #   "이 자리에 놓으면 **다음 박스들**이 어떻게 되나" 를 굴려 보고 고르기 때문이다.
        #   앞보기가 꺼져 있으면 cands[0] 이 예전의 best 와 정확히 같다.
        cands = []
        for yaw in (0, 90):
            w, d = (w_mm, d_mm) if yaw == 0 else (d_mm, w_mm)
            wc, dc = w // self.cell, d // self.cell
            bz = self._base_z(wc, dc)                       # ① 경계는 shape가 처리
            ok_h = bz + h_mm <= self.max_stack              # ② 높이
            # ④ 통로 (전 후보, 필터로 싸다). 떨어뜨릴 수 있는 박스는 벽을 더 봐준다.
            corridor = self._corridor_ok(wc, dc, bz, h_mm, relief)
            cand = ok_h & corridor
            # ⑤ 기둥 억제 (기본 꺼짐). ★ **한 칸도 안 남으면 포기하고 원래 후보를 쓴다.**
            #   이 규칙의 값어치는 "초반에 협곡을 안 만드는" 것이지 "막판에 못 놓게 하는"
            #   게 아니다. 실측(35박스 끝 시점): 3호가 높이·통로를 3038칸 통과하는데
            #   기둥 억제가 **전부** 죽여 버림이 10개씩 났다 (시드 99). 지지율이 상위
            #   50개에서 몰살당할 때 창을 넓히는 것과 같은 처리다 (아래 ③ 주석).
            tow = self._tower_ok(cand, bz, h_mm, wc, dc)
            cand = tow if tow.any() else cand
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
            # ★ 평면 유지: 내 윗면(bz+h)이 이웃 최고높이와 얼마나 맞나. 같으면 1.
            #   `_corridor_ok` 와 같은 팽창 최대필터지만 반경이 다르다 — 저긴 팔 굵기(12셀),
            #   여긴 "붙어 있다고 볼 거리"(4셀). 같은 값을 재사용하면 안 된다.
            level = 1.0 - np.abs(bz + h_mm - self._ring_max(wc, dc)) / self.max_stack
            score = np.where(cand, base_score + self.w_wall * wall + self.w_far * far
                             + self.w_level * level - self.tie_eps * order, -np.inf)
            # ③ 지지율: 점수 상위부터 정밀 계산 (전 후보는 너무 비쌈 — 명세서 §7.9)
            #
            # ★ 2026-08-25. 예전엔 상위 topk(50)개만 보고 거기서 다 떨어지면 **포기**했다.
            #   그게 "자리가 있는데 없다고 답하는" 버그였다. 실측(시드 6, 13개 적재 시점):
            #     3호   높이·통로 4336칸 · 그중 지지까지 통과 803칸  → plan 은 NO_FEASIBLE_POSE
            #     4호   3924칸 · 388칸                              → plan 은 NO_FEASIBLE_POSE
            #     5호   3139칸 ·  65칸                              → plan 은 NO_FEASIBLE_POSE
            #   기둥이 높아지면 점수 상위(= 낮고 벽에 붙은 자리)가 전부 좁은 골짜기라
            #   지지율에서 몰살당한다. 정작 쓸 만한 자리는 51등 밖에 있었다.
            #   실제 시스템에서는 멀쩡히 놓을 수 있는 박스가 리턴 라인으로 되돌려진다 (§7).
            #
            #   그래서 **한 칸도 못 찾았을 때만** 창을 넓힌다. 상위 50개에서 찾으면
            #   비용은 예전과 똑같고, 못 찾을 때만 값을 치른다.
            ranked = np.argsort(score, axis=None)[::-1]
            n_cand = int(cand.sum())
            lo, k = 0, self.topk
            found = False
            while lo < n_cand:
                for idx in ranked[lo:k]:
                    i, j = np.unravel_index(idx, score.shape)
                    if not cand[i, j]: continue
                    sup = self._support_ratio(i, j, wc, dc, int(bz[i, j]))
                    if sup < self.min_sup: continue
                    found = True
                    s = (self.w_h * (1 - bz[i, j] / self.max_stack) + self.w_s * sup
                         + self.w_wall * wall[i, j] + self.w_far * far[i, j]
                         + self.w_level * level[i, j])
                    key = (s, -order[i, j])                  # 동점 → 위 order 기준
                    cands.append((key, int(i), int(j), int(bz[i, j]), yaw, w, d))
                if found:
                    break                                    # 이 창 안에서 최선을 골랐다
                lo, k = k, min(k * 4, n_cand)                # 빈손이면 넓혀서 다시
        if not cands:
            # ③ 이 박스만 못 놓은 건지(NO_FEASIBLE_POSE), 팔레트가 정말 끝난 건지(PALLET_FULL) 구분
            return False, None, 0.0, "PALLET_FULL" if self._is_pallet_full() else "NO_FEASIBLE_POSE"
        cands.sort(key=lambda t: t[0], reverse=True)
        best = self._lookahead(cands, h_mm) if self.la_k > 1 else cands[0]
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
        # ★ 여기도 간격을 더한다. 안 더하면 "가장 작은 박스는 들어간다" 로 오판해
        #   PALLET_FULL 을 영영 안 내고 NO_FEASIBLE_POSE 만 반복한다.
        c = self.plan_clr
        for w, d in ((t["w_mm"] + c, t["d_mm"] + c), (t["d_mm"] + c, t["w_mm"] + c)):
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
        """발자국 셀 중 **밑면 높이와 같은 높이**인 칸의 비율 (0~1).

        "밑면이 얼마나 받쳐지나" 다. `min_support_ratio`(0.7) 미만이면 후보에서 뺀다 —
        외팔보처럼 걸쳐 놓으면 팔이 손을 떼는 순간 기운다.

        ★ `tol` 은 반드시 `cell_mm` 과 같아야 한다 (둘 다 10). 높이는 셀 격자로
          양자화돼 저장되므로 같은 평면이어도 한 칸(10mm)까지 어긋날 수 있다.
          5 로 줄이면 **2층부터 전부 지지율 미달**이 되어 `NO_FEASIBLE_POSE` 만 나온다
          (CLAUDE.md §10 의 증상표에 있는 그 항목이다).
        """
        foot = self.hm[i:i + wc, j:j + dc]
        return float((np.abs(foot - base_z) <= self.tol).mean())   # ★ tol=10 필수

    def _corridor_ok(self, wc, dc, bz, h_mm, relief=0):
        """주변 '팔 굵기' 만큼을 보되, 기준은 박스 **윗면 + 그리퍼 길이** 다.

        ★ 2026-08-24 수정. 구 코드는 반경 30mm 링을 박스 **밑면**(bz)과 비교했다.
          그게 두 가지를 동시에 망쳤다:

          1) **너무 빡빡하다.** 박스는 자기보다 높은 이웃 **옆에 미끄러져 들어가도**
             아무 문제가 없다. 그런데 밑면 기준은 그걸 금지한다. 그래서
             `gripper_clearance_mm` 을 올리면 협곡은 그대로인 채 적재율만 떨어졌다
             (실측 clr 30 -> 60: 협곡 2.55 -> 2.50, 적재율 -4.6%p).
          2) **정작 부딪히는 걸 못 본다.** 실제로 닿는 것은 박스 윗면 **위**를 지나는
             팔뚝(link_4, 길이 594mm)인데, 밑면 기준으로는 그 높이가 시야에 없다.
             실측: 팔레트 한가운데 바닥 배치에서 link_4 가 -165mm 파고들었고,
             그 자세는 IK 해 4개 전부 동일해 **기구학으로는 못 피한다**.

          박스 윗면부터 `arm_gap`(그리퍼 길이 150mm)까지는 그리퍼 몸통(Ø245)만 지나가고,
          그 몸통은 **모든 박스 발자국 안에 들어간다** (최소 3호 340x250 의 반폭 125 >
          몸통 반경 122.5). 그래서 그 구간은 이웃이 높아도 안 닿는다. 팔뚝은 그 위부터다.
        """
        c = self.arm_r
        # ★ 먼 쪽은 툴 **뒤**다 (2026-08-25 트랙 B 관찰). 팔은 py 0 쪽에서 들어오므로
        #   팔뚝이 지나는 공간은 박스의 로봇 쪽 위이고, 먼 쪽에 아무리 높이 쌓여 있어도
        #   툴이 이미 지나온 뒤라 안 걸린다. 대칭으로 보면 **먼 쪽까지 괜히 금지**해서
        #   적재율만 깎는다. `corridor_far_ratio` 로 먼 쪽 여유폭을 줄인다 (기본 1.0 = 기존).
        cf = int(round(c * self.corridor_far))
        pad = np.pad(self.hm, ((c, c), (c, cf)), mode="constant", constant_values=0)
        big = self._sliding_max(self._sliding_max(pad, wc + 2 * c, 0), dc + c + cf, 1)
        ok = big <= bz + h_mm + self.arm_gap + relief + self.tol
        # ★ 2단 — 더 넓은 반경은 **더 높은 벽까지만** 봐준다. 얕은 이웃은 1단만 걸린다.
        if self.arm_r2 > c:
            c2 = self.arm_r2
            cf2 = int(round(c2 * self.corridor_far))
            pad2 = np.pad(self.hm, ((c2, c2), (c2, cf2)), mode="constant", constant_values=0)
            big2 = self._sliding_max(self._sliding_max(pad2, wc + 2 * c2, 0),
                                     dc + c2 + cf2, 1)
            ok &= big2 <= bz + h_mm + self.arm_gap2 + self.tol
        if self.near_gap < self.arm_gap:
            # 박스 **앞**(로봇 쪽) c 셀만 따로 본다. 발자국은 빼고 순수하게 앞쪽이다 —
            #   팔이 넘어와야 하는 벽이 거기 있다.
            padn = np.pad(self.hm, ((c, c), (c, 0)), mode="constant", constant_values=0)
            near = self._sliding_max(self._sliding_max(padn, wc + 2 * c, 0), c, 1)
            ok &= near[:, : self.D - dc + 1] <= bz + h_mm + self.near_gap + self.tol
        return ok

    def update(self, box_id, w_mm, d_mm, h_mm, actual):
        """★ 실측 pose로 갱신 — 이게 자기교정의 핵심 (명세서 §7.6)

        ★★ **멱등이어야 한다.** CLAUDE.md §3-4 가 서비스 재시도 1회를 규약으로 정했으므로
        같은 box_id 로 두 번 들어오는 일이 정상 동작 중에 생긴다. 멱등이 아니면:

          · `self.placed` 에 두 번 쌓여 **적재 개수와 부피가 늘고 적재율이 부풀려진다**
          · `self.plans.pop` 이 두 번째엔 None 을 줘서 **yaw 검사가 통째로 건너뛰어진다**
            (planned 가 없으면 dyaw=0 이라 아무리 틀어져 있어도 통과한다)

        `self.hm` 만은 `np.maximum` 이라 원래 멱등이었다 — 그래서 heightmap 은 멀쩡한데
        숫자만 새는, 로그로 알아채기 어려운 형태로 나타난다.
        """
        why = self._bad_size(w_mm, d_mm, h_mm)
        if why:
            self.last_warning = f"{box_id}: 치수가 잘못됐다 ({why}) — 갱신하지 않는다"
            return False, self.fill_rate(), len(self.placed)
        w_mm, d_mm = w_mm + self.plan_clr, d_mm + self.plan_clr   # ★ plan 과 동일하게

        prev = self.applied.get(box_id)
        if prev is not None:
            # 재시도로 본다. 계획은 이미 회수됐고 hm 도 이미 반영됐다.
            # ok 는 **처음 판정을 그대로** 돌려주고, 개수·적재율만 현재 값으로 준다
            # (그 사이 다른 박스가 들어왔을 수 있다).
            self.last_warning = f"{box_id}: 이미 반영된 박스다 — 재시도로 보고 무시했다"
            return prev, self.fill_rate(), len(self.placed)

        planned = self.plans.pop(box_id, None)               # ★ box_id 로 계획을 회수 (없으면 None)
        ok = True
        self.last_warning = ""
        # ★ ok=False 의 **사유**를 남긴다. 사유가 없으면 통합할 때 "왜 거절됐지" 를
        #   로그만 보고 못 찾는다 — yaw 초과·경계 이탈·높이 초과가 다 같은 false 다.
        self.last_reason = ""
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
            self.last_warning = (f"{box_id}: 실측 yaw {yaw:.1f}° 가 계획 "
                                 f"{planned[0]['yaw_deg']:.0f}° 에서 {dyaw:.1f}° 벗어났다 "
                                 f"(허용 {self.yaw_tol}°) — AABB {w}x{d} 로 기록")
            self.last_reason = "YAW"
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
            self.last_reason = "OVER_MAX"
            ok = False

        # ② 실측 좌표가 배열 밖으로 나갈 때 안전하게 자르기.
        #    자르지 않으면 음수 인덱스 슬라이스가 되어 엉뚱한 칸을 덮어쓴다.
        i0, i1 = max(i, 0), min(i + wc, self.W)
        j0, j1 = max(j, 0), min(j + dc, self.D)
        if i0 >= i1 or j0 >= j1:                             # 통째로 팔레트 밖 — 기록할 것이 없다
            self.last_warning = (f"{box_id}: 실측 ({actual['x_mm']},{actual['y_mm']}) 이 "
                                 f"팔레트 밖이다 — 기록하지 않는다")
            self.last_reason = "OUTSIDE"
            # ★ 여기도 이 박스에 대한 **최종 판정**이다. applied 에 남겨야 재시도가
            #   같은 답을 받는다 (안 남기면 재시도 때 아래 경로로 새로 들어간다).
            self.applied[box_id] = False
            return False, self.fill_rate(), len(self.placed)
        if (i0, i1, j0, j1) != (i, i + wc, j, j + dc):       # 일부만 걸쳐 있다 = 이미 이상 상황
            if not self.last_reason:
                self.last_warning = (f"{box_id}: 실측 ({actual['x_mm']},{actual['y_mm']}) 에서 "
                                     f"박스가 팔레트 가장자리에 걸쳤다 — 걸친 만큼만 기록")
                self.last_reason = "CLIPPED"
            ok = False

        self.hm[i0:i1, j0:j1] = np.maximum(self.hm[i0:i1, j0:j1], top_q)
        self.placed.append((actual, w_mm, d_mm, h_mm))
        self.applied[box_id] = ok
        return ok, self.fill_rate(), len(self.placed)

    def cancel(self, box_id):
        """계획만 버린다. `hm` 은 안 건드린다 — 아직 아무것도 안 놓았기 때문이다.

        관제탑의 **이탈 3경로**(`NO_FEASIBLE_POSE` · `PALLET_FULL` · 실패/타임아웃)가
        전부 여기로 온다 (CLAUDE.md §8). 안 부르면 `plans` 에 stale plan 이 남고,
        그 박스가 리턴 라인을 돌아 다시 왔을 때 `update` 가 **옛 계획과 yaw 를 비교**해
        멀쩡한 배치를 거절한다.

        없는 box_id 여도 조용히 넘어간다 — 서비스 재시도 1회가 규약이라 두 번 들어온다.
        """
        self.plans.pop(box_id, None)

    def reset(self):
        """팔레트를 갈았다. 상태 넷을 **다 같이** 비운다 (`/algo/reset_heightmap`)."""
        # ★ applied 도 같이 비운다. 안 비우면 팔레트를 갈고 나서 같은 box_id 가
        #   다시 와도 "이미 반영됨" 으로 무시된다.
        self.hm[:] = 0
        self.plans.clear()
        self.placed.clear()
        self.applied.clear()

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
