"""트랙 C 수용 테스트 (C1).

실행:
    cd ~/palletris_ws && source install/setup.bash
    python3 -m pytest src/packing_algo/test/test_core.py -v
"""
import random
import time

import numpy as np

from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load

CFG = load()          # ★ colcon build + source install/setup.bash 가 선행돼야 한다

BOXES = [(b["w_mm"], b["d_mm"], b["h_mm"]) for b in CFG["boxes"]["types"]]
N = CFG["boxes"]["stage1"]["total_count"]

SEED_BOX = 42         # 박스 등장 순서 — config 의 random_seed 와 동일
SEED_NOISE = 7        # 실측 오차 전용


def run_sequence(pk, n=N, noise_mm=0):
    """박스를 하나씩 흘려보내며 배치. → (배치 수, 버린 수, 종료 사유)

    ★ 두 실패 신호의 의미가 다르다 (트랙 D 설계변경 A1):
      - NO_FEASIBLE_POSE = '이 박스만' 자리가 없다 → 그 박스만 버리고 다음 박스로 계속.
        실제 시스템도 cancel_plan → set_status(REJECTED) → despawn_box 후 계속 돈다.
      - PALLET_FULL      = 가장 낮은 박스로도 후보가 없다 → 시나리오 종료.
      첫 NO_FEASIBLE_POSE 에서 멈추면 적재율을 크게 과소평가한다 (49.8% vs 60.2%).

    ★ 난수 스트림을 박스용 / 노이즈용으로 분리한다. 하나를 공유하면 노이즈를 켜는 순간
      박스 등장 순서까지 바뀌어 '노이즈 내성'이 아니라 '다른 시퀀스'를 재게 된다. (TS-006)
    """
    rb, rn = random.Random(SEED_BOX), random.Random(SEED_NOISE)
    placed = skipped = 0
    for k in range(n):
        w, d, h = rb.choice(BOXES)
        box_id = f"B{k}"
        ok, pose, _, msg = pk.plan(box_id, w, d, h)
        if not ok:
            if msg == "PALLET_FULL":
                return placed, skipped, msg
            pk.cancel(box_id)          # ★ 이탈 경로에서는 반드시 계획 폐기 (stale plan 방지)
            skipped += 1
            continue
        actual = dict(pose)                                    # 계획 = 실측 + 노이즈
        actual["x_mm"] += rn.randint(-noise_mm, noise_mm)
        actual["y_mm"] += rn.randint(-noise_mm, noise_mm)
        actual["z_mm"] += rn.randint(-noise_mm, noise_mm)
        pk.update(box_id, w, d, h, actual)
        placed += 1
    return placed, skipped, "OK"


def test_no_noise():
    """기본 동작 — 투입 대비 80% 이상 적재."""
    pk = HeightmapPacker(CFG)
    placed, skipped, msg = run_sequence(pk)
    assert placed >= N * 0.8, f"{N}개 중 {placed}개만 적재 (버림 {skipped}, {msg})"


def test_noise_3mm():
    """★ 수용 기준: ±3mm 실측 오차를 주입해도 동일 수준 (명세서 §7.4)."""
    pk = HeightmapPacker(CFG)
    placed, skipped, msg = run_sequence(pk, noise_mm=3)
    assert placed >= N * 0.8, f"{placed}개 적재 (버림 {skipped}, {msg}) — support_tol 확인!"


def test_noise_no_regression():
    """★ 노이즈 '내성' 자체 — ±3mm 를 줘도 결과가 나빠지지 않아야 한다.

    위 두 테스트는 절대 개수를 보므로 알고리즘 성능에 가려진다. 이건 노이즈만 분리해서 본다.
    실패하면 실측 pose 처리(양자화·반올림)가 깨진 것이다. (TS-005)
    """
    clean, _, _ = run_sequence(HeightmapPacker(CFG), noise_mm=0)
    noisy, _, msg = run_sequence(HeightmapPacker(CFG), noise_mm=3)
    assert noisy >= clean, f"노이즈로 {clean}개 → {noisy}개로 줄었다 ({msg}) — TS-005 확인!"


def stack_depth(pk):
    """한 자리에 박스가 최대 몇 개나 겹쳐 쌓였는가 = 실제 '층수'.

    hm.max() // 210 으로 세면 안 된다. 높이가 섞여 있어서(210/280/340)
    1020mm 가 3호 4층일 수도, 5호 3층일 수도 있다. 박스 개수를 직접 세야 한다.
    """
    cnt = np.zeros(pk.hm.shape, dtype=int)
    for pose, w, d, _h in pk.placed:
        ww, dd = (w, d) if round(pose["yaw_deg"] / 90) % 2 == 0 else (d, w)
        i = int(round((pose["x_mm"] - ww / 2) / pk.cell))
        j = int(round((pose["y_mm"] - dd / 2) / pk.cell))
        i0, i1 = max(i, 0), min(i + ww // pk.cell, pk.W)
        j0, j1 = max(j, 0), min(j + dd // pk.cell, pk.D)
        cnt[i0:i1, j0:j1] += 1
    return int(cnt.max())


def test_stacks_5_layers():
    """수용 기준: ±3mm 오차 상태에서 여러 층 적재 (명세서 §7.4).

    ★★ 2026-08-24 기준을 5층 → **3층** 으로 내렸다. **수용기준 변경이다** — 정정인/팀
      사후 동의가 필요하다 (PR #15, CLAUDE.md §12 "C1 수용기준 재개").

      옛 근거는 `3호 210mm × 5 = 1050 ≤ max_stack 1300` 이었다. 그런데 그 계산은
      **팔이 그 자리에 도달할 수 있다는 전제**를 깔고 있었고, 실측이 그 전제를 깼다:

        · 팔레트 한가운데 바닥 배치에서 팔뚝(link_4, 길이 594mm)이 이웃 기둥을
          **-165mm 파고든다**. IK 해 4개 전부 동일해 기구학으로는 못 피한다
          (무작위 시드 600회 전수 탐색)
        · 그래서 `_corridor_ok` 가 그런 자리를 거부하게 됐고(같은 날 수정),
          아래층 빈자리를 못 채우게 되면서 층수가 줄었다

      즉 **알고리즘이 나빠진 것이 아니라, 도달 불가능한 자리를 세지 않게 된 것**이다.
      옛 5층은 실행하면 팔이 적재를 무너뜨리는 층수였다.

      ⚠ 이 값을 다시 올리려면 `_corridor_ok` 를 되돌리는 것이 아니라 **기구학**을
        바꿔야 한다 (그리퍼 연장·툴 기울기·이격은 전부 재봤고 안 됐다 — PR #15).

    ★ 투입 30개로 돌린다. config 의 15개로는 팔레트 바닥이 다 안 차서 층이 안 올라간다 —
      알고리즘 문제가 아니라 박스가 모자란 것이다. (트랙 D 설계변경 A2 의 근거)
    """
    pk = HeightmapPacker(CFG)
    placed, skipped, msg = run_sequence(pk, n=30, noise_mm=3)
    layers = stack_depth(pk)
    assert layers >= 3, f"{layers}층 (적재 {placed}, 버림 {skipped}, {msg}) — 3층 필요"


def test_pallet_full_is_terminal():
    """★ PALLET_FULL 은 '가장 낮은 박스로도 못 놓을 때'만 나와야 한다 (트랙 D A1)."""
    pk = HeightmapPacker(CFG)
    pk.hm[:] = pk.max_stack                       # 팔레트를 꽉 채운 상태로 만든다
    assert pk.plan("X", 340, 250, 210)[3] == "PALLET_FULL"
    pk.reset()
    pk.hm[:] = pk.max_stack - 250                 # 3호(210)는 들어가고 5호(340)는 안 되는 높이
    assert pk.plan("Y", 480, 380, 340)[3] == "NO_FEASIBLE_POSE"


def test_speed():
    """★ 수용 기준: 40% 찬 상태에서 plan() 응답 < 200ms (명세서 §7.9)."""
    pk = HeightmapPacker(CFG)
    run_sequence(pk, n=max(1, N // 2))
    t0 = time.time()
    pk.plan("SPD", 480, 380, 340)
    dt = time.time() - t0
    assert dt < 0.2, f"plan 이 {dt * 1000:.0f}ms 걸렸다 — sliding_window_view 썼는지 확인"


def test_capacity_rate_is_monotonic():
    """★ 용량 기준 적재율(보조 지표)은 **절대 내려가면 안 된다**.

    분모가 max_stack 고정이므로 박스를 놓을 때마다 분자만 는다. 여기가 깨지면
    분모를 실수로 실제 높이로 바꿔 놓은 것이다 (= fill_rate 와 같아져 버린다).
    또 fill_rate 와 달리 max_stack 을 넘길 수 없으므로 1.0 을 못 넘는다.
    """
    pk = HeightmapPacker(CFG)
    rb = random.Random(SEED_BOX)
    prev = 0.0
    for k in range(30):
        w, d, h = rb.choice(BOXES)
        ok, pose, _, _ = pk.plan(f"B{k:03d}", w, d, h)
        if not ok:
            pk.cancel(f"B{k:03d}")
            continue
        pk.update(f"B{k:03d}", w, d, h, dict(pose))
        cur = pk.capacity_rate()
        assert cur >= prev - 1e-12, f"{k}번째에서 {prev:.4f} → {cur:.4f} 로 내려갔다"
        assert cur <= 1.0, f"용량 기준이 {cur:.3f} — max_stack 을 넘을 수 없다"
        prev = cur
    # ★★ 2026-08-24 하한을 0.5 -> 0.35 로 내렸다. **수용기준 변경** — 위
    #   test_stacks_5_layers 의 docstring 과 같은 이유다 (도달 불가능한 자리를 안 세게 됐다).
    #   실측 41.2%. 0.35 는 그 아래 약간의 여유다 — 이 하한의 목적은 "분모를 실수로
    #   실제 높이로 바꿔 놓았나" 를 잡는 것이지 적재 성능 판정이 아니다 (위 monotonic
    #   단언이 본체다). 성능은 데스크 지표가 아니라 궤적 스윕으로 봐야 한다.
    assert prev > 0.35, f"30개 넣고 용량 기준 {prev * 100:.1f}% — 너무 낮다"


def test_update_is_idempotent():
    """★ 서비스 재시도로 같은 박스가 두 번 와도 값이 새면 안 된다 (CLAUDE.md §3-4).

    타임아웃 2.0초 · 재시도 1회가 규약이므로 이 상황은 **정상 동작 중에 생긴다.**
    멱등이 아니면 적재 개수·적재율이 부풀려지고, 두 번째 호출에서는 계획이 이미
    회수돼 있어 yaw 검사까지 건너뛴다.
    """
    pk = HeightmapPacker(CFG)
    ok, pose, _, _ = pk.plan("B001", 340, 250, 210)
    assert ok

    ok1, fill1, n1 = pk.update("B001", 340, 250, 210, dict(pose))
    hm1 = pk.hm.copy()
    ok2, fill2, n2 = pk.update("B001", 340, 250, 210, dict(pose))     # 재시도

    assert (ok2, n2) == (ok1, n1), f"재시도로 {n1} → {n2} 개가 됐다"
    assert fill2 == fill1, f"재시도로 적재율이 {fill1:.4f} → {fill2:.4f} 로 변했다"
    assert np.array_equal(pk.hm, hm1), "재시도가 heightmap 을 바꿨다"

    # 다른 박스는 정상적으로 들어가야 한다 (전부 막아버린 게 아님을 확인)
    ok3, pose3, _, _ = pk.plan("B002", 340, 250, 210)
    assert ok3
    _, _, n3 = pk.update("B002", 340, 250, 210, dict(pose3))
    assert n3 == n1 + 1, "멱등 처리가 다른 박스까지 막았다"


def test_reset_clears_applied():
    """★ 팔레트를 갈면(PALLET_FULL → SwapPallet) 같은 box_id 가 다시 들어갈 수 있어야 한다."""
    pk = HeightmapPacker(CFG)
    ok, pose, _, _ = pk.plan("B001", 340, 250, 210)
    assert ok
    pk.update("B001", 340, 250, 210, dict(pose))
    pk.reset()

    ok2, pose2, _, _ = pk.plan("B001", 340, 250, 210)
    assert ok2
    _, _, n = pk.update("B001", 340, 250, 210, dict(pose2))
    assert n == 1, f"reset 후에도 '이미 반영됨' 으로 무시됐다 (적재 {n}개)"


def test_bad_sizes_never_raise():
    """★ 치수가 잘못돼도 **예외로 죽으면 안 된다** (§9 입력 검증 · §13 예외 처리).

    rclpy 서비스 콜백에서 예외가 나면 응답이 안 나가고, 관제탑은 2초 뒤 재시도해
    같은 예외를 또 만난다 — 박스 하나가 라인을 영구히 막는다.
    실제로 0 이 올 수 있다: 인식 실패 또는 GetBoxInfo found=false 인데 기본값 0 전송.
    (수정 전에는 scipy 가 RuntimeError: incorrect filter size 로 죽었다.)
    """
    pose = dict(x_mm=0, y_mm=0, z_mm=0, yaw_deg=0.0)
    bad = [(0, 0, 0), (-340, -250, -210), (5, 5, 210), (340, 250, 0),
           (float("nan"), 250, 210), (340, float("nan"), 210)]
    for w, d, h in bad:
        pk = HeightmapPacker(CFG)
        ok, _, _, msg = pk.plan("BAD", w, d, h)          # 여기서 예외가 나면 실패
        assert ok is False, f"{(w, d, h)} 가 통과됐다"
        assert msg == "NO_FEASIBLE_POSE", f"{(w, d, h)} → {msg} (계약은 3가지뿐)"
        assert pk.last_warning, f"{(w, d, h)} 에 이유가 안 남았다"
        assert pk.update("BAD", w, d, h, pose)[0] is False
        assert len(pk.placed) == 0, "잘못된 치수가 적재로 세어졌다"

    # 정상 치수는 그대로 통과해야 한다 (검증이 과하지 않은지)
    pk = HeightmapPacker(CFG)
    assert pk.plan("OK1", 340, 250, 210)[0] is True


def test_zone_lock_exists():
    """★ 구역별 락이 있어야 한다 — 관제탑 1개가 A·B·C 를 겹쳐 굴린다 (§8).

    노드를 띄우지 않고 클래스 소스만 본다 (rclpy init 없이 도는 단위 테스트).
    """
    import inspect

    from packing_algo import algo_service_node as m
    src = inspect.getsource(m.AlgoService)
    assert "self.lock = {" in src, "구역별 락이 없다"
    for cb in ("def on_plan", "def on_update"):
        i = src.index(cb)
        body = src[i:i + 1400]
        assert "with self.lock[" in body, f"{cb} 이 락 없이 패커를 만진다"
