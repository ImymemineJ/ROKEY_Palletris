"""트랙 C 수용 테스트 (C1).

실행:
    cd ~/palletris_ws && source install/setup.bash
    python3 -m pytest src/packing_algo/test/test_core.py -v
"""
import random
import time

from packing_algo.heightmap_core import HeightmapPacker
from packing_common.config import load

CFG = load()          # ★ colcon build + source install/setup.bash 가 선행돼야 한다
                      #   (packing_common 을 import 하려면 빌드가 먼저다)

BOXES = [(340, 250, 210), (410, 310, 280), (480, 380, 340)]

SEED_BOX = 42         # 박스 등장 순서 — config 의 random_seed 와 동일
SEED_NOISE = 7        # 실측 오차 전용


def run_sequence(pk, n=15, noise_mm=0):
    """★ 난수 스트림을 박스용 / 노이즈용으로 분리한다.

    하나를 공유하면 random.randint 가 난수를 더 소비해서, 노이즈를 켜는 순간
    박스 등장 순서까지 바뀐다. 그러면 이 테스트는 '노이즈 내성'이 아니라
    '다른 박스 시퀀스'를 재게 되어, 원인을 오판하게 만든다. (트러블슈팅 TS-006)
    """
    rb, rn = random.Random(SEED_BOX), random.Random(SEED_NOISE)
    placed = 0
    for k in range(n):
        w, d, h = rb.choice(BOXES)
        ok, pose, _, msg = pk.plan(f"B{k}", w, d, h)
        if not ok:
            return placed, msg
        actual = dict(pose)                                    # 계획 = 실측 + 노이즈
        actual["x_mm"] += rn.randint(-noise_mm, noise_mm)
        actual["y_mm"] += rn.randint(-noise_mm, noise_mm)
        actual["z_mm"] += rn.randint(-noise_mm, noise_mm)
        pk.update(f"B{k}", w, d, h, actual)
        placed += 1
    return placed, "OK"


def test_no_noise():
    """기본 동작 — 15박스 중 12개 이상."""
    pk = HeightmapPacker(CFG)
    n, msg = run_sequence(pk)
    assert n >= 12, f"{n}개에서 {msg} (트러블슈팅 TS-008 참고)"


def test_noise_3mm():
    """★ 수용 기준: ±3mm 오차를 주입해도 12개 이상 (명세서 §7.4)."""
    pk = HeightmapPacker(CFG)
    n, msg = run_sequence(pk, noise_mm=3)
    assert n >= 12, f"{n}개에서 {msg} — support_tol 확인!"


def test_noise_no_regression():
    """★ 노이즈 '내성' 자체를 재는 테스트 — ±3mm 를 줘도 결과가 나빠지지 않아야 한다.

    위 두 테스트는 절대 개수를 보므로 알고리즘 성능(TS-008)에 가려진다.
    이 테스트는 노이즈만 분리해서 본다. 실패하면 실측 pose 처리(양자화·반올림)가 깨진 것.
    """
    clean, _ = run_sequence(HeightmapPacker(CFG), noise_mm=0)
    noisy, msg = run_sequence(HeightmapPacker(CFG), noise_mm=3)
    assert noisy >= clean, f"노이즈로 {clean}개 → {noisy}개로 줄었다 ({msg}) — TS-005 확인!"


def test_speed():
    """★ 수용 기준: 40% 찬 상태에서 plan() 응답 < 200ms (명세서 §7.9)."""
    pk = HeightmapPacker(CFG)
    run_sequence(pk, n=8)
    t0 = time.time()
    pk.plan("SPD", 480, 380, 340)
    dt = time.time() - t0
    assert dt < 0.2, f"plan 이 {dt * 1000:.0f}ms 걸렸다 — sliding_window_view 썼는지 확인"
