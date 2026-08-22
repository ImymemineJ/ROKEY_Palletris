#!/usr/bin/env python3
"""충돌 구체가 실제 메시를 덮는지 검사한다.

gen_collision_spheres.py 가 만든 구체는 눈으로 확인할 수 없다. 하나라도 빈
구간이 있으면 RMPflow 가 그 부분을 장애물로 보지 않아 팔이 적재된 박스를
그대로 통과한다. 그래서 표면 점이 전부 구체 안에 들어가는지 수치로 확인한다.

    python3 verify_spheres.py [descriptor.yaml]

판정: 미포함 점 0 개 · 인접 구체가 끊기지 않을 것.
"""
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths  # noqa: E402,F401  경로 해결 + fk import 준비
from gen_collision_spheres import (  # noqa: E402
    LINKS, load_surface_points,
    GRIPPER_BODY_R, GRIPPER_LEN, PAD_R, PAD_XY,
)

BOX3_HALF_Y = 0.125   # 3호 박스(340x250) 반폭. 모델 반경이 이걸 넘으면 옆 박스와 충돌 판정

HERE = Path(__file__).resolve().parent
MESH = _paths.MESH_COLLISION


def gripper_surface(n_ax=41, n_th=72, n_rad=9):
    """흡착 그리퍼 표면 점. 메시가 없으므로 치수에서 직접 만든다.

    몸통 원기둥(옆면 + 양 끝면)과 접촉면 위 패드 원판 4개를 채운다.
    좌표계는 tool0 — 그리퍼는 +z 로 뻗고 접촉면이 z=GRIPPER_LEN 이다.
    """
    import math
    pts = []
    for i in range(n_ax):
        z = GRIPPER_LEN * i / (n_ax - 1)
        for j in range(n_th):
            a = 2 * math.pi * j / n_th
            pts.append((GRIPPER_BODY_R * math.cos(a), GRIPPER_BODY_R * math.sin(a), z))
    for z in (0.0, GRIPPER_LEN):                       # 끝면
        for k in range(n_rad):
            rr = GRIPPER_BODY_R * k / (n_rad - 1)
            for j in range(n_th):
                a = 2 * math.pi * j / n_th
                pts.append((rr * math.cos(a), rr * math.sin(a), z))
    for px, py in PAD_XY:                              # 패드 원판 (사양서 4-4 D50)
        for k in range(n_rad):
            rr = PAD_R * k / (n_rad - 1)
            for j in range(n_th):
                a = 2 * math.pi * j / n_th
                pts.append((px + rr * math.cos(a), py + rr * math.sin(a), GRIPPER_LEN))
    return pts


def check_tool0(sph):
    """tool0 구체가 그리퍼를 덮는지. 메시가 없어 위 루프가 건너뛰는 유일한 링크다.

    여기가 새면 팔이 옆에 쌓인 박스를 스치고 지나간다. 눈으로는 안 보인다.
    """
    import math
    print()
    if not sph:
        print("  tool0      ✗ 구체 없음")
        return False

    pts = gripper_surface()
    worst, miss = 0.0, 0
    for p in pts:
        d = min(sum((p[i] - c[i]) ** 2 for i in range(3)) ** 0.5 - r for c, r in sph)
        if d > 1e-9:
            miss += 1
            worst = max(worst, d)

    env_r = max(math.hypot(c[0], c[1]) + r for c, r in sph)
    env_z = max(c[2] + r for c, r in sph)
    pad_rim = max(math.hypot(x, y) for x, y in PAD_XY) + PAD_R
    real_r = max(GRIPPER_BODY_R, pad_rim)          # 몸통과 패드 중 바깥쪽
    good = miss == 0
    print(f"  tool0      {'✓' if good else '✗'} 구체 {len(sph)}개 · 표면점 {len(pts):6d} · "
          f"미포함 {miss}개 · 최대 초과 {worst * 1000:.1f} mm")
    print(f"             실물 최대 반경 {real_r * 1000:.1f} mm -> 모델 {env_r * 1000:.1f} mm "
          f"(여유 {(env_r - real_r) * 1000:+.1f})")
    print(f"             접촉면 {GRIPPER_LEN * 1000:.0f} mm -> 모델 축방향 끝 "
          f"{env_z * 1000:.1f} mm (돌출 {(env_z - GRIPPER_LEN) * 1000:+.1f})")
    print(f"             몸통 {GRIPPER_BODY_R * 1000:.1f} · 패드 테두리 {pad_rim * 1000:.1f}"
          f"   -> 바깥쪽 {real_r * 1000:.1f} mm")
    if env_r < real_r - 1e-9:
        print("             ⚠ 모델이 실물보다 가늘다 — 그리퍼가 새어 나간다")
        good = False
    if pad_rim > GRIPPER_BODY_R + 1e-9:
        print(f"             ⚠ 패드가 몸통 밖에 있다 ({pad_rim * 1000:.1f} > "
              f"{GRIPPER_BODY_R * 1000:.1f}) — 씬의 몸통이 패드를 못 지탱한다")
    if env_r > BOX3_HALF_Y + 1e-9:
        print(f"             ⚠ 모델 반경 {env_r * 1000:.1f} > 3호 박스 반폭 "
              f"{BOX3_HALF_Y * 1000:.1f} mm — 배치 때 옆 박스와 충돌로 판정된다")
        good = False
    return good


def main():
    desc = Path(sys.argv[1]) if len(sys.argv) > 1 else _paths.DESCRIPTOR
    data = yaml.safe_load(desc.read_text())
    spheres = {}
    for entry in data["collision_spheres"]:
        for link, sph in entry.items():
            spheres[link] = [(s["center"], s["radius"]) for s in sph]

    ok = True
    for link, tags, _ in LINKS:
        pts = []
        for t in tags:
            pts += load_surface_points(MESH / f"H2017_{t}.dae")
        sph = spheres.get(link, [])
        if not sph:
            print(f"  {link:10s} ✗ 구체 없음")
            ok = False
            continue

        worst = 0.0
        miss = 0
        for p in pts:
            d = min(
                sum((p[i] - c[i]) ** 2 for i in range(3)) ** 0.5 - r for c, r in sph
            )
            if d > 1e-9:
                miss += 1
                worst = max(worst, d)
        mark = "✓" if miss == 0 else "✗"
        if miss:
            ok = False
        print(
            f"  {link:10s} {mark} 구체 {len(sph)}개 · 표면점 {len(pts):6d} · "
            f"미포함 {miss}개 · 최대 초과 {worst * 1000:.1f} mm"
        )

        # 인접 구체가 끊기는지 (체인 링크만 의미가 있다)
        for i in range(len(sph) - 1):
            (c1, r1), (c2, r2) = sph[i], sph[i + 1]
            gap = sum((c1[k] - c2[k]) ** 2 for k in range(3)) ** 0.5 - (r1 + r2)
            if gap > 0:
                print(f"      ⚠ 구체 {i}-{i+1} 사이 {gap * 1000:.1f} mm 끊김")
                ok = False

    ok &= check_tool0(spheres.get("tool0", []))

    print("\n판정:", "통과 — 메시 전체가 구체 안에 들어간다" if ok else "실패 — 위 항목을 고칠 것")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
