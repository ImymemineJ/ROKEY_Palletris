#!/usr/bin/env python3
"""H2017 충돌 구체 생성기 — 충돌 메시(.dae)에서 직접 유도한다.

M0609 descriptor 의 구체 좌표는 재사용할 수 없다. 상완 0.411 -> 0.845 m,
전완 0.368 -> 0.734 m 로 링크가 약 2배여서 스케일해도 맞지 않는다 (사양서 6-3).
그래서 손으로 찍지 않고 메시에서 뽑는다.

방법
  1. 링크별 충돌 메시 표면에서 점을 뽑는다.
     COLLADA / Z_UP / mm 단위이므로 URDF 의 scale 0.001 을 그대로 적용한다.
     - .dae 하나에 geometry 가 여러 개 들어 있으므로 전부 누적한다
       (하나만 읽으면 상완 튜브를 통째로 놓친다 — 실제로 H2017_2_1 이 그렇다).
     - 정점만 쓰면 안 된다. 상완 튜브는 긴 삼각형 몇 장으로 되어 있어 중간
       구간에 정점이 하나도 없다. 그대로 두면 팔 중간이 충돌 판정에서 빈다.
       그래서 큰 삼각형은 표면을 격자로 샘플링해 점을 채운다.
  2. PCA(멱승법)로 링크의 주축을 구한다. 전완처럼 링크 프레임에 대해
     비스듬히 놓인 링크도 축이 자동으로 잡힌다.
  3. 주축을 균등 분할하고, 구간마다
       중심 = (주축 위 구간 중점) + (구간 정점의 수직 방향 AABB 중심)
       반지름 = 그 중심에서 구간 정점까지의 최대 거리
     로 놓는다. 정점을 전부 감싸므로 과소 판정이 나지 않는다.
     무게중심을 쓰면 정점 밀도가 높은 하우징 쪽으로 구체가 쏠려 튜브 구간에
     구멍이 생긴다. 그래서 균등 분할을 쓴다.
  4. 가늘고 긴 링크가 아니면(종횡비 < MIN_ASPECT) 분할하지 않고 통째로 감싼다.

사용법
    python3 gen_collision_spheres.py [충돌메시_폴더] > spheres.yaml
"""
import math
import sys

import _paths                      # 경로 해결 (scripts/_paths.py)
import xml.etree.ElementTree as ET
from pathlib import Path

NS = "{http://www.collada.org/2005/11/COLLADASchema}"
SCALE = 0.001      # URDF 의 mesh scale 과 같은 값
MIN_ASPECT = 1.3   # 이 아래는 분할하지 않고 통째로 감싼다
MARGIN = 0.001     # 반지름 여유 1 mm. YAML 을 소수 4자리로 적으므로 반올림으로
                   # 표면점이 구체 밖으로 나가는 것을 막는다 (verify_spheres.py 가 잡는다)

# 링크 -> (충돌 메시 접미사, 구체 간격 m)
LINKS = [
    ("base_link", ["0_0"], 0.13),
    ("link_1", ["1_0"], 0.13),
    ("link_2", ["2_0", "2_1", "2_2"], 0.16),
    ("link_3", ["3_0"], 0.11),
    ("link_4", ["4_0", "4_1"], 0.15),
    ("link_5", ["5_0"], 0.10),
    ("link_6", ["6_0"], 0.07),
]


SAMPLE_STEP = 0.025  # 이보다 긴 변을 가진 삼각형은 표면을 채워 샘플링한다
MAX_SUBDIV = 8

# ---------------------------------------------------------- 흡착 그리퍼 (tool0)
# tool0 에는 충돌 메시가 없다. 그리퍼는 URDF 가 아니라 Isaac 씬에서 붙였기 때문이다.
# 그래서 위 유도 절차를 못 쓰고 **치수를 적어 구체를 계산한다.**
#
# 출처: usd/h2017_cell.usd 실측 (2026-08-21) + 사양서 §4-4.
#   그리퍼 프레임은 tool0 와 원점·자세가 일치하고, 그리퍼는 tool0 의 **+z** 로 뻗는다.
#   /World/gripper/body  Cylinder r 0.0900 h 0.1500 axis Z  -> tool0 z 0 ~ 0.150
#   /World/gripper/pads  4개, tool0 z +0.1500, (+-0.080, +-0.055) = 반경 0.0971
#   패드 지름은 사양서 §4-4 의 **Ø50 mm**.
#
# 2026-08-21: 몸통을 0.090 -> 0.1225 로 키웠다. 패드 중심이 반경 0.0971 이라
# 옛 몸통(0.090)은 자기 패드를 지탱하지 못하는 치수였다. 이제 패드 테두리
# 0.1221 이 몸통 0.1225 **안**에 들어오므로 패드를 따로 덮을 필요가 없다.
#
# ⚠ 상한이 있다. 몸통 밑면이 패드 접촉면과 같은 평면이라 **박스 상판에 얹힌다.**
#   가장 작은 3호 박스(340x250) 반폭이 y 로 0.125 이므로 그 밖으로 나가면
#   옆에 쌓인 박스 윗면을 올라탄다. 즉 0.1221 <= r <= 0.125 구간뿐이다.
#   원형 판이라 이렇게 빡빡하다 — 사각 판(210x160)이면 y 여유가 45 mm 로 벌어진다.
GRIPPER_BODY_R = 0.1225       # 몸통 원기둥 반지름 (USD /World/gripper/body 와 같아야 한다)
GRIPPER_LEN = 0.150           # tool0 -> 패드 접촉면
GRIPPER_BODY_N = 6            # 몸통을 덮을 구체 개수 (아래 주석 참조)
PAD_R = 0.025                 # 패드 반지름 (사양서 §4-4 Ø50)
PAD_XY = [(0.080, 0.055), (0.080, -0.055), (-0.080, 0.055), (-0.080, -0.055)]


def tool0_spheres():
    """그리퍼를 덮는 구체 목록. [(center, radius), ...]

    몸통 — 반지름 r · 길이 L 인 원기둥을 축 위 구체 n 개로 덮는 최소 반지름은
    `sqrt(r^2 + (L/2n)^2)` 다 (각 구간 끝 모서리가 가장 먼 점).

    패드 — 접촉면 z=L 위의 원판이므로 같은 중심 · 반지름 PAD_R 인 구체가 최소다.
    원판을 구체로 덮는 이상 접촉면 **아래로 PAD_R 만큼 튀어나오는 건 못 피한다.**

    n 을 왜 6 으로 두는가. 몸통이 r 0.1225 x L 0.15 로 **짧고 뚱뚱해서**
    (종횡비 0.6) 구체와 궁합이 나쁘다. n 을 키울수록 반경은 조이고 축방향은 늘어난다:
      n=1 : 반경 0.1436   n=2 : 0.1281   n=4 : 0.1239   n=6 : 0.1231
    반경 상한은 3호 박스 반폭 **0.125** 다 — 넘으면 모델이 박스 밖으로 나가
    옆에 쌓인 박스와 충돌한다고 판정한다. 여유(MARGIN)까지 얹고 그 아래로
    떨어지는 최소값이 6 이다 (n=4 는 반올림 후 정확히 0.1250 이라 여유가 0).

    축방향으로 길어지는 건 괜찮다. 접촉면 아래로 나오는 부분은 툴 축 근처라
    **지금 들고 있는 박스 안쪽**을 향한다 (반경이 0.125 를 안 넘으므로 항상 박스 안).
    """
    half = GRIPPER_LEN / (2 * GRIPPER_BODY_N)
    r_body = math.hypot(GRIPPER_BODY_R, half)
    out = [([0.0, 0.0, half * (2 * i + 1)], r_body) for i in range(GRIPPER_BODY_N)]

    # 패드가 몸통 밖으로 나올 때만 따로 덮는다. 몸통을 키우면 알아서 빠진다.
    pad_rim = max(math.hypot(x, y) for x, y in PAD_XY) + PAD_R
    if pad_rim > GRIPPER_BODY_R + 1e-9:
        out += [([x, y, GRIPPER_LEN], PAD_R) for x, y in PAD_XY]
    return out


def _floats(elem):
    return [float(x) for x in elem.text.split()]


def _tri_indices(prim):
    """triangles / polylist 에서 VERTEX 인덱스만 뽑는다."""
    inputs = prim.findall(NS + "input")
    stride = max(int(i.get("offset", 0)) for i in inputs) + 1
    off = next(
        int(i.get("offset", 0)) for i in inputs if i.get("semantic") == "VERTEX"
    )
    p = [int(x) for x in prim.find(NS + "p").text.split()]
    idx = p[off::stride]

    vcount = prim.find(NS + "vcount")
    if vcount is None:  # <triangles>
        return [idx[i:i + 3] for i in range(0, len(idx) - 2, 3)]

    tris, pos = [], 0  # <polylist> — 팬으로 삼각형화
    for c in (int(x) for x in vcount.text.split()):
        face = idx[pos:pos + c]
        pos += c
        for k in range(1, c - 1):
            tris.append([face[0], face[k], face[k + 1]])
    return tris


def _sample_triangle(a, b, c, out):
    e = max(
        sum((u[i] - v[i]) ** 2 for i in range(3)) ** 0.5
        for u, v in ((a, b), (b, c), (c, a))
    )
    n = min(MAX_SUBDIV, max(1, int(e / SAMPLE_STEP)))
    if n == 1:
        out.append([(a[i] + b[i] + c[i]) / 3 for i in range(3)])
        return
    for i in range(n + 1):
        for j in range(n + 1 - i):
            k = n - i - j
            out.append(
                [(i * a[m] + j * b[m] + k * c[m]) / n for m in range(3)]
            )


def load_surface_points(path):
    """메시 표면 점 구름을 만든다 (정점 + 큰 삼각형의 내부 샘플)."""
    root = ET.parse(path).getroot()
    pts = []
    for geo in root.iter(NS + "geometry"):
        mesh = geo.find(NS + "mesh")
        if mesh is None:
            continue
        src = {s.get("id"): s for s in mesh.findall(NS + "source")}
        vmap = {}
        for v in mesh.findall(NS + "vertices"):
            inp = next(
                i for i in v.findall(NS + "input") if i.get("semantic") == "POSITION"
            )
            vmap[v.get("id")] = inp.get("source").lstrip("#")

        for prim in mesh:
            tag = prim.tag.replace(NS, "")
            if tag not in ("triangles", "polylist"):
                continue
            vin = next(
                i for i in prim.findall(NS + "input") if i.get("semantic") == "VERTEX"
            )
            pos_id = vmap[vin.get("source").lstrip("#")]
            fa = _floats(src[pos_id].find(NS + "float_array"))
            verts = [
                (fa[i] * SCALE, fa[i + 1] * SCALE, fa[i + 2] * SCALE)
                for i in range(0, len(fa), 3)
            ]
            pts += verts
            for t in _tri_indices(prim):
                _sample_triangle(verts[t[0]], verts[t[1]], verts[t[2]], pts)

    if not pts:
        sys.exit(f"표면 점을 만들지 못했다: {path}")
    return pts


def principal_axis(pts, mean):
    """멱승법으로 최대 고윳값의 고유벡터를 구한다. numpy 없이 돈다."""
    cov = [[0.0] * 3 for _ in range(3)]
    for p in pts:
        d = [p[i] - mean[i] for i in range(3)]
        for i in range(3):
            for j in range(3):
                cov[i][j] += d[i] * d[j]
    v = [1.0, 0.3, 0.7]
    for _ in range(300):
        w = [sum(cov[i][j] * v[j] for j in range(3)) for i in range(3)]
        n = sum(x * x for x in w) ** 0.5
        if n < 1e-12:
            return [0.0, 0.0, 1.0]
        v = [x / n for x in w]
    return v


def enclosing_sphere(pts):
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    c = [(lo[i] + hi[i]) / 2 for i in range(3)]
    r = max(sum((p[i] - c[i]) ** 2 for i in range(3)) ** 0.5 for p in pts)
    return c, r


def spheres_for(pts, spacing):
    n = len(pts)
    mean = [sum(p[i] for p in pts) / n for i in range(3)]
    axis = principal_axis(pts, mean)

    t = [sum((p[i] - mean[i]) * axis[i] for i in range(3)) for p in pts]
    lo_t, hi_t = min(t), max(t)
    length = hi_t - lo_t

    # 수직 방향 두께로 종횡비를 본다
    perp = []
    for p, tv in zip(pts, t):
        d = [p[i] - mean[i] - axis[i] * tv for i in range(3)]
        perp.append(sum(x * x for x in d) ** 0.5)
    thickness = 2 * max(perp)
    if length < MIN_ASPECT * thickness:
        return [enclosing_sphere(pts)]

    k = max(2, min(8, int(round(length / spacing))))
    bins = [[] for _ in range(k)]
    for p, tv in zip(pts, t):
        b = int((tv - lo_t) / length * k)
        bins[min(b, k - 1)].append(p)

    out = []
    for b, sel in enumerate(bins):
        if len(sel) < 3:
            continue
        t_mid = lo_t + length * (b + 0.5) / k
        base = [mean[i] + axis[i] * t_mid for i in range(3)]
        # 수직 성분만 AABB 중심으로 보정한다 (축 방향은 균등 분할 위치를 지킨다)
        off = []
        for p in sel:
            d = [p[i] - base[i] for i in range(3)]
            dt = sum(d[i] * axis[i] for i in range(3))
            off.append([d[i] - axis[i] * dt for i in range(3)])
        cor = [
            (min(o[i] for o in off) + max(o[i] for o in off)) / 2 for i in range(3)
        ]
        c = [base[i] + cor[i] for i in range(3)]
        r = max(sum((p[i] - c[i]) ** 2 for i in range(3)) ** 0.5 for p in sel)
        out.append((c, r))
    return out


def emit(mesh_dir):
    """YAML 조각을 문자열로 만든다."""
    lines = ["collision_spheres:"]
    total = 0
    for link, tags, spacing in LINKS:
        pts = []
        for tag in tags:
            f = mesh_dir / f"H2017_{tag}.dae"
            if not f.exists():
                sys.exit(f"메시 없음: {f}")
            pts += load_surface_points(f)
        sph = spheres_for(pts, spacing)
        total += len(sph)
        lines.append(f"    - {link}:")
        for c, r in sph:
            r = math.ceil((r + MARGIN) * 1e4) / 1e4
            lines.append(f'        - "center": [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]')
            lines.append(f'          "radius": {r:.4f}')
        lines.append("")

    # tool0 — 흡착 그리퍼. 치수는 위 상수에 있다.
    # ⚠ 실물 치수가 바뀌면 위 상수와 rmpflow 의 body_collision_controllers 를 같이 고친다.
    sph = tool0_spheres()
    lines.append("    - tool0:")
    for c, r in sph:
        r = math.ceil((r + MARGIN) * 1e4) / 1e4
        lines.append(f'        - "center": [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]')
        lines.append(f'          "radius": {r:.4f}')
    lines.append("")
    total += len(sph)
    print(f"# 구체 총 {total}개", file=sys.stderr)
    return "\n".join(lines)


def main():
    default = (
        _paths.MESH_COLLISION
    )
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    mesh_dir = (Path(args[0]) if args else default).resolve()
    if not mesh_dir.is_dir():
        sys.exit(f"충돌 메시 폴더를 찾을 수 없다: {mesh_dir}")

    body = emit(mesh_dir)

    if "--update" in sys.argv:
        desc = _paths.DESCRIPTOR
        text = desc.read_text()
        head = text.split("collision_spheres:")[0]
        desc.write_text(head + body)
        print(f"갱신: {desc}")
    else:
        print(body)


if __name__ == "__main__":
    main()
