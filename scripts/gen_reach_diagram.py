#!/usr/bin/env python3
"""도달범위 측면도(`docs/가이드/img/reach_side.svg`)를 **system.yaml 에서 생성**한다.

    python3 scripts/gen_reach_diagram.py

## 왜 생성하나

이 그림은 손으로 그려져 있었고, 그래서 페데스탈·`max_stack`·팔레트 좌표가 바뀔 때마다
조용히 거짓말이 됐다 (실제로 페데스탈 500 · max_stack 850 인 채로 남아 있었다).
축척 도면이라 라벨만 고치면 그림과 숫자가 어긋난다. 그래서 config 에서 뽑는다.

## 어느 단면인가

셀 90° 회전 이후 **로봇 정면이 +Y** 이고 픽업 벨트는 **−X** 쪽이다.
두 방향이 직각이라 한 장의 측면도로 둘 다 보여줄 수 없다.
여기서는 **Y-Z 단면**(로봇 베이스를 지나 −X 를 보는 방향)을 그린다 — G1 의 최악점이
팔레트 먼 코너 상단이므로 그쪽이 판정면이다. 픽업은 면 밖이라 주석으로만 표시한다.
표기 거리는 원래 도면과 같은 규약으로 **X 성분까지 포함한 3D 값**이다.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths                                    # noqa: E402

OUT = _paths.ROOT / "docs" / "가이드" / "img" / "reach_side.svg"
PX = 170.0                                       # 1 m = 170 px (원래 도면과 같은 축척)
CAT = 1700.0                                     # 카탈로그 reach (tool0 까지)


def main():
    cfg = _paths.system_config()
    zone = "B"
    base = cfg["zones"]["robot_base_world_mm"][zone]
    org = cfg["zones"]["pallet_origin_world_mm"][zone]
    pal = cfg["pallet"]
    grip = cfg["robot"]["gripper_length_mm"]
    app = cfg["robot"]["approach_offset_mm"]
    ped = cfg["robot"]["pedestal_height_mm"]
    belt = cfg["conveyor"]["belt_top_z_mm"]
    stack = pal["max_stack_mm"]
    thk = pal["thickness_mm"]
    yard = -cfg.get("truck", {}).get("bed_height_mm", 850)
    eff = CAT - grip                             # 흡착면 기준 실효 reach

    # 단면 좌표 (mm) — 가로 = 월드 Y (베이스 기준 상대), 세로 = 월드 Z
    y0, y1 = org[1] - base[1], org[1] + pal["depth_mm"] - base[1]   # 팔레트 근/원변
    top = org[2] + stack                                            # 적재 상단
    face = top + app                                                # 최악점 흡착면
    d3 = math.hypot(math.hypot(pal["width_mm"] / 2.0, y1), face - base[2])

    # 뷰포트 — 여백 포함
    YL, YR = -1900.0, 1900.0
    ZB, ZT = yard - 250.0, face + 450.0
    W, H = (YR - YL) / 1000.0 * PX + 120, (ZT - ZB) / 1000.0 * PX + 90
    OX, OZ = 60.0 - YL / 1000.0 * PX, H - 50.0 + ZB / 1000.0 * PX

    def sx(y_mm):
        return OX + y_mm / 1000.0 * PX

    def sy(z_mm):
        return OZ - z_mm / 1000.0 * PX

    def rect(y_a, y_b, z_a, z_b, fill, stroke="#374151", op=1.0):
        return ('  <rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s" '
                'fill-opacity="%.2f" stroke="%s" stroke-width="1"/>\n'
                % (sx(y_a), sy(z_b), sx(y_b) - sx(y_a), sy(z_a) - sy(z_b), fill, op, stroke))

    def text(y_mm, z_mm, s, anchor="start", size=12, fill="#374151", dy=0, weight="normal"):
        return ('  <text x="%.1f" y="%.1f" text-anchor="%s" font-size="%d" fill="%s" '
                'font-weight="%s">%s</text>\n'
                % (sx(y_mm), sy(z_mm) + dy, anchor, size, fill, weight, s))

    def line(y_a, z_a, y_b, z_b, stroke, w=1.0, dash=""):
        d = ' stroke-dasharray="%s"' % dash if dash else ""
        return ('  <line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" '
                'stroke-width="%.1f"%s/>\n' % (sx(y_a), sy(z_a), sx(y_b), sy(z_b), stroke, w, d))

    s = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.0f" height="%.0f" '
         'viewBox="0 0 %.0f %.0f" font-family="sans-serif">\n' % (W, H, W, H)]
    s.append('  <rect width="%.0f" height="%.0f" fill="#ffffff"/>\n' % (W, H))
    s.append('  <text x="16" y="26" font-size="14" font-weight="bold" fill="#111827">'
             '도달범위 측면도 (Y-Z 단면) — G1 이 검증하는 것 · 축척 1m = %.0fpx</text>\n' % PX)
    s.append('  <text x="16" y="44" font-size="11" fill="#6b7280">'
             'scripts/gen_reach_diagram.py 가 system.yaml 에서 생성한다 — 손으로 고치지 말 것</text>\n')

    # 도달 원호 (베이스 중심). 흡착면 기준 실효 reach.
    s.append('  <path d="M %.1f %.1f A %.1f %.1f 0 0 1 %.1f %.1f" fill="none" '
             'stroke="#2563eb" stroke-width="1.2" stroke-dasharray="6 4"/>\n'
             % (sx(-eff), sy(base[2]), eff / 1000.0 * PX, eff / 1000.0 * PX,
                sx(eff), sy(base[2])))
    s.append(text(-eff + 60, base[2] + 120, "실효 reach %.0f = %.0f − 그리퍼 %.0f"
                  % (eff, CAT, grip), size=11, fill="#2563eb"))

    # 야드 지면 · 도크 바닥
    s.append(line(YL + 60, yard, YR - 60, yard, "#9ca3af", 1.0, "4 4"))
    s.append(text(YL + 70, yard, "야드 지면 z=%.0f (truck.bed_height_mm)" % yard,
                  size=11, fill="#9ca3af", dy=-6))
    s.append(line(YL + 60, 0, YR - 60, 0, "#111827", 2.0))
    s.append(text(YL + 70, 0, "★ z=0  도크 바닥 = 트럭 적재함 바닥 = 팔레트 밑면",
                  size=12, fill="#111827", dy=-7, weight="bold"))

    # 페데스탈 · 베이스
    s.append(rect(-200, 200, 0, ped, "#d1d5db"))
    s.append(text(-230, ped / 2.0, "페데스탈 %.0f" % ped, anchor="end", size=12))
    s.append('  <circle cx="%.1f" cy="%.1f" r="4" fill="#111827"/>\n' % (sx(0), sy(base[2])))
    s.append(text(-230, ped, "베이스 (%d, %d, %d)" % tuple(base), anchor="end", size=11, dy=-8))

    # 팔레트 · 적재
    s.append(rect(y0, y1, 0, thk, "#a16207", op=0.85))
    s.append(rect(y0, y1, thk, top, "#fcd34d", op=0.5))
    s.append(text((y0 + y1) / 2.0, thk + (top - thk) / 2.0,
                  "최대 적재 %.0f" % stack, anchor="middle", size=12))
    s.append(text(y1 + 40, thk / 2.0, "팔레트 (상면 z=%.0f)" % org[2], size=11))

    # 픽업 벨트 — 면 밖이라 점선으로만
    s.append(line(YL + 60, belt, -350, belt, "#6b7280", 1.2, "5 3"))
    s.append(text(YL + 70, belt, "픽업 벨트 z=%.0f  (면 밖 — −X 쪽)" % belt,
                  size=11, fill="#6b7280", dy=-6))

    # 최악점
    s.append(line(y1, top, y1, face, "#dc2626", 1.0, "3 3"))
    s.append('  <circle cx="%.1f" cy="%.1f" r="4.5" fill="#dc2626"/>\n' % (sx(y1), sy(face)))
    s.append(line(0, base[2], y1, face, "#dc2626", 1.4))
    s.append(text(y1 + 40, face, "최악점 = 먼 코너 상단 + approach %.0f" % app,
                  size=12, fill="#dc2626", weight="bold"))
    s.append(text(y1 + 40, face, "3D 거리 %.0f mm  →  여유 %+.0f mm" % (d3, eff - d3),
                  size=12, fill="#dc2626", dy=16))

    s.append('  <text x="16" y="%.0f" font-size="11" fill="#6b7280">'
             '※ 측면 투영도 — 표기 거리는 X 성분까지 포함한 3D 값이다. '
             'z=0 을 야드 지면으로 잡으면 같은 요구에 페데스탈 1,580 mm 이상이 필요해진다 '
             '(구 사양서 §5-1 이 기각).</text>\n' % (H - 14))
    s.append("</svg>\n")

    OUT.write_text("".join(s), encoding="utf-8")
    print("wrote %s" % OUT)
    print("  페데스탈 %.0f · 팔레트 상면 %.0f · 적재 상단 %.0f · 최악점 3D %.0f (여유 %+.0f)"
          % (ped, org[2], top, d3, eff - d3))
    return 0


if __name__ == "__main__":
    sys.exit(main())
