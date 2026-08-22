#!/usr/bin/env python3
"""H2017 정기구학 — URDF 원점만으로 계산한다 (numpy · ROS 없이 돈다).

Lula descriptor 의 default_q 를 고르고, 사양서 3-2 도달범위를 책상에서
1차 확인하는 용도다. 실측 검증은 Isaac Sim 에서 따로 한다.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

# packing_robot/packing_robot/fk.py -> packing_robot/urdf/h2017.urdf
URDF = (Path(__file__).resolve().parent / "../urdf/h2017.urdf").resolve()

CHAIN = [
    ("joint_1", "base_link", "link_1"),
    ("joint_2", "link_1", "link_2"),
    ("joint_3", "link_2", "link_3"),
    ("joint_4", "link_3", "link_4"),
    ("joint_5", "link_4", "link_5"),
    ("joint_6", "link_5", "link_6"),
    ("joint_6-tool0", "link_6", "tool0"),
]


def rpy2R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def axis2R(axis, q):
    x, y, z = axis
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def app(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def load_joints(urdf=URDF):
    root = ET.parse(urdf).getroot()
    out = {}
    for j in root.iter("joint"):
        o = j.find("origin")
        a = j.find("axis")
        lim = j.find("limit")
        out[j.get("name")] = {
            "xyz": [float(x) for x in o.get("xyz").split()] if o is not None else [0, 0, 0],
            "rpy": [float(x) for x in o.get("rpy").split()] if o is not None else [0, 0, 0],
            "axis": [float(x) for x in a.get("axis" if a.get("axis") else "xyz").split()]
            if a is not None
            else [0, 0, 1],
            "type": j.get("type"),
            "lower": float(lim.get("lower")) if lim is not None else None,
            "upper": float(lim.get("upper")) if lim is not None else None,
        }
    return out


def fk(q, joints=None):
    """q = joint_1..6 (rad). 링크 이름 -> (R, p) 를 돌려준다."""
    joints = joints or load_joints()
    T = {"base_link": ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])}
    qi = 0
    for jn, par, ch in CHAIN:
        j = joints[jn]
        Rp, pp = T[par]
        R = mul(Rp, rpy2R(*j["rpy"]))
        p = [pp[i] + app(Rp, j["xyz"])[i] for i in range(3)]
        if j["type"] == "revolute":
            R = mul(R, axis2R(j["axis"], q[qi]))
            qi += 1
        T[ch] = (R, p)
    return T


def check_limits(q, joints=None):
    joints = joints or load_joints()
    bad = []
    for i, jn in enumerate([f"joint_{k}" for k in range(1, 7)]):
        lo, hi = joints[jn]["lower"], joints[jn]["upper"]
        if not (lo <= q[i] <= hi):
            bad.append(f"{jn}={q[i]:+.4f} 범위 [{lo:+.4f}, {hi:+.4f}] 밖")
    return bad


if __name__ == "__main__":
    import sys

    q = [float(x) for x in sys.argv[1:7]] if len(sys.argv) >= 7 else [0.0] * 6
    bad = check_limits(q)
    T = fk(q)
    for ln in ["link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "tool0"]:
        p = T[ln][1]
        print(f"  {ln:8s} ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})")
    p = T["tool0"][1]
    print(f"  수평거리 {math.hypot(p[0], p[1]):.4f} m · 높이 {p[2]:+.4f} m")
    for b in bad:
        print("  ⚠", b)
