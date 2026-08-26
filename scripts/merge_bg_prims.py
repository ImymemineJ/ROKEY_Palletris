"""`test_3_1.usd` 에서 **배경 prim 만** 골라 `test_3.usd` 로 옮긴다.

    ~/isaacsim/python.sh scripts/merge_bg_prims.py      (pxr 경로를 잡아 주고)

★ 왜 통째로 안 쓰나. 배경을 넣은 세션이 `demo_stack15_test1_1.py` 였다. 그 스테이지에는
  드라이버가 런타임에 만든 것이 올라가 있다 — `/World/boxes/BOX000xx`,
  `PalletShipped_*`, `/World/ROS2WristCam`, 그리고 **로봇 3 대의 집던 관절 자세**.
  통째로 저장하면 그게 전부 씬에 굳는다. 그래서 새로 생긴 prim 만 집어 온다.

★ `Sdf.CopySpec` 으로 **레이어 대 레이어**로 옮긴다. 스테이지를 열어 복사하면
  페이로드가 풀린 결과가 따라와 파일이 부풀고, 원격 참조가 절대경로로 굳는다.
"""
import os
import sys

from pxr import Sdf

SRC = os.environ.get("PL_BG_SRC",
                     "/home/rokey/isaac_scenes/Collected_test_3/test_3_1.usd")
DST = os.environ.get("PL_STAGE",
                     "/home/rokey/isaac_scenes/Collected_test_3/test_3.usd")

src = Sdf.Layer.FindOrOpen(SRC)
dst = Sdf.Layer.FindOrOpen(DST)
if src is None or dst is None:
    sys.exit("레이어를 못 열었다: %s / %s" % (SRC, DST))

s_kids = {c.name for c in src.GetPrimAtPath("/World").nameChildren}
d_kids = {c.name for c in dst.GetPrimAtPath("/World").nameChildren}
new = sorted(s_kids - d_kids)
print("옮길 prim %d 개: %s" % (len(new), new))
if not new:
    sys.exit("옮길 게 없다")

for n in new:
    path = Sdf.Path("/World/" + n)
    Sdf.CreatePrimInLayer(dst, path)
    if not Sdf.CopySpec(src, path, dst, path):
        sys.exit("복사 실패: %s" % path)
    spec = dst.GetPrimAtPath(path)
    pay = [i.assetPath for i in (spec.payloadList.GetAddedOrExplicitItems() or [])]
    print("   %-34s payload=%s" % (n, pay or "-"))

dst.Save()
print("\n저장: %s" % DST)

# ── 검증: 다시 읽어서 확인 ──
chk = Sdf.Layer.FindOrOpen(DST)
got = {c.name for c in chk.GetPrimAtPath("/World").nameChildren}
miss = [n for n in new if n not in got]
print("검증: /World 자식 %d 개 · 누락 %s" % (len(got), miss or "없음"))
# 기존 작업이 살아 있나
for keep in ("ROS2Bridge", "rsd455_a", "rsd455_b", "rsd455_c",
             "rsd455_1", "rsd455_2"):
    print("   %-14s %s" % (keep, "OK" if keep in got else "★ 사라졌다"))
