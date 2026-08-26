# -*- coding: utf-8 -*-
"""인계 문서 §6-5 검증 — 격자 · plan/update 왕복."""
import sys, os, importlib.util, yaml
sys.path.insert(0, "scripts")
os.environ["PL_CONFIG_OVERLAY"] = "src/packing_bringup/config/test_3.yaml"
import _paths
cfg = _paths.system_config()
ovl = yaml.safe_load(open(os.environ["PL_CONFIG_OVERLAY"], encoding="utf-8"))
def deep(a,b):
    for k,v in b.items():
        if isinstance(v,dict) and isinstance(a.get(k),dict): deep(a[k],v)
        else: a[k]=v
deep(cfg, ovl)
sp=importlib.util.spec_from_file_location("_hm","src/packing_algo/packing_algo/heightmap_core.py")
m=importlib.util.module_from_spec(sp); sys.modules["_hm"]=m; sp.loader.exec_module(m)

pk = m.HeightmapPacker(cfg)
pal = cfg["pallet"]
print("팔레트(씬) %d x %d mm · 여유 ov=%d" % (pal["width_mm"], pal["depth_mm"], pk.ov))
print("계획 격자   W x D = %d x %d  (여유 없으면 %d x %d)"
      % (pk.W, pk.D, pal["width_mm"]//pk.cell, pal["depth_mm"]//pk.cell))
ok1 = (pk.W, pk.D) == (81, 121) or (pk.W, pk.D) == (121, 81)
print("  -> %s\n" % ("OK (한 변씩 +1 셀)" if ok1 else "★ 기대와 다르다"))

# 왕복: plan 한 자리를 그대로 update 에 넣으면 같은 셀로 돌아와야 한다
print("=== plan -> update 왕복 ===")
allok = True
for (w,d,h) in ((340,250,210),(410,310,280),(480,380,340)):
    ok,pose,_s,msg = pk.plan("T%d"%w, w,d,h)
    if not ok:
        print("  %dx%d  plan 실패 %s" % (w,d,msg)); allok=False; continue
    before = pk.hm.copy()
    ok2,_fr,_n = pk.update("T%d"%w, w,d,h, dict(pose))
    moved = int((pk.hm != before).sum())
    exp = ((w+pk.plan_clr)//pk.cell) * ((d+pk.plan_clr)//pk.cell)
    if int(round(abs(pose["yaw_deg"])/90.0)) % 2 == 1:
        exp = ((d+pk.plan_clr)//pk.cell) * ((w+pk.plan_clr)//pk.cell)
    good = ok2 and moved > 0
    allok &= good
    print("  %3dx%3d yaw%3.0f  pose=(%5d,%5d,%4d)  update=%s  갱신셀 %d (발자국 %d)  %s"
          % (w,d,pose["yaw_deg"],pose["x_mm"],pose["y_mm"],pose["z_mm"],
             ok2, moved, exp, "OK" if good else "★"))
neg = [p for p in ("x_mm","y_mm")]
print("\n※ 가장자리 자리는 x_mm/y_mm 이 -%d 까지 음수로 나오는 것이 정상이다" % pk.ov)
print("\nRESULT:", "통과" if (ok1 and allok) else "★ 확인 필요")
