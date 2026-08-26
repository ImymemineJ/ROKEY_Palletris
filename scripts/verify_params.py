# -*- coding: utf-8 -*-
"""드라이버와 **똑같은 방식**으로 cfg 를 만들고 코어에 걸리는 값을 찍는다."""
import sys, importlib.util, os
sys.path.insert(0, "scripts")
os.environ["PL_CONFIG_OVERLAY"] = "src/packing_bringup/config/test_3.yaml"
import _paths
cfg = _paths.system_config()
# 드라이버의 오버레이 병합과 같은 절차
import yaml
ovl = yaml.safe_load(open(os.environ["PL_CONFIG_OVERLAY"], encoding="utf-8"))
def deep(a, b):
    for k, v in b.items():
        if isinstance(v, dict) and isinstance(a.get(k), dict): deep(a[k], v)
        else: a[k] = v
deep(cfg, ovl)

sp = importlib.util.spec_from_file_location(
    "_hm", "src/packing_algo/packing_algo/heightmap_core.py")
m = importlib.util.module_from_spec(sp); sys.modules["_hm"] = m; sp.loader.exec_module(m)
pk = m.HeightmapPacker(cfg)

print("%-26s %-14s %s" % ("코어 속성", "실제 값", "판정"))
print("-"*56)
def chk(name, got, want):
    ok = str(got) == str(want)
    print("%-26s %-14s %s" % (name, got, "OK" if ok else "★ 다르다 (기대 %s)" % want))
    return ok
allok = True
allok &= chk("la_k (앞보기 k)",        pk.la_k, 4)
allok &= chk("la_rollouts",            getattr(pk, "la_rollouts", "?"), 3)
allok &= chk("la_depth",               getattr(pk, "la_depth", "?"), 8)
allok &= chk("w_level",                pk.w_level, 0.05)
allok &= chk("ring (level_ring_cells)",pk.ring, 4)
allok &= chk("tower_ring_mm",          pk.tower_ring_mm, 1000)
allok &= chk("tower_ring_cells",       pk.tower_ring_cells, 45)
allok &= chk("drop_relief",            pk.drop_relief, 200)
allok &= chk("drop_max_kg",            pk.drop_max_kg, 3.5)
allok &= chk("plan_clr",               pk.plan_clr, 10)
allok &= chk("min_sup",                pk.min_sup, 0.65)
allok &= chk("max_stack",              pk.max_stack, 1300)
allok &= chk("팔레트 셀 W x D",         "%dx%d" % (pk.W, pk.D), "80x120")
print()
print("RESULT:", "전부 걸렸다" if allok else "★ 안 걸린 값이 있다")
