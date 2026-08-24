# packing_common/units.py — 전원 import. 다른 곳에서 직접 나누기 금지
MM_PER_CELL = 10
def mm_to_m(v_mm: int) -> float:   return v_mm / 1000.0
def m_to_mm(v_m: float) -> int:    return int(round(v_m * 1000))
def mm_to_cell(v_mm: int) -> int:  return v_mm // MM_PER_CELL
def cell_to_mm(c: int) -> int:     return c * MM_PER_CELL
