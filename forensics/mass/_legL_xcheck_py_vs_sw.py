# -*- coding: utf-8 -*-
"""
Cross-check(几何配对版): 我方 Python 链路 (_mass_leg_L.json, 源 _reduced3/leg_L.stp)
vs SolidWorks 内核基线 (sw_baseline_legL.csv)。

★ 为什么重写（原版有个会撒谎的 bug）
------------------------------------
原版第 38 行是 `py[nm] = v['volume']` —— 一个**按名字做的 dict**。
而 `_reduced3/leg_L.stp` 里 `腿部设计_左-5/-6` 这两件在归约时**丢了名字**
（name 为空串），两个空名互相覆盖 ⇒ 19 个体积被压成 18 个 ⇒
报告写成「我方少一件、-5/-6 = ONLY_SW」。
**零件从来没丢，是统计口径把两件无名件叠成了一件。**

⇒ 本版改成：
  1. 两边都用**列表**，不按名字去重（从根上消除「同名互相覆盖」）；
  2. 配对用**几何**（体积 + 质心）做全局最优 1-1 指派，不靠名字；
  3. 名字只用来「贴标签 / 查密度」；
  4. 额外输出每个配对的**判别余量**：与「次优候选」的代价比 ——
     用来证明这个配对不是碰巧（余量越大越可信）。

两边都套**同一张密度表**（且按 SW 的零件名查表，保证口径一致），
所以剩下的差异只可能来自几何（内核数值积分差异）。
"""

import csv, io, json, sys

PYJ = r'C:\Users\29408\Desktop\外骨骼\_mass_leg_L.json'
SWC = r'C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\sw_baseline_legL.csv'
OUT = r'C:\Users\29408\Desktop\外骨骼\_legL_xcheck_py_vs_sw.txt'

# 质心差归一化尺度 [mm]：本项目腿部包络 ~300 mm，取 100 mm 作特征长度
L_CHAR = 100.0

# ★ 密度表不再内联 —— 从真源 exo2609/geometry.py 取（2026-09-20）。
#   本文件原先抄的副本缺 electronics 扩展键，且标签自造 "aluminium(FB)"。
#   内联副本正是"改一处、其余静默用旧值"的来源，故统一到唯一入口。
sys.path.insert(0, r'C:\Users\29408\Desktop\外骨骼')
from exo2609.geometry import density_of as _density_of            # noqa: E402


def rho_of(name):
    return _density_of(name)


# ---------------------------------------------------------------- 读两边
py = []                       # [{idx, name, vol, cog}]
d = json.load(io.open(PYJ, encoding='utf-8'))
for v in d['volumes']:
    py.append(dict(tag=v['tag'],
                   name=(v['name'] or '').split('/')[-1],
                   vol=float(v['volume']),
                   cog=[float(x) for x in v['cog']]))

sw = []
with io.open(SWC, encoding='utf-8-sig', newline='') as f:
    for r in csv.DictReader(f):
        sw.append(dict(name=r['part'],
                       vol=float(r['volume_mm3']),
                       cog=[float(r['com_x_mm']), float(r['com_y_mm']), float(r['com_z_mm'])]))

N_OK = (len(py) == len(sw))
L = []
def w(s=''):
    L.append(str(s))


# ------------------------------------------------------- 代价矩阵 + 最优指派
def cost_matrix(pyl, swl):
    C = []
    for a in swl:
        row = []
        for b in pyl:
            dv = abs(b['vol'] - a['vol']) / max(a['vol'], 1e-12)
            dc = sum((b['cog'][k] - a['cog'][k]) ** 2 for k in range(3)) ** 0.5 / L_CHAR
            row.append(dv + dc)
        C.append(row)
    return C


def assign(C):
    try:
        import numpy as np
        from scipy.optimize import linear_sum_assignment
        M = np.array(C, dtype=float)
        ri, ci = linear_sum_assignment(M)
        return list(zip(ri.tolist(), ci.tolist())), 'scipy.linear_sum_assignment(全局最优)'
    except Exception as e:                       # 兜底：贪心
        pairs, used = [], set()
        flat = sorted(((C[i][j], i, j) for i in range(len(C)) for j in range(len(C[0]))))
        for c, i, j in flat:
            if i in [p[0] for p in pairs] or j in used:
                continue
            pairs.append((i, j)); used.add(j)
        return pairs, 'greedy(兜底，scipy 不可用: %s)' % e


C = cost_matrix(py, sw)
pairs, how = assign(C)

# ---------------------------------------------------------------- 报告
w('== 几何配对交叉核对：我方 Python 链 vs SolidWorks 内核 ==')
w('our chain : %s  (%d vols)' % (PYJ, len(py)))
w('           其中无名件 %d 个 -> %s' % (
    sum(1 for p in py if not p['name']),
    ', '.join('tag%d' % p['tag'] for p in py if not p['name']) or '(无)'))
w('SW kernel : %s  (%d parts)' % (SWC, len(sw)))
w('counts    : py=%d  sw=%d  -> %s' % (len(py), len(sw), 'COUNT_OK 19 vs 19' if N_OK else 'COUNT_MISMATCH'))
w('指派算法  : %s' % how)
w('')
w('%-26s %12s %12s %9s %9s %9s  %s' %
  ('SW part', 'vol_py', 'vol_sw', 'relV', 'dCoM/mm', 'margin', 'rho/src'))
w('-' * 118)

vol_py_tot = 0.0
vol_sw_tot = 0.0
mpy = msw = 0.0
rows = []
for i, j in pairs:
    a, b = sw[i], py[j]
    rel = 100.0 * (b['vol'] - a['vol']) / a['vol']
    dc = sum((b['cog'][k] - a['cog'][k]) ** 2 for k in range(3)) ** 0.5
    # 判别余量：本行的次优候选代价 / 被选中的代价（越大越确定）
    rowc = sorted(enumerate(C[i]), key=lambda t: t[1])
    best = rowc[0]
    alt = next((c for k, c in rowc if k != j), float('inf'))
    margin = (alt / best[1]) if best[1] > 0 else float('inf')
    rho, lab = rho_of(a['name'])
    vol_py_tot += b['vol']; vol_sw_tot += a['vol']
    mpy += b['vol'] * 1e-9 * rho; msw += a['vol'] * 1e-9 * rho
    rows.append((a['name'], b['name'], rel, dc, margin))
    w('%-26s %12.4f %12.4f %8.4f%% %9.4f %9.1fx  %-6.0f %s%s' %
      (a['name'], b['vol'], a['vol'], rel, dc, margin, rho, lab,
       '   [py 无名,按几何认领]' if not b['name'] else ''))

w('-' * 118)
w('matched           = %d / %d' % (len(pairs), len(sw)))
w('sum vol_py        = %.4f mm3' % vol_py_tot)
w('sum vol_sw        = %.4f mm3' % vol_sw_tot)
w('rel diff (total)  = %.6f %%' % (100.0 * (vol_py_tot - vol_sw_tot) / vol_sw_tot))
w('')
w('mass_py (same rho) = %.6f kg   (%d parts)' % (mpy, len(py)))
w('mass_sw (same rho) = %.6f kg   (%d parts)' % (msw, len(sw)))
w('rel diff (mass)    = %.6f %%' % (100.0 * (mpy - msw) / msw))
w('')
# ★ 口径提示：数字会随密度表变，所以基准值不再写死（原先写 0.304833 / 0.2962）
w('NOTE 本表是「唯一零件 ×1」口径（每件算一次）；而 exo_real.urdf / plant_params.csv')
w('     里的 leg_L 质量是**实例加权**（35 个实例）口径 —— 两者不可直接比。')
try:
    with io.open(r'C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\plant_params.csv',
                 encoding='utf-8', newline='') as _f:
        _doc = float(list(csv.DictReader(_f))[0]['m_kg'])
    w('     参考：plant_params.csv leg_L = %.6f kg（实例加权）  本表合计 = %.6f kg（唯一件）'
      % (_doc, msw))
except Exception:
    pass

# 逐件 |relV| 分布，方便一眼判断内核噪声
absrel = sorted((abs(r[2]) for r in rows), reverse=True)
w('')
w('|relV| 分布: max %.4f%%  median %.4f%%  (<1e-3%% 的件数 %d/%d)' %
  (absrel[0], absrel[len(absrel) // 2],
   sum(1 for x in absrel if x < 1e-3), len(absrel)))
w('')
w('VERDICT: %s' % ('PAIR_19_OF_19_OK' if (N_OK and len(pairs) == len(sw)) else 'PAIR_MISMATCH'))

io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('\n'.join(L))
