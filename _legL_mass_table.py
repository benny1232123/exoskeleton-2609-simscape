# -*- coding: utf-8 -*-
"""
Per-part mass table: SW-kernel volumes (independent geometry) x our density table.
Purpose: rank parts by mass contribution -> tells us WHICH parts to weigh to pin
the total mass.  Also flags density-table coverage gaps (fallback hits).

★ 密度表**不再内联**：改为 import `exo2609/geometry.py` 的真源（2026-09-20）。
  以前这里抄了一份 `DENS` 副本，且抄的时候把标签写成 "aluminium (FALLBACK)"，
  与真源的 "aluminium (default)" 不一致 —— 这正是副本漂移的典型症状。
  现在统一从真源取，缺口判定一律用 `is_fallback()`。
"""
import csv, io, os, sys

SRC = r'C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\sw_baseline_legL.csv'
OUT = r'C:\Users\29408\exo_work\_legL_mass_table.txt'
CSV = r'C:\Users\29408\exo_work\_legL_mass_table.csv'

# --- 密度表真源（唯一）：exo2609/geometry.py ---
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from exo2609.geometry import (            # noqa: E402
    density_of, is_fallback, FALLBACK_DENSITY, FALLBACK_LABEL, PART_EXACT,
)


def rho_of(name):
    """兼容旧签名：返回 (rho, label)。"""
    return density_of(name)


rows = []
with io.open(SRC, 'r', encoding='utf-8-sig', newline='') as f:
    for r in csv.DictReader(f):
        vol = float(r['volume_mm3'])
        rho, lab = rho_of(r['part'])
        rows.append({'part': r['part'], 'vol': vol, 'rho': rho, 'lab': lab,
                     'mass': vol * 1e-9 * rho})

tot = sum(x['mass'] for x in rows)
rows.sort(key=lambda x: -x['mass'])
cum = 0.0
for x in rows:
    cum += x['mass']
    x['share'] = 100.0 * x['mass'] / tot
    x['cum'] = 100.0 * cum / tot

L = []
def w(s=''):
    L.append(str(s))

w('=' * 96)
w('左腿 19 件 · 质量贡献表')
w('  体积来源  : SolidWorks 内核（sw_baseline_legL.csv，独立于我们的解析链）')
w('  密度来源  : exo2609/geometry.py —— PART_EXACT（精确名）-> DEFAULT_DENSITIES（关键字）-> 兜底')
w('  质量 = 体积 x 密度；密度是唯一未知量 -> 该表用于定位"该称哪几件"')
w('  精确名覆盖: %s' % (', '.join(sorted(PART_EXACT)) or '(空)'))
w('=' * 96)
w('')
w('%-34s %12s %9s %8s %12s %8s %8s  %s'
  % ('part', 'vol_mm3', 'rho', 'src', 'mass_kg', 'share%', 'cum%', 'density basis'))
w('-' * 96)
for x in rows:
    tag = 'GAP' if is_fallback(x['lab']) else ''
    w('%-34s %12.4f %9.0f %8s %12.6f %7.2f%% %7.2f%%  %-22s %s'
      % (x['part'], x['vol'], x['rho'], '', x['mass'], x['share'], x['cum'], x['lab'], tag))
w('-' * 96)
w('%-34s %12s %9s %8s %12.6f %7.2f%%' % ('TOTAL(19)', '', '', '', tot, 100.0))

w('')
w('=== 按质量排序：累计占比达到 90% 所需的前 N 件 ===')
c = 0.0; n = 0
for x in rows:
    if c >= 90.0:
        break
    c += x['mass']; n += 1
    w('  %d. %-32s %7.2f%%  (cum %.2f%%)' % (n, x['part'], x['share'], 100.0 * c / tot))
w('  -> 称重这 %d 件即可锁定整腿质量的 %.1f%%' % (n, 100.0 * c / tot))

gaps = [x for x in rows if is_fallback(x['lab'])]
gap_mass = sum(x['mass'] for x in gaps)

w('')
w('=== ★ 密度表覆盖缺口（吃到兜底密度 %g 的件）===' % FALLBACK_DENSITY)
w('  %d / %d 件落到兜底，合计 %0.6f kg = 整腿质量的 %.2f%%'
  % (len(gaps), len(rows), gap_mass, 100.0 * gap_mass / tot))
for x in gaps:
    w('  %-34s vol=%9.3f mm3  m=%10.6f kg  (share %.2f%%)  <- 需要人工确认材料'
      % (x['part'], x['vol'], x['mass'], x['share']))

# sensitivity: if the ambiguous structural parts were steel 7850 instead of the fallback
w('')
w('=== 敏感性：把缺口件改成钢 7850 的影响 ===')
for x in gaps:
    m2 = x['vol'] * 1e-9 * 7850.0
    w('  %-34s  %10.6f -> %10.6f kg   (+%.6f, 整腿 +%.2f%%)'
      % (x['part'], x['mass'], m2, m2 - x['mass'], 100.0 * (m2 - x['mass']) / tot))

with io.open(OUT, 'w', encoding='utf-8') as f:
    f.write('\n'.join(L))
with io.open(CSV, 'w', encoding='utf-8-sig', newline='') as f:
    wr = csv.writer(f)
    wr.writerow(['part', 'volume_mm3', 'rho_kg_m3', 'density_basis',
                 'mass_kg', 'share_pct', 'cum_pct'])
    for x in rows:
        wr.writerow([x['part'], '%.6f' % x['vol'], '%.0f' % x['rho'], x['lab'],
                     '%.9f' % x['mass'], '%.3f' % x['share'], '%.3f' % x['cum']])
print('WROTE', OUT)
print('WROTE', CSV)
