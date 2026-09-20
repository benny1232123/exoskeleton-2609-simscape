# -*- coding: utf-8 -*-
"""
Reconcile the 19-unique-part table against the leg_L mass in plant_params.csv
(current baseline; was 0.304833 kg before the 2026-09-20 density fix) by
instance-weighting: parse <Instance> names from the Simscape XML (SW export),
count multiplicity per part, multiply volume x rho x count.
"""
import io, os, re, csv, sys
import xml.etree.ElementTree as ET

XML = r'D:\exo_xml\腿部设计_左.fixed.xml'
SWC = r'C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\sw_baseline_legL.csv'
OUT = r'C:\Users\29408\exo_work\_legL_instance_recon.txt'

# ★ 密度表不再内联 —— 从真源 exo2609/geometry.py 取（2026-09-20）。
#   本文件原先抄的副本已明显漂移：缺 electronics 扩展键（OPEN_CASCADE / SOT23 /
#   LQFP ...），标签也自造成 "aluminium(FB)"。只保留一个入口，避免"改了一处、
#   另一处静默用旧值"。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from exo2609.geometry import density_of as _density_of, FALLBACK_LABEL

def rho_of(n):
    return _density_of(n)

vol = {}
with io.open(SWC, encoding='utf-8-sig', newline='') as f:
    for r in csv.DictReader(f):
        vol[r['part']] = float(r['volume_mm3'])

raw = io.open(XML, encoding='utf-8').read()
inst = re.findall(r'<Instance\s+name="([^"]*)"', raw)
L = []
def w(s=''):
    L.append(str(s))
w('XML           = %s' % XML)
w('<Instance> 数 = %d' % len(inst))

# strip trailing -<digits> to get base part name
def base(n):
    return re.sub(r'-\d+$', '', n)

from collections import Counter
cnt = Counter(base(x) for x in inst)

w('')
w('%-34s %8s %12s %8s %-20s' % ('base part', 'count', 'vol_mm3', 'rho', 'basis'))
w('-' * 90)
tot = 0.0
unknown = []
for nm, c in sorted(cnt.items(), key=lambda kv: -kv[1]):
    if nm in vol:
        rho, lab = rho_of(nm)
        m = vol[nm] * 1e-9 * rho * c
        tot += m
        w('%-34s %8d %12.4f %8.0f %-20s' % (nm, c, vol[nm], rho, lab))
    else:
        unknown.append((nm, c))
        w('%-34s %8d %12s %8s %s' % (nm, c, '-', '-', 'NO VOLUME MATCH'))
w('-' * 90)
w('instance-weighted total = %.6f kg' % tot)
# ★ 文档值不再硬编码 —— 从 plant_params.csv（= exo_real.urdf 的 leg_L 质量）读。
#   以前写死 0.304833：密度表一改，这里就会报出「偏差 +2.6%」这种假警报，
#   而真因是我们的密度表被有意更新了，不是分组/多重度错了。
DOC = 0.312589
try:
    with io.open(r'C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\plant_params.csv',
                 encoding='utf-8', newline='') as _f:
        DOC = float(list(csv.DictReader(_f))[0]['m_kg'])
except Exception:
    pass
w('documented leg_L mass    = %.6f kg   (源: plant_params.csv)' % DOC)
w('rel diff                 = %.4f %%' % (100.0 * (tot - DOC) / DOC))
if unknown:
    w('')
    w('unmatched instance base names: %s' % unknown)
w('')
w('vs 19-unique-part total (= every part once) = %.6f kg' % sum(
    v * 1e-9 * rho_of(k)[0] for k, v in vol.items()))

io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('WROTE', OUT)
