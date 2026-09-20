# -*- coding: utf-8 -*-
"""
Pass C: full entity histogram (re.M) for the anomalous parts, to find where the
752 KB actually goes.  Also dump the file header and a sample of the first face.
"""
import io, os, glob, re
from collections import Counter

DIR = r'D:\exo_xml'
OUT = r'C:\Users\29408\exo_work\_step_probe_C.txt'
ENT = re.compile(r'^#\d+\s*=\s*([A-Z_0-9]+)', re.M)

L = []
def w(s=''):
    L.append(str(s))

TARGETS = ['腿部设计_左-5', '腿部_腿杆_片状V5', '电机_轴']
files = sorted(glob.glob(os.path.join(DIR, '*.STEP')))

for key in TARGETS:
    hits = [f for f in files if key in os.path.basename(f)]
    if not hits:
        w('!! %s' % key); continue
    f = hits[0]
    txt = io.open(f, encoding='utf-8', errors='replace').read()
    lines = txt.split('\n')
    cnt = Counter(ENT.findall(txt))
    w('=' * 88)
    w('%s   size=%d  lines=%d  entities=%d'
      % (os.path.basename(f), len(txt.encode('utf-8', 'replace')), len(lines), sum(cnt.values())))
    w('  comment lines starting with "!": %d' % sum(1 for ln in lines if ln.strip().startswith('!')))
    w('  longest line length          : %d' % max(len(ln) for ln in lines))
    w('  avg chars per entity line    : %.1f'
      % (sum(len(ln) for ln in lines) / max(1, sum(cnt.values()))))
    w('')
    w('  --- FULL entity histogram ---')
    for k, v in cnt.most_common():
        w('    %-40s %7d' % (k, v))
    w('')
    w('  --- header (first 12 non-empty lines) ---')
    ne = [ln for ln in lines[:40] if ln.strip()]
    for ln in ne[:12]:
        w('    ' + ln[:150])
    w('')

io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('WROTE', OUT)
