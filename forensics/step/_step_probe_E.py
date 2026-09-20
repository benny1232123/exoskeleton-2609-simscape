# -*- coding: utf-8 -*-
"""
Pass E: two decisive measurements on 腿部设计_左-5
  (1) print raw ADVANCED_FACE lines and resolve each surface ref -> type
  (2) orphan analysis: how many CARTESIAN_POINT are referenced by NOTHING?
"""
import io, os, glob, re
from collections import Counter

DIR = r'D:\exo_xml'
OUT = r'C:\Users\29408\exo_work\_step_probe_E.txt'
HEAD = re.compile(r'^\s*#(\d+)\s*=\s*([A-Z_0-9]+)')

L = []
def w(s=''):
    L.append(str(s))

key = '腿部设计_左-5'
f = [x for x in sorted(glob.glob(os.path.join(DIR, '*.STEP'))) if key in os.path.basename(x)][0]
txt = io.open(f, encoding='utf-8', errors='replace').read()

ent = {}
for chunk in txt.split(';'):
    m = HEAD.match(chunk)
    if m:
        ent[int(m.group(1))] = (m.group(2), chunk)

w('file = %s' % os.path.basename(f))
w('entities = %d' % len(ent))

w('')
w('--- raw ADVANCED_FACE (first 8) ---')
n = 0
for i, (t, c) in sorted(ent.items()):
    if t == 'ADVANCED_FACE':
        refs = re.findall(r'#(\d+)', c)
        tail = int(refs[-1]) if refs else -1
        st = ent.get(tail, ('<MISSING>', ''))[0]
        w('  #%-6d refs=%-28s last=#%-6d type=%s' % (i, refs, tail, st))
        w('        text: %s' % ' '.join(c.split())[:150])
        n += 1
        if n >= 8:
            break

w('')
w('--- surface type histogram (resolved) ---')
st = Counter()
for i, (t, c) in ent.items():
    if t == 'ADVANCED_FACE':
        refs = re.findall(r'#(\d+)', c)
        tail = int(refs[-1]) if refs else -1
        st[ent.get(tail, ('<MISSING>', ''))[0]] += 1
for k, v in st.most_common():
    w('    %-34s %5d' % (k, v))

# ---- orphan analysis ----
w('')
w('--- orphan analysis ---')
all_refs = Counter()
for i, (t, c) in ent.items():
    # refs inside the body only (strip the defining "#i =")
    body = c.split('=', 1)[1] if '=' in c else c
    for r in re.findall(r'#(\d+)', body):
        all_refs[int(r)] += 1

by_type = {}
for i, (t, c) in ent.items():
    by_type.setdefault(t, []).append(i)

w('%-30s %7s %7s %7s' % ('entity type', 'total', 'orphan', 'refd'))
w('-' * 56)
for t in sorted(by_type, key=lambda k: -len(by_type[k]))[:12]:
    ids = by_type[t]
    orph = [i for i in ids if all_refs.get(i, 0) == 0]
    w('%-30s %7d %7d %7d' % (t, len(ids), len(orph), len(ids) - len(orph)))

ptids = by_type.get('CARTESIAN_POINT', [])
orph_pts = [i for i in ptids if all_refs.get(i, 0) == 0]
w('')
w('CARTESIAN_POINT total  = %d' % len(ptids))
w('CARTESIAN_POINT orphan = %d  (%.1f%%)' % (len(orph_pts), 100.0 * len(orph_pts) / max(1, len(ptids))))
if orph_pts:
    w('sample orphan point text:')
    w('   ' + ' '.join(ent[orph_pts[0]][1].split())[:180])
# chars carried by orphan points
och = sum(len(ent[i][1]) for i in orph_pts)
w('chars carried by orphan points = %d  (%.1f%% of file)' % (och, 100.0 * och / len(txt)))

io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('WROTE', OUT)
