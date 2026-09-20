# -*- coding: utf-8 -*-
"""
Pass F: corrected parser (handles STEP complex instances '#id =( TYPE_A() TYPE_B() )')
Re-measure: surface types, orphan points, and where the bytes really go.
"""
import io, os, glob, re
from collections import Counter

DIR = r'D:\exo_xml'
OUT = r'C:\Users\29408\exo_work\_step_probe_F.txt'
HEAD = re.compile(r'^\s*#(\d+)\s*=')

L = []
def w(s=''):
    L.append(str(s))

def classify(body):
    toks = re.findall(r'\b([A-Z][A-Z_0-9]{2,})\b', body)
    for t in toks:
        if t not in ('NONE',):
            return t
    return '?'

TARGETS = ['腿部设计_左-5', '腿部_腿杆_片状V5', '电机_轴']
files = sorted(glob.glob(os.path.join(DIR, '*.STEP')))

for key in TARGETS:
    f = [x for x in files if key in os.path.basename(x)][0]
    txt = io.open(f, encoding='utf-8', errors='replace').read()
    ent = {}
    for chunk in txt.split(';'):
        m = HEAD.match(chunk)
        if m:
            ent[int(m.group(1))] = chunk

    w('=' * 88)
    w('%s   entities=%d  file=%d chars' % (os.path.basename(f), len(ent), len(txt)))

    # type per entity
    etype = {}
    for i, c in ent.items():
        body = c.split('=', 1)[1] if '=' in c else c
        etype[i] = classify(body)
    w('  --- entity type histogram (fixed parser) ---')
    for k, v in Counter(etype.values()).most_common(12):
        w('    %-34s %6d' % (k, v))

    # surface types of faces
    st = Counter()
    for i, c in ent.items():
        if etype[i] == 'ADVANCED_FACE':
            r = [int(x) for x in re.findall(r'#(\d+)', c.split('=', 1)[1])]
            st[etype.get(r[-1], '<MISSING>')] += 1
    w('  --- surfaces of the %d faces ---' % sum(st.values()))
    for k, v in st.most_common():
        w('    %-34s %6d' % (k, v))

    # orphan analysis (corrected)
    refd = Counter()
    for i, c in ent.items():
        body = c.split('=', 1)[1] if '=' in c else c
        for x in re.findall(r'#(\d+)', body):
            refd[int(x)] += 1
    tot = Counter(etype.values())
    orph = Counter()
    for i, t in etype.items():
        if refd.get(i, 0) == 0:
            orph[t] += 1
    w('  --- orphan counts (referenced by nothing) ---')
    for t in ['CARTESIAN_POINT', 'DIRECTION', 'AXIS2_PLACEMENT_3D', 'VECTOR', 'LINE', 'CIRCLE']:
        if tot.get(t):
            w('    %-30s total=%-6d orphan=%-6d' % (t, tot[t], orph[t]))
    ptids = [i for i, t in etype.items() if t == 'CARTESIAN_POINT']
    op = [i for i in ptids if refd.get(i, 0) == 0]
    w('    => CARTESIAN_POINT orphan %d / %d (%.1f%%)' % (len(op), len(ptids), 100.0 * len(op) / max(1, len(ptids))))
    w('')
    # where do bytes go: by entity type
    bychars = Counter()
    for i, t in etype.items():
        bychars[t] += len(ent[i])
    w('  --- bytes by entity type (top 8) ---')
    tot_ch = sum(bychars.values())
    for k, v in bychars.most_common(8):
        w('    %-34s %9d  %5.1f%%' % (k, v, 100.0 * v / tot_ch))
    w('')

io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('WROTE', OUT)
