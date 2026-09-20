# -*- coding: utf-8 -*-
"""
Pass D: resolve the STEP entity graph for the anomalous part.
  - build id -> (type, text) by splitting on ';'
  - for every ADVANCED_FACE, resolve its surface -> authoritative surface histogram
  - for every B_SPLINE_CURVE_WITH_KNOTS, count control points
Answers: what IS this part, and why is it 752 KB?
"""
import io, os, glob, re
from collections import Counter

DIR = r'D:\exo_xml'
OUT = r'C:\Users\29408\exo_work\_step_probe_D.txt'
HEAD = re.compile(r'^\s*#(\d+)\s*=\s*([A-Z_0-9]+)')

L = []
def w(s=''):
    L.append(str(s))

TARGETS = ['腿部设计_左-5', '腿部_腿杆_片状V5', '电机_轴']
files = sorted(glob.glob(os.path.join(DIR, '*.STEP')))

def refs(s):
    return re.findall(r'#(\d+)', s)

for key in TARGETS:
    hits = [f for f in files if key in os.path.basename(f)]
    if not hits:
        w('!! %s' % key); continue
    f = hits[0]
    txt = io.open(f, encoding='utf-8', errors='replace').read()
    ent = {}
    for chunk in txt.split(';'):
        m = HEAD.match(chunk)
        if m:
            ent[int(m.group(1))] = (m.group(2), chunk)
    w('=' * 88)
    w('%s' % os.path.basename(f))
    w('  parsed entities = %d' % len(ent))

    # ---- faces -> surfaces (authoritative) ----
    surf_types = Counter()
    face_edges = []
    for i, (t, c) in ent.items():
        if t == 'ADVANCED_FACE':
            r = refs(c)
            # last ref is the surface
            if r:
                sid = int(r[-1])
                st = ent.get(sid, ('?', ''))[0]
                surf_types[st] += 1
    w('  --- surface types actually referenced by ADVANCED_FACE ---')
    for k, v in surf_types.most_common():
        w('    %-34s %5d' % (k, v))

    # ---- spline curve control-point counts ----
    cps = []
    for i, (t, c) in ent.items():
        if t in ('B_SPLINE_CURVE_WITH_KNOTS', 'B_SPLINE_CURVE'):
            groups = re.findall(r'\(([^()]*)\)', c)
            best = max((g for g in groups if '#' in g), key=lambda g: g.count('#'), default='')
            cps.append(best.count('#'))
    cps.sort(reverse=True)
    w('  --- B-spline curves: count=%d  control-points (desc) ---' % len(cps))
    w('    %s' % cps[:20])
    w('    total control points = %d' % sum(cps))

    # ---- which entities are the biggest ----
    big = sorted(ent.items(), key=lambda kv: -len(kv[1][1]))[:6]
    w('  --- largest entity texts ---')
    for i, (t, c) in big:
        w('    #%d %-30s %6d chars  head: %s' % (i, t, len(c), c[:110].replace('\n', ' ')))
    w('')

io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('WROTE', OUT)
