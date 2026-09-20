# -*- coding: utf-8 -*-
"""
Pass A (fast, text-only): inventory the exported STEP files and dump the
structural statistics of the suspect ones, straight from the STEP text.
No heavy OCC geometry ops here.
"""
import io, os, glob, re
from collections import Counter

DIR = r'D:\exo_xml'
OUT = r'C:\Users\29408\exo_work\_step_probe_A.txt'

L = []
def w(s=''):
    L.append(str(s))

files = sorted(glob.glob(os.path.join(DIR, '*.STEP')) + glob.glob(os.path.join(DIR, '*.stp')))
w('dir = %s' % DIR)
w('STEP files = %d' % len(files))
w('')
w('%-52s %12s' % ('file', 'bytes'))
w('-' * 66)
for f in files:
    w('%-52s %12d' % (os.path.basename(f), os.path.getsize(f)))

# suspect parts (per earlier audit)
SUS = ['腿部设计_左-5', '腿部设计_左-6', '腿部_腿杆_片状V5', '腿部_轴盖']

w('')
w('=' * 78)
w('STEP structural stats (text level)')
w('=' * 78)

# entity type histogram: lines like "#123=ADVANCED_FACE(...)"
ENT = re.compile(r'^#\d+\s*=\s*([A-Z_0-9]+)')

for key in SUS:
    hits = [f for f in files if key in os.path.basename(f)]
    if not hits:
        w('')
        w('!! %s : no file' % key)
        continue
    f = hits[0]
    size = os.path.getsize(f)
    txt = io.open(f, encoding='utf-8', errors='replace').read()
    ents = ENT.findall(txt)
    cnt = Counter(ents)
    npt = len(re.findall(r'CARTESIAN_POINT', txt))
    w('')
    w('-' * 78)
    w('%s' % os.path.basename(f))
    w('  size = %d bytes   lines = %d   total entities = %d'
      % (size, txt.count('\n') + 1, len(ents)))
    # schema / header
    m = re.search(r'FILE_SCHEMA\s*\(\s*\(\s*[\'"]([^\'"]+)', txt)
    w('  schema = %s' % (m.group(1) if m else '?'))
    m = re.search(r'FILE_NAME\s*\(([^)]*)', txt)
    if m:
        w('  file_name header = %s' % m.group(1)[:160].replace('\n', ' '))
    man = re.search(r'FILE_DESCRIPTION\s*\(([^)]*)', txt)
    if man:
        w('  description = %s' % man.group(1)[:160].replace('\n', ' '))
    w('  --- top entity types ---')
    for k, v in cnt.most_common(14):
        w('    %-34s %7d' % (k, v))
    w('  solids(MANIFOLD_SOLID_BREP)       = %d' % cnt.get('MANIFOLD_SOLID_BREP', 0))
    w('  shells(CLOSED_SHELL)              = %d' % cnt.get('CLOSED_SHELL', 0))
    w('  OPEN_SHELL                        = %d' % cnt.get('OPEN_SHELL', 0))
    w('  ADVANCED_FACE                     = %d' % cnt.get('ADVANCED_FACE', 0))
    w('  CARTESIAN_POINT                   = %d' % npt)
    w('  B_SPLINE_SURFACE*                 = %d'
      % sum(v for k, v in cnt.items() if k.startswith('B_SPLINE_SURFACE')))
    w('  PLANE / CYLINDRICAL_SURFACE       = %d / %d'
      % (cnt.get('PLANE', 0), cnt.get('CYLINDRICAL_SURFACE', 0)))
    w('  TRIANGULATED_FACE_SET             = %d' % cnt.get('TRIANGULATED_FACE_SET', 0))

io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('WROTE', OUT)
