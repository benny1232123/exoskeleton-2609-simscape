# -*- coding: utf-8 -*-
"""
Pass B: fix the entity regex (needs re.M) and get REAL geometry via gmsh/OCC:
entity counts, bbox, volume, surface area, plus thin-shell indicators.
Focus: the 3 anomalous parts + controls.
"""
import io, os, glob, re
from collections import Counter
import gmsh

DIR = r'D:\exo_xml'
OUT = r'C:\Users\29408\exo_work\_step_probe_B.txt'
ENT = re.compile(r'^#\d+\s*=\s*([A-Z_0-9]+)', re.M)

L = []
def w(s=''):
    L.append(str(s))

SUS = ['腿部设计_左-5', '腿部设计_左-6', '腿部_腿杆_片状V5', '腿部_轴盖',
       '电机_轴', '腿部_滑轨挡片']

files = sorted(glob.glob(os.path.join(DIR, '*.STEP')))

w('step_probe_B —— 可疑件的真实几何')
w('=' * 88)

gmsh.initialize()
gmsh.option.setNumber('General.Terminal', 0)
gmsh.option.setNumber('Geometry.OCCImportLabels', 0)

for key in SUS:
    hits = [f for f in files if key in os.path.basename(f)]
    if not hits:
        w('!! %s no file' % key); continue
    f = hits[0]
    base = os.path.basename(f)
    size = os.path.getsize(f)
    txt = io.open(f, encoding='utf-8', errors='replace').read()
    cnt = Counter(ENT.findall(txt))

    w('')
    w('-' * 88)
    w('%s   (%d bytes)' % (base, size))
    w('  [text] ADVANCED_FACE=%d  CLOSED_SHELL=%d  OPEN_SHELL=%d  MANIFOLD_SOLID_BREP=%d'
      % (cnt.get('ADVANCED_FACE', 0), cnt.get('CLOSED_SHELL', 0),
         cnt.get('OPEN_SHELL', 0), cnt.get('MANIFOLD_SOLID_BREP', 0)))
    w('  [text] PLANE=%d  CYLINDRICAL_SURFACE=%d  BSPLINE_SURFACE=%d  CARTESIAN_POINT=%d'
      % (cnt.get('PLANE', 0), cnt.get('CYLINDRICAL_SURFACE', 0),
         sum(v for k, v in cnt.items() if k.startswith('B_SPLINE_SURFACE') or k.startswith('RATIONAL_B_SPLINE_SURFACE')),
         cnt.get('CARTESIAN_POINT', 0)))

    try:
        gmsh.clear()
        gmsh.open(f)
        ents = gmsh.model.getEntities()
        np_ = sum(1 for d, t in ents if d == 0)
        nc_ = sum(1 for d, t in ents if d == 1)
        ns_ = sum(1 for d, t in ents if d == 2)
        nv_ = sum(1 for d, t in ents if d == 3)
        bb = gmsh.model.getBoundingBox(-1, -1)
        dims = [bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]]
        vol = 0.0
        for d, t in ents:
            if d == 3:
                try:
                    vol += gmsh.model.occ.getMass(3, t)
                except Exception:
                    pass
        area = 0.0
        for d, t in ents:
            if d == 2:
                try:
                    area += gmsh.model.occ.getMass(2, t)
                except Exception:
                    pass
        w('  [occ ] points=%d curves=%d surfaces=%d volumes=%d' % (np_, nc_, ns_, nv_))
        w('  [occ ] bbox_min = (%.4f, %.4f, %.4f) mm' % (bb[0], bb[1], bb[2]))
        w('  [occ ] bbox_max = (%.4f, %.4f, %.4f) mm' % (bb[3], bb[4], bb[5]))
        w('  [occ ] size     = (%.4f, %.4f, %.4f) mm' % tuple(dims))
        w('  [occ ] volume   = %.6f mm^3' % vol)
        w('  [occ ] area     = %.6f mm^2' % area)
        # thin-shell indicator: area/volume ratio; for a sphere-ish solid of
        # "diameter" d, area/vol ~ 6/d.  Huge ratio  =>  thin shell / surface-only.
        dchar = max(dims) if max(dims) > 0 else 1.0
        w('  [diag] d_char = %.3f mm ; 6/d_char = %.4f 1/mm ; area/vol = %s'
          % (dchar, 6.0 / dchar, ('%.4f' % (area / vol)) if vol else 'inf'))
        # bbox volume vs real volume -> fill ratio
        bbv = dims[0] * dims[1] * dims[2]
        w('  [diag] bbox_volume = %.4f mm^3 ; fill = %s'
          % (bbv, ('%.3e' % (vol / bbv)) if bbv > 0 else 'inf'))
    except Exception as e:
        w('  [occ ] FAILED %r' % e)

gmsh.finalize()
io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('WROTE', OUT)
