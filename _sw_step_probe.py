# -*- coding: utf-8 -*-
"""三方对比：我方 gmsh/OCC 解析器 vs SolidWorks XML vs 我方现有 _reduced3 数据。

目的 —— 判定 `_xml_vs_py.py` 里那 5 件 0.002%~0.18% 的体积偏差到底出在哪：
  A = gmsh 读**插件导出的那份 STEP**（D:\\exo_xml\\<件名>..._sldprt.STEP）
  B = SW XML 反解（体积 = mass[kg] × 1e6，前提 ρ_SW = 1000 kg/m³）
  C = gmsh 读 **我方 `_reduced3/leg_L.stp`**（= 现有 `_mass_leg_L.json` 的来源）

判读：
  A ≈ B  → **我方解析器没问题**；C 偏说明 `_reduced3` 与 SW 零件几何不同（源头差异）
  A ≈ C  → 我方解析器在该件上有系统性偏差（与几何源头无关）
  B ≈ C  → 那 `_xml_vs_py` 里的偏差另有原因

用法：python _sw_step_probe.py <零件名前缀> [out.txt]
"""
import glob
import io
import json
import os
import sys
import xml.etree.ElementTree as ET

DIR     = r'D:\exo_xml'
PREFIX  = sys.argv[1] if len(sys.argv) > 1 else u'腿部_滑块_双键'
OUT     = sys.argv[2] if len(sys.argv) > 2 else r'C:\Users\29408\exo_work\_sw_step_probe.txt'
XMLF    = os.path.join(DIR, u'腿部设计_左.xml')
PYJSON  = r'C:\Users\29408\Desktop\外骨骼\_mass_leg_L.json'
RHO_SW  = 1000.0

L = []
def p(s=''):
    L.append(str(s))


def local(t):
    return t.rsplit('}', 1)[-1] if '}' in t else t


# ------------------------------------------------------------- 找文件
cands = [f for f in glob.glob(os.path.join(DIR, '*.STEP'))
         if os.path.basename(f).startswith(PREFIX)]
p(u'# 三方对比探针')
p(u'零件名前缀 = %s' % PREFIX)
p(u'匹配到的 STEP 文件 = %d 个' % len(cands))
for c in cands:
    p(u'    %r  (%.1f KB)' % (os.path.basename(c), os.path.getsize(c) / 1024.0))
if not cands:
    open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
    print('no file matched'); sys.exit(1)
TARGET = cands[0]

# ------------------------------------------------------------- A: gmsh 读插件 STEP
p('')
p(u'## A. gmsh/OCC 读「插件导出的 STEP」')
import gmsh                                            # noqa: E402
gmsh.initialize()
gmsh.option.setNumber('General.Terminal', 0)
for k in ('Geometry.OCCSewFaces', 'Geometry.OCCFixSmallEdges',
          'Geometry.OCCFixSmallFaces', 'Geometry.OCCFixDegenerated'):
    try:
        gmsh.option.setNumber(k, 0)
    except Exception:
        pass
gmsh.open(TARGET)
gmsh.model.occ.synchronize()
vols = gmsh.model.getEntities(3)
p(u'gmsh 版本 = %s' % gmsh.__version__ if hasattr(gmsh, '__version__') else u'?')
p(u'读到的 3D 实体数 = %d' % len(vols))

A_v = 0.0
A_m = [0.0, 0.0, 0.0]
A_tr = 0.0
for (dim, tag) in vols:
    v = gmsh.model.occ.getMass(3, tag)
    v = float(v) if isinstance(v, (int, float)) else float(v[0])
    c = [float(x) for x in gmsh.model.occ.getCenterOfMass(3, tag)]
    I = [float(x) for x in gmsh.model.occ.getMatrixOfInertia(3, tag)]
    A_v += v
    # 质量加权求总质心
    for i in range(3):
        A_m[i] += v * c[i]
    A_tr += (I[0] + I[4] + I[8])      # 列主序对角 = 迹
if A_v:
    A_m = [x / A_v for x in A_m]
p(u'A: volume = %.6f mm³' % A_v)
p(u'A: cog    = [%.6f, %.6f, %.6f] mm' % tuple(A_m))
p(u'A: trace(I) = %.6f mm⁵' % A_tr)
gmsh.finalize()

# ------------------------------------------------------------- B: SW XML
p('')
p(u'## B. SolidWorks XML 反解（ρ 假定 %.0f kg/m³）' % RHO_SW)
raw = open(XMLF, 'rb').read()
san = bytearray(raw)
for i in range(len(san)):
    if san[i] < 0x20 and san[i] not in (0x09, 0x0A, 0x0D):
        san[i] = 0x3F
root = ET.fromstring(bytes(san).decode('utf-8', 'replace'))
B_v = B_tr = None
B_c = None
for part in root.iter():
    if local(part.tag) != 'Part':
        continue
    if not part.attrib.get('name', '').startswith(PREFIX):
        continue
    for g in part:
        if local(g.tag) != 'MassProperties':
            continue
        for h in g:
            lh = local(h.tag)
            if lh == 'Mass':
                B_v = float(h.text.strip()) * 1e6            # kg → mm³
            elif lh == 'CenterOfMass':
                B_c = [float(x) * 1000.0 for x in h.text.split()]
            elif lh == 'Inertia':
                vals = [float(x) for x in h.text.split()]
                B_tr = sum(vals[:3]) * 1e12                  # kg·m² → mm⁵
p(u'B: volume = %s mm³' % (('%.6f' % B_v) if B_v else '未找到'))
p(u'B: cog    = %s mm' % ([round(x, 6) for x in B_c] if B_c else '未找到'))
p(u'B: trace(I) = %s mm⁵' % (('%.6f' % B_tr) if B_tr else '未找到'))

# ------------------------------------------------------------- C: 我方 _reduced3
p('')
p(u'## C. 我方 `_reduced3/leg_L.stp` 的既有结果')
with io.open(PYJSON, 'r', encoding='utf-8') as f:
    data = json.load(f)
C_v = C_tr = None
C_c = None
for d in data['volumes']:
    if d.get('name', '').split('/')[-1].startswith(PREFIX):
        C_v = d['volume']
        C_c = d['cog']
        ic = d['inertia_cog']
        C_tr = ic[0] + ic[4] + ic[8]
p(u'C: volume = %s mm³' % (('%.6f' % C_v) if C_v else u'未找到'))
p(u'C: cog    = %s mm' % ([round(x, 6) for x in C_c] if C_c else u'未找到'))
p(u'C: trace(I) = %s mm⁵' % (('%.6f' % C_tr) if C_tr else u'未找到'))

# ------------------------------------------------------------- 对比
p('')
p('=' * 74)
p(u'## 对比（相对差，×100 即 %）')
p('=' * 74)


def rel(a, b):
    if a is None or b is None or b == 0:
        return None
    return abs(a - b) / abs(b)


rows = [(u'volume mm³', A_v, B_v, C_v), (u'trace(I) mm⁵', A_tr, B_tr, C_tr)]
p(u'%-14s %14s %14s %14s %10s %10s' % (u'量', u'A 插件STEP', u'B SW XML', u'C _reduced3', u'A~B', u'A~C'))
for nm, a, b, c in rows:
    p(u'%-14s %14.4f %14s %14s %9s %9s'
      % (nm, a,
         ('%.4f' % b) if b else '-',
         ('%.4f' % c) if c else '-',
         ('%.2e%%' % (100 * rel(a, b))) if rel(a, b) is not None else '-',
         ('%.2e%%' % (100 * rel(a, c))) if rel(a, c) is not None else '-'))

p('')
if B_v and C_v:
    p(u'A~B vs A~C 谁更小（体积）：%.3e%% vs %.3e%%' % (100 * rel(A_v, B_v), 100 * rel(A_v, C_v)))
    if rel(A_v, B_v) < rel(A_v, C_v):
        p(u'')
        p(u'★ 结论：A 与 B 更接近 → **我方解析器没问题**。')
        p(u'  之前 `_xml_vs_py` 里看到的那点偏差，来自 `_reduced3/leg_L.stp`')
        p(u'  与 SolidWorks 零件本身的几何差异（源头差异），不是解析器误差。')
    else:
        p(u'')
        p(u'★ 结论：A 与 C 更接近 → 我方解析器在该件上有**系统性偏差**。')
        p(u'  需要查 gmsh/OCC 的积分容差设置（Geometry.OCC* 那些开关）。')

# 附带：CoM 三方
if B_c and C_c:
    p('')
    p(u'CoM A = [%.6f %.6f %.6f]' % tuple(A_m))
    p(u'CoM B = [%.6f %.6f %.6f]' % tuple(B_c))
    p(u'CoM C = [%.6f %.6f %.6f]' % tuple(C_c))

open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('written -> %s' % OUT)
