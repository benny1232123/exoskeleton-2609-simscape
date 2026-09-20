# -*- coding: utf-8 -*-
"""独立复核（全量 19 件）：用 gmsh/OpenCASCADE 读插件 & SW 导出的 STEP，
   把 volume / CoM / 惯量 与 SolidWorks 内核（XML 声明）比对。

关键实现细节（踩过）：
  * `getMass` 在该 gmsh 版本只返回质量；CoM / 惯量是独立函数。
  * 一个 STEP 里可能有**多个实体**，只取 vols[0] 会漏体积
    （腿部_腿杆_片状V5 曾因只取第一个实体，体积少 0.92%）。
  * OCC 的惯量矩阵是相对**原点**；SW 的是相对**质心**。先各自搬到全局原点
    再求和，最后用平行轴定理搬到合成质心。
  * 单位统一到 mm / mm^5（OCC 密度=1，SW 侧 inertia_mm5 = kg*m^2 * 1e12）。
"""
import json, os, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

BASE = r'C:\Users\29408\exo_work\_sw_baseline_legL.json'
DIR  = r'D:\exo_xml'
OUT  = r'C:\Users\29408\exo_work\_step_check_all.txt'

import gmsh
gmsh.initialize()
gmsh.option.setNumber('General.Terminal', 0)


def props(path):
    """返回 (体积 mm^3, 质心 mm, 相对原点的惯量矩阵 mm^5)，多实体自动合成。"""
    gmsh.model.add(os.path.basename(path))
    gmsh.model.occ.importShapes(path)
    gmsh.model.occ.synchronize()
    vols = gmsh.model.getEntities(3)
    V = 0.0
    Mx = My = Mz = 0.0
    Iorg = [[0.0] * 3 for _ in range(3)]
    for _, tag in vols:
        vi = gmsh.model.occ.getMass(3, tag)
        ci = list(gmsh.model.occ.getCenterOfMass(3, tag))
        mi = list(gmsh.model.occ.getMatrixOfInertia(3, tag))
        if len(mi) >= 9:
            Ii = [[mi[3 * a + b] for b in range(3)] for a in range(3)]
        else:
            Ii = [[mi[0], 0, 0], [0, mi[1], 0], [0, 0, mi[2]]]
        V += vi
        Mx += vi * ci[0]; My += vi * ci[1]; Mz += vi * ci[2]
        r2 = sum(x * x for x in ci)
        for a in range(3):
            for b in range(3):
                # 平行轴：把该实体相对自身原点的惯量搬到全局原点
                Iorg[a][b] += Ii[a][b] + vi * ((r2 if a == b else 0.0) - ci[a] * ci[b])
    gmsh.model.remove()
    if V <= 0:
        return V, [0, 0, 0], Iorg, len(vols)
    return V, [Mx / V, My / V, Mz / V], Iorg, len(vols)


def to_com(Iorg, V, cog):
    """把相对原点的惯量搬到质心。"""
    r2 = sum(x * x for x in cog)
    return [[Iorg[a][b] - V * ((r2 if a == b else 0.0) - cog[a] * cog[b])
             for b in range(3)] for a in range(3)]


base = json.load(open(BASE, encoding='utf-8'))['parts']
buf = []
def w(s=''):
    buf.append(str(s))

w('source XML  : %s' % BASE)
w('STEP dir    : %s' % DIR)
w('')
w('%-30s %12s %12s %9s | %9s | %9s | %s' %
  ('Part', 'SW vol mm3', 'OCC vol mm3', 'vol rel', 'dCoM mm', 'inertia rel', 'solids'))
w('-' * 108)

n_ok = 0; n = 0
worst_v = worst_c = worst_i = 0.0
for p in base:
    if not p['mass_kg']:
        continue
    n += 1
    path = os.path.join(DIR, p['step'])
    if not os.path.exists(path):
        w('%-30s 文件缺失 %s' % (p['part'], p['step'])); continue
    V, cog, Iorg, nsol = props(path)
    Icom = to_com(Iorg, V, cog)
    swv = p['volume_mm3']; swc = p['com_mm']; swI = (p['inertia_mm5'] or [])[:3]
    rel_v = abs(V - swv) / swv
    dcom  = max(abs(cog[i] - swc[i]) for i in range(3))
    den   = max(abs(x) for x in swI) or 1.0
    rel_i = max(abs(Icom[i][i] - swI[i]) for i in range(3)) / den
    ok = (rel_v < 1e-6) and (dcom < 1e-4) and (rel_i < 1e-6)
    n_ok += 1 if ok else 0
    worst_v = max(worst_v, rel_v); worst_c = max(worst_c, dcom); worst_i = max(worst_i, rel_i)
    w('%-30s %12.4f %12.4f %9.2e | %9.2e | %9.2e | %d  %s' %
      (p['part'], swv, V, rel_v, dcom, rel_i, nsol, 'OK' if ok else 'DIFF'))

w('')
w('全项一致 = %d / %d' % (n_ok, n))
w('最大偏差: vol rel %.2e | CoM abs %.2e mm | inertia rel %.2e' % (worst_v, worst_c, worst_i))
w('')
w('VERDICT: %s' % ('SW_STEP_XCHECK_OK' if n_ok == n else 'SW_STEP_XCHECK_MISMATCH'))

gmsh.finalize()
open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
