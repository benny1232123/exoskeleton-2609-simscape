# -*- coding: utf-8 -*-
"""把 SolidWorks 导出的 Simscape Multibody XML 的关键内容 dump 出来。

重点回答三个问题：
  A. 每个刚体的质量/惯量/质心是多少？有没有 mass=0（= 零件没赋材质）
  B. <Constraints> 里到底有没有关节？（路径 A 的核心产物，没有就白导）
  C. <InstanceTree> 的装配层次 + 哪些是 grounded（固定到世界）

用法：python _xml_dump.py <xml> [out.txt]
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

XML = sys.argv[1] if len(sys.argv) > 1 else r'D:\exo_xml\腿部设计_左.xml'
OUT = sys.argv[2] if len(sys.argv) > 2 else r'C:\Users\29408\exo_work\_xml_dump.txt'

L = []
def p(s=''):
    L.append(str(s))

def local(t):
    return t.rsplit('}', 1)[-1] if '}' in t else t

def A(e):
    return {local(k): v for k, v in e.attrib.items()}

def cut(s, n=58):
    s = str(s)
    return s if len(s) <= n else s[:n] + '…'

raw = open(XML, 'rb').read()
san = bytearray(raw)
for i in range(len(san)):
    if san[i] < 0x20 and san[i] not in (0x09, 0x0A, 0x0D):
        san[i] = 0x3F
root = ET.fromstring(bytes(san).decode('utf-8', 'replace'))

p('# XML 关键内容 dump')
p('file = %s  (%d bytes)' % (XML, len(raw)))
p('')

# ============================================================ A. Parts / 质量
p('=' * 70)
p('A. <Parts> —— 刚体质量属性')
p('=' * 70)
parts_el = None
for e in root:
    if local(e.tag) == 'Parts':
        parts_el = e
if parts_el is None:
    p('!! 没有 <Parts>')
else:
    allp = [c for c in parts_el if local(c.tag) == 'Part']
    p('<Part> 个数 = %d' % len(allp))
    p('')
    tot = 0.0
    zero = []
    for c in allp:
        a = A(c)
        nm = a.get('name', '(无名)')
        geo = ''
        mass = com = inr = None
        pf = ''
        for g in c:
            lt = local(g.tag)
            if lt == 'GeometryFile':
                geo = A(g).get('name', '')
            elif lt == 'PartFile':
                pf = A(g).get('name', '')
            elif lt == 'MassProperties':
                for h in g:
                    lh = local(h.tag)
                    if lh == 'Mass':
                        mass = h
                    elif lh == 'CenterOfMass':
                        com = h
                    elif lh == 'Inertia':
                        inr = h
        mv = None
        if mass is not None:
            ha = A(mass)
            v = ha.get('value', ha.get('mass', mass.text))
            try:
                mv = float(v)
            except (TypeError, ValueError):
                mv = None
        p('  Part: %s' % cut(nm))
        p('        uid=%s' % cut(A(c).get('uid', ''), 40))
        if mv is not None:
            p('        Mass = %.6f' % mv)
            if mv == 0:
                zero.append(nm)
            else:
                tot += mv
        if com is not None:
            p('        CoM  = %s' % A(com))
        if inr is not None:
            ia = A(inr)
            p('        Inertia keys = %s' % list(ia.keys()))
            p('        Inertia vals = %s' % [cut(v, 18) for v in ia.values()])
        p('        STEP = %s' % cut(geo, 70))
        if pf:
            p('        SLDPRT(回链) = %s' % cut(pf, 56))
        p('')
    p('-' * 70)
    p('Σ Mass = %.6f' % tot)
    p('Mass==0 的零件 = %d  %s' % (len(zero), ('→ ' + '、'.join(zero)) if zero else ''))
    p('★ 若有 mass==0 的零件，说明它在 SolidWorks 里没赋材质 →'
      ' Simscape 里会变成无质量刚体，动力学直接废掉。')

# ============================================================ B. Constraints
p('')
p('=' * 70)
p('B. <Constraints> —— 关节（mates 映射的结果）')
p('=' * 70)
cst = [e for e in root if local(e.tag) == 'Constraints']
if not cst:
    p('!! 根下没有 <Constraints>')
else:
    c = cst[0]
    ca = A(c)
    kids = list(c)
    p('<Constraints> 属性 = %s' % ca)
    p('<Constraints> 子元素数 = %d' % len(kids))
    if not kids:
        p('')
        p('★★ 空的 —— 这份 XML **一个关节都没有**。')
        p('   = 装配体的 mates 没有被映射成 joints（路径 A 的头号失败原因）。')
        p('   导进 Simscape 只会得到 19 个零件像一堆摆件一样固定在原位的静态模型。')
    else:
        for k in kids:
            p('  子元素 <%s> %s' % (local(k.tag), A(k)))
        p('')
        p('  各子元素再往下（最深 3 层）：')
        for k in kids:
            st = [(k, 1)]
            while st:
                e, d = st.pop()
                if d > 3:
                    continue
                p('    %s<%s> %s' % ('  ' * d, local(e.tag),
                                     {kk: cut(vv, 22) for kk, vv in A(e).items()}))
                for ch in e:
                    st.append((ch, d + 1))

# ============================================================ C. InstanceTree
p('')
p('=' * 70)
p('C. <InstanceTree> —— 装配层次 + grounded')
p('=' * 70)
it = [e for e in root if local(e.tag) == 'InstanceTree']
if not it:
    p('!! 根下没有 <InstanceTree>')
else:
    st = [(it[0], 0)]
    n_inst = 0
    n_ground = 0
    while st:
        e, d = st.pop()
        lt = local(e.tag)
        if lt == 'Instance':
            n_inst += 1
            a = A(e)
            gr = a.get('grounded', '')
            if str(gr).lower() in ('true', '1'):
                n_ground += 1
            p('  %s<Instance> name=%-40s grounded=%s' % ('  ' * d, cut(a.get('name', ''), 40), gr))
        elif lt in ('InstanceTree', 'Assembly', 'Assemblies', 'RootAssembly', 'Children'):
            p('  %s<%s>' % ('  ' * d, lt))
        for ch in e:
            st.append((ch, d + 1))
    p('')
    p('Instance 总数 = %d，其中 grounded = %d' % (n_inst, n_ground))

# ============================================================ D. Transform
p('')
p('=' * 70)
p('D. 每个 Instance 的 Transform（姿态/位置）')
p('=' * 70)
nt = 0
for e in root.iter():
    if local(e.tag) == 'Instance':
        a = A(e)
        tr = [c for c in e if local(c.tag) == 'Transform']
        if tr:
            nt += 1
            t = tr[0]
            tt = [c for c in t if local(c.tag) == 'Translation']
            rr = [c for c in t if local(c.tag) == 'Rotation']
            p('  %-38s  T=%s' % (cut(a.get('name', ''), 38), A(tt[0]) if tt else '?'))
            p('  %-38s  R=%s' % ('', A(rr[0]) if rr else '?'))
p('  有 Transform 的 Instance = %d' % nt)

# ============================================================ E. 单位
p('')
p('=' * 70)
p('E. 单位声明')
p('=' * 70)
for e in root.iter():
    lt = local(e.tag)
    if lt in ('ModelUnits', 'DataUnits', 'Created', 'AssemblyFile'):
        p('  <%s> %s' % (lt, {k: cut(v, 40) for k, v in A(e).items()}))

p('')
p('# 完成')
open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('written -> %s' % OUT)
