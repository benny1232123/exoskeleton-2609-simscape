# -*- coding: utf-8 -*-
"""XML 结构体检：在跑 smimport 之前先把 SolidWorks 导出的 XML 拆开看。

不开 MATLAB，纯 Python 解析。目的：
  1. XML 是否**合法**（非法 UTF-8 字节 + XML 禁止的控制字符，逐个定位）★ 最关键
  2. 一共几个刚体、几个关节（关节 = mates 映射的结果，路径 A 的核心产物）
  3. 每个刚体的质量 / 惯量 / 质心 —— 有没有 mass=0（= 零件没赋材质）
  4. XML 里引用的 STEP 几何文件，是否都在磁盘上真实存在
  5. 与 URDF 路的参考数字对账

坏字节处理：先逐字节扫出非法 UTF-8 与 XML 禁止的控制字符并报出行列号；
再把它们换成 '?' 造一份"可解析"的副本，好把结构信息照样挖出来
（结构本身通常是好的，坏的只是名字里的字节）。

用法：python _xml_inspect.py <xml路径> [输出txt路径]
"""
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter

XML = sys.argv[1] if len(sys.argv) > 1 else r'D:\exo_xml\腿部设计_左.xml'
OUT = sys.argv[2] if len(sys.argv) > 2 else r'C:\Users\29408\exo_work\_xml_inspect.txt'

L = []
def p(s=''):
    L.append(str(s))

def local(tag):
    return tag.rsplit('}', 1)[-1] if '}' in tag else tag

def lineno(raw, off):
    return raw.count(b'\n', 0, off) + 1

def colno(raw, off):
    nl = raw.rfind(b'\n', 0, off)
    return off - nl if nl >= 0 else off + 1

p('# Simscape Multibody Link XML 结构体检')
p('file = %s' % XML)
if not os.path.exists(XML):
    p('!! 文件不存在')
    open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
    sys.exit(1)

raw = open(XML, 'rb').read()
p('size = %d bytes' % len(raw))
p('head = %r' % raw[:200])

# ============================================================ 1. 合法性扫描
p('')
p('## 1. 合法性扫描')
CTRL_OK = (0x09, 0x0A, 0x0D)          # XML 1.0 允许的少数控制字符

# 1a. 非法 UTF-8
bad_spans = []
pos, n = 0, len(raw)
while pos < n:
    try:
        raw[pos:].decode('utf-8')
        break
    except UnicodeDecodeError as e:
        s, t = pos + e.start, pos + e.end
        if t <= s:
            t = s + 1
        bad_spans.append((s, t))
        pos = t

# 1b. XML 禁止的控制字符
ctrl = [(i, b) for i, b in enumerate(raw) if b < 0x20 and b not in CTRL_OK]

p('非法 UTF-8 字节段 = %d 处' % len(bad_spans))
for s, t in bad_spans[:60]:
    p('    L%-5d C%-4d  bytes=%s  ctx=%r'
      % (lineno(raw, s), colno(raw, s), raw[s:t].hex(), raw[max(0, s - 26):t + 26]))
if len(bad_spans) > 60:
    p('    ... 其余 %d 处省略' % (len(bad_spans) - 60))

p('XML 禁止的控制字符 = %d 个' % len(ctrl))
for i, b in ctrl[:40]:
    p('    L%-5d C%-4d  0x%02X' % (lineno(raw, i), colno(raw, i), b))
if len(ctrl) > 40:
    p('    ... 其余 %d 个省略' % (len(ctrl) - 40))

# 污染行现场
dock_lines = sorted({lineno(raw, i) for i, b in ctrl} | {lineno(raw, s) for s, t in bad_spans})
if dock_lines:
    pl = raw.split(b'\n')
    p('')
    p('  污染行原始字节（最多 5 行）：')
    for ln in dock_lines[:5]:
        if 0 <= ln - 1 < len(pl):
            p('    L%d: %r' % (ln, pl[ln - 1]))

legal = (not bad_spans) and (not ctrl)
p('')
if legal:
    p('  ✅ 合法 XML —— 可以放心交给 smimport')
else:
    p('  ★★ 不合法 XML —— smimport 必然报 "not well-formed (invalid token)"。')
    p('     必须先从源头修（见文末【修法】）。')

# ============================================================ 2. 清洗副本
san = bytearray(raw)
for i in range(len(san)):
    if san[i] < 0x20 and san[i] not in CTRL_OK:
        san[i] = 0x3F                  # '?'
san_txt = bytes(san).decode('utf-8', 'replace')

p('')
p('## 2. 结构解析（用清洗副本，坏字节已换成 ?）')
root = None
try:
    root = ET.fromstring(san_txt)
except Exception as e:
    p('!! 清洗后仍解析失败: %r' % e)

if root is not None:
    p('root tag    = %s   (local=%s)' % (root.tag, local(root.tag)))
    p('root attrib = %r' % (root.attrib,))
    m = re.match(r'\{.*\}', root.tag)
    p('namespace   = %r' % (m.group(0)[1:-1] if m else ''))

    cnt = Counter()
    dmax = 0
    stack = [(root, 1)]
    while stack:
        e, d = stack.pop()
        dmax = max(dmax, d)
        cnt[local(e.tag)] += 1
        for c in e:
            stack.append((c, d + 1))
    p('')
    p('## 3. 标签普查（树深 %d）' % dmax)
    for t, c in cnt.most_common():
        p('  %-36s %d' % (t, c))

    p('')
    p('## 4. 属性键普查')
    acnt = Counter()
    for e in root.iter():
        for k in e.attrib:
            acnt[local(k)] += 1
    for k, c in acnt.most_common():
        p('  %-36s %d' % (k, c))

    # ---------------------------------------------------- 质量属性
    # ⚠ 踩过：质量不是属性，是 <MassProperties><Mass>0.0123</Mass> 这种**元素文本**
    p('')
    p('## 5. 质量属性（逐个 <Part>）')
    rows = []
    for part in root.iter():
        if local(part.tag) != 'Part':
            continue
        pa = {local(k): v for k, v in part.attrib.items()}
        nm = pa.get('name', '(无名)')
        mv = float('nan')
        com = inr = geo = ''
        for g in part:
            lg = local(g.tag)
            ga = {local(k): v for k, v in g.attrib.items()}
            if lg == 'GeometryFile':
                geo = ga.get('name', '')
            elif lg == 'MassProperties':
                for h in g:
                    lh = local(h.tag)
                    ha = {local(k): v for k, v in h.attrib.items()}
                    if lh == 'Mass':
                        v = ha.get('value', ha.get('mass', (h.text or '').strip()))
                        try:
                            mv = float(v)
                        except (TypeError, ValueError):
                            pass
                    elif lh == 'CenterOfMass':
                        com = (h.text or '').strip() or str(ha)
                    elif lh == 'Inertia':
                        inr = (h.text or '').strip() or str(ha)
        rows.append((nm, mv, com, inr, geo))

    total = 0.0
    zero = []
    for name, mv, com, inr, geo in rows:
        p('  %-38s Mass=%-12s' % (name, mv))
        if com:
            p('        CoM     = %s' % com)
        if inr:
            p('        Inertia = %s' % inr)
        if geo:
            p('        STEP    = %s' % geo)
        if mv == 0:
            zero.append(name)
        elif mv == mv:
            total += mv
    p('')
    p('  <Part> 数     = %d' % len(rows))
    p('  Σ Mass        = %.6f kg' % total)
    p('  Mass==0 零件  = %d  %s'
      % (len(zero), ('→ ' + '、'.join(zero)) if zero else ''))
    tiny = [n for n, m, _, _, _ in rows if 0 < m < 1e-4]
    if tiny:
        p('  ★ Mass < 0.1 g 的零件 %d 个：%s' % (len(tiny), '、'.join(tiny)))
        p('    这些质量与其几何尺寸可能不匹配（没赋材质 / 是面体），值得单独确认')

    # ---------------------------------------------------- 关节
    p('')
    p('## 6. 关节 / 配合映射')
    jel = [e for e in root.iter() if 'oint' in local(e.tag)]
    p('  标签名含 "oint" 的元素 = %d' % len(jel))
    jt = Counter()
    for e in jel:
        a = {local(k): v for k, v in e.attrib.items()}
        jt[a.get('Type', '(无Type)')] += 1
    for t, c in jt.most_common():
        p('    Type=%-22s %d' % (t, c))
    if not jel:
        p('    ★ 一个关节都没有 —— 装配体 mates 没被识别（路径 A 头号失败原因）')

    # ---------------------------------------------------- 几何引用
    p('')
    p('## 7. 几何文件引用核对')
    refs = set()
    for e in root.iter():
        for v in e.attrib.values():
            if isinstance(v, str) and re.search(r'\.(STEP|stp|step|sldprt|SLDPRT)$', v):
                refs.add(v)
    xml_dir = os.path.dirname(XML)
    on_disk = set(os.listdir(xml_dir))
    p('  XML 引用 %d 个' % len(refs))
    okc = mc = 0
    for r in sorted(refs):
        b = os.path.basename(r)
        ex = b in on_disk
        okc += ex
        mc += (not ex)
        p('    [%s] %s' % ('OK ' if ex else 'MISS', b))
    p('  对得上 %d / 缺 %d' % (okc, mc))
    unref = [f for f in sorted(on_disk)
             if f != os.path.basename(XML)
             and f not in {os.path.basename(r) for r in refs}]
    p('')
    p('  磁盘上有但 XML 没引用：%s' % ('、'.join(unref) if unref else '（无）'))

# ============================================================ 8. 乱码文件名
p('')
p('## 8. 磁盘上可疑文件名')
sus = []
xml_dir = os.path.dirname(XML)
try:
    for f in sorted(os.listdir(xml_dir)):
        if '\ufffd' in f or '?' in f:
            sus.append(f)
except Exception as ex:
    p('  列目录失败 %r' % ex)
for f in sus:
    p('    %r' % f)
if not sus:
    p('  （未发现替换字符）')

# ============================================================ 9. 修法
p('')
p('# 修法（按推荐顺序）')
p('')
p('A. ★ 首选：把 SolidWorks 里那几个零件的「配置名」改成纯 ASCII 再重新导出。')
p('   坏字节来自配置名 —— 插件把它拼进 Name/文件名时吐出了原始字节。')
p('   操作：SolidWorks 打开零件 → 配置管理器 → 右键配置 → 属性 → 改名为')
p('   Default / cfg1 这类纯 ASCII → 保存 → 重新导出。')
p('')
p('B. 省事版：把 .SLDASM 另存为纯 ASCII 文件名再导出（顺带解决 XML 名带中文）。')
p('')
p('C. 应急版（先跑通流程用）：把 XML 里那 10 个控制字符替换成 ?，')
p('   并同步重命名磁盘上对应的 .STEP，使引用与文件名一致。')
p('   注意：这只修"合法性"，不改几何/质量，但不解决源头。')
p('')
p('# 完成')

open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('written -> %s' % OUT)
