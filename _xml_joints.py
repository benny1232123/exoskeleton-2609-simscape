# -*- coding: utf-8 -*-
"""定位并打印 XML 里 <Constraints>（关节）与 <Assemblies>（装配层次）两段的原文。

为什么单独写：上一版脚本用 root 的直接子节点找 <Constraints>，但实际它嵌在
<Assemblies> 里 → 误报"没有"。这里改用全文定位，直接把原文片段掏出来看。

顺带产出一份"清洗副本" <name>.sanitized.xml（XML 禁止的控制字符换成 '?'），
既可用来继续分析，也可作为应急修复的基础。

用法：python _xml_joints.py <xml> [out.txt]
"""
import os
import re
import sys

XML = sys.argv[1] if len(sys.argv) > 1 else r'D:\exo_xml\腿部设计_左.xml'
OUT = sys.argv[2] if len(sys.argv) > 2 else r'C:\Users\29408\exo_work\_xml_joints.txt'

L = []
def p(s=''):
    L.append(str(s))

raw = open(XML, 'rb').read()
san = bytearray(raw)
for i in range(len(san)):
    if san[i] < 0x20 and san[i] not in (0x09, 0x0A, 0x0D):
        san[i] = 0x3F
txt = bytes(san).decode('utf-8', 'replace')

# 顺带写一份清洗副本
san_path = os.path.splitext(XML)[0] + '.sanitized.xml'
with open(san_path, 'w', encoding='utf-8', newline='') as f:
    f.write(txt)

p('# XML 关键段落原文')
p('file      = %s  (%d bytes)' % (XML, len(raw)))
p('sanitized = %s' % san_path)
p('')

# ---------------------------------------------------------------- 顶层结构
p('=' * 70)
p('顶层元素一览（按出现顺序，只看深度 1）')
p('=' * 70)
for m in re.finditer(r'<(/?)([A-Za-z][\w:]*)((?:"[^"]*"|[^>"])*?)(/?)>', txt):
    close, tag, attrs, selfclose = m.groups()
    pass
# 用缩进近似判断深度：这里直接扫所有标签，统计哪些是"根的直接子"
depth = 0
i = 0
top = []
for m in re.finditer(r'<(/?)([A-Za-z][\w:]*)((?:"[^"]*"|[^>"])*?)(/?)>', txt):
    close, tag, attrs, selfclose = m.groups()
    if tag.startswith('?') or tag.startswith('!'):
        continue
    if close:
        depth -= 1
        continue
    if depth == 0:
        top.append((tag, attrs.strip()[:80], bool(selfclose)))
    if not selfclose:
        depth += 1
for t, a, sc in top:
    p('  <%s> %s %s' % (t, a, '(self-closed)' if sc else ''))
p('')

# ---------------------------------------------------------------- 段落提取
def slice_tag(text, tag):
    """掏出 <tag>...</tag> 的原文；找不到返回 None。"""
    m = re.search(r'<%s(\s[^>]*)?>' % tag, text)
    if not m:
        return None
    start = m.start()
    endm = text.find('</%s>' % tag, start)
    if endm < 0:
        return text[start:start + 4000]
    return text[start:endm + len(tag) + 3]

for tag in ('Constraints', 'Assemblies', 'InstanceTree', 'RootAssembly'):
    seg = slice_tag(txt, tag)
    p('=' * 70)
    p('<%s> 段落' % tag)
    p('=' * 70)
    if seg is None:
        p('  !! 全文里找不到 <%s>' % tag)
        p('')
        continue
    p('  原文长度 = %d 字符' % len(seg))
    p('')
    # 逐行美化：标签前换行，方便看
    pretty = re.sub(r'><', '>\n<', seg)
    lines = pretty.split('\n')
    p('  子元素/层级预览（前 160 行）：')
    for ln in lines[:160]:
        p('    ' + ln[:150])
    if len(lines) > 160:
        p('    ... 其余 %d 行省略' % (len(lines) - 160))
    p('')

# ---------------------------------------------------------------- 关节计数
p('=' * 70)
p('关节计数（多口径交叉验证）')
p('=' * 70)
seg = slice_tag(txt, 'Constraints') or ''
inner = re.sub(r'^<Constraints[^>]*>', '', seg)
inner = re.sub(r'</Constraints>$', '', inner)
p('  <Constraints> 内部长度 = %d' % len(inner))
p('  <Constraints> 内部原文 = %r' % inner[:400])
tags_in = re.findall(r'<([A-Za-z][\w:]*)[\s/>]', inner)
p('  内部标签 = %s' % (tags_in if tags_in else '（空）'))
p('')
for kw in ('Constraint', 'Joint', 'Mate', 'Revolute', 'Prismatic', 'Fixed', 'Weld', 'Cylindrical', 'Ball', 'Planar', 'Screw', 'Gear', 'Rack'):
    p('  含 %-12s 次数 = %d' % (kw, txt.count(kw)))

p('')
p('# 完成')
open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('written -> %s' % OUT)
print('sanitized -> %s' % san_path)
