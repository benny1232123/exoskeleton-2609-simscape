# -*- coding: utf-8 -*-
"""读每个配置的完整元数据（AlternateName / Comment / Description 等），
   找出插件拼进 STEP 文件名的那段"垃圾后缀"到底来自哪个字段。"""
import sys, traceback

OUT = r'C:\Users\29408\exo_work\_sw_cfg_meta.txt'
buf = []
def w(s=''):
    buf.append(str(s))
def esc(s):
    if s is None:
        return '<None>'
    out = []
    for ch in s:
        o = ord(ch)
        if 32 <= o < 127:
            out.append(ch)
        elif o < 32 or o == 127:
            out.append('<%02X>' % o)
        else:
            out.append(ch)
    return ''.join(out)
def getv(o, name, *args):
    a = getattr(o, name)
    return a(*args) if callable(a) else a

from win32com.client import GetActiveObject
sw = GetActiveObject('SldWorks.Application')
docs = list(getv(sw, 'GetDocuments'))
w('docs = %d' % len(docs))

# 关注这 8 个"脏"件 + 几个"干净"件做对照
DIRTY = ['电机_出轴', '电机_轴', '腿部_按键_双', '腿部_滑轨_片形',
         '腿部_滑轨挡片', '腿部_绑缚', '腿部_腿杆_片状V5', '腿部_轴盖']
CLEAN = ['腿部_螺丝_M3_4', '腿部_顶丝_M4_4', '腿部_滑块_双键', '电机_黄铜轴套8_10_18']

def short(p):
    return p.rsplit('\\', 1)[-1] if p else ''

for tag, want in (('DIRTY', DIRTY), ('CLEAN', CLEAN)):
    w('')
    w('########## %s ##########' % tag)
    for d in docs:
        try:
            title = getv(d, 'GetTitle')
        except Exception:
            continue
        if title not in want:
            continue
        w('')
        w('--- %s   (%s)' % (title, short(getv(d, 'GetPathName'))))
        # 文档级可能的字符串字段
        for f in ('GetAlternateName', 'AlternateName', 'GetDescription', 'GetType'):
            try:
                v = getv(d, f)
                if isinstance(v, str):
                    w('    doc.%-18s = |%s|' % (f, esc(v)))
            except Exception:
                pass
        try:
            cfg = d.GetConfigurationByName('默认')
        except Exception as e:
            w('    GetConfigurationByName ERR %r' % (e,))
            continue
        # 逐字段 dump
        props = [p for p in dir(cfg) if not p.startswith('_')]
        w('    cfg props (%d): %s' % (len(props), ', '.join(props)))
        for p in ('Name', 'AlternateName', 'Comment', 'Description', 'IsDerived',
                  'IsDefeatured', 'ParentConfigurationName', 'BOMPartNumberSource',
                  'ConfigurationType', 'HideInBOM', 'ShowChildComponentsInBOM',
                  'CustomProperties', 'RepresentationParent'):
            try:
                v = getattr(cfg, p)
                if callable(v):
                    continue
                if isinstance(v, str):
                    w('    cfg.%-24s = |%s|' % (p, esc(v)))
                else:
                    w('    cfg.%-24s = %r' % (p, v))
            except Exception as e:
                w('    cfg.%-24s ERR %r' % (p, e))

# ---- 配置的自定义属性（可能是"零件号"之类）----
w('')
w('########## 配置自定义属性 ##########')
for d in docs[:6]:
    try:
        title = getv(d, 'GetTitle')
        cfg = d.GetConfigurationByName('默认')
        cpm = cfg.CustomPropertyManager
        n = cpm.Count
        w('--- %s  props=%d' % (title, n))
        for i in range(n):
            nm = cpm.GetPropertyNameByIndex(i)
            v = cpm.Get(nm)
            w('      %-24s = |%s|' % (esc(nm), esc(str(v))))
    except Exception as e:
        w('    %s ERR %r' % (title, e))

open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
