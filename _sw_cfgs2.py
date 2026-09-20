# -*- coding: utf-8 -*-
"""SolidWorks COM 配置名探测 v2：早绑定 + 多条文档枚举路径取通的那条。"""
import sys, traceback

OUT = r'C:\Users\29408\exo_work\_sw_cfgs2.txt'
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
def na(s):
    return sum(1 for ch in s if ord(ch) > 126 or ord(ch) < 32)

sw = None

# --- 路径 1：gencache 早绑定（会从类型库生成包装，能解析出 GetFirstDocument） ---
try:
    import win32com.client.gencache as gc
    sw = gc.EnsureDispatch('SldWorks.Application')
    w('BIND = gencache.EnsureDispatch  OK')
except Exception as e:
    w('BIND gencache FAIL: %r' % (e,))
    try:
        from win32com.client import GetActiveObject
        sw = GetActiveObject('SldWorks.Application')
        w('BIND = GetActiveObject (late)  OK')
    except Exception as e2:
        w('BIND all FAIL: %r' % (e2,))
        open(OUT, 'w', encoding='utf-8').write('\n'.join(buf)); sys.exit(2)

w('RevisionNumber = %s' % (sw.RevisionNumber,))

docs = []
# --- 枚举路径 A: GetDocuments() ---
if not docs:
    try:
        arr = sw.GetDocuments()
        if arr:
            docs = list(arr)
        w('ENUM A GetDocuments -> %d' % len(docs))
    except Exception as e:
        w('ENUM A fail: %r' % (e,))

# --- 枚举路径 B: EnumDocuments2 ---
if not docs:
    try:
        en = sw.EnumDocuments2()
        tmp = []
        while True:
            batch = en.Next(1)
            if not batch:
                break
            tmp.extend(list(batch))
        docs = tmp
        w('ENUM B EnumDocuments2 -> %d' % len(docs))
    except Exception as e:
        w('ENUM B fail: %r' % (e,))

# --- 枚举路径 C: GetFirstDocument/GetNextDocument ---
if not docs:
    try:
        d = sw.GetFirstDocument()
        while d is not None:
            docs.append(d)
            d = sw.GetNextDocument(d)
        w('ENUM C GetFirstDocument -> %d' % len(docs))
    except Exception as e:
        w('ENUM C fail: %r' % (e,))

# --- 枚举路径 D: 只有 ActiveDoc ---
if not docs:
    try:
        d = sw.ActiveDoc
        if d is not None:
            docs = [d]
        w('ENUM D ActiveDoc -> %d' % len(docs))
    except Exception as e:
        w('ENUM D fail: %r' % (e,))

TYPEMAP = {1: 'PART', 2: 'ASSEMBLY', 3: 'DRAWING'}
w('open docs = %d' % len(docs))
w('')
for i, d in enumerate(docs):
    w('--- [%d]' % i)
    for label, fn in (('title', lambda: d.GetTitle()),
                      ('path', lambda: d.GetPathName()),
                      ('type', lambda: d.GetType())):
        try:
            v = fn()
            if label == 'type':
                w('    %-8s = %s (%s)' % (label, v, TYPEMAP.get(v, '?')))
            else:
                w('    %-8s = %s' % (label, esc(v)))
        except Exception as e:
            w('    %-8s ERR %r' % (label, e))
    try:
        ac = d.ConfigurationManager.ActiveConfiguration
        w('    active  = %s' % esc(ac.Name))
    except Exception as e:
        w('    active  ERR %r' % (e,))
    try:
        names = d.GetConfigurationNames()
        names = list(names) if names else []
        w('    cfgs(%d):' % len(names))
        for c in names:
            n = na(c)
            w('      |%s|%s' % (esc(c), ('   <<< NON-ASCII x%d' % n) if n else ''))
            if n:
                try:
                    w('          utf8 = %s' % c.encode('utf-8').hex(' '))
                except Exception:
                    pass
    except Exception as e:
        w('    cfgs ERR %r' % (e,))
    w('')

# 探测改名 API 是否存在
try:
    d = docs[0]
    for m in ('SetConfigurationName', 'RenameConfiguration', 'EditConfiguration',
              'AddConfiguration', 'DeleteConfiguration', 'GetConfigurationByName'):
        w('has_%s = %s' % (m, hasattr(d, m)))
    try:
        cm = d.ConfigurationManager
        for m in ('GetConfigurationByName', 'AddConfiguration', 'GetConfigurationCount',
                  'ActiveConfiguration', 'GetConfigurationByName'):
            w('CM.has_%s = %s' % (m, hasattr(cm, m)))
    except Exception as e:
        w('CM err %r' % (e,))
    try:
        c0 = d.GetConfigurationByName(d.GetConfigurationNames()[0])
        props = [p for p in dir(c0) if not p.startswith('_')]
        w('CFG props: %s' % ', '.join(props[:80]))
    except Exception as e:
        w('CFG probe err %r' % (e,))
except Exception as e:
    w('api probe err %r' % (e,))

open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
