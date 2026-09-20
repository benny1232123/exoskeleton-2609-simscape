# -*- coding: utf-8 -*-
"""SolidWorks COM 探测 v3：
1) 用「属性/方法自适应」的方式列出所有打开文档 + 配置名
2) 用 comtypes 加载 SW 类型库，挖出改配置名 API 的准确签名
"""
import sys, traceback

OUT = r'C:\Users\29408\exo_work\_sw_cfgs3.txt'
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

def getv(o, name, *args):
    a = getattr(o, name)
    return a(*args) if callable(a) else a

# ---------- 第一部分：文档与配置名 ----------
from win32com.client import GetActiveObject
sw = GetActiveObject('SldWorks.Application')
w('== SW revision %s ==' % (sw.RevisionNumber,))

docs = []
try:
    v = getv(sw, 'GetDocuments')
    docs = list(v) if v else []
    w('GetDocuments -> %d' % len(docs))
except Exception as e:
    w('GetDocuments fail %r' % (e,))
if not docs:
    try:
        d = sw.ActiveDoc
        if d is not None:
            docs = [d]
        w('fallback ActiveDoc -> %d' % len(docs))
    except Exception as e:
        w('ActiveDoc fail %r' % (e,))

TYPEMAP = {1: 'PART', 2: 'ASSEMBLY', 3: 'DRAWING'}
w('')
for i, d in enumerate(docs):
    w('--- [%d] ---' % i)
    for label in ('GetTitle', 'GetPathName', 'GetType'):
        try:
            w('    %-12s = %s' % (label, esc(getv(d, label))))
        except Exception as e:
            w('    %-12s ERR %r' % (label, e))
    try:
        t = getv(d, 'GetType')
        w('    typeof      = %s' % TYPEMAP.get(t, t))
    except Exception:
        pass
    try:
        ac = d.ConfigurationManager.ActiveConfiguration
        w('    active      = %s' % esc(ac.Name))
    except Exception as e:
        w('    active ERR %r' % (e,))
    try:
        names = getv(d, 'GetConfigurationNames')
        names = list(names) if names else []
        w('    cfgs(%d):' % len(names))
        for c in names:
            n = na(c)
            w('      |%s|%s' % (esc(c), ('   <<< NON-ASCII x%d' % n) if n else ''))
            if n:
                w('          utf8 = %s' % c.encode('utf-8', 'replace').hex(' '))
                w('          gbk  = %s' % c.encode('gbk', 'replace').hex(' '))
    except Exception as e:
        w('    cfgs ERR %r' % (e,))
    w('')

# ---------- 第二部分：类型库挖 API 签名 ----------
w('')
w('==== comtypes typelib probe ====')
try:
    import comtypes.client as cc
    exe = r'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\sldworks.exe'
    mod = cc.GetModule(exe)
    w('GetModule OK -> %s' % getattr(mod, '__name__', '?'))
    import comtypes
    hits = []
    for nm, obj in vars(mod).items():
        ms = getattr(obj, '_methods_', None)
        if not ms:
            continue
        for m in ms:
            if not isinstance(m, (list, tuple)) or not m:
                continue
            fn = m[0]
            if isinstance(fn, str) and 'onfiguration' in fn:
                hits.append((nm, m))
    w('methods containing "onfiguration": %d' % len(hits))
    seen = set()
    for iface, m in hits:
        key = (iface, m[0])
        if key in seen:
            continue
        seen.add(key)
        # comtypes _methods_ entry: (name, restype, argtypes_and_names...)
        w('')
        w('  [%s] %s' % (iface, m[0]))
        for j, item in enumerate(m[1:], 1):
            w('       %d) %s' % (j, repr(item)))
except Exception:
    w('TYPELIB PROBE FAIL')
    w(traceback.format_exc())

open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
