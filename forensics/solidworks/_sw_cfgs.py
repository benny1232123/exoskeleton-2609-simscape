# -*- coding: utf-8 -*-
"""只读探测：连上正在运行的 SolidWorks，列出所有打开文档 + 每个文档的配置名。
输出写 UTF-8 文件，避免 PowerShell 控制台编码问题。"""
import sys, io, os, traceback

OUT = r'C:\Users\29408\exo_work\_sw_cfgs.txt'
buf = []

def w(s=''):
    buf.append(str(s))

def esc(s):
    """把非 ASCII / 控制字符显式标出来，方便看坏字节"""
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

def hexdump(s):
    try:
        return s.encode('utf-8').hex(' ')
    except Exception:
        return '<enc-err>'

def nonascii_count(s):
    return sum(1 for ch in s if ord(ch) > 126 or ord(ch) < 32)

try:
    import win32com.client as wc
    from win32com.client import GetActiveObject
except Exception as e:
    w('IMPORT_FAIL ' + repr(e))
    open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
    sys.exit(1)

w('== SolidWorks COM probe ==')
try:
    sw = GetActiveObject('SldWorks.Application')
except Exception as e:
    w('NO_ACTIVE_SW: ' + repr(e))
    w('(SolidWorks 未在运行，或权限级别不一致)')
    open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
    sys.exit(2)

try:
    w('RevisionNumber = ' + str(sw.RevisionNumber))
except Exception as e:
    w('rev err ' + repr(e))

docs = []
try:
    d = sw.GetFirstDocument()
    while d is not None:
        rec = {}
        try:
            rec['title'] = d.GetTitle()
        except Exception:
            rec['title'] = '?'
        try:
            rec['path'] = d.GetPathName()
        except Exception:
            rec['path'] = '?'
        try:
            rec['type'] = d.GetType()   # 1=part 2=assembly 3=drawing
        except Exception:
            rec['type'] = -1
        try:
            names = d.GetConfigurationNames()
            rec['cfgs'] = list(names) if names else []
        except Exception as e:
            rec['cfgs'] = ['<err %r>' % e]
        try:
            ac = d.ConfigurationManager.ActiveConfiguration
            rec['active'] = ac.Name
        except Exception as e:
            rec['active'] = '<err %r>' % e
        # 试着看 Name 是否可写（不真的写，只探测属性是否存在）
        try:
            rec['has_setname'] = hasattr(d, 'SetConfigurationName')
        except Exception:
            rec['has_setname'] = False
        docs.append(rec)
        d = sw.GetNextDocument(d)
except Exception:
    w('ENUM_FAIL')
    w(traceback.format_exc())

w('open docs = %d' % len(docs))
TYPEMAP = {1: 'PART', 2: 'ASSEMBLY', 3: 'DRAWING'}
for i, r in enumerate(docs):
    w('')
    w('--- [%d] %s  type=%s' % (i, esc(r['title']), TYPEMAP.get(r['type'], r['type'])))
    w('    path   = %s' % esc(r['path']))
    w('    active = %s' % esc(r['active']))
    cfgs = r['cfgs']
    w('    cfgs(%d):' % len(cfgs))
    for c in cfgs:
        n = nonascii_count(c) if isinstance(c, str) else -1
        flag = '  <<< NON-ASCII %d' % n if n > 0 else ''
        w('      |%s|%s' % (esc(c), flag))
        if n > 0:
            w('         utf8 = %s' % hexdump(c))

open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
