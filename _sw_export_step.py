# -*- coding: utf-8 -*-
"""从 SolidWorks 直接把缺失的 STEP 导出到 D:\exo_xml。
只导出真正丢件的那 2 个（其余 17 个插件已经写出来了）。
用 SaveAs4 + swSaveAsOptions_Copy(2)|Silent(1)，保证源文档路径不被改动。"""
import os, sys, traceback

OUT = r'C:\Users\29408\exo_work\_sw_export_step.txt'
buf = []
def w(s=''):
    buf.append(str(s))
def esc(s):
    if s is None:
        return '<None>'
    return ''.join(ch if 32 <= ord(ch) < 127 else ('<%02X>' % ord(ch) if ord(ch) < 32 else ch)
                   for ch in s)
def prop(o, n, d=None):
    try: return getattr(o, n)
    except Exception: return d

from win32com.client import GetActiveObject
sw = GetActiveObject('SldWorks.Application')
docs = list(prop(sw, 'GetDocuments') or [])
w('docs = %d' % len(docs))

DEST = r'D:\exo_xml'
TARGETS = {
    '腿部_腿杆_片状V5': '腿部_腿杆_片状V5_Default_sldprt.STEP',
    '腿部_轴盖':        '腿部_轴盖_Default_sldprt.STEP',
}

w('SaveAs4 可用 = %s' % hasattr(docs[0] if docs else object(), 'SaveAs4'))
w('')

for title, newname in TARGETS.items():
    doc = None
    for d in docs:
        if prop(d, 'GetTitle') == title:
            doc = d
            break
    w('=== %s ===' % title)
    if doc is None:
        w('   文档未打开，跳过')
        continue
    before = prop(doc, 'GetPathName')
    w('   原路径 = %s' % esc(before))
    dest = os.path.join(DEST, newname)
    if os.path.exists(dest):
        w('   目标已存在，跳过: %s' % newname)
        continue
    ok = False
    err = warn = 0
    try:
        # SaveAs4(Name, Version, Options, Errors, Warnings)
        r = doc.SaveAs4(dest, 0, 3, 0, 0)
        w('   SaveAs4 -> %r' % (r,))
        ok = bool(r) if r is not None else True
    except Exception as e:
        w('   SaveAs4 异常: %r' % (e,))
        try:
            r = doc.SaveAs(dest, 0, 3, 0)
            w('   SaveAs -> %r' % (r,))
            ok = True
        except Exception as e2:
            w('   SaveAs 异常: %r' % (e2,))
    after = prop(doc, 'GetPathName')
    w('   之后路径 = %s   %s' % (esc(after), 'PASS 未改变' if after == before else '!!! 路径被改了'))
    if os.path.exists(dest):
        w('   产出: %s  (%d bytes)' % (newname, os.path.getsize(dest)))
    else:
        w('   产出: 缺失 !!')
    w('')

w('== D:\\exo_xml 现状 ==')
try:
    for f in sorted(os.listdir(DEST)):
        w('   %-70s %d' % (esc(f), os.path.getsize(os.path.join(DEST, f))))
except Exception as e:
    w('listdir err %r' % (e,))

open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
