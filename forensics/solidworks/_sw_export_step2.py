# -*- coding: utf-8 -*-
"""从 SW 导出缺失 STEP（修正 by-ref 参数：SaveAs4 的 Errors/Warnings 必须传 VARIANT byref）。"""
import os, sys, traceback
import pythoncom
import win32com.client

OUT = r'C:\Users\29408\exo_work\_sw_export_step2.txt'
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

sw = win32com.client.GetActiveObject('SldWorks.Application')
docs = list(prop(sw, 'GetDocuments') or [])
w('docs = %d' % len(docs))

DEST = r'D:\exo_xml'
TARGETS = {
    '腿部_腿杆_片状V5': '腿部_腿杆_片状V5_Default_sldprt.STEP',
    '腿部_轴盖':        '腿部_轴盖_Default_sldprt.STEP',
}

def byref_i4(v=0):
    return win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, v)

for title, newname in TARGETS.items():
    doc = None
    for d in docs:
        if prop(d, 'GetTitle') == title:
            doc = d
            break
    w('')
    w('=== %s ===' % title)
    if doc is None:
        w('   文档未打开，跳过'); continue
    before = prop(doc, 'GetPathName')
    dest = os.path.join(DEST, newname)
    if os.path.exists(dest):
        w('   目标已存在，跳过: %s' % esc(newname)); continue

    ok = False
    # 候选 1: ModelDoc2.SaveAs4(Name, Version, Options, Errors, Warnings)
    try:
        e, wn = byref_i4(0), byref_i4(0)
        r = doc.SaveAs4(dest, 0, 3, e, wn)
        w('   SaveAs4 -> %r  err=%s warn=%s' % (r, e.value, wn.value))
        ok = bool(r)
    except Exception as ex:
        w('   SaveAs4 异常: %r' % (ex,))
    # 候选 2: ModelDocExtension.SaveAs3(Name, Version, Options, ExportData, Errors, Warnings)
    if not ok:
        try:
            ext = prop(doc, 'Extension')
            e, wn = byref_i4(0), byref_i4(0)
            r = ext.SaveAs3(dest, 0, 3, None, e, wn)
            w('   Ext.SaveAs3 -> %r  err=%s warn=%s' % (r, e.value, wn.value))
            ok = bool(r)
        except Exception as ex:
            w('   Ext.SaveAs3 异常: %r' % (ex,))
    # 候选 3: ModelDocExtension.SaveAs(Name, Version, Options, ExportData, Errors, Warnings)
    if not ok:
        try:
            ext = prop(doc, 'Extension')
            e, wn = byref_i4(0), byref_i4(0)
            r = ext.SaveAs(dest, 0, 3, None, e, wn)
            w('   Ext.SaveAs -> %r  err=%s warn=%s' % (r, e.value, wn.value))
            ok = bool(r)
        except Exception as ex:
            w('   Ext.SaveAs 异常: %r' % (ex,))

    after = prop(doc, 'GetPathName')
    w('   路径 %s   %s' % (esc(after), 'PASS 未改变' if after == before else '!!! 被改了'))
    if os.path.exists(dest):
        w('   产出 OK: %s  (%d bytes)' % (esc(newname), os.path.getsize(dest)))
    else:
        w('   产出: 缺失 !!')

w('')
w('== D:\\exo_xml *.STEP 现状 ==')
for f in sorted(os.listdir(DEST)):
    if f.lower().endswith('.step'):
        w('   %-64s %d' % (esc(f), os.path.getsize(os.path.join(DEST, f))))

open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
