# -*- coding: utf-8 -*-
"""
READ-ONLY probe of SolidWorks material / density state.
Goal: find out whether ANY part already carries a real material (non-default density).
Writes UTF-8 log to _sw_mat_probe.txt so Chinese survives.
No CAD data is modified.
"""
import io, sys, traceback
import win32com.client as wc

LOG = r'C:\Users\29408\exo_work\_sw_mat_probe.txt'
lines = []
def L(s):
    lines.append(str(s))

def getv(o, name, *a):
    v = getattr(o, name)
    return v(*a) if callable(v) else v

def safe(fn, tag=''):
    try:
        return fn()
    except Exception as e:
        return 'ERR[%s]:%s' % (tag, type(e).__name__ + ':' + str(e))

sw = None
try:
    sw = wc.GetActiveObject('SldWorks.Application')
    L('SW_REV = %s' % safe(lambda: getv(sw, 'RevisionNumber'), 'rev'))
except Exception as e:
    L('NO_ACTIVE_SW: %r' % e)

if sw is not None:
    docs = []
    try:
        docs = getv(sw, 'GetDocuments')
    except Exception as e:
        L('GetDocuments FAILED: %r' % e)
    try:
        n = len(docs)
    except Exception:
        n = -1
    L('nDocs = %s' % n)

    for i in range(max(0, n)):
        d = docs[i]
        L('')
        L('=== doc[%d] ===' % i)
        title = safe(lambda: getv(d, 'GetTitle'), 'title')
        path = safe(lambda: getv(d, 'GetPathName'), 'path')
        typ = safe(lambda: getv(d, 'GetType'), 'type')
        L('  title = %s' % title)
        L('  path  = %s' % path)
        L('  type  = %s   (1=PART 2=ASSEMBLY 3=DRAWING)' % typ)

        ext = safe(lambda: getv(d, 'Extension'), 'ext')
        L('  ext   = %s' % type(ext))

        # --- material name attempts (out-param tolerant) ---
        for tag, fn in [
            ('doc.GMPN2("","")',      lambda: getv(d, 'GetMaterialPropertyName2', '', '')),
            ('doc.GMPN2("默认")',      lambda: getv(d, 'GetMaterialPropertyName2', '默认')),
            ('ext.GMPN2("","")',      lambda: getv(ext, 'GetMaterialPropertyName2', '', '')),
            ('ext.GMPN2("默认","")',   lambda: getv(ext, 'GetMaterialPropertyName2', '默认', '')),
            ('doc.MaterialIdName',    lambda: getv(d, 'MaterialIdName')),
            ('doc.GetMaterialPropertyName', lambda: getv(d, 'GetMaterialPropertyName', '')),
        ]:
            if not hasattr(ext, '__class__'):
                pass
            L('  %-26s -> %s' % (tag, safe(fn, tag)))

        # --- mass properties (density is the decisive number) ---
        mp = None
        for tag, fn in [
            ('ext.CreateMassProperty2()', lambda: getv(ext, 'CreateMassProperty2')),
            ('ext.CreateMassProperty()',  lambda: getv(ext, 'CreateMassProperty')),
        ]:
            r = safe(fn, tag)
            L('  %-28s -> %s' % (tag, type(r)))
            if not isinstance(r, str) and r is not None:
                mp = r
                break

        if mp is not None:
            for tag, fn in [
                ('mp.Density',      lambda: getv(mp, 'Density')),
                ('mp.Mass',         lambda: getv(mp, 'Mass')),
                ('mp.Volume',       lambda: getv(mp, 'Volume')),
                ('mp.CenterOfMass', lambda: getv(mp, 'CenterOfMass')),
                ('mp.UseSystemUnits', lambda: getv(mp, 'UseSystemUnits')),
            ]:
                L('    %-20s -> %s' % (tag, safe(fn, tag)))
        else:
            L('  (no mass-property object)')

    # count how many are parts
    L('')
    L('=== SUMMARY ===')
    try:
        np_ = sum(1 for i in range(n) if safe(lambda: getv(docs[i], 'GetType'), 't') == 1)
        L('parts = %d' % np_)
    except Exception as e:
        L('summary err %r' % e)

with io.open(LOG, 'w', encoding='utf-8') as f:
    f.write('\n'.join(lines))
print('WROTE', LOG)
