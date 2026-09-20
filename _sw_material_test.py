# -*- coding: utf-8 -*-
"""
Single-part material experiment (SW side).

Question: does SolidWorks' mass property honor an ASSIGNED material's density?
Method : pick one open part, read baseline (should be rho=1000, no material),
         assign a real material, read again, then REVERT and verify.

Safety : only modifies ONE part in memory; never saves; never closes any doc;
         reverts via EditUndo2 / SetMaterialPropertyName2(...,'','') and VERIFIES.
"""
import io
import pythoncom
import win32com.client as wc

LOG = r'C:\Users\29408\exo_work\_sw_material_test.txt'
out = []
def L(s=''):
    out.append(str(s))

sw = wc.GetActiveObject('SldWorks.Application')
docs = sw.GetDocuments
GET = pythoncom.DISPATCH_PROPERTYGET

def invoke(disp, dispid, flags, *args):
    for form in ('nores', 'res0', 'res1'):
        try:
            if form == 'nores':
                return disp.Invoke(dispid, 0, flags, *args)
            if form == 'res0':
                return disp.Invoke(dispid, 0, flags, 0, *args)
            return disp.Invoke(dispid, 0, flags, 1, *args)
        except Exception:
            continue
    raise RuntimeError('invoke failed dispid=%s args=%s' % (dispid, args))

def did(o, name):
    r = o.GetIDsOfNames(name)
    return r[0] if isinstance(r, tuple) else r

def read_mp(doc):
    ext = invoke(doc._oleobj_, 66306, GET)
    eo = ext if hasattr(ext, 'GetIDsOfNames') else ext._oleobj_
    mp = invoke(eo, did(eo, 'CreateMassProperty2'), GET)
    return (invoke(mp, did(mp, 'Mass'), GET),
            invoke(mp, did(mp, 'Volume'), GET),
            invoke(mp, did(mp, 'Density'), GET))

TARGET = '电机_轴'
doc = None
for i in range(len(docs)):
    if docs[i].GetTitle == TARGET:
        doc = docs[i]; break
if doc is None:
    L('TARGET %s NOT FOUND' % TARGET)
else:
    L('target = %s' % doc.GetTitle)
    L('path   = %s' % doc.GetPathName)

    m0, v0, r0 = read_mp(doc)
    L('')
    L('--- BASELINE ---')
    L('  MaterialIdName = %r' % doc.MaterialIdName)
    L('  MaterialUserName = %r' % doc.MaterialUserName)
    L('  Mass   = %.15g kg' % m0)
    L('  Volume = %.15g m3   (= %.4f mm3)' % (v0, v0 * 1e9))
    L('  Density= %.6f kg/m3' % r0)

    CAND = [
        ('SolidWorks Materials', '1023 碳钢板 (SS)'),
        ('SolidWorks 材料',       '1023 碳钢板 (SS)'),
        ('solidworks materials', '1023 碳钢板 (SS)'),
        ('SOLIDWORKS Materials', '1023 碳钢板 (SS)'),
        ('SolidWorks Materials', 'Plain Carbon Steel'),
    ]
    L('')
    L('--- ASSIGN MATERIAL ---')
    applied = None
    for db, nm in CAND:
        try:
            ok = doc.SetMaterialPropertyName2('默认', db, nm)
            cur = doc.MaterialIdName
            L('  db=%-22s name=%-24s ret=%r  MaterialIdName=%r' % (db, nm, ok, cur))
            if cur:
                applied = (db, nm); break
        except Exception as e:
            L('  db=%-22s name=%-24s ERR %r' % (db, nm, e))

    if applied:
        try:
            doc.EditRebuild3()
        except Exception as e:
            L('  EditRebuild3 ERR %r' % e)
        m1, v1, r1 = read_mp(doc)
        L('')
        L('--- AFTER ASSIGN ---  (db=%s name=%s)' % applied)
        L('  MaterialIdName = %r' % doc.MaterialIdName)
        L('  Mass   = %.15g kg' % m1)
        L('  Volume = %.15g m3   (= %.4f mm3)' % (v1, v1 * 1e9))
        L('  Density= %.6f kg/m3' % r1)
        L('  mass ratio  = %.6f' % (m1 / m0 if m0 else 0))
        L('  volume ratio= %.6f   (should be ~1.0 -> pure density effect)' % (v1 / v0 if v0 else 0))
        L('  => SW honours assigned material density? %s'
          % ('YES' if abs((m1 / m0 if m0 else 0) - r1 / r0) < 1e-6 else 'CHECK'))

        # ---------- REVERT ----------
        L('')
        L('--- REVERT ---')
        reverted = False
        try:
            uo = doc._oleobj_
            u = invoke(uo, did(uo, 'EditUndo2'), pythoncom.DISPATCH_METHOD, 1)
            L('  EditUndo2(1) -> %r' % u)
        except Exception as e:
            L('  EditUndo2 ERR %r' % e)
        if doc.MaterialIdName:
            try:
                doc.SetMaterialPropertyName2('默认', '', '')
                L('  SetMaterialPropertyName2(blank) -> MaterialIdName=%r' % doc.MaterialIdName)
            except Exception as e:
                L('  blank-set ERR %r' % e)
        m2, v2, r2 = read_mp(doc)
        L('  MaterialIdName = %r' % doc.MaterialIdName)
        L('  Mass   = %.15g kg   (baseline %.15g)' % (m2, m0))
        L('  Density= %.6f kg/m3' % r2)
        reverted = (not doc.MaterialIdName) and abs(m2 - m0) < 1e-15
        L('  REVERT_OK = %s' % reverted)
    else:
        L('no candidate material could be assigned')

io.open(LOG, 'w', encoding='utf-8').write('\n'.join(out))
print('WROTE', LOG)
