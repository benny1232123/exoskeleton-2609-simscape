# -*- coding: utf-8 -*-
"""
Enumerate the IDispatch member names actually reachable on an SW part document
(late binding), filtering for mass / material / density APIs.
READ-ONLY.
"""
import io
import win32com.client as wc

LOG = r'C:\Users\29408\exo_work\_sw_api_names.txt'
out = []
def L(s=''):
    out.append(str(s))

sw = wc.GetActiveObject('SldWorks.Application')
docs = sw.GetDocuments
d = docs[0]   # a part

def member_names(o, what):
    L('===== %s =====' % what)
    try:
        oi = o._oleobj_
    except Exception as e:
        L('no _oleobj_: %r' % e); return []
    try:
        ti = oi.GetTypeInfo()
    except Exception as e:
        L('GetTypeInfo failed: %r' % e); return []
    try:
        ta = ti.GetTypeAttr()
    except Exception as e:
        L('GetTypeAttr failed: %r' % e); return []
    L('cFuncs=%s cVars=%s' % (getattr(ta, 'cFuncs', '?'), getattr(ta, 'cVars', '?')))
    names = []
    for i in range(getattr(ta, 'cFuncs', 0)):
        try:
            fd = ti.GetFuncDesc(i)
            nm = ti.GetNames(fd.memid)
            names.append(nm[0])
        except Exception as e:
            names.append('<err %d: %r>' % (i, e))
    for i in range(getattr(ta, 'cVars', 0)):
        try:
            vd = ti.GetVarDesc(i)
            nm = ti.GetNames(vd.memid)
            names.append('VAR:' + nm[0])
        except Exception as e:
            names.append('<varerr %d: %r>' % (i, e))
    return names

names = member_names(d, 'part doc')
L('total members = %d' % len(names))
kw = ('mass', 'material', 'densit', 'extension', 'volume', 'property')
hit = [n for n in names if any(k in n.lower() for k in kw)]
L('')
L('--- keyword hits ---')
for n in hit:
    L('  ' + n)

L('')
L('--- all members (sorted) ---')
for n in sorted(set(names)):
    L('  ' + n)

# also the app object
anames = member_names(sw, 'SldWorks app')
L('')
L('app total members = %d' % len(anames))
ahit = [n for n in anames if any(k in n.lower() for k in ('mass', 'material', 'densit'))]
L('app keyword hits: %s' % ahit)

io.open(LOG, 'w', encoding='utf-8').write('\n'.join(out))
print('WROTE', LOG)
