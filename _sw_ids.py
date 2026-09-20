# -*- coding: utf-8 -*-
"""
Test late-binding reachability (GetIDsOfNames) of the SW members we need for the
material experiment. READ-ONLY.
"""
import io
import win32com.client as wc

LOG = r'C:\Users\29408\exo_work\_sw_ids.txt'
out = []
def L(s=''):
    out.append(str(s))

sw = wc.GetActiveObject('SldWorks.Application')
docs = sw.GetDocuments
d = docs[0]
oi = d._oleobj_

CAND = [
    # material (set / get)
    'MaterialIdName', 'MaterialUserName', 'SetMaterialPropertyName', 'SetMaterialPropertyName2',
    'GetMaterialPropertyName', 'GetMaterialPropertyName2', 'RemoveMaterialProperty',
    # mass properties on IModelDoc2 / extension
    'Extension', 'GetMassProperties', 'GetMassProperties2', 'IGetMassProperties2',
    'CreateMassProperty', 'CreateMassProperty2', 'GetMassProperty', 'UpdateMassProperties',
    'GetBomMassProperties', 'Density', 'GetDensity',
    # misc sanity
    'GetTitle', 'GetPathName', 'GetType', 'GetConfigurationByName', 'GetActiveConfiguration',
]

L('=== GetIDsOfNames reachability (part doc) ===')
for nm in CAND:
    try:
        ids = oi.GetIDsOfNames(nm)
        L('  OK    %-28s -> dispid %s' % (nm, ids))
    except Exception as e:
        L('  MISS  %-28s -> %s' % (nm, e))

L('')
L('=== same on SldWorks app ===')
aoi = sw._oleobj_
for nm in ['GetDocuments', 'RevisionNumber', 'ActiveDoc', 'IActiveDoc2', 'GetUserPreferenceIntegerValue']:
    try:
        ids = aoi.GetIDsOfNames(nm)
        L('  OK    %-28s -> dispid %s' % (nm, ids))
    except Exception as e:
        L('  MISS  %-28s -> %s' % (nm, e))

io.open(LOG, 'w', encoding='utf-8').write('\n'.join(out))
print('WROTE', LOG)
