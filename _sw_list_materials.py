# -*- coding: utf-8 -*-
"""List materials + densities from the SW material databases. READ-ONLY."""
import io, os, re
import xml.etree.ElementTree as ET

DBS = [
    r'c:\program files\solidworks corp\solidworks\lang\chinese-simplified\sldmaterials\solidworks materials.sldmat',
    r'c:\programdata\solidworks\solidworks 2024\自定义材料\自定义材料.sldmat',
]
LOG = r'C:\Users\29408\exo_work\_sw_materials.txt'
out = []
def L(s=''):
    out.append(str(s))

for db in DBS:
    L('=' * 70)
    L('DB: %s' % db)
    if not os.path.exists(db):
        L('  (missing)'); continue
    L('  size = %d bytes' % os.path.getsize(db))
    try:
        root = ET.parse(db).getroot()
    except Exception as e:
        L('  parse ERR %r' % e); continue
    mats = []
    for m in root.iter('material'):
        nm = m.get('name')
        dens = None
        for d in m.iter('density'):
            v = d.get('value')
            if v:
                dens = float(v); break
        if nm:
            mats.append((nm, dens))
    L('  materials = %d' % len(mats))
    have = [x for x in mats if x[1]]
    have.sort(key=lambda t: -t[1])
    L('  --- top 25 by density (name, kg/m3) ---')
    for nm, d in have[:25]:
        L('    %-44s %10.1f' % (nm, d))
    L('  --- a few common ones ---')
    for key in ['钢', '铝', '尼', '黄铜', '铜', 'ABS', '碳']:
        for nm, d in have:
            if key in nm:
                L('    %-44s %10.1f' % (nm, d))
                break

io.open(LOG, 'w', encoding='utf-8').write('\n'.join(out))
print('WROTE', LOG)
