# -*- coding: utf-8 -*-
"""SW 内核外部基准表的**唯一正规生成器**（读修好的 XML，输出量纲写进字段名）。

背景坑（已踩过，别再踩）：
  * XML 里 <Mass> 单位 kg、<CenterOfMass> 单位 **m**、<Inertia> 单位 **kg*m^2**。
    三个量纲不统一，早先版本把 m 当 mm 直接存成 com_mm，导致 CoM 差 1000 倍。
  * SW 侧未赋材质 -> 默认密度 1000 kg/m^3，所以
      mass_kg / volume_mm3  **不能**当作真实质量基准；
      com_mm / inertia_*    **可以**当作纯几何基准（与密度无关）。

字段
  mass_kg        kg
  volume_mm3     mm^3      = mass_kg * 1e6      （仅当密度 = 1000 kg/m^3 时成立）
  com_mm         mm        = XML 的 m * 1000
  inertia_kgm2   kg*m^2    XML 原样
  inertia_kgmm2  kg*mm^2   = kg*m^2 * 1e6       （Simscape File Solid 就是按这个量纲喂的）
  inertia_mm5    mm^5      = kg*m^2 * 1e12      （密度 = 1 g/cm^3 的 ∫r^2 dV，
                               与 exo2609/geometry.py 的 inertia_local 同约定）
"""
import os, re, json, csv
import xml.etree.ElementTree as ET

XML  = r'D:\exo_xml\腿部设计_左.fixed.xml'
NS   = '{urn:mathworks:SimscapeMultibody:import}'
JP   = r'C:\Users\29408\exo_work\_sw_baseline_legL.json'
CP   = r'C:\Users\29408\exo_work\matlab2609\simscape\sw_baseline_legL.csv'
OUT  = r'C:\Users\29408\exo_work\_sw_baseline.txt'

def esc(s):
    return ''.join(ch if 32 <= ord(ch) < 127 else ('<%02X>' % ord(ch) if ord(ch) < 32 else ch)
                   for ch in str(s))

root = ET.parse(XML).getroot()
rows = []
for p in root.iter(NS + 'Part'):
    nm  = p.get('name')
    mp  = p.find(NS + 'MassProperties')
    gf  = p.find(NS + 'GeometryFile')
    m = None; cx = cy = cz = None; Iraw = [None] * 9
    if mp is not None:
        e = mp.find(NS + 'Mass')
        if e is not None and e.text:
            m = float(e.text)
        e = mp.find(NS + 'CenterOfMass')
        if e is not None and e.text:
            v = [float(x) for x in e.text.split()]
            if len(v) >= 3:
                cx, cy, cz = [x * 1000.0 for x in v[:3]]     # m -> mm
        e = mp.find(NS + 'Inertia')
        if e is not None and e.text:
            v = [float(x) for x in e.text.split()]
            Iraw = (v + [None] * 9)[:9]
    vol   = (m * 1e6) if m is not None else None
    Ikgmm = [x * 1e6  if x is not None else None for x in Iraw]
    Imm5  = [x * 1e12 if x is not None else None for x in Iraw]
    rows.append({
        'part': nm, 'mass_kg': m, 'volume_mm3': vol,
        'density_g_cm3': (m * 1e6 / vol) if (m and vol) else None,
        'com_mm': [cx, cy, cz],
        'inertia_kgm2': Iraw, 'inertia_kgmm2': Ikgmm, 'inertia_mm5': Imm5,
        'step': gf.get('name') if gf is not None else None,
    })

lines = []
def w(s=''):
    lines.append(str(s))

w('source = %s' % XML)
w('parts  = %d' % len(rows))
w('')
w('%-32s %13s %13s %11s' % ('Part', 'Mass_kg', 'Vol_mm3', 'rho_g/cm3'))
for r in rows:
    w('%-32s %13.9f %13.4f %11.4f'
      % (esc(r['part']), r['mass_kg'] or -1, r['volume_mm3'] or -1, r['density_g_cm3'] or -1))
w('Σ mass = %.9f kg' % sum(r['mass_kg'] for r in rows if r['mass_kg']))

json.dump({'source': XML,
           'caveat': 'SW 未赋材质 -> 默认密度 1000 kg/m^3；'
                     'mass_kg/volume_mm3 不是真实质量基准，com_mm/inertia_* 是纯几何基准',
           'units': {'mass_kg': 'kg', 'volume_mm3': 'mm^3', 'com_mm': 'mm',
                     'inertia_kgm2': 'kg*m^2', 'inertia_kgmm2': 'kg*mm^2',
                     'inertia_mm5': 'mm^5'},
           'parts': rows},
          open(JP, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)

os.makedirs(os.path.dirname(CP), exist_ok=True)
with open(CP, 'w', encoding='utf-8-sig', newline='') as f:
    wr = csv.writer(f)
    wr.writerow(['part', 'mass_kg', 'volume_mm3', 'density_g_cm3',
                 'com_x_mm', 'com_y_mm', 'com_z_mm',
                 'Ixx_kgmm2', 'Iyy_kgmm2', 'Izz_kgmm2',
                 'Ixy_kgmm2', 'Ixz_kgmm2', 'Iyz_kgmm2',
                 'Ixx_mm5', 'Iyy_mm5', 'Izz_mm5', 'step'])
    for r in rows:
        wr.writerow([r['part'], r['mass_kg'], r['volume_mm3'], r['density_g_cm3']] +
                    (r['com_mm'] or ['', '', '']) +
                    (r['inertia_kgmm2'] or [''] * 9)[:6] +
                    (r['inertia_mm5'] or [''] * 3)[:3] +
                    [r['step']])

w(''); w('JSON -> %s' % JP); w('CSV  -> %s' % CP)
open(OUT, 'w', encoding='utf-8').write('\n'.join(lines))
print('WROTE ' + OUT)
