# -*- coding: utf-8 -*-
"""修 XML + 规范 STEP 文件名 + 导出 SW 外部基准表。

插件把中文配置名 `默认` 转 ANSI 时缓冲区溢出，吐出的垃圾字节有两类后果：
  (A) 合法 UTF-8（U+FFFD 等）-> 文件名乱码，XML 仍合法
  (B) 原始控制字符          -> 文件根本没写出来 + XML 非法
本脚本不改 SolidWorks 源文件，只在 D:\\exo_xml 出产物上做归一化：
  1) 19 个 GeometryFile 名统一改成 `<零件名>_Default_sldprt.STEP`
  2) 磁盘上对应 STEP 一并改名（缺的 2 个已由 SW 直接导出）
  3) 输出 腿部设计_左.fixed.xml
  4) 附带 SW 内核基准表（质量/体积/质心/惯量）
"""
import os, re, json, csv, sys
import xml.etree.ElementTree as ET

DIR   = r'D:\exo_xml'
RAW   = os.path.join(DIR, '腿部设计_左.xml')
SAN   = os.path.join(DIR, '腿部设计_左.sanitized.xml')
FIXED = os.path.join(DIR, '腿部设计_左.fixed.xml')
NS    = '{urn:mathworks:SimscapeMultibody:import}'

buf = []
def w(s=''):
    buf.append(str(s))

def esc(s):
    if s is None:
        return '<None>'
    return ''.join(ch if 32 <= ord(ch) < 127 else ('<%02X>' % ord(ch) if ord(ch) < 32 else ch)
                   for ch in s)

w('== A. 解析清洗副本，取文档序 (Part, GeometryFile) ==')
tree = ET.parse(SAN)
root = tree.getroot()
parts_el = list(root.iter(NS + 'Part'))
gf_el    = list(root.iter(NS + 'GeometryFile'))
w('Part = %d , GeometryFile = %d' % (len(parts_el), len(gf_el)))
if len(parts_el) != len(gf_el) or not parts_el:
    w('!!! 数量不等，中止'); open(r'C:\Users\29408\exo_work\_xml_fix.txt','w',encoding='utf-8').write('\n'.join(buf)); sys.exit(1)

pairs = []
bad = 0
for i, (p, g) in enumerate(zip(parts_el, gf_el)):
    pname = p.get('name')
    gname = g.get('name')
    pairs.append((pname, gname))
    if not gname.startswith(pname + '_'):
        bad += 1
        w('   [%2d] !!! 前缀不符: Part=|%s|  Geom=|%s|' % (i, esc(pname), esc(gname)))
w('前缀自检：不符 %d / %d' % (bad, len(pairs)))
if bad:
    w('!!! 顺序假设不成立，中止'); open(r'C:\Users\29408\exo_work\_xml_fix.txt','w',encoding='utf-8').write('\n'.join(buf)); sys.exit(1)

# ---- 规范名 ----
def canon(pname):
    return '%s_Default_sldprt.STEP' % pname

canon_list = [canon(p) for p, _ in pairs]
if len(set(canon_list)) != len(canon_list):
    w('!!! 规范名有重名，中止'); open(r'C:\Users\29408\exo_work\_xml_fix.txt','w',encoding='utf-8').write('\n'.join(buf)); sys.exit(1)

w('')
w('== B. STEP 改名 / 盘点 ==')
step_files = [f for f in os.listdir(DIR) if f.lower().endswith('.step')]
w('磁盘 STEP 数 = %d' % len(step_files))
rename_log = []
for (pname, gname), cname in zip(pairs, canon_list):
    if gname == cname and os.path.exists(os.path.join(DIR, cname)):
        rename_log.append((pname, gname, cname, '已就位'))
        continue
    src = gname if os.path.exists(os.path.join(DIR, gname)) else None
    if src is None:
        cands = [f for f in step_files if f.startswith(pname + '_')]
        if len(cands) == 1:
            src = cands[0]
        elif len(cands) > 1:
            rename_log.append((pname, gname, cname, '多候选!' + str(cands))); continue
    if src is None:
        rename_log.append((pname, gname, cname, '缺失!!')); continue
    if os.path.exists(os.path.join(DIR, cname)):
        rename_log.append((pname, src, cname, '目标已存在')); continue
    os.rename(os.path.join(DIR, src), os.path.join(DIR, cname))
    rename_log.append((pname, src, cname, 'renamed'))

for pname, src, cname, st in rename_log:
    w('   %-30s | %-52s -> %-46s %s' % (esc(pname), esc(src), esc(cname), st))
n_ok = sum(1 for r in rename_log if r[3] in ('renamed', '已就位'))
w('就位 %d / %d' % (n_ok, len(rename_log)))

w('')
w('== C. 重写 XML（按文档序，逐位替换 GeometryFile 的 name 属性）==')
raw = open(RAW, 'rb').read()
pat = re.compile(rb'(<GeometryFile\s+name=")([^"]*)(")')
hits = list(pat.finditer(raw))
w('raw 里 GeometryFile name 命中 = %d（期望 %d）' % (len(hits), len(pairs)))
if len(hits) != len(pairs):
    w('!!! 命中数不符，改写中止（仍写盘做人工核对）')
    new = raw
else:
    out = []
    last = 0
    for m, cname in zip(hits, canon_list):
        out.append(raw[last:m.start()])
        out.append(m.group(1))
        out.append(cname.encode('utf-8'))
        out.append(m.group(3))
        last = m.end()
    out.append(raw[last:])
    new = b''.join(out)
open(FIXED, 'wb').write(new)
w('写出 %s (%d bytes)' % (FIXED, len(new)))

# 合法性复检
try:
    t2 = ET.parse(FIXED)
    w('XML 解析 : OK  根=%s' % t2.getroot().tag)
    xml_ok = True
except Exception as e:
    w('XML 解析 : FAILED %r' % (e,))
    xml_ok = False
ctrl = [b for b in open(FIXED, 'rb').read() if b < 0x20 and b not in (0x09, 0x0A, 0x0D)]
w('XML 禁止控制字符 = %d' % len(ctrl))

refs = [g.get('name') for g in ET.parse(FIXED).getroot().iter(NS + 'GeometryFile')]
miss = [r for r in refs if not os.path.exists(os.path.join(DIR, r))]
w('GeometryFile 引用 %d , 磁盘缺失 %d' % (len(refs), len(miss)))
for m in miss:
    w('   MISS %s' % esc(m))

w('')
w('== D. SW 内核外部基准表 ==')
rows = []
for p in parts_el:
    nm  = p.get('name')
    mass_el = p.find(NS + 'MassProperties')
    m = cx = cy = cz = None
    I = [None]*9
    if mass_el is not None:
        me = mass_el.find(NS + 'Mass')
        if me is not None and me.text:
            m = float(me.text)
        ce = mass_el.find(NS + 'CenterOfMass')
        if ce is not None and ce.text:
            vals = [float(x) for x in ce.text.split()]
            if len(vals) >= 3:
                # ★ 单位坑：XML 里 CoM 是 **米**（与 Mass 的 kg 配套），
                #   本表统一用 mm -> 必须 x1000。漏了这步会把两个量的单位混掉。
                cx, cy, cz = [v * 1000.0 for v in vals[:3]]
        ie = mass_el.find(NS + 'Inertia')
        if ie is not None and ie.text:
            vals = [float(x) for x in ie.text.split()]
            I = (vals + [None]*9)[:9]
    vol = (m * 1e6) if m is not None else None      # mm^3（SW 默认密度 1000 kg/m^3 时成立）
    rows.append({
        'part': nm, 'mass_kg': m, 'volume_mm3': vol,
        'density_g_cm3': (m * 1e6 / vol) if (m and vol) else None,
        'com_mm': [cx, cy, cz], 'inertia_mm5': I,
        'step': canon(nm),
    })

w('   %-32s %12s %14s %14s' % ('Part', 'Mass_kg', 'Vol_mm3', 'rho_g/cm3'))
for r in rows:
    w('   %-32s %12.9f %14.4f %14.4f'
      % (esc(r['part']), r['mass_kg'] or -1, r['volume_mm3'] or -1, r['density_g_cm3'] or -1))
w('   Σ mass = %.9f kg' % sum(r['mass_kg'] for r in rows if r['mass_kg']))

jp = r'C:\Users\29408\exo_work\_sw_baseline_legL.json'
json.dump({'source': RAW, 'note': 'SolidWorks 内核基准；默认密度 1000 kg/m3',
           'parts': rows}, open(jp, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
cp = r'C:\Users\29408\exo_work\matlab2609\simscape\sw_baseline_legL.csv'
os.makedirs(os.path.dirname(cp), exist_ok=True)
with open(cp, 'w', encoding='utf-8-sig', newline='') as f:
    wr = csv.writer(f)
    wr.writerow(['part', 'mass_kg', 'volume_mm3', 'density_g_cm3',
                 'com_x_mm', 'com_y_mm', 'com_z_mm',
                 'Ixx_mm5', 'Iyy_mm5', 'Izz_mm5', 'Ixy_mm5', 'Ixz_mm5', 'Iyz_mm5', 'step'])
    for r in rows:
        wr.writerow([r['part'], r['mass_kg'], r['volume_mm3'], r['density_g_cm3']] +
                    (r['com_mm'] or ['', '', '']) + (r['inertia_mm5'] or ['']*9) + [r['step']])
w('')
w('JSON -> %s' % jp)
w('CSV  -> %s' % cp)
w('')
w('VERDICT: xml_valid=%s  step_refs_missing=%d  parts=%d' % (xml_ok, len(miss), len(rows)))

open(r'C:\Users\29408\exo_work\_xml_fix.txt', 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE')
