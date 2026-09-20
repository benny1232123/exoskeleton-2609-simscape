# -*- coding: utf-8 -*-
"""SolidWorks XML（独立上游） vs 我方 Python STEP 解析 —— 逐件几何对账。

★ 核心思路：**不需要知道材料也能对账几何。**
   SW 侧默认密度精确是 1000 kg/m³（实测 7 位有效数字），于是在这个已知比值下：
        SW 体积   [mm³] = mass[kg] × 1e6
        SW ∫r²dV  [mm⁵] = I[kg·m²] × 1e12
   而我方 exo2609/geometry.py 输出的 inertia 本身就是 **mm⁵、密度=1** 的几何量
   （见该文件 SolidProps.inertia_local 的注释）。
   → 两边都能还原成 **与材料无关的几何量**，可以直接逐件比。

   这正是路径 A 的真正价值：几何来自 CAD 内核，跟我们自己的 STEP 解析完全独立。
   材料（绝对质量/惯量）是**另一个问题**，不阻塞这层验证。

⚠ 若以后在 SolidWorks 里给零件赋了真材料，把 RHO_SW 改成实际密度即可
   （或改成每件各自的密度表）。

用法：python _xml_vs_py.py [xml] [我方json] [out.txt]
"""
import io
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

XML  = sys.argv[1] if len(sys.argv) > 1 else r'D:\exo_xml\腿部设计_左.xml'
PYJ  = sys.argv[2] if len(sys.argv) > 2 else r'C:\Users\29408\Desktop\外骨骼\_mass_leg_L.json'
OUT  = sys.argv[3] if len(sys.argv) > 3 else r'C:\Users\29408\exo_work\_xml_vs_py.txt'

RHO_SW = 1000.0        # kg/m³ —— SolidWorks 未赋材质时的默认密度（实测值）

# ★ 密度表不再内联 —— 真源 = exo2609/geometry.py（2026-09-20）。
#   原先这里抄了一份 DENS，还手改了标签（"铝(兜底)"），与真源 "aluminium (default)"
#   不一致；且缺 electronics 扩展键。内联副本 = 静默漂移，故统一到唯一入口。
#   注意：真源比这里多一层 PART_EXACT（精确名优先），单靠 DENS 是拿不到的。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from exo2609.geometry import density_of as _density_of            # noqa: E402


def rho_of(name):
    return _density_of(name)


def local(t):
    return t.rsplit('}', 1)[-1] if '}' in t else t


def A(e):
    return {local(k): v for k, v in e.attrib.items()}


L = []
def p(s=''):
    L.append(str(s))


# ============================================================ 读 SW XML
raw = open(XML, 'rb').read()
san = bytearray(raw)
for i in range(len(san)):
    if san[i] < 0x20 and san[i] not in (0x09, 0x0A, 0x0D):
        san[i] = 0x3F
root = ET.fromstring(bytes(san).decode('utf-8', 'replace'))

sw = {}
for part in root.iter():
    if local(part.tag) != 'Part':
        continue
    nm = A(part).get('name', '')
    mass = com = inr = None
    for g in part:
        if local(g.tag) != 'MassProperties':
            continue
        for h in g:
            lh, ha = local(h.tag), A(h)
            if lh == 'Mass':
                v = ha.get('value', ha.get('mass', (h.text or '').strip()))
                try:
                    mass = float(v)
                except (TypeError, ValueError):
                    pass
            elif lh == 'CenterOfMass':
                com = [float(x) for x in (h.text or '').split()]
            elif lh == 'Inertia':
                inr = [float(x) for x in (h.text or '').split()]
    sw[nm] = {'mass': mass, 'com_m': com, 'inertia': inr}

# ============================================================ 读我方 JSON
py = {}
with io.open(PYJ, 'r', encoding='utf-8') as f:
    data = json.load(f)
# 兼容两种形态：顶层 dict（{"step":..,"nvol":..,"volumes":[...]}）或直接就是 list
arr = data['volumes'] if isinstance(data, dict) else data
for d in arr:
    nm = d.get('name', '').split('/')[-1]
    py[nm] = d

# ============================================================ 对账
p('# 路径 A 逐件几何对账（SW CAD 内核 vs 我方 STEP 解析）')
p('')
p('SW 侧源   : %s' % XML)
p('我方侧源  : %s' % PYJ)
p('SW 密度假定: %.1f kg/m³（未赋材质的默认值；实测质量÷体积 = 1.000000 g/cm³）' % RHO_SW)
p('换算       : SW体积[mm³] = mass[kg]×1e6 ； SW∫r²dV[mm⁵] = I[kg·m²]×1e12')
p('我方口径   : volume 已是 mm³；inertia_cog 已是 mm⁵、密度=1')
p('')

common = [n for n in sw if n in py]
only_sw  = [n for n in sw if n not in py]
only_py  = [n for n in py if n not in sw]

p('SW 件数 = %d   我方件数 = %d   能配上 = %d' % (len(sw), len(py), len(common)))
if only_sw:
    p('  仅在 SW 里：%s' % '、'.join(only_sw))
if only_py:
    p('  仅在我方  ：%s' % '、'.join(only_py))
p('')

hdr = ('%-30s %12s %12s %9s %12s %9s' %
       ('零件', 'SW体积mm³', '我方体积', '体积rel', 'SW∫r²dV', '惯量rel'))
p(hdr)
p('-' * 96)

sv_tot = pv_tot = 0.0
worst_v = (0, '')
worst_i = (0, '')
rows = []
for n in sorted(common):
    m = sw[n]['mass']
    v_sw = m * 1e6 if m else float('nan')          # ρ=1000 → mm³
    v_py = py[n]['volume']
    I_sw = sw[n]['inertia']
    I_sw_zz = I_sw[2] * 1e12 if I_sw else float('nan')   # 取某个对角分量比
    I_py = py[n]['inertia_cog']
    I_py_zz = I_py[8]                               # 3x3 row-major 的 [2][2]
    rv = abs(v_sw - v_py) / v_py if v_py else float('nan')
    ri = abs(I_sw_zz - I_py_zz) / I_py_zz if I_py_zz else float('nan')
    if v_py:
        sv_tot += v_sw
        pv_tot += v_py
    if rv == rv and rv > worst_v[0]:
        worst_v = (rv, n)
    if ri == ri and ri > worst_i[0]:
        worst_i = (ri, n)
    rows.append((n, v_sw, v_py, rv, I_sw_zz, I_py_zz, ri))
    p('%-30s %12.4f %12.4f %8.1e %12.2f %8.1e'
      % (n[:30], v_sw, v_py, 100 * rv, I_sw_zz, 100 * ri))

p('-' * 96)
tot_rel = abs(sv_tot - pv_tot) / pv_tot if pv_tot else float('nan')
p('%-30s %12.4f %12.4f %8.1e' % ('**合计**', sv_tot, pv_tot, 100 * tot_rel))
p('')
p('  最大体积相对差 = %.3e  (%s)' % (100 * worst_v[0], worst_v[1]))
p('  最大惯量相对差 = %.3e  (%s)' % (100 * worst_i[0], worst_i[1]))

# ---------------------------------------------------------------- CoM 对比
p('')
p('## 质心对比（SW 的 CoM 是米，已 ×1000 转 mm；都在零件自身坐标系）')
p('%-30s %-34s %-34s %9s' % ('零件', 'SW CoM mm', '我方 cog mm', 'rel'))
p('-' * 110)
maxc = (0, '')
for n in sorted(common):
    c_sw = sw[n]['com_m']
    c_py = py[n]['cog']
    if not c_sw or not c_py:
        continue
    c_sw_mm = [x * 1000.0 for x in c_sw]
    nr = max(abs(c_sw_mm[i] - c_py[i]) for i in range(3))
    sc = max(abs(c_py[i]) for i in range(3)) or 1.0
    rel = nr / sc
    if rel > maxc[0]:
        maxc = (rel, n)
    p('%-30s %-34s %-34s %8.1e'
      % (n[:30],
         '[%9.4f %9.4f %9.4f]' % tuple(c_sw_mm),
         '[%9.4f %9.4f %9.4f]' % tuple(c_py),
         100 * rel))
p('')
p('  最大质心相对差 = %.3e  (%s)' % (100 * maxc[0], maxc[1]))

# ---------------------------------------------------------------- 材料建议
p('')
p('## 材料建议（按零件名推断，仅供在 SolidWorks 里挑材料时参考）')
p('%-32s %10s %-16s %12s' % ('零件', 'ρ kg/m³', '材料猜测', '按此ρ的质量 g'))
p('-' * 78)
for n in sorted(common):
    rho, lab = rho_of(n)
    v = py[n]['volume']
    p('%-32s %10.0f %-16s %12.3f' % (n[:32], rho, lab, rho * v / 1e6))

# ---------------------------------------------------------------- 判定
p('')
p('=' * 78)
ok_v = (worst_v[0] < 1e-4)
ok_i = (worst_i[0] < 1e-4)
ok_c = (maxc[0] < 1e-4)
if ok_v and ok_i and ok_c:
    p('VERDICT: GEOM_XCHECK_OK')
    p('  全部 %d 件的体积/惯量/质心与 SolidWorks 内核一致（相对差 < 1e-4）。' % len(common))
    p('  → **这是第一条真正独立于我方解析器的外部几何证据**，且不需要任何材料信息。')
else:
    p('VERDICT: GEOM_XCHECK_MISMATCH')
    if not ok_v:
        p('  体积对不上（最大 %.3e，%s）→ 几何解析或分组有问题' % (100 * worst_v[0], worst_v[1]))
    if not ok_i:
        p('  惯量对不上（最大 %.3e，%s）→ 惯量积分/单位有问题' % (100 * worst_i[0], worst_i[1]))
    if not ok_c:
        p('  质心对不上（最大 %.3e，%s）→ 局部坐标系约定有问题' % (100 * maxc[0], maxc[1]))

open(OUT, 'w', encoding='utf-8').write('\n'.join(L))
print('written -> %s' % OUT)
