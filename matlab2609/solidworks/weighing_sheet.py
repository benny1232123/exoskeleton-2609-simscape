# -*- coding: utf-8 -*-
"""
weighing_sheet.py —— 可回填称重工作表（项目内可复用工具）

背景（一句话）
--------------
质量 = 体积 × 密度。**体积已被 SolidWorks 内核独立核对过**（19/19 几何配对，
总体积 rel diff -0.055%），所以密度是**唯一**未知量。
密度表是「关键字查表 + 兜底」，左腿 19 件里 8 件落兜底 —— 而且**即使命中了关键字，
那也只是猜**（兜底件里最大的 `腿部_腿杆_片状V5` 占整腿 27.9%，若其实是钢，整腿 +53%）。
2026-09-20 已修掉一处确定错误：`电机_轴` 由兜底铝 2700 改判钢 7850（与同族 `电机_出轴` 一致）。
⇒ 唯一能把这件事从「猜测」变成「证据」的动作是**称重**：
     真实密度 = 单件实测质量 ÷ 已知体积

本工具做什么
------------
1. 按质量降序列出 19 个**唯一零件**，带**实例多重度**（来自 SW 装配体 XML 的
   35 个 `<Instance>`）—— 因为同规格件称一个就等于称了 n 个。
2. 给出空白测量列 + **每件的秤精度要求**（该用 0.001 g 还是 0.01 g 的秤）。
3. 给**两种口径**的兜底占比（唯一零件 / 实例加权），并说明哪个对得上文档值。
4. 给**敏感性**：某件改判材料后整腿质量怎么变 —— 这才是决定"先称哪个"的依据。
5. **可回填**：把实测质量(g)填进生成的 `WEIGHING_SHEET.csv` 的 `measured_g` 列，
   再跑一次本脚本，它会反算真实密度、给出修正后整腿质量、并报**残余不确定度**。

用法
----
    python weighing_sheet.py                # 生成/刷新工作表
    # 填 WEIGHING_SHEET.csv 的 measured_g 列 -> 再跑一次 -> 看修正结果

⚠ 只读原始数据、只写自己的输出文件；**不改任何密度表**。

单位约定：**全程克(g)**（电子秤读什么就是什么）。体积 mm³，密度 kg/m³。
          m[g] = vol_mm3 * rho / 1e6      反算密度[g/cm³] = m[g] / (vol_mm3/1000)
"""

import csv, io, os, re, sys, collections

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, '..', '..'))
SIMD = os.path.join(PROJ, 'matlab2609', 'simscape')

SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(SIMD, 'sw_baseline_legL.csv')
XML = sys.argv[2] if len(sys.argv) > 2 else r'D:\exo_xml\腿部设计_左.fixed.xml'
OUTDIR = sys.argv[3] if len(sys.argv) > 3 else HERE

CSV_OUT = os.path.join(OUTDIR, 'WEIGHING_SHEET.csv')
TXT_OUT = os.path.join(OUTDIR, 'WEIGHING_SHEET.txt')
# ★ 文档值（实例加权口径）= exo_real.urdf 里 leg_L 的质量，真源是 plant_params.csv。
#   以前硬编码 0.304833：密度表一改（本次 2700->7850 修正了 `电机_轴`），
#   本工作表就会报出「偏差 +2.6% ⚠ 查多重度/分组」这种**指错方向**的警告。
DOC_MASS_KG = 0.312589          # 兜底（改后基准）
try:
    with open(os.path.join(SIMD, 'plant_params.csv'), encoding='utf-8', newline='') as _f:
        DOC_MASS_KG = float(list(csv.DictReader(_f))[0]['m_kg'])
except Exception:
    pass

# 用于敏感性的候选材料（不写回密度表，只用来算"如果它其实是X"）
ALTS = [('铝 2700', 2700.0), ('钢 7850', 7850.0), ('尼龙 1150', 1150.0),
        ('聚合物 1200', 1200.0), ('黄铜 8500', 8500.0)]

# --- 密度表：优先用项目真源，取不到则内联同序副本 ---
try:
    sys.path.insert(0, PROJ)
    from exo2609.geometry import density_of as _dof, FALLBACK_DENSITY, FALLBACK_LABEL
    rho_of = _dof
    FB_DENS, FB_LABEL = FALLBACK_DENSITY, FALLBACK_LABEL
    SRC_OF_TABLE = 'exo2609.geometry.DEFAULT_DENSITIES (imported)'
except Exception:
    DENS = [
        (("黄铜", "铜"), 8500.0, "brass"),
        (("弹簧钢", "钢珠", "轴承"), 7850.0, "bearing steel"),
        (("螺丝", "螺钉", "螺母", "螺柱", "顶丝", "销", "卡簧",
          "垫片", "弹垫", "平垫", "轴套", "出轴"), 7850.0, "steel fastener/shaft"),
        (("绑缚", "魔术贴", "织带"), 1150.0, "nylon/textile"),
        (("麦拉片", "按键", "导光", "泡棉", "胶"), 1200.0, "polymer"),
        (("PCB", "电路板", "元件"), 1500.0, "electronics"),
    ]
    # ★ 标签必须带 'FALLBACK'，否则下面 is_fb() 判不出来
    #   （真源标签是 "aluminium (default)"，本分支只在 import 失败时启用）
    FB_DENS, FB_LABEL = 2700.0, "aluminium (FALLBACK)"

    def rho_of(name):
        up = (name or '').upper()
        for keys, rho, lab in DENS:
            for k in keys:
                if k.upper() in up:
                    return rho, lab
        return FB_DENS, FB_LABEL
    SRC_OF_TABLE = 'inline copy (exo2609.geometry not importable)'


def is_fb(label):
    return label == FB_LABEL or 'FALLBACK' in (label or '').upper()


def g_of(vol_mm3, rho):
    return vol_mm3 * rho / 1e6


def instance_mult(xml_path):
    """从 SW 导出的装配体 XML 数每个零件的实例数（按 <Instance entityUid="件名*...">）。"""
    if not os.path.isfile(xml_path):
        return {}, 'XML 不存在：%s（多重度一律按 1 算）' % xml_path
    s = io.open(xml_path, encoding='utf-8', errors='replace').read()
    c = collections.Counter()
    for t in re.findall(r'<Instance\b[^>]*>', s):
        m = re.search(r'entityUid="([^*]+)\*', t)
        if m:
            c[m.group(1).strip()] += 1
    return dict(c), '%s  (%d 个 <Instance>, %d 种零件)' % (
        os.path.basename(xml_path), sum(c.values()), len(c))


def read_measured(path):
    """回填用：从上一版 CSV 里读回 measured_g 列（空的记为未测）。"""
    got = {}
    if not os.path.isfile(path):
        return got
    with io.open(path, encoding='utf-8-sig', newline='') as f:
        for r in csv.DictReader(f):
            v = (r.get('measured_g') or '').strip()
            if v and v != '-':
                try:
                    got[r['part'].strip()] = float(v)
                except ValueError:
                    pass
    return got


def scale_advice(m1_g):
    if m1_g >= 20.0:
        return '0.01 g 秤'
    if m1_g >= 2.0:
        return '0.001 g 秤'
    if m1_g >= 0.5:
        return '0.001 g 秤 + 多件合并称'
    return '不建议单件称（合并同规格件或称整组）'


def main():
    mult_map, mult_note = instance_mult(XML)
    meas = read_measured(CSV_OUT)

    rows = []
    with io.open(SRC, encoding='utf-8-sig', newline='') as f:
        for r in csv.DictReader(f):
            name = r['part']
            vol = float(r['volume_mm3'])
            rho, lab = rho_of(name)
            mult = mult_map.get(name, 1)
            m1 = g_of(vol, rho)
            rows.append(dict(part=name, vol=vol, rho=rho, lab=lab, fb=is_fb(lab),
                             mult=mult, m1=m1, m_all=m1 * mult))

    tot_1x = sum(x['m1'] for x in rows)            # g，唯一零件口径
    tot_35 = sum(x['m_all'] for x in rows)         # g，实例加权口径
    tot_1x_kg, tot_35_kg = tot_1x / 1000.0, tot_35 / 1000.0
    for x in rows:
        x['share'] = 100.0 * x['m_all'] / tot_35

    order = sorted(rows, key=lambda t: -t['m_all'])
    fb = [x for x in rows if x['fb']]
    fb1 = sum(x['m1'] for x in fb)
    fb35 = sum(x['m_all'] for x in fb)
    fb35_steel = sum(g_of(x['vol'], 7850.0) * x['mult'] for x in fb)

    # 回填
    for x in rows:
        mv = meas.get(x['part'])
        x['measured'] = mv
        if mv is not None and x['vol'] > 0:
            x['implied_rho'] = mv / (x['vol'] * 1e-3)      # g/cm3
            x['rev_all'] = mv * x['mult']
        else:
            x['implied_rho'] = None
            x['rev_all'] = x['m_all']
    n_meas = sum(1 for x in rows if x['measured'] is not None)
    rev_mass = sum(x['rev_all'] for x in rows)
    unmeas_mass = sum(x['m_all'] for x in rows if x['measured'] is None)

    L = []
    def w(s=''):
        L.append(str(s))

    W = 108
    w('=' * W)
    w('可回填称重工作表  —— 只读原始数据，不改密度表   [单位：克 g]')
    w('  体积/质心来源: %s' % SRC)
    w('                 (SW 内核；已被我方 Python 链路 19/19 几何配对复现)')
    w('  实例多重度   : %s' % mult_note)
    w('  密度假设来源 : %s' % SRC_OF_TABLE)
    w('=' * W)
    w('')
    w('%-3s %-30s %3s %10s %6s %9s %9s %7s %7s  %s' %
      ('#', 'part', 'xN', 'vol_mm3', 'rho', 'm1_g', 'mALL_g', 'share%', 'cum%', 'basis'))
    w('-' * W)
    cum = 0.0
    for i, x in enumerate(order, 1):
        cum += x['share']
        w('%-3d %-30s %3d %10.3f %6.0f %9.4f %9.4f %6.2f%% %6.2f%%  %-22s%s' %
          (i, x['part'], x['mult'], x['vol'], x['rho'], x['m1'], x['m_all'],
           x['share'], cum, x['lab'], '  <== FALLBACK' if x['fb'] else ''))
    w('-' * W)
    w('%-3s %-30s %3d %10s %6s %9.4f %9.4f %6.2f%%' %
      ('', 'TOTAL', sum(x['mult'] for x in rows), '', '', tot_1x, tot_35, 100.0))
    w('')
    w('口径（务必分清，两个数都对，但含义不同）:')
    w('  A) 唯一零件口径   19 件各算一次      = %9.4f g = %.6f kg' % (tot_1x, tot_1x_kg))
    w('  B) 实例加权口径   35 件（含重复件）   = %9.4f g = %.6f kg   <== 与文档值同口径'
      % (tot_35, tot_35_kg))
    dev = 100.0 * (tot_35_kg - DOC_MASS_KG) / DOC_MASS_KG
    w('     文档值 = %.6f kg  ->  偏差 %+.4f%%  %s' %
      (DOC_MASS_KG, dev, 'OK' if abs(dev) < 0.5 else '⚠ 查多重度/分组'))
    w('')
    w('★ 兜底命中: %d/%d 件' % (len(fb), len(rows)))
    w('    口径 A（唯一零件）: %8.4f / %9.4f g = %.2f%%   <== 与 MATERIAL_AND_MASS_RECON.md 一致'
      % (fb1, tot_1x, 100.0 * fb1 / tot_1x))
    w('    口径 B（实例加权）: %8.4f / %9.4f g = %.2f%%' % (fb35, tot_35, 100.0 * fb35 / tot_35))
    w('    极端上界（%d 件兜底全按钢 7850）: %.4f g = %.6f kg  (%+.2f%%)'
      % (len(fb), tot_35 - fb35 + fb35_steel, (tot_35 - fb35 + fb35_steel) / 1000.0,
         100.0 * (fb35_steel - fb35) / tot_35))
    w('')

    # ---------------- 敏感性：决定"先称哪个" ----------------
    w('=' * W)
    w('敏感性：某个零件改判材料后，整腿质量会变多少（这是"先称哪个"的真正依据）')
    w('=' * W)
    hdr = '%-30s %9s' % ('part', 'mALL_g')
    for nm, _ in ALTS:
        hdr += ' %11s' % nm
    w(hdr)
    w('-' * W)
    for x in order[:8]:
        line = '%-30s %9.4f' % (x['part'], x['m_all'])
        for nm, rho in ALTS:
            m2 = g_of(x['vol'], rho) * x['mult']
            d = 100.0 * (m2 - x['m_all']) / tot_35
            line += ' %+6.3f/%+5.1f%%' % (m2, d)
        w(line)
    w('-' * W)
    w('读法：格子 = 「若该件其实是这种材料」的 mALL_g / 对整腿的百分比影响。')
    w('      ★ 最刺眼的一格是 `电机_出轴`（当前按钢 7850 算）：若它其实是铝 2700，')
    w('        整腿直接掉 ~21% —— 比 8 件兜底加起来还大的单一风险。而兜底件里最大的')
    w('        `腿部_腿杆_片状V5`（占 27.9%）若其实是钢，整腿 +53% —— 两者都靠名字猜不准，')
    w('        所以必须靠称重定。')
    w('')

    # ---------------- 称重优先级 ----------------
    w('=' * W)
    w('称重优先级（按占质量降序；该称几步、每步需要什么秤）')
    w('=' * W)
    w('%-3s %-30s %3s %9s %8s %10s %-12s' %
      ('#', 'part', 'xN', 'm1_g', 'share%', '±5%需[g]', '秤 / 做法'))
    w('-' * W)
    cum = 0.0
    for i, x in enumerate(order, 1):
        cum += x['share']
        if cum > 99.6:
            break
        w('%-3d %-30s %3d %9.4f %7.2f%% %10.4f  %s%s' %
          (i, x['part'], x['mult'], x['m1'], x['share'], x['m1'] * 0.05,
           scale_advice(x['m1']), '  [兜底]' if x['fb'] else ''))
    w('%-3s %-30s %3s %9s %7.2f%%' % ('', '累计到此', '', '', cum))
    w('')
    w('读数提示：')
    w('  · 前 2 件（电机_出轴 + 腿部_腿杆_片状V5）= %.2f%% 的整腿质量 —— **称两次定掉六成**。'
      % (order[0]['share'] + order[1]['share']))
    w('  · 同一规格有 n 个实例时，**把 n 个一起称再除 n** 更准（抵消单件读数误差）：')
    for x in order:
        if x['mult'] > 1 and x['m1'] >= 0.2:
            w('      %-30s x%d  合并共 %7.4f g' % (x['part'], x['mult'], x['m1'] * x['mult']))
    w('  · 拆不下来的（绑缚已缝在腿上）→ 用「带/不带」差值；或向设计者要材料牌号。')
    w('  · 密度反算：rho[g/cm3] = m[g] / (vol_mm3 / 1000)')
    w('')

    # ---------------- 回填结果 ----------------
    w('=' * W)
    w('回填结果（填 WEIGHING_SHEET.csv 的 measured_g 列，再跑一次本脚本）')
    w('=' * W)
    if n_meas == 0:
        w('  目前一件都没填 —— 空白模板如下（把秤上读数写到 measured_g 列）：')
        w('')
        w('  %-30s %3s %10s %9s %11s  %s' %
          ('part', 'xN', 'vol_mm3', 'rho假设', 'mALL当前[g]', 'measured_g'))
        w('  ' + '-' * 92)
        for x in order:
            w('  %-30s %3d %10.3f %9.0f %11.4f  %s' %
              (x['part'], x['mult'], x['vol'], x['rho'], x['m_all'], '__________'))
    else:
        w('  %-30s %3s %10s %11s %11s %12s' %
          ('part', 'xN', 'vol_mm3', 'measured[g]', 'rho反算', 'mALL修正[g]'))
        w('  ' + '-' * 96)
        for x in order:
            if x['measured'] is None:
                continue
            flag = ''
            # ⚠ 单位陷阱：implied_rho 是 **g/cm³**，x['rho'] 是 **kg/m³**，差 1000 倍。
            #   直接相减会得到恒定的 ~-100%（踩过），必须先换算到同一单位。
            rho_meas = x['implied_rho'] * 1000.0                    # g/cm3 -> kg/m3
            if abs(rho_meas - x['rho']) / x['rho'] > 0.05:
                flag = '   <== 实测 %.0f kg/m3(%.3f g/cm3)，与假设 %+.1f%%' % (
                    rho_meas, x['implied_rho'], 100.0 * (rho_meas - x['rho']) / x['rho'])
            w('  %-30s %3d %10.3f %11.4f %11.3f %12.4f%s' %
              (x['part'], x['mult'], x['vol'], x['measured'], x['implied_rho'],
               x['rev_all'], flag))
        w('')
        w('  已测零件        = %d / %d' % (n_meas, len(rows)))
        w('  整腿质量（实例加权口径）:')
        w('      当前（全假设）  = %9.4f g = %.6f kg' % (tot_35, tot_35_kg))
        w('      修正后          = %9.4f g = %.6f kg' % (rev_mass, rev_mass / 1000.0))
        w('      文档值参考      = %9.4f g = %.6f kg' % (DOC_MASS_KG * 1000, DOC_MASS_KG))
        w('  残余不确定度    = %.2f%% 的整腿质量仍靠假设（%d 件未测，共 %.4f g）'
          % (100.0 * unmeas_mass / tot_35, len(rows) - n_meas, unmeas_mass))
    w('')
    w('⚠ 称重得到的密度**只对这一个零件类型**负责。写回密度表前先确认不是"单件特例"')
    w('   （同规格多件称 2 个对比；或同一实例组内不同位置的件分别称）。')

    io.open(TXT_OUT, 'w', encoding='utf-8').write('\n'.join(L))

    with io.open(CSV_OUT, 'w', encoding='utf-8-sig', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['part', 'mul', 'vol_mm3', 'rho_assumed', 'density_basis', 'fallback',
                     'mass_1x_g', 'mass_all_g', 'share_pct',
                     'measured_g',              # <<< 只填这一列，填完重跑
                     'implied_rho_g_cm3', 'revised_mass_all_g', 'scale_hint'])
        for x in order:
            wr.writerow([x['part'], x['mult'], '%.4f' % x['vol'], '%.0f' % x['rho'], x['lab'],
                         'Y' if x['fb'] else '', '%.4f' % x['m1'], '%.4f' % x['m_all'],
                         '%.3f' % x['share'],
                         '' if x['measured'] is None else '%.4f' % x['measured'],
                         '' if x['implied_rho'] is None else '%.3f' % x['implied_rho'],
                         '%.4f' % x['rev_all'], scale_advice(x['m1'])])

    print('\n'.join(L))
    print('')
    print('WROTE', TXT_OUT)
    print('WROTE', CSV_OUT)


if __name__ == '__main__':
    main()
