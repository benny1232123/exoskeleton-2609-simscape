# -*- coding: utf-8 -*-
"""
mass_budget.py —— 质量预算 / 称重清单生成器（项目内可复用工具）

为什么要它：质量 = 体积 x 密度。**体积已经用 SW 内核独立核对过**，
所以密度是唯一未知量。而密度表是「关键字查表 + 兜底」，一旦关键字没命中
就会静默落到 fallback —— 实测左腿 19 件里 8 件落兜底、占唯一零件口径质量约 52%
（2026-09-20 密度修正后：`电机_轴` 已由兜底铝改判钢 7850）。
本工具把这件事**显式化**：按质量排序、标出兜底命中、给出称重清单。

用法:
    python mass_budget.py [sw_baseline.csv] [out_dir]
默认读 matlab2609/simscape/sw_baseline_legL.csv，输出 .txt/.csv 到同目录。

⚠ 本工具**只读、只报告**，不修改任何密度表。密度表是标定旋钮，
   改之前先看这份报告里的「兜底占比」和「称重清单」。
"""
import csv, io, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, '..', '..'))

SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    PROJ, 'matlab2609', 'simscape', 'sw_baseline_legL.csv')
OUTDIR = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(SRC)

# --- 与 exo2609/geometry.py: DEFAULT_DENSITIES 保持一致（顺序敏感，先命中先算） ---
try:
    sys.path.insert(0, PROJ)
    from exo2609.geometry import density_of as _dof, FALLBACK_DENSITY, FALLBACK_LABEL
    def rho_of(name):
        return _dof(name)
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
    # ★ 标签必须带 'FALLBACK'，否则下面那个 is_fallback() 判不出来
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


def is_fallback(label):
    return label == FB_LABEL or 'FALLBACK' in (label or '').upper() or '兜底' in (label or '')


def main():
    rows = []
    with io.open(SRC, 'r', encoding='utf-8-sig', newline='') as f:
        for r in csv.DictReader(f):
            vol = float(r['volume_mm3'])
            rho, lab = rho_of(r['part'])
            rows.append({'part': r['part'], 'vol': vol, 'rho': rho, 'lab': lab,
                         'fb': is_fallback(lab), 'mass': vol * 1e-9 * rho})
    tot = sum(x['mass'] for x in rows)
    rows.sort(key=lambda x: -x['mass'])
    cum = 0.0
    for x in rows:
        cum += x['mass']
        x['share'] = 100.0 * x['mass'] / tot
        x['cum'] = 100.0 * cum / tot

    L = []
    def w(s=''):
        L.append(str(s))

    w('=' * 94)
    w('质量预算 / 称重清单   —— 只读报告，不改密度表')
    w('  体积来源: %s' % SRC)
    w('  密度来源: %s' % SRC_OF_TABLE)
    w('=' * 94)
    w('%-34s %12s %8s %12s %8s %8s  %s'
      % ('part', 'vol_mm3', 'rho', 'mass_kg', 'share%', 'cum%', 'density basis'))
    w('-' * 94)
    for x in rows:
        w('%-34s %12.4f %8.0f %12.6f %7.2f%% %7.2f%%  %-22s%s'
          % (x['part'], x['vol'], x['rho'], x['mass'], x['share'], x['cum'],
             x['lab'], '  <== FALLBACK' if x['fb'] else ''))
    w('-' * 94)
    w('%-34s %12s %8s %12.6f %7.2f%%' % ('TOTAL(%d)' % len(rows), '', '', tot, 100.0))

    fb = [x for x in rows if x['fb']]
    fbm = sum(x['mass'] for x in fb)
    w('')
    w('★ 兜底命中: %d/%d 件，占质量 %.2f%%' % (len(fb), len(rows), 100.0 * fbm / tot))

    w('')
    w('=== 称重清单（按质量降序，取到累计 ~90%）===')
    c = 0.0
    for i, x in enumerate(rows, 1):
        if c >= 90.0:
            break
        c += x['mass']
        tag = ' [兜底]' if x['fb'] else ''
        w('  %2d. %-32s %6.2f%%  cum=%5.2f%%%s' % (i, x['part'], x['share'], 100.0 * c / tot, tag))
    w('  -> 称重这 %d 件即可锁定整腿质量约 %.1f%%' % (i, 100.0 * c / tot))

    w('')
    w('=== 缺口件改成钢 7850 的敏感性 ===')
    for x in fb:
        m2 = x['vol'] * 1e-9 * 7850.0
        w('  %-32s %10.6f -> %10.6f kg   (%+.2f%% of total)'
          % (x['part'], x['mass'], m2, 100.0 * (m2 - x['mass']) / tot))

    w('')
    w('⚠ 改密度表 ≠ 修 bug —— 那只是把「一个猜测」换成「另一个猜测」。')
    w('   唯一能真正提升可信度的动作是**称重**（实物单件质量 ÷ 已知体积）。')

    txt = os.path.join(OUTDIR, 'mass_budget.txt')
    csvp = os.path.join(OUTDIR, 'mass_budget.csv')
    io.open(txt, 'w', encoding='utf-8').write('\n'.join(L))
    with io.open(csvp, 'w', encoding='utf-8-sig', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['part', 'volume_mm3', 'rho_kg_m3', 'density_basis', 'fallback',
                     'mass_kg', 'share_pct', 'cum_pct'])
        for x in rows:
            wr.writerow([x['part'], '%.6f' % x['vol'], '%.0f' % x['rho'], x['lab'],
                         'Y' if x['fb'] else '', '%.9f' % x['mass'],
                         '%.3f' % x['share'], '%.3f' % x['cum']])
    print('WROTE', txt)
    print('WROTE', csvp)
    print('fallback parts = %d/%d  (%.2f%% of mass)' % (len(fb), len(rows), 100.0 * fbm / tot))


if __name__ == '__main__':
    main()
