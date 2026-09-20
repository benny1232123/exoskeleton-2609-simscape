# -*- coding: utf-8 -*-
"""对账：MATLAB smimport 导入出来的 smiData  vs  SolidWorks 内核基准。

单位（基准表已统一，别再手工换算）：
  mass      : kg
  CoM       : mm    （基准表 com_mm 已完成 m->mm）
  MoI       : kg*mm^2 = 基准表 inertia_kgmm2 的前 3 个对角元

匹配算法：**贪心 + 已占用**。
  早先用「质量最近」匹配，但 腿部设计_左-5 / -6 质量完全相同（都是 20.335 mg），
  两条基准会取到同一条 smiData，看起来像 CoM 差 8 mm —— 纯粹是匹配伪影。
"""
import json, csv

BASE = r'C:\Users\29408\exo_work\_sw_baseline_legL.json'
SMI  = r'C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\smiData_dump.csv'
OUT  = r'C:\Users\29408\exo_work\_sw_ml_xcheck.txt'

buf = []
def w(s=''):
    buf.append(str(s))

base = json.load(open(BASE, encoding='utf-8'))['parts']
smi = []
with open(SMI, encoding='utf-8-sig', newline='') as f:
    for r in csv.DictReader(f):
        smi.append({'idx': int(r['idx']), 'mass': float(r['mass_kg']),
                    'com': [float(r['comx_mm']), float(r['comy_mm']), float(r['comz_mm'])],
                    'MoI': [float(r['Ixx']), float(r['Iyy']), float(r['Izz'])]})

w('SolidWorks 基准条目 = %d' % len(base))
w('smiData 条目       = %d' % len(smi))
w('')
w('%-30s %13s %13s %9s | %-24s | %-24s | %s' %
  ('Part', 'SW mass_kg', 'ML mass_kg', 'rel', 'SW CoM mm', 'ML CoM mm', 'MoI'))
w('-' * 132)

used = set()
n_ok = 0
w_m = w_c = w_i = 0.0
for b in base:
    m0 = b['mass_kg']
    if not m0:
        continue
    com0 = b['com_mm']
    cands = [s for s in smi if s['idx'] not in used] or smi

    def cost(s):
        # ★ 坑：质量完全相同的孪生件（腿部设计_左-5 / -6 都是 20.335 mg）必须靠 CoM
        #   打破平局，否则贪心会配对颠倒，看上去像 CoM 差 8 mm。
        dm = abs(s['mass'] - m0) / m0
        dc = 0.0
        if com0[0] is not None:
            dc = max(abs(s['com'][i] - com0[i]) for i in range(3)) / 1000.0
        return dm + dc

    c = min(cands, key=cost)
    used.add(c['idx'])

    rel = abs(c['mass'] - m0) / m0
    dcom = max(abs(c['com'][i] - com0[i]) for i in range(3)) if com0[0] is not None else float('nan')
    I0 = (b.get('inertia_kgmm2') or [])[:3]
    if all(x is not None for x in I0):
        den = max(abs(x) for x in I0) or 1.0
        dmoi = max(abs(c['MoI'][i] - I0[i]) for i in range(3)) / den
    else:
        dmoi = float('nan')

    ok = (rel < 1e-9) and (dcom < 1e-6) and (dmoi < 1e-6)
    n_ok += 1 if ok else 0
    w_m = max(w_m, rel)
    if dcom == dcom:  w_c = max(w_c, dcom)
    if dmoi == dmoi:  w_i = max(w_i, dmoi)

    w('%-30s %13.9f %13.9f %9.1e | %-24s | %-24s | %-9.1e %s' %
      (b['part'], m0, c['mass'], rel,
       '[%.3f %.3f %.3f]' % tuple(com0) if com0[0] is not None else '-',
       '[%.3f %.3f %.3f]' % tuple(c['com']), dmoi, 'OK' if ok else 'DIFF'))

n = len([b for b in base if b['mass_kg']])
w('')
w('逐件全项一致 = %d / %d' % (n_ok, n))
w('最大偏差: mass rel %.2e | CoM abs %.2e mm | MoI rel %.2e' % (w_m, w_c, w_i))
w('Σ SW mass(19 唯一件) = %.9f kg' % sum(b['mass_kg'] for b in base if b['mass_kg']))
w('Σ ML mass(19 条)     = %.9f kg' % sum(s['mass'] for s in smi))
w('（模型里 35 个 File Solid 按**实例**重复计数，合计 0.101060 kg，'
  '差值 0.001976 kg 全部来自多实例零件）')
w('')
verdict = 'SW_ML_XCHECK_OK' if (n_ok == n and w_m < 1e-9 and w_c < 1e-6 and w_i < 1e-6) \
          else 'SW_ML_XCHECK_MISMATCH'
w('VERDICT: %s' % verdict)

open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
