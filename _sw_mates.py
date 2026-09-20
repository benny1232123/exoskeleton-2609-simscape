# -*- coding: utf-8 -*-
"""只读盘点：腿部设计_左.SLDASM 的配合(mates)与零部件。
目的：判定 Simscape 插件导出 0 关节的根因 —— 是"装配体本来就没配合"，
      还是"有配合但插件没映射"。"""
import sys, traceback

OUT = r'C:\Users\29408\exo_work\_sw_mates.txt'
buf = []
def w(s=''):
    buf.append(str(s))
def esc(s):
    if s is None:
        return '<None>'
    return ''.join(ch if 32 <= ord(ch) < 127 else ('<%02X>' % ord(ch) if ord(ch) < 32 else ch)
                   for ch in s)
def getv(o, name, *args):
    a = getattr(o, name)
    return a(*args) if callable(a) else a

MATE_TYPES = {
    0: 'COINCIDENT', 1: 'CONCENTRIC', 2: 'PERPENDICULAR', 3: 'PARALLEL',
    4: 'TANGENT', 5: 'DISTANCE', 6: 'ANGLE', 7: 'UNKNOWN', 8: 'SYMMETRIC',
    9: 'CAM', 10: 'GEAR', 11: 'RACKPINION', 12: 'SCREW', 13: 'UNIVERSAL',
    14: 'HINGE', 15: 'LOCK', 16: 'COORDINATE', 17: 'WIDTH',
    18: 'LIMITDISTANCE', 19: 'LIMITANGLE', 20: 'SLOT',
    21: 'BEARING', 22: 'SCREW', 23: 'PATH',
}
# SW 里 mate 的 swMateType_e 官方值（常见）
MATE_TYPES.update({0: 'COINCIDENT', 1: 'CONCENTRIC', 2: 'PERPENDICULAR', 3: 'PARALLEL',
                   4: 'TANGENT', 5: 'DISTANCE', 6: 'ANGLE', 7: 'UNKNOWN',
                   8: 'SYMMETRIC', 9: 'CAM', 10: 'GEAR', 11: 'RACK_AND_PINION',
                   12: 'SCREW', 13: 'UNIVERSAL_JOINT', 14: 'HINGE', 15: 'LOCK'})

from win32com.client import GetActiveObject
sw = GetActiveObject('SldWorks.Application')
docs = list(getv(sw, 'GetDocuments'))

asm = None
for d in docs:
    try:
        if getv(d, 'GetType') == 2:
            asm = d
            break
    except Exception:
        pass

if asm is None:
    w('NO ASSEMBLY OPEN')
    open(OUT, 'w', encoding='utf-8').write('\n'.join(buf)); sys.exit(0)

w('assembly = %s' % esc(getv(asm, 'GetTitle')))
w('path     = %s' % esc(getv(asm, 'GetPathName')))
w('')

# ---- 零部件 ----
w('==== 零部件 (components) ====')
try:
    comps = getv(asm, 'GetComponents', True)   # True = 含隐藏/压缩
    comps = list(comps) if comps else []
    w('count = %d' % len(comps))
    fixed_n = 0
    for i, c in enumerate(comps):
        try:
            nm = getv(c, 'Name2')
        except Exception:
            nm = '?'
        try:
            p = getv(c, 'GetPathName')
        except Exception:
            p = '?'
        try:
            isfix = getv(c, 'IsFixed')
        except Exception:
            isfix = None
        try:
            sup = getv(c, 'IsSuppressed')
        except Exception:
            sup = None
        if isfix:
            fixed_n += 1
        w('  [%2d] %-34s fixed=%-5s supp=%-5s  %s'
          % (i, esc(nm), isfix, sup, esc(p.rsplit('\\', 1)[-1] if p else p)))
    w('IsFixed==True 的零件 = %d / %d' % (fixed_n, len(comps)))
except Exception:
    w('GetComponents FAIL')
    w(traceback.format_exc())

# ---- 配合 ----
w('')
w('==== 配合 (mates) ====')
try:
    mates = getv(asm, 'GetMates')
    mates = list(mates) if mates else []
except Exception:
    mates = []
    w('GetMates FAIL: %s' % traceback.format_exc())

w('mate feature 数 = %d' % len(mates))

from collections import Counter
cnt = Counter()
detail = []
for i, mf in enumerate(mates):
    rec = {'i': i}
    try:
        rec['fname'] = getv(mf, 'Name')
    except Exception:
        rec['fname'] = '?'
    try:
        rec['ftype'] = getv(mf, 'GetTypeName2')
    except Exception:
        rec['ftype'] = '?'
    try:
        m2 = getv(mf, 'GetSpecificFeature2')
    except Exception:
        m2 = None
    rec['mate'] = m2
    if m2 is not None:
        try:
            rec['mtype'] = getv(m2, 'Type')
        except Exception:
            rec['mtype'] = None
        try:
            rec['nent'] = getv(m2, 'MateEntityCount')
        except Exception:
            rec['nent'] = None
        ents = []
        try:
            n = rec['nent'] or 0
            for j in range(n):
                me = getv(m2, 'MateEntity', j)
                e = {}
                try:
                    e['reftype'] = getv(me, 'ReferenceType')
                except Exception:
                    e['reftype'] = None
                try:
                    rc = getv(me, 'ReferenceComponent')
                    e['comp'] = getv(rc, 'Name2') if rc is not None else None
                except Exception:
                    e['comp'] = None
                try:
                    e['sel'] = getv(me, 'ReferenceType2')
                except Exception:
                    pass
                ents.append(e)
        except Exception as ex:
            ents.append({'err': repr(ex)})
        rec['ents'] = ents
    detail.append(rec)
    t = rec.get('ftype') or '?'
    cnt[t] += 1

w('')
w('按 feature 类型统计：')
for k, v in cnt.most_common():
    w('   %-28s %d' % (esc(k), v))

w('')
w('按 mate 类型统计：')
mc = Counter()
for r in detail:
    mt = r.get('mtype')
    mc[MATE_TYPES.get(mt, 'Type=%s' % mt)] += 1
for k, v in mc.most_common():
    w('   %-28s %d' % (esc(str(k)), v))

w('')
w('==== 逐条明细（前 60 条）====')
for r in detail[:60]:
    mt = r.get('mtype')
    w('  [%3d] feat=%-22s ftype=%-16s mateType=%s(%s) nent=%s '
      % (r['i'], esc(str(r.get('fname'))), esc(str(r.get('ftype'))),
         mt, MATE_TYPES.get(mt, '?'), r.get('nent')))
    for e in (r.get('ents') or []):
        w('         - reftype=%-6s comp=%s' % (e.get('reftype'), esc(str(e.get('comp')))))

# ---- 装配体的特征树里所有 "Mate" 类特征 ----
w('')
w('==== 特征树扫描（含 Mate 关键字的特征）====')
try:
    feat = getv(asm, 'FirstFeature')
    n = 0
    hits = 0
    while feat is not None and n < 5000:
        n += 1
        try:
            tn = getv(feat, 'GetTypeName2')
        except Exception:
            tn = ''
        try:
            fn = getv(feat, 'Name')
        except Exception:
            fn = ''
        if tn and ('Mate' in tn or 'mate' in tn):
            hits += 1
            if hits <= 40:
                w('   %-28s | %s' % (esc(tn), esc(fn)))
        try:
            feat = getv(feat, 'GetNextFeature')
        except Exception:
            break
    w('特征总数 = %d ，其中 Mate 类 = %d' % (n, hits))
except Exception:
    w('feature scan FAIL')
    w(traceback.format_exc())

open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
