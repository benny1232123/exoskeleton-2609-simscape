# -*- coding: utf-8 -*-
"""核验：装配体到底有没有配合关系。
1) 扫 腿部设计_左.SLDASM 的完整特征树，统计 feature 类型
2) 静默打开顶层主装配体，统计"固定"零部件数与配合特征数
全程只读，不保存。"""
import sys, traceback, time

OUT = r'C:\Users\29408\exo_work\_sw_mates2.txt'
buf = []
def w(s=''):
    buf.append(str(s))
def esc(s):
    if s is None:
        return '<None>'
    return ''.join(ch if 32 <= ord(ch) < 127 else ('<%02X>' % ord(ch) if ord(ch) < 32 else ch)
                   for ch in s)

def prop(o, name, default=None):
    try:
        return getattr(o, name)
    except Exception:
        return default

def meth(o, name, *args):
    try:
        f = getattr(o, name)
        return f(*args)
    except Exception:
        return None

from win32com.client import GetActiveObject
sw = GetActiveObject('SldWorks.Application')

def scan_features(doc, label, dump_limit=25):
    """遍历特征树，统计类型"""
    from collections import Counter
    cnt = Counter()
    names_by_type = {}
    feat = prop(doc, 'FirstFeature')
    n = 0
    while feat is not None and n < 20000:
        n += 1
        tn = meth(feat, 'GetTypeName2') or '?'
        nm = prop(feat, 'Name') or ''
        cnt[tn] += 1
        names_by_type.setdefault(tn, []).append(nm)
        feat = meth(feat, 'GetNextFeature')
    w('---- %s 特征树 ----' % label)
    w('特征总数 = %d' % n)
    for k, v in cnt.most_common():
        w('   %-34s %d' % (esc(k), v))
    # 把所有非"零件本体/基准面"的特征名也列出来
    for k, v in cnt.most_common():
        if any(s in k for s in ('Mate', 'mate', '配合')):
            w('   >>> 配合特征 %s : %s' % (esc(k), esc(str(names_by_type[k][:dump_limit]))))
    w('')
    return cnt

# ===== 1) 当前打开的 腿部设计_左 =====
asm = None
for d in list(prop(sw, 'GetDocuments') or []):
    if prop(d, 'GetType') == 2:
        asm = d
        break
if asm is not None:
    w('== 已打开的装配体: %s ==' % esc(prop(asm, 'GetTitle')))
    w('')
    # 用 GetMates 再试一次（早绑定缺失时走特征树兜底）
    m = meth(asm, 'GetMates')
    w('IassemblyDoc.GetMates -> %r' % (None if m is None else ('array len=%d' % len(list(m)))))
    scan_features(asm, esc(prop(asm, 'GetTitle')))
    # 逐零件再确认一次
    comps = meth(asm, 'GetComponents', True)
    comps = list(comps) if comps else []
    nfix = sum(1 for c in comps if prop(c, 'IsFixed'))
    nfloat = sum(1 for c in comps if prop(c, 'IsFixed') is False)
    w('零部件 %d : IsFixed=True %d , IsFixed=False %d' % (len(comps), nfix, nfloat))
    w('')

# ===== 2) 静默打开顶层主装配体 =====
MAIN = r'C:\Users\29408\Desktop\外骨骼\装配体\“林-Ⅰ”髋关节外骨骼机器人开发平台V0_1_1.SLDASM'
w('== 顶层主装配体 ==')
w('path = %s' % esc(MAIN))
w('exists on disk = %s' % __import__('os').path.exists(MAIN))
t0 = time.time()
try:
    errs = None
    res = None
    # swOpenDocOptions_Silent = 1 ; type 2 = assembly
    try:
        res = sw.OpenDoc6(MAIN, 2, 1, '', 0, 0)
    except Exception:
        res = None
    if res is None:
        try:
            res = sw.OpenDoc(MAIN, 2)
        except Exception:
            res = None
    w('OpenDoc -> %r   (%.1f s)' % (res is not None, time.time() - t0))
    doc = prop(sw, 'ActiveDoc')
    if doc is not None:
        w('ActiveDoc = %s' % esc(prop(doc, 'GetTitle')))
        comps = meth(doc, 'GetComponents', True)
        comps = list(comps) if comps else []
        w('零部件总数 = %d' % len(comps))
        if comps:
            nfix = sum(1 for c in comps if prop(c, 'IsFixed'))
            w('IsFixed=True = %d / %d' % (nfix, len(comps)))
        scan_features(doc, '主装配体')
except Exception:
    w('主装配体检查失败')
    w(traceback.format_exc())

open(OUT, 'w', encoding='utf-8').write('\n'.join(buf))
print('WROTE ' + OUT)
