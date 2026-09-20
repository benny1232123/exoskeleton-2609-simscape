#!/usr/bin/env python3
"""STEP完整解析: NAUO(中文名) -> PD -> PDS -> SDR -> SR -> SRR -> ABREP -> 点云 -> 尺寸"""
import re, os, time
from collections import defaultdict
import numpy as np

_STP_DIR = r'C:\Users\29408\Desktop\外骨骼'
STP = os.path.join(_STP_DIR, [f for f in os.listdir(_STP_DIR) if f.endswith('.stp')][0])
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cad_analysis.txt')

t0 = time.time()

def decode_name(name):
    def repl(m):
        h = m.group(1)
        return ''.join(chr(int(h[i:i+4], 16)) for i in range(0, len(h), 4))
    return re.sub(r'\\X2\\([0-9A-Fa-f]+)\\X0\\', repl, name)

print("Step 1: 读取全文件(处理多行实体)...")
with open(STP, 'r', errors='ignore') as f:
    text = f.read()
print(f"  {len(text)/1e6:.0f}MB in {time.time()-t0:.1f}s")

# 按 ';' 分割实体
print("Step 2: 解析实体...")
entity_re = re.compile(r'#(\d+)=([A-Z_0-9]+)\((.*)', re.DOTALL)
chunks = text.split(';')
all_entities = {}   # eid -> (etype, body)
refs = defaultdict(list)
for chunk in chunks:
    m = entity_re.match(chunk.strip())
    if m:
        eid = int(m.group(1))
        etype = m.group(2)
        body = m.group(3)
        all_entities[eid] = (etype, body)
        for ref in re.findall(r'#(\d+)', body):
            refs[int(ref)].append(eid)
print(f"  {len(all_entities)} 实体, {time.time()-t0:.1f}s")

ref_re = re.compile(r'#(\d+)')
pt_re = re.compile(r'CARTESIAN_POINT\([\'"\'\'\"]*\s*,\s*\(([^)]+)\)')

# Step 3: NAUO -> 中文名 + child_PD
print("Step 3: 解析NAUO(中文名)...")
nauo_name = {}       # child_pd_id -> 中文名
nauo_pairs = []      # (parent_pd, child_pd, name)
for eid, (etype, body) in all_entities.items():
    if etype == 'NEXT_ASSEMBLY_USAGE_OCCURRENCE':
        # ('1001',' ','\X2\..\X0\',#parentPD,#childPD,$)
        m = re.search(r"'([^']*)','([^']*)','((?:[^'\\]|\\.|\\X2\\.*?\\X0\\)*)',#(\d+),#(\d+)", body, re.DOTALL)
        if m:
            name = decode_name(m.group(3))
            parent_pd = int(m.group(4))
            child_pd = int(m.group(5))
            nauo_name[child_pd] = name
            nauo_pairs.append((parent_pd, child_pd, name))
print(f"  {len(nauo_name)} NAUO条目")

# Step 4: 链条构建
print("Step 4: 构建链条...")
# PDS -> (SDR, SR)
pds_to_sr = {}
for eid, (etype, body) in all_entities.items():
    if etype == 'SHAPE_DEFINITION_REPRESENTATION':
        rl = [int(x) for x in ref_re.findall(body)]
        if len(rl) >= 2:
            pds_to_sr[rl[0]] = rl[1]
# SRR: sr <-> abrep (多行已处理)
sr_to_abrep = {}
for eid, (etype, body) in all_entities.items():
    if etype == 'SHAPE_REPRESENTATION_RELATIONSHIP':
        rl = [int(x) for x in ref_re.findall(body)]
        if len(rl) >= 2:
            a, b = rl[0], rl[1]
            if b in all_entities and all_entities[b][0] == 'ADVANCED_BREP_SHAPE_REPRESENTATION':
                sr_to_abrep[a] = b
            elif a in all_entities and all_entities[a][0] == 'ADVANCED_BREP_SHAPE_REPRESENTATION':
                sr_to_abrep[b] = a
print(f"  SDR: {len(pds_to_sr)}, SRR->ABREP: {len(sr_to_abrep)}")

# PD -> PDS -> SR -> ABREP
pd_to_abrep = {}
for eid, (etype, body) in all_entities.items():
    if etype == 'PRODUCT_DEFINITION_SHAPE':
        rl = [int(x) for x in ref_re.findall(body)]
        for r in rl:
            if eid in pds_to_sr:
                sr_id = pds_to_sr[eid]
                if sr_id in sr_to_abrep:
                    # PDS正引用PD
                    pd_refs = [int(x) for x in ref_re.findall(body)]
                    for pd_id in pd_refs:
                        if pd_id in all_entities and all_entities[pd_id][0] == 'PRODUCT_DEFINITION':
                            pd_to_abrep[pd_id] = sr_to_abrep[sr_id]
                    break
print(f"  PD->ABREP: {len(pd_to_abrep)}")

# 中文名 -> ABREP (通过nauo的child_pd)
name_to_abrep = {}
for pd_id, abrep_id in pd_to_abrep.items():
    if pd_id in nauo_name:
        name_to_abrep.setdefault(nauo_name[pd_id], []).append(abrep_id)
print(f"  中文名->ABREP: {len(name_to_abrep)}")

# Step 5: 提取每个ABREP的CARTESIAN_POINT
print("\nStep 5: 提取几何点...")

def extract_points(start_id, max_depth=50):
    visited = set()
    pts = []
    stack = [(start_id, 0)]
    while stack:
        eid, depth = stack.pop()
        if eid in visited or depth > max_depth:
            continue
        visited.add(eid)
        ent = all_entities.get(eid)
        if ent is None:
            continue
        etype, body = ent
        if etype == 'CARTESIAN_POINT':
            m = re.search(r'\((\s*[-+\d.Ee]+\s*,\s*[-+\d.Ee]+\s*,\s*[-+\d.Ee]+\s*)\)', body)
            if m:
                try:
                    x, y, z = [float(v) for v in m.group(1).split(',')]
                    pts.append((x, y, z))
                except ValueError:
                    pass
        for ref in ref_re.findall(body):
            stack.append((int(ref), depth + 1))
    return pts

results = []
structural_kw = ['腿部', '背部', '电机', '管夹', '面板', '绑缚', '装配', '主机架', '电池仓']
for name, abreps in sorted(name_to_abrep.items()):
    all_pts = []
    for aid in abreps:
        all_pts.extend(extract_points(aid))
    if not all_pts:
        continue
    arr = np.array(all_pts)
    # 去离群点(参考平面±55000)
    med = np.median(arr, axis=0)
    d = np.abs(arr - med)
    core = arr[np.all(d < 2000, axis=1)]
    if len(core) < 10:
        core = arr
    bmin, bmax = core.min(axis=0), core.max(axis=0)
    size = bmax - bmin
    results.append((name, bmin, bmax, size, len(core)))
    flag = '***' if any(kw in name for kw in structural_kw) else '   '
    print(f"  {flag} {name[:40]:40s} pts={len(core):6d} 尺寸={size[0]:7.1f}x{size[1]:7.1f}x{size[2]:7.1f}mm  "
          f"Z=[{bmin[2]:8.1f},{bmax[2]:8.1f}]")

# Step 6: 关节轴
print("\nStep 6: AXIS1_PLACEMENT 关节轴:")
for eid, (etype, body) in all_entities.items():
    if etype == 'AXIS1_PLACEMENT':
        rl = [int(x) for x in ref_re.findall(body)]
        loc = "?"
        if rl and rl[0] in all_entities:
            m = re.search(r'\(([-+\d.Ee]+),([-+\d.Ee]+),([-+\d.Ee]+)\)', all_entities[rl[0]][1])
            if m:
                loc = f"({float(m.group(1)):.1f},{float(m.group(2)):.1f},{float(m.group(3)):.1f})"
        dr = "?"
        if len(rl) > 1 and rl[1] in all_entities:
            m = re.search(r'\(([-+\d.Ee]+),([-+\d.Ee]+),([-+\d.Ee]+)\)', all_entities[rl[1]][1])
            if m:
                dr = f"({float(m.group(1)):.2f},{float(m.group(2)):.2f},{float(m.group(3)):.2f})"
        parents = [all_entities[p][0] for p in refs.get(eid, [])[:2] if p in all_entities]
        print(f"  #{eid}: loc={loc} dir={dr} in={parents}")

# 保存结果
with open(OUT, 'w', encoding='utf-8') as f:
    f.write(f"外骨骼STP解析结果 {time.strftime('%Y-%m-%d %H:%M')}\n")
    f.write("=" * 100 + "\n")
    for name, bmin, bmax, size, n in sorted(results, key=lambda r: r[1][2]):
        f.write(f"{name[:50]:50s} pts={n:6d} bbox=[{bmin[0]:.1f},{bmin[1]:.1f},{bmin[2]:.1f}]~"
                f"[{bmax[0]:.1f},{bmax[1]:.1f},{bmax[2]:.1f}] size={size[0]:.1f}x{size[1]:.1f}x{size[2]:.1f}mm\n")
print(f"\n结果已保存: {OUT}")
print(f"总耗时 {time.time()-t0:.1f}s")
