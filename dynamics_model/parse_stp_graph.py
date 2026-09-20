#!/usr/bin/env python3
"""STEP parser: extract per-product bounding boxes via correct entity chain
Chain: PRODUCT -> PD -> PDS -> SDR -> SR -> SRR -> ABREP -> MSB -> CLOSED_SHELL -> ADVANCED_FACE -> ... -> CARTESIAN_POINT
"""
import re, os, time
from collections import defaultdict
import numpy as np

_STP_DIR = r'C:\Users\29408\Desktop\外骨骼'
STP = os.path.join(_STP_DIR, [f for f in os.listdir(_STP_DIR) if f.endswith('.stp')][0])

t0 = time.time()

all_entities = {}   # eid -> (etype, body)
refs = defaultdict(list)  # target -> [sources]
entity_re = re.compile(r'^#(\d+)=([A-Z_]+)\((.*);$')
ref_re = re.compile(r'#(\d+)')

print("Step 1: Reading STEP file...")
count = 0
with open(STP, 'r', errors='ignore') as f:
    for line in f:
        m = entity_re.match(line.strip())
        if m:
            eid = int(m.group(1))
            all_entities[eid] = (m.group(2), m.group(3))
            for ref in ref_re.findall(m.group(3)):
                refs[int(ref)].append(eid)
            count += 1
            if count % 500000 == 0:
                print(f"  {count/1e6:.1f}M...")
print(f"  Total: {count} entities in {time.time()-t0:.1f}s")

# Step 2: Find PRODUCT names
print("\nStep 2: Mapping PRODUCT names...")
product_names = {}
for eid, (etype, body) in all_entities.items():
    if etype == 'PRODUCT':
        m = re.match(r"'([^']*)'", body)
        if m:
            product_names[eid] = m.group(1)

# Decode \X2\...\X0\ encoding
def decode_name(name):
    def repl(m):
        hex_str = m.group(1)
        chars = []
        for i in range(0, len(hex_str), 4):
            chars.append(chr(int(hex_str[i:i+4], 16)))
        return ''.join(chars)
    return re.sub(r'\\X2\\(.*?)\\X0\\', repl, name)

# Filter structural parts
structural_keywords = ['腿部', '背部', '电机', '管夹', '面板', '装配', '绑缚', '腿杆', '滑轨', '滑块', '轴盖', '出轴']
print("  Key structural products:")
for pid, name in sorted(product_names.items()):
    decoded = decode_name(name)
    if any(kw in decoded for kw in structural_keywords):
        print(f"    #{pid}: {decoded}")

# Step 3: Build product -> ABREP chain
print("\nStep 3: Building entity chains...")

# 3a: PDS -> SDR (reverse: SDR references PDS)
pds_to_sdr = {}
for eid, (etype, body) in all_entities.items():
    if etype == 'SHAPE_DEFINITION_REPRESENTATION':
        refs_list = [int(x) for x in ref_re.findall(body)]
        if len(refs_list) >= 2:
            pds_id = refs_list[0]
            sr_id = refs_list[1]
            pds_to_sdr[pds_id] = (eid, sr_id)
print(f"  SDR count: {len(pds_to_sdr)}")

# 3b: SR -> ABREP via SHAPE_REPRESENTATION_RELATIONSHIP
sr_to_abrep = {}
for eid, (etype, body) in all_entities.items():
    if etype == 'SHAPE_REPRESENTATION_RELATIONSHIP':
        refs_list = [int(x) for x in ref_re.findall(body)]
        if len(refs_list) >= 2:
            sr_from = refs_list[0]
            sr_to = refs_list[1]
            # sr_to could be ABREP
            if sr_to in all_entities and all_entities[sr_to][0] == 'ADVANCED_BREP_SHAPE_REPRESENTATION':
                sr_to_abrep[sr_from] = sr_to
            elif sr_from in all_entities and all_entities[sr_from][0] == 'ADVANCED_BREP_SHAPE_REPRESENTATION':
                sr_to_abrep[sr_to] = sr_from
print(f"  SRR->ABREP links: {len(sr_to_abrep)}")

# 3c: PD -> PDS (reverse: PDS forward-refs PD)
pd_to_pds = {}
for pds_id, (sdr_id, sr_id) in pds_to_sdr.items():
    if pds_id in all_entities and all_entities[pds_id][0] == 'PRODUCT_DEFINITION_SHAPE':
        pd_refs = [int(x) for x in ref_re.findall(all_entities[pds_id][1])]
        for pd_id in pd_refs:
            if pd_id in all_entities and all_entities[pd_id][0] == 'PRODUCT_DEFINITION':
                pd_to_pds[pd_id] = pds_id

# 3d: PRODUCT -> PD (reverse: PD forward-refs PRODUCT)
product_to_pd = {}
for pd_id, pds_id in pd_to_pds.items():
    pd_refs = [int(x) for x in ref_re.findall(all_entities[pd_id][1])]
    for ref in pd_refs:
        if ref in product_names:
            product_to_pd[ref] = pd_id

# Full chain: PRODUCT -> PD -> PDS -> SDR -> SR -> ABREP
product_to_abrep = {}
for prod_id, pd_id in product_to_pd.items():
    if pd_id in pd_to_pds:
        pds_id = pd_to_pds[pd_id]
        if pds_id in pds_to_sdr:
            sdr_id, sr_id = pds_to_sdr[pds_id]
            if sr_id in sr_to_abrep:
                abrep_id = sr_to_abrep[sr_id]
                product_to_abrep[prod_id] = abrep_id

print(f"  Complete product->ABREP chains: {len(product_to_abrep)}")
for pid in sorted(product_to_abrep):
    name = decode_name(product_names.get(pid, '?'))
    print(f"    #{pid}: {name} -> ABREP#{product_to_abrep[pid]}")

# Step 4: Extract CARTESIAN_POINTs from ABREP -> MSB -> geometry chain
print("\nStep 4: Extracting geometry per product...")

def extract_cartesian_points(start_ids, max_depth=30):
    visited = set()
    pts = []
    stack = [(sid, 0) for sid in start_ids]
    while stack:
        eid, depth = stack.pop()
        if depth > max_depth or eid in visited:
            continue
        visited.add(eid)
        if eid not in all_entities:
            continue
        etype, body = all_entities[eid]
        if etype == 'CARTESIAN_POINT':
            m = re.search(r'\(([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?),([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?),([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?)\)', body)
            if m:
                try:
                    x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
                    pts.append((x, y, z))
                except ValueError:
                    pass
        for ref in ref_re.findall(body):
            rid = int(ref)
            if rid not in visited:
                stack.append((rid, depth + 1))
    return pts

product_boxes = {}
for pid, abrep_id in product_to_abrep.items():
    name = decode_name(product_names.get(pid, '?'))
    pts = extract_cartesian_points([abrep_id], max_depth=40)
    if pts:
        arr = np.array(pts)
        bmin, bmax = arr.min(axis=0), arr.max(axis=0)
        size = bmax - bmin
        product_boxes[pid] = (name, bmin, bmax, size, len(pts))
        # Filter out outliers: compute stats excluding points >10000mm from centroid
        centroid = arr.mean(axis=0)
        distances = np.linalg.norm(arr - centroid, axis=1)
        mask = distances < 10000
        if mask.sum() > 10:
            arr_core = arr[mask]
            bmin_c, bmax_c = arr_core.min(axis=0), arr_core.max(axis=0)
            size_c = bmax_c - bmin_c
            print(f"  #{pid} {name}: {len(pts)} pts, "
                  f"core_size={size_c[0]:.1f}x{size_c[1]:.1f}x{size_c[2]:.1f}mm  "
                  f"Z=[{bmin_c[2]:.1f},{bmax_c[2]:.1f}]mm")
        else:
            print(f"  #{pid} {name}: {len(pts)} pts, "
                  f"size={size[0]:.1f}x{size[1]:.1f}x{size[2]:.1f}mm  "
                  f"Z=[{bmin[2]:.1f},{bmax[2]:.1f}]mm")

# Step 5: Identify leg links (腿部 parts) and compute link dimensions
print("\n\n=== LEG LINK DIMENSIONS ===")
leg_parts = {}
for pid, (name, bmin, bmax, size, npts) in product_boxes.items():
    if '腿部' in name or '电机' in name:
        leg_parts[pid] = (name, bmin, bmax, size, npts)
        print(f"  {name}: {size[0]:.1f}x{size[1]:.1f}x{size[2]:.1f}mm")

# Identify thigh and shank by Z-range
if leg_parts:
    print("\n  Sorted by Z-min:")
    sorted_legs = sorted(leg_parts.values(), key=lambda x: x[1][2])
    for name, bmin, bmax, size, npts in sorted_legs:
        print(f"    {name}: Z=[{bmin[2]:.1f},{bmax[2]:.1f}] (span={bmax[2]-bmin[2]:.1f}mm)")

# Step 6: Extract joint axis positions
print("\n\n=== AXIS1_PLACEMENT (joint axes) ===")
for eid, (etype, body) in all_entities.items():
    if etype in ('AXIS1_PLACEMENT', 'AXIS2_PLACEMENT_3D'):
        refs_list = [int(x) for x in ref_re.findall(body)]
        loc_id = refs_list[0] if refs_list else None
        dir_id = refs_list[1] if len(refs_list) > 1 else None
        loc_str = "N/A"
        if loc_id and loc_id in all_entities:
            lb = all_entities[loc_id][1]
            m = re.search(r'\(([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?),([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?),([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?)\)', lb)
            if m:
                loc_str = f"({float(m.group(1)):.2f}, {float(m.group(2)):.2f}, {float(m.group(3)):.2f})"
        dir_str = "N/A"
        if dir_id and dir_id in all_entities:
            db = all_entities[dir_id][1]
            m = re.search(r'\(([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?),([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?),([-+]?\d+\.?\d*(?:[Ee][-+]?\d+)?)\)', db)
            if m:
                dir_str = f"({float(m.group(1)):.3f}, {float(m.group(2)):.3f}, {float(m.group(3)):.3f})"
        # Find parent
        parent_info = ""
        for r in refs.get(eid, [])[:3]:
            if r in all_entities:
                parent_info += f" {all_entities[r][0]}#{r}"
        print(f"  #{eid} {etype}: loc={loc_str} dir={dir_str} in:{parent_info}")

print(f"\nDone in {time.time()-t0:.1f}s")
