#!/usr/bin/env python3
"""Debug: trace entity reference chains from PDS"""
import re, os, time
from collections import defaultdict

_STP_DIR = r'C:\Users\29408\Desktop\外骨骼'
STP = os.path.join(_STP_DIR, [f for f in os.listdir(_STP_DIR) if f.endswith('.stp')][0])

t0 = time.time()

refs = defaultdict(list)
all_entities = {}
entity_re = re.compile(r'^#(\d+)=([A-Z_]+)\((.*);$')
ref_re = re.compile(r'#(\d+)')

print("Reading STEP file...")
count = 0
with open(STP, 'r', errors='ignore') as f:
    for line in f:
        m = entity_re.match(line.strip())
        if m:
            eid = int(m.group(1))
            etype = m.group(2)
            body = m.group(3)
            all_entities[eid] = (etype, body)
            for ref in ref_re.findall(body):
                refs[int(ref)].append(eid)
            count += 1

print(f"Read {count} entities in {time.time()-t0:.1f}s")

# 1. Find a known PDS and trace its chain
# Pick a few PDS IDs
pds_list = [eid for eid, (t, _) in all_entities.items() if t == 'PRODUCT_DEFINITION_SHAPE']
print(f"\nTotal PDS: {len(pds_list)}")

# Trace first 5 PDS
for pds_id in pds_list[:5]:
    print(f"\n--- PDS #{pds_id} ---")
    # What references this PDS? (reverse lookup)
    parents = refs.get(pds_id, [])
    print(f"  Referenced by: {len(parents)} entities")
    for pid in parents[:5]:
        if pid in all_entities:
            pt, pb = all_entities[pid]
            print(f"    #{pid} = {pt}({pb[:80]}...)")
    
    # Forward references in PDS body
    pds_refs = [int(x) for x in ref_re.findall(all_entities[pds_id][1])]
    print(f"  Forward refs: {pds_refs[:5]}")
    for ref_id in pds_refs[:3]:
        if ref_id in all_entities:
            rt, rb = all_entities[ref_id]
            print(f"    #{ref_id} = {rt}({rb[:100]}...)")

# 2. Check if there are PRODUCT_DEFINITION in the forward refs of PDS
print("\n\n=== Checking PDS -> PD forward chain ===")
for pds_id in pds_list[:3]:
    pds_refs = [int(x) for x in ref_re.findall(all_entities[pds_id][1])]
    for ref_id in pds_refs:
        if ref_id in all_entities:
            rt, rb = all_entities[ref_id]
            if rt == 'PRODUCT_DEFINITION':
                print(f"PDS #{pds_id} -> PD #{ref_id}")
                # What references this PD?
                pd_parents = refs.get(ref_id, [])
                for pp in pd_parents[:3]:
                    if pp in all_entities:
                        print(f"  PD #{ref_id} referenced by #{pp} = {all_entities[pp][0]}")

# 3. Find SHAPE_DEFINITION_REPRESENTATION and its chain
print("\n\n=== SHAPE_DEFINITION_REPRESENTATION chain ===")
sdr_list = [eid for eid, (t, _) in all_entities.items() if t == 'SHAPE_DEFINITION_REPRESENTATION']
print(f"Total SDR: {len(sdr_list)}")
for sdr_id in sdr_list[:3]:
    print(f"\nSDR #{sdr_id}: {all_entities[sdr_id][1][:120]}...")
    # Forward refs
    sdr_refs = [int(x) for x in ref_re.findall(all_entities[sdr_id][1])]
    for ref_id in sdr_refs:
        if ref_id in all_entities:
            rt, rb = all_entities[ref_id]
            print(f"  -> #{ref_id} = {rt}")

# 4. Check ABREP chain
print("\n\n=== ABREP references ===")
abrep_list = [eid for eid, (t, _) in all_entities.items() if t == 'ADVANCED_BREP_SHAPE_REPRESENTATION']
print(f"Total ABREP: {len(abrep_list)}")
for abrep_id in abrep_list[:3]:
    print(f"\nABREP #{abrep_id}: {all_entities[abrep_id][1][:120]}...")
    abrep_refs = [int(x) for x in ref_re.findall(all_entities[abrep_id][1])]
    for ref_id in abrep_refs[:5]:
        if ref_id in all_entities:
            rt, rb = all_entities[ref_id]
            print(f"  -> #{ref_id} = {rt}")

print(f"\nDone in {time.time()-t0:.1f}s")
