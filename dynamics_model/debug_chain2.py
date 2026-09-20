#!/usr/bin/env python3
"""Debug: trace SHAPE_REPRESENTATION chain to ABREP"""
import re, os, time
from collections import defaultdict

_STP_DIR = r'C:\Users\29408\Desktop\外骨骼'
STP = os.path.join(_STP_DIR, [f for f in os.listdir(_STP_DIR) if f.endswith('.stp')][0])

t0 = time.time()

refs = defaultdict(list)
all_entities = {}
entity_re = re.compile(r'^#(\d+)=([A-Z_]+)\((.*);$')
ref_re = re.compile(r'#(\d+)')

print("Reading...")
with open(STP, 'r', errors='ignore') as f:
    for line in f:
        m = entity_re.match(line.strip())
        if m:
            eid = int(m.group(1))
            all_entities[eid] = (m.group(2), m.group(3))
            for ref in ref_re.findall(m.group(3)):
                refs[int(ref)].append(eid)

print(f"Read in {time.time()-t0:.1f}s")

# SDR #12394 -> SHAPE_REPRESENTATION #27534
# Check what SR #27534 references
sr_id = 27534
print(f"\nSR #{sr_id}: {all_entities[sr_id][1][:150]}...")
sr_refs = [int(x) for x in ref_re.findall(all_entities[sr_id][1])]
for ref_id in sr_refs:
    if ref_id in all_entities:
        rt, rb = all_entities[ref_id]
        print(f"  -> #{ref_id} = {rt}({rb[:100]}...)")

# Find how many SR -> ABREP links exist
print("\n=== Checking SR -> ABREP links ===")
sr_list = [eid for eid, (t, _) in all_entities.items() if t == 'SHAPE_REPRESENTATION']
print(f"Total SHAPE_REPRESENTATION: {len(sr_list)}")

sr_to_abrep = {}
for sr_id in sr_list[:20]:
    sr_refs = [int(x) for x in ref_re.findall(all_entities[sr_id][1])]
    abrep_refs = [r for r in sr_refs if r in all_entities and all_entities[r][0] == 'ADVANCED_BREP_SHAPE_REPRESENTATION']
    if abrep_refs:
        sr_to_abrep[sr_id] = abrep_refs
        print(f"  SR #{sr_id} -> ABREP: {abrep_refs}")

# Actually, maybe SR doesn't directly reference ABREP
# Let's check: what references ABREP?
print("\n=== Who references ABREP? ===")
abrep_list = [eid for eid, (t, _) in all_entities.items() if t == 'ADVANCED_BREP_SHAPE_REPRESENTATION']
for abrep_id in abrep_list[:5]:
    parents = refs.get(abrep_id, [])
    for pid in parents[:3]:
        if pid in all_entities:
            pt, _ = all_entities[pid]
            print(f"  ABREP #{abrep_id} referenced by #{pid} = {pt}")

# The chain is likely: SR -> next representation items -> ABREP
# Check what's in SR's body
print("\n=== SR #27534 forward chain (depth 3) ===")
def trace(eid, depth=0, maxd=3):
    if eid not in all_entities or depth > maxd:
        return
    et, eb = all_entities[eid]
    print("  " * depth + f"#{eid} = {et}({eb[:80]})")
    for ref in ref_re.findall(eb)[:10]:
        trace(int(ref), depth + 1, maxd)

trace(sr_id)

print(f"\nDone in {time.time()-t0:.1f}s")
