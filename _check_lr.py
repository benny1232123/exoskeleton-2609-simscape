# -*- coding: utf-8 -*-
"""交叉核对：左右腿的 Y 位移、参考件的全部实例"""
import io, json, os
import numpy as np
from collections import defaultdict

ROOT = r"C:\Users\29408\Desktop\外骨骼"
inst = json.load(io.open(os.path.join(ROOT, "_props_instances.json"), encoding="utf-8"))

groups = defaultdict(list)
for p in inst["placed"]:
    anc = p.get("ancestors") or []
    g = anc[0] if anc else "?"
    groups[g].append(p)

print("=== 顶层分组 ===")
for g in sorted(groups):
    ts = np.asarray([np.asarray(p["T"], float)[:3, 3] for p in groups[g]], float)
    print("  %-16s n=%-5d  y范围=[%.1f, %.1f]  x范围=[%.1f, %.1f]  z范围=[%.1f, %.1f]"
          % (g, len(ts), ts[:, 1].min(), ts[:, 1].max(),
             ts[:, 0].min(), ts[:, 0].max(), ts[:, 2].min(), ts[:, 2].max()))

print()
print("=== 参考件 腿部_轴盖 的全部实例 ===")
for p in inst["placed"]:
    if (p.get("part_name") or "") != "腿部_轴盖":
        continue
    t = np.asarray(p["T"], float)[:3, 3]
    print("  路径=%-40s  t=(%.2f, %.2f, %.2f)"
          % (" / ".join(p.get("ancestors") or [])[-40:], t[0], t[1], t[2]))

print()
print("=== 左腿 vs 右腿：同类零件 Y 平移差的中位数 ===")
byL, byR = {}, {}
for p in groups.get("腿部设计_左", []):
    byL.setdefault(p.get("part_name") or "?", []).append(np.asarray(p["T"], float)[:3, 3])
for p in groups.get("腿部设计_右", []):
    byR.setdefault(p.get("part_name") or "?", []).append(np.asarray(p["T"], float)[:3, 3])
dy = []
for nm in sorted(set(byL) & set(byR)):
    a = np.asarray(byL[nm], float); b = np.asarray(byR[nm], float)
    if a.shape == b.shape:
        dy.append((nm, float(np.median(a[:, 1] - b[:, 1])), a.shape[0]))
if dy:
    arr = np.asarray([d[1] for d in dy], float)
    print("  可比零件 %d 个, Δy 中位数=%.2f mm  (min %.2f / max %.2f)"
          % (len(dy), float(np.median(arr)), float(arr.min()), float(arr.max())))
    for nm, v, n in dy[:8]:
        print("    %-24s n=%d  Δy=%.2f" % (nm, n, v))
