"""Check AXIS1_PLACEMENT format and STEP header units"""
import os, re, numpy as np

base = r'C:\Users\29408\Desktop\外骨骼'
stp = None
for f in os.listdir(base):
    if f.endswith('.stp'):
        stp = os.path.join(base, f)
        break

# 1. Check header for units
print("=== STEP Header (first 300 lines) ===")
with open(stp, 'r', errors='replace') as f:
    for i, line in enumerate(f):
        if i > 300:
            break
        if 'UNIT' in line.upper() or 'SCALE' in line.upper() or 'LENGTH' in line.upper():
            print(f"  Line {i}: {line.strip()[:200]}")

# 2. Check AXIS1_PLACEMENT format
print("\n=== AXIS1_PLACEMENT format ===")
count = 0
with open(stp, 'r', errors='replace') as f:
    for i, line in enumerate(f):
        if 'AXIS1_PLACEMENT' in line:
            count += 1
            if count <= 5:
                print(f"  Line {i}: {line.strip()[:300]}")

# 3. Check LINE entities (they connect CARTESIAN_POINTs)
print("\n=== LINE entity format ===")
count = 0
with open(stp, 'r', errors='replace') as f:
    for line in f:
        if line.strip().startswith('#') and '=LINE(' in line.upper():
            count += 1
            if count <= 3:
                print(f"  {line.strip()[:200]}")
            if count > 3:
                break

# 4. Also look for DIRECTION entities
print("\n=== DIRECTION format ===")
count = 0
with open(stp, 'r', errors='replace') as f:
    for line in f:
        if 'DIRECTION(' in line and '=DIRECTION(' in line:
            count += 1
            if count <= 5:
                print(f"  {line.strip()[:200]}")
            if count > 5:
                break

# 5. Extract ALL points again and analyze the spatial distribution more carefully
print("\n=== Detailed point cloud analysis ===")
points = []
pat = re.compile(r"CARTESIAN_POINT\('[^']*',\(([-+\d.Ee]+),([-+\d.Ee]+),([-+\d.Ee]+)\)\)")
with open(stp, 'r', errors='replace') as f:
    for line in f:
        m = pat.search(line)
        if m:
            try:
                x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
                points.append([x, y, z])
            except:
                pass

pts = np.array(points)
print(f"Total: {len(pts)} points")

# Remove outliers (99th percentile)
for axis, name in [(0, 'X'), (1, 'Y'), (2, 'Z')]:
    p01 = np.percentile(pts[:, axis], 1)
    p99 = np.percentile(pts[:, axis], 99)
    mask = (pts[:, axis] >= p01) & (pts[:, axis] <= p99)
    core = pts[mask]
    print(f"\n{name} core range (1-99%): [{core[:, axis].min():.4f}, {core[:, axis].max():.4f}] range={core[:, axis].max()-core[:, axis].min():.4f}")

# Show Z distribution of core points (after removing top/bottom 1%)
p01_z = np.percentile(pts[:, 2], 1)
p99_z = np.percentile(pts[:, 2], 99)
mask = (pts[:, 2] >= p01_z) & (pts[:, 2] <= p99_z)
core = pts[mask]
print(f"\nCore points: {len(core)}")
print(f"Core bounding box:")
for i, name in enumerate(['X', 'Y', 'Z']):
    print(f"  {name}: [{core[:, i].min():.4f}, {core[:, i].max():.4f}] range={core[:, i].max()-core[:, i].min():.4f}")

# Histogram of core Z values
z_hist, z_bins = np.histogram(core[:, 2], bins=50)
print("\nCore Z-axis distribution:")
for i in range(len(z_hist)):
    if z_hist[i] > 100:
        print(f"  Z=[{z_bins[i]:.2f}, {z_bins[i+1]:.2f}]: {z_hist[i]:>8d} points")

# Histogram of core X values
x_hist, x_bins = np.histogram(core[:, 0], bins=50)
print("\nCore X-axis distribution:")
for i in range(len(x_hist)):
    if x_hist[i] > 100:
        print(f"  X=[{x_bins[i]:.2f}, {x_bins[i+1]:.2f}]: {x_hist[i]:>8d} points")
