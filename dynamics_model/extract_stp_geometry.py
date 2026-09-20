"""
提取STP文件中的关键结构信息
- 质量属性 (MASS_PROPERTIES)
- 几何尺寸 (通过BREP分析)
- 关节位置 (AXIS1_PLACEMENT坐标)
- 结构件尺寸
"""
import re
import os
import numpy as np


def decode_stp_string(s):
    def replace_match(m):
        hex_str = m.group(1)
        chars = []
        for i in range(0, len(hex_str), 4):
            chars.append(chr(int(hex_str[i:i+4], 16)))
        return ''.join(chars)
    return re.sub(r'\\X2\\([0-9A-F]+)\\X0\\', replace_match, s)


def extract_cartesian_points(filepath):
    """提取所有CARTESIAN_POINT坐标"""
    points = {}
    pattern = re.compile(r'#(\d+)=CARTESIAN_POINT\(\s*[\'"]*[^)]*\'?\s*,\s*\(\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*\)\s*\)')
    
    with open(filepath, 'r', errors='replace') as f:
        for line in f:
            m = pattern.search(line)
            if m:
                pid = int(m.group(1))
                x, y, z = float(m.group(2)), float(m.group(3)), float(m.group(4))
                points[pid] = np.array([x, y, z])
    return points


def extract_axis_placements(filepath, points):
    """提取AXIS1_PLACEMENT - 关节轴位置和方向"""
    axes = []
    # AXIS1_PLACEMENT(#id, #location, #direction)
    # 或 AXIS1_PLACEMENT(name, #location, #direction)
    pattern = re.compile(r'#(\d+)=AXIS1_PLACEMENT\(\s*\(\s*\)\s*,\s*#(\d+)\s*,\s*#(\d+)\s*\)')
    
    with open(filepath, 'r', errors='replace') as f:
        for line in f:
            m = pattern.search(line)
            if m:
                axis_id = int(m.group(1))
                loc_id = int(m.group(2))
                dir_id = int(m.group(3))
                loc = points.get(loc_id, None)
                axes.append({'id': axis_id, 'loc_id': loc_id, 'dir_id': dir_id, 'location': loc})
    return axes


def extract_directions(filepath):
    """提取DIRECTION向量"""
    dirs = {}
    pattern = re.compile(r'#(\d+)=DIRECTION\(\s*\(\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*\)\s*\)')
    
    with open(filepath, 'r', errors='replace') as f:
        for line in f:
            m = pattern.search(line)
            if m:
                did = int(m.group(1))
                dx, dy, dz = float(m.group(2)), float(m.group(3)), float(m.group(4))
                dirs[did] = np.array([dx, dy, dz])
    return dirs


def extract_mass_properties(filepath):
    """提取MASS_PROPERTIES（如果存在）"""
    mass_props = []
    pattern = re.compile(r'MASS_PROPERTIES\(([^)]+)\)')
    
    with open(filepath, 'r', errors='replace') as f:
        for i, line in enumerate(f):
            m = pattern.search(line)
            if m:
                mass_props.append({'line': i, 'raw': m.group(0)[:200]})
    return mass_props


def extract_structural_parts(filepath):
    """提取关键结构件的BREP信息"""
    parts_info = []
    
    key_parts = ['腿部_腿杆', '腿部_滑轨', '背部_主机架', '背部_电池仓机架',
                 '电机_电机底架', '电机_轴', '电机_出轴', '管夹', '面板组件',
                 '装配_电池仓', '主控制件', '背部_主控板']
    
    with open(filepath, 'r', errors='replace') as f:
        for i, line in enumerate(f):
            decoded = decode_stp_string(line)
            for part_name in key_parts:
                if part_name in decoded:
                    parts_info.append({'line': i, 'name': part_name, 'raw': decoded[:300]})
    return parts_info


def analyze_point_cloud(points):
    """分析点云，识别主要结构"""
    if not points:
        return
    
    all_coords = np.array(list(points.values()))
    
    print(f"Total points: {len(points)}")
    print(f"Bounding box:")
    print(f"  X: [{all_coords[:, 0].min():.4f}, {all_coords[:, 0].max():.4f}] range: {all_coords[:, 0].max() - all_coords[:, 0].min():.4f} m")
    print(f"  Y: [{all_coords[:, 1].min():.4f}, {all_coords[:, 1].max():.4f}] range: {all_coords[:, 1].max() - all_coords[:, 1].min():.4f} m")
    print(f"  Z: [{all_coords[:, 2].min():.4f}, {all_coords[:, 2].max():.4f}] range: {all_coords[:, 2].max() - all_coords[:, 2].min():.4f} m")
    
    # 查找远离中心的点群（可能对应不同部件）
    center = all_coords.mean(axis=0)
    distances = np.linalg.norm(all_coords - center, axis=1)
    
    # 按Z坐标分层
    z_vals = all_coords[:, 2]
    z_hist, z_bins = np.histogram(z_vals, bins=50)
    
    print(f"\nZ-axis distribution:")
    for i in range(len(z_hist)):
        if z_hist[i] > 10:
            print(f"  Z=[{z_bins[i]:.3f}, {z_bins[i+1]:.3f}]: {z_hist[i]} points")


def main():
    base = r'C:\Users\29408\Desktop\外骨骼'
    stp_file = None
    for f in os.listdir(base):
        if f.endswith('.stp') or f.endswith('.step'):
            stp_file = os.path.join(base, f)
            break
    
    print(f"Analyzing: {stp_file}\n")
    
    # 1. 提取坐标点
    print("=== Phase 1: Extracting Cartesian Points ===")
    points = extract_cartesian_points(stp_file)
    print(f"Found {len(points)} Cartesian points")
    analyze_point_cloud(points)
    
    # 2. 提取方向向量
    print("\n=== Phase 2: Extracting Directions ===")
    dirs = extract_directions(stp_file)
    print(f"Found {len(dirs)} direction vectors")
    
    # 3. 提取轴线
    print("\n=== Phase 3: Extracting Axis Placements ===")
    axes = extract_axis_placements(stp_file, points)
    print(f"Found {len(axes)} axis placements")
    for ax in axes:
        if ax['location'] is not None:
            print(f"  Axis #{ax['id']}: location=({ax['location'][0]:.4f}, {ax['location'][1]:.4f}, {ax['location'][2]:.4f})")
    
    # 4. 提取质量属性
    print("\n=== Phase 4: Extracting Mass Properties ===")
    mass_props = extract_mass_properties(stp_file)
    print(f"Found {len(mass_props)} MASS_PROPERTIES entries")
    for mp in mass_props[:10]:
        print(f"  Line {mp['line']}: {mp['raw']}")
    
    # 5. 提取结构件
    print("\n=== Phase 5: Key Structural Parts ===")
    parts = extract_structural_parts(stp_file)
    for p in parts[:20]:
        print(f"  Line {p['line']}: {p['name']}")
        # 尝试从BREP中提取几何信息


if __name__ == '__main__':
    main()
