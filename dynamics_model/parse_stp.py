"""
解析STP文件 - 提取零件信息和几何参数
用于构建基于真实CAD的动力学模型
"""
import re
import os
import sys


def read_stp_header(filepath, max_lines=200):
    """读取STP文件头部"""
    with open(filepath, 'r', errors='replace') as f:
        lines = []
        for i, line in enumerate(f):
            if i >= max_lines:
                break
            lines.append(line.rstrip())
    return lines


def extract_product_info(filepath):
    """提取产品信息"""
    products = []
    assemblies = []
    
    with open(filepath, 'r', errors='replace') as f:
        content = f.read()
    
    # 提取 PRODUCT 实体
    # PRODUCT(#id, 'name', 'description', 'formation', 'frame_of_ref)
    product_pattern = re.compile(r'PRODUCT\((\d+),\s*[\'"]([^"\']+)[\'"],\s*[\'"]*[^)]*\)')
    for match in product_pattern.finditer(content):
        products.append({
            'id': int(match.group(1)),
            'name': match.group(2)
        })
    
    # 提取 PRODUCT_DEFINITION
    # PRODUCT_DEFINITION('design', ..., PRODUCT_DEFINITION_SHAPE(...))
    # 用于关联零件和形状
    
    # 提取形体信息
    # SHAPE_DEFINITION_REPRESENTATION
    # MANIFOLD_SOLID_BREP
    # ADVANCED_BREP_SHAPE_REPRESENTATION
    
    # 统计主要实体类型
    entity_counts = {}
    entity_pattern = re.compile(r'^([A-Z_]+)\(')
    for line in content.split('\n'):
        m = entity_pattern.match(line.strip())
        if m:
            etype = m.group(1)
            entity_counts[etype] = entity_counts.get(etype, 0) + 1
    
    return products, entity_counts


def find_joint_features(filepath):
    """查找可能的关节特征"""
    joints = []
    
    with open(filepath, 'r', errors='replace') as f:
        content = f.read()
    
    # 查找圆柱面（可能的关节轴）
    # CYLINDRICAL_SURFACE
    cyl_pattern = re.compile(r'CYLINDRICAL_SURFACE\((\d+),\s*\(')
    cyl_matches = cyl_pattern.findall(content)
    
    # 查找旋转体
    # SURFACE_OF_REVOLUTION
    rev_pattern = re.compile(r'SURFACE_OF_REVOLUTION\(')
    rev_matches = rev_pattern.findall(content)
    
    # 查找轴线
    # LINE
    # AXIS1_PLACEMENT
    axis_pattern = re.compile(r'AXIS1_PLACEMENT\(')
    axis_matches = axis_pattern.findall(content)
    
    # 查找坐标系
    # CARTESIAN_POINT
    point_pattern = re.compile(r'CARTESIAN_POINT\((\d+),\s*\(([^)]+)\)\)')
    points = []
    for m in point_pattern.finditer(content):
        coords = [float(x.strip()) for x in m.group(2).split(',')]
        if len(coords) == 3:
            points.append({
                'id': int(m.group(1)),
                'xyz': coords
            })
    
    return {
        'cylindrical_surfaces': len(cyl_matches),
        'revolution_surfaces': len(rev_matches),
        'axis_placements': len(axis_matches),
        'cartesian_points': len(points),
        'sample_points': points[:20] if points else []
    }


def get_file_size_info(filepath):
    """获取文件大小信息"""
    size = os.path.getsize(filepath)
    with open(filepath, 'r', errors='replace') as f:
        line_count = sum(1 for _ in f)
    return size, line_count


def main():
    stp_file = r'C:\Users\29408\Desktop\外骨骼\林-I\髋关节外骨骼机器人开发平台V0_1_1.stp'
    
    if not os.path.exists(stp_file):
        print(f"File not found: {stp_file}")
        # 尝试其他路径
        base = r'C:\Users\29408\Desktop\外骨骼'
        for f in os.listdir(base):
            if f.endswith('.stp') or f.endswith('.step'):
                stp_file = os.path.join(base, f)
                print(f"Found: {stp_file}")
                break
    
    print(f"\n=== STP File Analysis ===")
    print(f"File: {stp_file}")
    
    if not os.path.exists(stp_file):
        print("STP file not found!")
        # List available files
        base = r'C:\Users\29408\Desktop\外骨骼'
        print("\nAvailable files:")
        for f in os.listdir(base):
            fp = os.path.join(base, f)
            if os.path.isdir(fp):
                print(f"  [DIR] {f}")
                for sf in os.listdir(fp):
                    print(f"        {sf}")
            else:
                print(f"  {f} ({os.path.getsize(fp) / 1024 / 1024:.1f} MB)")
        return
    
    size, lines = get_file_size_info(stp_file)
    print(f"Size: {size / 1024 / 1024:.1f} MB, Lines: {lines}")
    
    # 头部
    print("\n--- Header (first 30 lines) ---")
    header = read_stp_header(stp_file, 30)
    for i, line in enumerate(header):
        print(f"  {i:3d}: {line[:120]}")
    
    # 产品信息
    print("\n--- Extracting Product Info ---")
    products, entity_counts = extract_product_info(stp_file)
    print(f"Products found: {len(products)}")
    for p in products[:30]:
        print(f"  #{p['id']}: {p['name']}")
    
    # 实体统计
    print("\n--- Entity Type Counts (top 20) ---")
    sorted_entities = sorted(entity_counts.items(), key=lambda x: -x[1])
    for etype, count in sorted_entities[:20]:
        print(f"  {etype}: {count}")
    
    # 关节特征
    print("\n--- Joint Features ---")
    joints = find_joint_features(stp_file)
    print(f"  Cylindrical surfaces: {joints['cylindrical_surfaces']}")
    print(f"  Revolution surfaces: {joints['revolution_surfaces']}")
    print(f"  Axis placements: {joints['axis_placements']}")
    print(f"  Cartesian points: {joints['cartesian_points']}")
    
    print("\n  Sample Cartesian Points (first 10):")
    for pt in joints['sample_points'][:10]:
        print(f"    #{pt['id']}: ({pt['xyz'][0]:.4f}, {pt['xyz'][1]:.4f}, {pt['xyz'][2]:.4f})")


if __name__ == '__main__':
    main()
