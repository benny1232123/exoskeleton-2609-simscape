"""
解码STP文件中的中文零件名称，提取完整装配结构
"""
import re
import os


def decode_stp_string(s):
    """解码STEP文件中的 \X2\...\X0\ Unicode编码"""
    def replace_match(m):
        hex_str = m.group(1)
        chars = []
        for i in range(0, len(hex_str), 4):
            chars.append(chr(int(hex_str[i:i+4], 16)))
        return ''.join(chars)
    return re.sub(r'\\X2\\([0-9A-F]+)\\X0\\', replace_match, s)


def extract_full_assembly(filepath):
    """提取完整装配体结构"""
    assembly_parts = []
    all_shapes = []
    
    with open(filepath, 'r', errors='replace') as f:
        for i, line in enumerate(f):
            decoded = decode_stp_string(line)
            
            # 提取 NEXT_ASSEMBLY_USAGE_OCCURRENCE (装配关系)
            # NAUO(id, name, id, parent_product, child_product, reference_designator, quantity)
            if 'NEXT_ASSEMBLY_USAGE_OCCURRENCE' in decoded:
                # 提取引号内的名称
                names = re.findall(r"'([^']*)'", decoded)
                if len(names) >= 3:
                    assembly_parts.append({
                        'line': i,
                        'occ_id': names[0],
                        'name': names[2] if len(names) > 2 else names[1],
                        'raw': decoded[:300]
                    })
            
            # 提取 ADVANCED_BREP_SHAPE_REPRESENTATION (零件形状)
            if 'ADVANCED_BREP_SHAPE_REPRESENTATION' in decoded:
                names = re.findall(r"'([^']*)'", decoded)
                if names:
                    all_shapes.append({
                        'line': i,
                        'name': names[0],
                        'raw': decoded[:300]
                    })
    
    return assembly_parts, all_shapes


def categorize_parts(parts):
    """按功能分类零件"""
    categories = {
        '腿部': [],      # 腿部结构件
        '背部': [],      # 背部结构件
        '电机': [],      # 电机相关
        '管夹': [],      # 管夹紧固件
        '紧固件': [],    # 螺丝螺母等
        '其他': []
    }
    
    for p in parts:
        name = p['name']
        if '腿部' in name:
            categories['腿部'].append(p)
        elif '背部' in name:
            categories['背部'].append(p)
        elif '电机' in name:
            categories['电机'].append(p)
        elif '管夹' in name:
            categories['管夹'].append(p)
        elif any(kw in name for kw in ['螺', '销', '垫', '平头']):
            categories['紧固件'].append(p)
        else:
            categories['其他'].append(p)
    
    return categories


def main():
    base = r'C:\Users\29408\Desktop\外骨骼'
    stp_file = None
    for f in os.listdir(base):
        if f.endswith('.stp') or f.endswith('.step'):
            stp_file = os.path.join(base, f)
            break
    
    if not stp_file:
        print("No STP file found!")
        return
    
    print(f"Parsing: {stp_file}")
    print(f"Size: {os.path.getsize(stp_file) / 1024 / 1024:.1f} MB\n")
    
    assembly_parts, all_shapes = extract_full_assembly(stp_file)
    
    # 解码所有名称
    print(f"=== Assembly Parts (NEXT_ASSEMBLY_USAGE_OCCURRENCE): {len(assembly_parts)} ===")
    decoded_parts = []
    for p in assembly_parts:
        name = decode_stp_string(p['name'])
        decoded_parts.append({'name': name, 'line': p['line']})
    
    # 按类别显示
    categories = categorize_parts(decoded_parts)
    
    for cat_name, cat_parts in categories.items():
        if cat_parts:
            print(f"\n--- {cat_name} ({len(cat_parts)} parts) ---")
            # 去重显示
            seen = set()
            for p in cat_parts:
                short_name = p['name']
                if short_name not in seen:
                    seen.add(short_name)
                    print(f"  {short_name}")
    
    print(f"\n=== Shape Representations: {len(all_shapes)} ===")
    decoded_shapes = []
    for s in all_shapes:
        name = decode_stp_string(s['name'])
        decoded_shapes.append(name)
    
    # 统计形状名称
    shape_counts = {}
    for name in decoded_shapes:
        shape_counts[name] = shape_counts.get(name, 0) + 1
    
    print("Shape names (count > 1):")
    for name, count in sorted(shape_counts.items(), key=lambda x: -x[1]):
        if count > 1:
            print(f"  {name}: x{count}")
    
    print("\nUnique shape names:")
    seen = set()
    for name in decoded_shapes:
        if name not in seen:
            seen.add(name)
            print(f"  {name}")


if __name__ == '__main__':
    main()
