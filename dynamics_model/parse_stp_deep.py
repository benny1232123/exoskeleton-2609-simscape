"""
深度解析STP文件 - 提取NX装配体结构
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


def search_stp(filepath, max_lines=500000):
    """搜索STP文件中的关键信息"""
    product_names = []
    assembly_usage = []
    shape_rep = []
    next_assembly = []
    product_def = []
    placements = []
    
    with open(filepath, 'r', errors='replace') as f:
        for i, line in enumerate(f):
            if i >= max_lines:
                break
            line = line.strip()
            
            # 查找产品名称 - NX格式
            if 'PRODUCT(' in line or 'product(' in line:
                decoded = decode_stp_string(line)
                product_names.append((i, decoded[:200]))
            
            # 查找装配使用关系
            if 'NEXT_ASSEMBLY_USAGE_OCCURRENCE' in line:
                assembly_usage.append((i, line[:200]))
            
            # 查找产品定义
            if 'PRODUCT_DEFINITION(' in line:
                product_def.append((i, line[:200]))
                
            # 查找轴线放置
            if 'AXIS1_PLACEMENT' in line:
                decoded = decode_stp_string(line)
                placements.append((i, decoded[:200]))
    
    return {
        'products': product_names,
        'assembly_usage': assembly_usage,
        'product_def': product_def,
        'placements': placements,
        'total_lines_scanned': max_lines
    }


def search_full_file(filepath, patterns):
    """全文件搜索特定模式"""
    results = {p: [] for p in patterns}
    with open(filepath, 'r', errors='replace') as f:
        for i, line in enumerate(f):
            decoded = decode_stp_string(line)
            for p in patterns:
                if p in decoded:
                    results[p].append((i, decoded[:250]))
    return results


def main():
    stp_file = r'C:\Users\29408\Desktop\外骨骼\"林-I"髋关节外骨骼机器人开发平台V0_1_1.stp'
    
    # Find actual file
    base = r'C:\Users\29408\Desktop\外骨骼'
    for f in os.listdir(base):
        if f.endswith('.stp') or f.endswith('.step'):
            stp_file = os.path.join(base, f)
            break
    
    print(f"Parsing: {stp_file}")
    print(f"Size: {os.path.getsize(stp_file) / 1024 / 1024:.1f} MB")
    
    # 先快速扫描前50万行
    print("\n=== Phase 1: Quick scan (first 500K lines) ===")
    info = search_stp(stp_file, 500000)
    
    print(f"\nProducts found: {len(info['products'])}")
    for linenum, name in info['products'][:20]:
        print(f"  Line {linenum}: {name}")
    
    print(f"\nAssembly usage (NEXT_ASSEMBLY): {len(info['assembly_usage'])}")
    for linenum, name in info['assembly_usage'][:10]:
        print(f"  Line {linenum}: {name}")
    
    print(f"\nProduct definitions: {len(info['product_def'])}")
    for linenum, name in info['product_def'][:10]:
        print(f"  Line {linenum}: {name}")
    
    print(f"\nAxis placements: {len(info['placements'])}")
    for linenum, name in info['placements'][:10]:
        decoded = decode_stp_string(name)
        print(f"  Line {linenum}: {decoded}")
    
    # 全文件搜索关键模式
    print("\n=== Phase 2: Full file keyword search ===")
    keywords = ['MANIFOLD_SOLID_BREP', 'ADVANCED_BREP_SHAPE_REPRESENTATION', 
                'SHAPE_DEFINITION_REPRESENTATION', 'GEOMETRIC_ITEM_SPECIFIC_USAGE',
                'PRODUCT_DEFINITION_SHAPE', 'NEXT_ASSEMBLY_USAGE_OCCURRENCE',
                'CONTEXT_DEPENDENT_SHAPE_REPRESENTATION']
    
    # 用快速计数
    counts = {}
    with open(stp_file, 'r', errors='replace') as f:
        for i, line in enumerate(f):
            decoded = decode_stp_string(line)
            for kw in keywords:
                if kw in decoded:
                    counts[kw] = counts.get(kw, 0) + 1
    
    print("Entity counts:")
    for kw, c in sorted(counts.items(), key=lambda x: -x[1]):
        print(f"  {kw}: {c}")
    
    # 搜索中文部件名
    print("\n=== Phase 3: Search for Chinese part names ===")
    chinese_parts = []
    with open(stp_file, 'r', errors='replace') as f:
        for i, line in enumerate(f):
            if '\\X2\\' in line:
                decoded = decode_stp_string(line)
                if 'PRODUCT' in decoded or 'product' in decoded:
                    chinese_parts.append((i, decoded[:300]))
                # 也搜索形状名称
                if 'ADVANCED_BREP_SHAPE' in decoded:
                    chinese_parts.append((i, decoded[:300]))
    
    print(f"Chinese encoded entries with PRODUCT/SHAPE: {len(chinese_parts)}")
    for linenum, text in chinese_parts[:30]:
        print(f"  Line {linenum}: {text}")


if __name__ == '__main__':
    main()
