"""
Comprehensive STEP file parser for extracting exoskeleton geometry.
Builds entity reference graph to map CARTESIAN_POINTs to products,
then computes per-component bounding boxes for link length estimation.
"""
import re
import os
import sys
from collections import defaultdict
import numpy as np


def decode_stp_string(s):
    """Decode NX \\X2\\...\\X0\\ Unicode encoding"""
    def replace_match(m):
        hex_str = m.group(1)
        chars = []
        for i in range(0, len(hex_str), 4):
            chars.append(chr(int(hex_str[i:i+4], 16)))
        return ''.join(chars)
    return re.sub(r'\\X2\\([0-9A-F]+)\\X0\\', replace_match, s)


def parse_step_file(filepath):
    """Parse STEP file into entity dict: id -> raw text"""
    entities = {}
    with open(filepath, 'r', errors='replace') as f:
        for line in f:
            line = line.strip()
            m = re.match(r'^#(\d+)\s*=\s*(.*)', line)
            if m:
                eid = int(m.group(1))
                entities[eid] = m.group(2)
    return entities


def build_reference_graph(entities):
    """Build forward reference graph: for each entity, find IDs it references"""
    refs = {}
    for eid, raw in entities.items():
        ref_ids = [int(x) for x in re.findall(r'#(\d+)', raw)]
        refs[eid] = ref_ids
    return refs


def build_reverse_graph(refs):
    """Build reverse reference graph: for each entity, find IDs that reference it"""
    rev = defaultdict(list)
    for eid, ref_list in refs.items():
        for ref in ref_list:
            rev[ref].append(eid)
    return rev


def find_products(entities):
    """Find all PRODUCT entities with decoded names"""
    products = {}
    for eid, raw in entities.items():
        if raw.upper().startswith('PRODUCT('):
            decoded = decode_stp_string(raw)
            # Extract name: PRODUCT('name',...)
            m = re.match(r"PRODUCT\(\s*'([^']*)'", decoded)
            if m:
                products[eid] = m.group(1)
            else:
                m = re.match(r'PRODUCT\(\s*"([^"]*)"', decoded)
                if m:
                    products[eid] = m.group(1)
    return products


def find_product_definitions(entities):
    """Find PRODUCT_DEFINITION entities and their shapes"""
    pdefs = {}
    for eid, raw in entities.items():
        if raw.upper().startswith('PRODUCT_DEFINITION('):
            decoded = decode_stp_string(raw)
            pdefs[eid] = decoded
    return pdefs


def find_assembly_usage(entities):
    """Find NEXT_ASSEMBLY_USAGE_OCCURRENCE (parent-child assembly relationships)"""
    naus = {}
    for eid, raw in entities.items():
        if raw.upper().startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE'):
            decoded = decode_stp_string(raw)
            ref_ids = [int(x) for x in re.findall(r'#(\d+)', decoded)]
            naus[eid] = {'raw': decoded, 'refs': ref_ids}
    return naus


def find_cartesian_points(entities):
    """Extract all CARTESIAN_POINT coordinates - handle multiple NX formats"""
    points = {}
    # Format 1: CARTESIAN_POINT('name',(x,y,z))
    # Format 2: CARTESIAN_POINT((x,y,z))
    # Format 3: multi-line with coordinates
    pat = re.compile(
        r"#(\d+)=CARTESIAN_POINT\(\s*['\"][^)]*['\"][\s,]*\(\s*([-+\d.Ee]+)\s*,\s*([-+\d.Ee]+)\s*,\s*([-+\d.Ee]+)\s*\)"
    )
    pat2 = re.compile(
        r"#(\d+)=CARTESIAN_POINT\(\s*\(\s*([-+\d.Ee]+)\s*,\s*([-+\d.Ee]+)\s*,\s*([-+\d.Ee]+)\s*\)"
    )
    
    for eid, raw in entities.items():
        m = pat.search(raw)
        if not m:
            m = pat2.search(raw)
        if m:
            try:
                x = float(m.group(2))
                y = float(m.group(3))
                z = float(m.group(4))
                points[eid] = np.array([x, y, z])
            except:
                pass
    return points


def find_shape_representations(entities):
    """Find ADVANCED_BREP_SHAPE_REPRESENTATION and related shape entities"""
    shapes = {}
    for eid, raw in entities.items():
        decoded = raw.upper()
        if 'ADVANCED_BREP_SHAPE_REPRESENTATION' in decoded:
            ref_ids = [int(x) for x in re.findall(r'#(\d+)', raw)]
            shapes[eid] = ref_ids
    return shapes


def find_manifold_solids(entities):
    """Find MANIFOLD_SOLID_BREP entities"""
    solids = {}
    for eid, raw in entities.items():
        if raw.upper().startswith('MANIFOLD_SOLID_BREP'):
            decoded = decode_stp_string(raw)
            ref_ids = [int(x) for x in re.findall(r'#(\d+)', decoded)]
            solids[eid] = ref_ids
    return solids


def find_axis_placements(entities, points):
    """Find AXIS1_PLACEMENT entities"""
    axes = []
    for eid, raw in entities.items():
        if raw.upper().startswith('AXIS1_PLACEMENT'):
            decoded = decode_stp_string(raw)
            ref_ids = [int(x) for x in re.findall(r'#(\d+)', decoded)]
            loc = None
            direction = None
            if len(ref_ids) >= 2:
                loc = points.get(ref_ids[0], None)
            if len(ref_ids) >= 3:
                direction = points.get(ref_ids[1], None)
            axes.append({
                'id': eid, 
                'refs': ref_ids,
                'location': loc, 
                'direction': direction,
                'raw': decoded[:200]
            })
    return axes


def trace_to_product(eid, rev_graph, entities, depth=0, max_depth=10):
    """Trace entity back to its owning PRODUCT through reverse references"""
    if depth > max_depth:
        return None
    
    for parent_id in rev_graph.get(eid, []):
        raw = entities.get(parent_id, '')
        if raw.upper().startswith('PRODUCT('):
            return parent_id
        result = trace_to_product(parent_id, rev_graph, entities, depth+1, max_depth)
        if result:
            return result
    return None


def assign_points_to_products(points, entities, rev_graph):
    """For each CARTESIAN_POINT, find which PRODUCT it belongs to"""
    point_products = defaultdict(list)
    for pid in points:
        product_id = trace_to_product(pid, rev_graph, entities)
        if product_id:
            point_products[product_id].append(points[pid])
    return point_products


def compute_bounding_boxes(point_products):
    """Compute bounding box for each product"""
    bboxes = {}
    for pid, pts in point_products.items():
        if len(pts) < 2:
            continue
        pts_arr = np.array(pts)
        bbox = {
            'min': pts_arr.min(axis=0),
            'max': pts_arr.max(axis=0),
            'size': pts_arr.max(axis=0) - pts_arr.min(axis=0),
            'center': pts_arr.mean(axis=0),
            'n_points': len(pts)
        }
        bboxes[pid] = bbox
    return bboxes


def identify_subsystems(products):
    """Group products into subsystems based on Chinese naming"""
    subsystems = defaultdict(list)
    for pid, name in products.items():
        decoded_name = decode_stp_string(name)
        # Normalize
        if '腿部' in decoded_name:
            subsystems['腿部(leg)'].append((pid, decoded_name))
        elif '背部' in decoded_name:
            subsystems['背部(back)'].append((pid, decoded_name))
        elif '电机' in decoded_name:
            subsystems['电机(motor)'].append((pid, decoded_name))
        elif '管夹' in decoded_name:
            subsystems['管夹(pipe_clamp)'].append((pid, decoded_name))
        elif '面板' in decoded_name:
            subsystems['面板(panel)'].append((pid, decoded_name))
        elif '电池' in decoded_name or '18650' in decoded_name:
            subsystems['电池(battery)'].append((pid, decoded_name))
        elif '主控' in decoded_name:
            subsystems['主控(main_control)'].append((pid, decoded_name))
        elif '绑缚' in decoded_name:
            subsystems['绑缚(strap)'].append((pid, decoded_name))
        else:
            # Check for fastener patterns
            if any(f in decoded_name for f in ['M2', 'M3', 'M4', '螺丝', '螺母', '顶丝']):
                subsystems['紧固件(fasteners)'].append((pid, decoded_name))
            elif any(f in decoded_name for f in ['EC800', 'PCB', 'USB', 'LED', 'SIM', 'LQFP']):
                subsystems['电子(electronics)'].append((pid, decoded_name))
            else:
                subsystems['其他(other)'].append((pid, decoded_name))
    return dict(subsystems)


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
    print(f"Size: {os.path.getsize(stp_file) / 1024 / 1024:.1f} MB")
    
    # Phase 1: Parse entities
    print("\n=== Phase 1: Parsing STEP entities ===")
    entities = parse_step_file(stp_file)
    print(f"Total entities: {len(entities)}")
    
    # Phase 2: Build reference graph
    print("\n=== Phase 2: Building reference graph ===")
    refs = build_reference_graph(entities)
    rev = build_reverse_graph(refs)
    
    # Phase 3: Find products
    print("\n=== Phase 3: Finding products ===")
    products = find_products(entities)
    print(f"Total products: {len(products)}")
    
    # Phase 4: Find cartesian points
    print("\n=== Phase 4: Finding Cartesian points ===")
    points = find_cartesian_points(entities)
    print(f"Total points: {len(points)}")
    
    # Phase 5: Find axis placements
    print("\n=== Phase 5: Finding axis placements ===")
    axes = find_axis_placements(entities, points)
    print(f"Total axis placements: {len(axes)}")
    for ax in axes:
        if ax['location'] is not None:
            loc = ax['location']
            # Convert from mm to meters if needed (check scale)
            print(f"  Axis #{ax['id']}: loc=({loc[0]:.4f}, {loc[1]:.4f}, {loc[2]:.4f})")
        if ax['direction'] is not None:
            d = ax['direction']
            print(f"    direction=({d[0]:.4f}, {d[1]:.4f}, {d[2]:.4f})")
    
    # Phase 6: Assign points to products
    print("\n=== Phase 6: Assigning points to products (this may take a while) ===")
    point_products = assign_points_to_products(points, entities, rev)
    print(f"Products with geometry: {len(point_products)}")
    
    # Phase 7: Compute bounding boxes
    print("\n=== Phase 7: Computing bounding boxes ===")
    bboxes = compute_bounding_boxes(point_products)
    
    # Phase 8: Identify subsystems
    print("\n=== Phase 8: Identifying subsystems ===")
    subsystems = identify_subsystems(products)
    for sys_name, parts in sorted(subsystems.items()):
        print(f"\n  {sys_name}: {len(parts)} parts")
        # Show bounding box info for structural parts
        for pid, pname in parts[:10]:
            if pid in bboxes:
                bb = bboxes[pid]
                size_mm = bb['size']
                print(f"    {pname}: size=({size_mm[0]:.1f}, {size_mm[1]:.1f}, {size_mm[2]:.1f}) [units], {bb['n_points']} pts")
        if len(parts) > 10:
            print(f"    ... and {len(parts)-10} more parts")
    
    # Phase 9: Analyze leg structure specifically
    print("\n=== Phase 9: Leg structure analysis ===")
    if '腿部(leg)' in subsystems:
        for pid, pname in subsystems['腿部(leg)']:
            decoded_name = decode_stp_string(pname)
            if '腿杆' in decoded_name:
                if pid in bboxes:
                    bb = bboxes[pid]
                    print(f"\n  THIGH/SHANK LINK: {decoded_name}")
                    print(f"    Bounding box size: {bb['size']}")
                    print(f"    Bounding box min:  {bb['min']}")
                    print(f"    Bounding box max:  {bb['max']}")
                    print(f"    Points: {bb['n_points']}")
    
    # Phase 10: Motor analysis
    print("\n=== Phase 10: Motor structure analysis ===")
    if '电机(motor)' in subsystems:
        for pid, pname in subsystems['电机(motor)']:
            decoded_name = decode_stp_string(pname)
            if pid in bboxes:
                bb = bboxes[pid]
                print(f"\n  MOTOR PART: {decoded_name}")
                print(f"    Bounding box size: {bb['size']}")
                print(f"    Points: {bb['n_points']}")
    
    # Summary: key dimensions
    print("\n=== SUMMARY: Key Dimensions ===")
    print("(All dimensions are in STEP file units - likely mm)")
    print("\nTo determine actual units, check the STEP file header for:")
    print("  GLOBAL_UNIT_ASSIGNED_CONTEXT or LENGTH_UNIT")
    
    # Try to find unit info
    print("\n=== Checking STEP header for units ===")
    with open(stp_file, 'r', errors='replace') as f:
        for i, line in enumerate(f):
            if i > 200:
                break
            if 'UNIT' in line.upper():
                decoded = decode_stp_string(line.strip())
                if 'UNIT' in decoded.upper():
                    print(f"  Line {i}: {decoded[:200]}")
            if 'SCALER' in line.upper() or 'SCALE' in line.upper():
                decoded = decode_stp_string(line.strip())
                print(f"  Line {i}: {decoded[:200]}")


if __name__ == '__main__':
    main()
