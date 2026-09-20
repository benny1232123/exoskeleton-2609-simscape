import win32com.client
import numpy as np

sw = win32com.client.Dispatch('SldWorks.Application')
doc = sw.ActiveDoc

print('=== 直接测量电机输出轴方向 ===\n')

# Strategy: open each motor subassembly and measure the output shaft axis
comps = doc.GetComponents(True)

# Find motor subassemblies
for c in comps:
    name = c.Name2
    if '电机设计' in name:
        print(f'\n--- {name} ---')
        
        # Get the subassembly model
        try:
            compDoc = c.GetModelDoc2
            if compDoc:
                print(f'Type: {compDoc.GetType}')
                
                # Get all components in the subassembly
                subComps = compDoc.GetComponents(False)
                print(f'Total sub-components: {len(subComps)}')
                
                for sc in subComps:
                    scName = sc.Name2
                    if '测绘' in scName or '出轴' in scName:
                        print(f'\n  Found: {scName}')
                        
                        # Get the part model
                        try:
                            partDoc = sc.GetModelDoc2
                            if partDoc:
                                print(f'  Part type: {partDoc.GetType}')
                                
                                # Get features
                                feat = partDoc.FirstFeature
                                while feat:
                                    ft = feat.GetTypeName2
                                    fname = feat.Name
                                    
                                    # Look for cylindrical faces or axes
                                    if 'Cylind' in ft or 'Axis' in ft:
                                        print(f'  Feature: {fname} ({ft})')
                                        
                                        # Try to get face geometry
                                        try:
                                            face = feat.GetFace
                                            if face:
                                                # Get face type
                                                faceType = face.Type
                                                print(f'    Face type: {faceType}')
                                                
                                                # For cylindrical face, get axis
                                                if faceType == 2:  # CYLINDER
                                                    # Get cylinder parameters
                                                    params = face.CylinderParams
                                                    print(f'    Cylinder params: {params}')
                                        except Exception as e:
                                            print(f'    Face error: {e}')
                                    
                                    feat = feat.GetNextFeature
                        except Exception as e:
                            print(f'  Part error: {e}')
        except Exception as e:
            print(f'Subassembly error: {e}')

# Alternative: use measurement tool
print('\n\n=== 尝试使用测量工具 ===')
try:
    # Create measurement object
    meas = sw.GetMeasureUtility
    print(f'MeasureUtility: {meas}')
except Exception as e:
    print(f'MeasureUtility error: {e}')

# Try to get the motor output shaft from the leg subassembly
print('\n\n=== 从腿部子装配测量 ===')
for c in comps:
    name = c.Name2
    if '腿部设计' in name:
        print(f'\n--- {name} ---')
        try:
            compDoc = c.GetModelDoc2
            if compDoc:
                subComps = compDoc.GetComponents(False)
                for sc in subComps:
                    scName = sc.Name2
                    if '出轴' in scName or '轴' in scName:
                        print(f'  Found: {scName}')
                        try:
                            partDoc = sc.GetModelDoc2
                            if partDoc:
                                # Get all faces
                                body = partDoc.GetBodies2(0, True)  # Solid body
                                if body:
                                    for b in body:
                                        faces = b.GetFaces()
                                        for f in faces:
                                            if f.Type == 2:  # Cylinder
                                                params = f.CylinderParams
                                                print(f'    Cylinder face: {params}')
                        except Exception as e:
                            print(f'    Error: {e}')
        except Exception as e:
            print(f'Error: {e}')
