import win32com.client
import numpy as np

sw = win32com.client.Dispatch('SldWorks.Application')
doc = sw.ActiveDoc
print(f'Doc: {doc.GetPathName}')

comps = doc.GetComponents(True)

for c in comps:
    name = c.Name2
    print(f'\n=== {name} ===')
    
    # Position
    try:
        pos = c.GetPosition
        print(f'  Position: {pos}')
    except Exception as e:
        print(f'  Position error: {e}')
    
    # Transform
    try:
        xform = c.Transform2
        arr = xform.ArrayData
        # 4x4 matrix stored as 16 values
        mat = np.array(arr[:16]).reshape(4,4)
        print(f'  Transform matrix:')
        for row in mat:
            print(f'    [{row[0]:10.4f} {row[1]:10.4f} {row[2]:10.4f} {row[3]:10.4f}]')
    except Exception as e:
        print(f'  Transform error: {e}')
    
    # Get sub-components (children)
    try:
        children = c.GetChildren
        if children:
            print(f'  Children ({len(children)}):')
            for ch in children[:20]:
                print(f'    - {ch.Name2}')
    except Exception as e:
        print(f'  Children error: {e}')
    
    # Get component model features
    try:
        compDoc = c.GetModelDoc2
        if compDoc:
            print(f'  Model type: {compDoc.GetType}')
            feat = compDoc.FirstFeature
            feat_count = 0
            axes = []
            while feat and feat_count < 1000:
                ft = feat.GetTypeName2
                fname = feat.Name
                if 'Axis' in ft or 'axis' in fname.lower():
                    axes.append(f'{fname} ({ft})')
                feat = feat.GetNextFeature
                feat_count += 1
            if axes:
                print(f'  Axes: {axes}')
            else:
                print(f'  Features scanned: {feat_count}, no axes found')
    except Exception as e:
        print(f'  ModelDoc error: {e}')
