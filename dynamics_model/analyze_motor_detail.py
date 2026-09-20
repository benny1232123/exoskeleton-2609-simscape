import win32com.client
import numpy as np
import math

sw = win32com.client.Dispatch('SldWorks.Application')
doc = sw.ActiveDoc

print('=== 深入分析电机和腿部子装配 ===\n')

# Get motor subassembly components
comps = doc.GetComponents(True)

for c in comps:
    name = c.Name2
    if '电机设计' in name:
        print(f'\n{"="*60}')
        print(f'{name}')
        print(f'{"="*60}')
        
        # Get transform
        try:
            xform = c.Transform2
            arr = xform.ArrayData
            print(f'Raw transform array ({len(arr)} values):')
            for i in range(0, min(16, len(arr)), 4):
                print(f'  [{arr[i]:10.6f} {arr[i+1]:10.6f} {arr[i+2]:10.6f} {arr[i+3]:10.6f}]')
        except Exception as e:
            print(f'Transform error: {e}')
        
        # Get sub-components
        children = c.GetChildren
        print(f'\nSub-components ({len(children)}):')
        
        # Find motor model and output shaft
        for ch in children:
            ch_name = ch.Name2
            if '测绘' in ch_name or '出轴' in ch_name or '轴' in ch_name:
                print(f'\n  >> {ch_name}')
                try:
                    chDoc = ch.GetModelDoc2
                    if chDoc:
                        print(f'    Type: {chDoc.GetType}')
                        
                        # Get features
                        feat = chDoc.FirstFeature
                        while feat:
                            ft = feat.GetTypeName2
                            fname = feat.Name
                            if 'Axis' in ft or 'axis' in fname.lower() or 'Cylind' in ft:
                                print(f'    Feature: {fname} ({ft})')
                            feat = feat.GetNextFeature
                except Exception as e:
                    print(f'    Error: {e}')
        
        # Also get direct children's transforms
        print(f'\n  Direct children transforms:')
        for ch in children[:5]:
            ch_name = ch.Name2
            if '测绘' in ch_name or '出轴' in ch_name:
                try:
                    ch_xform = ch.Transform2
                    ch_arr = ch_xform.ArrayData
                    print(f'\n  {ch_name}:')
                    for i in range(0, min(16, len(ch_arr)), 4):
                        print(f'    [{ch_arr[i]:10.6f} {ch_arr[i+1]:10.6f} {ch_arr[i+2]:10.6f} {ch_arr[i+3]:10.6f}]')
                except Exception as e:
                    print(f'  {ch_name} transform error: {e}')
