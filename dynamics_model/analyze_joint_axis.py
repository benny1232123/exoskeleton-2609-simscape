import win32com.client
import numpy as np
import math

sw = win32com.client.Dispatch('SldWorks.Application')
doc = sw.ActiveDoc
print(f'Doc: {doc.GetPathName}')

comps = doc.GetComponents(True)

def get_transform_matrix(comp):
    """Extract 4x4 transform matrix from SolidWorks component"""
    try:
        xform = comp.Transform2
        arr = xform.ArrayData
        # SolidWorks returns row-major 4x4
        mat = np.array(arr[:16]).reshape(4,4)
        return mat
    except:
        return None

def get_rotation_angles(mat):
    """Extract Euler angles (ZYZ) from rotation part of 4x4 matrix"""
    R = mat[:3, :3]
    # ZYZ decomposition
    beta = math.acos(max(-1, min(1, R[2,2])))
    if abs(math.sin(beta)) > 1e-6:
        alpha = math.atan2(R[0,2]/math.sin(beta), R[1,2]/math.sin(beta))
        gamma = math.atan2(R[2,0]/math.sin(beta), -R[2,1]/math.sin(beta))
    else:
        alpha = math.atan2(R[0,1], R[0,0])
        gamma = 0
    return math.degrees(alpha), math.degrees(beta), math.degrees(gamma)

print('\n=== 电机设计 变换分析 ===')
motor_data = {}
for c in comps:
    name = c.Name2
    if '电机设计' in name:
        mat = get_transform_matrix(c)
        if mat is not None:
            pos = mat[:3, 3]
            R = mat[:3, :3]
            
            # Extract rotation angles
            alpha, beta, gamma = get_rotation_angles(mat)
            
            print(f'\n{name}:')
            print(f'  Position: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}')
            print(f'  Rotation (ZYZ): alpha={alpha:.1f}°, beta={beta:.1f}°, gamma={gamma:.1f}°')
            print(f'  Rotation matrix:')
            for row in R:
                print(f'    [{row[0]:10.4f} {row[1]:10.4f} {row[2]:10.4f}]')
            
            # The motor output axis in local frame is typically Z-axis
            # Transform to global
            local_z = np.array([0, 0, 1, 0])  # local Z-axis (motor shaft direction)
            global_axis = mat @ local_z
            global_axis = global_axis[:3] / np.linalg.norm(global_axis[:3])
            print(f'  Motor shaft direction (local Z -> global): [{global_axis[0]:.4f}, {global_axis[1]:.4f}, {global_axis[2]:.4f}]')
            
            motor_data[name] = {
                'position': pos,
                'axis': global_axis,
                'R': R
            }

print('\n=== 腿部设计 变换分析 ===')
for c in comps:
    name = c.Name2
    if '腿部设计' in name:
        mat = get_transform_matrix(c)
        if mat is not None:
            pos = mat[:3, 3]
            alpha, beta, gamma = get_rotation_angles(mat)
            print(f'\n{name}:')
            print(f'  Position: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}')
            print(f'  Rotation (ZYZ): alpha={alpha:.1f}°, beta={beta:.1f}°, gamma={gamma:.1f}°')

# Analyze joint axis relationship
print('\n=== 关节轴分析 ===')
left_motor = motor_data.get('电机设计_左-1')
right_motor = motor_data.get('电机设计_右-1')

if left_motor and right_motor:
    # Both motors should have similar axis directions (both are hip joints)
    left_axis = left_motor['axis']
    right_axis = right_motor['axis']
    
    print(f'Left motor axis:  [{left_axis[0]:.4f}, {left_axis[1]:.4f}, {left_axis[2]:.4f}]')
    print(f'Right motor axis: [{right_axis[0]:.4f}, {right_axis[1]:.4f}, {right_axis[2]:.4f}]')
    
    # Check if axes are parallel (both hip flexion/extension)
    dot = abs(np.dot(left_axis, right_axis))
    print(f'Axis parallelism (dot product): {dot:.4f}')
    
    # The hip flexion/extension axis should be approximately along the body's left-right direction
    # In SolidWorks coordinate system:
    # X = right, Y = up, Z = forward (or similar)
    # Hip flexion axis = X-axis (left-right)
    
    # Check angle with X-axis (left-right)
    x_axis = np.array([1, 0, 0])
    left_angle_x = math.degrees(math.acos(abs(np.dot(left_axis, x_axis))))
    right_angle_x = math.degrees(math.acos(abs(np.dot(right_axis, x_axis))))
    
    print(f'\nLeft motor axis vs X-axis (left-right): {left_angle_x:.1f}°')
    print(f'Right motor axis vs X-axis (left-right): {right_angle_x:.1f}°')
    
    # Check angle with Y-axis (up-down)
    y_axis = np.array([0, 1, 0])
    left_angle_y = math.degrees(math.acos(abs(np.dot(left_axis, y_axis))))
    right_angle_y = math.degrees(math.acos(abs(np.dot(right_axis, y_axis))))
    
    print(f'Left motor axis vs Y-axis (up-down): {left_angle_y:.1f}°')
    print(f'Right motor axis vs Y-axis (up-down): {right_angle_y:.1f}°')
    
    # Check angle with Z-axis (forward-back)
    z_axis = np.array([0, 0, 1])
    left_angle_z = math.degrees(math.acos(abs(np.dot(left_axis, z_axis))))
    right_angle_z = math.degrees(math.acos(abs(np.dot(right_axis, z_axis))))
    
    print(f'Left motor axis vs Z-axis (forward): {left_angle_z:.1f}°')
    print(f'Right motor axis vs Z-axis (forward): {right_angle_z:.1f}°')
    
    print('\n=== 结论 ===')
    if left_angle_x < 30 and right_angle_x < 30:
        print('电机轴接近X轴(左右方向) -> 支持髋屈伸模型')
    elif left_angle_y < 30 and right_angle_y < 30:
        print('电机轴接近Y轴(上下方向) -> 可能是髋外展/内收')
    elif left_angle_z < 30 and right_angle_z < 30:
        print('电机轴接近Z轴(前后方向) -> 可能是髋内旋/外旋')
    else:
        print(f'电机轴不完全对齐任何主轴 -> 需要更精确测量')
