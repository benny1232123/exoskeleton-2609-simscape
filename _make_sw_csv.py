# -*- coding: utf-8 -*-
"""
从本机 STP 的几何反算结果，生成一份「SolidWorks 质量属性导出格式」的 CSV 样例。

用途
----
1. 给用户一个**格式正确、可直接跑**的样例，MATLAB 侧 exo2609.params_from_csv
   读它就能复现 M / G_amp —— 形成 gmsh -> CSV -> MATLAB 的闭环校验。
2. 提供**质量交叉核对基准**：用户在 SolidWorks 里做完同样的分组后，
   应得到同样的 m / d / I；不一致就说明参考坐标系没设对或零件漏选。

参考坐标系约定（与 SolidWorks 侧一致）
--------------------------------------
    origin = 髋轴线上一点
    e3     = 髋屈伸轴方向（= 单位化的 hip axis dir）
    e1,e2  = 与 e3 正交的任意两个单位向量
    CoM_ref = [e1·(cog - origin), e2·(cog - origin), e3·(cog - origin)]
    => hypot(cx, cy) = 髋轴到质心的垂距 d

Izz_O   = I_axis（绕髋轴，含平行轴项）        [kg*mm^2]
Izz_com = I_axis - m*d^2（去平行轴项）        [kg*mm^2]

输出
----
matlab2609/solidworks/mass_props_example_from_stp.csv
"""
import io
import os
import sys

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)

from exo2609.geometry import CadGeometry

OUT = os.path.join(ROOT, "matlab2609", "solidworks",
                   "mass_props_example_from_stp.csv")

cad = CadGeometry(ROOT)
geo = cad.build()

o = np.asarray(geo.axis_point, float)          # mm
e3 = np.asarray(geo.axis_dir, float)
e3 = e3 / np.linalg.norm(e3)

# 构造与 e3 正交的 e1, e2
tmp = np.array([0.0, 0.0, 1.0])
if abs(float(tmp @ e3)) > 0.9:
    tmp = np.array([1.0, 0.0, 0.0])
e1 = np.cross(e3, tmp)
e1 = e1 / np.linalg.norm(e1)
e2 = np.cross(e3, e1)

COLS = ["group", "part", "material", "mass_kg",
        "cx_mm", "cy_mm", "cz_mm",
        "Ixx_com", "Iyy_com", "Izz_com", "Ixy_com", "Ixz_com", "Iyz_com",
        "Ixx_O", "Iyy_O", "Izz_O", "Ixy_O", "Ixz_O", "Iyz_O", "note"]

# 注意：不写 "#" 注释行 —— readtable 未必跳过，会污染表头。
# 文本列一律用 ASCII，避免 MATLAB readtable 的编码坑；元信息放进 note 列。
AXIS_NOTE = ("from_local_STP_parse; hip_axis_point_mm=%s; hip_axis_dir=%s"
             % (np.round(o, 4).tolist(), np.round(e3, 8).tolist()))

lines = [",".join(COLS)]

rows = []
for g, r in geo.groups.items():
    cog = np.asarray(r.cog_world, float)
    rel = cog - o
    cx, cy, cz = float(e1 @ rel), float(e2 @ rel), float(e3 @ rel)
    d_mm = float(np.hypot(cx, cy))
    m = float(r.m_kg)
    I_axis = float(r.I_axis)                       # kg*m^2
    I_com = I_axis - m * (d_mm * 1e-3) ** 2        # kg*m^2
    rows.append([
        g, g + "_total", "from_STP_density_table", "%.6f" % m,
        "%.4f" % cx, "%.4f" % cy, "%.4f" % cz,
        "", "", "%.6f" % (I_com * 1e6), "", "", "",
        "", "", "%.6f" % (I_axis * 1e6), "", "", "",
        '"' + AXIS_NOTE + '"',
    ])
    # 自检：hypot(cx,cy) 必须等于 d
    assert abs(d_mm - r.d_perp_m * 1e3) < 1e-6 * max(1.0, d_mm), (
        g, d_mm, r.d_perp_m * 1e3)

lines += [",".join(r_) for r_ in rows]

os.makedirs(os.path.dirname(OUT), exist_ok=True)
io.open(OUT, "w", encoding="utf-8-sig", newline="").write("\n".join(lines) + "\n")

print("WROTE %s" % OUT)
print()
hdr = "%-9s %10s %10s %12s %12s %12s" % (
    "group", "m[kg]", "cx[mm]", "cy[mm]", "d[mm]", "I_axis[kg*m2]")
print(hdr)
print("-" * len(hdr))
for g, r in geo.groups.items():
    cog = np.asarray(r.cog_world, float)
    rel = cog - o
    cx, cy = float(e1 @ rel), float(e2 @ rel)
    print("%-9s %10.4f %10.4f %10.4f %10.4f %12.6f"
          % (g, r.m_kg, cx, cy, np.hypot(cx, cy), r.I_axis))
print()
print("== 本 CSV 的权威值（_stp2urdf.py 会读本文件做对账）==")
for g, r in geo.groups.items():
    print("  %-9s I_axis = %.9f kg*m^2   |tau_g| = %.9f N*m"
          % (g, r.I_axis, r.G_amp))
print("  注：上表 |tau_g| = m*g*d（g=%.5f）。CSV 本身只带 Izz_O，不带力矩；" % 9.80665)
print("      _stp2urdf.py 用 m*g*d 重建对账值，与真实幅值 m*g*d*|r3_perp|")
print("      差 ~|a_r3|^2/2 ≈ 4e-5（a_r3 = 关节轴的世界 z 分量，本例 -0.009056）。")
