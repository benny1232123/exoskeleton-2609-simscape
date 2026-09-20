# -*- coding: utf-8 -*-
"""
从真实装配体 STEP 直接生成 Simscape Multibody 可用的 URDF。

动机
----
`smimport` 只接受 Simscape Multibody XML 或 URDF，**不接受 .stp**。
但 .stp 里其实啥都有（几何 + 由它算出的完整惯量张量），
缺的只是 **拓扑语义**（哪些零件属于同一个刚体、转轴在哪）。
本脚本补上这一层：用 `exo2609.geometry` 里已跑通的 STEP 解析，
按分组聚合成刚体，用拟合出的髋轴当转动副，直接吐出 URDF。
=> 不需要 SolidWorks 插件，也不需要人工在 CAD 里量坐标。

设计决定：**link frame 与世界坐标系对齐（rpy 恒为 0）**
-----------------------------------------------------
第一版给关节写了 `rpy`，结果踩了坐标系约定的坑：
URDF 的 rpy 语义与工具链里其它地方的"XYZ 欧拉角"很容易不一致，
而一旦不一致，质心就被放错位置、重力矩就悄悄变了 —— 而且**不报错**。

所以这里把关节 `<origin rpy>` 恒设为 `0 0 0`，让 **child link frame ≡ 世界系**：

    origin xyz = 髋轴上一点（世界系，m）
    origin rpy = 0 0 0
    axis       = 髋轴方向（世界系，单位向量）      <-- 轴靠 3 分量显式给出
    inertial.origin xyz = 质心相对关节原点的世界系偏移（m）
    inertia            = 绕质心、**世界轴向**的惯量张量（kg·m²）

这样整条链上只有「世界系」一个坐标系，**没有任何旋转约定需要猜**。

分组约定（与 mass_props_example_from_stp.csv 一致）
--------------------------------------------------
    motor_L + motor_R + back  -> base   （相对躯干不动）
    leg_L / leg_R             -> 各一条腿（绕髋轴摆动）

输出
----
    matlab2609/simscape/exo_real.urdf
    matlab2609/simscape/exo_real_report.txt
"""
import csv
import io
import os
import sys

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)

from exo2609.geometry import (CadGeometry, density_of, MM3_TO_M3,
                              MM5_TO_KGM2_PER_RHO, G)

OUT_URDF = os.path.join(ROOT, "matlab2609", "simscape", "exo_real.urdf")
OUT_RPT = os.path.join(ROOT, "matlab2609", "simscape", "exo_real_report.txt")
MESH_DIR = os.path.join(ROOT, "matlab2609", "simscape", "meshes")

BASE_GROUPS = ["motor_L", "motor_R", "back"]
LEG_GROUPS = ["leg_L", "leg_R"]
G_HARNESS = 9.81        # 必须与 build_harness_exo2dof.m 的 GravityVector 一致

# 是否把 `meshes/<link>.stl` 写进 <visual>（由 _stp2stl.py 产出）。
# 用途：让 Mechanics Explorer 显示真实外形；动力学不受影响（<inertial> 才是动力学）。
# 用相对路径（相对 URDF 所在目录）——MATLAB 侧通过 ASCII junction
# C:\Users\29408\exo_work 访问，绕开中文路径在 STL 读取时的编码问题。
EMIT_VISUAL = True
MESH_REL = "meshes/%s.stl"
VIZ_COLOR = {"base": "0.55 0.57 0.62 1.0",
             "leg_L": "0.20 0.45 0.85 1.0",
             "leg_R": "0.85 0.35 0.20 1.0"}

ID3 = np.eye(3)


# ----------------------------------------------------------------------
def group_props(cad, group):
    """聚合一个刚体：总质量、世界系质心(mm)、绕质心的完整惯量张量(kg·m², 世界轴向)。"""
    solids, warns = cad._map_solids(group)
    insts = [i for i in cad.placed if cad.group_of(i) == group]
    items = []
    for inst in insts:
        R, t = inst.T[:3, :3], inst.T[:3, 3]
        rho, _label = density_of(inst.part_name, cad.densities, cad.fallback)
        for msb in inst.solids:
            sp = solids.get(msb)
            if sp is None:
                continue
            m = rho * sp.volume_mm3 * MM3_TO_M3
            c = R @ sp.cog_local + t
            I = R @ sp.inertia_local @ R.T * rho * MM5_TO_KGM2_PER_RHO
            items.append((m, c, I))
    m_tot = sum(it[0] for it in items)
    cog = sum(it[0] * it[1] for it in items) / m_tot
    I_tot = np.zeros((3, 3))
    for m, c, I in items:
        v = (c - cog) * 1e-3
        I_tot += I + m * ((v @ v) * ID3 - np.outer(v, v))
    return dict(name=group, m=m_tot, cog=cog, I=I_tot, warns=warns, n=len(items))


def merge(*props):
    m_tot = sum(p["m"] for p in props)
    cog = sum(p["m"] * p["cog"] for p in props) / m_tot
    I = np.zeros((3, 3))
    for p in props:
        v = (p["cog"] - cog) * 1e-3
        I += p["I"] + p["m"] * ((v @ v) * ID3 - np.outer(v, v))
    return dict(name="+".join(p["name"] for p in props),
                m=m_tot, cog=cog, I=I, warns=[], n=sum(p["n"] for p in props))


def fix_triangle(I, tol_rel=1e-3):
    """URDF/Simscape 要求主轴惯量满足三角不等式 li<=lj+lk；
    薄壁件按查表密度算出的惯量常轻微违反 —— 加最小各向同性项修正。"""
    w, _ = np.linalg.eigh(I)
    w = np.sort(w)
    need = max(0.0, (w[2] - w[0] - w[1])) * (1.0 + tol_rel)
    if need <= 0:
        return I, 0.0, w
    return I + (need / 3.0) * ID3, need / 3.0, w


def inertia_xyz(I):
    return I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2]


def rot_axis(a, q):
    """Rodrigues：绕单位轴 a 转 q 的旋转矩阵。"""
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return ID3 + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def um(v, nd=9):
    return " ".join("%.*f" % (nd, x) for x in v)


# ----------------------------------------------------------------------
def main():
    cad = CadGeometry(ROOT)
    geo = cad.build()

    o = np.asarray(geo.axis_point, float)            # mm，髋轴上一点
    a = np.asarray(geo.axis_dir, float)
    a = a / np.linalg.norm(a)                        # 髋屈伸轴（世界系单位向量）
    r3 = np.array([0.0, 0.0, 1.0])                   # link frame ≡ 世界系

    L = []
    L.append("== exo_real.urdf 生成报告 (link frame = 世界系) ==")
    L.append("hip axis point [mm] = %s" % um(o, 4))
    L.append("hip axis dir         = %s  (norm=%.6f, 即 link frame 里的 z 分量 gamma=%.6f)"
             % (um(a, 8), np.linalg.norm(a), a[2]))
    L.append("关节 origin rpy      = 0 0 0   (child link frame ≡ base/世界系)")
    L.append("")

    # ---- 分组聚合 ----
    legs, bases = {}, []
    for g in LEG_GROUPS:
        legs[g] = group_props(cad, g)
    for g in BASE_GROUPS:
        bases.append(group_props(cad, g))
    base = merge(*bases)

    L.append("-- 刚体（世界系） --")
    for p in bases + list(legs.values()):
        L.append("  %-8s m=%9.6f kg  cog=[%s] mm  n_solid=%d %s"
                 % (p["name"], p["m"], um(p["cog"], 4), p["n"],
                    ("WARN:" + ";".join(p["warns"])) if p["warns"] else ""))
    L.append("  %-8s m=%9.6f kg  cog=[%s] mm  <= base 合并"
             % ("base", base["m"], um(base["cog"], 4)))
    L.append("  整机总质量 = %.6f kg" % (base["m"] + sum(p["m"] for p in legs.values())))
    L.append("")

    # ---- 每条腿：质心偏移 + 绕质心世界轴向惯量 + 重力矩系数 ----
    leg_data = {}
    for g, p in legs.items():
        c = (p["cog"] - o) * 1e-3                    # 世界系偏移（m），因 Rl=I
        I_fix, add, eig = fix_triangle(p["I"])
        c_perp = c - a * float(a @ c)
        d = float(np.linalg.norm(c_perp))
        I_axis = float(a @ I_fix @ a) + p["m"] * d * d   # 绕「过 o 的髋轴」

        # 重力矩：z_com(q) = C0 + W cos q + V sin q
        #
        # 绕轴 a 把质心偏移 c 转 q 后，z 分量的**完整**仿射展开（a_r3 = a·r3）：
        #     (R c)_z = C0 + W cos q + V sin q
        #     C0 = a_r3 · (a·c)                      <- 仿射项，最容易漏
        #     W  = c_z - a_r3 (a·c) = r3·c - (a·r3)(a·c)
        #     V  = (a×c)_z          = r3·(a×c)
        # C0 是常数项，而 tau_g = -dU/dq 对常数不敏感 -> 对力矩无影响；
        # 但自检对拍时必须带上它，否则会误报「闭式解失配」。
        a_r3 = float(a @ r3)
        a_c = float(a @ c)
        C0 = a_r3 * a_c
        W = float(r3 @ c) - a_r3 * a_c
        V = float(r3 @ np.cross(a, c))
        A = p["m"] * G_HARNESS * W
        B = -p["m"] * G_HARNESS * V
        amp = float(np.hypot(A, B))
        phase = float(np.degrees(np.arctan2(B, A)))

        # ---- 数值交叉校验：显式 Rodrigues 采点，与 W/V/C0 闭式解对拍 ----
        qs = np.linspace(0.0, 2 * np.pi, 721)
        z = np.array([o[2] * 1e-3 + (rot_axis(a, q) @ c)[2] for q in qs])
        z_fit = (o[2] * 1e-3) + C0 + W * np.cos(qs) + V * np.sin(qs)
        err = float(np.max(np.abs(z - z_fit)))
        assert err < 1e-12, ("closed form mismatch", err)

        leg_data[g] = dict(m=p["m"], c=c, I=I_fix, add=add, eig=eig,
                           d=d, I_axis=I_axis, A=A, B=B, amp=amp, phase=phase)

        L.append("-- %s（link frame = 世界系，原点 = 髋轴上一点）--" % g)
        L.append("  CoM offset (世界系, 相对关节原点) = [%s] m" % um(c, 9))
        L.append("  到髋轴垂距 d = %.9f m" % d)
        L.append("  主轴惯量(世界轴向) = [%s]  (三角不等式余量 %.3e)"
                 % (um(eig, 9), min(eig[0] + eig[1] - eig[2],
                                    eig[1] + eig[2] - eig[0])))
        if add > 0:
            L.append("  ⚠ 三角不等式轻微违反 -> 各向同性补正 %.3e kg·m²" % add)
        L.append("  绕髋轴有效惯量 I_axis = aᵀIa + m·d² = %.9f kg·m²" % I_axis)
        L.append("  质心高度 z_com(q) = C0 + W cos q + V sin q   (仿射项 C0 不影响力矩)")
        L.append("        C0 = %+.9f m ,  W = %+.9f m ,  V = %+.9f m" % (C0, W, V))
        L.append("  重力矩 tau_g(q) = A sin q + B cos q")
        L.append("        A = %+.9f ,  B = %+.9f  [N·m]" % (A, B))
        L.append("        |tau_g| = %.9f N·m , 相位 = %+.4f deg" % (amp, phase))
        L.append("        静止位姿 tau_g(0) = B = %+.6f N·m   <= 非 0！" % B)
        L.append("  闭式解 vs Rodrigues 采点最大偏差 = %.3e m  (应 ~1e-16)" % err)
        L.append("")

    # ---- 与 CSV 基准对账 ----
    # ★ 基准真源 = mass_props_example_from_stp.csv，由 _make_sw_csv.py 经
    #   `CadGeometry.build()` 这条**独立聚合路径**生成 —— 所以这不是自证，
    #   两边的质心/惯量求和代码是分开写的。
    #
    #   以前这里硬编码了一组数字。密度表一改，它就从"对账"退化成"对着旧值打钩"：
    #   照样打印 OK，实际什么也没校验（本次密度修正就撞上了这个陷阱）。
    #   改为读文件；读不到才回退常量，并在报告里**显式标出回退**。
    CSV_REF = os.path.join(ROOT, "matlab2609", "solidworks",
                           "mass_props_example_from_stp.csv")
    REF = {"leg_L": (0.016997395283, 0.558396084),
           "leg_R": (0.016997437256, 0.558402151)}    # 回退值 = 改后基准
    ref_src = "HARDCODED FALLBACK ***"
    try:
        with io.open(CSV_REF, "r", encoding="utf-8-sig", newline="") as f:
            n_ref = 0
            for row in csv.DictReader(f):
                gg = (row.get("group") or "").strip()
                if gg not in ("leg_L", "leg_R"):
                    continue
                d_mm = float(np.hypot(float(row["cx_mm"]), float(row["cy_mm"])))
                REF[gg] = (float(row["Izz_O"]) * 1e-6,
                           float(row["mass_kg"]) * G_HARNESS * d_mm * 1e-3)
                n_ref += 1
            if n_ref == 2:
                ref_src = os.path.basename(CSV_REF)
    except Exception as _e:
        ref_src = "HARDCODED FALLBACK *** (%s)" % _e
    L.append("-- 对账（vs %s 的 Izz_O 与 m·g·d）--" % ref_src)
    ok = True
    for g in LEG_GROUPS:
        dd = leg_data[g]
        rM, rA = REF[g]
        eM = abs(dd["I_axis"] - rM) / rM
        eA = abs(dd["amp"] - rA) / rA
        good = (eM < 5e-3) and (eA < 5e-3)
        ok = ok and good
        L.append("  %-6s I_axis %.9f vs %.9f (rel %.2e) |  |tau_g| %.9f vs %.9f (rel %.2e)  %s"
                 % (g, dd["I_axis"], rM, eM, dd["amp"], rA, eA,
                    "OK" if good else "MISMATCH"))
    L.append("")

    # ---- 生成 URDF ----
    x = ['<?xml version="1.0"?>', "<!--"]
    x.append("  exo_real.urdf : 由真实装配体 STEP 自动生成（_stp2urdf.py）")
    x.append("  ==================================================================")
    x.append("  分组 : base = motor_L + motor_R + back ; leg_L / leg_R 绕髋轴摆动")
    x.append("  髋轴 : point=[%s] mm  dir=[%s]" % (um(o, 4), um(a, 8)))
    x.append("  frame: 关节 origin rpy = 0 0 0 -> child link frame ≡ 世界系")
    x.append("  单位 : m / kg / kg*m^2")
    x.append("  可视 : <visual><mesh filename='meshes/<link>.stl'/> 世界系顶点(m)，")
    x.append("         由 _stp2stl.py 从 _reduced3/*.stp 网格化产出；不影响动力学")
    x.append("-->")
    x.append('<robot name="exo_real">')

    def link(name, m, c, I):
        ixx, ixy, ixz, iyy, iyz, izz = inertia_xyz(I)
        out = ["  <link name=\"%s\">" % name,
               "    <inertial>",
               "      <origin xyz=\"%s\" rpy=\"0 0 0\"/>" % um(c, 9),
               "      <mass value=\"%.9f\"/>" % m,
               "      <inertia ixx=\"%.12f\" ixy=\"%.12f\" ixz=\"%.12f\"" % (ixx, ixy, ixz),
               "               iyy=\"%.12f\" iyz=\"%.12f\" izz=\"%.12f\"/>" % (iyy, iyz, izz),
               "    </inertial>"]
        if EMIT_VISUAL and os.path.exists(os.path.join(MESH_DIR, "%s.stl" % name)):
            # 顶点已在世界系（m），link frame ≡ 世界系 => origin 取 0 即可
            out += ["    <visual>",
                    "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>",
                    "      <geometry>",
                    "        <mesh filename=\"%s\"/>" % (MESH_REL % name),
                    "      </geometry>",
                    "      <material name=\"exo_%s\">" % name,
                    "        <color rgba=\"%s\"/>" % VIZ_COLOR.get(name, "0.70 0.72 0.76 1.0"),
                    "      </material>",
                    "    </visual>"]
        out += ["  </link>"]
        return out

    x += link("base", base["m"], (base["cog"] - o) * 1e-3, base["I"])
    for g in LEG_GROUPS:
        dd = leg_data[g]
        x += link(g, dd["m"], dd["c"], dd["I"])

    for g in LEG_GROUPS:
        x += ["  <joint name=\"hip_%s\" type=\"revolute\">" % g.split("_")[1],
              "    <parent link=\"base\"/>",
              "    <child  link=\"%s\"/>" % g,
              "    <origin xyz=\"%s\" rpy=\"0 0 0\"/>" % um(o * 1e-3, 9),
              "    <axis xyz=\"%s\"/>" % um(a, 12),
              "    <limit lower=\"-1.5\" upper=\"1.5\" effort=\"18\" velocity=\"10\"/>",
              "    <dynamics damping=\"0.0\" friction=\"0.0\"/>",
              "  </joint>"]

    x.append("</robot>")
    x.append("")

    os.makedirs(os.path.dirname(OUT_URDF), exist_ok=True)
    io.open(OUT_URDF, "w", encoding="utf-8", newline="\n").write("\n".join(x))

    L.append("URDF -> %s" % OUT_URDF)
    L.append("  关节 origin (世界系) = [%s] m   rpy = 0 0 0" % um(o * 1e-3, 9))
    L.append("  关节 axis  (世界系) = [%s]" % um(a, 12))
    L.append("")
    L.append("-- <visual> 网格 --")
    if not EMIT_VISUAL:
        L.append("  EMIT_VISUAL=False -> 本次不写可视化网格（纯动力学版）")
    for nm in ["base"] + LEG_GROUPS:
        mp = os.path.join(MESH_DIR, "%s.stl" % nm)
        if os.path.exists(mp):
            L.append("  %-6s %-28s %7.2f MB  -> filename=\"%s\""
                     % (nm, mp, os.path.getsize(mp) / 1048576.0, MESH_REL % nm))
        else:
            L.append("  %-6s 缺失 %s  -> 该 link 无 visual（Mechanics Explorer 只显示占位体）"
                     % (nm, mp))
    L.append("  提示：仅当 'meshes/<link>.stl' 存在时才会写 <visual>；")
    L.append("        没有网格不影响动力学，只是 Mechanics Explorer 没有外形。")
    L.append("")
    L.append("VERDICT: %s" % ("PASS" if ok else "FAIL"))
    io.open(OUT_RPT, "w", encoding="utf-8").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
