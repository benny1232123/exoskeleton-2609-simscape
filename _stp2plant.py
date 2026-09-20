# -*- coding: utf-8 -*-
"""
从 exo_real.urdf 反解「解析对照」需要的等效单腿参数。

为什么必须做这一步
------------------
从 CAD 导入的多体模型，**关节零位 = CAD 建模时的位姿**，一般不是「腿自然下垂」。
所以绕髋轴的重力矩**不是一个纯 sin(q)**：

    tau_g(q) = -dU/dq ,  U(q) = m*g*z_com(q)

把 z_com(q) 写开（a = 关节轴单位向量，q 绕 a 按右手定则；Rl 是 child frame -> base）：

    z_com(q) = o_z + [Rl R(a,q) c]_z = const + W cos q + V sin q
      const = o_z + (a·r3)(a·c)                     (仿射项，别漏)
      W = r3·c - (a·r3)(a·c) ,   V = r3·(a x c)      (r3 = Rl 第 3 行)

    =>  tau_g(q) = m*g*(W sin q - V cos q) = A sin q + B cos q
        A = m*g*W ,  B = -m*g*V

**必须用这个形式对照**：若直接套教科书 M*q̈ = T - G*sin(q)，
会因为相位差而「假性失配」（本机实测 rms/range = 2.17，看着像模型全错）。

注意 W、V 在绕关节轴旋转坐标系时不变，所以 B=0 做不到，
除非腿恰好落在竖直面内 —— 也就是说这个相位是**物理位姿**决定的，不是坐标系选择能消掉的。

输出
----
matlab2609/simscape/plant_params.txt   （人读）
matlab2609/simscape/plant_params.csv   （MATLAB readmatrix 读；纯数值，中文/说明文字会让它解析不可靠）
    m_kg, I_axis, A, B, amp, phase_deg
"""
import io
import os
import re

import numpy as np
from scipy.spatial.transform import Rotation as Rot

SIMD = r"C:\Users\29408\Desktop\外骨骼\matlab2609\simscape"
URDF = os.path.join(SIMD, "exo_real.urdf")
OUT = os.path.join(SIMD, "plant_params.txt")

G = 9.81          # 必须与 build_harness_exo2dof.m 里设的 GravityVector 一致
LEGS = ["leg_L", "leg_R"]


def find(pat, s, flags=0):
    m = re.search(pat, s, flags)
    if not m:
        raise ValueError("pattern not found: %s" % pat)
    return m


def nums(s):
    return [float(x) for x in s.replace(",", " ").split()]


def rot_axis(a, q):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def um(v, nd=9):
    return " ".join("%.*f" % (nd, x) for x in v)


def main():
    txt = io.open(URDF, encoding="utf-8").read()

    lines = []
    lines.append("== plant_params  (derived from %s) ==" % os.path.basename(URDF))
    lines.append("g = %.4f" % G)
    lines.append("")

    hdr = ("%-7s %10s %13s %11s %11s %10s %11s"
           % ("joint", "m[kg]", "I_axis", "A", "B", "amp", "phase[deg]"))
    lines.append(hdr)
    lines.append("-" * len(hdr))

    rows = []
    for leg in LEGS:
        blk = find(r'<link name="%s">(.*?)</link>' % leg, txt, re.S).group(1)
        m = float(find(r'<mass value="([^"]+)"', blk).group(1))
        io_ = find(r'<origin xyz="([^"]+)" rpy="([^"]+)"', blk)
        c = np.array(nums(io_.group(1)))
        ixyz = {k: float(find(r'\b%s="([^"]+)"' % k, blk).group(1))
                for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
        I = np.array([[ixyz["ixx"], ixyz["ixy"], ixyz["ixz"]],
                      [ixyz["ixy"], ixyz["iyy"], ixyz["iyz"]],
                      [ixyz["ixz"], ixyz["iyz"], ixyz["izz"]]])

        side = leg.split("_")[1]
        jblk = find(r'<joint name="hip_%s"[^>]*>(.*?)</joint>' % side, txt, re.S).group(1)
        jor = find(r'<origin xyz="([^"]+)" rpy="([^"]+)"', jblk)
        o = np.array(nums(jor.group(1)))
        rpy = nums(jor.group(2))
        Rl = Rot.from_euler("XYZ", rpy).as_matrix()
        a = np.array(nums(find(r'<axis xyz="([^"]+)"', jblk).group(1)))
        a = a / np.linalg.norm(a)
        r3 = Rl[2, :]

        # ---- 闭式解 ----
        # 完整的仿射展开：(R c)_z = C0 + W cos q + V sin q
        a_r3 = float(a @ r3)
        a_c = float(a @ c)
        C0 = a_r3 * a_c
        W = float(r3 @ c) - a_r3 * a_c
        V = float(r3 @ np.cross(a, c))
        A = m * G * W
        B = -m * G * V

        # ---- 绕关节轴的等效惯量（过关节原点）----
        c_perp = c - a * a_c
        d = float(np.linalg.norm(c_perp))
        I_axis = float(a @ I @ a) + m * d * d

        amp = float(np.hypot(A, B))
        amp_exp = m * G * d * float(np.sqrt(max(0.0, 1.0 - a_r3 ** 2)))
        phase = float(np.degrees(np.arctan2(B, A)))

        # ---- 数值交叉校验：显式 Rodrigues 对拍 ----
        # 必带 C0：tau_g 对常数不敏感，但位姿对拍对常数敏感。
        qs = np.linspace(0.0, 2 * np.pi, 721)
        z = np.array([o[2] + (Rl @ rot_axis(a, q) @ c)[2] for q in qs])
        z_fit = o[2] + C0 + W * np.cos(qs) + V * np.sin(qs)
        err = float(np.max(np.abs(z - z_fit)))
        assert err < 1e-12, ("closed form mismatch", err)
        assert abs(amp - amp_exp) < 1e-9 * max(1.0, amp_exp), (amp, amp_exp)

        lines.append("")
        lines.append("[%s]" % leg)
        lines.append("  o (base)  = [%s] m" % um(o))
        lines.append("  a (base)  = [%s]   a·r3 = gamma = %.9f" % (um(a, 12), a_r3))
        lines.append("  c (child) = [%s] m   d = %.9f m" % (um(c), d))
        lines.append("  C0 = %+.9f  (= a_r3·(a·c)，仿射项；对 tau_g 无贡献)"
                     % C0)
        lines.append("  W = %+.9f   V = %+.9f" % (W, V))
        lines.append("  tau_g(q) = A sin q + B cos q = %+.9f sin q %+.9f cos q  [N*m]" % (A, B))
        lines.append("  |tau_g| = %.9f N*m   (闭式校验 m·g·d·sqrt(1-gamma²) = %.9f, 差 %.2e)"
                     % (amp, amp_exp, abs(amp - amp_exp)))
        lines.append("  phase = %+.6f deg" % phase)
        lines.append("  tau_g(0) = B = %+.6f N*m   <= 静止位姿就有重力矩，不为 0" % B)
        lines.append("  I_axis = aᵀIa + m·d² = %.9f + %.9f = %.9f kg*m^2"
                     % (float(a @ I @ a), m * d * d, I_axis))
        lines.append("  闭式解 vs Rodrigues 采点最大偏差 = %.3e m" % err)
        lines.append("  [对照] 若误用 -G*sin(q)：G_amp=m·g·d=%.6f, 相位差 %+.4f deg"
                     % (m * G * d, phase))
        rows.append((leg, m, I_axis, A, B, amp, phase))

    lines.append("")
    lines.append("== machine-readable (plant_params.csv) ==")
    csv_lines = ["m_kg,I_axis,A,B,amp,phase_deg"]
    for (leg, m, I_axis, A, B, amp, phase) in rows:
        s = "%.9f,%.9f,%+.9f,%+.9f,%.9f,%.6f" % (m, I_axis, A, B, amp, phase)
        lines.append("  %-7s %s" % (leg, s))
        csv_lines.append(s)
    lines.append("  order: hip_L (row 1), hip_R (row 2)")

    body = "\n".join(lines) + "\n"
    io.open(OUT, "w", encoding="utf-8").write(body)
    io.open(os.path.join(SIMD, "plant_params.csv"), "w",
            encoding="utf-8").write("\n".join(csv_lines) + "\n")
    print(body)


if __name__ == "__main__":
    main()
