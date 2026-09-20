# -*- coding: utf-8 -*-
"""
_multidof_conditions.py —— 按**论文的工况**驱动 4-DOF 模型，每个工况渲染一个动图
================================================================================

论文（arXiv:2609.15352）的实验工况
----------------------------------
- 结构化：**平地 1.0 m/s**、**5° 上坡 1.0 m/s**、**上楼梯**（"three canonical
  locomotion tasks: level walking, inclined walking, and stair climbing"）
- 非结构化户外：岩石 / 斜坡 / 草地、攀爬 / 跑 / 走
- 变速跑步机：0.6 / 1.0 / 1.4 m/s

本脚本做前三个（模型里能表达的）。被动自由摆动那一条已有 Simscape 轨迹，
见 `multidof_drop.gif`，不在这里重复仿真。

★ 建模口径（务必分清，别当成实测）
----------------------------------
  · 地形本身**不进入模型** —— 我们这套是**髋关节多刚体动力学**，没有地面接触/
    足底反力。所谓"工况"体现在**髋关节参考轨迹**上：上坡/上楼 = 屈髋范围更大、
    更偏屈曲、步频更低。
  · 参考轨迹是 **surrogate（代理）步态**：与 `+exo2609/gait_ref.m` 同一套做法
    （2 次谐波傅里叶，对 8 个关键事件点最小二乘）。工况差异 = **关键事件角度表 +
    步频**。论文没有公布髋关节轨迹，这里是按步态常识设的**示意值**，不是实测。
  · 驱动 = 每腿髋关节 **PD + 重力前馈**（Q_hip = G_hip + Kp·e + Kd·ė）。
  · abduct 关节**无主动驱动**，但有**被动约束**：衬套摩擦（阻尼）+ 弱回中刚度
    （模拟黄铜衬套摩擦与软组织约束）。**这不是可有可无的**：先跑了无约束版本，
    上坡工况下无驱动的 abduct 被髋运动**共振泵到 2.85 rad（163°）** —— 人腿不可能
    外展 163°，说明缺了耗散项。加上 K_abd / C_abd 后落在合理范围。

产物
----
  matlab2609/out_simscape/cond_<key>.csv / .gif / _strip.png
  matlab2609/out_simscape/conditions_summary.txt
"""
import io
import os
import sys
import time

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)

from _multidof_dyn import MultiBody            # noqa: E402
from _multidof_anim import render_csv          # noqa: E402

SIM = os.path.join(ROOT, "matlab2609", "simscape")
OUTD = os.path.join(ROOT, "matlab2609", "out_simscape")
URDF4 = os.path.join(SIM, "exo_multidof.urdf")
SUM = os.path.join(OUTD, "conditions_summary.txt")

# keydeg = 归一化周期上的 8 个关键事件屈髋角 [deg]（末点 = 首点，保证周期）
#   平地  : 正常步态事件点（与 gait_ref.m 同源）
#   上坡  : 整体前移 + 屈髋加大（减少髋后伸）
#   上楼梯: 抬腿为主，屈髋峰值 ~60°，后伸明显减小，步频更低
CONDS = [
    dict(key="level",   name="平地行走 1.0 m/s", f=0.89,
         keydeg=[25, 18, 5, -10, 5, 28, 22, 25], dur=2.5),
    dict(key="incline", name="上坡 5°  1.0 m/s", f=0.80,
         keydeg=[33, 26, 13, -2, 12, 36, 30, 33], dur=2.8),
    dict(key="stair",   name="上楼梯", f=0.62,
         keydeg=[50, 42, 26, 10, 22, 60, 52, 50], dur=3.2),
]

KEYPHI = np.array([0.00, 0.10, 0.30, 0.50, 0.60, 0.75, 0.85, 1.00])

# PD 增益：腿摆动件惯量只有 ~0.017 kg·m²，Kd ≈ 2√(Kp·M) ≈ 0.74 就是临界阻尼
KP, KD = 8.0, 0.8

# abduct 关节的**被动**约束：衬套摩擦 + 弱回中刚度（无主动驱动）
#   ω = √(K/M) ≈ 10.8 rad/s，ζ = C/(2√(KM)) ≈ 0.45 —— 略微欠阻尼，看起来像"被软组织拉着"
K_ABD, C_ABD = 2.0, 0.20

QS = ["hip_L", "abduct_L", "hip_R", "abduct_R"]
HIP_IDX = [0, 2]
ABD_IDX = [1, 3]
LR_DELAY = 0.5          # 左右腿相位差（对侧步态）
DT = 1e-3

T0 = time.time()


def fit_coef(keydeg):
    """8 个关键事件角 -> 2 次谐波傅里叶系数 [a0, a1, b1, a2, b2]。"""
    ykey = np.deg2rad(np.asarray(keydeg, float))
    A = np.ones((len(KEYPHI), 5))
    for k in range(1, 3):
        A[:, 2 * k - 1] = np.cos(2 * np.pi * k * KEYPHI)
        A[:, 2 * k] = np.sin(2 * np.pi * k * KEYPHI)
    return np.linalg.lstsq(A, ykey, rcond=None)[0]


def ref_series(coef, f, t, delay=0.0):
    """给定傅里叶系数与步频，返回 q_ref, qd_ref。"""
    phi = np.mod(f * t + delay, 1.0)
    w = 2 * np.pi * f
    q = np.full_like(phi, coef[0], dtype=float)
    qd = np.zeros_like(phi)
    for k in range(1, 3):
        c, s = coef[2 * k - 1], coef[2 * k]
        q += c * np.cos(2 * np.pi * k * phi) + s * np.sin(2 * np.pi * k * phi)
        qd += w * k * (-c * np.sin(2 * np.pi * k * phi) + s * np.cos(2 * np.pi * k * phi))
    return q, qd


def rk4(f, y, dt):
    k1 = f(y); k2 = f(y + 0.5 * dt * k1)
    k3 = f(y + 0.5 * dt * k2); k4 = f(y + dt * k3)
    return y + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(mb, cond):
    """闭环仿真一个工况，返回 t / q(n,4) / tau 峰值。"""
    t = np.arange(0.0, cond["dur"], DT)
    n = len(t)
    coef = fit_coef(cond["keydeg"])
    qL, qdL = ref_series(coef, cond["f"], t, 0.0)
    qR, qdR = ref_series(coef, cond["f"], t, LR_DELAY)
    qref = np.column_stack([qL, np.zeros(n), qR, np.zeros(n)])
    qdref = np.column_stack([qdL, np.zeros(n), qdR, np.zeros(n)])

    y = np.concatenate([qref[0], qdref[0]])     # 从参考轨迹起步 -> 无起始瞬态
    Q = np.zeros((n, 4))
    tau_pk = 0.0

    for k in range(n):
        if k:
            kk = k

            def deriv(yy):
                qq, vv = yy[:4], yy[4:]
                Gv = mb.G(qq)                   # ★ 只算一次（qdd() 内部还会再算一次）
                e = qref[kk] - qq
                ed = qdref[kk] - vv
                tau = np.zeros(4)
                tau[HIP_IDX] = Gv[HIP_IDX] + KP * e[HIP_IDX] + KD * ed[HIP_IDX]
                tau[ABD_IDX] = -K_ABD * qq[ABD_IDX] - C_ABD * vv[ABD_IDX]
                rhs = tau - mb.coriolis(qq, vv) - Gv
                return np.concatenate([vv, np.linalg.solve(mb.M(qq), rhs)])

            y = rk4(deriv, y, DT)
        Q[k] = y[:4]
        Gv = mb.G(y[:4])
        e = qref[k] - y[:4]
        ed = qdref[k] - y[4:]
        tau_now = np.zeros(4)
        tau_now[HIP_IDX] = Gv[HIP_IDX] + KP * e[HIP_IDX] + KD * ed[HIP_IDX]
        tau_now[ABD_IDX] = -K_ABD * y[ABD_IDX] - C_ABD * y[4:][ABD_IDX]
        tau_pk = max(tau_pk, float(np.max(np.abs(tau_now[HIP_IDX]))))
        if (k + 1) % 500 == 0:
            print("    %s  %d/%d  (%.0fs)" % (cond["key"], k + 1, n, time.time() - T0))
    return t, Q, qref, tau_pk


def write_csv(path, t, Q):
    with io.open(path, "w", encoding="utf-8", newline="") as f:
        f.write("t," + ",".join(q + ".q" for q in QS) + "\n")
        for i in range(len(t)):
            f.write("%.6f," % t[i] + ",".join("%.9f" % Q[i, j] for j in range(4)) + "\n")


def main():
    mb = MultiBody(URDF4, verbose=True)
    print("  总质量 = %.9f kg" % sum(mb.link[l]["mass"] for l in mb.links))

    lines = []

    def say(s=""):
        lines.append(s)
        print(s)

    say("=" * 84)
    say("多工况驱动（对齐论文的三个典型工况；surrogate 参考 + hip PD/重力前馈）")
    say("=" * 84)
    say("  模型: %s" % os.path.basename(URDF4))
    say("  驱动: Q_hip = G_hip + Kp*e + Kd*ed   (Kp=%.1f, Kd=%.1f)" % (KP, KD))
    say("        abduct 无主动驱动，但有被动约束 -K_abd*q - C_abd*qd"
        "  (K_abd=%.1f N*m/rad, C_abd=%.2f N*m*s/rad)" % (K_ABD, C_ABD))
    say("")
    say("  %-9s %-20s %7s %10s %10s %10s %9s"
        % ("key", "name", "f/Hz", "hip摆幅", "abd摆幅", "跟踪RMS", "|tau|pk"))
    say("  " + "-" * 80)

    for cond in CONDS:
        print("\n== %s ==" % cond["name"])
        t, Q, qref, tp = simulate(mb, cond)
        csvp = os.path.join(OUTD, "cond_%s.csv" % cond["key"])
        write_csv(csvp, t, Q)

        s = int(0.2 * len(t))                   # 丢掉前 20% 看稳态
        trk = float(np.sqrt(np.mean((Q[s:, HIP_IDX] - qref[s:, HIP_IDX]) ** 2)))
        hip_amp = float(np.ptp(Q[:, 0]))
        abd_amp = float(np.ptp(Q[:, 1]))

        head = (cond["name"] + "   t = %.2f s\n"
                "hip_L %+.2f  abduct_L %+.2f  hip_R %+.2f  abduct_R %+.2f  [rad]")
        foot = ("surrogate 参考 + hip PD/重力前馈；abduct 被动约束（无驱动）"
                "   4-DOF 解析模型")
        render_csv(csvp, os.path.join(OUTD, "cond_%s.gif" % cond["key"]),
                   os.path.join(OUTD, "cond_%s_strip.png" % cond["key"]),
                   head, foot,
                   strip_title="%s —— 4-DOF 闭环仿真（surrogate 参考 + PD/重力前馈）"
                               % cond["name"])

        say("  %-9s %-20s %7.2f %10.4f %10.4f %10.5f %9.4f"
            % (cond["key"], cond["name"], cond["f"], hip_amp, abd_amp, trk, tp))

    say("")
    say("  摆幅单位 rad；跟踪RMS 只统计 hip 关节、丢掉前 20% 起始段；|tau|pk 为髋关节")
    say("  总力矩峰值（含重力前馈）[N*m]，可与论文 |tau_g| 幅值 0.558 N*m 对照。")
    say("")
    say("★ 口径提醒：地形**不进入模型**（无接触/足底反力）；上坡/上楼的差别只体现在")
    say("  髋关节参考轨迹（屈髋更大、更偏屈曲、步频更低）。参考是 surrogate，非实测。")
    say("")
    say("CONDITIONS_OK")
    io.open(SUM, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\nWROTE", SUM)


if __name__ == "__main__":
    main()
