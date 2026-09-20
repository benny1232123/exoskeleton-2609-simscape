# -*- coding: utf-8 -*-
"""
_stp2urdf_multidof.py —— 把「腿」拆成多段，把 CAD 里能动的结构建成自由度

为什么不能只加 <joint>
----------------------
`exo_real.urdf` 里 leg_L / leg_R 各是**一个刚体**（一个 link 一个 <inertial>）。
要在腿内部加关节，必须先把这条腿**切成若干段**，每段自己算质量/质心/惯量。

本脚本按证据（不是按名字猜）做三件事
------------------------------------
1) 自动测出 abduct 关节的**轴位/轴向**（复用「同轴+同半径」圆柱面配对）：
     abduct  : 轴线 ⊥ 髋轴，来自 {电机_轴, 电机_黄铜轴套8_10_18, 电机_出轴} 的 Ø8 配合
2) 按**显式零件归属**把腿分成 3 段（切割平面法在这个装配体上不成立：
   绑缚区零件彼此只差几 mm，没有自然间隙 —— 见 _multidof_plan.txt）
3) **守恒校验**：Σm、合成质心、绕原质心的合成惯量 必须与原 leg 一致（否则拆分就错了）

★ 已废弃：`strap`（横穿滑块的防转键）—— 别再加回来
---------------------------------------------------
早期版本还建过一个 `strap` revolute（轴线 ∥ 髋轴，来自 {腿部_绑缚 Ø7.6 孔, 滑块_双键 Ø8 销}）。
**复查后判定是错的**：那根销的功能就是**阻止**滑块转动，把它建成转动副等于
「把被约束的自由度当成自由度」（实测 0.6 rad 只推动质心 3.98 mm）。
现状：每腿 3 段 = L1 电机输出 / L2 腿杆 / L3 滑块+按键；
关节链 = hip(revolute) → abduct(revolute) → slide(**fixed**，行程仅 2 mm 装配间隙)。
⇒ **真实可动自由度 = 4**（左右各有 hip 屈伸 + abduct 髋部横向）。

frame 约定（沿用 _stp2urdf.py 的既有约定，别改）
-----------------------------------------------
  · 每个 link 的 frame 都与世界系**轴对齐**，只是原点平移到该段的关节原点
    （joint <origin rpy="0 0 0">）
  · 因此某段的 inertial origin = (该段质心 − 该段入口关节的原点)，世界系分量
  · 关节的 <axis> 因为 rpy=0，直接就是**世界系**单位向量
  · 串联链上，第 k 个关节的 origin 必须表达在**父 link frame** 里
      => origin_xyz = O_k − O_{k-1}   （O 均为世界系）

产物
----
  matlab2609/simscape/exo_multidof.urdf           4 自由度（每腿 hip + abduct；slide=fixed）
  matlab2609/simscape/exo_multidof_locked.urdf    同样拓扑，但 abduct 也 type="fixed"
                                                  （回归用：应与原 2-DOF **逐位一致**）
  matlab2609/simscape/meshes/leg_{L,R}_{1,2,3}.stl   各段外形网格
  matlab2609/simscape/exo_multidof_report.txt
"""
import io
import os
import struct
import sys
from collections import defaultdict

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)
from exo2609.geometry import (CadGeometry, density_of, MM3_TO_M3, MM5_TO_KGM2_PER_RHO)

SIM = os.path.join(ROOT, "matlab2609", "simscape")
MESH_DIR = os.path.join(SIM, "meshes")
OUT_URDF = os.path.join(SIM, "exo_multidof.urdf")
OUT_URDF_LOCK = os.path.join(SIM, "exo_multidof_locked.urdf")
OUT_RPT = os.path.join(SIM, "exo_multidof_report.txt")

G_HARNESS = 9.81
ID3 = np.eye(3)

# ------------------------------------------------------------------ 移动副（第三个自由度）
# 滑动轴 s = 导轨/滑块/挡片 的公共长轴（PCA 三件完全一致），⊥髋轴（|s·髋轴| = 0.0007）
# 证据（_slide_check.txt）：滑块↔导轨 4 组面对面贴合，法向 ±Y(间隙0.5mm) 与
#   ±(0.91,-0.01,0.42)(间隙0mm)，两组都 ⊥ s ⇒ 四面抱死，只剩沿 s 的平移。
# ⚠ 行程口径有冲突，这里取「接触面保持引导」的实测窗口，并在报告里标注不确定度：
#     · 接触窗口口径: +18.6 / -24.3 mm   <- 采用
#     · 整体包围盒口径: 0 mm（导轨/滑块/挡片沿 s 投影全是 72.00mm，像已坐到位）
S_SLIDE = np.array([0.423036, 0.008001, -0.906077])
S_SLIDE = S_SLIDE / np.linalg.norm(S_SLIDE)
SLIDE_LO = -0.0243          # m
SLIDE_HI = +0.0186          # m
# ★★ 是否把「导轨滑移」建成真自由度。当前 = **否**，理由（2026-09-19 复查确认）：
#   1) 滑块与导轨/挡片沿 s **完全等长（都是 72.00mm）** ⇒ 行程上限 0，不是设计行程；
#   2) 真正的限位是滑块↔绑缚的**端面间隙 2.0mm**，即"能挪的是间隙不是行程"；
#   3) 滑块横截面比导轨还大 ⇒ 它是抱在导轨上的**夹紧块**，工作时被双键/按键锁住；
#   4) 滑动质量仅 0.028 kg / 整腿 0.305 kg（9.2%），满行程也只让整腿质心动 ≈1.8mm。
#   ⇒ 改成 True 即可恢复（会把 L2→L3 写成 prismatic，行程用上面的 LO/HI）。
ENABLE_SLIDE = False

# ★★ 3D 渲染开关（只影响 <visual>，**完全不影响动力学**）
# 用户要求：slide 既然已按 fixed 忽略，那个"抱在腿杆下端的滑块"就只剩视觉噪声，
# 在 Mechanics Explorer 里把它消掉。质量/惯量/关节一个都不动 ⇒ 回归结果逐位不变。
#   · "L3" 是滑块(腿部_滑块_双键) + 按键(腿部_按键_双)，8290 三角面 / 0.028 kg / 整腿 9.2%
#   · 它的 STL 仍保留在 meshes/ 里，随时把这里改回 True 即可复原
SHOW_SEG = {"L1": True, "L2": True, "L3": False}

# ------------------------------------------------------------------ 段归属（显式）
# L1 = 电机输出侧（绕髋轴驱动的那一段），L2 = 腿杆+绑缚环+导轨，L3 = 滑块+按键
SEG_L1 = ("电机_出轴", "电机_轴", "电机_黄铜轴套8_10_18", "腿部_顶丝_M4_4")
SEG_L3 = ("腿部_滑块_双键", "腿部_按键_双")
SEG_NAMES = ("L1", "L2", "L3")

# 零件名 -> 段（前缀匹配）；未命中的一律 L2
def seg_of(part_name):
    for k in SEG_L1:
        if k in part_name:
            return "L1"
    for k in SEG_L3:
        if k in part_name:
            return "L3"
    return "L2"


# ------------------------------------------------------------------ 工具
def cluster_faces(fl, tol_dir=1e-3, tol_dist=0.6):
    """fl = [(part, o(world), d(unit), r)] -> 两级聚类结果"""
    O = np.asarray([f[1] for f in fl]); D = np.asarray([f[2] for f in fl])
    RR = np.asarray([f[3] for f in fl]); PN = [f[0] for f in fl]
    n = len(fl)
    k = np.argmax(np.abs(D), axis=1)
    DC = D * np.where(D[np.arange(n), k] > 0, 1.0, -1.0)[:, None]
    reps, lab = [], np.full(n, -1)
    for i in range(n):
        if reps:
            cr = np.linalg.norm(np.cross(np.asarray(reps), DC[i]), axis=1)
            j = int(np.argmin(cr))
            if cr[j] <= tol_dir:
                lab[i] = j; continue
        reps.append(DC[i]); lab[i] = len(reps) - 1
    out = []
    for g in range(len(reps)):
        sub = []
        for i in np.where(lab == g)[0]:
            hit = False
            for s in sub:
                if np.linalg.norm(np.cross(s[1], DC[i])) > tol_dir:
                    continue
                v = O[i] - s[2]
                if np.linalg.norm(v - s[1] * float(v @ s[1])) <= tol_dist:
                    s[0].append(i); hit = True; break
            if not hit:
                sub.append([[i], DC[i].copy(), O[i].copy()])
        for s in sub:
            idx = np.asarray(s[0])
            if len(idx) < 4:
                continue
            out.append(dict(names=sorted({PN[i] for i in idx}),
                            radii=sorted({round(float(RR[i]), 3) for i in idx}),
                            d=s[1] / np.linalg.norm(s[1]),
                            pt=O[idx].mean(axis=0), n=len(idx)))
    return out


def pick_cluster(cls, keys, rmin, rmax, hip_a, orient):
    """在簇里挑出「含指定零件 + 含指定半径档 + 指定与髋轴关系」的那一个。"""
    best = None
    for c in cls:
        if not any(any(k in nm for k in keys) for nm in c["names"]):
            continue
        if not any(rmin <= r <= rmax for r in c["radii"]):
            continue
        ab = abs(float(c["d"] @ hip_a))
        ang = np.degrees(np.arccos(np.clip(ab, 0, 1)))
        ok = (ang > 85.0) if orient == "perp" else (ang < 5.0)
        if not ok:
            continue
        if best is None or c["n"] > best["n"]:
            best = c
    return best


def on_line_near(P, d, target):
    """把轴点 P 沿轴向平移到离 target 最近的那一点（保证仍在同一条直线上）。"""
    d = d / np.linalg.norm(d)
    return P + d * float((target - P) @ d)


def axis_line_gap(P, d, hip_p, hip_a):
    """候选轴线 与 髋轴线 的**线-线**距离(mm)。
    注意：不能用「点到髋轴的距离」代替 —— 对 ⊥ 轴，轴点可以沿轴任意滑动，
    点-轴距离会随轴点选择变化（实测同一根轴可算出 52 / 72mm 两个值）。
    两轴平行时退化为「点到轴的距离」。"""
    cr = np.cross(hip_a, d)
    nc = float(np.linalg.norm(cr))
    rel = P - hip_p
    if nc > 0.1:                                   # 不平行 -> 线-线
        return abs(float(rel @ (cr / nc)))
    return float(np.linalg.norm(rel - hip_a * float(rel @ hip_a)))


def fix_triangle(I, tol_rel=1e-3):
    w = np.sort(np.linalg.eigvalsh(I))
    need = max(0.0, (w[2] - w[0] - w[1])) * (1.0 + tol_rel)
    return (I + (need / 3.0) * ID3, need / 3.0) if need > 0 else (I, 0.0)


def inertia_xyz(I):
    return (I[0, 0], I[0, 1], I[0, 2], I[1, 1], I[1, 2], I[2, 2])


def um(v, nd=9):
    return " ".join("%.*f" % (nd, x) for x in v)


# ------------------------------------------------------------------ STL
def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        n = struct.unpack("<I", head[80:84])[0]
        f.seek(0, 2)
        if f.tell() != 84 + n * 50:
            raise RuntimeError("expect binary STL: %s" % path)
        f.seek(84)
        raw = np.fromfile(f, dtype=np.uint8, count=n * 50).reshape(n, 50)
        tri = raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
    return tri                                   # (n,3,3) 顶点世界系(mm)


def write_stl(path, tri, name="seg"):
    with open(path, "wb") as f:
        f.write((name[:79].ljust(80, "\0")).encode("ascii", "ignore"))
        f.write(struct.pack("<I", len(tri)))
        for t in tri:
            e1, e2 = t[1] - t[0], t[2] - t[0]
            nv = np.cross(e1, e2)
            ln = np.linalg.norm(nv)
            nv = nv / ln if ln > 0 else np.zeros(3)
            f.write(struct.pack("<12fH", *nv, *t[0], *t[1], *t[2], 0))


# ------------------------------------------------------------------ 主流程
def main():
    cad = CadGeometry(ROOT)
    geo = cad.build()
    hip_p = np.asarray(geo.axis_point, float)
    hip_a = np.asarray(geo.axis_dir, float); hip_a /= np.linalg.norm(hip_a)

    R = []                                     # 报告
    def say(s=""):
        R.append(s)

    say("== exo_multidof.urdf 生成报告 ==")
    say("髋轴: point=%s mm  dir=%s" % (np.round(hip_p, 4).tolist(), np.round(hip_a, 8).tolist()))
    say("")

    # base 沿用原来的三组合并（口径不变）
    def group_items(group):
        solids, warns = cad._map_solids(group)
        out = []
        for inst in cad.placed:
            if cad.group_of(inst) != group:
                continue
            Rm, t = inst.T[:3, :3], inst.T[:3, 3]
            rho, lab = density_of(inst.part_name, cad.densities, cad.fallback)
            for msb in inst.solids:
                sp = solids.get(msb)
                if sp is None:
                    continue
                out.append(dict(name=inst.part_name, rho=rho, label=lab,
                                m=rho * sp.volume_mm3 * MM3_TO_M3,
                                c=Rm @ sp.cog_local + t,
                                I=Rm @ sp.inertia_local @ Rm.T * rho * MM5_TO_KGM2_PER_RHO,
                                msb=msb))
        return out, warns

    def leg_faces(group):
        fl = []
        for inst in cad.placed:
            if cad.group_of(inst) != group:
                continue
            Rm, t = inst.T[:3, :3], inst.T[:3, 3]
            for msb in inst.solids:
                for c in cad.cyl.get(str(msb), []):
                    o = Rm @ np.asarray(c["o"], float) + t
                    d = Rm @ np.asarray(c["d"], float)
                    nd = float(np.linalg.norm(d))
                    if nd < 1e-12:
                        continue
                    fl.append((inst.part_name, o, d / nd, float(c["r"] or 0.0)))
        return fl

    def agg(items):
        m = sum(i["m"] for i in items)
        cog = sum(i["m"] * i["c"] for i in items) / m
        I = np.zeros((3, 3))
        for i in items:
            v = (i["c"] - cog) * 1e-3
            I += i["I"] + i["m"] * ((v @ v) * ID3 - np.outer(v, v))
        return m, cog, I

    # ---- base ----
    base_items = []
    for g in ("motor_L", "motor_R", "back"):
        it, _ = group_items(g)
        base_items += it
    m_base, cog_base, I_base = agg(base_items)

    legs = {}
    for group in ("leg_L", "leg_R"):
        items, warns = group_items(group)
        m_leg, cog_leg, I_leg = agg(items)
        fl = leg_faces(group)
        sgn = 1.0 if group.endswith("L") else -1.0
        side = group.split("_")[1]

        # 腿伸展方向
        v = cog_leg - hip_p
        u = v - hip_a * float(v @ hip_a); u /= np.linalg.norm(u)

        cls = cluster_faces(fl)
        # --- J2: 髋部横向 Ø8 轴系 ---
        c2 = pick_cluster(cls, ("电机_轴", "黄铜轴套", "电机_出轴"), 3.8, 4.2, hip_a, "perp")
        # --- J3: 绑缚销（∥髋轴, Ø7.6孔 × Ø8销）---
        c3 = pick_cluster(cls, ("腿部_绑缚", "腿部_滑块_双键"), 3.6, 4.2, hip_a, "para")
        assert c2 is not None and c3 is not None, "关节轴未找到 (c2=%s c3=%s)" % (c2 is not None, c3 is not None)

        # 分段
        seg = defaultdict(list)
        for i in items:
            seg[seg_of(i["name"])].append(i)
        props = {}
        for k in SEG_NAMES:
            props[k] = agg(seg[k]) if seg[k] else (0.0, cog_leg.copy(), np.zeros((3, 3)))

        # 关节原点：把拟合轴平移到场内最靠近该关节子段的质心处（仍在同一条直线上）
        O_hip = hip_p.copy()
        O_ab = on_line_near(c2["pt"], c2["d"], props["L2"][1])
        O_st = on_line_near(c3["pt"], c3["d"], props["L3"][1])
        # ⚠ J3 **不用** c3 的轴向当转轴：那根 ∥髋轴 的销是横穿滑块的**防转键**
        #   （「双键」= 两枚键，「按键_双」= 释放按钮）—— 键的作用恰恰是**阻止转动**。
        #   把它建成 revolute = 把被约束掉的自由度当成自由度（实测转 0.6rad 只推动
        #   段质心 3.98mm，正是"不该转"的证据）。真正的自由度是沿 s 的直线滑移。
        #   c3 现在只用来**定位**关节原点（销就在滑块上），轴向改用 S_SLIDE。
        O_sl = on_line_near(O_st, S_SLIDE, props["L3"][1])
        joints = [dict(name="hip", kind="revolute", axis=hip_a, O=O_hip, parent="base", child="L1"),
                  dict(name="abduct", kind="revolute", axis=c2["d"], O=O_ab, parent="L1", child="L2"),
                  dict(name="slide", axis=S_SLIDE, O=O_sl, parent="L2", child="L3",
                       lo=SLIDE_LO, hi=SLIDE_HI,
                       kind=("prismatic" if ENABLE_SLIDE else "fixed"))]

        # ---------------- 守恒校验 ----------------
        m_sum = sum(props[k][0] for k in SEG_NAMES)
        cog_sum = sum(props[k][0] * props[k][1] for k in SEG_NAMES) / m_sum
        I_sum = np.zeros((3, 3))
        for k in SEG_NAMES:
            mk, ck, Ik = props[k]
            d = (ck - cog_sum) * 1e-3
            I_sum += Ik + mk * ((d @ d) * ID3 - np.outer(d, d))

        say("#" * 104)
        say("## %s   原: m=%.9f kg  cog=%s mm" % (group, m_leg, np.round(cog_leg, 4).tolist()))
        say("#" * 104)
        say("  段  质量(kg)      质心(mm)                              零件")
        for k in SEG_NAMES:
            mk, ck, _ = props[k]
            names = sorted({i["name"] for i in seg[k]})
            say("  %-3s %10.6f   %-38s %s" % (k, mk, str(np.round(ck, 2).tolist()),
                                              ",".join(names)[:70]))
        say("  合计 %.9f kg   (原 %.9f, rel %.2e)   %s"
            % (m_sum, m_leg, abs(m_sum - m_leg) / m_leg,
               "OK" if abs(m_sum - m_leg) / m_leg < 1e-9 else "MISMATCH"))
        say("  合成质心 %s  vs 原 %s   |Δ|=%.3e mm"
            % (np.round(cog_sum, 4).tolist(), np.round(cog_leg, 4).tolist(),
               float(np.linalg.norm(cog_sum - cog_leg))))
        say("  绕原质心惯量差 max|ΔI| = %.3e kg·m²  (rel %.2e)"
            % (float(np.max(np.abs(I_sum - I_leg))), float(np.max(np.abs(I_sum - I_leg))) / float(np.max(np.abs(I_leg)))))
        say("")
        say("  关节 (原点=世界系 mm, 轴=世界系单位向量)")
        for j in joints:
            rhip = axis_line_gap(j["O"], j["axis"], hip_p, hip_a)
            ang = np.degrees(np.arccos(np.clip(abs(float(j["axis"] @ hip_a)), 0, 1)))
            say("    %-8s parent=%-4s child=%-4s  O=%s  轴=%s  与髋轴=%.2f°  距髋=%.1fmm"
                % (j["name"], j["parent"], j["child"], np.round(j["O"], 3).tolist(),
                   np.round(j["axis"], 6).tolist(), ang, rhip))
        say("")
        legs[group] = dict(props=props, joints=joints, seg=seg, m=m_leg,
                           cog=cog_leg, I=I_leg, u=u, c2=c2, c3=c3, side=side,
                           items=items)
        if warns:
            say("  (warnings: %s)" % warns)
        say("")

    # ---------------- 写 URDF ----------------
    def emit(path, lock_new):
        x = ['<?xml version="1.0"?>', "<!--",
             "  exo_multidof.urdf : 由 _stp2urdf_multidof.py 从真实装配体 STEP 生成",
             "  每条腿 3 段 / 3 个关节（hip 屈伸 + abduct 髋部横向被动轴 + slide 导轨滑移接口）",
             "  ★ 真正可动的只有 hip + abduct 两个；slide 的几何是移动副（轴 ⊥髋轴），"
             "    但复查后判定它的「行程」其实只是 2mm 装配间隙（滑块与导轨等长 72.00mm），"
             "    且工作时被双键/按键锁住、质量仅 28g ⇒ **本次按 fixed 忽略**。"
             "    恢复方法：把 ENABLE_SLIDE 改成 True（行程见 SLIDE_LO / SLIDE_HI）。",
             "  （再早的 strap(revolute, ∥髋轴) 已废止 —— 那根销是防转键，键阻止转动，不是铰链。）",
             "  ★ 渲染：%s（SHOW_SEG 控制，只删 <visual> 不删 <inertial>，动力学逐位不变）。" % (
                 "全部段都画" if all(SHOW_SEG.values())
                 else "已隐藏 " + ",".join(k for k, v in SHOW_SEG.items() if not v)
                      + " 段的 <visual>（滑块/按键抱在腿杆下端，slide 既已 fixed 就不再渲染）"),
             "  link frame 全部与世界系轴对齐（joint origin rpy = 0 0 0）",
             "  %s" % ("★ 本文件 abduct 也写成 fixed（回归用，应精确重现 2-DOF）"
                     if lock_new else "★ 本文件 abduct 为真实可动 joint，slide 为 fixed"),
             "-->", '<robot name="exo_multidof">']
        # base 的 link frame = 世界系（与 _stp2urdf.py 一致：inertial origin 取 cog - 髋轴点）
        def link(name, m, cog, I, O, visual):
            ixx, ixy, ixz, iyy, iyz, izz = inertia_xyz(I)
            out = ["  <link name=\"%s\">" % name, "    <inertial>",
                   "      <origin xyz=\"%s\" rpy=\"0 0 0\"/>" % um((cog - O) * 1e-3, 9),
                   "      <mass value=\"%.9f\"/>" % m,
                   "      <inertia ixx=\"%.12f\" ixy=\"%.12f\" ixz=\"%.12f\"" % (ixx, ixy, ixz),
                   "               iyy=\"%.12f\" iyz=\"%.12f\" izz=\"%.12f\"/>" % (iyy, iyz, izz),
                   "    </inertial>"]
            if visual:
                out += ["    <visual>", "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>",
                        "      <geometry>", "        <mesh filename=\"%s\"/>" % visual,
                        "      </geometry>", "      <material name=\"%s\">" % name.replace("_", ""),
                        "        <color rgba=\"%s\"/>" % ("0.55 0.57 0.62 1.0" if name == "base"
                                                       else ("0.20 0.45 0.85 1.0" if "_L" in name
                                                             else "0.85 0.35 0.20 1.0")),
                        "      </material>", "    </visual>"]
            out += ["  </link>"]
            return out

        x += link("base", m_base, cog_base, I_base, hip_p, "meshes/base.stl")
        for group, dd in legs.items():
            side = dd["side"]
            for k in SEG_NAMES:
                mk, ck, Ik = dd["props"][k]
                Ifix, add = fix_triangle(Ik)
                O = dd["joints"][0 if k == "L1" else (1 if k == "L2" else 2)]["O"]
                vf = "meshes/leg_%s_%s.stl" % (side, k[1])
                vf = vf if os.path.exists(os.path.join(SIM, vf)) else None
                if not SHOW_SEG.get(k, True):        # 只去渲染，惯性块照写
                    vf = None
                x += link("leg_%s_%s" % (side, k[1]), mk, ck, Ifix, O, vf)

        for group, dd in legs.items():
            side = dd["side"]
            prev_O = None
            for j, k in zip(dd["joints"], SEG_NAMES):
                O = j["O"]
                org = O if prev_O is None else (O - prev_O)
                jt = j["kind"]
                if lock_new and j["name"] != "hip":
                    jt = "fixed"
                x += ["  <joint name=\"%s_%s\" type=\"%s\">" % (j["name"], side, jt),
                      "    <parent link=\"%s\"/>" % ("base" if j["parent"] == "base"
                                                     else "leg_%s_%s" % (side, j["parent"][1])),
                      "    <child  link=\"leg_%s_%s\"/>" % (side, j["child"][1]),
                      "    <origin xyz=\"%s\" rpy=\"0 0 0\"/>" % um(org * 1e-3, 9)]
                if jt != "fixed":
                    if jt == "prismatic":
                        lim = "lower=\"%.6f\" upper=\"%.6f\" effort=\"50\" velocity=\"0.5\"" % (j["lo"], j["hi"])
                    else:
                        lim = "lower=\"-1.5\" upper=\"1.5\" effort=\"18\" velocity=\"10\""
                    x += ["    <axis xyz=\"%s\"/>" % um(j["axis"], 12),
                          "    <limit %s/>" % lim,
                          "    <dynamics damping=\"0.0\" friction=\"0.0\"/>"]
                x += ["  </joint>"]
                prev_O = O
        x += ["</robot>", ""]
        io.open(path, "w", encoding="utf-8", newline="\n").write("\n".join(x))
        return path

    # ---------------- 拆网格（必须在 emit 之前，emit 要判断每段网格是否存在）----------------
    say("-- 外形网格拆分 --")
    for group, dd in legs.items():
        side = dd["side"]
        src = os.path.join(MESH_DIR, "leg_%s.stl" % side)
        if not os.path.exists(src):
            say("  leg_%s.stl 缺失 -> 跳过" % side)
            continue
        tri = read_stl(src)
        # ⚠ leg_*.stl 的顶点单位是**米**（_stp2stl.py 输出世界系 m），而零件质心是 mm
        cen = tri.mean(axis=1) * 1000.0
        # L1|L2 的切割面放在「顶丝 与 片形腿杆连接件」之间的空隙（_multidof_plan.txt:
        # s2 上 顶丝=+6.0, 连接件=+14.5 -> 取 +10；且 s2 对轴点沿轴平移不变，故阈值可直接用）
        n2 = u - c2["d"] * float(u @ c2["d"]); n2 /= np.linalg.norm(n2)
        s2 = (cen - dd["joints"][1]["O"]) @ n2
        lab = np.where(s2 < 10.0, 0, -1)
        # L2|L3 不能用平面切：绑缚环 / 滑块 / 导轨 在空间上互相咬合（质心只差几 mm），
        # 任何平面都会把某个零件切开。改用「三角形质心离哪个零件质心最近」= Voronoi 归属，
        # 这样每个零件整体落到它该在的段里。
        rest = [i for i in dd["items"] if seg_of(i["name"]) != "L1"]
        if rest:
            C = np.asarray([i["c"] for i in rest])                  # (k,3) mm
            S = np.asarray([SEG_NAMES.index(seg_of(i["name"])) for i in rest])
            idx = np.where(lab < 0)[0]
            if len(idx):
                d2c = ((cen[idx][:, None, :] - C[None, :, :]) ** 2).sum(axis=2)
                lab[idx] = S[np.argmin(d2c, axis=1)]
        cnt = []
        for si, k in zip(range(3), SEG_NAMES):
            t = tri[lab == si]
            out = os.path.join(MESH_DIR, "leg_%s_%s.stl" % (side, k[1]))
            write_stl(out, t, "leg_%s_%s" % (side, k[1]))
            cnt.append((k, len(t)))
        say("  %s: %d 三角面 -> %s" % (side, len(tri),
                                        "  ".join("%s:%d" % (k, n) for k, n in cnt)))

    p1 = emit(OUT_URDF, False)
    p2 = emit(OUT_URDF_LOCK, True)
    say("")
    say("URDF        -> %s" % p1)
    say("URDF(locked)-> %s" % p2)
    io.open(OUT_RPT, "w", encoding="utf-8").write("\n".join(R) + "\n")
    print("\n".join(R))
    print("\nWROTE report")


if __name__ == "__main__":
    main()
