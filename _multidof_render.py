# -*- coding: utf-8 -*-
"""
把「加完自由度的模型」画出来
============================
直接用 URDF 的拓扑做前向运动学（不依赖 MATLAB），把 exo_multidof.urdf 的
6 个关节逐个摆出来，证明新加的两个自由度真的在动。

URDF 语义（本工程统一口径）
  · 所有 link frame 与世界系**轴对齐**，joint <origin rpy="0 0 0">
  · M_child = M_parent · Trans(o_j) · R(axis_j, q_j)
  · 第 k 段的 STL 顶点存的是**零位世界系坐标** => p_world = M_k · (p_stl − o_entry_k)

输出: matlab2609/out_simscape/multidof_poses.png

★ 本文件同时是被 `_multidof_anim.py` 复用的**运动学模块**：
  可复用的东西（parse_urdf / rot / read_stl / link_fk / entry_origin /
  load_meshes / COLOR / SIM / ROOT）都放在模块层，出图逻辑收在 main() 里，
  所以 `import _multidof_render` **不会**触发任何绘图或写文件。
"""
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
SIM = os.path.join(ROOT, "matlab2609", "simscape")
URDF = os.path.join(SIM, "exo_multidof.urdf")
OUT = os.path.join(ROOT, "matlab2609", "out_simscape", "multidof_poses.png")

COLOR = {"base": "#8c8f96", "leg_L_1": "#4c86e0", "leg_L_2": "#1f4fa8",
         "leg_L_3": "#7fb2f0", "leg_R_1": "#e07a4c", "leg_R_2": "#a83a1f",
         "leg_R_3": "#f0b27f"}


# ---------------------------------------------------------------- URDF 解析
def parse_urdf(path):
    """返回 (links, joints, inert, has_vis)。

    ★ has_vis：哪些 link 在 URDF 里**真的带 <visual>**。
      必须照它决定画不画——否则本脚本会从磁盘把所有 STL 都读来画，
      与 Mechanics Explorer（只渲染 <visual> 声明的网格）不一致。
      （加这一步的起因：slide 按 fixed 忽略后，用户要求在 3D 里消掉滑块，
        URDF 已去掉 leg_*_3 的 <visual>，但旧的渲染仍会把滑块画出来。）
    """
    root = ET.parse(path).getroot()
    links = [l.get("name") for l in root.findall("link")]
    inert = {}
    has_vis = {}
    for l in root.findall("link"):
        io_ = l.find("inertial/origin")
        if io_ is not None:
            inert[l.get("name")] = np.array([float(x) for x in io_.get("xyz").split()])
        has_vis[l.get("name")] = l.find("visual") is not None
    joints = []
    for j in root.findall("joint"):
        o = np.array([float(x) for x in j.find("origin").get("xyz").split()])
        ax = j.find("axis")
        d = np.array([float(x) for x in ax.get("xyz").split()]) if ax is not None else None
        joints.append(dict(name=j.get("name"), type=j.get("type"),
                           parent=j.find("parent").get("link"),
                           child=j.find("child").get("link"), o=o, axis=d))
    return links, joints, inert, has_vis


def rot(a, q):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def read_stl(path):
    """返回 (n,3,3) 的三角形顶点数组（散点渲染用）。"""
    with open(path, "rb") as f:
        head = f.read(84)
        n = struct.unpack("<I", head[80:84])[0]
        f.seek(84)
        raw = np.fromfile(f, dtype=np.uint8, count=n * 50).reshape(n, 50)
    return raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


# ---------------------------------------------------------------- 实体面渲染
# 为什么需要这一套：散点云（上面的 read_stl 路径）看不出"是个零件"，
# 用户会说「没看到 3D 模型」。要像模型就得画**着色实体面**。
# 但 base.stl 有 74.8 万面，Poly3DCollection 直接画会卡死 ⇒ 必须先减面。
def read_stl_faces(path):
    """返回 (V, F)：V = (nV,3)，F = (nF,3) int。顶点未去重，但面拓扑完整。"""
    tri = read_stl(path)
    nF = tri.shape[0]
    V = tri.reshape(-1, 3)
    F = np.arange(nF * 3, dtype=np.int64).reshape(nF, 3)
    return V, F


def decimate_cluster(V, F, target_faces, max_res=160):
    """顶点聚类减面（无外部依赖）。

    把顶点按 bbox 均分的立方网格吸附，同格点合并成一个代表点，
    再丢掉退化面（有两个角落在同一格）与重复面。
    视觉上保留外形轮廓，面数可压两个数量级。

    返回 (V2, F2)。若已 <= target_faces 则原样返回。
    """
    if F.shape[0] <= target_faces:
        return V, F
    lo = V.min(axis=0)
    hi = V.max(axis=0)
    span = np.where(hi - lo > 0, hi - lo, 1.0)

    def build(res):
        cell = span / res
        idx = np.clip(np.floor((V - lo) / cell).astype(np.int64), 0, res - 1)
        key = idx[:, 0] + res * (idx[:, 1] + res * idx[:, 2])
        uniq, inv = np.unique(key, return_inverse=True)
        # 新顶点 = 每簇的均值
        cnt = np.bincount(inv, minlength=len(uniq)).astype(np.float64)
        acc = np.zeros((len(uniq), 3))
        for d in range(3):
            acc[:, d] = np.bincount(inv, weights=V[:, d], minlength=len(uniq))
        Vn = acc / cnt[:, None]
        Fn = inv[F]
        keep = (Fn[:, 0] != Fn[:, 1]) & (Fn[:, 1] != Fn[:, 2]) & (Fn[:, 0] != Fn[:, 2])
        Fn = Fn[keep]
        if len(Fn):
            Fs = np.sort(Fn, axis=1)
            _, u = np.unique(Fs, axis=0, return_index=True)
            Fn = Fn[np.sort(u)]
        return Vn, Fn

    # 二分找刚好 <= target 的网格分辨率（单调：res 越大面越多）
    best = build(4)
    if best[1].shape[0] <= target_faces:
        for res in (6, 8, 12, 16, 24, 32, 48, 64, 96, max_res):
            cur = build(res)
            if cur[1].shape[0] <= target_faces:
                best = cur
            else:
                break
        return best
    return best


def load_solid_meshes(links, has_vis, target_faces=900, verbose=True):
    """像 load_meshes，但返回**带面拓扑、已减面**的实体网格。

    返回 (SOLID, skipped)：SOLID = {link: (V, F)}。
    """
    SOLID = {}
    skipped = []
    for nm in links:
        p = os.path.join(SIM, "meshes", "%s.stl" % nm)
        if os.path.exists(p) and has_vis.get(nm, True):
            V, F = read_stl_faces(p)
            V2, F2 = decimate_cluster(V, F, target_faces)
            SOLID[nm] = (V2, F2)
            if verbose:
                print("   solid %-12s %7d -> %6d faces" % (nm, F.shape[0], F2.shape[0]))
        elif os.path.exists(p):
            skipped.append(nm)
    return SOLID, skipped


def shade_faces(P, base, light=(-0.35, -0.55, 0.76)):
    """按面法向做 Lambert 着色，返回每面的 RGBA。

    P = (nF,3,3) 世界顶点。base 为十六进制色串。
    用固定光向 + 环境光下限，避免背面全黑看不出形状。
    """
    e1 = P[:, 1] - P[:, 0]
    e2 = P[:, 2] - P[:, 0]
    n = np.cross(e1, e2)
    ln = np.linalg.norm(n, axis=1)
    ln[ln == 0] = 1.0
    n = n / ln[:, None]
    L = np.asarray(light, dtype=np.float64)
    L = L / np.linalg.norm(L)
    lam = np.abs(n @ L)                      # 取绝对値 ⇒ 双面可见
    inten = 0.38 + 0.62 * lam
    rgb = np.array([int(base[1:3], 16), int(base[3:5], 16), int(base[5:7], 16)]) / 255.0
    out = np.clip(inten[:, None] * rgb[None, :], 0, 1)
    return np.concatenate([out, np.ones((len(P), 1))], axis=1)


def link_fk(joints, qmap):
    """返回 link -> 4x4 世界变换 M（零位时 = Trans(o) 链）。"""
    M = {"base": np.eye(4)}
    order = [j for j in joints]
    changed = True
    while changed:
        changed = False
        for j in order:
            if j["parent"] in M and j["child"] not in M:
                T = np.eye(4)
                T[:3, 3] = j["o"]
                R = np.eye(4)
                if j["type"] == "revolute":
                    R[:3, :3] = rot(j["axis"], qmap.get(j["name"], 0.0))
                elif j["type"] == "prismatic":
                    # 移动副：子 frame 沿轴**平移** q（米），不旋转
                    R[:3, 3] = j["axis"] * qmap.get(j["name"], 0.0)
                M[j["child"]] = M[j["parent"]] @ T @ R
                changed = True
    return M


def entry_origin(joints, link):
    for j in joints:
        if j["child"] == link:
            return j["o"]
    return np.zeros(3)


def load_meshes(links, has_vis, max_pts=9000, seed=0):
    """读 STL 顶点云（**只读 URDF 声明了 <visual> 的 link**）。

    返回 (MESH, skipped)：MESH = {link: (N,3) 顶点}，skipped = 有 STL 但被隐藏的 link。
    max_pts 控制每段抽多少个点 —— 做动画时可以调小换速度。
    """
    MESH = {}
    skipped = []
    for nm in links:
        p = os.path.join(SIM, "meshes", "%s.stl" % nm)
        if os.path.exists(p) and has_vis.get(nm, True):
            V = read_stl(p).reshape(-1, 3)
            rng = np.random.default_rng(seed)
            MESH[nm] = V[rng.choice(len(V), size=min(max_pts, len(V)), replace=False)]
        elif os.path.exists(p):
            skipped.append(nm)
    return MESH, skipped


def seg_com_mm(M, link, inert):
    """段质心世界系坐标 (mm)。URDF 长度单位为 m，故 ×1000。"""
    if link not in inert:
        return None
    return (M[link][:3, :3] @ inert[link] + M[link][:3, 3]) * 1000.0


# ---------------------------------------------------------------- 主流程
def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

    links, joints, inert, has_vis = parse_urdf(URDF)
    jnames = [j["name"] for j in joints]
    print("joints:", jnames)
    print("-- 减面（实体着色渲染用；散点版看不出「是个零件」）--")
    SOLID, skipped = load_solid_meshes(links, has_vis, target_faces=900)
    print("drawn :", sorted(SOLID))
    print("hidden:", sorted(skipped), "(URDF 里没有 <visual>，与 Mechanics Explorer 一致)")

    def draw(ax, qmap, title, elev=22, azim=-60):
        M = link_fk(joints, qmap)
        for nm, (V, F) in SOLID.items():
            T = M[nm]
            o = entry_origin(joints, nm)
            P = (T[:3, :3] @ (V - o).T).T + T[:3, 3]        # (nV,3) 世界顶点
            tris = P[F]                                      # (nF,3,3)
            pc = Poly3DCollection(tris,
                                  facecolors=shade_faces(tris, COLOR.get(nm, "#999")),
                                  edgecolors="none", linewidths=0, antialiased=False)
            ax.add_collection3d(pc)
        # 画新关节的轴
        for j in joints:
            if j["name"] in ("abduct_L", "abduct_R", "slide_L", "slide_R") \
                    and j["axis"] is not None:      # fixed 关节没有 <axis> 元素
                Mp = M[j["parent"]]
                o = j["o"]; d = j["axis"]
                t = np.linspace(-0.035, 0.035, 2)
                L = (Mp[:3, :3] @ (o[:, None] + np.outer(d, t))).T + Mp[:3, 3]
                ax.plot(L[:, 0], L[:, 1], L[:, 2], "-", lw=2.4,
                        color="#00a000" if "abduct" in j["name"] else "#c000c0")
        ax.set_title(title, fontsize=10.5)
        ax.set_xlabel("x / m"); ax.set_ylabel("y / m"); ax.set_zlabel("z / m")
        ax.set_xlim(-0.12, 0.45); ax.set_ylim(-0.30, 0.30); ax.set_zlim(-0.42, 0.10)
        ax.view_init(elev=elev, azim=azim)
        try:
            ax.set_box_aspect((0.57, 0.60, 0.52))
        except Exception:
            pass
        ax.grid(alpha=.2)

    def q(**kw):
        d = {}
        for k, v in kw.items():
            d[k + "_L"] = v
            d[k + "_R"] = v
        return d

    fig = plt.figure(figsize=(17, 10))
    panels = [
        (q(), "(a) 零位（每腿 3 个关节都在 0；L3 段的 <visual> 已隐藏）"),
        (q(hip=0.35), "(b) 髋屈伸 hip = +0.35 rad（原有自由度）"),
        (q(abduct=0.45), "(c) 髋部横向 abduct = +0.45 rad（新，绕绿轴转）"),
        (q(abduct=-0.45), "(d) abduct = −0.45 rad（反向，腿向内/外）"),
        (q(slide=0.0186), "(e) slide 已按 fixed 忽略：指令 +18.6mm 但**不动**"),
        (dict(hip_L=0.30, hip_R=-0.30, abduct_L=0.35, abduct_R=-0.35,
              slide_L=0.0186, slide_R=-0.0243), "(f) 复合：hip+abduct 同时动（slide 不动）"),
    ]
    for i, (qm, ttl) in enumerate(panels):
        ax = fig.add_subplot(2, 3, i + 1, projection="3d")
        draw(ax, qm, ttl)
        # 标出关节名
        if i == 0:
            ax.text2D(0.02, 0.96, "绿=abduct 轴（转）", transform=ax.transAxes,
                      fontsize=9, color="#333")

    fig.suptitle("exo_multidof.urdf —— 可动自由度 = 每腿 hip 屈伸 + abduct 髋部横向（共 4 DOF）；"
                 "导轨滑移 slide 判定为「2mm 间隙非行程」已按 fixed 忽略，"
                 "其滑块/按键(L3)的 <visual> 亦已隐藏（质量/惯量仍保留，动力学不变）",
                 fontsize=12.0, y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=125)
    print("WROTE", OUT)

    # 数值自检：各配置下每段质心（世界系，mm）相对零位的位移
    # 段质心世界系 = M_seg · inert_seg，其中 inert_seg 是 URDF 里 <inertial><origin xyz>
    # （= 段 CoM − 段入口关节原点，世界系口径）
    M0 = link_fk(joints, {})
    COM0 = {nm: seg_com_mm(M0, nm, inert) for nm in ("leg_L_1", "leg_L_2", "leg_L_3")}

    print("")
    print("=" * 92)
    print("数值自检 · 各段质心位移 (mm)   [相对零位；>1mm 才认为是真动]")
    print("=" * 92)
    print("%-52s %12s %12s %12s" % ("配置", "L1段", "L2段", "L3段"))
    for qm, name in panels:
        M = link_fk(joints, qm)
        cells = []
        for nm in ("leg_L_1", "leg_L_2", "leg_L_3"):
            c = seg_com_mm(M, nm, inert)
            cells.append(np.linalg.norm(c - COM0[nm]) if c is not None else float("nan"))
        print("%-52s %12.2f %12.2f %12.2f" % (name, cells[0], cells[1], cells[2]))

    # 单独把 slide / abduct 的效应量化
    print("")
    for tag, kw, tgt in (("slide 指令+18.6mm", q(slide=0.0186), "leg_L_3"),
                         ("slide 指令-24.3mm", q(slide=-0.0243), "leg_L_3"),
                         ("abduct=+0.45rad", q(abduct=0.45), "leg_L_2")):
        Mx = link_fk(joints, kw)
        cx = seg_com_mm(Mx, tgt, inert)
        print("%-16s %s 段质心 %s -> %s mm   位移 %.2f mm" % (
            tag, tgt,
            np.array2string(COM0[tgt], precision=1),
            np.array2string(cx, precision=1),
            np.linalg.norm(cx - COM0[tgt])))


if __name__ == "__main__":
    main()
