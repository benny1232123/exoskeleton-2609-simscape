# -*- coding: utf-8 -*-
"""
把 _stp2stl.py 导出的 STL 渲染成预览图（无需打开 MATLAB）。

用途
----
1. 立刻自查「网格是不是对的位置、对的大小、对的朝向」—— 比开 MATLAB 快得多，
   也便于贴进报告 / 组会 PPT。
2. 画一张**关节摆动对比图**：按 URDF 里的髋轴对腿做 Rodrigues 旋转，
   直观展示这个模型只有一个转动自由度、且转的就是那条轴。

注意：这只是「离线预览」。Simscape 里真正的显示由 Mechanics Explorer
读同一批 STL 完成（见 SOP 3.10 节）。

两个必要的坑
-----------
* **中文字体**：matplotlib 默认 DejaVu Sans 没有 CJK 字形，汉字会全被画成豆腐块，
  而且**只报警告、不报错**，极易漏掉。必须显式挂 Microsoft YaHei / SimHei。
* **抽稀**：base 有 74.8 万三角面，全画会让 Poly3DCollection 跑很久。
  按刚体分别抽稀（腿只有 4.8 万，不抽）。抽稀是"每 k 个取一个"，
  所以画面呈点状稀疏 —— 看图时记得这是**抽样**，不是网格有洞。
"""
from __future__ import annotations

import argparse
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# 中文标注必须显式挂 CJK 字体（否则整幅图的汉字都是豆腐块）
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

ROOT = r"C:\Users\29408\Desktop\外骨骼"
SIM = os.path.join(ROOT, "matlab2609", "simscape")
MESH_DIR = os.path.join(SIM, "meshes")
OUT_DIR = os.path.join(ROOT, "matlab2609", "out_simscape")

COLORS = {"base": (0.62, 0.63, 0.66),
          "leg_L": (0.18, 0.45, 0.85),
          "leg_R": (0.86, 0.34, 0.18)}
DECIMATE_CAP = {"base": 40000, "leg_L": 60000, "leg_R": 60000}
LIGHT = np.array([0.45, -0.35, 0.82])
LIGHT = LIGHT / np.linalg.norm(LIGHT)


def read_stl(path: str) -> np.ndarray:
    """读二进制 STL，返回 (n,3,3) float64。"""
    with open(path, "rb") as f:
        head = f.read(84)
        if len(head) < 84:
            raise ValueError("bad stl %s" % path)
        n = struct.unpack("<I", head[80:84])[0]
        body = f.read()
        if len(body) >= n * 50:
            arr = np.frombuffer(body[:n * 50], dtype=np.uint8).reshape(n, 50)
            return arr[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
    raise ValueError("not a supported STL: %s" % path)


def parse_urdf(path: str):
    """取每个 joint 的 (origin xyz, 单位轴)。"""
    root = ET.parse(path).getroot()
    out = {}
    for j in root.findall("joint"):
        child = j.find("child").get("link")
        o = np.array([float(x) for x in j.find("origin").get("xyz").split()])
        a = np.array([float(x) for x in j.find("axis").get("xyz").split()])
        out[child] = (o, a / np.linalg.norm(a))
    return out


def rot(a, q):
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def pose(tris, o, a, q):
    """绕「过 o、方向 a」的轴把刚体转 q，返回同形状 (n,3,3)。"""
    if abs(q) < 1e-12:
        return tris
    R = rot(a, q)
    return ((tris.reshape(-1, 3) - o) @ R.T + o).reshape(tris.shape)


def decimate(tris, cap):
    if cap <= 0 or len(tris) <= cap:
        return tris
    return tris[::int(np.ceil(len(tris) / cap))]


def add(ax, tris, rgb, cap, name=None):
    """把一组三角面加进 3D 轴，按面法向做简单朗伯着色。"""
    t = decimate(tris, cap)
    v0, v1, v2 = t[:, 0], t[:, 1], t[:, 2]
    nn = np.cross(v1 - v0, v2 - v0)
    ln = np.linalg.norm(nn, axis=1, keepdims=True)
    nn = np.divide(nn, np.where(ln > 0, ln, 1.0))
    inten = 0.35 + 0.65 * np.abs(nn @ LIGHT)
    fc = np.clip(np.asarray(rgb)[None, :] * inten[:, None], 0, 1)
    fc = np.concatenate([fc, np.ones((len(fc), 1))], axis=1)
    pc = Poly3DCollection(t, facecolors=fc, edgecolors="none", linewidths=0)
    if name:
        pc.set_label(name)
    ax.add_collection3d(pc)
    return len(t)


def frame(ax, xyz, title):
    ax.set_xlim(xyz[0]); ax.set_ylim(xyz[1]); ax.set_zlim(xyz[2])
    ax.set_box_aspect((np.ptp(xyz[0]), np.ptp(xyz[1]), np.ptp(xyz[2])))
    ax.set_xlabel("x / m", fontsize=7)
    ax.set_ylabel("y / m", fontsize=7)
    ax.set_zlabel("z / m", fontsize=7)
    ax.tick_params(labelsize=6)
    ax.set_title(title, fontsize=10)
    ax.grid(True, alpha=0.25)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--q", type=float, default=0.35, help="摆位视图的髋角 (rad)")
    ap.add_argument("--out", default=os.path.join(OUT_DIR, "exo_real_mesh_preview.png"))
    ap.add_argument("--base-cap", type=int, default=DECIMATE_CAP["base"])
    ap.add_argument("--leg-cap", type=int, default=DECIMATE_CAP["leg_L"])
    args = ap.parse_args()

    names = ["base", "leg_L", "leg_R"]
    cap = {"base": args.base_cap, "leg_L": args.leg_cap, "leg_R": args.leg_cap}
    tri = {}
    for n in names:
        p = os.path.join(MESH_DIR, "%s.stl" % n)
        if not os.path.exists(p):
            raise SystemExit("缺少 %s —— 先跑 _stp2stl.py" % p)
        tri[n] = read_stl(p)
        print("%-6s %7d tri  bbox z=[%.4f, %.4f]" % (n, len(tri[n]),
                                                     tri[n][..., 2].min(),
                                                     tri[n][..., 2].max()))

    joints = parse_urdf(os.path.join(SIM, "exo_real.urdf"))
    print("joints:", {k: (np.round(v[0], 6).tolist(), np.round(v[1], 6).tolist())
                      for k, v in joints.items()})

    allp = np.concatenate([t.reshape(-1, 3) for t in tri.values()], axis=0)
    lo, hi = allp.min(0), allp.max(0)
    pad = 0.03
    xyz = [(lo[i] - pad, hi[i] + pad) for i in range(3)]
    print("bbox min=%s max=%s m" % (np.round(lo, 4).tolist(), np.round(hi, 4).tolist()))

    dec = ", ".join("%s %d→%d" % (n, len(tri[n]), len(decimate(tri[n], cap[n])))
                    for n in names)
    fig = plt.figure(figsize=(15.5, 10.5))

    views = [((22, -60), "(a) 等轴测  q = 0（CAD 零位）"),
             ((0, -90),  "(b) 正视（沿 +y 看）"),
             ((0, 0),    "(c) 侧视（沿 +x 看）"),
             ((90, -90), "(d) 俯视")]
    for k, (elaz, ttl) in enumerate(views, start=1):
        ax = fig.add_subplot(2, 3, k, projection="3d")
        for n in names:
            add(ax, tri[n], COLORS[n], cap[n])
        frame(ax, xyz, ttl)
        ax.view_init(elev=elaz[0], azim=elaz[1])
        try:
            ax.set_proj_type("ortho")
        except Exception:
            pass

    q = args.q
    ax = fig.add_subplot(2, 3, 5, projection="3d")
    add(ax, tri["base"], COLORS["base"], cap["base"], "base")
    for n, sgn in (("leg_L", -1.0), ("leg_R", +1.0)):
        o, a = joints[n]
        add(ax, pose(tri[n], o, a, sgn * q), COLORS[n], cap[n], n)
    frame(ax, xyz, "(e) 摆位  leg_L q=%+0.2f, leg_R q=%+0.2f rad" % (-q, q))
    ax.view_init(elev=22, azim=-60)
    try:
        ax.set_proj_type("ortho")
    except Exception:
        pass

    ax = fig.add_subplot(2, 3, 6, projection="3d")
    for n in names:
        add(ax, tri[n], COLORS[n], cap[n])
    o, a = joints["leg_L"]
    L = 0.16
    ax.plot([o[0] - L * a[0], o[0] + L * a[0]],
            [o[1] - L * a[1], o[1] + L * a[1]],
            [o[2] - L * a[2], o[2] + L * a[2]], "-", color="k", lw=2.0)
    com = np.array([0.26061, 0.16167, -0.16512])
    ax.plot([com[0]], [com[1]], [com[2]], "o", color="k", ms=5)
    ax.plot([com[0], com[0]], [com[1], com[1]], [com[2], -0.33], "k--", lw=1.0)
    ax.text(com[0], com[1], -0.35, "CoM（重力方向）", fontsize=7, ha="center", va="top")
    frame(ax, xyz, "(f) 髋轴（黑线）与 leg_L 质心（黑点）")
    ax.view_init(elev=22, azim=-60)
    try:
        ax.set_proj_type("ortho")
    except Exception:
        pass

    fig.suptitle("exo_real：真实装配体 STEP → URDF → Simscape 的可视化网格\n"
                 "顶点＝世界系（m），与 <inertial> 同源；抽样绘制：%s" % dec,
                 fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    os.makedirs(OUT_DIR, exist_ok=True)
    fig.savefig(args.out, dpi=110, facecolor="white")
    print("WROTE", args.out)


if __name__ == "__main__":
    main()
