# -*- coding: utf-8 -*-
"""
_multidof_anim.py —— 把 4-DOF 轨迹 CSV 渲染成离线 3D 动画（**可复用于任意工况**）
============================================================================

为什么要这个
------------
Mechanics Explorer 只能在 MATLAB GUI 里看，截不了、也分享不了。
本脚本用**完全相同的网格 + 相同的前向运动学**离线渲染，
让 3D 效果进得了文档、也进得了对话。

★ 轨迹来源 = Simscape 自己导出的 `out_simscape/multidof_traj.csv`
  （由 `dump_multidof_traj.m` 生成，限位已关、g=[0 0 -9.81]）。
  所以默认动画**不是"另一个模型的动画"**，就是 Simscape 那条轨迹的可视化。

命令行
------
    python _multidof_anim.py                                # 默认：被动自由落体
    python _multidof_anim.py <csv> <out.gif> <out_strip.png> "<标题首行>" "<落款>"

作为模块（`_multidof_conditions.py` 就靠这个跑多工况）
    from _multidof_anim import render_csv
    t, qq = render_csv(csv, gif, strip, head="…工况…", footer="…")

输出
----
  matlab2609/out_simscape/multidof_drop.gif          运动动画（可循环播放）
  matlab2609/out_simscape/multidof_drop_strip.png    6 格胶片（静态，给文档用）

见 also MULTIDOF_3D_HOWTO.md、_multidof_render.py
"""
import io
import os
import sys

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)

from _multidof_render import (            # noqa: E402
    SIM, COLOR, parse_urdf, link_fk, entry_origin, load_solid_meshes, shade_faces,
)

OUTD = os.path.join(ROOT, "matlab2609", "out_simscape")
CSV = os.path.join(OUTD, "multidof_traj.csv")
URDF = os.path.join(SIM, "exo_multidof.urdf")
OUT_GIF = os.path.join(OUTD, "multidof_drop.gif")
OUT_STRIP = os.path.join(OUTD, "multidof_drop_strip.png")

HEAD0 = "sm_exo_multidof  4-DOF 自由落体   t = %.2f s"
FOOT0 = "绿 = abduct 轴（新自由度）   轨迹源：Simscape multidof_traj.csv"

QS = ["hip_L", "abduct_L", "hip_R", "abduct_R"]   # URDF 里的 4 个 revolute
NFRAME = 60
TARGETF = 700          # 每段减面目标（动画要逐帧重画，比静态图更省）

_ASSETS = {}


def load_assets(target_faces=TARGETF):
    """解析 URDF + 读网格（进程内缓存；多工况连续渲染只读一次）。"""
    if "a" not in _ASSETS:
        links, joints, inert, has_vis = parse_urdf(URDF)
        print("-- 减面（实体着色渲染）--")
        SOLID, skipped = load_solid_meshes(links, has_vis, target_faces=target_faces)
        drawn = [nm for nm in links if nm in SOLID]
        _ASSETS["a"] = (SOLID, joints, drawn)
    return _ASSETS["a"]


def load_traj(csv=None, qs=QS):
    csv = csv or CSV
    with io.open(csv, encoding="utf-8") as f:
        hdr = f.readline().strip().split(",")
    d = np.loadtxt(csv, delimiter=",", skiprows=1)
    col = {h: i for i, h in enumerate(hdr)}
    for q in qs:
        if q + ".q" not in col:
            raise SystemExit("CSV 里找不到 %s.q —— 先确认轨迹导出脚本" % q)
    t = d[:, 0]
    qq = np.column_stack([d[:, col[q + ".q"]] for q in qs])
    return t, qq


def render_csv(csv, out_gif, out_strip, head, footer,
               nframe=NFRAME, qs=QS, strip_title=None):
    """把 csv 的 q(t) 渲染成 GIF + 6 格胶片。head 里可含 % 占位（t 与各 q）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    from PIL import Image

    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    SOLID, joints, drawn = load_assets()
    t, qq = load_traj(csv, qs)
    tt = np.linspace(t[0], t[-1], nframe)

    def qmap_at(tk):
        return {qs[i]: float(np.interp(tk, t, qq[:, i])) for i in range(len(qs))}

    def cloud(M, nm):
        """段 nm 在配置 M 下的世界顶点 (nV,3)。"""
        V, _F = SOLID[nm]
        T = M[nm]
        o = entry_origin(joints, nm)
        return (T[:3, :3] @ (V - o).T).T + T[:3, 3]

    def add_solid(ax, M, nm):
        """把段 nm 作为**着色实体面**画上去。"""
        _V, F = SOLID[nm]
        P = cloud(M, nm)
        tris = P[F]
        pc = Poly3DCollection(tris,
                              facecolors=shade_faces(tris, COLOR.get(nm, "#999")),
                              edgecolors="none", linewidths=0, antialiased=False)
        ax.add_collection3d(pc)

    # ---- 先扫一遍，定出固定视角的坐标范围（不让镜头跟着抖）----
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for tk in tt:
        M = link_fk(joints, qmap_at(tk))
        for nm in drawn:
            P = cloud(M, nm)
            lo = np.minimum(lo, P.min(axis=0))
            hi = np.maximum(hi, P.max(axis=0))
    mid = (lo + hi) / 2.0
    rad = max((hi - lo).max() / 2.0, 0.1) * 1.06

    def setup_ax(ax):
        ax.set_xlim(mid[0] - rad, mid[0] + rad)
        ax.set_ylim(mid[1] - rad, mid[1] + rad)
        ax.set_zlim(mid[2] - rad, mid[2] + rad)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=18, azim=-62)
        ax.grid(alpha=.22)
        ax.set_xlabel("x / m", fontsize=8)
        ax.set_ylabel("y / m", fontsize=8)
        ax.set_zlabel("z / m", fontsize=8)
        ax.tick_params(labelsize=7)
        # 视野半宽 ~0.44 m，默认刻度会挤成一片带小数点的长标签 —— 限到 5 格
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_major_locator(MaxNLocator(5))

    def paint(ax, tk, tag=""):
        """把配置 tk 画到 ax 上。tag='' 则不写标题（胶片用）。"""
        qm = qmap_at(tk)
        M = link_fk(joints, qm)
        for nm in drawn:
            add_solid(ax, M, nm)
        # 髋轴（∥Y，原有自由度）与 abduct 轴（⊥，新自由度）各画一小段
        for j in joints:
            if j["name"] not in ("abduct_L", "abduct_R"):
                continue
            Mp = M[j["parent"]]
            o = j["o"]; d = j["axis"]
            s_ = np.linspace(-0.045, 0.045, 2)
            L = (Mp[:3, :3] @ (o[:, None] + np.outer(d, s_))).T + Mp[:3, 3]
            ax.plot(L[:, 0], L[:, 1], L[:, 2], "-", lw=2.2, color="#00b050")
        if tag:
            ax.set_title(tag, fontsize=10)

    # ================================================================ GIF
    fig = plt.figure(figsize=(6.6, 7.0))
    ax = fig.add_subplot(111, projection="3d")
    frames = []
    for k, tk in enumerate(tt):
        ax.clear()
        setup_ax(ax)
        qm = qmap_at(tk)
        paint(ax, tk)
        ax.set_title(head % ((tk,) + tuple(qm[q] for q in qs)), fontsize=10.5)
        if footer:
            ax.text2D(0.02, 0.02, footer, transform=ax.transAxes,
                      fontsize=8, color="#444")
        fig.tight_layout()
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())).convert("RGB"))
        if (k + 1) % 10 == 0:
            print("  frame %d/%d" % (k + 1, nframe))

    frames[0].save(out_gif, save_all=True, append_images=frames[1:],
                   duration=60, loop=0, optimize=True)
    print("WROTE", out_gif)

    # ============================================================ 胶片
    picks = np.linspace(0, nframe - 1, 6).round().astype(int)
    fig2 = plt.figure(figsize=(17, 10))
    for i, k in enumerate(picks):
        a = fig2.add_subplot(2, 3, i + 1, projection="3d")
        setup_ax(a)
        paint(a, tt[k], "(t = %.2f s)" % tt[k])
    fig2.suptitle(strip_title or head % ((tt[0],) + tuple(qmap_at(tt[0])[q] for q in qs)),
                  fontsize=12.5, y=0.97)
    fig2.tight_layout(rect=[0, 0, 1, 0.94])
    fig2.savefig(out_strip, dpi=120)
    print("WROTE", out_strip)
    plt.close("all")
    return t, qq


def main():
    argv = sys.argv[1:]
    csv, gif, strip = CSV, OUT_GIF, OUT_STRIP
    head, foot, stitle = HEAD0, FOOT0, None
    if len(argv) >= 3:
        csv, gif, strip = argv[0], argv[1], argv[2]
    if len(argv) >= 4:
        head = argv[3]
    if len(argv) >= 5:
        foot = argv[4]
    if len(argv) >= 6:
        stitle = argv[5]

    t, qq = render_csv(csv, gif, strip, head, foot, strip_title=stitle)

    # ---- 摆幅自检：确认 abduct 真被激励（否则动画看不出区别）----
    print("")
    print("=" * 74)
    print("摆幅自检（abduct 若≈0 则动画会退化）")
    print("=" * 74)
    for i, qn in enumerate(QS):
        r = qq[:, i].max() - qq[:, i].min()
        print("  %-10s  %.6f rad  (%.2f deg)   [%.4f, %.4f]"
              % (qn, r, np.degrees(r), qq[:, i].min(), qq[:, i].max()))
    print("")
    print("ANIM_MULTIDOF_OK")


if __name__ == "__main__":
    main()
