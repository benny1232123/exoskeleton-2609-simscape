# -*- coding: utf-8 -*-
"""
_multidof_anim.py —— 把 `sm_exo_multidof` 的 4-DOF 运动渲染成 3D 动画
============================================================================

为什么要这个
------------
Mechanics Explorer 只能在 MATLAB GUI 里看，截不了、也分享不了。
本脚本用**完全相同的网格 + 相同的前向运动学**离线渲染，
让 3D 效果进得了文档、也进得了对话。

★ 轨迹来源 = **Simscape 自己导出的** `out_simscape/multidof_traj.csv`
  （由 `dump_multidof_traj.m` 生成，4763 点、t∈[0,2.5]、限位已关、g=[0 0 -9.81]）。
  所以这动画**不是"另一个模型的动画"**，就是 Simscape 那条轨迹的可视化 ——
  与 Mechanics Explorer 里看到的逐位相同。

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

QS = ["hip_L", "abduct_L", "hip_R", "abduct_R"]   # URDF 里的 4 个 revolute
NFRAME = 60
TARGETF = 700          # 每段减面目标（动画要逐帧重画，比静态图更省）


def load_traj():
    with io.open(CSV, encoding="utf-8") as f:
        hdr = f.readline().strip().split(",")
    d = np.loadtxt(CSV, delimiter=",", skiprows=1)
    col = {h: i for i, h in enumerate(hdr)}
    for q in QS:
        if q + ".q" not in col:
            raise SystemExit("CSV 里找不到 %s.q —— 先把 dump_multidof_traj.m 跑一遍" % q)
    t = d[:, 0]
    qq = np.column_stack([d[:, col[q + ".q"]] for q in QS])
    return t, qq


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    from PIL import Image

    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    links, joints, inert, has_vis = parse_urdf(URDF)
    print("-- 减面（实体着色渲染）--")
    SOLID, skipped = load_solid_meshes(links, has_vis, target_faces=TARGETF)
    drawn = [nm for nm in links if nm in SOLID]

    t, qq = load_traj()
    tt = np.linspace(t[0], t[-1], NFRAME)

    def qmap_at(tk):
        return {QS[i]: float(np.interp(tk, t, qq[:, i])) for i in range(len(QS))}

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
    print("视野: center = %s  half-size = %.4f m" % (np.round(mid, 4), rad))

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
        ax.set_title("sm_exo_multidof  4-DOF 自由落体   t = %.2f s\n"
                     "hip_L %+.2f  abduct_L %+.2f  hip_R %+.2f  abduct_R %+.2f  [rad]"
                     % (tk, qm["hip_L"], qm["abduct_L"], qm["hip_R"], qm["abduct_R"]),
                     fontsize=10.5)
        ax.text2D(0.02, 0.02, "绿 = abduct 轴（新自由度）   "
                              "轨迹源：Simscape multidof_traj.csv",
                  transform=ax.transAxes, fontsize=8, color="#444")
        fig.tight_layout()
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())).convert("RGB"))
        if (k + 1) % 10 == 0:
            print("  frame %d/%d" % (k + 1, NFRAME))

    frames[0].save(OUT_GIF, save_all=True, append_images=frames[1:],
                   duration=60, loop=0, optimize=True)
    print("WROTE", OUT_GIF)

    # ============================================================ 胶片
    picks = np.linspace(0, NFRAME - 1, 6).round().astype(int)
    fig2 = plt.figure(figsize=(17, 10))
    for i, k in enumerate(picks):
        a = fig2.add_subplot(2, 3, i + 1, projection="3d")
        setup_ax(a)
        paint(a, tt[k], "(t = %.2f s)" % tt[k])
    fig2.suptitle("sm_exo_multidof —— 4-DOF 自由落体（T=0，重力自然摆动）"
                  "   轨迹来自 Simscape 导出的 multidof_traj.csv；L3 滑块 <visual> 已隐藏",
                  fontsize=12.5, y=0.97)
    fig2.tight_layout(rect=[0, 0, 1, 0.94])
    fig2.savefig(OUT_STRIP, dpi=120)
    print("WROTE", OUT_STRIP)

    # ---- 摆幅自检：确认 abduct 真被激励（否则动画看不出区别）----
    print("")
    print("=" * 74)
    print("摆幅自检（Simscape 轨迹；abduct 若≈0 则动画会退化成 2-DOF）")
    print("=" * 74)
    for i, qn in enumerate(QS):
        r = qq[:, i].max() - qq[:, i].min()
        print("  %-10s  %.6f rad  (%.2f deg)   [%.4f, %.4f]"
              % (qn, r, np.degrees(r), qq[:, i].min(), qq[:, i].max()))
    print("")
    print("ANIM_MULTIDOF_OK")


if __name__ == "__main__":
    main()
