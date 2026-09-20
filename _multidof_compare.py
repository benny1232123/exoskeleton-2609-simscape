# -*- coding: utf-8 -*-
"""
_multidof_compare.py —— 解析 4-DOF  vs  Simscape `sm_exo_multidof` 交叉验证
============================================================================

这是 2-DOF 那条链（compare_simscape_vs_analytical.m）在 4-DOF 上的对应物。

为什么能这么比
--------------
`sm_exo_multidof` 是**纯 CAD 导入**模型：没有 From Workspace、没有力矩输入，
所以它天然就是 T = 0。而 CAD 零位不是重力平衡位（零位挂着约 0.44 N*m），
因此 T=0 就已经是一次充分的自由落体，把 M(q) 与 G(q) 同时激励起来。

对照组
------
  Simscape : `dump_multidof_traj.m` 导出（限位已关、重力 [0 0 -9.81]）
  解析     : `_multidof_dyn.MultiBody` 从**同一个 URDF** 组装拉格朗日方程，
             用 RK4 从同一初始条件（零位零速）积分

两侧唯一的共同输入是 URDF；力学是从零各自推的（Simscape 用它的多体引擎，
解析用 Jacobian 组装 + 势能梯度），所以吻合才有意义。

★ 状态顺序**必须按列名映射**
  Simscape 的 xout 是按块名字母序**交错**的：
      abduct_L.q, abduct_L.w, abduct_R.q, abduct_R.w, hip_L.q, hip_L.w, hip_R.q, hip_R.w
  与本模块的 [hip_L, abduct_L, hip_R, abduct_R] 不同。
  （第一版曾从 0.5 s 末态数值倒推顺序，得出错误结论 —— 见 dump 脚本里的注释。）

判据：rms/range < 1e-2（沿用 compare_simscape_vs_analytical.m 的阈值）
"""
import io
import os
import sys

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)
from _multidof_dyn import MultiBody          # noqa: E402

SIM = os.path.join(ROOT, "matlab2609", "simscape")
OUTD = os.path.join(ROOT, "matlab2609", "out_simscape")
CSV = os.path.join(OUTD, "multidof_traj.csv")
URDF = os.path.join(SIM, "exo_multidof.urdf")
OUT = os.path.join(ROOT, "_multidof_compare.txt")

L = []
def say(s=""):
    L.append(s)
    print(s)


def main():
    if not os.path.exists(CSV):
        say("FAIL: %s 不存在 —— 先跑 dump_multidof_traj.m" % CSV)
        return "NO_DATA"

    with io.open(CSV, encoding="utf-8") as f:
        hdr = f.readline().strip().split(",")
    d = np.loadtxt(CSV, delimiter=",", skiprows=1)
    t_sim = d[:, 0]
    col = {h: i for i, h in enumerate(hdr)}

    say("=" * 92)
    say("解析 4-DOF  vs  Simscape sm_exo_multidof")
    say("=" * 92)
    say("Simscape CSV  : %d 点, %d 列, t ∈ [%.4f, %.4f]"
        % (d.shape[0], d.shape[1] - 1, t_sim[0], t_sim[-1]))
    say("  CSV 列序      : %s" % ", ".join(hdr[1:]))

    mb = MultiBody(URDF)
    nq = mb.nq
    say("MultiBody     : nq=%d  顺序 = %s" % (nq, mb.q_names))
    say("总质量        : %.9f kg" % sum(mb.link[l]["mass"] for l in mb.links))

    # ---- 按名字映射（不要假设顺序）----
    idx_q, idx_w = [], []
    for nm in mb.q_names:
        kq, kw = nm + ".q", nm + ".w"
        if kq not in col or kw not in col:
            say("FAIL: Simscape CSV 里找不到 %s / %s" % (kq, kw))
            return "NO_DATA"
        idx_q.append(col[kq]); idx_w.append(col[kw])
    say("  列映射        : %s"
        % ", ".join("%s<-col%d" % (nm, i) for nm, i in zip(mb.q_names, idx_q)))

    qs_sim = d[:, idx_q]
    qds_sim = d[:, idx_w]

    # ---------------------------------------------------------- 解析积分
    dt = 1e-4
    nst = int(round(t_sim[-1] / dt))
    say("")
    say("-- 解析侧 RK4 积分（dt = %.1e, %d 步 = %.2f s）--" % (dt, nst, nst * dt))

    def rk4(f, y, h):
        k1 = f(y); k2 = f(y + 0.5*h*k1); k3 = f(y + 0.5*h*k2); k4 = f(y + h*k3)
        return y + h / 6.0 * (k1 + 2*k2 + 2*k3 + k4)

    zero = np.zeros(nq)
    y = np.zeros(2 * nq)
    tg = [0.0]
    Yg = [y.copy()]
    for k in range(nst):
        y = rk4(lambda yy: np.concatenate(
            [yy[nq:], mb.qdd(yy[:nq], yy[nq:], zero)]), y, dt)
        tg.append((k + 1) * dt)
        Yg.append(y.copy())
    tg = np.asarray(tg)
    Yg = np.asarray(Yg)

    # 插到 Simscape 的时间点上（Simscape 是变步长，点不均匀）
    # 解析轨迹是光滑的，插值误差远小于判据阈值。
    Ya = np.empty((len(t_sim), 2 * nq))
    for k in range(2 * nq):
        Ya[:, k] = np.interp(t_sim, tg, Yg[:, k])
    Ya[-1] = Yg[-1]        # 末点用积分真值，避免外插

    say("  解析末态 q     = %s" % np.array2string(Ya[-1, :nq], precision=6))
    say("  Simscape 末态 q = %s" % np.array2string(qs_sim[-1], precision=6))

    # ---------------------------------------------------------- 逐通道
    names = list(mb.q_names) + ["d_" + n for n in mb.q_names]
    Ysim = np.concatenate([qs_sim, qds_sim], axis=1)
    say("")
    say("-- 逐通道误差（相对量程）--")
    say("  %-14s %13s %13s %13s %12s"
        % ("channel", "max|err|", "rms(err)", "range(sim)", "rms/range"))
    worst, worstAt = 0.0, ""
    for k in range(2 * nq):
        ref = Ysim[:, k]
        e = Ya[:, k] - ref
        rg = float(np.max(ref) - np.min(ref))
        rms = float(np.sqrt(np.mean(e ** 2)))
        rel = rms / max(rg, 1e-12)
        say("  %-14s %13.4e %13.4e %13.6f %12.4e"
            % (names[k], float(np.max(np.abs(e))), rms, rg, rel))
        if rel > worst:
            worst, worstAt = rel, names[k]

    # ---------------------------------------------------------- 摆幅
    say("")
    say("-- 摆幅（判断自由度是否真的被激励）--")
    for k in range(nq):
        rg_s = float(np.max(qs_sim[:, k]) - np.min(qs_sim[:, k]))
        rg_a = float(np.max(Ya[:, k]) - np.min(Ya[:, k]))
        tag = ""
        if "abduct" in mb.q_names[k]:
            tag = "  <- 若≈0 则退化成 2-DOF，对照没意义"
        say("  %-12s sim %10.6f rad   analytic %10.6f rad%s"
            % (mb.q_names[k], rg_s, rg_a, tag))

    # ---------------------------------------------------------- verdict
    say("")
    say("worst channel: %s   rms/range = %.4e" % (worstAt, worst))
    if worst < 1e-2:
        verdict = "OK"
    elif worst < 5e-2:
        verdict = "MARGINAL"
    else:
        verdict = "FAIL"
    say("verdict: %s   (threshold 1e-2，沿用 2-DOF 那条链)" % verdict)
    say("")
    say("COMPARE_MULTIDOF_%s" % verdict)

    # ---------------------------------------------------------- figure
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
        matplotlib.rcParams["axes.unicode_minus"] = False

        fig, axs = plt.subplots(2, nq, figsize=(4.3 * nq, 7.0))
        for k in range(nq):
            a = axs[0, k]
            a.plot(t_sim, qs_sim[:, k], "r--", lw=2.6, label="Simscape")
            a.plot(t_sim, Ya[:, k], "k-", lw=1.1, label="analytic 4-DOF")
            a.set_title(mb.q_names[k]); a.grid(alpha=.3); a.set_xlabel("t [s]")
            a.set_ylabel("q [rad]")
            if k == 0:
                a.legend(loc="best", fontsize=8)
            b = axs[1, k]
            b.plot(t_sim, Ya[:, k] - qs_sim[:, k], "b-", lw=1.2)
            b.set_title("%s : mismatch" % mb.q_names[k])
            b.grid(alpha=.3); b.set_xlabel("t [s]"); b.set_ylabel("err [rad]")
        fig.suptitle("exo_multidof 4-DOF：Simscape（红虚线） vs 解析拉格朗日（黑实线）"
                     "   worst rms/range = %.2e  → %s" % (worst, verdict), fontsize=12)
        fig.tight_layout(rect=[0, 0, 1, 0.94])
        png = os.path.join(OUTD, "compare_multidof.png")
        fig.savefig(png, dpi=125)
        say("figure: %s" % png)
    except Exception as ex:
        say("(figure skipped: %s)" % ex)

    io.open(OUT, "w", encoding="utf-8").write("\n".join(L) + "\n")
    return verdict


if __name__ == "__main__":
    main()
