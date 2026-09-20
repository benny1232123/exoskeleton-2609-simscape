# -*- coding: utf-8 -*-
"""
参数定义：论文 Table II 标定值 + 由 CAD 几何反算的值，全部带来源标注。

论文 Table II（Torque Estimation 列，原文照抄）
------------------------------------------------
    Inertial mat. H   M = 0.0156 I
    Velocity mat. C   C = 0
    Gravity mat.  G   G = 0.879 sin(y)
    Coefficient  K1   K1 = 3 I
    Error mat.    Q   Q = diag{10, 0.1} I
    Torque mat.   P   P = 0.5 I

注意（论文自身的记号不一致，本实现显式处理）
------------------------------------------------
1) 引力项符号
   Eq.(1) :  M qdd + C qd + G(q) = T - T_int
   Eq.(4) :  ydot = [ y1 ; Minv( T - C*y1 + G(y0) - T_int(y1) ) ]
   Eq.(4) 中 G 的符号与 Eq.(1) 推导结果相反，属论文笔误。
   物理上，悬挂在髋轴下方的大腿受到的重力矩是「回复力矩」，应作阻力。
   本实现默认采用与 Eq.(1) 一致的物理约定：
        M qdd = T - C qd - G(q) - T_int(qd)
   可通过 `gravity_sign=+1` 复现论文 Eq.(4) 的字面形式（用于对照实验）。

2) P / Q 的维数
   原文 "P^{1:L} = Ca I4"，但 P ∈ R^{2L x 2L}、每个 P_k 必须是 2x2，
   故按 P_k = Ca I2 理解（Ca 为标量 0.5）。
   同理 Q_k(k<L) = diag(Cq I2, Cv I2) 是 4x4，Q_L = Cp I4。
   论文未给出 Cp，本实现默认 Cp = Cq（可通过参数覆盖）。
"""

from dataclasses import dataclass, field, asdict
import numpy as np


# --------------------------------------------------------------------------
# 参数来源标签：便于在任何输出里追溯每个数字是怎么来的
# --------------------------------------------------------------------------
class Source:
    PAPER = "paper"          # 论文 Table II / 正文明确给出的标定值
    CAD = "cad"              # 由本机 STEP 实体的 B-rep 质量属性反算
    DATASHEET = "datasheet"  # 器件手册
    ASSUMED = "assumed"      # 无依据的假设，必须显式暴露
    DERIVED = "derived"      # 由上述量推导得到


def scalar_ratio(a, b, rtol: float = 1e-3, unit: str = "x paper") -> str:
    """把「数组 a 相对数组 b 的倍数」压成一个标量字符串。

    用途：参数表里的「本 CAD / 论文」比值列。要点
      - 只在 b 的非零元素上求比（避免 0/0 -> nan 让 np.allclose 静默失败，
        那是本函数存在的原因：矩阵参数 M/K1 含零对角外元素）。
      - 所有比值在 rtol 内一致 -> 输出单值；否则给出区间。
      - b 全零（如 C=0、T0=0）-> 返回空串，表示「无数值可比」。
    """
    try:
        aa = np.atleast_1d(np.asarray(a, float)).ravel()
        bb = np.atleast_1d(np.asarray(b, float)).ravel()
        mask = np.abs(bb) > 1e-15
        if not np.any(mask):
            return ""
        rr = aa[mask] / bb[mask]
        if np.allclose(rr, rr.mean(), rtol=rtol):
            return "%.3f %s" % (rr.mean(), unit)
        return "varies (%.3f..%.3f)" % (rr.min(), rr.max())
    except Exception:
        return ""


@dataclass
class Dynamics2609Params:
    """2609 论文的 2 自由度（左髋 + 右髋）动力学模型参数。

    状态 y = [q; qd] ∈ R^4, 控制 T ∈ R^2, n = 2 个驱动关节。
    所有量均为 SI：m, kg, s, rad, N*m, kg*m^2。

    Attributes
    ----------
    M : (2,2) ndarray
        惯量矩阵。论文标定值 M = 0.0156 I（常数对角）。
    C_coef : float
        速度/科氏项系数。论文标定值 0；即 C(q,qd) = C_coef * I。
    G_amp : (2,) ndarray
        重力矩幅值 [N*m]，G(q) = G_amp * sin(q)。
        物理含义：G_amp_i = m_i * g * d_i（m_i 为第 i 侧摆动部件质量，
        d_i 为髋轴到该侧部件质心的距离）。
        论文标定值 G = 0.879 sin(q)。
    K1 : (2,2) ndarray
        人机交互力矩系数矩阵，T_int = K1 @ qd + T0。论文标定值 K1 = 3 I。
    T0 : (2,) ndarray
        交互力矩常数项（论文未给出，默认 0）。
    dt : float
        离散化步长 [s]。论文未给出，实时控制常用 1e-3 ~ 2e-2。
    q_lim : (2,2) ndarray
        关节限位 [[lo,hi],[lo,hi]]，单位 rad。
    tau_lim : (2,) ndarray
        电机力矩限幅 [N*m]，取自 AK80-9 手册。
    gravity_sign : int
        -1 采用物理约定（阻力）; +1 复现论文 Eq.(4) 字面形式。
    sources : dict
        每个参数名 -> Source 标签。
    """

    M: np.ndarray = field(default_factory=lambda: np.eye(2) * 0.0156)
    C_coef: float = 0.0
    G_amp: np.ndarray = field(default_factory=lambda: np.array([0.879, 0.879]))
    K1: np.ndarray = field(default_factory=lambda: np.eye(2) * 3.0)
    T0: np.ndarray = field(default_factory=lambda: np.zeros(2))
    dt: float = 0.01
    q_lim: np.ndarray = field(default_factory=lambda: np.array([[-0.6, 1.2], [-0.6, 1.2]]))
    tau_lim: np.ndarray = field(default_factory=lambda: np.array([18.0, 18.0]))
    gravity_sign: int = -1
    g: float = 9.81
    sources: dict = field(default_factory=dict)

    # -------------------- 构造与校验 --------------------
    def __post_init__(self):
        n = 2
        self.M = np.asarray(self.M, dtype=float).reshape(n, n)
        self.G_amp = np.asarray(self.G_amp, dtype=float).reshape(n)
        self.K1 = np.asarray(self.K1, dtype=float).reshape(n, n)
        self.T0 = np.asarray(self.T0, dtype=float).reshape(n)
        self.q_lim = np.asarray(self.q_lim, dtype=float).reshape(n, 2)
        self.tau_lim = np.asarray(self.tau_lim, dtype=float).reshape(n)
        if not self.sources:
            self.sources = {
                "M": Source.PAPER,
                "C_coef": Source.PAPER,
                "G_amp": Source.PAPER,
                "K1": Source.PAPER,
                "T0": Source.ASSUMED,
                "dt": Source.ASSUMED,
                "q_lim": Source.ASSUMED,
                "tau_lim": Source.DATASHEET,
            }

    @property
    def n(self) -> int:
        return 2

    def validate(self) -> list:
        """返回问题列表；空列表表示通过。

        检查项（对应论文模型的数学要求）：
          - M 对称正定
          - K1 半正定（否则交互力矩会注入能量）
          - G_amp 非负、q_lim 上界大于下界、tau_lim 为正
        """
        issues = []
        if not np.allclose(self.M, self.M.T):
            issues.append("M 不对称")
        w = np.linalg.eigvalsh(self.M)
        if not np.all(w > 0):
            issues.append(f"M 非正定, eigvalsh={w}")
        kw = np.linalg.eigvalsh(0.5 * (self.K1 + self.K1.T))
        if not np.all(kw >= -1e-12):
            issues.append(f"K1 非半正定, eigvalsh={kw}")
        if np.any(self.G_amp < 0):
            issues.append("G_amp 出现负值，符号约定应统一")
        if np.any(self.q_lim[:, 0] >= self.q_lim[:, 1]):
            issues.append("q_lim 下界 >= 上界")
        if np.any(self.tau_lim <= 0):
            issues.append("tau_lim 非正")
        if self.dt <= 0:
            issues.append("dt 必须为正")
        return issues

    # -------------------- 数值积分诊断（不并入 validate，避免改变结构校验语义） --------------------
    def damping_time_constant(self) -> float:
        """内部交互阻尼的最快时间常数 [s]：min eig(M) / max eig(K1)。

        连续系统的速度自由度按 exp(-t/tau) 衰减，tau = M/K1（K1 为纯阻尼）。
        显式欧拉要正确分辨它，需要 dt << tau；若 dt*max(K1)/min(M) 接近或超过 1，
        离散步进会出现符号交替并严重失真，进而扭曲 Eq.(6) 对力矩的响应。
        """
        return float(np.min(np.linalg.eigvalsh(self.M))
                     / max(1e-30, float(np.max(np.linalg.eigvalsh(self.K1)))))

    def recommended_dt(self, safety: float = 0.25) -> float:
        """满足安全裕度的建议步长上限 [s]：safety * M/K1。"""
        return safety * self.damping_time_constant()

    def discretization(self) -> dict:
        """显式欧拉的离散化诊断。

        Returns
        -------
        dict(tau_damp, ratio=dt/tau_damp, pole=1-dt*K1/M, ok, recommended_dt, recommended_hz)
        """
        tau = self.damping_time_constant()
        ratio = self.dt / max(1e-30, tau)
        pole = 1.0 - self.dt * float(np.max(np.linalg.eigvalsh(self.K1))) / \
            max(1e-30, float(np.min(np.linalg.eigvalsh(self.M))))
        rec = self.recommended_dt()
        return {
            "tau_damp": tau,
            "ratio": ratio,
            "pole": pole,
            "ok": bool(ratio <= 0.5),
            "recommended_dt": rec,
            "recommended_hz": (1.0 / rec if rec > 0 else float("inf")),
        }

    def discretization_note(self) -> str:
        d = self.discretization()
        if d["ok"]:
            return ("dt=%.4g s 可分辨内部阻尼时间常数 M/K1=%.4g s（dt*K1/M=%.2f, 极点 %+.3f）"
                    % (self.dt, d["tau_damp"], d["ratio"], d["pole"]))
        return ("⚠ dt=%.4g s 相对内部阻尼时间常数 M/K1=%.4g s 过大（dt*K1/M=%.2f, "
                "极点 %+.3f 符号交替）：显式欧拉欠采样阻尼，Eq.(6) 的力矩响应会被失真放大。"
                "建议 dt<=%.4g s（即 >=%.0f Hz），或对 K1 阻尼项做隐式/半隐式积分。"
                % (self.dt, d["tau_damp"], d["ratio"], d["pole"],
                   d["recommended_dt"], d["recommended_hz"]))

    # -------------------- 摘要 --------------------
    def summary(self) -> str:
        lines = []
        lines.append("Dynamics2609Params")
        lines.append("  n           = %d (左髋, 右髋)" % self.n)
        lines.append("  M           =\n%s" % np.array2string(self.M, prefix="    "))
        lines.append("  C_coef      = %.6g" % self.C_coef)
        lines.append("  G_amp       = %s  [N*m]   (m*g*d 折算)" % np.array2string(self.G_amp))
        lines.append("  K1          =\n%s" % np.array2string(self.K1, prefix="    "))
        lines.append("  T0          = %s" % np.array2string(self.T0))
        lines.append("  dt          = %.4g s" % self.dt)
        lines.append("  q_lim       = %s rad (%.1f~%.1f deg)" % (
            np.array2string(self.q_lim),
            np.degrees(self.q_lim[0, 0]), np.degrees(self.q_lim[0, 1])))
        lines.append("  tau_lim     = %s N*m" % np.array2string(self.tau_lim))
        lines.append("  gravity_sign= %+d (%s)" % (
            self.gravity_sign,
            "物理约定, 重力为阻力" if self.gravity_sign < 0 else "论文 Eq.(4) 字面形式"))
        lines.append("  --- 来源标注 ---")
        for k, v in sorted(self.sources.items()):
            lines.append("    %-12s %s" % (k, v))
        lines.append("  --- 离散化 ---")
        lines.append("    %s" % self.discretization_note())
        return "\n".join(lines)

    def to_dict(self) -> dict:
        d = asdict(self)
        for k in ("M", "G_amp", "K1", "T0", "q_lim", "tau_lim"):
            d[k] = np.asarray(d[k]).tolist()
        return d

    @classmethod
    def paper_baseline(cls, dt: float = 0.01) -> "Dynamics2609Params":
        """论文 Table II 的标定值，作为基线。"""
        return cls(dt=dt)

    @classmethod
    def from_geometry(cls, geo, leg_left: str = "leg_L", leg_right: str = "leg_R",
                      dt: float = 0.01) -> "Dynamics2609Params":
        """由 CAD 几何反算结果（exo2609.geometry.StepGeometryResult）构造参数。

        折算关系
        --------
        M_i     = I_hip_i = 摆动件绕髋轴的惯量（含绕自身质心的项 + 平行轴项）
        G_amp_i = m_i * g * d_i
                  m_i 第 i 侧摆动部件总质量；d_i 髋轴到该侧合成质心的垂距
        K1/T0   无法由几何得到（人机交互阻尼/摩擦），沿用论文标定值并显式标注来源。
        """
        gL = geo.groups[leg_left]
        gR = geo.groups[leg_right]
        M = np.diag([gL.M, gR.M])
        G_amp = np.array([gL.G_amp, gR.G_amp])
        p = cls(M=M, G_amp=G_amp, dt=dt)
        p.sources.update({
            "M": Source.CAD,
            "G_amp": Source.CAD,
            "C_coef": Source.CAD,       # C = 0，几何上无科氏项（两髋解耦、无偏置）
            "K1": Source.PAPER,         # 交互阻尼不可由几何反算 -> 沿用论文，需实测标定
            "T0": Source.PAPER,
        })
        # 把可审计的中间量挂上，便于报告与复核
        p.geometry = {
            "axis_point_mm": np.asarray(geo.axis_point, float).tolist(),
            "axis_dir": np.asarray(geo.axis_dir, float).tolist(),
            "axis_evidence": geo.axis_evidence,
            "leg_left": {"m_kg": gL.m_kg, "d_perp_m": gL.d_perp_m, "I_axis": gL.I_axis,
                         "cog_world_mm": np.asarray(gL.cog_world, float).tolist()},
            "leg_right": {"m_kg": gR.m_kg, "d_perp_m": gR.d_perp_m, "I_axis": gR.I_axis,
                          "cog_world_mm": np.asarray(gR.cog_world, float).tolist()},
            "total_mass_kg": geo.total_mass_kg,
        }
        return p

    # ------------------------------------------------------------------
    def compare_paper(self, names=("M", "C_coef", "G_amp", "K1", "T0")) -> str:
        """本参数与论文 Table II 基线的逐项对比表。"""
        base = Dynamics2609Params.paper_baseline(dt=self.dt)
        rows = []
        for k in names:
            a, b = getattr(self, k), getattr(base, k)
            try:
                sa, sb = np.array2string(np.atleast_1d(a), precision=6), \
                         np.array2string(np.atleast_1d(b), precision=6)
            except Exception:
                sa, sb = str(a), str(b)
            ratio = scalar_ratio(a, b)
            rows.append((k, self.sources.get(k, "?"), sa, sb, ratio))
        w = [max(len(str(r[i])) for r in rows + [("param", "source", "this CAD", "paper", "ratio")])
             for i in range(5)]
        hdr = ("param", "source", "this CAD", "paper", "ratio")
        out = ["%-*s | %-*s | %-*s | %-*s | %s" % (
            w[0], hdr[0], w[1], hdr[1], w[2], hdr[2], w[3], hdr[3], hdr[4]),
            "-" * (sum(w) + 16)]
        for r in rows:
            out.append("%-*s | %-*s | %-*s | %-*s | %s" % (
                w[0], r[0], w[1], r[1], w[2], r[2], w[3], r[3], r[4]))
        return "\n".join(out)


# --------------------------------------------------------------------------
# 优化器权重（论文 Eq.(6) / Table II）
# --------------------------------------------------------------------------
@dataclass
class OptimizerWeights:
    """Eq.(6) 的权重矩阵标量。

        Q_k (k=1..L-1) = diag(Cq*I2, Cv*I2)  ∈ R^{4x4}   角度误差 / 角速度误差
        Q_L            = Cp * I4             ∈ R^{4x4}   终端误差
        P_k            = Ca * I2             ∈ R^{2x2}   力矩惩罚（正则项）

    Table II: Q = diag{10, 0.1} I  ->  Cq = 10, Cv = 0.1
              P = 0.5 I            ->  Ca = 0.5
    终端权重 Cp 论文未给出，默认取 Cq。
    """
    Cq: float = 10.0
    Cv: float = 0.1
    Ca: float = 0.5
    Cp: float = None  # None -> 用 Cq

    def __post_init__(self):
        if self.Cp is None:
            self.Cp = self.Cq

    def Q_block(self, k: int, L: int) -> np.ndarray:
        """第 k 步（1-based）的 4x4 误差权重。"""
        if k == L:
            return self.Cp * np.eye(4)
        return np.diag([self.Cq, self.Cq, self.Cv, self.Cv])

    def P_block(self) -> np.ndarray:
        return self.Ca * np.eye(2)

    def sources(self) -> dict:
        return {
            "Cq": Source.PAPER,
            "Cv": Source.PAPER,
            "Ca": Source.PAPER,
            "Cp": Source.ASSUMED if getattr(self, "_cp_assumed", True) else Source.PAPER,
        }
