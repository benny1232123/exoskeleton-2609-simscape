# -*- coding: utf-8 -*-
"""
论文 Eq.(6)：基于动力学模型的助力力矩优化

问题
----
    argmin_A  (Yd - Y)^T Q (Yd - Y) + A^T P A
    s.t.      y_{t+1} = f(y_t, T_t)

    Yd = [y1; ...; y_L] ∈ R^{4L}    期望轨迹（实测电机角度/角速度）
    Y  = [y1; ...; y_L] ∈ R^{4L}    由动力学模型滚动得到的实际轨迹
    A  = [T1; ...; T_L] ∈ R^{2L}    待优化助力力矩

    Q = diag{Q_1..Q_L},  Q_k = diag{Cq I2, Cv I2} (k<L),  Q_L = Cp I4
    P = diag{P_1..P_L},  P_k = Ca I2

求解
----
窗口内是带约束的非线性规划。本实现用**离散伴随法求精确解析梯度**（无有限差分），
外层交给 scipy 的 SLSQP（支持箱式约束 = 电机力矩限幅）。

伴随递推（把状态导数记为列向量）
    λ_L   = 2 Q_L (Y_L - Yd_L)
    λ_{k-1} = A_k^T λ_k + 2 Q_{k-1} (Y_{k-1} - Yd_{k-1})
    ∂J/∂T_k = B_k^T λ_k + 2 P_k T_k
其中 A_k = ∂Y_k/∂Y_{k-1}, B_k = ∂Y_k/∂T_k 由 Dynamics2609.jacobians 给出。

滑窗策略（论文原文）
--------------------
"the optimization is performed via a sliding-window scheme with window length L
and sliding step size 1. The first torque value of A solved from each window-wise
optimization is regarded as the estimated assistance torque corresponding to the
current motion state."
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import minimize

from .dynamics import Dynamics2609
from .params import OptimizerWeights


class SlidingWindowTorqueEstimator:
    """滑窗式助力力矩估计器（Eq.(6)）。

    Parameters
    ----------
    dyn : Dynamics2609
        外骨骼动力学模型。
    L : int
        窗口长度（时间步数）。
    weights : OptimizerWeights | None
        权重矩阵标量；默认论文 Table II。
    enforce_tau_limit : bool
        是否施加电机力矩箱式约束。
    solver : str
        交给 scipy.optimize.minimize 的方法；'SLSQP' 支持约束与边界。
    """

    def __init__(self, dyn: Dynamics2609 | None = None,
                 L: int = 25,
                 weights: OptimizerWeights | None = None,
                 enforce_tau_limit: bool = True,
                 solver: str = "L-BFGS-B",
                 maxiter: int = 2000,
                 ftol: float = 1e-12,
                 gtol: float = 1e-10):
        self.dyn = dyn if dyn is not None else Dynamics2609()
        self.L = int(L)
        self.w = weights if weights is not None else OptimizerWeights()
        self.enforce_tau_limit = enforce_tau_limit
        self.solver = solver
        self.maxiter = maxiter
        self.ftol = ftol
        self.gtol = gtol

        # 预置权重块
        self.Q = [self.w.Q_block(k, self.L) for k in range(1, self.L + 1)]
        self.P = [self.w.P_block() for _ in range(self.L)]

        self.last_diag = {}

    # ------------------------------------------------------------------
    # 目标函数与梯度
    # ------------------------------------------------------------------
    def _cost_and_grad(self, A_flat, y0, Yd):
        nq, n = self.dyn.nq, self.dyn.n
        L = self.L
        A = A_flat.reshape(L, nq)

        # ---- 前向滚动，缓存每步雅可比 ----
        Y = np.empty((L, n))
        A_jac = np.empty((L, n, n))
        B_jac = np.empty((L, n, nq))
        y = y0.copy()
        for k in range(L):
            Ak, Bk = self.dyn.jacobians(y, A[k])
            A_jac[k], B_jac[k] = Ak, Bk
            y = y + self.dyn.p.dt * self.dyn.f(y, A[k])
            Y[k] = y

        # ---- 代价 ----
        J = 0.0
        E = Y - Yd
        for k in range(L):
            J += E[k] @ self.Q[k] @ E[k]
        for k in range(L):
            J += A[k] @ self.P[k] @ A[k]

        # ---- 伴随递推 ----
        lam = 2.0 * (self.Q[L - 1] @ E[L - 1])
        G = np.empty((L, nq))
        G[L - 1] = B_jac[L - 1].T @ lam + 2.0 * (self.P[L - 1] @ A[L - 1])
        for k in range(L - 2, -1, -1):
            lam = A_jac[k + 1].T @ lam + 2.0 * (self.Q[k] @ E[k])
            G[k] = B_jac[k].T @ lam + 2.0 * (self.P[k] @ A[k])

        return float(J), G.ravel()

    # ------------------------------------------------------------------
    # 单窗口求解
    # ------------------------------------------------------------------
    def solve_window(self, y_meas, Yd, A_init=None):
        """求解单个窗口。

        Parameters
        ----------
        y_meas : (4,)  窗口起始时刻实测状态 y_0 = [q; qd]
        Yd     : (L,4) 该窗口的期望轨迹 [y_1 ... y_L]
        A_init : (L,2) | None 初值

        Returns
        -------
        dict(T=(L,2), Y=(L,4), cost, success, nit, msg)
        """
        L, nq = self.L, self.dyn.nq
        Yd = np.asarray(Yd, dtype=float).reshape(L, self.dyn.n)
        y_meas = np.asarray(y_meas, dtype=float).ravel()

        A0 = np.zeros((L, nq)) if A_init is None else np.asarray(A_init, float).reshape(L, nq)
        bnds = None
        if self.enforce_tau_limit:
            bnds = [(-self.dyn.p.tau_lim[i % nq], self.dyn.p.tau_lim[i % nq])
                    for i in range(L * nq)]

        res = minimize(
            self._cost_and_grad, A0.ravel(),
            args=(y_meas, Yd), jac=True, method=self.solver,
            bounds=bnds,
            options={"maxiter": self.maxiter, "ftol": self.ftol, "gtol": self.gtol},
        )
        A = res.x.reshape(L, nq)
        Y = self.dyn.roll_forward(y_meas, A)
        return {
            "T": A, "Y": Y, "cost": float(res.fun),
            "success": bool(res.success), "nit": int(getattr(res, "nit", -1)),
            "msg": str(res.message),
            "grad_norm": float(np.linalg.norm(res.jac)),
        }

    # ------------------------------------------------------------------
    # 整段轨迹滑窗估计
    # ------------------------------------------------------------------
    def estimate(self, Q_ref, Qd_ref, Y0=None, warm_start=True):
        """对整段实测轨迹做滑窗估计。

        Parameters
        ----------
        Q_ref  : (N,)  实测关节角 [rad]，列 0=左髋, 列 1=右髋
        Qd_ref : (N,)  实测关节角速度 [rad/s]
        Y0     : (4,) | None  首点前的初始状态；None 时用 (Q_ref[0], Qd_ref[0])
        warm_start : bool  用上一窗解的平移作为下一窗初值（论文未提及，仅加速）

        Returns
        -------
        dict(T_est=(N,2), info=list[dict], Yd=(N,4))
            T_est[t] 为第 t 个「当前运动状态」对应的估计助力力矩。
            窗口 t 覆盖参考点 t+1..t+L，故仅前 N-L 个点有解（其余为 nan）。
        """
        Q_ref = np.asarray(Q_ref, float).reshape(-1, 2)
        Qd_ref = np.asarray(Qd_ref, float).reshape(-1, 2)
        N = Q_ref.shape[0]
        L = self.L
        if N < L + 1:
            raise ValueError("轨迹长度 N=%d 必须 > 窗口长度 L=%d" % (N, L))

        Yd_all = np.hstack([Q_ref, Qd_ref])           # (N,4)
        T_est = np.full((N, 2), np.nan)
        info = [None] * N
        A_prev = None

        for t in range(0, N - L):
            y_meas = (np.hstack([Q_ref[t], Qd_ref[t]]) if Y0 is None
                      else np.asarray(Y0, float) if t == 0 else np.hstack([Q_ref[t], Qd_ref[t]]))
            Yd = Yd_all[t + 1: t + 1 + L]
            A0 = None
            if warm_start and A_prev is not None:
                A0 = np.vstack([A_prev[1:], A_prev[-1:]])
            r = self.solve_window(y_meas, Yd, A_init=A0)
            T_est[t] = r["T"][0]           # 取窗口第一个力矩
            info[t] = {k: r[k] for k in ("cost", "success", "nit", "msg", "grad_norm")}
            A_prev = r["T"]

        self.last_diag = {
            "n_windows": int(N - L),
            "n_failed": int(sum(1 for d in info if d is not None and not d["success"])),
            "L": L,
            "mean_cost": float(np.mean([d["cost"] for d in info if d is not None])),
            "max_grad": float(np.max([d["grad_norm"] for d in info if d is not None])),
            "tau_peak": float(np.nanmax(np.abs(T_est))),
        }
        return {"T_est": T_est, "info": info, "Yd": Yd_all}
