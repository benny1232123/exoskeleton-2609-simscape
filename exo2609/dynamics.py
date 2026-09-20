# -*- coding: utf-8 -*-
"""
2609 论文动力学模型（Section III.A Eq.(1)-(5)）

模型
----
外骨骼侧（论文明确"仅计及外骨骼动力学模型"）:

    M qdd + C(q,qd) qd + G(q) = T - T_int                    ... Eq.(1)
    T_int = K1 qd + T0                                        ... Eq.(3)

状态 y = [q; qd] ∈ R^4，控制 T ∈ R^2：

    ydot = [ y1 ; M^{-1} ( T - C*y1 - G(y0) - T_int(y1) ) ]   ... Eq.(4) 物理约定
    y_{t+1} = y_t + ydot_t * dt                               ... Eq.(5)

其中（论文 Table II）
    M          = 0.0156 I        （常数对角，左右髋解耦）
    C(q,qd)    = 0
    G(q)       = 0.879 sin(q)    （幅值 = m*g*d，重力为阻力）
    K1         = 3 I
    T0         = 0（论文未给，默认）

模型的结构性质（用于校验）
--------------------------
* M 常数 ⇒ 质量矩阵对称正定，M^{-1} 有闭式解，前向动力学无需数值求解。
* C = 0  ⇒ 无科氏项，两髋完全解耦 ⇒ M 为分块对角，left/right 互不影响。
* G = G_amp sin(q) ⇒ q=0 处重力矩为 0（悬挂平衡位），G 关于 q 反对称。
* 系统为「刚性摆 + 粘性阻尼」：qd 的系数 -K1 恒为负 ⇒ 无自激振荡。
  平衡点 q_eq 满足  G_amp sin(q_eq) = -T0  （T=0、qd=0 时）。
"""

from __future__ import annotations

import numpy as np

from .params import Dynamics2609Params


class Dynamics2609:
    """2 自由度髋关节外骨骼动力学（左髋 q[0]、右髋 q[1]）。

    Parameters
    ----------
    params : Dynamics2609Params
        模型参数；默认取论文 Table II 标定值。

    Examples
    --------
    >>> import numpy as np
    >>> from exo2609 import Dynamics2609
    >>> dyn = Dynamics2609()
    >>> dyn.f(np.array([0.2, -0.1, 0.5, 0.0]), np.array([1.0, 0.0]))
    array([ 0.5       ,  0.        , ...])
    """

    def __init__(self, params: Dynamics2609Params | None = None):
        self.p = params if params is not None else Dynamics2609Params()
        self._Minv = np.linalg.inv(self.p.M)
        # 预分解，供解析雅可比使用
        self._Minv_scaled_K1 = self._Minv @ self.p.K1

    # ------------------------------------------------------------------
    # 基本量
    # ------------------------------------------------------------------
    @property
    def nq(self) -> int:
        return self.p.n

    @property
    def n(self) -> int:
        """状态维数 2*nq。"""
        return 2 * self.p.n

    @property
    def nu(self) -> int:
        return self.p.n

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """M(q)。论文 Table II 中 M 为常数，但保留 q 参数以兼容通用模型。"""
        return self.p.M.copy()

    def coriolis_matrix(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        """C(q,qd)。论文 Table II 中 C = 0。"""
        n = self.nq
        return self.p.C_coef * np.eye(n)

    def gravity_vector(self, q: np.ndarray) -> np.ndarray:
        """G(q) = G_amp * sin(q)  [N*m]，物理约定下为阻力（前向动力学中取负）。"""
        return self.p.G_amp * np.sin(q)

    def interaction_torque(self, qd: np.ndarray) -> np.ndarray:
        """人机交互力矩 T_int = K1 qd + T0  [N*m]  —— Eq.(3)。"""
        return self.p.K1 @ qd + self.p.T0

    # ------------------------------------------------------------------
    # 动力学
    # ------------------------------------------------------------------
    def f(self, y: np.ndarray, T: np.ndarray) -> np.ndarray:
        """连续状态导数 ydot = f(y, T)。Eq.(4)。

        y = [q; qd] ∈ R^{2nq}，T ∈ R^{nq}。
        """
        y = np.asarray(y, dtype=float).ravel()
        T = np.asarray(T, dtype=float).ravel()
        nq = self.nq
        q, qd = y[:nq], y[nq:]
        qdd = self._accel(q, qd, T)
        out = np.empty_like(y)
        out[:nq] = qd
        out[nq:] = qdd
        return out

    def _accel(self, q, qd, T) -> np.ndarray:
        """qdd = M^{-1}( T - C qd + g*G(q) - T_int )，g = gravity_sign。"""
        rhs = T - self.coriolis_matrix(q, qd) @ qd \
            + self.p.gravity_sign * self.gravity_vector(q) \
            - self.interaction_torque(qd)
        return self._Minv @ rhs

    def step(self, y: np.ndarray, T: np.ndarray, dt: float | None = None) -> np.ndarray:
        """离散状态转移 y_{t+1} = f_d(y_t, T_t) —— Eq.(5)，显式欧拉（与论文一致）。

        论文原文："In the discrete-time system, the next state can be denoted as
        y_{t+1} = y_t + ydot_t * dt."
        """
        h = self.p.dt if dt is None else dt
        return np.asarray(y, dtype=float).ravel() + h * self.f(y, T)

    def roll_forward(self, y0: np.ndarray, U: np.ndarray,
                     dt: float | None = None) -> np.ndarray:
        """给定初始状态与整段控制序列，前向积分。

        Parameters
        ----------
        y0 : (2nq,) 初始状态
        U  : (L, nq) 控制序列 [T_1 ... T_L]

        Returns
        -------
        Y : (L, 2nq) 每步之后的实际状态 [y_1 ... y_L]（论文 Y = [y1;...;yL]）
        """
        U = np.atleast_2d(np.asarray(U, dtype=float))
        Y = np.empty((U.shape[0], self.n))
        y = np.asarray(y0, dtype=float).ravel().copy()
        for k in range(U.shape[0]):
            y = self.step(y, U[k], dt)
            Y[k] = y
        return Y

    # ------------------------------------------------------------------
    # 解析雅可比（供 Eq.(6) 的精确梯度使用）
    # ------------------------------------------------------------------
    def jacobians(self, y: np.ndarray, T: np.ndarray,
                  dt: float | None = None):
        """返回离散转移的偏导 (A_k, B_k)：

            y_{k+1} = y_k + h f(y_k, T_k)
            A_k = ∂y_{k+1}/∂y_k = I + h ∂f/∂y
            B_k = ∂y_{k+1}/∂T_k =     h ∂f/∂T

        ∂f/∂y 的闭式（nq=2，M 与 K1 均为常数对角/常数矩阵）：
            ∂f/∂y = [[ 0              , I                 ],
                     [ -Minv*Gamp*cos(q) , -Minv*K1        ]]
        注意 ∂/∂q 中已含 gravity_sign：∂(g*G)/∂q = g*G_amp*cos(q)。
        """
        h = self.p.dt if dt is None else dt
        y = np.asarray(y, dtype=float).ravel()
        nq = self.nq
        q = y[:nq]
        dfdy = np.zeros((self.n, self.n))
        dfdy[:nq, nq:] = np.eye(nq)
        # ∂(gravity_sign * G(q))/∂q = gravity_sign * diag(G_amp * cos(q))
        # 必须写成矩阵乘积 Minv @ diag(...)，不能按元素广播（Minv 一般非对角）。
        dfdy[nq:, :nq] = self.p.gravity_sign * (self._Minv @ np.diag(self.p.G_amp * np.cos(q)))
        dfdy[nq:, nq:] = -self._Minv_scaled_K1
        dfdT = np.zeros((self.n, self.nu))
        dfdT[nq:, :] = self._Minv
        return np.eye(self.n) + h * dfdy, h * dfdT

    # ------------------------------------------------------------------
    # 便利 / 物理量
    # ------------------------------------------------------------------
    def energy(self, y: np.ndarray) -> dict:
        """动能 / 势能 / 总能量（用于能量守恒与耗散校验）。

        势能必须与动力学中的重力矩自洽：
            前向动力学中重力矩为 gravity_sign * G(q) = -(G_amp sin q)（阻力）
            由 -dV/dq = -G_amp sin(q)  =>  V(q) = -G_amp cos(q)
        极小点在 q=0（大腿自然下垂），符合物理。
        """
        y = np.asarray(y, dtype=float).ravel()
        nq = self.nq
        q, qd = y[:nq], y[nq:]
        T_kin = 0.5 * qd @ self.p.M @ qd
        V = float(np.sum(-self.p.G_amp * np.cos(q)))
        return {"kinetic": float(T_kin), "potential": V, "total": float(T_kin + V)}

    def equilibrium_q(self) -> np.ndarray:
        """T=0、qd=0 时由重力与 T0 决定的平衡角：G_amp sin(q) = -T0。"""
        return np.arcsin(np.clip(-self.p.T0 / self.p.G_amp, -1.0, 1.0))

    def summary(self) -> str:
        p = self.p
        s = ["Dynamics2609 (2-DOF 左髋/右髋)"]
        s.append("  时间常数 M/K1 = %.5f s" % (p.M[0, 0] / p.K1[0, 0]))
        s.append("  固有频率 sqrt(G_amp/M) = %.3f rad/s (小角度线性化)" %
                 float(np.sqrt(p.G_amp[0] / p.M[0, 0])))
        s.append("  平衡角 = %s rad" % np.array2string(self.equilibrium_q(), precision=4))
        s.append("  状态维数 %d, 控制维数 %d" % (self.n, self.nu))
        return "\n".join(s)
