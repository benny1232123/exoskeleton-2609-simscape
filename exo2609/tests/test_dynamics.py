# -*- coding: utf-8 -*-
"""Dynamics2609 的数学性质测试（全部针对论文 Eq.(1)-(5) 的结构性要求）。"""
import numpy as np
import pytest

from exo2609.dynamics import Dynamics2609
from exo2609.params import Dynamics2609Params


@pytest.fixture
def dyn():
    return Dynamics2609()


# ---------------------------------------------------------------- 参数层面
def test_paper_table_ii_values(dyn):
    """默认参数必须严格等于论文 Table II 的标定值。"""
    assert np.allclose(dyn.p.M, 0.0156 * np.eye(2))
    assert dyn.p.C_coef == 0.0
    assert np.allclose(dyn.p.G_amp, [0.879, 0.879])
    assert np.allclose(dyn.p.K1, 3.0 * np.eye(2))


def test_params_validate_ok(dyn):
    assert dyn.p.validate() == []


def test_validate_catches_bad_mass_matrix():
    p = Dynamics2609Params(M=np.array([[1.0, 2.0], [0.0, 1.0]]))  # 非对称
    assert any("不对称" in s for s in p.validate())
    p2 = Dynamics2609Params(M=np.array([[-1.0, 0.0], [0.0, 1.0]]))  # 非正定
    assert any("非正定" in s for s in p2.validate())


# ---------------------------------------------------------------- 基本量
def test_mass_matrix_symmetric_positive_definite(dyn):
    for q in ([0.0, 0.0], [0.4, -0.3], [1.1, 0.9]):
        M = dyn.mass_matrix(np.array(q))
        assert M.shape == (2, 2)
        assert np.allclose(M, M.T, atol=1e-12)
        assert np.all(np.linalg.eigvalsh(M) > 0)


def test_gravity_zero_at_upright(dyn):
    assert np.allclose(dyn.gravity_vector(np.array([0.0, 0.0])), 0.0, atol=1e-15)


def test_gravity_is_odd(dyn):
    """G(q) = G_amp sin(q) 关于 q 反对称。"""
    q = np.array([0.37, -0.82])
    assert np.allclose(dyn.gravity_vector(-q), -dyn.gravity_vector(q), atol=1e-15)


def test_gravity_amplitude(dyn):
    """q=pi/2 时 G 应等于 Table II 的 0.879。"""
    G = dyn.gravity_vector(np.array([np.pi / 2, 0.0]))
    assert abs(G[0] - 0.879) < 1e-12


def test_interaction_torque_eq3(dyn):
    """T_int = K1 qd + T0 —— Eq.(3)。"""
    qd = np.array([1.3, -0.7])
    assert np.allclose(dyn.interaction_torque(qd), 3.0 * qd)


# ---------------------------------------------------------------- 动力学
def test_accel_matches_analytic_formula(dyn):
    """qdd = Minv(T - C qd - G(q) - T_int(qd)) 逐项核对。"""
    q = np.array([0.3, -0.2])
    qd = np.array([0.5, 0.1])
    T = np.array([1.0, -0.4])
    y = np.hstack([q, qd])
    a = dyn.f(y, T)[2:]
    expect = np.linalg.inv(dyn.p.M) @ (T - dyn.p.K1 @ qd - dyn.gravity_vector(q))
    assert np.allclose(a, expect, atol=1e-12)


def test_state_derivative_layout(dyn):
    """ydot 前 2 维必须是 qd。"""
    y = np.array([0.1, 0.2, 0.7, -0.9])
    yd = dyn.f(y, np.zeros(2))
    assert np.allclose(yd[:2], [0.7, -0.9])


def test_gravity_opposes_motion(dyn):
    """物理约定下重力是阻力：q>0 且 qd=T=0 时角加速度应为负。"""
    a = dyn.f(np.array([0.5, 0.0, 0.0, 0.0]), np.zeros(2))[2:]
    assert a[0] < 0


def test_gravity_sign_flag_reproduces_paper_eq4(dyn):
    """gravity_sign=+1 时复现论文 Eq.(4) 的字面符号。"""
    p = Dynamics2609Params(gravity_sign=+1)
    d2 = Dynamics2609(p)
    a = d2.f(np.array([0.5, 0.0, 0.0, 0.0]), np.zeros(2))[2:]
    assert a[0] > 0


def test_step_is_explicit_euler(dyn):
    y = np.array([0.2, -0.1, 0.4, 0.3])
    T = np.array([0.5, 0.2])
    assert np.allclose(dyn.step(y, T), y + dyn.p.dt * dyn.f(y, T), atol=1e-14)


def test_two_joints_decoupled(dyn):
    """M 对角 + C=0 + G 逐元素 -> 左右髋完全解耦。"""
    y1 = np.array([0.4, 0.0, 0.3, 0.0])
    y2 = np.array([0.4, 0.9, 0.3, -1.1])   # 右髋状态不同
    a1 = dyn.f(y1, np.array([0.0, 0.0]))[2:]
    a2 = dyn.f(y2, np.array([0.0, 0.0]))[2:]
    assert abs(a1[0] - a2[0]) < 1e-14


def test_equilibrium(dyn):
    """平衡点：q=q_eq, qd=0, T=0 -> qdd=0。"""
    q_eq = dyn.equilibrium_q()
    a = dyn.f(np.hstack([q_eq, np.zeros(2)]), np.zeros(2))[2:]
    assert np.allclose(a, 0.0, atol=1e-12)


def test_energy_dissipation_exact(dyn):
    """连续时间下 dE/dt = -qd^T K1 qd <= 0（K1 为纯阻尼，系统被动）。"""
    y = np.array([0.6, -0.5, 1.5, 1.1])
    qd = y[2:].copy()
    h = 1e-7
    E = lambda yy: dyn.energy(yy)["total"]
    fy = dyn.f(y, np.zeros(2))
    dEdt = (E(y + h * fy) - E(y - h * fy)) / (2 * h)
    assert abs(dEdt - (-(qd @ dyn.p.K1 @ qd))) < 1e-6
    assert dEdt <= 0


def test_free_response_decays_to_equilibrium(dyn):
    """T=0 自由响应应收敛到平衡点（K1 阻尼耗散 + 重力），体现被动稳定性。

    注：显式欧拉有 O(dt^2) 离散误差，故这里检验 20 s 后的收敛残差而非逐步单调性。
    """
    y0 = np.array([0.6, -0.5, 1.5, 1.1])
    E0 = dyn.energy(y0)["total"]
    y = y0.copy()
    for _ in range(20000):
        y = dyn.step(y, np.zeros(2), dt=0.001)
    assert np.abs(y[2:]).max() < 1e-2, "角速度未衰减"
    assert np.abs(y[:2] - dyn.equilibrium_q()).max() < 1e-2, "未收敛到平衡角"
    assert dyn.energy(y)["total"] < E0, "总能量未下降"


def test_roll_forward_shape_and_consistency(dyn):
    y0 = np.array([0.0, 0.0, 0.0, 0.0])
    U = np.full((12, 2), 0.4)
    Y = dyn.roll_forward(y0, U, dt=0.005)
    assert Y.shape == (12, 4)
    # 逐步核对
    y = y0.copy()
    for k in range(12):
        y = dyn.step(y, U[k], dt=0.005)
        assert np.allclose(Y[k], y, atol=1e-14)


# ---------------------------------------------------------------- 雅可比
def test_jacobians_match_finite_difference(dyn):
    """解析雅可比必须与数值差分一致（Eq.(6) 的精确梯度依赖它）。"""
    rng = np.random.default_rng(0)
    y0 = rng.normal(scale=0.5, size=4)
    T = rng.normal(scale=0.5, size=2)
    h = 1e-6
    A, B = dyn.jacobians(y0, T)

    A_fd = np.zeros((4, 4))
    for i in range(4):
        dy = np.zeros(4); dy[i] = h
        A_fd[:, i] = (dyn.step(y0 + dy, T) - dyn.step(y0 - dy, T)) / (2 * h)
    assert np.allclose(A, A_fd, atol=1e-7)

    B_fd = np.zeros((4, 2))
    for i in range(2):
        dT = np.zeros(2); dT[i] = h
        B_fd[:, i] = (dyn.step(y0, T + dT) - dyn.step(y0, T - dT)) / (2 * h)
    assert np.allclose(B, B_fd, atol=1e-7)


def test_jacobian_gravity_sign(dyn):
    """A 左下块 = -Minv * gravity_sign * G_amp * cos(q)。"""
    p = Dynamics2609Params(gravity_sign=-1)
    d = Dynamics2609(p)
    q = np.array([0.4, -0.3])
    A, _ = d.jacobians(np.hstack([q, [0.0, 0.0]]), np.zeros(2), dt=1.0)
    expect = -np.linalg.inv(p.M) @ np.diag(p.G_amp * np.cos(q))
    assert np.allclose(A[2:, :2], expect, atol=1e-12)
