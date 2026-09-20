# -*- coding: utf-8 -*-
"""Eq.(6) 滑窗助力力矩优化的测试。"""
import numpy as np
import pytest

from exo2609.dynamics import Dynamics2609
from exo2609.params import Dynamics2609Params, OptimizerWeights
from exo2609.torque_opt import SlidingWindowTorqueEstimator
from exo2609 import gait


@pytest.fixture
def dyn():
    return Dynamics2609()


@pytest.fixture
def est(dyn):
    return SlidingWindowTorqueEstimator(dyn, L=12, enforce_tau_limit=True)


# ---------------------------------------------------------------- 权重
def test_weight_blocks_match_table_ii(dyn):
    w = OptimizerWeights()
    Q = w.Q_block(3, 10)
    assert np.allclose(np.diag(Q), [10.0, 10.0, 0.1, 0.1])
    assert np.allclose(w.Q_block(10, 10), 10.0 * np.eye(4))
    assert np.allclose(w.P_block(), 0.5 * np.eye(2))


# ---------------------------------------------------------------- 解析梯度
def test_analytic_gradient_matches_fd(est, dyn):
    """伴随法解析梯度 vs 中心差分。"""
    rng = np.random.default_rng(1)
    L = est.L
    y0 = rng.normal(scale=0.2, size=4)
    Yd = rng.normal(scale=0.3, size=(L, 4))
    A = rng.normal(scale=0.2, size=(L, 2))

    J, G = est._cost_and_grad(A.ravel(), y0, Yd)

    h = 1e-6
    Gfd = np.zeros_like(A)
    for i in range(L):
        for j in range(2):
            Ap = A.copy(); Ap[i, j] += h
            Am = A.copy(); Am[i, j] -= h
            Gfd[i, j] = (est._cost_and_grad(Ap.ravel(), y0, Yd)[0]
                         - est._cost_and_grad(Am.ravel(), y0, Yd)[0]) / (2 * h)
    assert np.allclose(G, Gfd.ravel(), atol=1e-5)


# ---------------------------------------------------------------- 最优性
def test_zero_torque_is_optimal_when_reference_is_free_response(est, dyn):
    """若参考轨迹本身就是 T=0 的自由响应，最优解应为 A≈0（唯一代价来自 P 项）。"""
    y0 = np.array([0.3, -0.2, 0.9, -0.6])
    Yd = dyn.roll_forward(y0, np.zeros((est.L, 2)))
    r = est.solve_window(y0, Yd)
    assert r["success"]
    assert np.max(np.abs(r["T"])) < 1e-6


def test_constant_offset_requires_nonzero_torque(est):
    """参考轨迹整体偏置时，必须解出非零力矩。"""
    y0 = np.zeros(4)
    Yd = np.tile(np.array([0.0, 0.0, 0.8, 0.4]), (est.L, 1))
    r = est.solve_window(y0, Yd)
    assert r["success"]
    assert np.max(np.abs(r["T"])) > 1e-3


def test_torque_respects_limit(dyn):
    """力矩限幅生效（论文硬件 AK80-9）。"""
    dyn_lo = Dynamics2609(Dynamics2609Params(tau_lim=np.array([0.5, 0.5])))
    est = SlidingWindowTorqueEstimator(dyn_lo, L=10, enforce_tau_limit=True)
    Yd = np.tile(np.array([0.0, 0.0, 2.0, 1.0]), (est.L, 1))
    r = est.solve_window(np.zeros(4), Yd)
    assert np.max(np.abs(r["T"])) <= 0.5 + 1e-9


def test_larger_penalty_gives_smaller_torque(dyn):
    """P 越大（Ca 越大），解出的力矩幅值应越小 —— 正则项语义正确性。"""
    Yd = np.tile(np.array([0.0, 0.0, 1.0, 0.5]), (10, 1))
    torques = []
    for Ca in (0.05, 0.5, 5.0):
        e = SlidingWindowTorqueEstimator(
            dyn, L=10, weights=OptimizerWeights(Ca=Ca), enforce_tau_limit=False)
        r = e.solve_window(np.zeros(4), Yd)
        torques.append(np.max(np.abs(r["T"])))
    assert torques[0] > torques[1] > torques[2]


# ---------------------------------------------------------------- 整段估计
def test_estimate_full_trajectory(est):
    """整段滑窗：形状正确、无失败窗口、解有界。"""
    g = gait.generate(speed=1.0, duration=2.0, dt=0.01)
    out = est.estimate(g["q"], g["qd"])
    N = g["q"].shape[0]
    assert out["T_est"].shape == (N, 2)
    # 末尾 L 个点无解 -> nan
    assert np.all(np.isnan(out["T_est"][N - est.L:]))
    assert np.isfinite(out["T_est"][:N - est.L]).all()
    assert est.last_diag["n_failed"] == 0
    assert est.last_diag["tau_peak"] < est.dyn.p.tau_lim.max() + 1e-6


def test_estimate_rejects_short_trajectory(est):
    with pytest.raises(ValueError):
        est.estimate(np.zeros((5, 2)), np.zeros((5, 2)))


# ---------------------------------------------------------------- 步态模块
def test_gait_phase_relations():
    g = gait.generate(speed=1.0, duration=4.0, dt=0.01)
    # 左右腿相位差半个周期
    assert not np.allclose(g["q"][:, 0], g["q"][:, 1])
    # 角速度与角位移的解析导数一致（数值差分核对）
    qd_fd = np.gradient(g["q"], g["t"], axis=0)
    assert np.corrcoef(qd_fd[5:-5, 0], g["qd"][5:-5, 0])[0, 1] > 0.999


def test_gait_amplitude_grows_with_speed():
    a = [np.ptp(gait.generate(v, 4.0, 0.01)["q"][:, 0]) for v in (0.6, 1.0, 1.4)]
    assert a[0] < a[1] < a[2]
    f = [gait.cadence_hz(v) for v in (0.6, 1.0, 1.4)]
    assert f[0] < f[1] < f[2]
