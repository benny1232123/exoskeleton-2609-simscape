# -*- coding: utf-8 -*-
"""
参考轨迹（步态）生成

论文用的是「实测电机角度/角速度」作为期望轨迹 Yd。我们没有你的实机数据，
所以这里生成**生理学上合理的替代轨迹**用于跑通与验证，并提供 load_reference()
供你直接接入真实录制的 (q, qd) 数据。

！！！重要！！！
    本模块产出的是 surrogate（替代）数据，不是你的实测步态。
    任何基于它得出的「助力力矩绝对值」都只能用于验证模型行为是否合理，
    不能当作你硬件的最终标定结果。真机标定必须用实测轨迹。

轨迹构造
--------
髋屈伸角取一个步态周期内的 2 次谐波傅里叶级数，用正常步态的关键事件点拟合：

    相位 φ     屈髋角(deg)   事件
    0.00        25          首次触地 (initial contact)
    0.10        18          承重反应 (loading response)
    0.30         5          支撑中期 (mid-stance)
    0.50       -10          支撑末期 (terminal stance, 髋后伸最大)
    0.60         5          预摆动 (pre-swing)
    0.75        28          摆动初期（屈髋峰值）
    0.85        22          摆动中期
    1.00        25          摆动末期

步频随速度线性变化（依据正常步态步频-速度关系）：
    0.6 m/s -> 0.75 Hz,  1.0 m/s -> 0.89 Hz,  1.4 m/s -> 1.02 Hz
幅值随速度轻微放大（0.6 m/s: 0.85x, 1.0 m/s: 1.00x, 1.4 m/s: 1.12x）。
"""

from __future__ import annotations

import numpy as np

# 关键事件点（相位, 屈髋角 rad）
_KEY = np.array([
    [0.00, 25.0],
    [0.10, 18.0],
    [0.30,  5.0],
    [0.50, -10.0],
    [0.60,  5.0],
    [0.75, 28.0],
    [0.85, 22.0],
    [1.00, 25.0],
])

# 步频/幅值 对速度的锚点
_F_ANCHOR_V = np.array([0.6, 1.0, 1.4])
_F_ANCHOR_F = np.array([0.75, 0.89, 1.02])
_A_ANCHOR = np.array([0.85, 1.00, 1.12])


def _fit_fourier(n_harm: int = 2) -> np.ndarray:
    """对关键事件点做最小二乘拟合，返回系数 [a0, (ak, bk) for k in 1..n]."""
    phi = _KEY[:, 0]
    y = np.deg2rad(_KEY[:, 1])
    cols = [np.ones_like(phi)]
    for k in range(1, n_harm + 1):
        cols.append(np.cos(2 * np.pi * k * phi))
        cols.append(np.sin(2 * np.pi * k * phi))
    A = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    return coef


_COEF = _fit_fourier(2)


def hip_flexion(phi: np.ndarray, amp_scale: float = 1.0) -> np.ndarray:
    """归一化相位 φ∈[0,1) -> 髋屈伸角 [rad]。amp_scale 缩放的仅是交流分量。"""
    phi = np.asarray(phi, dtype=float)
    out = np.full_like(phi, _COEF[0], dtype=float)
    for k in range(1, 3):
        ak, bk = _COEF[2 * k - 1], _COEF[2 * k]
        out = out + amp_scale * (ak * np.cos(2 * np.pi * k * phi) + bk * np.sin(2 * np.pi * k * phi))
    return out


def cadence_hz(speed: float) -> float:
    """速度 [m/s] -> 步态周期频率 [Hz]（一个完整步态周期/秒）。"""
    return float(np.interp(speed, _F_ANCHOR_V, _F_ANCHOR_F))


def amp_scale(speed: float) -> float:
    return float(np.interp(speed, _F_ANCHOR_V, _A_ANCHOR))


def generate(speed: float = 1.0, duration: float = 6.0, dt: float = 0.01,
             phase_offset: float = 0.0, left_right_phase_delay: float = 0.5):
    """生成双侧髋关节参考轨迹。

    Parameters
    ----------
    speed : float   行走速度 [m/s]，对应论文 Fig.4 的 0.6 / 1.0 / 1.4
    duration : float    时长 [s]
    dt : float      采样步长 [s]
    phase_offset : float    整体相位偏移（归一化周期）
    left_right_phase_delay : float  左右腿相位差（0.5 = 对侧步态）

    Returns
    -------
    dict(t, q=(N,2), qd=(N,2), qdd=(N,2), f_cycle, amp_scale)
        q[:,0]/qd[:,0] = 左髋；q[:,1]/qd[:,1] = 右髋
    """
    f = cadence_hz(speed)
    s = amp_scale(speed)
    N = int(round(duration / dt))
    t = np.arange(N) * dt
    phi_L = (f * t + phase_offset) % 1.0
    phi_R = (f * t + phase_offset + left_right_phase_delay) % 1.0

    q = np.column_stack([hip_flexion(phi_L, s), hip_flexion(phi_R, s)])

    # 角速度/角加速度用解析导数，避免差分噪声
    dcoef = _COEF.copy()
    w = 2 * np.pi * f

    def dq(phi):
        out = np.zeros_like(phi)
        for k in range(1, 3):
            ak, bk = dcoef[2 * k - 1], dcoef[2 * k]
            out = out + s * w * k * (-ak * np.sin(2 * np.pi * k * phi)
                                     + bk * np.cos(2 * np.pi * k * phi))
        return out

    def ddq(phi):
        out = np.zeros_like(phi)
        for k in range(1, 3):
            ak, bk = dcoef[2 * k - 1], dcoef[2 * k]
            out = out - s * (w * k) ** 2 * (ak * np.cos(2 * np.pi * k * phi)
                                            + bk * np.sin(2 * np.pi * k * phi))
        return out

    qd = np.column_stack([dq(phi_L), dq(phi_R)])
    qdd = np.column_stack([ddq(phi_L), ddq(phi_R)])
    return {"t": t, "q": q, "qd": qd, "qdd": qdd,
            "f_cycle": f, "amp_scale": s, "speed": speed}


def load_reference(path: str, dt: float | None = None) -> dict:
    """接入真机录制的参考轨迹。

    期望文件为 .csv，含表头，至少 4 列：
        q_left, q_right, qd_left, qd_right        （SI: rad, rad/s）
    可选列:
        t, qdd_left, qdd_right
    或者一个 .npz，含键 q(N,2), qd(N,2), 可选 t, qdd。
    """
    p = str(path).lower()
    if p.endswith((".npz", ".npy")):
        z = np.load(path)
        q = np.asarray(z["q"], float).reshape(-1, 2)
        qd = np.asarray(z["qd"], float).reshape(-1, 2)
        t = np.asarray(z["t"], float) if "t" in z.files else None
        qdd = np.asarray(z["qdd"], float).reshape(-1, 2) if "qdd" in z.files else None
    else:
        raw = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
        names = raw.dtype.names
        q = np.column_stack([raw["q_left"], raw["q_right"]])
        qd = np.column_stack([raw["qd_left"], raw["qd_right"]])
        t = raw["t"] if "t" in names else None
        qdd = (np.column_stack([raw["qdd_left"], raw["qdd_right"]])
               if "qdd_left" in names else None)
    N = q.shape[0]
    if t is None:
        if dt is None:
            raise ValueError("缺少时间列 t，请显式给出 dt")
        t = np.arange(N) * dt
    if qdd is None:
        qdd = np.gradient(qd, t, axis=0)
    return {"t": t, "q": q, "qd": qd, "qdd": qdd,
            "f_cycle": np.nan, "amp_scale": np.nan, "speed": np.nan,
            "source": "measured"}
