# -*- coding: utf-8 -*-
"""
exo2609 —— 2609 论文（Assistance Torque Estimation via Dynamics-Aware
Optimization for Lower-Limb Exoskeleton in Complex Environments,
arXiv:2609.15352v1）动力学模型的复现，参数以本机 CAD 模型为基础标定。

模块
----
params       参数定义（论文 Table II 标定值 / CAD 反算值，带来源标注）
dynamics     Eq.(1)-(5)：2 自由度髋关节外骨骼动力学（左髋 + 右髋）
torque_opt   Eq.(6)：滑窗式助力力矩优化（解析伴随梯度）
gait         参考轨迹生成 / 实测轨迹接入
geometry     CAD 几何 -> M / G_amp 折算（见 geometry.py）
visualize    结果可视化
"""

from .params import Dynamics2609Params, OptimizerWeights, Source, scalar_ratio
from .dynamics import Dynamics2609
from .torque_opt import SlidingWindowTorqueEstimator

__all__ = [
    "Dynamics2609Params",
    "OptimizerWeights",
    "Source",
    "scalar_ratio",
    "Dynamics2609",
    "SlidingWindowTorqueEstimator",
]

__version__ = "0.1.0"
