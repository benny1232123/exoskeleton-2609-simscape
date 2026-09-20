"""
外骨骼动力学模型可视化
- 两连杆骨架图 (不同姿态)
- 质量矩阵 M(q) 随关节角度变化
- 重力矩 G(q) 随关节角度变化
- 能量分析
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch
from exoskeleton_dynamics import ExoskeletonDynamics, ExoskeletonParams

plt.rcParams['font.family'] = 'SimHei'
plt.rcParams['axes.unicode_minus'] = False


def draw_exoskeleton(ax, q1, q2, params, alpha=1.0, color='steelblue', label=None):
    """绘制外骨骼两连杆机构"""
    L1 = params.L_thigh
    L2 = params.L_shank

    # 髋关节位置 (固定)
    hip = np.array([0, 0])
    # 膝关节位置
    knee = hip + L1 * np.array([np.sin(q1), -np.cos(q1)])
    # 踝关节位置
    ankle = knee + L2 * np.array([np.sin(q1 + q2), -np.cos(q1 + q2)])

    # 画连杆
    lw = 6
    ax.plot([hip[0], knee[0]], [hip[1], knee[1]], color=color, linewidth=lw, alpha=alpha, solid_capstyle='round')
    ax.plot([knee[0], ankle[0]], [knee[1], ankle[1]], color=color, linewidth=lw * 0.8, alpha=alpha, solid_capstyle='round')

    # 关节
    joint_size = 10
    ax.plot(*hip, 'o', color='#333', markersize=joint_size + 4, zorder=5)
    ax.plot(*hip, 'o', color='white', markersize=joint_size, zorder=6)
    ax.plot(*knee, 'o', color='#333', markersize=joint_size, zorder=5)
    ax.plot(*ankle, 'o', color='#333', markersize=joint_size - 2, zorder=5)

    # 质心标记
    r1 = params.r_thigh * params.L_thigh
    r2 = params.r_shank * params.L_shank
    com_thigh = hip + r1 * np.array([np.sin(q1), -np.cos(q1)])
    com_shank = knee + r2 * np.array([np.sin(q1 + q2), -np.cos(q1 + q2)])
    ax.plot(*com_thigh, 's', color='orange', markersize=6, zorder=7)
    ax.plot(*com_shank, 's', color='orange', markersize=6, zorder=7)

    # 角度弧线 - 髋关节
    theta1_range = np.linspace(-np.pi / 2, -np.pi / 2 + q1, 30)
    r_arc = 0.12
    ax.plot(r_arc * np.cos(theta1_range), r_arc * np.sin(theta1_range), color='red', linewidth=1.5, alpha=0.8)
    mid1 = -np.pi / 2 + q1 / 2
    ax.text(r_arc * 1.3 * np.cos(mid1), r_arc * 1.3 * np.sin(mid1),
            f'{np.degrees(q1):.0f}°', fontsize=8, color='red', ha='center', va='center')

    # 角度弧线 - 膝关节
    theta2_range = np.linspace(-np.pi / 2 + q1, -np.pi / 2 + q1 + q2, 30)
    r_arc2 = 0.10
    ax.plot(knee[0] + r_arc2 * np.cos(theta2_range), knee[1] + r_arc2 * np.sin(theta2_range),
            color='darkred', linewidth=1.5, alpha=0.8)
    mid2 = -np.pi / 2 + q1 + q2 / 2
    ax.text(knee[0] + r_arc2 * 1.5 * np.cos(mid2), knee[1] + r_arc2 * 1.5 * np.sin(mid2),
            f'{np.degrees(q2):.0f}°', fontsize=7, color='darkred', ha='center', va='center')

    # 重力箭头
    g_scale = 0.08
    g_len = params.m_thigh * params.g * g_scale
    ax.annotate('', xy=(com_thigh[0], com_thigh[1] - g_len),
                xytext=(com_thigh[0], com_thigh[1]),
                arrowprops=dict(arrowstyle='->', color='green', lw=2))
    g_len2 = params.m_shank * params.g * g_scale
    ax.annotate('', xy=(com_shank[0], com_shank[1] - g_len2),
                xytext=(com_shank[0], com_shank[1]),
                arrowprops=dict(arrowstyle='->', color='green', lw=2))

    # 标签
    if label:
        ax.text(0, 0.08, label, fontsize=10, fontweight='bold', ha='center', color=color)


def visualize_model():
    """主可视化函数"""
    params = ExoskeletonParams()
    dyn = ExoskeletonDynamics(params)

    fig = plt.figure(figsize=(16, 12))

    # ========== 子图1: 多姿态骨架图 ==========
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_xlim(-0.3, 0.7)
    ax1.set_ylim(-1.0, 0.2)
    ax1.set_aspect('equal')
    ax1.set_title('Exoskeleton 2-DOF Model: Multiple Poses', fontsize=12, fontweight='bold')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.grid(True, alpha=0.3)

    poses = [
        (0.0, 0.0, 'gray', 'Stand', 0.3),
        (0.4, -0.6, 'steelblue', 'Walk', 0.7),
        (0.8, -1.2, 'royalblue', 'Lunge', 1.0),
        (0.3, 0.8, 'navy', 'Swing', 0.8),
    ]
    for q1, q2, c, name, a in poses:
        draw_exoskeleton(ax1, q1, q2, params, alpha=a, color=c, label=name)

    # 地面线
    ax1.axhline(y=-params.L_thigh - params.L_shank - 0.05, color='brown', linewidth=2, linestyle='--', alpha=0.5)
    ax1.text(0.5, -params.L_thigh - params.L_shank - 0.08, 'Ground', fontsize=8, color='brown')

    # ========== 子图2: 质量矩阵元素 ==========
    ax2 = fig.add_subplot(2, 2, 2)
    q2_range = np.linspace(-np.pi, np.pi, 200)
    p = params

    m11 = (p.m_thigh * (p.r_thigh * p.L_thigh)**2 +
           p.m_shank * (p.L_thigh**2 + (p.r_shank * p.L_shank)**2 +
                        2 * p.L_thigh * p.r_shank * p.L_shank * np.cos(q2_range)) +
           p.I_thigh + p.I_shank)
    m12 = p.m_shank * ((p.r_shank * p.L_shank)**2 +
                        p.L_thigh * p.r_shank * p.L_shank * np.cos(q2_range)) + p.I_shank
    m22 = np.full_like(q2_range, p.m_shank * (p.r_shank * p.L_shank)**2 + p.I_shank)

    ax2.plot(np.degrees(q2_range), m11, 'b-', linewidth=2, label='M11 (hip inertia)')
    ax2.plot(np.degrees(q2_range), m12, 'r-', linewidth=2, label='M12 (coupling)')
    ax2.plot(np.degrees(q2_range), m22, 'g-', linewidth=2, label='M22 (knee inertia)')
    ax2.set_xlabel('Knee Angle q₂ (deg)')
    ax2.set_ylabel('Mass Matrix Value (kg·m²)')
    ax2.set_title('Mass Matrix M(q₂) vs Knee Angle', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    # ========== 子图3: 重力矩 ==========
    ax3 = fig.add_subplot(2, 2, 3)
    q1_range = np.linspace(-np.pi / 2, np.pi / 2, 200)
    q2_fixed = -0.5  # 膝关节固定

    g1 = ((p.m_thigh * p.r_thigh * p.L_thigh + p.m_shank * p.L_thigh) * p.g * np.cos(q1_range) +
           p.m_shank * p.r_shank * p.L_shank * p.g * np.cos(q1_range + q2_fixed))
    g2 = p.m_shank * p.r_shank * p.L_shank * p.g * np.cos(q1_range + q2_fixed)

    ax3.plot(np.degrees(q1_range), g1, 'b-', linewidth=2, label='G₁ (hip gravity)')
    ax3.plot(np.degrees(q1_range), g2, 'r-', linewidth=2, label='G₂ (knee gravity)')
    ax3.axhline(y=0, color='k', linewidth=0.5, linestyle='--')
    ax3.set_xlabel('Hip Angle q₁ (deg)')
    ax3.set_ylabel('Gravity Torque (Nm)')
    ax3.set_title(f'Gravity Vector G(q₁) at q₂={np.degrees(q2_fixed):.0f}°', fontsize=12, fontweight='bold')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)

    # ========== 子图4: 3D重力矩曲面 ==========
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    q1_3d = np.linspace(-np.pi / 2, np.pi / 2, 80)
    q2_3d = np.linspace(-np.pi, np.pi, 80)
    Q1, Q2 = np.meshgrid(q1_3d, q2_3d)
    G1_3d = ((p.m_thigh * p.r_thigh * p.L_thigh + p.m_shank * p.L_thigh) * p.g * np.cos(Q1) +
              p.m_shank * p.r_shank * p.L_shank * p.g * np.cos(Q1 + Q2))
    G2_3d = p.m_shank * p.r_shank * p.L_shank * p.g * np.cos(Q1 + Q2)

    surf = ax4.plot_surface(np.degrees(Q1), np.degrees(Q2), G1_3d, cmap='coolwarm', alpha=0.8)
    ax4.set_xlabel('Hip q₁ (deg)')
    ax4.set_ylabel('Knee q₂ (deg)')
    ax4.set_zlabel('G₁ (Nm)')
    ax4.set_title('Hip Gravity Torque Surface', fontsize=12, fontweight='bold')
    ax4.view_init(elev=25, azim=130)

    plt.tight_layout()
    plt.savefig('dynamics_model_visualization.png', dpi=150, bbox_inches='tight')
    print('Saved: dynamics_model_visualization.png')
    return fig


def visualize_mpc_tuning():
    """MPC参数调优对比"""
    params = ExoskeletonParams()
    dyn = ExoskeletonDynamics(params)
    from mpc_controller import ExoskeletonMPC

    fig, axes = plt.subplots(2, 3, figsize=(16, 8))

    configs = [
        {'N': 10, 'dt': 0.02, 'title': 'N=10, dt=0.02 (short)'},
        {'N': 20, 'dt': 0.02, 'title': 'N=20, dt=0.02 (baseline)'},
        {'N': 30, 'dt': 0.02, 'title': 'N=30, dt=0.02 (long)'},
    ]

    state0 = np.array([0.0, 0.0, 0.0, 0.0])

    for i, cfg in enumerate(configs):
        mpc = ExoskeletonMPC(dyn, N=cfg['N'], dt=cfg['dt'])
        x_ref = mpc.generate_reference_trajectory(state0, v_hip=1.0, v_knee=-0.5)
        u_opt, x_pred = mpc.solve(state0, x_ref)

        t = np.arange(mpc.N + 1) * mpc.dt

        # 角度
        axes[0, i].plot(t, np.degrees(x_pred[0, :]), 'b-', linewidth=2, label='Hip pred')
        axes[0, i].plot(t, np.degrees(x_ref[0, :]), 'b--', alpha=0.5, label='Hip ref')
        axes[0, i].plot(t, np.degrees(x_pred[1, :]), 'r-', linewidth=2, label='Knee pred')
        axes[0, i].plot(t, np.degrees(x_ref[1, :]), 'r--', alpha=0.5, label='Knee ref')
        axes[0, i].set_ylabel('Angle (deg)')
        axes[0, i].set_title(cfg['title'], fontsize=10, fontweight='bold')
        axes[0, i].legend(fontsize=7)
        axes[0, i].grid(True, alpha=0.3)

        # 扭矩
        t_u = np.arange(mpc.N) * mpc.dt
        axes[1, i].step(t_u, u_opt[0, :], 'b-', linewidth=2, where='post', label='Hip')
        axes[1, i].step(t_u, u_opt[1, :], 'r-', linewidth=2, where='post', label='Knee')
        axes[1, i].set_xlabel('Time (s)')
        axes[1, i].set_ylabel('Torque (Nm)')
        axes[1, i].legend(fontsize=7)
        axes[1, i].grid(True, alpha=0.3)

    axes[0, 0].set_ylabel('Angle (deg)\nN=10')
    axes[1, 0].set_ylabel('Torque (Nm)\nN=10')

    plt.suptitle('MPC Prediction Horizon Comparison', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('mpc_horizon_comparison.png', dpi=150, bbox_inches='tight')
    print('Saved: mpc_horizon_comparison.png')
    return fig


if __name__ == '__main__':
    print('=== Exoskeleton Dynamics Visualization ===')
    print('\n[1] Drawing model poses & parameter surfaces...')
    visualize_model()
    print('\n[2] Comparing MPC horizons...')
    visualize_mpc_tuning()
    print('\nDone! Open the PNG files to view.')

    # 打开图片
    import os
    os.startfile('dynamics_model_visualization.png')
    os.startfile('mpc_horizon_comparison.png')
