"""
主程序: 下肢外骨骼动力学模型与MPC控制演示
"""
import numpy as np
import matplotlib.pyplot as plt
from exoskeleton_dynamics import ExoskeletonDynamics, ExoskeletonParams
from mpc_controller import ExoskeletonMPC


def main():
    print("=" * 60)
    print("下肢外骨骼动力学模型与MPC控制演示")
    print("=" * 60)
    
    # 1. 创建动力学模型
    print("\n[1] 创建动力学模型...")
    params = ExoskeletonParams()
    dyn = ExoskeletonDynamics(params)
    print("    质量矩阵 M(q):")
    print(f"    {dyn.M}")
    
    # 2. 测试逆动力学
    print("\n[2] 测试逆动力学...")
    q_test = np.array([0.2, -0.3])  # 关节角度
    qd_test = np.array([0.5, -0.2])  # 关节角速度
    qdd_test = np.array([0.1, -0.1])  # 关节角加速度
    
    tau = dyn.inverse_dynamics(q_test, qd_test, qdd_test)
    print(f"    输入: q={np.degrees(q_test)}°, qd={np.degrees(qd_test)}°/s, qdd={np.degrees(qdd_test)}°/s²")
    print(f"    所需扭矩: τ_hip={tau[0]:.2f} Nm, τ_knee={tau[1]:.2f} Nm")
    
    # 3. 测试前向动力学
    print("\n[3] 测试前向动力学...")
    tau_test = np.array([10.0, -5.0])  # 施加扭矩
    qdd_calc = dyn.forward_dynamics(q_test, qd_test, tau_test)
    print(f"    输入: q={np.degrees(q_test)}°, qd={np.degrees(qd_test)}°/s, τ={tau_test} Nm")
    print(f"    计算加速度: qdd={np.degrees(qdd_calc)}°/s²")
    
    # 4. 创建MPC控制器
    print("\n[4] 创建MPC控制器...")
    mpc = ExoskeletonMPC(dyn, N=20, dt=0.02)
    print(f"    预测步数: {mpc.N}")
    print(f"    时间步长: {mpc.dt} s")
    
    # 5. 求解MPC
    print("\n[5] 求解MPC优化问题...")
    state0 = np.array([0.0, 0.0, 0.0, 0.0])
    x_ref = mpc.generate_reference_trajectory(state0, v_hip=1.0, v_knee=-0.5)
    
    u_opt, x_pred = mpc.solve(state0, x_ref)
    print(f"    初始控制输入: τ_hip={u_opt[0, 0]:.2f} Nm, τ_knee={u_opt[1, 0]:.2f} Nm")
    
    # 6. 可视化
    print("\n[6] 生成可视化图表...")
    visualize_results(dyn, mpc, state0, x_ref, u_opt, x_pred)
    
    print("\n" + "=" * 60)
    print("演示完成!")
    print("生成的文件:")
    print("  - dynamics_demo.png: 动力学模型演示")
    print("  - mpc_demo.png: MPC控制演示")
    print("=" * 60)


def visualize_results(dyn, mpc, state0, x_ref, u_opt, x_pred):
    """可视化结果"""
    fig = plt.figure(figsize=(14, 10))
    
    # 1. 状态轨迹
    ax1 = fig.add_subplot(2, 2, 1)
    t_pred = np.arange(mpc.N + 1) * mpc.dt
    ax1.plot(t_pred, np.degrees(x_pred[0, :]), 'b-', linewidth=2, label='预测')
    ax1.plot(t_pred, np.degrees(x_ref[0, :]), 'b--', alpha=0.7, label='参考')
    ax1.plot(t_pred, np.degrees(x_pred[1, :]), 'r-', linewidth=2)
    ax1.plot(t_pred, np.degrees(x_ref[1, :]), 'r--', alpha=0.7)
    ax1.set_xlabel('时间 (s)')
    ax1.set_ylabel('角度 (°)')
    ax1.set_title('关节角度预测 vs 参考')
    ax1.legend(['髋关节预测', '髋关节参考', '膝关节预测', '膝关节参考'])
    ax1.grid(True)
    
    # 2. 角速度轨迹
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t_pred, np.degrees(x_pred[2, :]), 'b-', linewidth=2, label='髋关节')
    ax2.plot(t_pred, np.degrees(x_pred[3, :]), 'r-', linewidth=2, label='膝关节')
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('角速度 (°/s)')
    ax2.set_title('关节角速度预测')
    ax2.legend()
    ax2.grid(True)
    
    # 3. 控制输入
    ax3 = fig.add_subplot(2, 2, 3)
    t_ctrl = np.arange(mpc.N) * mpc.dt
    ax3.step(t_ctrl, u_opt[0, :], 'b-', linewidth=2, where='post', label='髋关节')
    ax3.step(t_ctrl, u_opt[1, :], 'r-', linewidth=2, where='post', label='膝关节')
    ax3.set_xlabel('时间 (s)')
    ax3.set_ylabel('扭矩 (Nm)')
    ax3.set_title('MPC优化控制输入')
    ax3.legend()
    ax3.grid(True)
    
    # 4. 相平面
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(np.degrees(x_pred[0, :]), np.degrees(x_pred[2, :]), 'b-o', linewidth=2, markersize=4, label='髋关节')
    ax4.plot(np.degrees(x_pred[1, :]), np.degrees(x_pred[3, :]), 'r-o', linewidth=2, markersize=4, label='膝关节')
    ax4.plot(np.degrees(state0[0]), np.degrees(state0[2]), 'ko', markersize=10, label='初始点')
    ax4.set_xlabel('角度 (°)')
    ax4.set_ylabel('角速度 (°/s)')
    ax4.set_title('相平面轨迹')
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    plt.savefig('mpc_controller_demo.png', dpi=150, bbox_inches='tight')
    plt.show()


if __name__ == "__main__":
    main()
