"""
Generate animated GIF of exoskeleton dynamics.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import odeint
from bilateral_hip_dynamics import BilateralHipDynamics
from exoskeleton_dynamics import ExoskeletonDynamics


def generate_gif():
    """Generate and save exoskeleton animation as GIF."""
    bilateral = BilateralHipDynamics()
    hip_knee = ExoskeletonDynamics()

    dt = 0.02
    t_total = 3.0  # 3 seconds for GIF
    n_steps = int(t_total / dt)

    state_bilateral = np.array([0.0, 0.0, 0.0, 0.0])

    def tau_left(t):
        return 5.0 * np.sin(2 * np.pi * 0.5 * t)

    def tau_right(t):
        return 5.0 * np.sin(2 * np.pi * 0.5 * t + np.pi/2)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.canvas.manager.set_window_title('Exoskeleton Animation')

    def draw_skeleton(ax, q1, q2):
        ax.clear()
        ax.set_xlim(-0.8, 0.8)
        ax.set_ylim(-0.2, 1.2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        # Body
        ax.plot([0, 0], [0.8, 1.0], 'k-', linewidth=4)
        ax.plot(0, 0.8, 'ko', markersize=8)

        L = 0.504
        # Left leg
        lx = L * np.sin(q1)
        ly = 0.8 - L * np.cos(q1)
        ax.plot([0, lx], [0.8, ly], 'b-', linewidth=3, label='Left')
        ax.plot(lx, ly, 'bo', markersize=8)

        # Right leg
        rx = L * np.sin(q2)
        ry = 0.8 - L * np.cos(q2)
        ax.plot([0, rx], [0.8, ry], 'r-', linewidth=3, label='Right')
        ax.plot(rx, ry, 'ro', markersize=8)

        # Motors
        ax.plot(-0.1, 0.8, 'bs', markersize=12)
        ax.plot(0.1, 0.8, 'rs', markersize=12)

        ax.set_title(f'Bilateral Hip\nLeft: {np.degrees(q1):.1f}°  Right: {np.degrees(q2):.1f}°')
        ax.legend()

    def animate(frame):
        t = frame * dt
        tau_l = tau_left(t)
        tau_r = tau_right(t)

        def dyn_b(s, t_val):
            return bilateral.state_space(s, t_val, np.array([tau_l, tau_r]))

        sol_b = odeint(dyn_b, state_bilateral, [0, dt])
        state_bilateral[:] = sol_b[-1]

        draw_skeleton(ax1, state_bilateral[0], state_bilateral[1])

        # Angle plot
        ax2.clear()
        ax2.plot([0, t], [0, np.degrees(state_bilateral[0])], 'b-', linewidth=2)
        ax2.plot([0, t], [0, np.degrees(state_bilateral[1])], 'r-', linewidth=2)
        ax2.set_xlim(0, t_total)
        ax2.set_ylim(-60, 60)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angle (deg)')
        ax2.set_title('Joint Angles')
        ax2.grid(True)
        ax2.legend(['Left', 'Right'])

        return []

    print("Generating animation...")
    anim = animation.FuncAnimation(
        fig, animate, frames=n_steps, interval=50, blit=False, cache_frame_data=False
    )

    anim.save('exoskeleton.gif', writer='pillow', fps=20)
    print("Saved: exoskeleton.gif")

    plt.show()


if __name__ == '__main__':
    generate_gif()
