import csv
import numpy as np
from scipy.integrate import solve_ivp

# ★ 参数真源 = matlab2609/simscape/plant_params.csv
#   （由 _stp2plant.py 从 exo_real.urdf 派生；URDF 又由密度表派生）
#   以前这里硬编码 I/A/B：密度表一改（0.016975774 -> 0.016997395、
#   -0.438275227 -> -0.441845084）本脚本就会拿**旧模型**扫参数，而且不报错。
_PP = r"C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\plant_params.csv"
with open(_PP, encoding="utf-8", newline="") as _f:
    _rows = list(csv.DictReader(_f))
_r = _rows[0]                                  # hip_L
I = float(_r["I_axis"]); A = float(_r["A"]); B = float(_r["B"])
TDC0 = -B                                      # 让 q=0 成为平衡点的重力补偿量


def run(Tdc, Tamp, f, Tend=6.0, q0=0.0):
    def rhs(t, y):
        q, w = y
        T = Tdc + Tamp*np.sin(2*np.pi*f*t)
        return [w, (T + A*np.sin(q) + B*np.cos(q))/I]
    sol = solve_ivp(rhs, [0, Tend], [q0, 0.0], rtol=1e-10, atol=1e-12, dense_output=True)
    tt = np.linspace(0, Tend, 6000)
    q = sol.sol(tt)[0]
    return float(np.max(np.abs(q))), float(np.max(q)), float(np.min(q))


lines = ["重力补偿式驱动 T(t) = Tdc + Tamp*sin(2*pi*f*t)"
         "   (髋轴静止重力矩 tau_g(0) = %.6f N*m，源: plant_params.csv hip_L)" % B,
         "所以 Tdc = %.6f 让 q=0 成为平衡点，腿在零位附近往复 —— 这才像步态摆动。" % TDC0, "",
         " Tdc     Tamp    f[Hz]  max|q|[rad]   qmax      qmin      tumble"]
for Tdc in (round(TDC0, 4),):
    for f in (0.3, 0.4, 0.5, 0.6):
        for Tamp in (0.10, 0.15, 0.20, 0.25, 0.30):
            mx, qmax, qmin = run(Tdc, Tamp, f)
            lines.append(f"{Tdc:6.4f} {Tamp:7.2f} {f:6.2f} {mx:12.4f} {qmax:+9.4f} {qmin:+9.4f}   {'YES' if mx>np.pi else '-'}")
# 只看摆幅（去掉前 1 s 瞬态）更接近稳态振幅
lines += ["", "稳态振幅（丢掉前 1.0 s 瞬态）:", " Tdc     Tamp    f[Hz]  max|q|[rad]   qmax      qmin"]
for f in (0.3, 0.4, 0.5):
    for Tamp in (0.10, 0.15, 0.20, 0.25):
        mx, qmax, qmin = run(round(TDC0, 4), Tamp, f, Tend=8.0)
        lines.append(f"{round(TDC0,4):6.4f} {Tamp:7.2f} {f:6.2f} {mx:12.4f} {qmax:+9.4f} {qmin:+9.4f}")
open(r"C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\_sweep_dc.txt", "w", encoding="utf-8").write("\n".join(lines))
print("\n".join(lines))
