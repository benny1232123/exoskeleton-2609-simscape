import csv
import numpy as np
from scipy.integrate import solve_ivp

# ★ 参数真源 = matlab2609/simscape/plant_params.csv（同 _sweep_dc.py）。
#   以前硬编码 I/A/B，密度表一改就会静默用旧模型扫参数。
_PP = r"C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\plant_params.csv"
with open(_PP, encoding="utf-8", newline="") as _f:
    _rows = list(csv.DictReader(_f))
_r = _rows[0]                                  # hip_L
I = float(_r["I_axis"]); A = float(_r["A"]); B = float(_r["B"])


def run(Tamp, f, Tend=6.0):
    def rhs(t, y):
        q, w = y
        T = Tamp*np.sin(2*np.pi*f*t)
        return [w, (T + A*np.sin(q) + B*np.cos(q))/I]
    sol = solve_ivp(rhs, [0, Tend], [0.0, 0.0], rtol=1e-10, atol=1e-12, dense_output=True)
    tt = np.linspace(0, Tend, 4000)
    q = sol.sol(tt)[0]
    return float(np.max(np.abs(q))), float(np.max(q)), float(np.min(q))


rows = []
for f in (0.3, 0.4, 0.5, 0.7, 0.9):
    for Tamp in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.45):
        mx, qmax, qmin = run(Tamp, f)
        rows.append((f, Tamp, mx, qmax, qmin))

lines = ["f[Hz] Tamp[Nm]  max|q|[rad]   qmax      qmin      tumble(|q|>pi)"]
lines.append("  参数源: plant_params.csv hip_L   I=%.9f  A=%+.9f  B=%+.9f" % (I, A, B))
for f, Tamp, mx, qmax, qmin in rows:
    lines.append(f"{f:5.2f} {Tamp:8.2f} {mx:12.4f} {qmax:+9.4f} {qmin:+9.4f}   {'YES' if mx > np.pi else '-'}")
open(r"C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\_sweep_drive.txt", "w", encoding="utf-8").write("\n".join(lines))
print("\n".join(lines))
