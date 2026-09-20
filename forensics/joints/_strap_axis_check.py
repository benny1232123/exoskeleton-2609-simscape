# -*- coding: utf-8 -*-
"""
诊断：新加的两根轴（abduct / strap）各自到底带动了哪些零件？
================================================================
对 leg_L 的每个零件算：
  · m            质量
  · c            CoM（世界系 mm）
  · d_ab         到 abduct 轴的垂距（mm）
  · d_st         到 strap  轴的垂距（mm）
  · m*d^2        该零件绕该轴的"点质量转动惯量"贡献（kg·mm^2）
  · I_axis       含零件自身惯量的该轴总转动惯量（kg·m^2）

判据：如果一根轴的 Σm·d² 里绝大部分来自"贴在轴上的小件"，那这根轴就是**销钉自转**
（装配级转轴），不是功能性屈伸。

轴的取法：直接从 exo_multidof.urdf 解析，沿 base→hip→abduct→strap 累积原点，
          零位时各 link frame 与世界系轴对齐，故 joint <axis> 即世界系方向。
"""
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)
from exo2609.geometry import (CadGeometry, density_of, MM3_TO_M3, MM5_TO_KGM2_PER_RHO)

SIM = os.path.join(ROOT, "matlab2609", "simscape")
URDF = os.path.join(SIM, "exo_multidof.urdf")
OUT = os.path.join(ROOT, "_strap_axis_check.txt")

L = []


def say(s=""):
    L.append(s)
    print(s)


# ------------------------------------------------------------ 从 URDF 取轴
def axes_from_urdf(path, side="L"):
    root = ET.parse(path).getroot()
    par = {}
    for j in root.findall("joint"):
        ax = j.find("axis")
        par[j.find("child").get("link")] = dict(
            name=j.get("name"), parent=j.find("parent").get("link"),
            o=np.array([float(x) for x in j.find("origin").get("xyz").split()]),
            a=(np.array([float(x) for x in ax.get("xyz").split()]) if ax is not None else None),
            ty=j.get("type"))
    O = {"base": np.zeros(3)}
    changed = True
    while changed:
        changed = False
        for ch, j in par.items():
            if j["parent"] in O and ch not in O:
                O[ch] = O[j["parent"]] + j["o"]
                changed = True
    out = {}
    for ch, j in par.items():
        if j["name"].endswith("_" + side):
            out[j["name"][: -len(side) - 1]] = (O[ch] * 1000.0, j["a"])
    return out


AX = axes_from_urdf(URDF, "L")

say("== exo_multidof.urdf 的轴（世界系，零位） ==")
for k in ("hip", "abduct", "strap"):
    p, d = AX[k]
    say("  %-7s point=[%9.3f %9.3f %9.3f] mm   dir=[%9.6f %9.6f %9.6f]"
        % (k, p[0], p[1], p[2], d[0], d[1], d[2]))
say("")

# ------------------------------------------------------------ 每个零件
cad = CadGeometry(ROOT)
geo = cad.build()
say("(build ok, 髋轴 point=%s dir=%s)"
    % (np.round(geo.axis_point, 3).tolist(), np.round(geo.axis_dir, 6).tolist()))
say("")

for group in ("leg_L", "leg_R"):
    solids, warns = cad._map_solids(group)
    per = {}
    for inst in cad.placed:
        if cad.group_of(inst) != group:
            continue
        Rm, t = inst.T[:3, :3], inst.T[:3, 3]
        rho, lab = density_of(inst.part_name, cad.densities, cad.fallback)
        for msb in inst.solids:
            sp = solids.get(msb)
            if sp is None:
                continue
            dv = rho * sp.volume_mm3 * MM3_TO_M3
            e = per.setdefault(inst.part_name, dict(m=0.0, mv=np.zeros(3),
                                                    I=np.zeros((3, 3)), lab=lab))
            e["m"] += dv
            e["mv"] += dv * (Rm @ sp.cog_local + t)
            e["I"] += Rm @ sp.inertia_local @ Rm.T * rho * MM5_TO_KGM2_PER_RHO

    side = group[-1]
    ax = axes_from_urdf(URDF, side)
    rows = []
    for nm, e in per.items():
        c = e["mv"] / e["m"]
        r = dict(nm=nm, m=e["m"], c=c, lab=e["lab"], I=e["I"])
        for k in ("abduct", "strap"):
            p, d = ax[k]
            d = d / np.linalg.norm(d)
            v = (c - p) - np.dot(c - p, d) * d
            rk = np.linalg.norm(v)                     # mm
            r["d_" + k] = rk
            du = d                                            # 单位向
            r["Icm_" + k] = float(du @ r["I"] @ du)           # 自身惯量在轴上的投影 kg·m²
            r["md2_" + k] = r["m"] * (rk * 1e-3) ** 2         # 点质量项 kg·m²
        rows.append(r)

    m_tot = sum(r["m"] for r in rows)
    say("=" * 100)
    say("## %s   总质量 %.6f kg   零件 %d 个" % (group, m_tot, len(rows)))
    say("=" * 100)
    for k in ("abduct", "strap"):
        Iax = sum(r["Icm_" + k] + r["md2_" + k] for r in rows)
        Imd = sum(r["md2_" + k] for r in rows)
        p, d = ax[k]
        say("  绕 %-6s 轴:  总 I = %.6e kg·m^2   (点质量项 %.6e, 自身项 %.6e)"
            % (k, Iax, Imd, Iax - Imd))
    say("")
    say("  %-26s %10s %16s %9s %11s %9s %11s"
        % ("零件", "m(kg)", "CoM(mm)", "d_ab(mm)", "m*dab^2", "d_st(mm)", "m*dst^2"))
    say("  " + "-" * 98)
    for r in sorted(rows, key=lambda z: -(z["md2_abduct"] + z["md2_strap"])):
        say("  %-26s %10.6f %16s %9.2f %11.3e %9.2f %11.3e"
            % (r["nm"][:26], r["m"],
               "[%7.1f %7.1f %7.1f]" % (r["c"][0], r["c"][1], r["c"][2]),
               r["d_abduct"], r["md2_abduct"], r["d_strap"], r["md2_strap"]))
    say("")
    # 结论：这根轴上"有效力臂"= 质量加权均方根半径
    for k in ("abduct", "strap"):
        Imd = sum(r["md2_" + k] for r in rows)
        say("  %-6s 轴 等效回转半径 = sqrt(Σm d²/Σm) = %.2f mm" % (k, np.sqrt(Imd / m_tot) * 1000))
    say("")

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(L))
print("WROTE", OUT)
