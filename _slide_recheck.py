# -*- coding: utf-8 -*-
"""
复查最下面那个自由度 `slide`：滑块到底在**哪根件**上滑、能滑多远？
====================================================================
疑点：按包围盒，滑块(69.18×40.55×30.96) 比 导轨(67.97×35.85×24.58) **还大**。
滑块比导轨还长 ⇒ 哪来的行程？这与「接触窗口口径 +18.6/−24.3mm」互相矛盾。

本脚本做三件事：
  (A) 关键零件沿 (s, n1, n2) 三个正交方向的**真实跨度**  —— 谁是长轨、谁是短滑块？
  (B) 放松阈值（间隙 ≤3mm、重叠 ≥2mm²）枚举滑块与其它件的贴合 —— 导轨是不是唯一的导轨？
  (C) 用「短件在长件内的可平移量」算行程上限，与已采用的 ±18.6/24.3mm 对照。
"""
import json
import os
import sys

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)
from exo2609.geometry import CadGeometry

PF = os.path.join(ROOT, "_plane_faces.json")
OUT = os.path.join(ROOT, "_slide_recheck.txt")

L = []


def say(s=""):
    L.append(s)
    print(s)


PFD = json.load(open(PF, encoding="utf-8"))
cad = CadGeometry(ROOT)
geo = cad.build()

S = np.array([0.423036, 0.008001, -0.906077]); S /= np.linalg.norm(S)
N1 = np.asarray(geo.axis_dir, float); N1 /= np.linalg.norm(N1)      # 髋轴 ≈ -Y
N2 = np.cross(S, N1); N2 /= np.linalg.norm(N2)
say("基: s=%s  n1(髋轴)=%s  n2=%s" % (np.round(S, 4).tolist(),
                                     np.round(N1, 4).tolist(), np.round(N2, 4).tolist()))
say("正交性检查: s·n1=%.2e  s·n2=%.2e  n1·n2=%.2e"
    % (S @ N1, S @ N2, N1 @ N2))
say("")

KEY = ["滑块_双键", "滑轨_片形", "滑轨挡片", "腿杆_片状V5", "绑缚", "按键_双", "轴盖", "螺丝_M3_4"]


def in_poly(pts, P):
    x, y = P[:, 0], P[:, 1]
    xs, ys = pts[:, 0], pts[:, 1]
    ins = np.zeros(len(P), dtype=bool)
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi, xj, yj = xs[i], ys[i], xs[j], ys[j]
        cond = ((yi > y) != (yj > y))
        with np.errstate(invalid="ignore", divide="ignore"):
            xin = x + (y - yi) / (yj - yi + 1e-300) * (xj - xi)
        ins ^= (cond & (x < xin))
        j = i
    return ins


def basis(n):
    t = np.array([1.0, 0, 0])
    if abs(float(t @ n)) > 0.9:
        t = np.array([0, 1.0, 0])
    u = t - n * float(t @ n); u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def ovl(a, b, n, res=48):
    u, w = basis(n)
    A = np.column_stack([a @ u, a @ w]); B = np.column_stack([b @ u, b @ w])
    lo = np.maximum(A.min(0), B.min(0)); hi = np.minimum(A.max(0), B.max(0))
    if np.any(hi - lo <= 0):
        return 0.0
    gx = np.linspace(lo[0], hi[0], res); gy = np.linspace(lo[1], hi[1], res)
    GX, GY = np.meshgrid(gx, gy)
    P = np.column_stack([GX.ravel(), GY.ravel()])
    return float((hi[0] - lo[0]) * (hi[1] - lo[1]) * (in_poly(A, P) & in_poly(B, P)).mean())


for side in ("L", "R"):
    faces = {}
    for inst in cad.placed:
        if cad.group_of(inst) != "leg_" + side:
            continue
        R, t = inst.T[:3, :3], inst.T[:3, 3]
        for msb in inst.solids:
            for f in PFD.get(side, {}).get(str(msb), []):
                poly = (R @ np.asarray(f["poly"], float).T).T + t
                n = R @ np.asarray(f["n"], float)
                faces.setdefault(inst.part_name, []).append((poly, n))

    say("=" * 100)
    say("## leg_%s" % side)
    say("=" * 100)

    # (A) 三向跨度
    say("")
    say("-- (A) 关键零件沿 (s 滑动向 / n1 髋轴 / n2) 的跨度 (mm) --")
    say("  %-28s %9s %9s %9s   %s" % ("零件", "沿s", "沿n1", "沿n2", "s区间"))
    say("  " + "-" * 96)
    ext = {}
    for nm in sorted(faces):
        if not any(k in nm for k in KEY):
            continue
        V = np.vstack([p for p, _ in faces[nm]])
        es = (V @ S); e1 = (V @ N1); e2 = (V @ N2)
        ext[nm] = (es.min(), es.max())
        say("  %-28s %9.2f %9.2f %9.2f   [%.2f, %.2f]"
            % (nm[:28], es.max() - es.min(), e1.max() - e1.min(),
               e2.max() - e2.min(), es.min(), es.max()))

    slid = [k for k in faces if "滑块_双键" in k][0]
    say("")
    say("-- (B) 滑块 %s 与其它件的贴合（放松到 间隙≤3mm / 重叠≥2mm²）--" % slid)
    say("  %-28s %9s %10s %9s   %s" % ("对方", "间隙mm", "重叠mm²", "cos", "法向"))
    say("  " + "-" * 96)
    rows = []
    for onm, of in faces.items():
        if onm == slid:
            continue
        for pa, na in faces[slid]:
            for pb, nb in of:
                c = float(na @ nb)
                if c > -0.9990:
                    continue
                gap = float(abs((pb.mean(0) - pa.mean(0)) @ na))
                if gap > 3.0:
                    continue
                o = ovl(pa, pb, na)
                if o < 2.0:
                    continue
                rows.append((o, gap, onm, na / np.linalg.norm(na)))
    rows.sort(key=lambda z: -z[0])
    for o, gap, onm, nv in rows[:25]:
        say("  %-28s %9.3f %10.2f %9.5f   [%5.2f %5.2f %5.2f]"
            % (onm[:28], gap, o, -1.0, nv[0], nv[1], nv[2]))
    if not rows:
        say("  （无）")

    # (C) 行程上限
    say("")
    say("-- (C) 「短件在长件内」的行程上限 = 对方沿s跨度 − 滑块沿s跨度 --")
    ss = ext[slid][1] - ext[slid][0]
    say("  滑块沿 s 跨度 = %.2f mm" % ss)
    for nm in sorted(ext):
        if nm == slid:
            continue
        e = ext[nm][1] - ext[nm][0]
        say("  %-28s 沿s跨度 %8.2f   差 %+8.2f mm %s"
            % (nm[:28], e, e - ss,
               "← 滑块更长，装不下" if e < ss else ""))
    say("")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(L))
print("WROTE", OUT)
