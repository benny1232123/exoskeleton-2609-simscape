# -*- coding: utf-8 -*-
"""
算「滑块沿导轨」能滑多远 —— 给 URDF <joint type="prismatic"> 的 <limit>
====================================================================
滑动轴 s = 导轨/滑块/挡片 的公共长轴（⊥髋轴）。
行程判定用三种独立口径，互相印证：
  1) 引导保持：滑块接触面滑到导轨接触面之外前能走多远（重叠面积掉到 X% 以下）
  2) 硬限位：滑块多边形沿 +s/−s 平移，首次与「挡片」多边形在 s 上的间隙归零
  3) 导轨实体范围：滑块完全落在导轨实体 s 区间内的可平移量
"""
import json
import os
import sys

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)
from exo2609.geometry import CadGeometry

PF = os.path.join(ROOT, "_plane_faces.json")
OUT = os.path.join(ROOT, "_slide_range.txt")

L = []


def say(s=""):
    L.append(s)
    print(s)


PFD = json.load(open(PF, encoding="utf-8"))
cad = CadGeometry(ROOT)
geo = cad.build()

S = np.array([0.4230, 0.0080, -0.9060])
S = S / np.linalg.norm(S)
say("滑动轴 s (世界系, 单位) = %s" % np.round(S, 6).tolist())
say("  |s·hip_axis| = %.4f  (=%s 髋轴)"
    % (abs(float(S @ (np.asarray(geo.axis_dir, float) / np.linalg.norm(geo.axis_dir)))),
       "⊥" if abs(float(S @ (np.asarray(geo.axis_dir, float)
                             / np.linalg.norm(geo.axis_dir)))) < 0.05 else "∥"))
say("")


def parts_faces(side):
    out = {}
    for inst in cad.placed:
        if cad.group_of(inst) != "leg_" + side:
            continue
        R, t = inst.T[:3, :3], inst.T[:3, 3]
        for msb in inst.solids:
            for f in PFD.get(side, {}).get(str(msb), []):
                poly = (R @ np.asarray(f["poly"], float).T).T + t
                n = R @ np.asarray(f["n"], float)
                out.setdefault(inst.part_name, []).append((poly, n))
    return out


def in_poly(pts, P):
    x, y = P[:, 0], P[:, 1]
    xs, ys = pts[:, 0], pts[:, 1]
    inside = np.zeros(len(P), dtype=bool)
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi, xj, yj = xs[i], ys[i], xs[j], ys[j]
        cond = ((yi > y) != (yj > y))
        with np.errstate(invalid="ignore", divide="ignore"):
            xin = x + (y - yi) / (yj - yi + 1e-300) * (xj - xi)
        inside ^= (cond & (x < xin))
        j = i
    return inside


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
        return 0.0, None
    gx = np.linspace(lo[0], hi[0], res); gy = np.linspace(lo[1], hi[1], res)
    GX, GY = np.meshgrid(gx, gy)
    P = np.column_stack([GX.ravel(), GY.ravel()])
    frac = (in_poly(A, P) & in_poly(B, P)).mean()
    return float((hi[0] - lo[0]) * (hi[1] - lo[1]) * frac), (lo, hi)


for side in ("L", "R"):
    F = parts_faces(side)
    say("=" * 96)
    say("## leg_%s" % side)
    say("=" * 96)

    def key(sub):
        return [k for k in F if sub in k]

    rail = key("滑轨_片形")[0]
    slid = key("滑块_双键")[0]

    # ---- (A) 各零件沿 s 的投影区间 ----
    say("")
    say("-- (A) 沿 s 的投影区间 (mm) --")
    say("  %-26s %10s %10s %10s" % ("零件", "min", "max", "长度"))
    pr = {}
    for nm in sorted(F):
        V = np.vstack([p for p, _ in F[nm]])
        q = V @ S
        pr[nm] = (float(q.min()), float(q.max()))
        say("  %-26s %10.2f %10.2f %10.2f" % (nm[:26], q.min(), q.max(), q.max() - q.min()))

    # ---- (B) 引导保持：接触面沿 s 的相对窗口 ----
    say("")
    say("-- (B) 滑块↔导轨 4 组贴合面，沿 s 的重叠窗口与可滑量 --")
    say("  %-22s %10s %10s %10s %10s" % ("法向", "窗口长mm", "可滑+mm", "可滑−mm", "重叠mm²"))
    lim_p, lim_n = [], []
    for pa, na in F[slid]:
        for pb, nb in F[rail]:
            if float(na @ nb) > -0.9995:
                continue
            gap = float(abs((pb.mean(0) - pa.mean(0)) @ na))
            if gap > 1.2:
                continue
            a_, b_ = pa @ S, pb @ S
            lo = max(a_.min(), b_.min()); hi = min(a_.max(), b_.max())
            if hi <= lo:
                continue
            win = hi - lo
            # 滑块沿 +s 平移 δ 后仍保持重叠：a_.min()+δ < b_.max()
            tp = b_.max() - a_.min()
            tn = a_.max() - b_.min()
            o, _ = ovl(pa, pb, na)
            if o < 5:
                continue
            lim_p.append(tp); lim_n.append(tn)
            say("  %-22s %10.2f %10.2f %10.2f %10.2f"
                % ("[%5.2f %5.2f %5.2f]" % tuple(na / np.linalg.norm(na)),
                   win, tp, tn, o))
    say("  >> 引导保持口径:  +s 可滑 %.2f mm,  −s 可滑 %.2f mm"
        % (min(lim_p), min(lim_n)))

    # ---- (C) 硬限位：挡片 ----
    say("")
    say("-- (C) 硬限位（挡片） --")
    stop = key("滑轨挡片")
    if stop:
        st = stop[0]
        Vs = np.vstack([p for p, _ in F[st]]) @ S
        Vk = np.vstack([p for p, _ in F[slid]]) @ S
        say("  挡片 %s 沿 s: [%.2f, %.2f]" % (st, Vs.min(), Vs.max()))
        say("  滑块 沿 s: [%.2f, %.2f]" % (Vk.min(), Vk.max()))
        d_p = Vs.min() - Vk.max()
        d_n = Vk.min() - Vs.max()
        say("  >> 到挡片的间隙:  +s 方向 %.2f mm,  −s 方向 %.2f mm" % (d_p, d_n))
        say("     (正值=还能走这么远；负值=已重叠，说明当前位姿就是限位位)")

    # ---- (D) 导轨实体范围 ----
    Vr = np.vstack([p for p, _ in F[rail]]) @ S
    Vk = np.vstack([p for p, _ in F[slid]]) @ S
    say("")
    say("-- (D) 导轨实体 s 区间 [%.2f, %.2f]  vs  滑块 [%.2f, %.2f] --"
        % (Vr.min(), Vr.max(), Vk.min(), Vk.max()))
    say("  导轨长 %.2f mm, 滑块长 %.2f mm  → 若以「滑块不脱出导轨」为界，"
        "理论行程 %.2f mm" % (Vr.max() - Vr.min(), Vk.max() - Vk.min(),
                            (Vr.max() - Vr.min()) - (Vk.max() - Vk.min())))
    say("")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(L))
print("WROTE", OUT)
