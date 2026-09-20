# -*- coding: utf-8 -*-
"""
第三个候选取证：导轨/滑块 到底是不是**移动副**？
==================================================
判据（不靠零件名，只看几何）
  一个「滑动接触」的签名 = 两个**不同零件**的平面面满足：
    (a) 平行且相对（n1·n2 ≈ −1，面对面）
    (b) 间隙小（0 ~ 1.0 mm，配合/油膜级）
    (c) 面内**投影重叠面积**足够大（真贴合，不是"恰好共面但离得远"）
  若导轨与滑块之间存在这样一组（且不止一组，导轨两侧各一组 ⇒ 形成燕尾/槽约束），
  ⇒ 直线滑移成立；滑动方向 = 该平面内、导轨的**长轴方向**。

数据: _plane_faces.json（局部坐标） × Instance 变换（世界系）
"""
import os
import sys

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)
from exo2609.geometry import CadGeometry

PF = os.path.join(ROOT, "_plane_faces.json")
OUT = os.path.join(ROOT, "_slide_check.txt")

GAP_MAX = 1.2         # mm  配合间隙上限
OVL_MIN = 8.0         # mm^2 最小重叠面积
DOT_FACE = -0.9995    # 面对面阈值

L = []


def say(s=""):
    L.append(s)
    print(s)


# ---------------------------------------------------------------- 载入
import json
PFD = json.load(open(PF, encoding="utf-8"))
cad = CadGeometry(ROOT)
geo = cad.build()

hip_p = np.asarray(geo.axis_point, float)
hip_a = np.asarray(geo.axis_dir, float)
hip_a /= np.linalg.norm(hip_a)


def plane_basis(n):
    t = np.array([1.0, 0.0, 0.0])
    if abs(float(t @ n)) > 0.9:
        t = np.array([0.0, 1.0, 0.0])
    u = t - n * float(t @ n)
    u /= np.linalg.norm(u)
    w = np.cross(n, u)
    return u, w


def in_poly(pts, P):
    """射线法：P(N,2) 是否在多边形 pts(M,2) 内。"""
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


def overlap_area(pa, pb, n, res=44):
    """两个三维多边形在同一平面法向 n 下的投影重叠面积（mm^2）。"""
    u, w = plane_basis(n)
    A = np.asarray(pa, float); B = np.asarray(pb, float)
    A2 = np.column_stack([A @ u, A @ w])
    B2 = np.column_stack([B @ u, B @ w])
    lo = np.maximum(A2.min(0), B2.min(0))
    hi = np.minimum(A2.max(0), B2.max(0))
    if np.any(hi - lo <= 0):
        return 0.0
    gx = np.linspace(lo[0], hi[0], res)
    gy = np.linspace(lo[1], hi[1], res)
    GX, GY = np.meshgrid(gx, gy)
    P = np.column_stack([GX.ravel(), GY.ravel()])
    ok = in_poly(A2, P) & in_poly(B2, P)
    frac = ok.mean()
    return float((hi[0] - lo[0]) * (hi[1] - lo[1]) * frac)


# ---------------------------------------------------------------- 世界系平面面 / 每零件
def build(side):
    faces = []          # (part, n_world, o_world, poly_world, area)
    for inst in cad.placed:
        if cad.group_of(inst) != "leg_" + side:
            continue
        R, t = inst.T[:3, :3], inst.T[:3, 3]
        for msb in inst.solids:
            for f in PFD.get(side, {}).get(str(msb), []):
                n = R @ np.asarray(f["n"], float)
                o = R @ np.asarray(f["o"], float) + t
                poly = (R @ np.asarray(f["poly"], float).T).T + t
                faces.append(dict(part=inst.part_name, n=n, o=o, poly=poly,
                                  area=float(f["area"])))
    return faces


for side in ("L", "R"):
    faces = build(side)
    parts = sorted({f["part"] for f in faces})
    say("=" * 104)
    say("## leg_%s   平面面 %d 个，来自 %d 个零件" % (side, len(faces), len(parts)))
    say("=" * 104)

    # ---- 每零件包围盒（用平面面顶点近似）----
    say("")
    say("  %-30s %8s %8s %8s   %s" % ("零件", "长", "中", "短", "长轴方向(世界系)"))
    say("  " + "-" * 100)
    bbox = {}
    for p in parts:
        V = np.vstack([f["poly"] for f in faces if f["part"] == p])
        bbox[p] = V
        lo, hi = V.min(0), V.max(0)
        ext = hi - lo
        order = np.argsort(-ext)
        ax = np.zeros(3); ax[order[0]] = 1.0
        # 长轴 = 点云 PCA 第一主成分（比 bbox 轴更稳）
        C = V - V.mean(0)
        ev, evec = np.linalg.eigh(C.T @ C)
        long_ax = evec[:, int(np.argmax(ev))]
        if long_ax[0] < 0:
            long_ax = -long_ax
        say("  %-30s %8.2f %8.2f %8.2f   [%6.3f %6.3f %6.3f]"
            % (p[:30], ext[order[0]], ext[order[1]], ext[order[2]], *long_ax))

    # ---- 配对扫描 ----
    say("")
    say("-- 滑动接触候选（不同零件 / 面对面 / 间隙 ≤ %.1fmm / 重叠 ≥ %.0fmm²）--"
        % (GAP_MAX, OVL_MIN))
    say("  %-26s %-26s %7s %9s %10s %8s"
        % ("零件A", "零件B", "cos", "间隙mm", "重叠mm²", "法向"))
    say("  " + "-" * 100)
    pairs = []
    for i in range(len(faces)):
        for j in range(i + 1, len(faces)):
            a, b = faces[i], faces[j]
            if a["part"] == b["part"]:
                continue
            c = float(a["n"] @ b["n"])
            if c > DOT_FACE:
                continue
            gap = float(abs((b["o"] - a["o"]) @ a["n"]))
            if gap > GAP_MAX:
                continue
            ov = overlap_area(a["poly"], b["poly"], a["n"])
            if ov < OVL_MIN:
                continue
            pairs.append((ov, gap, a, b, c))
    pairs.sort(key=lambda z: -z[0])
    for ov, gap, a, b, c in pairs[:40]:
        nrm = a["n"] / np.linalg.norm(a["n"])
        say("  %-26s %-26s %7.4f %9.3f %10.2f %8s"
            % (a["part"][:26], b["part"][:26], c, gap, ov,
               "[%5.2f %5.2f %5.2f]" % (nrm[0], nrm[1], nrm[2])))
    if not pairs:
        say("  （无）")
    say("")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(L))
print("WROTE", OUT)
