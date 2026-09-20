# -*- coding: utf-8 -*-
"""
自由度取证图
============
把「与髋轴垂直（= 会让腿向内侧/外侧摆）」的圆柱配合从整条腿里挑出来画出来。

输入: _props_instances.json / _cyl_faces.json / _hip_axis.json / meshes/leg_L.stl
输出: out_simscape/dof_forensic.png  +  _dof_forensic.txt
"""
import io, json, math, os, struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict

matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

ROOT = r"C:\Users\29408\Desktop\外骨骼"
MESH = os.path.join(ROOT, "matlab2609", "simscape", "meshes", "leg_L.stl")
OUT_PNG = os.path.join(ROOT, "matlab2609", "out_simscape", "dof_forensic.png")
OUT_TXT = os.path.join(ROOT, "_dof_forensic.txt")

inst = json.load(io.open(os.path.join(ROOT, "_props_instances.json"), encoding="utf-8"))
cyl = json.load(io.open(os.path.join(ROOT, "_cyl_faces.json"), encoding="utf-8"))
hip = json.load(io.open(os.path.join(ROOT, "_hip_axis.json"), encoding="utf-8"))
hipd = np.asarray(hip["axis"]["dir"], float); hipd /= np.linalg.norm(hipd)
hipp = np.asarray(hip["axis"]["point"], float)


def read_stl(path):
    """返回 (V, F)。支持二进制/ASCII。单位与源文件一致(此项目为 mm)。"""
    with open(path, "rb") as f:
        head = f.read(84)
        if len(head) < 84:
            return None, None
        n = struct.unpack("<I", head[80:84])[0]
        f.seek(0, 2)
        size = f.tell()
        if size == 84 + n * 50:                       # 二进制
            f.seek(84)
            raw = np.fromfile(f, dtype=np.uint8, count=n * 50).reshape(n, 50)
            tri = raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3)
            return tri.reshape(-1, 3).astype(float), np.arange(n * 3).reshape(n, 3)
        f.seek(0)                                      # ASCII
    txt = io.open(path, "r", errors="ignore").read()
    vs = [list(map(float, L.split()[1:4])) for L in txt.splitlines() if L.strip().startswith("vertex")]
    V = np.asarray(vs, float)
    return V, np.arange(len(V)).reshape(-1, 3)


# ---------- 腿零件面 -> 世界系(只取左腿) ----------
recs = []
for p in inst["placed"]:
    path = " / ".join(p.get("ancestors") or []) + " / " + (p.get("anc") or "")
    if "腿部设计" not in path or "左" not in path:
        continue
    T = np.asarray(p["T"], float)
    R, t = T[:3, :3], T[:3, 3]
    pname = p.get("part_name") or ""
    for msb in p["solids"]:
        for c in cyl.get(str(msb), []):
            o = R @ np.asarray(c["o"], float) + t
            d = R @ np.asarray(c["d"], float)
            nd = float(np.linalg.norm(d))
            if nd < 1e-12:
                continue
            recs.append((pname, o, d / nd, float(c["r"] or 0.0)))

O = np.asarray([r[1] for r in recs]); D = np.asarray([r[2] for r in recs])
RR = np.asarray([r[3] for r in recs]); PN = [r[0] for r in recs]
far = np.linalg.norm(O - hipp, axis=1) > 600.0
bad = defaultdict(int)
for i in np.where(far)[0]:
    bad[PN[i]] += 1
O, D, RR = O[~far], D[~far], RR[~far]
PN = [p for i, p in enumerate(PN) if not far[i]]

ab = np.clip(np.abs(D @ hipd), 0, 1)
ang = np.degrees(np.arccos(ab))               # 与髋轴夹角
perp = ang > 85.0                             # ⟂ 髋轴 -> 落在 XZ 面 -> 左右(内收/外展)运动
para = ang < 5.0                              # ∥ 髋轴 -> 屈伸
# 只有「半径 3~25mm」的圆柱面才可能是真正的轴/孔配合；
# r<3mm 是 M3 螺纹/倒角/卡扣面；r>25mm 多半是零件外形轮廓圆弧(如绑带环)，都不是关节证据。
BIG = (RR >= 3.0) & (RR <= 25.0)
SMALL = ~BIG

rel = O - hipp
r_hip = np.linalg.norm(rel - np.outer(rel @ hipd, hipd), axis=1)

_t = np.linspace(-0.16, 0.16, 2)                      # 髋轴线段(m)，供各子图复用
HL = hipp / 1000.0 + np.outer(_t * 1000.0, hipd) / 1000.0

# ---------- 图 ----------
V, _ = read_stl(MESH)
fig = plt.figure(figsize=(16.5, 9.6))

def scatter(ax, idxs, title, eq=False):
    rng = np.random.default_rng(0)
    def put(P, **kw):
        ax.scatter(*[P[:, k] for k in idxs], **kw)
    if V is not None:
        P = V[rng.choice(len(V), size=min(60000, len(V)), replace=False)]
        put(P, s=0.6, c="#d9d9d9", linewidths=0, alpha=.55, label="腿外形网格")
    put(O[SMALL], s=5, c="#c9c9c9", marker="o", linewidths=0, label="其它面（r<3 螺纹·倒角 / r>25 外形弧）")
    put(O[para & BIG], s=30, c="#1f77b4", marker="o", linewidths=.4, edgecolors="k", label="轴∥髋轴 & r=3~25（沿铰链方向的孔/轴）")
    put(O[perp & BIG], s=52, c="#d62728", marker="s", linewidths=.5, edgecolors="k", label="轴⊥髋轴 & r=3~25（左右/内收·内旋类）")
    # 髋轴
    ax.plot(*[HL[:, k] for k in idxs], "k--", lw=2.2, label="髋轴(唯一建模自由度)")
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("x / m"); ax.set_ylabel("y / m" if idxs[1] == 1 else "z / m")
    ax.grid(alpha=.25)
    if eq:
        ax.set_zlabel("z / m")

ax1 = fig.add_subplot(2, 3, 1, projection="3d")
scatter(ax1, (0, 1, 2), "(a) 等轴测：红色=轴线垂直髋轴的结构", eq=True)

ax2 = fig.add_subplot(2, 3, 2)
scatter(ax2, (0, 1), "(b) 俯视 x–y：左右方向（“向内”就是这个方向）")
ax2.set_aspect("equal")

ax3 = fig.add_subplot(2, 3, 3)
scatter(ax3, (0, 2), "(c) 侧视 x–z")
ax3.set_aspect("equal")

# 把左右腿两份 + 髋轴做放大专图
ax4 = fig.add_subplot(2, 3, 4)
CEN = defaultdict(list)
for i in np.where(perp & BIG)[0]:
    CEN[PN[i]].append(O[i, :2])
for nm in sorted(CEN, key=lambda k: -len(CEN[k])):
    P = np.asarray(CEN[nm]).mean(axis=0)
    ax4.scatter(P[0], P[1], s=30 + 8 * len(CEN[nm]), c="#d62728", marker="s",
                linewidths=.8, edgecolors="k")
    ax4.annotate("%s (n=%d)" % (nm, len(CEN[nm])), (P[0], P[1]), fontsize=8.5,
                 xytext=(8, 5), textcoords="offset points",
                 arrowprops=dict(arrowstyle="-", lw=.7, color="#888"))
ax4.plot(HL[:, 0] * 1000, HL[:, 1] * 1000, "k--", lw=2, label="髋轴")
ax4.set_xlim(60, 470)
ax4.set_title("(d) “⊥髋轴 & r=3~25mm”的零件位置（同一零件合并为一点）", fontsize=10.5)
ax4.set_xlabel("x / mm"); ax4.set_ylabel("y / mm"); ax4.grid(alpha=.25); ax4.set_aspect("equal")

# 图例
ax5 = fig.add_subplot(2, 3, 5); ax5.axis("off")
h, l = ax2.get_legend_handles_labels()
ax5.legend(h, l, loc="upper left", fontsize=10, frameon=False, title="图例")

# 数值摘要
ax6 = fig.add_subplot(2, 3, 6); ax6.axis("off")
lines = []
n_perp = int((perp & BIG).sum()); n_para = int((para & BIG).sum())
lines.append("腿部圆柱面(左腿) %d 个（剔远端 %d）" % (len(O), int(far.sum())))
lines.append("其中 r>=3mm(=Ø>=6) 的面: %d" % int(BIG.sum()))
lines.append("")
lines.append("⊥髋轴 & r>=3mm : %d    ∥髋轴 & r>=3mm : %d" % (n_perp, n_para))
lines.append("")
lines.append("【⊥髋轴 & r>=3mm】候选按零件:")
grp = defaultdict(list)
for i in np.where(perp & BIG)[0]:
    grp[PN[i]].append(i)
for nm in sorted(grp, key=lambda k: -len(grp[k])):
    idx = grp[nm]
    rr = sorted({round(float(RR[i]), 2) for i in idx})
    a = float(np.mean(ang[idx]))
    lines.append("  %-26s n=%-3d r=%-20s 夹角%.1f° 距髋%.0fmm"
                 % (nm, len(idx), str(rr[:5]), a, float(np.mean(r_hip[idx]))))
lines.append("")
lines.append("【∥髋轴 & r>=3mm】候选按零件:")
grp2 = defaultdict(list)
for i in np.where(para & BIG)[0]:
    grp2[PN[i]].append(i)
for nm in sorted(grp2, key=lambda k: -len(grp2[k])):
    idx = grp2[nm]
    rr = sorted({round(float(RR[i]), 2) for i in idx})
    a = float(np.mean(ang[idx]))
    lines.append("  %-26s n=%-3d r=%-20s 夹角%.1f° 距髋%.0fmm"
                 % (nm, len(idx), str(rr[:5]), a, float(np.mean(r_hip[idx]))))
ax6.text(0, 1, "\n".join(lines), va="top", ha="left", fontsize=9)
fig.suptitle("外骨骼腿部「候选关节轴」取证图  ——  模型实建自由度 = 2（仅 hip_L / hip_R，轴∥Y）",
             fontsize=13.5, y=0.985)
fig.tight_layout(rect=[0, 0, 1, 0.955])
os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
fig.savefig(OUT_PNG, dpi=140)
io.open(OUT_TXT, "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines))
print("WROTE", OUT_PNG)
print("mesh verts:", 0 if V is None else len(V))
