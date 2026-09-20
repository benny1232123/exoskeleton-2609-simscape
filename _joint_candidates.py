# -*- coding: utf-8 -*-
"""
腿机构「候选关节轴」取证 v3
==========================
v1 教训: 主 STEP 含整块电控板(无名小件半径 0.01~0.25mm) -> 必须先收敛到腿部设计_左/右。
v2 教训: 用「整腿面重心」做左右对照的基准 -> 会被个别远置件拖歪(出现 12 m 的轴点)。
v3 做法:
  1) 腿零件每件恰好 2 个实例(左/右), y 相差 ~434mm(髋宽), x/z 差 <1mm。
  2) 取一件两腿都有的参考件(默认 腿部_轴盖), 求 M = T_L @ inv(T_R),
     把**右腿所有面精确映射到左腿坐标系** -> 两腿几何重合。
  3) 在「左腿 + 已映射的右腿」上做两级聚类(方向平行 + 轴线垂距)。
     于是"左右对称的设计特征"会自动塌缩成同一个簇, 无需再猜配对。
  4) 位置以**到髋轴的垂距**表达(髋轴线在沿轴平移下不变, 两腿共用同一条线)。
输出 _joint_candidates.txt / _joint_candidates.json
"""
import io, json, math, os, re
from collections import defaultdict

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
TOL_DIR = 1e-3
TOL_DIST = 0.60
TOL_RFIT = 0.60
REF_PART = "腿部_轴盖"
FASTENER_KEYS = ("螺丝", "螺母", "顶丝", "垫片", "挡片", "按键", "销")
ANON = re.compile(r"^[0-9A-Za-z_\-\.]+$")

inst = json.load(io.open(os.path.join(ROOT, "_props_instances.json"), encoding="utf-8"))
cyl = json.load(io.open(os.path.join(ROOT, "_cyl_faces.json"), encoding="utf-8"))
hip = json.load(io.open(os.path.join(ROOT, "_hip_axis.json"), encoding="utf-8"))
hipd = np.asarray(hip["axis"]["dir"], float); hipd /= np.linalg.norm(hipd)
hipp = np.asarray(hip["axis"]["point"], float)

# ---------- 参考变换 M: 右腿 -> 左腿 ----------
TL = TR = None
for p in inst["placed"]:
    if (p.get("part_name") or "") != REF_PART:
        continue
    path = " / ".join(p.get("ancestors") or [])
    T = np.asarray(p["T"], float)
    if "左" in path:
        TL = T
    elif "右" in path:
        TR = T
assert TL is not None and TR is not None, "找不到参考件 %s 的左右实例" % REF_PART
M = TL @ np.linalg.inv(TR)          # 右腿世界系 -> 左腿坐标系
print("参考件=%s  M 平移=%s" % (REF_PART, np.round(M[:3, 3], 3)))
print("M 正交性(应为 I)=%s" % np.round(M[:3, :3] @ M[:3, :3].T, 6))

# ---------- 收集腿部面, 右腿映射到左腿系 ----------
recs = []
for p in inst["placed"]:
    path = " / ".join(p.get("ancestors") or []) + " / " + (p.get("anc") or "")
    if "腿部设计" not in path:
        continue
    side = "L" if "左" in path else ("R" if "右" in path else "?")
    T = np.asarray(p["T"], float)
    if side == "R":
        T = M @ T
    R, t = T[:3, :3], T[:3, 3]
    pname = p.get("part_name") or ""
    for msb in p["solids"]:
        for c in cyl.get(str(msb), []):
            o = R @ np.asarray(c["o"], float) + t
            d = R @ np.asarray(c["d"], float)
            nd = float(np.linalg.norm(d))
            if nd < 1e-12:
                continue
            recs.append((pname, side, o, d / nd, float(c["r"] or 0.0), c["t"]))

O = np.asarray([r[2] for r in recs]); D = np.asarray([r[3] for r in recs])
RR = np.asarray([r[4] for r in recs]); PN = [r[0] for r in recs]; SD = [r[1] for r in recs]

# ---- 剔除「分散摆放」的异常实例：腿长 ~0.31 m，离髋轴点 > 600 mm 的一律不可信 ----
far = np.linalg.norm(O - hipp, axis=1) > 600.0
if far.any():
    bad = defaultdict(int)
    for i in np.where(far)[0]:
        bad[PN[i]] += 1
    print("剔除远端实例面 %d 个，涉及零件: %s" % (int(far.sum()), dict(bad)))
keep = ~far
recs = [r for i, r in enumerate(recs) if keep[i]]
O = O[keep]; D = D[keep]; RR = RR[keep]
PN = [p for i, p in enumerate(PN) if keep[i]]
SD = [s for i, s in enumerate(SD) if keep[i]]

n = len(recs)
k = np.argmax(np.abs(D), axis=1)
DC = D * np.where(D[np.arange(n), k] > 0, 1.0, -1.0)[:, None]

# ---------- 两级聚类 ----------
reps, lab = [], np.full(n, -1)
for i in range(n):
    if reps:
        Rm = np.asarray(reps)
        cr = np.linalg.norm(np.cross(Rm, DC[i]), axis=1)
        j = int(np.argmin(cr))
        if cr[j] <= TOL_DIR:
            lab[i] = j; continue
    reps.append(DC[i]); lab[i] = len(reps) - 1

clusters = []
for g in range(len(reps)):
    sub = []
    for i in np.where(lab == g)[0]:
        hit = False
        for s in sub:
            if np.linalg.norm(np.cross(s[1], DC[i])) > TOL_DIR:
                continue
            v = O[i] - s[2]
            if np.linalg.norm(v - s[1] * float(v @ s[1])) <= TOL_DIST:
                s[0].append(i); hit = True; break
        if not hit:
            sub.append([[i], DC[i].copy(), O[i].copy()])
    clusters.extend(sub)

# ---------- 特征化 ----------
cands = []
for s in clusters:
    idx = np.asarray(s[0])
    names = sorted({PN[i] for i in idx})
    if len(names) < 2:
        continue
    sides = sorted({SD[i] for i in idx})
    d = s[1] / np.linalg.norm(s[1])
    v = O[idx] - O[idx[0]]
    pt = (O[idx[0]] + np.outer(v @ d, d)).mean(axis=0)

    by_r = defaultdict(set)
    for i in idx:
        if RR[i] > 0:
            by_r[round(float(RR[i]), 3)].add(PN[i])
    rs = sorted(by_r)
    fits, seen = [], set()
    for a in range(len(rs)):
        for b in range(a + 1, len(rs)):
            if abs(rs[a] - rs[b]) > TOL_RFIT:
                continue
            for pa in sorted(by_r[rs[a]]):
                for pb in sorted(by_r[rs[b]]):
                    if pa == pb:
                        continue
                    kk = tuple(sorted([pa, pb])) + (rs[a], rs[b])
                    if kk not in seen:
                        seen.add(kk); fits.append((pa, rs[a], pb, rs[b]))

    named = [x for x in names if not ANON.match(x)]
    struct = [x for x in named if not any(kk in x for kk in FASTENER_KEYS)]
    ab = np.clip(abs(float(d @ hipd)), 0, 1)
    ang = math.degrees(math.acos(ab))
    rel = pt - hipp
    r_hip = float(np.linalg.norm(rel - hipd * float(rel @ hipd)))
    # 运动面分类：与髋轴平行 = 同一运动平面(矢状面屈伸类)；垂直 = 额状面(内收外展)或横断面
    if ang < 5:
        plane = "平行髋轴(屈伸类平面)"
    elif ang > 85:
        plane = "垂直髋轴(内收外展/内外旋类)"
    else:
        plane = "斜交 %.0f°" % ang
    # 沿腿向下的深度：以髋轴为原点，取垂直分量在「腿伸展方向」上的投影
    #   腿伸展方向 ≈ 从髋轴指向该腿 bbox 重心的垂直分量
    cands.append({"names": names, "named": named, "struct": struct,
                  "n_faces": int(len(idx)), "sides": sides,
                  "dir": [round(float(x), 6) for x in d],
                  "pt": pt, "radii": rs[:12],
                  "fits": fits[:8], "n_fits": len(fits),
                  "vs_hip_deg": round(ang, 3), "plane": plane,
                  "r_from_hip_mm": round(r_hip, 2)})

cands.sort(key=lambda c: (-(len(c["struct"]) + (2 if c["n_fits"] else 0)), -c["n_faces"]))

L = []
L.append("腿机构候选关节轴 v3  —— 右腿经 %s 的刚体变换映射到左腿系后合并聚类" % REF_PART)
L.append("腿部面数 = %d (左+已映射右)   M 平移 = %s mm" % (n, np.round(M[:3, 3], 2).tolist()))
L.append("容差: 方向 %.0e / 轴线垂距 %.2f mm / 半径配对 %.2f mm" % (TOL_DIR, TOL_DIST, TOL_RFIT))
L.append("髋轴: 点=%s 方向=%s" % (np.round(hipp, 3).tolist(), np.round(hipd, 6).tolist()))
L.append("")
L.append("=" * 116)
L.append("候选（参与零件 >= 2）。★ = 含\"轴-孔半径配对\"；两腿都出现 = sides=[L,R]")
L.append("=" * 116)
for i, c in enumerate(cands[:28]):
    star = "★" if c["n_fits"] else " "
    L.append("%s#%-3d 结构件(%d)=%s" % (star, i, len(c["struct"]), ", ".join(c["struct"])))
    L.append("      全部零件=%s" % ", ".join(c["named"])[:140])
    L.append("      faces=%-4d sides=%-8s 半径=%-40s 配对=%d" %
             (c["n_faces"], ",".join(c["sides"]), str(c["radii"][:8]), c["n_fits"]))
    L.append("      方向=%s  与髋轴夹角=%.2f°  到髋轴垂距=%.1f mm" %
             (c["dir"], c["vs_hip_deg"], c["r_from_hip_mm"]))
    if c["fits"]:
        L.append("      轴-孔配对: " + "; ".join("%s r=%.2f <> %s r=%.2f" % f for f in c["fits"][:4]))
    L.append("")

io.open(os.path.join(ROOT, "_joint_candidates.txt"), "w", encoding="utf-8").write("\n".join(L) + "\n")
json.dump({"meta": {"faces": n, "ref": REF_PART, "M_trans_mm": M[:3, 3].tolist(),
                    "tol": [TOL_DIR, TOL_DIST, TOL_RFIT]},
           "candidates": [{k: (v.tolist() if isinstance(v, np.ndarray) else v)
                           for k, v in c.items()} for c in cands[:60]]},
          io.open(os.path.join(ROOT, "_joint_candidates.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)
print("候选数 = %d  WROTE report" % len(cands))
