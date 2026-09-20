# -*- coding: utf-8 -*-
"""
从 STEP 抽出**平面面**（PLANE）及其真实足迹多边形
=================================================
`_axis_find.py` 只抽了 CYL/TOR —— 于是"导轨/滑块"这类**平面配合**从来没被看过，
这就是第三个候选（移动副）一直"没有几何证据"的根因。本脚本把它补上。

拓扑链（STEP AP203/214）
  MANIFOLD_SOLID_BREP -> CLOSED_SHELL -> ADVANCED_FACE -> FACE_BOUND/FACE_OUTER_BOUND
                                                      -> EDGE_LOOP -> ORIENTED_EDGE
                                                      -> EDGE_CURVE -> VERTEX_POINT
                                                      -> CARTESIAN_POINT
  ADVANCED_FACE 的最后一个引用 = 曲面；PLANE 的引用[0] = AXIS2_PLACEMENT_3D
  AXIS2_PLACEMENT_3D 的引用 = [location(CARTESIAN_POINT), axis(DIRECTION), refdir(DIRECTION)]

输出: _plane_faces.json
  { "<msb_id>": [ {"n":[nx,ny,nz], "o":[x,y,z], "area":mm^2,
                    "uv":[[u1,v1],[u2,v2]],        # 面内 2D 包围盒
                    "poly":[[x,y,z], ...]          # 足迹多边形顶点
                  } ] }
坐标: **零件局部坐标**（与 _cyl_faces.json 一致），跨零件比较时用 Instance 变换变到世界系。
"""
import io
import json
import os
import re
import time

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
SRC = {s: os.path.join(ROOT, "_reduced3", "leg_%s.stp" % s) for s in ("L", "R")}
OUT = os.path.join(ROOT, "_plane_faces.json")
LOG = os.path.join(ROOT, "_plane_faces.log")

T0 = time.time()
_lf = io.open(LOG, "w", encoding="utf-8", buffering=1)


def log(m):
    _lf.write("[%7.1fs] %s\n" % (time.time() - T0, m))
    _lf.flush()


RE_ENT = re.compile(r"^#(\d+)\s*=\s*\(?\s*([A-Z0-9_]+)\s*\((.*)\)\s*\)?\s*;\s*$", re.DOTALL)
RE_STR = re.compile(r"'((?:[^']|'')*)'")
RE_REF = re.compile(r"#(\d+)")
RE_NUM = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")


def refs(a):
    return [int(m.group(1)) for m in RE_REF.finditer(a)]


def nums(a):
    c = RE_STR.sub(" ", a)
    c = RE_REF.sub(" ", c)
    return [float(x) for x in RE_NUM.findall(c)]


def parse(path, want_types):
    """流式扫描，只保留 want_types 里的实体。返回 {id: (type, args)} + 类型直方图。"""
    ent = {}
    hist = {}
    buf = b""
    with io.open(path, "r", encoding="latin-1", errors="replace") as f:
        for line in f:
            if not buf:
                if not line.startswith("#"):
                    continue
                buf = line
            else:
                buf += line
            if not line.rstrip("\r\n").endswith(";"):
                continue
            stmt = buf.replace("\r", " ").replace("\n", " ")
            buf = ""
            m = RE_ENT.match(stmt)
            if not m:
                continue
            eid, et, args = int(m.group(1)), m.group(2), m.group(3)
            hist[et] = hist.get(et, 0) + 1
            if et in want_types:
                ent[eid] = (et, args)
    return ent, hist


WANT = {"MANIFOLD_SOLID_BREP", "FACETED_BREP", "BREP_WITH_VOIDS",
        "CLOSED_SHELL", "OPEN_SHELL",
        "ADVANCED_FACE", "FACE_BOUND", "FACE_OUTER_BOUND",
        "EDGE_LOOP", "ORIENTED_EDGE", "EDGE_CURVE",
        "VERTEX_POINT", "CARTESIAN_POINT",
        "PLANE", "AXIS2_PLACEMENT_3D", "DIRECTION"}

result = {}
for side, path in SRC.items():
    log("=" * 70)
    log("side=%s  %s" % (side, path))
    ent, hist = parse(path, WANT)
    log("  entities kept=%d" % len(ent))
    top = sorted(hist.items(), key=lambda kv: -kv[1])[:8]
    log("  top types: %s" % top)

    # ---- 小表 ----
    P = {}          # CARTESIAN_POINT -> xyz
    D = {}          # DIRECTION -> unit vec
    A2P = {}        # AXIS2_PLACEMENT_3D -> (loc_id, axis_id, refdir_id)
    PL = {}         # PLANE -> a2p_id
    for eid, (et, a) in ent.items():
        if et == "CARTESIAN_POINT":
            v = nums(a)
            if len(v) >= 3:
                P[eid] = (v[0], v[1], v[2])
        elif et == "DIRECTION":
            v = nums(a)
            if len(v) >= 2:
                n = np.array([v[0], v[1], v[2] if len(v) >= 3 else 0.0])
                nn = np.linalg.norm(n)
                if nn > 1e-12:
                    D[eid] = n / nn
        elif et == "AXIS2_PLACEMENT_3D":
            r = refs(a)
            A2P[eid] = (r[0] if len(r) > 0 else None,
                        r[1] if len(r) > 1 else None,
                        r[2] if len(r) > 2 else None)
        elif et == "PLANE":
            r = refs(a)
            if r:
                PL[eid] = r[0]
    log("  points=%d dirs=%d a2p=%d planes=%d" % (len(P), len(D), len(A2P), len(PL)))

    # ---- 顶点 / 边 ----
    VP = {}         # VERTEX_POINT -> xyz
    for eid, (et, a) in ent.items():
        if et == "VERTEX_POINT":
            r = refs(a)
            if r and r[0] in P:
                VP[eid] = P[r[0]]
    EC = {}         # EDGE_CURVE -> (p1, p2)
    for eid, (et, a) in ent.items():
        if et == "EDGE_CURVE":
            r = refs(a)
            if len(r) >= 2 and r[0] in VP and r[1] in VP:
                EC[eid] = (VP[r[0]], VP[r[1]])
    OE = {}         # ORIENTED_EDGE -> edge_curve id
    for eid, (et, a) in ent.items():
        if et == "ORIENTED_EDGE":
            r = refs(a)
            if r:
                OE[eid] = r[-1]
    EL = {}         # EDGE_LOOP -> [oe ids]
    for eid, (et, a) in ent.items():
        if et == "EDGE_LOOP":
            EL[eid] = refs(a)
    FB = {}         # FACE_BOUND/OUTER -> loop id
    for eid, (et, a) in ent.items():
        if et in ("FACE_BOUND", "FACE_OUTER_BOUND"):
            r = refs(a)
            if r:
                FB[eid] = r[0]
    log("  verts=%d edgecurves=%d oriented=%d loops=%d fbounds=%d"
        % (len(VP), len(EC), len(OE), len(EL), len(FB)))

    # ---- 面 ----
    def face_poly(face_id):
        """返回该面的足迹顶点列表（顺序可能不连续，够用即可）。"""
        et, a = ent[face_id]
        r = refs(a)
        if not r:
            return []
        loop_ids = [FB[b] for b in r[:-1] if b in FB]
        pts = []
        for lid in loop_ids:
            for oe in EL.get(lid, []):
                ec = OE.get(oe)
                if ec in EC:
                    pts.append(EC[ec][0]); pts.append(EC[ec][1])
        return pts

    msb_faces = {}
    nplane = 0
    for eid, (et, a) in ent.items():
        if et not in ("MANIFOLD_SOLID_BREP", "FACETED_BREP", "BREP_WITH_VOIDS"):
            continue
        r = refs(a)
        if not r:
            continue
        out = []
        for sh in r:
            if sh not in ent:
                continue
            for f in refs(ent[sh][1]):
                if f not in ent:
                    continue
                fa = ent[f][1]
                fr = refs(fa)
                if not fr:
                    continue
                sid = fr[-1]
                if sid not in PL:
                    continue                      # 只留平面面
                a2p = PL[sid]
                if a2p not in A2P:
                    continue
                loc, ax, rd = A2P[a2p]
                if loc not in P or ax not in D:
                    continue
                n = D[ax]
                o = np.array(P[loc], float)
                poly = face_poly(f)
                if len(poly) < 3:
                    continue
                Q = np.asarray(poly, float)
                # 面内 2D 坐标
                u = D[rd] if (rd in D) else None
                if u is None or abs(float(u @ n)) > 0.9:
                    # refdir 退化 -> 任取一个与 n 不平行的向量
                    t = np.array([1.0, 0, 0])
                    if abs(float(t @ n)) > 0.9:
                        t = np.array([0, 1.0, 0])
                    u = t - n * float(t @ n)
                    u = u / np.linalg.norm(u)
                w = np.cross(n, u)
                rel = Q - o
                uu = rel @ u
                vv = rel @ w
                # 面积（鞋带公式，顶点顺序不可靠时退化为 2D 包围盒面积）
                A = 0.5 * abs(float(np.cross(np.column_stack([uu, vv])[:-1],
                                             np.column_stack([uu, vv])[1:]).sum()))
                out.append(dict(n=n.round(9).tolist(), o=o.round(6).tolist(),
                                area=round(float(A), 4),
                                uv=[[round(float(uu.min()), 4), round(float(vv.min()), 4)],
                                     [round(float(uu.max()), 4), round(float(vv.max()), 4)]],
                                poly=[[round(float(x), 4) for x in p] for p in Q]))
                nplane += 1
        if out:
            msb_faces[str(eid)] = out

    result[side] = msb_faces
    log("  solids with plane faces = %d   total plane faces = %d"
        % (len(msb_faces), nplane))
    log("  msb ids: %s" % sorted(int(k) for k in msb_faces)[:20])

with io.open(OUT, "w", encoding="utf-8") as f:
    json.dump(result, f, ensure_ascii=False)
log("WROTE %s" % OUT)
log("DONE")
