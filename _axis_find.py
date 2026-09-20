# -*- coding: utf-8 -*-
"""
从 STEP 提取每个 solid 的**圆柱/环形面轴线**（局部坐标）。
用途: 反推髋关节转轴（电机出轴 / 黄铜轴套 的公共轴心）。

设计要点（踩过的坑）
--------------------
* STEP 实体**不保证依赖顺序**：MANIFOLD_SOLID_BREP(#85945) 引用的 CLOSED_SHELL
  可能是 #86474（id 更大）。所以一律「先存引用，遍历完再统一解析」，
  不要边扫边查（旧版查 types 会全部落空）。
* ADVANCED_FACE 的参数是 (name, (bounds...), face_geometry, same_sense)，
  **最后一个引用就是曲面**，不必依赖类型表。
* 数字提取前必须先剥掉字符串和 `#id` 引用，否则会把 entity id 的数字当半径。

输出: _cyl_faces.json
  { "<msb_id>": [ {"t":"CYL|TOR","r":major,"r2":minor,"o":[x,y,z],"d":[x,y,z]} ] }
"""

import io, json, os, re, time

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
STEP = os.path.join(ROOT, "“林-Ⅰ”髋关节外骨骼机器人开发平台V0_1_1.stp")
OUT = os.path.join(ROOT, "_cyl_faces.json")
LOG = os.path.join(ROOT, "_cyl.log")

_lf = io.open(LOG, "w", encoding="utf-8", buffering=1)
T0 = time.time()


def log(m):
    _lf.write("[%7.1fs] %s\n" % (time.time() - T0, m)); _lf.flush()


RE_ENT = re.compile(r"^#(\d+)\s*=\s*\(?\s*([A-Z0-9_]+)\s*\((.*)\)\s*\)?\s*;\s*$", re.DOTALL)
RE_STR = re.compile(r"'((?:[^']|'')*)'")
RE_REF = re.compile(r"#(\d+)")
RE_NUM = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")


def refs(a):
    return [int(m.group(1)) for m in RE_REF.finditer(a)]


def nums_noref(a):
    """剥掉字符串与 #id 后取全部数字。"""
    c = RE_STR.sub(" ", a)
    c = RE_REF.sub(" ", c)
    return [float(x) for x in RE_NUM.findall(c)]


def nums_in_last_paren(a):
    c = RE_STR.sub("''", a)
    g = re.findall(r"\(([^()]*)\)", c)
    return [float(x) for x in RE_NUM.findall(g[-1])] if g else []


shell_faces = {}      # shell_id -> [face_ids]
face_lastref = {}     # face_id  -> 最后一个引用（= 曲面 id）
surf = {}             # surf_id  -> {"t":..,"r":..,"r2":..,"a2p":..}
msb_shell = {}        # msb_id   -> [shell_ids]（第 1 个是外表面）
msb_order = []
a2p_refs = {}

log("PASS1 ...")
buf, nstmt = "", 0
with io.open(STEP, "r", encoding="latin-1", errors="replace") as f:
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
        nstmt += 1
        m = RE_ENT.match(stmt)
        if not m:
            continue
        eid, et, args = int(m.group(1)), m.group(2), m.group(3)

        if et in ("CLOSED_SHELL", "OPEN_SHELL"):
            shell_faces[eid] = refs(args)
        elif et == "ADVANCED_FACE":
            rr = refs(args)
            if rr:
                face_lastref[eid] = rr[-1]
        elif et == "CYLINDRICAL_SURFACE":
            rr = refs(args)
            ns = nums_noref(args)
            if rr:
                surf[eid] = {"t": "CYL", "a2p": rr[0], "r": ns[-1] if ns else None, "r2": None}
        elif et == "TOROIDAL_SURFACE":
            rr = refs(args)
            ns = nums_noref(args)
            if rr:
                surf[eid] = {"t": "TOR", "a2p": rr[0],
                             "r": ns[-2] if len(ns) >= 2 else None,
                             "r2": ns[-1] if ns else None}
        elif et in ("MANIFOLD_SOLID_BREP", "FACETED_BREP", "BREP_WITH_VOIDS"):
            msb_shell[eid] = refs(args)
            msb_order.append(eid)
        elif et == "AXIS2_PLACEMENT_3D":
            a2p_refs[eid] = refs(args)

log("PASS1 done stmts=%d shells=%d faces=%d cylsurf=%d msb=%d a2p=%d"
    % (nstmt, len(shell_faces), len(face_lastref), len(surf), len(msb_shell), len(a2p_refs)))

# ---- 解析 face -> surf（此时 surf 已全量就绪）
face_surf = {}
for f, sid in face_lastref.items():
    if sid in surf:
        face_surf[f] = sid
log("faces on cyl/tor surfaces: %d / %d" % (len(face_surf), len(face_lastref)))

# ---- 只解析用到的 a2p
want = set()
for s in surf.values():
    r = a2p_refs.get(s["a2p"])
    if r:
        want.update(r)
log("need %d point/dir ids" % len(want))

points, dirs = {}, {}
log("PASS2 ...")
with io.open(STEP, "r", encoding="latin-1", errors="replace") as f:
    for line in f:
        if line[:1] != "#":
            continue
        s = line.rstrip("\r\n")
        m = re.match(r"^#(\d+)\s*=\s*(CARTESIAN_POINT|DIRECTION)\s*\(", s)
        if not m:
            continue
        eid = int(m.group(1))
        if eid not in want:
            continue
        v = nums_in_last_paren(s)
        if len(v) >= 3:
            (points if m.group(2) == "CARTESIAN_POINT" else dirs)[eid] = v[-3:]
log("PASS2 done points=%d dirs=%d" % (len(points), len(dirs)))

def axis_of(a2p_id):
    r = a2p_refs.get(a2p_id)
    if not r or r[0] not in points:
        return None
    o = np.asarray(points[r[0]], float)
    d = np.asarray(dirs.get(r[1], [0.0, 0.0, 1.0]) if len(r) > 1 else [0.0, 0.0, 1.0], float)
    nd = np.linalg.norm(d)
    if nd < 1e-12:
        return None
    return o, d / nd

out = {}
for msb, shells in msb_shell.items():
    lst = []
    for sh in shells:
        for f in shell_faces.get(sh, []):
            sid = face_surf.get(f)
            if sid is None:
                continue
            s = surf[sid]
            ax = axis_of(s["a2p"])
            if ax is None:
                continue
            o, d = ax
            rec = {"t": s["t"], "r": s["r"], "o": [float(x) for x in o], "d": [float(x) for x in d]}
            if s["t"] == "TOR":
                rec["r2"] = s["r2"]
            lst.append(rec)
    if lst:
        out[str(msb)] = lst

with io.open(OUT, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False)
log("WROTE %s  msb_with_cylinder=%d / %d  (%.1f KB)"
    % (OUT, len(out), len(msb_shell), os.path.getsize(OUT) / 1024.0))
log("ALL DONE %.1fs" % (time.time() - T0))
_lf.close()
