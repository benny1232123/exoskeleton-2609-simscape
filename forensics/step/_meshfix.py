# -*- coding: utf-8 -*-
"""定向修复实验：只拿那一颗含坏面的 volume，快速扫「heal 选项 × 容差 × 算法」。

背景
----
`back.stp` 导入后有 29916 个面，其中 surface 19349（面积 0.0368 mm²，
包围盒约 0.22×0.20×0.19 mm）被 gmsh 判为 periodic surface，
`meshGFace` 在**算法分派之前**就报 "Impossible to mesh periodic surface"，
所以换算法（MeshAdapt / Delaunay / Frontal …）全部同样失败 —— 已实测 7 个算法。
=> 只能从几何上修：要么把它塌掉（heal + 粗容差），要么把整颗体积丢掉。

做法
----
导入一次 → 只留这颗 volume → 写成 BREP（后续反复读 BREP，秒级，不必再解析 58MB STEP）
→ 扫参数组合。BREP 载入 ~1 s，可扫很多组合。
"""
from __future__ import annotations

import io
import os
import re
import sys
import time

import gmsh

ROOT = r"C:\Users\29408\Desktop\外骨骼"
RED = os.path.join(ROOT, "_reduced3")

GROUP = sys.argv[1] if len(sys.argv) > 1 else "back"
STEP = os.path.join(RED, "%s.stp" % GROUP)
BREP = os.path.join(ROOT, "_badvol_%s.brep" % GROUP)
OUT = os.path.join(ROOT, "_meshfix_%s.txt" % GROUP)

L = []


def say(s):
    L.append(s)
    print(s)
    io.open(OUT, "w", encoding="utf-8").write("\n".join(L) + "\n")


BASE = {
    "General.Terminal": 0,
    "Geometry.OCCParallel": 1,
    "Mesh.MeshSizeMin": 1.8,
    "Mesh.MeshSizeMax": 5.0,
    "Mesh.CharacteristicLengthFromCurvature": 0,
    "Mesh.CharacteristicLengthFromPoints": 0,
    "Mesh.CharacteristicLengthExtendFromBoundary": 1,
}
HEAL_ALL = ["Geometry.OCCFixDegenerated", "Geometry.OCCFixSmallEdges",
            "Geometry.OCCFixSmallFaces", "Geometry.OCCSewFaces"]


def apply(opts):
    for k, v in BASE.items():
        gmsh.option.setNumber(k, v)
    for k, v in opts.items():
        gmsh.option.setNumber(k, v)


def first_bad_surface(exc: str):
    m = re.search(r"surface (\d+)", exc)
    return int(m.group(1)) if m else None


def main():
    say("== meshfix probe: %s ==" % GROUP)

    # ---------- 1. 导入一次，摸清坏面与它的父体积 ----------
    t0 = time.time()
    gmsh.initialize()
    apply({})
    gmsh.open(STEP)
    gmsh.model.occ.synchronize()
    say("import+sync: %.1f s ; volumes=%d surfaces=%d"
        % (time.time() - t0, len(gmsh.model.getEntities(3)),
           len(gmsh.model.getEntities(2))))

    bad = None
    badvols = []
    try:
        gmsh.model.mesh.generate(2)
        say("!! 意外成功，无需修复")
    except Exception as e:
        bad = first_bad_surface(str(e))
        say("first failure: %s" % str(e)[:120])

    if bad is not None:
        try:
            say("  surface %d type=%s" % (bad, gmsh.model.getType(2, bad)))
        except Exception as e:
            say("  getType failed: %s" % e)
        try:
            lo_hi = gmsh.model.getParametrizationBounds(2, bad)
            say("  param bounds: %s" % [list(x) for x in lo_hi])
        except Exception as e:
            say("  param bounds failed: %s" % e)
        try:
            pl = gmsh.model.getPeriodicLinks(2, bad)
            say("  periodicLinks: %s" % pl)
        except Exception as e:
            say("  periodicLinks failed: %s" % e)
        try:
            up = gmsh.model.getAdjacencies(2, bad)[0]
            say("  parent volumes: %s" % list(up))
            for vt in up:
                vt = int(vt)
                va = gmsh.model.occ.getMass(3, vt)
                bb = gmsh.model.occ.getBoundingBox(3, vt)
                nm = ""
                try:
                    nm = gmsh.model.getEntityName(3, vt)
                except Exception:
                    pass
                diag = ((bb[3] - bb[0]) ** 2 + (bb[4] - bb[1]) ** 2 + (bb[5] - bb[2]) ** 2) ** 0.5
                say("    vol %d vol=%.9f mm^3 diag=%.3f mm name='%s'"
                    % (vt, va, diag, nm))
                badvols.append(vt)
        except Exception as e:
            say("  adjacency failed: %s" % e)

    # ---------- 2. 只留坏体积，写 BREP ----------
    allv = [(3, t) for (d, t) in gmsh.model.getEntities(3)]
    keep = set(badvols)
    drop = [v for v in allv if v[1] not in keep]
    if drop:
        gmsh.model.occ.remove(drop, recursive=True)
        gmsh.model.occ.synchronize()
    say("kept %d / %d volumes ; writing %s"
        % (len(gmsh.model.getEntities(3)), len(allv), BREP))
    gmsh.write(BREP)
    try:
        gmsh.finalize()
    except Exception:
        pass

    # ---------- 3. 扫参数组合（读 BREP，快）----------
    combos = []
    for algo, an in [(6, "FrontalDelaunay"), (1, "MeshAdapt"), (5, "Delaunay")]:
        combos.append(({}, 1e-8, algo, "noheal/tol=1e-8/" + an))
        for tol in [1e-3, 1e-2, 5e-2, 1e-1, 3e-1]:
            o = {k: 1 for k in HEAL_ALL}
            combos.append((o, tol, algo, "healall/tol=%-6g/%s" % (tol, an)))
    # 只开 FixSmallFaces
    for tol in [1e-2, 1e-1, 3e-1]:
        o = {"Geometry.OCCFixSmallFaces": 1}
        combos.append((o, tol, 6, "fixsmallfaces/tol=%-6g/FrontalDelaunay" % tol))

    wins = []
    for opts, tol, algo, label in combos:
        try:
            gmsh.initialize()
            apply(opts)
            gmsh.option.setNumber("Geometry.Tolerance", tol)
            gmsh.option.setNumber("Mesh.Algorithm", algo)
            gmsh.open(BREP)
            gmsh.model.occ.synchronize()
            nv = len(gmsh.model.getEntities(3))
            ns = len(gmsh.model.getEntities(2))
            t = time.time()
            gmsh.model.mesh.generate(2)
            say("  %-40s OK   vols=%d surfs=%d  (%.1f s)" % (label, nv, ns, time.time() - t))
            wins.append(label)
        except Exception as e:
            say("  %-40s FAIL %s" % (label, str(e)[:70]))
        finally:
            try:
                gmsh.finalize()
            except Exception:
                pass

    say("")
    say("VERDICT: %s  %s" % ("FIX_FOUND" if wins else "NO_SIMPLE_FIX", wins))


if __name__ == "__main__":
    main()
