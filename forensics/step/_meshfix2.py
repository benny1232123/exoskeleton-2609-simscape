# -*- coding: utf-8 -*-
"""在「只含坏体积」的 BREP 上快速试各种**几何修复**策略。

`Impossible to mesh periodic surface 19349` 在算法分派前就抛出，换算法无效。
所以只能改几何。候选策略：
  S1  只 heal（由 _meshfix.py 扫容差负责）
  S2  把那张坏面**非递归**删掉（recursive=False）—— 体积保留，只留一个针眼大的洞
  S3  把坏面**递归**删掉（连带只被它使用的边/点）
  S4  把整颗 volume 丢掉
  S5  用 plane/box 把该 volume 打碎（occ.fragment），让坏面被重新参数化
  S6  heal(FixSmallFaces) 之后再删面

对每种策略记录：网格化是否成功、剩余体积数、三角面数（判断几何是否还在）。
"""
from __future__ import annotations

import io
import os
import re
import sys
import time

import gmsh

ROOT = r"C:\Users\29408\Desktop\外骨骼"
GROUP = sys.argv[1] if len(sys.argv) > 1 else "back"
BREP = os.path.join(ROOT, "_badvol_%s.brep" % GROUP)
OUT = os.path.join(ROOT, "_meshfix2_%s.txt" % GROUP)

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

L = []


def say(s):
    L.append(s)
    print(s)
    io.open(OUT, "w", encoding="utf-8").write("\n".join(L) + "\n")


def bad_surface(exc):
    m = re.search(r"surface (\d+)", str(exc))
    return int(m.group(1)) if m else None


def tri_count():
    n = 0
    for (d, t) in gmsh.model.getEntities(2):
        et, _, en = gmsh.model.mesh.getElements(2, t)
        for a, b in zip(et, en):
            if a == 2:
                n += len(b) // 3
    return n


def fresh(heal=False, tol=1e-8):
    gmsh.initialize()
    for k, v in BASE.items():
        gmsh.option.setNumber(k, v)
    if heal:
        for k in HEAL_ALL:
            gmsh.option.setNumber(k, 1)
    gmsh.option.setNumber("Geometry.Tolerance", tol)
    gmsh.open(BREP)
    gmsh.model.occ.synchronize()


def attempt(label, prepare, heal=False, tol=1e-8):
    """prepare(): 在同步后做几何手术，返回描述。"""
    t0 = time.time()
    try:
        fresh(heal=heal, tol=tol)
        note = prepare()
        try:
            gmsh.model.mesh.generate(2)
            say("  %-34s OK   vols=%d surfs=%d tris=%d  (%.1f s)%s"
                % (label, len(gmsh.model.getEntities(3)),
                   len(gmsh.model.getEntities(2)), tri_count(), time.time() - t0,
                   ("  [%s]" % note) if note else ""))
            return True
        except Exception as e:
            say("  %-34s FAIL %s" % (label, str(e)[:70]))
            return False
    except Exception as e:
        say("  %-34s SETUP-FAIL %s" % (label, str(e)[:70]))
        return False
    finally:
        try:
            gmsh.finalize()
        except Exception:
            pass


def find_bad():
    """返回当前模型里第一个 mesh 失败的 surface tag（先试一次全量网格化）。"""
    try:
        gmsh.model.mesh.generate(2)
        return None
    except Exception as e:
        return bad_surface(e)


def main():
    if not os.path.exists(BREP):
        raise SystemExit("缺 %s —— 先跑 _meshfix.py" % BREP)
    say("== meshfix2 strategies on %s ==" % BREP)
    say("size %.3f MB" % (os.path.getsize(BREP) / 1048576))

    # 先确认基线确实失败，并拿到坏面 tag
    fresh(heal=False)
    bs = find_bad()
    say("baseline bad surface = %s" % bs)
    try:
        up = gmsh.model.getAdjacencies(2, bs)[0]
        say("  parents = %s" % list(up))
    except Exception as e:
        up = []
        say("  adjacency failed: %s" % e)
    gmsh.finalize()

    def s2():
        gmsh.model.occ.remove([(2, bs)], recursive=False)
        gmsh.model.occ.synchronize()
        return "remove face non-recursive"

    def s2r():
        gmsh.model.occ.remove([(2, bs)], recursive=True)
        gmsh.model.occ.synchronize()
        return "remove face recursive"

    def s4():
        gmsh.model.occ.remove([(3, int(v)) for v in up], recursive=True)
        gmsh.model.occ.synchronize()
        return "drop volume(s) %s" % list(up)

    def s5():
        bb = gmsh.model.occ.getBoundingBox(2, bs)
        cx, cy, cz = [(bb[i] + bb[i + 3]) / 2 for i in range(3)]
        d = max(bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2])
        box = gmsh.model.occ.addBox(cx - d, cy - d, cz - d, 2 * d, 2 * d, 2 * d)
        gmsh.model.occ.fragment([(3, int(v)) for v in up], [(3, box)])
        gmsh.model.occ.synchronize()
        gmsh.model.occ.remove([(3, box)], recursive=True)
        gmsh.model.occ.synchronize()
        return "fragment with box d=%.3f mm" % d

    results = {}
    results["S2 remove-face(rec=F)"] = attempt("S2 remove-face(rec=F)", s2)
    results["S3 remove-face(rec=T)"] = attempt("S3 remove-face(rec=T)", s2r)
    results["S6 heal+S2"] = attempt("S6 heal(tol=1e-2)+remove-face", s2, heal=True, tol=1e-2)
    results["S5 fragment-box"] = attempt("S5 fragment-box", s5)
    results["S4 drop-volume"] = attempt("S4 drop-volume", s4)

    say("")
    say("-- 汇总 --")
    for k, v in results.items():
        say("  %-24s %s" % (k, "OK" if v else "fail"))
    win = [k for k, v in results.items() if v]
    say("VERDICT: %s  %s" % ("STRATEGY_FOUND" if win else "ALL_FAILED", win))


if __name__ == "__main__":
    main()
