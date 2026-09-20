# -*- coding: utf-8 -*-
"""
STEP 质量属性探针
1) 自检：对单位立方体验证 getMass / getMatrixOfInertia 的语义与参考点
2) 真跑：导入 136MB 外骨骼 STEP，输出每个实体的 volume / COM / inertia / bbox
"""
import sys, time, json, math
import gmsh

OUT = r"C:\Users\29408\Desktop\外骨骼\_step_probe.txt"
STEP = r"C:\Users\29408\Desktop\外骨骼\“林-Ⅰ”髋关节外骨骼机器人开发平台V0_1_1.stp"

log = []


def p(s):
    log.append(str(s))
    print(s, flush=True)


# ---------- 1) 自检 ----------
gmsh.initialize()
gmsh.option.setNumber("General.Terminal", 0)
gmsh.model.add("selftest")
box = gmsh.model.occ.addBox(0, 0, 0, 1.0, 1.0, 1.0)
gmsh.model.occ.synchronize()
m = gmsh.model.occ.getMass(3, box)
I = gmsh.model.occ.getMatrixOfInertia(3, box)
p("=== SELFTEST unit cube at origin, corner (0,0,0) ===")
p("getMass          -> %s   (expect mass=1, cog=(0.5,0.5,0.5))" % m)
p("getMatrixOfInertia -> %s" % I)
p("  expect about COG: diag(1/6,1/6,1/6)=0.166667 off-diag=0")
p("  expect about ORIGIN: Ixx=2/3=0.666667, Ixy=-1/4=-0.25")
p("")

# ---------- 2) 真跑 ----------
p("=== IMPORT STEP (may take minutes) ===")
t0 = time.time()
gmsh.model.add("exo")
try:
    gmsh.model.occ.importShapes(STEP)
except Exception as e:
    p("importShapes raised: %r" % e)
gmsh.model.occ.synchronize()
p("import+synchronize took %.1f s" % (time.time() - t0))

gmsh.model.occ.removeAllDuplicates()
gmsh.model.occ.synchronize()

ents = {}
for dim in (0, 1, 2, 3):
    ents[dim] = gmsh.model.getEntities(dim)
    p("entities dim=%d count=%d" % (dim, len(ents[dim])))

# 名称是否保留？
named = 0
for d in (2, 3):
    for (dim, tag) in ents[d][:50]:
        try:
            nm = gmsh.model.getEntityName(dim, tag)
        except Exception:
            nm = ""
        if nm:
            named += 1
p("entities with non-empty name (sampled): %d" % named)


def massprops(dim, tag):
    """返回 dict；若失败返回 None"""
    try:
        m = gmsh.model.occ.getMass(dim, tag)
    except Exception as e:
        return {"err": "getMass:%r" % e}
    d = {"volume": m[0], "cog": [m[1], m[2], m[3]]}
    try:
        d["I"] = list(gmsh.model.occ.getMatrixOfInertia(dim, tag))
    except Exception as e:
        d["I_err"] = repr(e)
    try:
        bb = gmsh.model.occ.getBoundingBox(dim, tag)
        d["bbox"] = list(bb)
        d["size"] = [bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]]
    except Exception as e:
        d["bb_err"] = repr(e)
    try:
        d["name"] = gmsh.model.getEntityName(dim, tag)
    except Exception:
        d["name"] = ""
    return d


vols = ents[3]
p("")
p("=== VOLUME PROPS: %d solids ===" % len(vols))
t0 = time.time()
res = []
tot_v = 0.0
for i, (dim, tag) in enumerate(vols):
    d = massprops(dim, tag)
    d["tag"] = tag
    res.append(d)
    if "volume" in d:
        tot_v += d["volume"]
    if i % 50 == 0:
        p("  ... %d/%d  (%.1fs)" % (i, len(vols), time.time() - t0))
p("volume props took %.1f s" % (time.time() - t0))
p("TOTAL VOLUME = %.6e  (mm^3 presumed)" % tot_v)

# 写明细
lines = []
lines.append("=== SELFTEST ===")
lines.append("mass=%s" % (m,))
lines.append("I=%s" % (I,))
lines.append("")
lines.append("=== SOLIDS: %d  total_volume=%.6e ===" % (len(vols), tot_v))
lines.append("tag\tvolume\tcog_x\tcog_y\tcog_z\tbb_x0\tbb_y0\tbb_z0\tbb_x1\tbb_y1\tbb_z1\tname")
for d in res:
    if "volume" not in d:
        lines.append("%s\tERR %s" % (d.get("tag"), d.get("err")))
        continue
    bb = d.get("bbox", [0] * 6)
    lines.append("%d\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%s" % (
        d["tag"], d["volume"], d["cog"][0], d["cog"][1], d["cog"][2],
        bb[0], bb[1], bb[2], bb[3], bb[4], bb[5], d.get("name", "")))

# 全局 bbox
try:
    gb = gmsh.model.getBoundingBox(-1, -1)
    lines.append("")
    lines.append("GLOBAL BBOX = %s" % (gb,))
    lines.append("GLOBAL SIZE = %s" % ([gb[3] - gb[0], gb[4] - gb[1], gb[5] - gb[2]],))
except Exception as e:
    lines.append("global bbox err %r" % e)

# 完整 inertia 到单独 JSON
with open(r"C:\Users\29408\Desktop\外骨骼\_step_inertia.json", "w", encoding="utf-8") as f:
    json.dump(res, f, ensure_ascii=False, indent=1)

open(OUT, "w", encoding="utf-8").write("\n".join(lines))
open(r"C:\Users\29408\Desktop\外骨骼\_step_probe.log.txt", "w", encoding="utf-8").write("\n".join(log))
p("ALL DONE")
