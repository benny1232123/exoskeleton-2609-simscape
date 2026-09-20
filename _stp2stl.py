# -*- coding: utf-8 -*-
"""
从「分组裁剪后的 STEP」导出 URDF 可视化网格（STL），让 Simscape Multibody
的 Mechanics Explorer 显示**真实外形**，而不是 smimport 给的占位体。

为什么需要这一步
----------------
URDF 的 `<inertial>` 只描述动力学，**不含任何几何**。所以哪怕
`exo_real.urdf` 的惯量/质心逐位正确，Mechanics Explorer 里也只能看到
一坨占位方块 —— 能验证动力学，但没法给人看"这就是那个外骨骼"。

把 STEP 里的面网格出来、按刚体分组、变换到世界系、写成 STL，
再在 URDF 里加 `<visual><geometry><mesh .../></geometry></visual>` 即可。

坐标系统一（复用 _stp2urdf.py 的设计决定）
-------------------------------------------
`exo_real.urdf` 里关节 `origin rpy="0 0 0"` => **child link frame ≡ 世界系**。
因此 STL 顶点直接写**世界系坐标（m）**，`<visual>` 的 origin 取 0 即可。
全链只有世界系一个坐标系，不需要再引入任何旋转约定。

几何来源与实例变换的拼接
------------------------
`_reduced3/<group>.stp` 是按零件定义**去重**后的裁剪文件：
  每个 `pd`（零件定义）只留一份几何，坐标在**零件局部系**（mm）。
装配摆放信息在 `_props_instances.json` 的 `placed[i].T`（局部 -> 世界）。
所以「一个 pd 有几个实例，就复制几份网格，各自套自己的 T」。

`tag -> msb` 的对应来自 `_mass_<group>.json`（tag 是 gmsh 体积标号），
用 体积 + 局部质心 双判据逐位比对反查，再由 `_map_solids` 的 msb->pd 得到 pd。

== 关于 periodic surface（本组最麻烦的坑）==
`back` / `motor_R` 这两组的 OCC 翻译体里有若干**参数域反转**的面：
    surface 19349  type=Cone
      u bounds = [-4.148e-05, -1.414e-01]     <- u_lo > u_hi
      v bounds = [ 8.667e-01,  1.414e-01]     <- v_lo > v_hi
gmsh 拿不到合法 UV 参数化，于是在 `meshGFace` 里**算法分派之前**就报
    Impossible to mesh periodic surface 19349
=> 换算法完全无用（实测 7 个 2D 算法 MeshAdapt/Delaunay/Frontal/… 全部同样失败）。
只能从几何上修。本脚本的策略：
  1) 主动扫描全部面，把 **u 或 v 上下界反转**的面挑出来（比"失败再重试"便宜得多：
     扫 3 万个面 ~数十秒，而每次失败的网格尝试要先把前两万多个面网格化，好几分钟）；
  2) 把这些面**非递归**删除（`recursive=False`）：体积保留，只留下一个针眼大的洞
     （实测该面面积 0.0368 mm²，0.2 mm 尺度，肉眼看不出）；
  3) 然后再网格化；若仍有漏网的面，从报错里解析面号，逐个删，直到成功。
本步骤**只影响可视化**：`<inertial>` 来自 `_mass_*.json` 那条独立链路，
删面/丢面都不可能漏进动力学。

输出
----
    matlab2609/simscape/meshes/base.stl
    matlab2609/simscape/meshes/leg_L.stl
    matlab2609/simscape/meshes/leg_R.stl
    matlab2609/simscape/stl_export_report.txt
"""
from __future__ import annotations

import argparse
import io
import json
import os
import re
import sys
import time

import numpy as np

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)

from exo2609.geometry import CadGeometry          # noqa: E402

RED_DIR = os.path.join(ROOT, "_reduced3")
SIM_DIR = os.path.join(ROOT, "matlab2609", "simscape")
MESH_DIR = os.path.join(SIM_DIR, "meshes")
OUT_RPT = os.path.join(SIM_DIR, "stl_export_report.txt")

GROUP_TO_LINK = {
    "motor_L": "base", "motor_R": "base", "back": "base",
    "leg_L": "leg_L", "leg_R": "leg_R",
}
# 网格尺寸（mm）。腿零件小、要看清结构，给 3.5；躯干/背部零件多且大，
# 给 6.0 把三角面数量压下来（这是**可视化**网格，过密只会拖慢 Mechanics Explorer）。
MESH_SIZE = {"leg_L": 3.5, "leg_R": 3.5, "motor_L": 6.0, "motor_R": 6.0, "back": 6.0}
LINK_ORDER = ["base", "leg_L", "leg_R"]
MAX_ROUNDS = 10            # 迭代修面的上限，防病态几何把脚本挂死

# 可选：导入后对**全模型** healShapes 一次。默认关闭（None），理由见 mesh_group 注释：
# 实测 260 体积 / 29916 面的全模型 heal 单次要几分钟，而且它会把判为非法的体积
# 直接删掉 —— 与「丢体积」等价却贵 20 s。
HEAL_TOL = None
# 局部 heal 的容差：1e-8 就足以让 OCC 判定该体积非法并清掉（实测）。
HEAL_TOL_LOCAL = 1e-8


# ---------------------------------------------------------------- 映射
def match_msb_to_tag(props, tag_geom, tol_vol=1e-9, tol_cog=1e-9):
    """把 `cad._map_solids()` 给出的 `msb -> SolidProps` 与**现场 gmsh 体积**
    按 (体积, 局部质心) 双判据对上，返回 `(msb -> tag, 未匹配 msb, 剩余 tag 数)`。

    为什么不用 `_mass_<group>.json` 里的 `tag` 字段：
    那条依赖 gmsh 的**体积编号**，而 `occ.healShapes()` / `occ.remove()` 都会让
    编号变化 —— 编号一变，老映射就整体错位（**静默错配零件**，惯量看着还挺正常）。
    按几何匹配对编号免疫，且双判据是逐位比对（体积相对 1e-9 + 质心绝对 1e-9 mm），
    同名零件也不会串。`_mass_*.json` 仍用于动力学侧（`_map_solids`）。
    """
    cand = {int(t): (float(v), np.asarray(c, float)) for t, (v, c) in tag_geom.items()}
    out, taken, unmatched = {}, set(), []
    for msb, sp in props.items():
        hit = None
        for tag, (vol, cog) in cand.items():
            if tag in taken:
                continue
            if abs(vol - sp.volume_mm3) <= tol_vol * max(abs(sp.volume_mm3), 1.0) and \
               float(np.max(np.abs(cog - sp.cog_local))) < tol_cog:
                hit = tag
                break
        if hit is None:
            unmatched.append(msb)
        else:
            taken.add(hit)
            out[msb] = hit
    return out, unmatched, len(cand) - len(taken)


def pd_of_msb(cad: CadGeometry, group: str):
    m = {}
    for inst in cad.placed:
        if cad.group_of(inst) != group:
            continue
        for msb in inst.solids:
            m.setdefault(msb, inst.pd)
    return m


def instances_of_pd(cad: CadGeometry, group: str):
    out = {}
    for inst in cad.placed:
        if cad.group_of(inst) != group:
            continue
        out.setdefault(inst.pd, []).append(inst)
    return out


# ---------------------------------------------------------------- gmsh
def _bad_surface(msg: str):
    m = re.search(r"surface (\d+)", msg)
    return int(m.group(1)) if m else None


def scan_inverted_faces(log):
    """挑出参数域上下界反转的面（u_lo>u_hi 或 v_lo>v_hi）——这些必定网格化失败。"""
    import gmsh
    bad = []
    surfs = [t for (d, t) in gmsh.model.getEntities(2)]
    for t in surfs:
        try:
            b = gmsh.model.getParametrizationBounds(2, t)
        except Exception:
            continue
        (u0, u1), (v0, v1) = b[0], b[1]
        if u0 > u1 or v0 > v1:
            bad.append(t)
    log("  参数域反转面扫描：%d 个面中命中 %d 个" % (len(surfs), len(bad)))
    return bad


def describe_faces(tags, maxshow=8):
    import gmsh
    out = []
    for t in tags[:maxshow]:
        try:
            area = gmsh.model.occ.getMass(2, t)
        except Exception:
            area = float("nan")
        try:
            ty = gmsh.model.getType(2, t)
        except Exception:
            ty = "?"
        try:
            par = list(gmsh.model.getAdjacencies(2, t)[0])
        except Exception:
            par = []
        out.append((t, ty, area, par))
    return out


def mesh_group(group: str, log, repair=True, scan=False, heal_tol=HEAL_TOL):
    """导入 -> 修坏面 -> 2D 网格化。

    返回 `(tag -> (n,3,3) 局部 mm 三角面, tag -> (体积 mm^3, 局部质心 mm), 统计 dict)`

    == 关于臭名昭著的 "Impossible to mesh periodic surface" ==
    `back` / `motor_R` 的 OCC 翻译体里有**参数域上下界反转**的面，例如
        surface 19349  type=Cone
          u = [-4.148e-05, -1.414e-01]     <- u_lo > u_hi
          v = [ 8.667e-01,  1.414e-01]     <- v_lo > v_hi
    gmsh 取不到合法 UV 参数化，于是在 `meshGFace` 里**算法分派之前**
    就报错 —— 换 2D 算法完全无用（实测 MeshAdapt/Delaunay/Frontal/… 7 个全一样）。
    只能改几何。三级手段，按序尝试：
      1) **heal**（真正的解法）：`occ.healShapes([], tolerance=T, …)`。
         成功时会**重编号实体**，所以调用方必须用 `match_msb_to_tag()` 按几何重匹。
      2) **删面**：`occ.remove([(2,s)], recursive=False)` —— **实测无效**
         （`synchronize()` 之后同一张面原样回来，连删 26 轮都还在），仅留档。
      3) **丢体积**：`occ.remove([(3,v)], recursive=True)` —— 兜底。
         会损失该零件的外形（本机案例：18.75 mm / 128 mm³ 的小件）。
    """
    import gmsh

    def vol_sum():
        s = 0.0
        for (_d, t) in gmsh.model.getEntities(3):
            try:
                s += gmsh.model.occ.getMass(3, t)
            except Exception:
                pass
        return s

    step = os.path.join(RED_DIR, "%s.stp" % group)
    size = MESH_SIZE[group]
    t0 = time.time()
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setNumber("Geometry.OCCParallel", 1)
    gmsh.open(step)
    gmsh.model.occ.synchronize()
    t_imp = time.time() - t0
    n_vol0, vol0 = len(gmsh.model.getEntities(3)), vol_sum()

    healed_tol, bad_faces, removed_faces, dropped_vols = None, [], [], []
    if repair and heal_tol is not None:
        # 全模型 heal。实测 260 体积 / 29916 面单次要几分钟，且 healShapes
        # 会把判为**非法**的体积直接删掉（本机案例：vol_sum 恰好 −128.086800 mm³
        # = 该体积自身体积）—— 与"丢体积"等价却贵 20 s，故默认关闭（HEAL_TOL=None）。
        try:
            gmsh.model.occ.healShapes([], tolerance=heal_tol,
                                      fixDegenerated=True, fixSmallEdges=True,
                                      fixSmallFaces=True, sewFaces=True,
                                      makeSolids=True)
            gmsh.model.occ.synchronize()
            healed_tol = heal_tol
        except Exception as e:
            log("  heal 失败: %s" % str(e)[:90])
    if repair and scan:
        bad_faces = scan_inverted_faces(log)
        for (t, ty, area, par) in describe_faces(bad_faces):
            log("    bad face %-8s type=%-6s area=%.6f mm^2 parents=%s"
                % (t, ty, area, par))
    n_vol1, vol1 = len(gmsh.model.getEntities(3)), vol_sum()
    log("  导入 %.1f s ; 全模型 heal=%s ; volumes %d->%d ; vol_sum %.3f->%.3f mm^3 (d=%+.5f)"
        % (t_imp, ("tol=%g" % healed_tol) if healed_tol else "off",
           n_vol0, n_vol1, vol0, vol1, vol1 - vol0))

    gmsh.option.setNumber("Mesh.MeshSizeMin", max(1.0, size * 0.35))
    gmsh.option.setNumber("Mesh.MeshSizeMax", size)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromCurvature", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromPoints", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthExtendFromBoundary", 1)
    gmsh.option.setNumber("Mesh.Algorithm", 6)

    def try_mesh():
        t = time.time()
        try:
            gmsh.model.mesh.generate(2)
            return True, "", time.time() - t
        except Exception as e:
            return False, str(e), time.time() - t

    # 修复级联。手段按「便宜且确定」排序：
    #   A 有父体积 -> 丢体积（recursive remove，~1 s，确定生效）
    #   B 无父体积（自由面）-> 删这张面（非递归，~1 s）
    #   C 同一张面反复出现 3 次 -> 放弃修复，**收割已网格化的部分**
    # 关键事实：`generate(2)` 在第一张坏面处抛异常，但**此前已网格化的面其网格仍在**，
    # 所以"修不动"不等于"什么都拿不到" —— 照样能把大部分体积的三角面捞出来，
    # 只是覆盖度有缺口，报告里如实写清。
    ok, rounds, last = False, 0, ""
    seen = {}
    while rounds < MAX_ROUNDS:
        rounds += 1
        ok, last, dt = try_mesh()
        if ok:
            log("  网格化成功（第 %d 轮，%.1f s）" % (rounds, dt))
            break
        s = _bad_surface(last)
        log("  网格化失败(第 %d 轮，%.1f s): %s" % (rounds, dt, last[:90]))
        if s is None or not repair:
            break
        seen[s] = seen.get(s, 0) + 1
        if seen[s] >= 3:
            log("    -> 同一张面 %s 反复出现 %d 次，修复无效；停止修复，收割已网格化部分"
                % (s, seen[s]))
            break
        try:
            par = [int(v) for v in gmsh.model.getAdjacencies(2, s)[0]]
        except Exception:
            par = []
        if hasattr(gmsh.model.mesh, "clear"):
            try:
                gmsh.model.mesh.clear()
            except Exception:
                pass

        if par:
            # 用**局部 healShapes** 清掉这些非法体积，而不是 `occ.remove`。
            # 原因（本机实测，血泪）：
            #   * healShapes(vols, tol) 是局部操作，20 s 出结果；实测它会把判为非法的
            #     体积整个删掉（vol_sum 恰好 −128.086800 mm³ = 该体积自身体积），
            #     等价于"丢体积"，但**不触发全模型 re-bind**。
            #   * occ.remove(...) + synchronize() 在 30k 面的模型上会重绑所有实体，
            #     实测十几分钟都回不来，看起来就是"进程假死"（踩过一次）。
            try:
                gmsh.model.occ.healShapes([(3, v) for v in par], tolerance=HEAL_TOL_LOCAL,
                                          fixDegenerated=True, fixSmallEdges=True,
                                          fixSmallFaces=True, sewFaces=True,
                                          makeSolids=True)
                gmsh.model.occ.synchronize()
                dropped_vols.extend(par)
                log("    -> 局部 heal 清掉非法体积 %s（其外形缺失），重试" % par)
                continue
            except Exception as e3:
                log("    -> 局部 heal 失败: %s" % str(e3)[:60])
        try:
            gmsh.model.occ.remove([(2, s)], recursive=False)
            gmsh.model.occ.synchronize()
            removed_faces.append(s)
            log("    -> 删自由面 %s" % s)
            continue
        except Exception as e2:
            log("    -> 删面失败: %s" % str(e2)[:60])
            break

    if not ok:
        log("  ⚠ 网格化未完全成功（%d 轮），改为收割已网格化的部分" % rounds)

    # ---- 取三角面 ----
    vol_tags = [t for (d, t) in gmsh.model.getEntities(3)]
    # 现场几何指纹（体积 + 局部质心）。heal/remove 会重编号 tag，所以后续一律用
    # 这组指纹去和 `_map_solids` 的 SolidProps 对，而不是沿用 `_mass_*.json` 的 tag。
    tag_geom = {}
    for vt in vol_tags:
        try:
            tag_geom[int(vt)] = (
                float(gmsh.model.occ.getMass(3, vt)),
                np.asarray(gmsh.model.occ.getCenterOfMass(3, vt), float))
        except Exception as e:
            log("    volume %s 几何指纹取不到: %s" % (vt, str(e)[:60]))
    ntag, ncoord, _ = gmsh.model.mesh.getNodes()
    X = np.asarray(ncoord, float).reshape(-1, 3)
    idx = {int(t): i for i, t in enumerate(np.asarray(ntag, int))}

    out, n_fail = {}, 0
    n_surf_seen, n_surf_meshed = set(), set()
    for vt in vol_tags:
        surf = [t for (d, t) in gmsh.model.getBoundary([(3, vt)], oriented=False,
                                                       recursive=False) if d == 2]
        blocks = []
        for s in surf:
            try:
                etypes, etags, enodes = gmsh.model.mesh.getElements(2, s)
            except Exception:
                continue
            got = False
            for et, nd in zip(etypes, enodes):
                if et != 2:
                    continue
                arr = np.asarray(nd, int).reshape(-1, 3)
                if len(arr):
                    got = True
                blocks.append(X[[idx[int(k)] for k in arr.ravel()]].reshape(-1, 3, 3))
            n_surf_seen.add(int(s))
            if got:
                n_surf_meshed.add(int(s))
        if not blocks:
            n_fail += 1
            out[int(vt)] = np.zeros((0, 3, 3))
        else:
            out[int(vt)] = np.concatenate(blocks, axis=0)

    surf_cov = (len(n_surf_meshed) / len(n_surf_seen)) if n_surf_seen else 1.0
    # 判定：所有**保留下来**的体积都拿到三角面 且 面覆盖率 ≈100% 才算完整；
    # 否则如实报 PARTIAL（宁可说清楚缺口，不要让人以为网格是全的）。
    ok_final = bool(ok) or (n_fail == 0 and surf_cov > 0.999)
    stats = dict(n_vol=len(vol_tags), n_bad_scanned=len(bad_faces),
                 n_face_removed=len(removed_faces), n_vol_dropped=len(dropped_vols),
                 dropped_vols=sorted(set(dropped_vols)), ok=ok_final, mesh_ok=ok,
                 rounds=rounds, n_vol_nomesh=n_fail, last_err=last,
                 healed_tol=healed_tol, vol_sum=vol1, vol_sum0=vol0, n_vol0=n_vol0,
                 n_fingerprint=len(tag_geom), surf_cov=surf_cov,
                 n_surf_seen=len(n_surf_seen), n_surf_meshed=len(n_surf_meshed))
    gmsh.finalize()
    return out, tag_geom, stats


# ---------------------------------------------------------------- STL
def write_binary_stl(path: str, tris: np.ndarray, name: str = "exo"):
    """写二进制 STL。tris: (n,3,3) float64，单位 m，世界系。"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    n = len(tris)
    rec = np.zeros(n, dtype=[("nrm", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    nrm = np.cross(v1 - v0, v2 - v0)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, np.where(ln > 0, ln, 1.0))
    rec["nrm"] = nrm.astype("<f4")
    rec["v"] = tris.astype("<f4")
    with open(path, "wb") as f:
        f.write(name.encode("ascii", "replace")[:79].ljust(80, b"\0"))
        f.write(np.array([n], dtype="<u4").tobytes())
        f.write(rec.tobytes())
    return n


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", action="store_true", help="只做映射统计，不网格化")
    ap.add_argument("--groups", default="", help="逗号分隔，只处理这些分组")
    ap.add_argument("--no-repair", action="store_true", help="不修坏面（A/B 对照用）")
    ap.add_argument("--scan", action="store_true",
                    help="先主动扫描参数域反转面（很慢：3 万个面 >10 min）")
    args = ap.parse_args()
    only = set(x.strip() for x in args.groups.split(",") if x.strip())

    cad = CadGeometry(ROOT)
    t0 = time.time()
    lines = []

    def log(s):
        lines.append(s)
        print(s)

    log("== STEP -> STL 可视化网格导出报告 ==")
    log("link frame ≡ 世界系（关节 origin rpy = 0 0 0），STL 顶点直接写世界系坐标（m）")
    log("来源: %s" % RED_DIR)
    log("修坏面: %s" % ("关闭（--no-repair）" if args.no_repair else "开启"))
    log("")

    link_tris = {k: [] for k in LINK_ORDER}
    grand, all_ok = 0, True
    grp_stats = {}

    for group in sorted(GROUP_TO_LINK):
        if only and group not in only:
            continue
        link = GROUP_TO_LINK[group]
        props, warns = cad._map_solids(group)
        msb2pd = pd_of_msb(cad, group)
        pd2inst = instances_of_pd(cad, group)
        n_solids = sum(len(v) for v in pd2inst.values())

        log("-- %s -> link '%s' --" % (group, link))
        log("  _map_solids 映射到 msb = %d ; 零件定义 pd = %d ; 装配实例槽位 = %d"
            % (len(props), len(pd2inst), n_solids))
        if warns:
            for w in warns[:3]:
                log("  ! %s" % w)

        if args.probe:
            log("")
            continue

        tri_local, tag_geom, st = mesh_group(group, log, repair=not args.no_repair,
                                             scan=args.scan)
        grp_stats[group] = st
        all_ok = all_ok and st["ok"]

        # 现场几何指纹 -> msb。heal/remove 会重编号体积 tag，
        # 所以这里按 (体积, 局部质心) 逐位重匹，**不沿用** _mass_*.json 的 tag。
        msb2tag, unmatched_msb, n_tag_left = match_msb_to_tag(props, tag_geom)
        log("  几何匹配: 现场体积=%d ; msb->tag 命中=%d ; 未匹配 msb=%d ; 未用 tag=%d"
            % (st["n_fingerprint"], len(msb2tag), len(unmatched_msb), n_tag_left))
        if unmatched_msb:
            log("  ⚠ 未匹配 msb（前 8）= %s" % unmatched_msb[:8])
        if st["healed_tol"] is not None:
            log("  OCC heal: tol=%g ; volumes %d->%d ; vol_sum %.3f->%.3f mm^3 (d=%+.5f)"
                % (st["healed_tol"], st["n_vol0"], st["n_vol"],
                   st["vol_sum0"], st["vol_sum"], st["vol_sum"] - st["vol_sum0"]))
        n_tri = sum(len(v) for v in tri_local.values())
        log("  网格化: volumes=%d  有三角面=%d  三角面总数=%d  轮次=%d"
            % (st["n_vol"], st["n_vol"] - st["n_vol_nomesh"], n_tri, st["rounds"]))
        log("  面覆盖率: %d/%d = %.4f%%%s"
            % (st["n_surf_meshed"], st["n_surf_seen"], 100 * st["surf_cov"],
               "" if st["mesh_ok"] else "   (generate 未跑完全程)"))
        if st["n_vol_dropped"]:
            log("  ⚠ 丢弃的体积 = %s（对应零件外形缺失）" % st["dropped_vols"])

        n_copy, n_skip = 0, 0
        for msb, tag in msb2tag.items():
            pd = msb2pd.get(msb)
            tri = tri_local.get(tag)
            if pd is None or tri is None or len(tri) == 0:
                n_skip += 1
                continue
            for inst in pd2inst.get(pd, []):
                T = np.asarray(inst.T, float)
                w = tri.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]
                link_tris[link].append((w * 1e-3).reshape(-1, 3, 3))
                n_copy += 1
        log("  写入实例副本 = %d（%d 个 msb 因无网格/无 pd 被跳过）" % (n_copy, n_skip))
        log("")

    if args.probe:
        log("PROBE 模式：未生成 STL。PROBE_OK")
        io.open(OUT_RPT, "w", encoding="utf-8").write("\n".join(lines) + "\n")
        return

    log("-- 写出 STL --")
    for link in LINK_ORDER:
        if not link_tris[link]:
            log("  %-6s 无三角面，跳过" % link)
            continue
        tris = np.concatenate(link_tris[link], axis=0)
        p = os.path.join(MESH_DIR, "%s.stl" % link)
        write_binary_stl(p, tris, "exo_%s" % link)
        grand += len(tris)
        lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
        log("  %-6s %7d 三角面  %6.2f MB  bbox=[%s] .. [%s] m"
            % (link, len(tris), os.path.getsize(p) / 1048576.0,
               " ".join("%.4f" % v for v in lo), " ".join("%.4f" % v for v in hi)))
    log("  合计 %d 三角面" % grand)
    log("")
    log("-- 坏面修复汇总 --")
    for g, st in sorted(grp_stats.items()):
        log("  %-8s 网格=%s 轮次=%2d 删面=%2d 丢体积=%2d 无网格体积=%d 面覆盖=%.3f%%"
            % (g, "OK" if st["ok"] else "PARTIAL", st["rounds"], st["n_face_removed"],
               st["n_vol_dropped"], st["n_vol_nomesh"], 100 * st["surf_cov"]))
    log("  注意：本步骤只产出**可视化**网格；<inertial> 来自 _mass_*.json 那条独立链路，")
    log("        删面/丢面不可能影响动力学。")
    log("")
    log("VERDICT: %s   耗时 %.1f s" % ("STL_OK" if all_ok else "STL_PARTIAL", time.time() - t0))

    io.open(OUT_RPT, "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
