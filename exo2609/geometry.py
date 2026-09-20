# -*- coding: utf-8 -*-
"""
exo2609.geometry —— 从 CAD 几何反算 2609 动力学模型的等效参数
===========================================================
输入（全部由 STEP 解析链路产出，见仓库根目录脚本）:
  _props_instances.json   装配实例树：每个放置实例的 4x4 全局变换、零件名、所含实体
  _mass_<group>.json      gmsh/OCC 提取的每个 solid 的 体积 / 质心 / 绕质心惯量（密度=1）
  _cyl_faces.json         每个 solid 的圆柱面轴线（局部），用于反推关节轴

输出:
  StepGeometryResult      左/右腿的等效摆参数（质量 m、质心、绕髋轴惯量 I、轴到质心垂距 d）

单位约定（关键，容易错）
------------------------
* 长度 mm，质量 kg，惯量 kg·m²，密度 kg/m³。
* gmsh 的 getMatrixOfInertia 给的是**密度=1**的惯量，单位 mm⁵
  （体积 mm³ × 长度² mm²）。换算到 kg·m²：
      I[kg·m²] = ρ[kg/m³] · I_local[mm⁵] · 1e-15
  推导：ρ[g/mm³] = ρ[kg/m³]·1e-6；I[g·mm²] = ρ[g/mm³]·I_local；
        1 g·mm² = 1e-9 kg·m²  ⇒  ρ·1e-6·1e-9 = ρ·1e-15。
* 质量：m[kg] = ρ[kg/m³] · V[mm³] · 1e-9
* 平行轴项：m[kg] · perp[mm]² · 1e-6

密度表是可配置项（CAD 未携带材质信息），全程可审计：结果里带每个零件的密度与质量。
"""

from __future__ import annotations

import io
import json
import os
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

G = 9.80665                 # m/s^2
MM3_TO_M3 = 1e-9
MM2_TO_M2 = 1e-6
MM5_TO_KGM2_PER_RHO = 1e-15

# ---------------------------------------------------------------------------
# ★ 精确名覆盖表（kg/m³）：**优先于**下面的关键字表。
#
#   为什么需要这一层：关键字表是「子串匹配 + 先命中先算」，往里加模糊键会误伤
#   同族名（例：加裸 `轴` 会把 `腿部_轴盖` 也拉到钢，而轴盖功能上是铝盖板）。
#   所以凡「语义已明确、无歧义」的件，一律在这里**逐字列名**。
#
#   ⚠ 名字必须与 `_props_instances.json` 的 `product_names` **逐字相同**。
#     实测：`电机_轴` 无前缀；但**腿杆左右不对称** ——
#     leg_L = `腿部_腿杆_片状V5`，leg_R = `腿部_腿杆_片状V5MR`。
#     加条目之前先跑：
#       python -c "import sys;sys.path.insert(0,'.');from exo2609.geometry import CadGeometry as C;print(sorted({i.part_name for i in C('.').placed}))"
#
#   依据：matlab2609/simscape/MATERIAL_FUNCTIONAL_INFERENCE.md
# ---------------------------------------------------------------------------
PART_EXACT: Dict[str, Tuple[float, str]] = {
    # 电机轴：Ø8×32 实心轴（实测体积 = 实心圆柱的 93.6%），且它正是
    # `电机_黄铜轴套8_10_18`（Ø8 内孔，实测吻合 99.6%）的配合轴 -> 必须钢。
    # 以前因关键字表只有 `轴套`/`出轴`、**没有裸 `轴`** 而静默落到兜底铝。
    "电机_轴":              (7850.0, "steel shaft (exact)"),
    # 输出轴/轮毂；髋轴 r=22 圆面即此件 -> 钢。
    "电机_出轴":            (7850.0, "steel output shaft (exact)"),
    # 名字自带「黄铜」；显式列出，以免受关键字表顺序变动影响。
    "电机_黄铜轴套8_10_18":  (8500.0, "brass bushing (exact)"),
}

# ---------------------------------------------------------------------------
# 密度表（kg/m³）。按零件名关键字**顺序**匹配，先命中先算。
#
# ⚠ 没命中的件会落到 FALLBACK_DENSITY —— 这是整条链路**唯一**的假设，也是最大
#   的风险源：实测曾一次有 9 件 / 占整腿质量 **54.87%** 压在兜底值上，而流程
#   一声不吭。⇒ 调用方必须用 `is_fallback()` 把覆盖面审计出来
#   （见 `_stp2urdf.py` / `_stp2urdf_multidof.py` 收尾报告里的 "密度表覆盖审计"）。
#
# ⚠ 改这张表之前先全仓 grep 字面量副本（曾有 2 处硬内联副本 + 2 处 try/except 兜底）：
#     grep -rn "DENSITIES\|FALLBACK_DENSITY\|density_of" --include=*.py .
# ---------------------------------------------------------------------------
DEFAULT_DENSITIES: List[Tuple[Tuple[str, ...], float, str]] = [
    (("黄铜", "铜"),                                      8500.0, "brass"),
    (("弹簧钢", "钢珠", "轴承"),                            7850.0, "bearing steel"),
    (("螺丝", "螺钉", "螺母", "螺柱", "顶丝", "销", "卡簧",
      "垫片", "弹垫", "平垫", "轴套", "出轴"),               7850.0, "steel fastener/shaft"),
    (("绑缚", "魔术贴", "织带"),                           1150.0, "nylon/textile"),
    (("麦拉片", "按键", "导光", "泡棉", "胶"),               1200.0, "polymer"),
    (("PCB", "电路板", "元件", "OPEN_CASCADE", "USER_LIBRARY",
      "SOT23", "LQFP", "HDR1", "ACP3225", "LEADER"),      1500.0, "electronics"),
]
FALLBACK_DENSITY = 2700.0        # aluminium
FALLBACK_LABEL = "aluminium (default)"


def density_of(part_name: str, table=DEFAULT_DENSITIES, fallback=FALLBACK_DENSITY):
    """零件名 -> (密度 kg/m³, 标签)。★ 精确名优先，其次关键字表，最后兜底。"""
    if part_name in PART_EXACT:                  # ★ 精确名优先（见 PART_EXACT 注释）
        return PART_EXACT[part_name]
    up = (part_name or "").upper()
    for keys, rho, label in table:
        for k in keys:
            if k.upper() in up:
                return rho, label
    return fallback, FALLBACK_LABEL


def is_fallback(label) -> bool:
    """该标签是否来自兜底密度（= 我们没认出这个零件名）。

    兼容三种历史写法：'aluminium (default)' / 'aluminium (FALLBACK)' / '铝(兜底)'。
    """
    lab = str(label or "")
    return (lab == FALLBACK_LABEL) or ("FALLBACK" in lab.upper()) or ("兜底" in lab)


# ---------------------------------------------------------------------------
@dataclass
class SolidProps:
    msb: int
    volume_mm3: float
    cog_local: np.ndarray        # (3,) mm
    inertia_local: np.ndarray    # (3,3) mm^5（密度=1）


@dataclass
class PlacedInstance:
    nauo: int
    pd: int
    part_name: str
    ancestors: List[str]
    T: np.ndarray                # (4,4)
    solids: List[int]


@dataclass
class LegMassProps:
    name: str
    m_kg: float
    cog_world: np.ndarray
    I_axis: float                # kg·m²
    d_perp_m: float              # m
    G_amp: float                 # N·m
    M: float                     # kg·m²
    n_instances: int
    n_solids: int
    per_part: List[dict] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


@dataclass
class StepGeometryResult:
    axis_point: np.ndarray
    axis_dir: np.ndarray
    axis_evidence: dict
    groups: Dict[str, LegMassProps]
    total_mass_kg: float
    notes: List[str] = field(default_factory=list)

    def summary(self) -> str:
        L = ["hip axis: point=%s mm  dir=%s" % (np.round(self.axis_point, 3).tolist(),
                                                np.round(self.axis_dir, 6).tolist()),
             "  evidence: %s" % json.dumps(self.axis_evidence, ensure_ascii=False), ""]
        for g, r in self.groups.items():
            L.append("[%-8s] m=%.4f kg  d=%.4f m  I_axis=%.6f kg·m²  G_amp=%.4f N·m  "
                     "(%d inst / %d solids)"
                     % (g, r.m_kg, r.d_perp_m, r.I_axis, r.G_amp, r.n_instances, r.n_solids))
            for w in r.warnings:
                L.append("      ! %s" % w)
        L += ["", "total CAD mass = %.4f kg" % self.total_mass_kg]
        for n in self.notes:
            L.append("note: %s" % n)
        return "\n".join(L)


GROUP_OF_TOP = {
    "腿部设计_ 左": "leg_L", "腿部设计_ 右": "leg_R",
    "电机设计_ 左": "motor_L", "电机设计_ 右": "motor_R",
    "背部设计": "back",
}


class CadGeometry:
    def __init__(self, root: str, densities=DEFAULT_DENSITIES, fallback=FALLBACK_DENSITY):
        self.root = root
        self.densities = densities
        self.fallback = fallback
        with io.open(os.path.join(root, "_props_instances.json"), "r", encoding="utf-8") as f:
            self.inst_json = json.load(f)
        with io.open(os.path.join(root, "_cyl_faces.json"), "r", encoding="utf-8") as f:
            self.cyl = json.load(f)
        self.pd_meta = self.inst_json["pd_meta"]
        self.product_names = self.inst_json["product_names"]
        self.placed = [PlacedInstance(nauo=p["nauo"], pd=int(p["pd"]),
                                      part_name=p["part_name"], ancestors=p["ancestors"],
                                      T=np.asarray(p["T"], float),
                                      solids=[int(x) for x in p["solids"]])
                       for p in self.inst_json["placed"]]
        self._mass_cache: Dict[str, Optional[dict]] = {}

    # ---------------- 基础 ----------------
    def group_of(self, inst: PlacedInstance) -> str:
        return GROUP_OF_TOP.get(inst.ancestors[0] if inst.ancestors else "", "other")

    def mass_json(self, group: str) -> Optional[dict]:
        if group not in self._mass_cache:
            p = os.path.join(self.root, "_mass_%s.json" % group)
            if os.path.exists(p):
                with io.open(p, "r", encoding="utf-8") as f:
                    self._mass_cache[group] = json.load(f)
            else:
                self._mass_cache[group] = None
        return self._mass_cache[group]

    # ---------------- 关节轴 ----------------
    def _leg_faces(self):
        """所有腿组零件的圆柱面（世界系）。"""
        out = []
        for inst in self.placed:
            g = self.group_of(inst)
            if not g.startswith("leg"):
                continue
            R, t = inst.T[:3, :3], inst.T[:3, 3]
            for msb in inst.solids:
                for c in self.cyl.get(str(msb), []):
                    d = R @ np.asarray(c["d"], float)
                    nd = np.linalg.norm(d)
                    if nd < 1e-12:
                        continue
                    out.append({"grp": g, "part": inst.part_name,
                                "o": R @ np.asarray(c["o"], float) + t,
                                "d": d / nd, "r": c["r"] or 0.0, "t": c["t"]})
        return out

    @staticmethod
    def _canon(d):
        k = int(np.argmax(np.abs(d)))
        return d if d[k] > 0 else -d

    def hip_axis_from_shafts(self, part_keys=("出轴",)):
        """
        首选判据：左右两侧「电机出轴」上**半径最大**的圆柱面，其轴线即髋关节转轴。
        两条轴线各自给出一个点，取公共拟合直线（方向取平均后归一，点取各轴线垂足均值）。
        返回 None 表示找不到候选。
        """
        lines = []
        for inst in self.placed:
            if not self.group_of(inst).startswith("leg"):
                continue
            if not any(k in (inst.part_name or "") for k in part_keys):
                continue
            R, t = inst.T[:3, :3], inst.T[:3, 3]
            best = None
            for msb in inst.solids:
                for c in self.cyl.get(str(msb), []):
                    if c["t"] != "CYL":
                        continue
                    if best is None or (c["r"] or 0.0) > (best["r"] or 0.0):
                        best = c
            if best is None:
                continue
            d = R @ np.asarray(best["d"], float)
            lines.append({"grp": self.group_of(inst), "part": inst.part_name,
                          "o": R @ np.asarray(best["o"], float) + t,
                          "d": d / np.linalg.norm(d), "r": float(best["r"] or 0.0)})
        if not lines:
            return None
        ref = lines[0]["d"]
        dirs = [l["d"] if float(l["d"] @ ref) > 0 else -l["d"] for l in lines]
        d_avg = np.mean(np.asarray(dirs), axis=0)
        d_avg = d_avg / np.linalg.norm(d_avg)
        p0 = np.mean([l["o"] for l in lines], axis=0)
        feet = []
        for l in lines:
            v = l["o"] - p0
            feet.append(l["o"] - d_avg * float(v @ d_avg))
        p_fit = np.mean(np.asarray(feet), axis=0)
        resid = []
        for l in lines:
            v = l["o"] - p_fit
            resid.append(float(np.linalg.norm(v - d_avg * float(v @ d_avg))))
        return {"dir": d_avg, "point": p_fit, "lines": lines,
                "residual_mm": resid, "max_residual_mm": max(resid)}

    def find_hip_axis(self, axial_keys=("出轴", "轴套", "轴承", "转子"),
                      tol_dir=1e-3, tol_dist=0.6):
        """备选判据：全腿组圆柱面共轴投票（关节处共轴面最多）。"""
        faces = self._leg_faces()
        seeds = [i for i, f in enumerate(faces)
                 if any(k in (f["part"] or "") for k in axial_keys)]
        best = None
        for si in seeds:
            fs = faces[si]
            d0 = self._canon(fs["d"])
            members = []
            for j, f in enumerate(faces):
                if np.linalg.norm(np.cross(d0, self._canon(f["d"]))) > tol_dir:
                    continue
                v = f["o"] - fs["o"]
                if np.linalg.norm(v - d0 * float(v @ d0)) > tol_dist:
                    continue
                members.append(j)
            if not members:
                continue
            feet = [faces[j]["o"] - d0 * float((faces[j]["o"] - fs["o"]) @ d0) for j in members]
            p_center = np.mean(np.asarray(feet), axis=0)
            res = float(np.mean([np.linalg.norm((faces[j]["o"] - p_center)
                                                - d0 * float((faces[j]["o"] - p_center) @ d0))
                                 for j in members]))
            cand = {"seed": si, "part": fs["part"], "grp": fs["grp"], "r": fs["r"],
                    "n": len(members), "dir": d0.tolist(), "point": p_center.tolist(),
                    "residual_mm": res,
                    "grps": sorted({faces[j]["grp"] for j in members})}
            if best is None or cand["n"] > best["n"]:
                best = cand
        return best, faces

    # ---------------- 体积 -> 实体 ----------------
    def _map_solids(self, group: str) -> Tuple[Dict[int, SolidProps], List[str]]:
        mj = self.mass_json(group)
        if not mj:
            return {}, ["no _mass_%s.json" % group]
        warns: List[str] = []
        recs = list(mj["volumes"])
        for r in recs:
            r["_short"] = (r.get("name") or "").split("/")[-1]

        expected: Dict[int, List[int]] = {}
        for inst in self.placed:
            if self.group_of(inst) != group:
                continue
            m = self.pd_meta.get(str(inst.pd))
            if m and m.get("solids"):
                expected[inst.pd] = [int(x) for x in m["solids"]]

        used = [False] * len(recs)
        out: Dict[int, SolidProps] = {}
        for pd, msbs in expected.items():
            nm = self.product_names.get(str(self.pd_meta[str(pd)].get("product")), "")
            cand = [k for k, r in enumerate(recs) if not used[k] and r["_short"] == nm]
            if len(cand) == len(msbs):
                for k, msb in zip(cand, msbs):
                    used[k] = True
                    out[msb] = self._to_solid(msb, recs[k])
            elif cand:
                warns.append("part '%s' pd=%d: %d named volume vs %d solid -> left for step 2"
                             % (nm, pd, len(cand), len(msbs)))
        left = [k for k, r in enumerate(recs) if not used[k]]
        need = [(pd, [s for s in msbs if s not in out]) for pd, msbs in expected.items()]
        need = [(pd, s) for pd, s in need if s]
        n_slots = sum(len(s) for _, s in need)
        if left and n_slots:
            warns.append("step2: %d unnamed volume(s) -> %d empty solid slot(s)" % (len(left), n_slots))
            it = iter(left)
            for pd, msbs in need:
                for msb in msbs:
                    try:
                        k = next(it)
                    except StopIteration:
                        break
                    out[msb] = self._to_solid(msb, recs[k])
                    used[k] = True
        n_un = len(recs) - sum(used)
        if n_un:
            warns.append("%d volume(s) unattributed" % n_un)
        return out, warns

    @staticmethod
    def _to_solid(msb: int, rec: dict) -> SolidProps:
        return SolidProps(msb=msb, volume_mm3=float(rec["volume"]),
                          cog_local=np.asarray(rec["cog"], float),
                          inertia_local=np.asarray(rec["inertia_cog"], float).reshape(3, 3))

    # ---------------- 聚合 ----------------
    def aggregate(self, group: str, axis_point, axis_dir) -> LegMassProps:
        a = np.asarray(axis_dir, float)
        a = a / np.linalg.norm(a)
        p = np.asarray(axis_point, float)

        solids, warns = self._map_solids(group)
        insts = [i for i in self.placed if self.group_of(i) == group]

        m_tot = 0.0
        first = np.zeros(3)
        I_axis = 0.0
        per_part = defaultdict(lambda: {"rho": 0.0, "label": "", "V_mm3": 0.0, "m_kg": 0.0, "n": 0})
        n_used = 0
        missing = 0
        for inst in insts:
            R, t = inst.T[:3, :3], inst.T[:3, 3]
            rho, label = density_of(inst.part_name, self.densities, self.fallback)
            for msb in inst.solids:
                sp = solids.get(msb)
                if sp is None:
                    missing += 1
                    continue
                n_used += 1
                m = rho * sp.volume_mm3 * MM3_TO_M3
                c_w = R @ sp.cog_local + t
                I_w = R @ sp.inertia_local @ R.T
                I_axis += float(a @ I_w @ a) * rho * MM5_TO_KGM2_PER_RHO
                v = c_w - p
                perp = v - a * float(v @ a)
                I_axis += m * float(perp @ perp) * MM2_TO_M2
                m_tot += m
                first += m * c_w
                d = per_part[inst.part_name]
                d["rho"], d["label"] = rho, label
                d["V_mm3"] += sp.volume_mm3
                d["m_kg"] += m
                d["n"] += 1
        if m_tot <= 0:
            raise ValueError("group %s: total mass 0 (mapping failed)" % group)
        cog = first / m_tot
        v = cog - p
        perp = v - a * float(v @ a)
        d_perp = float(np.linalg.norm(perp))
        if missing:
            warns.append("%d solid instance(s) lack mass props" % missing)
        return LegMassProps(
            name=group, m_kg=m_tot, cog_world=cog, I_axis=I_axis,
            d_perp_m=d_perp * 1e-3, G_amp=m_tot * G * d_perp * 1e-3, M=I_axis,
            n_instances=len(insts), n_solids=n_used,
            per_part=[{"part": k, **v} for k, v in
                      sorted(per_part.items(), key=lambda kv: -kv[1]["m_kg"])],
            warnings=warns)

    # ---------------- 入口 ----------------
    def build(self, prefer_shaft_axis: bool = True) -> StepGeometryResult:
        notes, ev = [], {}
        axis_point = axis_dir = None
        if prefer_shaft_axis:
            fit = self.hip_axis_from_shafts()
            if fit is not None:
                axis_point, axis_dir = fit["point"], fit["dir"]
                ev = {"method": "motor-output-shaft cylinder axes (both legs)",
                      "n_lines": len(fit["lines"]),
                      "max_residual_mm": round(float(fit["max_residual_mm"]), 4),
                      "lines": [{"grp": l["grp"], "part": l["part"], "r_mm": l["r"]}
                                for l in fit["lines"]]}
                notes.append("hip axis fitted from %d output-shaft cylinder axes; "
                             "max residual %.4f mm" % (len(fit["lines"]), fit["max_residual_mm"]))
        if axis_point is None:
            best, _ = self.find_hip_axis()
            if best is None:
                raise ValueError("cannot identify hip axis")
            axis_point, axis_dir = np.asarray(best["point"]), np.asarray(best["dir"])
            ev = {"method": "coaxial-cylinder vote", "n_coaxial": best["n"],
                  "part": best["part"], "residual_mm": round(best["residual_mm"], 4),
                  "grps": best["grps"]}
            notes.append("hip axis from coaxial vote: part=%s n_coaxial=%d residual=%.3f mm"
                         % (best["part"], best["n"], best["residual_mm"]))
        # 备选判据也留档，便于交叉核对
        try:
            best, _ = self.find_hip_axis()
            if best:
                ev["cross_check_coaxial_vote"] = {
                    "part": best["part"], "n_coaxial": best["n"],
                    "dir": [round(x, 6) for x in best["dir"]],
                    "point": [round(x, 3) for x in best["point"]],
                    "residual_mm": round(best["residual_mm"], 4), "grps": best["grps"]}
        except Exception as e:
            notes.append("coaxial cross-check failed: %s" % e)

        groups = {}
        for g in ("leg_L", "leg_R", "motor_L", "motor_R", "back"):
            if self.mass_json(g) is None:
                continue
            groups[g] = self.aggregate(g, axis_point, axis_dir)
        return StepGeometryResult(axis_point=axis_point, axis_dir=axis_dir, axis_evidence=ev,
                                  groups=groups,
                                  total_mass_kg=sum(r.m_kg for r in groups.values()),
                                  notes=notes)


def _main():
    root = r"C:\Users\29408\Desktop\外骨骼"
    cad = CadGeometry(root)
    res = cad.build()
    payload = {
        "axis": {"point": res.axis_point.tolist(), "dir": res.axis_dir.tolist(),
                 "evidence": res.axis_evidence},
        "groups": {g: {"m_kg": r.m_kg, "cog_world_mm": r.cog_world.tolist(),
                       "I_axis_kgm2": r.I_axis, "d_perp_m": r.d_perp_m,
                       "G_amp_Nm": r.G_amp, "M_kgm2": r.M,
                       "n_instances": r.n_instances, "n_solids": r.n_solids,
                       "warnings": r.warnings, "per_part": r.per_part}
                   for g, r in res.groups.items()},
        "total_mass_kg": res.total_mass_kg,
        "notes": res.notes,
    }
    out = os.path.join(root, "_geometry_result.json")
    with io.open(out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=1)
    txt = res.summary() + "\n\nWROTE " + out
    with io.open(os.path.join(root, "_geometry_result.txt"), "w", encoding="utf-8") as f:
        f.write(txt)
    print(txt)


if __name__ == "__main__":
    _main()
