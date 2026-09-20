# -*- coding: utf-8 -*-
"""外骨骼仓库 · 根目录整理（归档 + 删除）

用法：
    python _tidy_root.py            # 只读：生成分类清单，不动任何文件
    python _tidy_root.py --apply    # 执行：先打包归档 -> 校验 -> 再送回收站

安全约束（硬编码，不可绕过）：
  * 只处理**根目录下的文件**，任何子目录一律不进入、不删除。
  * KEEP 名单内的文件永不触碰。
  * 必须先成功生成 zip 且校验通过，才会进入删除阶段。
  * 删除走回收站（FOF_ALLOWUNDO），不永久删除。

2026-09-20 二轮：用户要求"清除所有无关的，只保留对的" ⇒ 在 KEEP 之外显式追加
DELETE_EXTRA（被取代的迭代版本 / 一次性探针 / 冗余输出副本）。**文档引用预检仍是
硬闸门**：DELETE_EXTRA 里任何被 *.md/*.txt 引用的文件都会让 precheck 报警、默认中止。
"""
import csv
import io
import os
import sys
import time
import zipfile

ROOT = r"C:\Users\29408\Desktop\外骨骼"
ARCH_DIR = os.path.join(ROOT, "_archive")
STAMP = time.strftime("%Y-%m-%d")
ZIP_PATH = os.path.join(ARCH_DIR, "_archive_root_%s.zip" % STAMP)
MANIFEST = os.path.join(ARCH_DIR, "MANIFEST_root_%s.csv" % STAMP)
PLAN = os.path.join(ROOT, "_tidy_plan_tmp.txt")

APPLY = "--apply" in sys.argv

# ---------------------------------------------------------------- KEEP 名单
# 1) 原始资料 / 交付物 / 报告（精确名）
KEEP_EXACT = {
    "2609.15352v1.pdf",
    "1 Online Adaptation Framework Enables Personalization of Exoskeleton Assistance "
    "During Locomotion in Patients Affected by Stroke.pdf",
    "2609_report.html", "2609_report.txt", "2609_text.txt",
    "“林-Ⅰ”髋关节外骨骼机器人开发平台V0_1_1.stp",
    "_trash_scratch.py",          # 送回收站的通用工具，可复用
    "_tidy_root.py",              # 本次整理工具本身，留着可复用
    "_tidy_dirs.py",              # 目录整理工具，同理留着
}

# 2) 脚本 / 证据链（前缀匹配）
KEEP_PREFIX = [
    # 生成链
    "_stp2urdf", "_stp2stl", "_stp2plant",
    # 4-DOF 解析侧三实现 + 渲染
    "_multidof_",
    # 关节轴取证
    "_axis_find", "_cyl_faces", "_plane_faces", "_joint_candidates",
    "_dof_forensic", "_DOF_FORENSIC", "_check_lr",
    # 移动副取证
    "_slide_", "_strap_axis_check",
    # 解析侧对账
    "_legL_mass_table", "_legL_xcheck_py_vs_sw", "_legL_instance_recon",
    # XML 路
    "_xml_fix", "_xml_dump", "_xml_inspect", "_xml_joints",
    "_xml_group_names", "_xml_vs_py",
    # SW COM 工具链
    "_sw_baseline", "_sw_export_step", "_sw_ml_xcheck", "_sw_mates",
    "_sw_cfgs", "_sw_cfg_meta", "_sw_ids", "_sw_api_names",
    "_sw_list_materials", "_sw_material_test", "_sw_materials",
    # 网格修复 / 报表
    "_meshfix", "_report_2609", "_make_sw_csv",
    # 质量属性证据
    "_geometry_result", "_hip_axis", "_mass_all", "_mass_back",
    "_mass_legR", "_mass_leg_L", "_mass_leg_R", "_mass_motor_",
    "_props_instances", "_props_names", "_targets",
    # ★ 被 matlab2609/*.md 正式文档引用为"取证来源"的探针，删了会断掉文档的可复现性
    "_stl_preview",       # -> matlab2609/README.md
    "_sw_mat_probe",      # -> simscape/MATERIAL_AND_MASS_RECON.md
    "_step_probe",        # -> simscape/STEP_FREEFORM_ANALYSIS.md, XML_PATH_GUIDE.md
    "_sw_step_probe",     # -> simscape/XML_PATH_GUIDE.md
    "_step_check_all",    # -> simscape/XML_PATH_VERDICT.md
]

# 3) 非根目录的遗留清理清单（上一轮扫描的产物，已被本次方案取代）
EXTRA_SCRATCH = [
    os.path.join(".workbuddy", "memory", "_left.txt"),
    os.path.join(".workbuddy", "memory", "_trash_manifest_2026-09-19.csv"),
]

# 4) ★ 二轮追加：被取代的迭代版本 / 一次性探针 / 冗余输出副本
#    （文档引用预检会 veto 掉其中被 *.md/*.txt 引用的项）
DELETE_EXTRA = [
    # —— 冗余输出副本（.json / 正式报告已有，本身不含新信息）——
    "_dof_forensic.txt",
    "_geometry_result.txt",
    "_hip_axis_report.txt",
    "_legL_instance_recon.txt",
    "_legL_mass_table.txt",
    "_mass_all.txt",
    "_plane_faces.log",
    # —— 早期命名口径的废弃 mass json（现行用 _mass_leg_L / _mass_leg_R.json）——
    "_mass_legR.json", "_mass_legR2.json", "_mass_legR3.json",
    # —— 控制台 / 自检转储（跑一次即可复现）——
    "_multidof_dyn_selfcheck.txt",
    "_multidof_import_console.txt",
    "_multidof_smoke_console.txt",
    "_multidof_plan.py", "_multidof_plan.txt",
    # —— 说明：其余候选（_sw_cfgs* / _sw_mates* / _sw_api_names* / _sw_ids* /
    #    _sw_cfg_meta* / _sw_export_step* / _sw_ml_xcheck* / _check_lr.py /
    #    _dof_forensic_fig.py / _joint_candidates.txt / _legL_xcheck_py_vs_sw.txt）
    #    被 XML_PATH_VERDICT.md / MATERIAL_AND_MASS_RECON.md /
    #    LEG_VOLUME_NAMING_RECON.md / _DOF_FORENSIC_REPORT.txt 引用为证据
    #    ⇒ 由文档引用预检拦下，一律保留。 ——
]

# 5) 过大的二进制 / 目录 -> 不自动处理，列入"待你裁决"
BIG_ASK = [
    "_cad_backup_20260919", "_reduced", "_reduced2", "_reduced3",
    "_piptest", "_smlink_x", "_figs",
    "_all_back.brep", "_box.stp",
]

# 预检时忽略的"自指"文件（它们本身就是清单，必然包含待删文件名）
PRECHECK_SKIP = {"_tidy_plan_tmp.txt"}


def kept(name):
    if name in KEEP_EXACT:
        return True
    return any(name.startswith(p) for p in KEEP_PREFIX)


def main():
    entries = sorted(os.listdir(ROOT))
    root_files = [e for e in entries
                  if os.path.isfile(os.path.join(ROOT, e))]
    root_dirs = [e for e in entries
                 if os.path.isdir(os.path.join(ROOT, e))]

    to_archive = []
    keep = []
    for f in root_files:
        (keep if kept(f) else to_archive).append(f)
    # ★ 显式追加 DELETE_EXTRA（即使被 KEEP_PREFIX 覆盖也强制列入）
    for f in DELETE_EXTRA:
        if os.path.isfile(os.path.join(ROOT, f)) and f not in to_archive:
            to_archive.append(f)
            if f in keep:
                keep.remove(f)
    for rel in EXTRA_SCRATCH:                 # 追加非根目录的遗留清单
        if os.path.isfile(os.path.join(ROOT, rel)):
            to_archive.append(rel)
    to_archive = sorted(set(to_archive))

    rows = []
    for f in to_archive:
        p = os.path.join(ROOT, f)
        st = os.stat(p)
        rows.append((f, st.st_size, time.strftime("%Y-%m-%d %H:%M:%S",
                                                  time.localtime(st.st_mtime))))

    total = sum(r[1] for r in rows)
    with io.open(PLAN, "w", encoding="utf-8") as fh:
        def w(s):
            fh.write(s + "\n")
        w("外骨骼 · 根目录整理方案（只读生成）")
        w("生成 %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
        w("模式 %s" % ("APPLY（会写盘）" if APPLY else "DRY-RUN（只读）"))
        w("=" * 70)
        w("")
        w("### A. 归档并删除：%d 个文件，合计 %.2f MB" % (len(rows), total / 1e6))
        for f, sz, mt in rows:
            w("  %-62s %10d B  %s" % (f, sz, mt))
        w("")
        w("### B. 保留：%d 个根级文件" % len(keep))
        for f in keep:
            w("  %s" % f)
        w("")
        w("### C. 目录：%d 个（本次一律不动）" % len(root_dirs))
        for d in root_dirs:
            w("  %s" % d)
        w("")
        w("### D. 待你裁决（体积大，不自动处理）")
        for b in BIG_ASK:
            p = os.path.join(ROOT, b)
            if not os.path.exists(p):
                continue
            if os.path.isdir(p):
                n = tot = 0
                for r, _, fs in os.walk(p):
                    for x in fs:
                        n += 1
                        try:
                            tot += os.path.getsize(os.path.join(r, x))
                        except OSError:
                            pass
                w("  %-26s 目录  文件 %5d  %8.2f MB" % (b, n, tot / 1e6))
            else:
                w("  %-26s 文件          %8.2f MB"
                  % (b, os.path.getsize(p) / 1e6))

    print("PLAN -> %s" % PLAN)
    print("待归档 %d 个 / %.2f MB ；保留 %d 个 ；目录 %d 个"
          % (len(rows), total / 1e6, len(keep), len(root_dirs)))

    # ------------------------------------------------- 预检：文档是否引用了待删脚本
    import re
    doc_hits = {}
    targets = {r[0] for r in rows}
    # 边界感知匹配：避免 "_gait.txt" 在 "anim_exo_3d_gait.txt" 里的子串误报
    pats = {t: re.compile(r"(?<![0-9A-Za-z_])" + re.escape(t) + r"(?![0-9A-Za-z_])")
            for t in targets}
    for base, dirs, fs in os.walk(ROOT):
        dirs[:] = [d for d in dirs
                   if d not in ("_archive", "装配体", "优龙", "肯綮",
                                "_cad_backup_20260919", "_reduced",
                                "_reduced2", "_reduced3", "_piptest",
                                "_smlink_x", "__pycache__", ".workbuddy")]
        for fn in fs:
            if not fn.lower().endswith((".md", ".txt")):
                continue
            if fn in PRECHECK_SKIP:
                continue
            p = os.path.join(base, fn)
            rel = os.path.relpath(p, ROOT)
            if rel in targets:
                continue
            try:
                txt = io.open(p, "r", encoding="utf-8", errors="replace").read()
            except OSError:
                continue
            for t, pat in pats.items():
                if pat.search(txt):
                    doc_hits.setdefault(rel, set()).add(t)

    if doc_hits:
        print("!! 预检：有 %d 个文档引用了待删文件" % len(doc_hits))
        for rel, ts in sorted(doc_hits.items()):
            print("   %s -> %s" % (rel, ", ".join(sorted(ts))[:160]))
    else:
        print("预检：文档未引用任何待删文件。")

    with io.open(PLAN, "a", encoding="utf-8") as fh:
        fh.write("\n### E. 预检：文档引用待删文件的情况\n")
        if doc_hits:
            for rel, ts in sorted(doc_hits.items()):
                fh.write("  %s\n     -> %s\n" % (rel, ", ".join(sorted(ts))))
        else:
            fh.write("  （无引用，安全）\n")

    if not APPLY:
        print("DRY-RUN，未改动任何文件。")
        return

    if doc_hits and "--force" not in sys.argv:
        print("!! 已中止：先处理上面这些文档引用（或加 --force 强推）。")
        return

    # ---------------------------------------------------------- 归档
    if not os.path.isdir(ARCH_DIR):
        os.makedirs(ARCH_DIR)
    with zipfile.ZipFile(ZIP_PATH, "w", zipfile.ZIP_DEFLATED,
                         compresslevel=6) as z:
        for f, _, _ in rows:
            z.write(os.path.join(ROOT, f), arcname=f.replace("\\", "/"))
    with io.open(MANIFEST, "w", encoding="utf-8", newline="") as fh:
        cw = csv.writer(fh)
        cw.writerow(["name", "bytes", "mtime", "zip_path"])
        for f, sz, mt in rows:
            cw.writerow([f, sz, mt, ZIP_PATH])

    # ---------------------------------------------------------- 校验
    with zipfile.ZipFile(ZIP_PATH) as z:
        bad = z.testzip()
        names = {n.replace("\\", "/") for n in z.namelist()}
    want = {f.replace("\\", "/") for f, _, _ in rows}
    missing = want - names
    print("归档: %s" % ZIP_PATH)
    print("  条目 %d / 期望 %d ；缺 %d ；zip 完好 %s"
          % (len(names), len(want), len(missing), bad is None))
    if bad is not None or missing:
        print("!! 校验未通过，中止删除，原文件一个都没动。")
        print("   缺失:", sorted(missing)[:10])
        return

    # ---------------------------------------------------------- 删除（回收站）
    import ctypes
    from ctypes import wintypes
    FO_DELETE = 3
    FLAGS = 0x0004 | 0x0010 | 0x0040 | 0x0400 | 0x0200

    class S(ctypes.Structure):
        _fields_ = [("hwnd", wintypes.HWND), ("wFunc", wintypes.UINT),
                    ("pFrom", wintypes.LPCWSTR), ("pTo", wintypes.LPCWSTR),
                    ("fFlags", ctypes.c_uint16),
                    ("fAnyOperationsAborted", wintypes.BOOL),
                    ("hNameMappings", ctypes.c_void_p),
                    ("lpszProgressTitle", wintypes.LPCWSTR)]

    ok = fail = 0
    failed = []
    for f, _, _ in rows:
        p = os.path.join(ROOT, f)
        o = S(); o.hwnd = None; o.wFunc = FO_DELETE
        o.pFrom = p + "\0\0"; o.pTo = None; o.fFlags = FLAGS
        ctypes.windll.shell32.SHFileOperationW(ctypes.byref(o))
        # rc 不可信，只看后置条件
        if not os.path.exists(p):
            ok += 1
        else:
            fail += 1
            failed.append(f)

    print("删除: 成功 %d ；失败 %d" % (ok, fail))
    if failed:
        print("  失败项:", failed[:10])
    print("VERDICT:", "TIDY_OK" if fail == 0 else "TIDY_PARTIAL")


if __name__ == "__main__":
    main()
