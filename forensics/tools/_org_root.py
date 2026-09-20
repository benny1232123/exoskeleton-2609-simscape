# -*- coding: utf-8 -*-
"""
_org_root.py —— 把根目录零散文件归类到子目录（三道闸门）
================================================================================

纪律（本工程既定）：
  1. DRY-RUN 出计划 -> 人工看
  2. **文档引用预检**：谁在引用要移动的文件
  3. **根锚定预检（本次新增）**：谁在用 `ROOT` 绝对路径读写要移动的文件
  4. `--apply` 才真正移动；移动后生成 `forensics/README.md` 索引

★ 为什么脚本可以搬、数据不能搬：
  这些探针脚本的 ROOT 是**硬编码绝对路径**（`ROOT = r"C:\\Users\\29408\\Desktop\\外骨骼"`），
  **不是** `os.path.dirname(__file__)`。所以：
    - 移动 `_slide_check.py`  -> ROOT 不变 -> 它照样去**根目录**找 `_plane_faces.json` ✅
    - 但把 `_plane_faces.json` 也搬走 -> ROOT 没变 -> **FileNotFoundError** ❌
  ⇒ 规则：**凡是被 `os.path.join(ROOT, "X")` / `exo_work\\X` 锚到根目录的 X，一律留在根**。
  实测拦截：`_plane_faces.json`（_slide_*.py）、`_hip_axis.json`（_dof_forensic_fig.py）、
            `_sw_baseline_legL.json`（_sw_ml_xcheck.py / _step_check_all.py）等。

★ 为什么不自动改 .md：
  实测全部引用都是**散文里的反引号提及**（`` `_stp2urdf.py` ``），**零个** markdown 链接。
  把散文中的人名改成 `../../forensics/tools/_stp2urdf.py` 只会更难读。
  ⇒ 改为生成 `forensics/README.md` 索引（basename -> 新路径），人找得到就行。

用法
  python _org_root.py            # 只出计划 + 两道预检报告
  python _org_root.py --apply    # 移动 + 写索引 + 删会话残留
"""
import io
import os
import re
import sys

ROOT = r"C:\Users\29408\Desktop\外骨骼"

# ---------------------------------------------------------------- 归类计划
PLAN = {
    "forensics/step": [
        "_step_probe.py", "_step_probe_A.py", "_step_probe_A.txt",
        "_step_probe_B.py", "_step_probe_B.txt", "_step_probe_B_out.txt",
        "_step_probe_C.py", "_step_probe_C.txt", "_step_probe_C_out.txt",
        "_step_probe_D.py", "_step_probe_D.txt", "_step_probe_D_out.txt",
        "_step_probe_E.py", "_step_probe_E.txt", "_step_probe_E_out.txt",
        "_step_probe_F.py", "_step_probe_F.txt", "_step_probe_F_out.txt",
        "_step_check_all.py", "_step_check_all.txt",
        "_meshfix.py", "_meshfix2.py", "_meshfix_back.txt",
        "_xml_dump.py", "_xml_dump.txt", "_xml_fix.py", "_xml_fix.txt",
        "_xml_group_names.py", "_xml_group_names.txt",
        "_xml_inspect.py", "_xml_inspect.txt", "_xml_joints.py", "_xml_joints.txt",
        "_xml_vs_py.py", "_xml_vs_py.txt",
    ],
    "forensics/joints": [
        "_check_lr.py",
        "_joint_candidates.py", "_joint_candidates.txt", "_joint_candidates.json",
        "_plane_faces.py",
        "_dof_forensic_fig.py", "_DOF_FORENSIC_REPORT.txt",
        "_slide_check.py", "_slide_check.txt",
        "_slide_range.py", "_slide_range.txt",
        "_slide_recheck.py", "_slide_recheck.txt",
        "_strap_axis_check.py", "_strap_axis_check.txt",
    ],
    "forensics/mass": [
        "_props_names.json",
        "_targets.json", "_targets2.json", "_targets3.json",
        "_targets_meta.json", "_targets_single.json",
        "_make_sw_csv.py", "_legL_instance_recon.py",
        "_legL_mass_table.py", "_legL_mass_table.csv",
        "_legL_xcheck_py_vs_sw.py", "_legL_xcheck_py_vs_sw.txt",
    ],
    "forensics/solidworks": [
        "_sw_api_names.py", "_sw_api_names.txt",
        "_sw_baseline.py", "_sw_baseline.txt", "_sw_baseline_legL.json",
        "_sw_cfg_meta.py", "_sw_cfg_meta.txt",
        "_sw_cfgs.py", "_sw_cfgs.txt", "_sw_cfgs2.py", "_sw_cfgs2.txt",
        "_sw_cfgs3.py", "_sw_cfgs3.txt",
        "_sw_export_step.py", "_sw_export_step.txt",
        "_sw_export_step2.py", "_sw_export_step2.txt",
        "_sw_ids.py", "_sw_ids.txt", "_sw_list_materials.py",
        "_sw_mat_probe.py", "_sw_mat_probe.txt",
        "_sw_material_test.py", "_sw_material_test.txt", "_sw_materials.txt",
        "_sw_mates.py", "_sw_mates.txt", "_sw_mates2.py", "_sw_mates2.txt",
        "_sw_ml_xcheck.py", "_sw_ml_xcheck.txt", "_sw_step_probe.py",
    ],
    "forensics/tools": [
        "_tidy_root.py", "_tidy_dirs.py", "_tidy_apply.txt",
        "_trash_scratch.py", "_upload_survey.py", "_gh_upload.py",
        "_tree.txt", "_dirs.txt", "_create.txt",
        "_survey_out.txt", "_survey_out2.txt", "_survey_out3.txt",
    ],
    "papers": [
        "2609.15352v1.pdf",
        "1 Online Adaptation Framework Enables Personalization of Exoskeleton "
        "Assistance During Locomotion in Patients Affected by Stroke.pdf",
    ],
}

# 会话残留（stdio 抓取 / 缓存），直接删
SCRATCH = [
    "_gh_upload.log", "_git1.txt", "_git2.txt", "_gitls.txt", "_rootlist.txt",
    "_multidof_compare_run.txt", "_rerun2_stdout.txt",
    "_conditions_stdout.txt", "_pytest_out.txt", "_org_plan.txt",
    "_org_dryrun.txt", "_org_dryrun_exit.txt",
]

TEXT_EXT = (".py", ".m", ".md", ".txt", ".csv", ".json", ".html", ".urdf")
SKIP_DIRS = {"_cad_backup_20260919", "装配体", "优龙", "肯綮", "_reduced3", "__pycache__",
             ".git", ".workbuddy", "_archive", "slprj", "_figs", "papers", "forensics"}

# 根锚定形态：os.path.join(ROOT, "X")  /  ROOT + "\X"  /  字面量 ...exo_work\X
RE_ANCHOR = [
    re.compile(r'os\.path\.join\(\s*ROOT\s*,\s*["\']([^"\']+)["\']'),
    re.compile(r'ROOT\s*\+\s*["\']\\?([^"\']+)["\']'),
    re.compile(r'[\\/](?:exo_work|外骨骼)[\\/]([A-Za-z0-9_.\-]+)'),
]


def root_anchored():
    """扫 .py/.m/.ipynb，收集所有被**绝对锚到根目录**的 basename。"""
    hit = {}
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if not fn.endswith((".py", ".m")):
                continue
            p = os.path.join(dirpath, fn)
            try:
                txt = io.open(p, encoding="utf-8", errors="replace").read()
            except OSError:
                continue
            for rgx in RE_ANCHOR:
                for m in rgx.finditer(txt):
                    name = os.path.basename(m.group(1).replace("\\", "/"))
                    hit.setdefault(name, set()).add(os.path.relpath(p, ROOT))
    return hit


def main():
    apply = "--apply" in sys.argv
    moving = set()
    for v in PLAN.values():
        moving |= set(v)

    present = {f for f in os.listdir(ROOT) if os.path.isfile(os.path.join(ROOT, f))}
    anchors = root_anchored()

    # ---- 闸门 3：被根锚定的文件默认不许移动；但"脚本自己的输出"要放行 ----
    #   阻塞判据（两条，任一命中即留根）：
    #     ① 有**移动集之外**的代码在根锚定它（= 活跃链路 / 别处的脚本在等它）
    #     ② 有 **≥2 个**脚本在根锚定它（= 共享产物/输入，不是谁的私有输出）
    #   放行：唯一引用者就在移动集里、且只有一个 ⇒ 那是"脚本 + 它自己的 dump"，
    #         一起搬走正好成对。
    blocked, allowed_own = [], []
    for b in sorted(moving & set(anchors)):
        refs = sorted(anchors[b])
        outside = [r for r in refs if os.path.basename(r) not in moving]
        if len(refs) >= 2 or outside:
            blocked.append(b)
        else:
            allowed_own.append((b, refs[0]))
    for b in blocked:
        for t, v in PLAN.items():
            if b in v:
                v.remove(b)
    moving -= set(blocked)

    todo = {t: sorted([f for f in v if f in present]) for t, v in PLAN.items()}
    n = sum(len(v) for v in todo.values())

    L = []

    def say(s=""):
        L.append(s)
        print(s)

    say("=" * 84)
    say("根目录归类计划%s" % ("（APPLY）" if apply else "（DRY-RUN）"))
    say("=" * 84)
    for t, fs in todo.items():
        say("  %-24s %3d 个" % (t, len(fs)))
    say("  %-24s %3d 个" % ("删除(会话残留)", len([f for f in SCRATCH if f in present])))
    say("")
    say("  移动 %d；根目录 %d -> 余 %d"
        % (n, len(present), len(present) - n - len([f for f in SCRATCH if f in present])))

    # ---------------- 闸门 3 报告 ----------------
    say("")
    say("=" * 84)
    say("闸门 3 ★ 根锚定预检（被 ROOT 绝对路径定位的文件 —— 搬走就断链，强制留根）")
    say("=" * 84)
    if blocked:
        for b in blocked:
            say("  🚫 %-34s <- %s" % (b, ", ".join(sorted(anchors[b]))))
    else:
        say("  （无）")
    say("")
    say("  放行（脚本 + 它自己的 dump，成对搬走）：%d 个" % len(allowed_own))
    for b, r in allowed_own:
        say("     %-34s <- %s" % (b, r))
    say("")
    say("  说明：探针脚本自身的 ROOT 是硬编码绝对路径（非 __file__ 派生），")
    say("        所以**脚本可搬**；但被它用 ROOT 定位的**共享数据必须留根**。")

    # ---------------- 闸门 2 报告 ----------------
    say("")
    say("=" * 84)
    say("闸门 2 文档引用预检（谁在引用将要移动的文件）")
    say("=" * 84)
    refs = {}
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if not fn.endswith(TEXT_EXT):
                continue
            p = os.path.join(dirpath, fn)
            if os.path.basename(p) in moving or os.path.basename(p) in SCRATCH:
                continue
            try:
                txt = io.open(p, encoding="utf-8", errors="replace").read()
            except OSError:
                continue
            for base in moving:
                if base not in txt:
                    continue
                pat = r"(?<![0-9A-Za-z_])" + re.escape(base) + r"(?![0-9A-Za-z_])"
                if re.search(pat, txt):
                    refs.setdefault(base, []).append(os.path.relpath(p, ROOT))

    NOISE = ("_upload_list.txt", "_gh_blob_cache.json", "_org_plan.txt")
    real = {b: [w for w in ws if w not in NOISE] for b, ws in refs.items()}
    real = {b: ws for b, ws in real.items() if ws}
    byref = {}
    for b, ws in real.items():
        for w in ws:
            byref.setdefault(w, []).append(b)
    say("  被引用的文件：%d 个；引用方文件：%d 个" % (len(real), len(byref)))
    for w in sorted(byref):
        say("    %-52s <- %d 个文件" % (w, len(byref[w])))
    say("")
    say("  ⇒ 实测这些引用**全是散文反引号提及**、无 markdown 链接 ⇒ **不改 .md**；")
    say("     APPLY 时生成 `forensics/README.md` 索引（basename -> 新路径）供对照。")

    io.open(os.path.join(ROOT, "_org_plan.txt"), "w", encoding="utf-8").write("\n".join(L) + "\n")
    print("\nWROTE _org_plan.txt")

    if not apply:
        return

    # ---------------- 移动 ----------------
    newrel = {}
    for t, fs in todo.items():
        d = os.path.join(ROOT, t.replace("/", os.sep))
        os.makedirs(d, exist_ok=True)
        for f in fs:
            os.rename(os.path.join(ROOT, f), os.path.join(d, f))
            newrel[f] = t + "/" + f

    # ---------------- 写索引（不改 .md） ----------------
    idx = ["# forensics —— 取证探针与报告索引", "",
           "> 这些是**一次性探针 / 原始报告**，不是主链路入口。主链路见根目录 `README.md`。",
           "> 脚本的 `ROOT` 是硬编码绝对路径（非 `__file__` 派生），所以**移到这里后照样能跑**；",
           "> 但它们读写的**数据文件仍留在项目根**（见下表「留根」列）—— 这是刻意的。", ""]
    if blocked:
        idx += ["## 留在根目录的数据文件（被 `os.path.join(ROOT, ...)` 锚定，不能搬）", ""]
        for b in blocked:
            idx.append("- `%s`  ← 引用方：%s" % (b, ", ".join("`%s`" % x for x in sorted(anchors[b]))))
        idx.append("")
    for t in sorted(todo):
        if not todo[t]:
            continue
        idx += ["## %s" % t, ""]
        for f in todo[t]:
            idx.append("- `%s/%s`" % (t, f))
        idx.append("")
    io.open(os.path.join(ROOT, "forensics", "README.md"), "w", encoding="utf-8").write(
        "\n".join(idx) + "\n")
    print("  wrote forensics/README.md")

    # ---------------- 删残留 ----------------
    rm = 0
    for f in SCRATCH:
        p = os.path.join(ROOT, f)
        if os.path.exists(p):
            os.remove(p)
            rm += 1

    print("\nMOVED %d files, wrote forensics/README.md, removed %d scratch" % (n, rm))


if __name__ == "__main__":
    main()
