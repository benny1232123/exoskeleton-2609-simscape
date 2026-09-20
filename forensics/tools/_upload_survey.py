# -*- coding: utf-8 -*-
"""_upload_survey.py —— 选出「代码 + 文档 + 报告」上传集，排除大二进制与第三方 PDF。

输出： _arxiv_upload_list.txt （每行一个相对路径），并打印统计。
"""
import io
import os

ROOT = r"C:\Users\29408\Desktop\外骨骼"
LIST = os.path.join(ROOT, "_upload_list.txt")

EXCL_DIRS = {
    "装配体", "优龙", "肯綮", "_cad_backup_20260919", "_reduced3", "_archive",
    "__pycache__", ".pytest_cache", ".workbuddy", "_figs", "_reduced",
    "_reduced2", "_piptest", "_smlink_x", "slprj",
}
EXCL_EXT = {
    ".stl", ".stp", ".step", ".slx", ".slxc", ".png", ".fig", ".mat", ".zip",
    ".sldprt", ".sldasm", ".brep", ".pdf", ".gif", ".jpg", ".jpeg", ".avi",
    ".mp4", ".exe", ".dll", ".log", ".pyc", ".swp", ".pyo",
}
# 运行日志后缀（可复现，且部分仍带密度修正前的旧值，不上传）
EXCL_SUFFIX = ("_log.txt", "_console.txt", "_stdout.txt")
EXCL_NAMES = {"_upload_survey.py", "_upload_list.txt", "_tidy_plan_tmp.txt"}
MAX_BYTES = 5_000_000        # >此值跳过（GitHub 单文件硬限 100 MB；这里留足余量）

rows = []
skipped_big = []
for base, dirs, fs in os.walk(ROOT):
    dirs[:] = [d for d in dirs if d not in EXCL_DIRS and not d.startswith("_cad_backup")]
    for fn in fs:
        ext = os.path.splitext(fn)[1].lower()
        if ext in EXCL_EXT:
            continue
        if fn in EXCL_NAMES or fn.lower().endswith(EXCL_SUFFIX):
            continue
        p = os.path.join(base, fn)
        rel = os.path.relpath(p, ROOT)
        try:
            sz = os.path.getsize(p)
        except OSError:
            continue
        if sz > MAX_BYTES:
            skipped_big.append((rel, sz))
            continue
        rows.append((rel, sz))

rows.sort()
total = sum(s for _, s in rows)
lines = [r[0].replace("\\", "/") for r in rows]
io.open(LIST, "w", encoding="utf-8").write("\n".join(lines) + "\n")

print("upload set: %d files, %.2f MB" % (len(rows), total / 1e6))
print("--- by top dir ---")
from collections import Counter
c = Counter(r[0].split("\\")[0] if "\\" in r[0] else "(root)" for r in rows)
sz = Counter()
for rel, s in rows:
    k = rel.split("\\")[0] if "\\" in rel else "(root)"
    sz[k] += s
for k, v in sorted(c.items()):
    print("  %-20s %4d files  %8.2f MB" % (k, v, sz[k] / 1e6))
if skipped_big:
    print("--- skipped (>%d B) ---" % MAX_BYTES)
    for rel, s in skipped_big:
        print("  %-60s %9d" % (rel, s))
print("LIST ->", LIST)
