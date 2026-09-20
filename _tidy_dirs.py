# -*- coding: utf-8 -*-
"""第二阶段整理：把已被取代/可再生的中间目录归档后删除。

对象（均已核对过"无现役脚本引用 / 内容可重建"）：
  _piptest/   一个 gmsh wheel（可重下）
  _smlink_x/  Simscape Multibody Link 解包残片（8 个几十字节的小文件）
  _reduced/   早期裁剪 CAD，已被 _reduced3 取代
  _reduced2/  同上
  _figs/      空目录

不动：_cad_backup_20260919/（CAD 的唯一备份，留给用户裁决）

用法： python _tidy_dirs.py [--apply]
"""
import ctypes
import io
import os
import sys
import time
import zipfile
from ctypes import wintypes

ROOT = r"C:\Users\29408\Desktop\外骨骼"
ARCH_DIR = os.path.join(ROOT, "_archive")
STAMP = time.strftime("%Y-%m-%d")
ZIP_PATH = os.path.join(ARCH_DIR, "_archive_dirs_%s.zip" % STAMP)
MANIFEST = os.path.join(ARCH_DIR, "MANIFEST_dirs_%s.csv" % STAMP)
REPORT = os.path.join(ROOT, "_tidy_dirs_report_tmp.txt")

APPLY = "--apply" in sys.argv

TO_ARCHIVE = ["_piptest", "_smlink_x", "_reduced", "_reduced2"]
TO_PURGE_EMPTY = ["_figs"]

FO_DELETE = 3
FLAGS = 0x0004 | 0x0010 | 0x0040 | 0x0400 | 0x0200


class S(ctypes.Structure):
    _fields_ = [("hwnd", wintypes.HWND), ("wFunc", wintypes.UINT),
                ("pFrom", wintypes.LPCWSTR), ("pTo", wintypes.LPCWSTR),
                ("fFlags", ctypes.c_uint16),
                ("fAnyOperationsAborted", wintypes.BOOL),
                ("hNameMappings", ctypes.c_void_p),
                ("lpszProgressTitle", wintypes.LPCWSTR)]


def recycle(path):
    """送回收站。rc 不可信，只按后置条件判成败。"""
    p = os.path.abspath(path)
    if not os.path.exists(p):
        return True
    o = S(); o.hwnd = None; o.wFunc = FO_DELETE
    o.pFrom = p + "\0\0"; o.pTo = None; o.fFlags = FLAGS
    ctypes.windll.shell32.SHFileOperationW(ctypes.byref(o))
    return not os.path.exists(p)


def collect(d):
    out = []
    for base, _, fs in os.walk(d):
        for f in fs:
            p = os.path.join(base, f)
            out.append((p, os.path.relpath(p, ROOT), os.path.getsize(p)))
    return out


def main():
    lines = []

    def say(s):
        lines.append(s)
        print(s)

    say("外骨骼 · 第二阶段整理（%s）" % ("APPLY" if APPLY else "DRY-RUN"))
    say("=" * 66)

    plan = {}
    total = 0
    for d in TO_ARCHIVE:
        p = os.path.join(ROOT, d)
        if not os.path.isdir(p):
            say("  %-14s (不存在，跳过)" % d)
            continue
        items = collect(p)
        sz = sum(i[2] for i in items)
        plan[d] = items
        total += sz
        say("  %-14s 文件 %4d  %8.2f MB" % (d, len(items), sz / 1e6))
    say("  合计 %.2f MB" % (total / 1e6))
    say("  空目录直接删: %s" % ", ".join(
        d for d in TO_PURGE_EMPTY if os.path.isdir(os.path.join(ROOT, d))))

    if not APPLY:
        with io.open(REPORT, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        say("DRY-RUN，未改动任何文件。")
        return

    # ---------------------------------------------------------- 打包
    if not os.path.isdir(ARCH_DIR):
        os.makedirs(ARCH_DIR)
    n_expect = 0
    with zipfile.ZipFile(ZIP_PATH, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for d, items in plan.items():
            for full, rel, _ in items:
                z.write(full, arcname=rel.replace("\\", "/"))
                n_expect += 1
    with io.open(MANIFEST, "w", encoding="utf-8", newline="") as fh:
        fh.write("rel_path,bytes\n")
        for d, items in plan.items():
            for full, rel, sz in items:
                fh.write("%s,%d\n" % (rel.replace("\\", "/"), sz))

    # ---------------------------------------------------------- 校验
    with zipfile.ZipFile(ZIP_PATH) as z:
        bad = z.testzip()
        names = {n.replace("\\", "/") for n in z.namelist()}
    want = set()
    for d, items in plan.items():
        for _, rel, _ in items:
            want.add(rel.replace("\\", "/"))
    miss = want - names
    say("")
    say("归档: %s" % ZIP_PATH)
    say("  压缩后 %.2f MB ；条目 %d / 期望 %d ；缺 %d ；完好 %s"
        % (os.path.getsize(ZIP_PATH) / 1e6, len(names), n_expect,
           len(miss), bad is None))
    if bad is not None or miss:
        say("!! 校验未通过 -> 中止，原目录一个都没动")
        with io.open(REPORT, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        return

    # ---------------------------------------------------------- 删除
    say("")
    ok = fail = 0
    for d in plan:
        if recycle(os.path.join(ROOT, d)):
            ok += 1
        else:
            fail += 1
            say("  !! 删不掉: %s" % d)
    for d in TO_PURGE_EMPTY:
        p = os.path.join(ROOT, d)
        if not os.path.isdir(p):
            continue
        if not os.listdir(p):
            if recycle(p):
                ok += 1
                say("  空目录已删: %s" % d)
            else:
                fail += 1
        else:
            say("  跳过（非空）: %s" % d)

    say("")
    say("删除: 成功 %d ；失败 %d" % (ok, fail))
    say("VERDICT: %s" % ("DIRS_TIDY_OK" if fail == 0 else "DIRS_TIDY_PARTIAL"))
    with io.open(REPORT, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
