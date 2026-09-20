# -*- coding: utf-8 -*-
"""把本轮为「让 PowerShell 回显」而写的抓取残留送回收站（可还原）。"""
import ctypes
import os
from ctypes import wintypes

ROOT = r"C:\Users\29408\Desktop\外骨骼"
FILES = [
    os.path.join(ROOT, "_render_out.txt"),
    os.path.join(ROOT, "_render_exit.txt"),
    os.path.join(ROOT, "_strap_run.txt"),
    os.path.join(ROOT, "_strap_exit.txt"),
    os.path.join(ROOT, "_json_keys.txt"),
    os.path.join(ROOT, "_pf_run.txt"),
    os.path.join(ROOT, "_pf_exit.txt"),
    os.path.join(ROOT, "_sc_run.txt"),
    os.path.join(ROOT, "_sc_exit.txt"),
    os.path.join(ROOT, "_sr_run.txt"),
    os.path.join(ROOT, "_sr_exit.txt"),
    os.path.join(ROOT, "_stpsize.txt"),
    os.path.join(ROOT, "_trash_log.txt"),
    os.path.join(os.environ.get("TEMP", ""), "_k.py"),
    os.path.join(os.environ.get("TEMP", ""), "_sz.py"),
    # 本轮（隐藏滑块）抓取残留
    os.path.join(ROOT, "_gen4.txt"),
    os.path.join(ROOT, "_gen4_rc.txt"),
    os.path.join(ROOT, "_ml3.txt"),
    os.path.join(ROOT, "_ml3_rc.txt"),
    os.path.join(ROOT, "_r5.txt"),
    os.path.join(ROOT, "_r5_rc.txt"),
    # 为辨认用户截图而产生的裁剪图
    os.path.join(ROOT, "_shot_big.png"),
    os.path.join(ROOT, "_shot_L.png"),
    os.path.join(ROOT, "_shot_R.png"),
    os.path.join(ROOT, "_shot_R2.png"),
    os.path.join(ROOT, "_panel_a.png"),
    os.path.join(ROOT, "_panel_a2.png"),
    os.path.join(ROOT, "_panel_a3.png"),
    os.path.join(ROOT, "_panel_a4.png"),
    os.path.join(ROOT, "_panel_a5.png"),
    os.path.join(ROOT, "_red_low.png"),
    os.path.join(ROOT, "_blue_hang.png"),
]

FO_DELETE = 3
FOF_SILENT = 0x0004
FOF_NOCONFIRMATION = 0x0010
FOF_ALLOWUNDO = 0x0040
FOF_NOERRORUI = 0x0400
FOF_NOCONFIRMMKDIR = 0x0200


class SHFILEOPSTRUCTW(ctypes.Structure):
    _fields_ = [
        ("hwnd", wintypes.HWND),
        ("wFunc", wintypes.UINT),
        ("pFrom", wintypes.LPCWSTR),
        ("pTo", wintypes.LPCWSTR),
        ("fFlags", ctypes.c_uint16),
        ("fAnyOperationsAborted", wintypes.BOOL),
        ("hNameMappings", ctypes.c_void_p),
        ("lpszProgressTitle", wintypes.LPCWSTR),
    ]


shell32 = ctypes.windll.shell32
shell32.SHFileOperationW.argtypes = [ctypes.POINTER(SHFILEOPSTRUCTW)]
shell32.SHFileOperationW.restype = ctypes.c_int

ok, skip = 0, 0
for p in FILES:
    if not os.path.exists(p):
        print("SKIP(not exist) %s" % p)
        skip += 1
        continue
    op = SHFILEOPSTRUCTW()
    op.hwnd = None
    op.wFunc = FO_DELETE
    op.pFrom = p + "\0\0"          # 必须双 null 结尾
    op.pTo = None
    op.fFlags = (FOF_SILENT | FOF_NOCONFIRMATION | FOF_ALLOWUNDO
                 | FOF_NOERRORUI | FOF_NOCONFIRMMKDIR)
    op.fAnyOperationsAborted = False
    rc = shell32.SHFileOperationW(ctypes.byref(op))
    good = (rc == 0 and not op.fAnyOperationsAborted)
    print("%-8s %s" % ("OK" if good else "FAIL", p))
    ok += 1 if good else 0

print("")
print("RESULT: OK=%d  FAIL=%d  SKIP=%d" % (ok, len(FILES) - ok - skip, skip))
print("VERIFY: 仍存在 =", [os.path.basename(p) for p in FILES if os.path.exists(p)])
