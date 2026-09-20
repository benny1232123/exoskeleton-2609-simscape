# -*- coding: utf-8 -*-
"""让测试可以直接 import exo2609（把工作区根目录加入 sys.path）。"""
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[2]   # C:\Users\29408\Desktop\外骨骼
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
