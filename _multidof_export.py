# -*- coding: utf-8 -*-
"""
_multidof_export.py —— 把 exo_multidof.urdf 的刚体参数导出成 JSON，供 MATLAB 侧组装。

为什么不让 MATLAB 自己读 URDF
-----------------------------
可以，但没必要：MATLAB 解析 XML 要么走 `xmlread`（Java DOM，慢），
要么自己写 regexp（易错）。而本工程**既有**的先例就是
「Python 从 URDF 派生 → 生成机器可读文件 → MATLAB 读」
（见 `_stp2plant.py` → `plant_params.csv` → `compare_simscape_vs_analytical.m`）。
沿用同一个模式，MATLAB 侧就只需要实现「组装算法」，不重复实现「解析」。

⚠ 注意分工：本文件只导出 **URDF 里已有的量**（质量/质心/惯量/关节几何），
   **不导出任何动力学结果**（M(q)/G(q) 一律不出现）。这样 MATLAB 侧
   是从头算力学的 —— 两侧独立，吻合才有意义。

输出 matlab2609/simscape/plant_multidof.json
"""
import io
import json
import os
import sys

ROOT = r"C:\Users\29408\Desktop\外骨骼"
sys.path.insert(0, ROOT)
from _multidof_dyn import MultiBody          # noqa: E402

SIM = os.path.join(ROOT, "matlab2609", "simscape")
URDF = os.path.join(SIM, "exo_multidof.urdf")
OUT = os.path.join(SIM, "plant_multidof.json")


def main():
    mb = MultiBody(URDF)
    obj = {
        "source": os.path.basename(URDF),
        "note": "由 _multidof_export.py 从 URDF 派生；只含 URDF 已有的刚体/关节参数，"
                "不含任何动力学结果（M/G 由 MATLAB 侧自行组装）",
        "gravity": mb.g,
        "nq": mb.nq,
        "q_names": list(mb.q_names),
        "links": [
            {"name": l,
             "mass": mb.link[l]["mass"],
             "cog": [float(x) for x in mb.link[l]["cog"]],
             "I": [[float(x) for x in row] for row in mb.link[l]["I"]]}
            for l in mb.links
        ],
        "joints": [
            {"name": j["name"],
             "type": j["type"],
             "parent": j["parent"],
             "child": j["child"],
             "o": [float(x) for x in j["o"]],
             "axis": None if j["axis"] is None else [float(x) for x in j["axis"]],
             "movable": bool(j["movable"]),
             "qidx": -1 if j["qidx"] is None else int(j["qidx"])}
            for j in mb.joints
        ],
        # 拓扑顺序（MATLAB 侧按它遍历即可保证父先于子）
        "order": [j["name"] for j in mb._order],
    }
    io.open(OUT, "w", encoding="utf-8").write(json.dumps(obj, indent=1, ensure_ascii=False))
    print("WROTE %s" % OUT)
    print("  nq = %d   q_names = %s" % (obj["nq"], obj["q_names"]))
    print("  links = %d, joints = %d" % (len(obj["links"]), len(obj["joints"])))
    print("  order = %s" % obj["order"])


if __name__ == "__main__":
    main()
