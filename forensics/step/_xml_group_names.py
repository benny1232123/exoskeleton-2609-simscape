# -*- coding: utf-8 -*-
"""把 _targets3.json 里每个组的 SDR 零件名导成清单，供 SolidWorks 侧勾选对账。

用途：XML 路径要拿 SolidWorks 自己的质量属性做独立上游基准，
      就必须在 SW 里选中 **和 Python 侧完全相同的零件集合**。
      本脚本把那个集合打印出来。
"""
import io, json, os

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, "_targets3.json")
OUT = os.path.join(ROOT, "_xml_group_names.txt")

with io.open(SRC, "r", encoding="utf-8") as f:
    tg = json.load(f)

L = []
L.append("== 每个刚体分组对应的零件清单（来自 _targets3.json）==\n")

for g, spec in tg.items():
    names = [s["name"] for s in spec["sdr"]]
    uniq = sorted(set(names))
    L.append("-- %s --" % g)
    L.append("  零件定义(SDR)数 = %d   唯一零件名 = %d   种子节点 = %d"
             % (len(names), len(uniq), len(spec["seeds"])))
    # 子装配体本身（组名同源的那条）单独标出来
    subs = [n for n in uniq if n == g or n.endswith("设计_左")
            or n.endswith("设计_右") or n.endswith("设计")]
    if subs:
        L.append("  [含子装配体] " + ", ".join(subs))
    L.append("")
    for n in uniq:
        L.append("    " + n)
    L.append("")

L.append("== 说明 ==")
L.append("  * 这些名字来自 STEP 里的 PRODUCT/SHAPE_DEFINITION_REPRESENTATION 名称，")
L.append("    与 SolidWorks 里显示的零部件名基本一一对应（前缀如 腿部_ 电机_ 背部_）。")
L.append("  * leg_L 组里包含 电机_轴 / 电机_出轴 / 电机_黄铜轴套8_10_18 /")
L.append("    电机_片形腿杆连接件 —— 这是**随腿一起转动的电机件**，")
L.append("    对账时不能只选'腿部设计_左'子装配体，否则质量会偏小。")

with io.open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(L))
print("WROTE", OUT)
