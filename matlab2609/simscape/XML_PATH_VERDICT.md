# XML 路径（路径 A）终局结论 · 2026-09-19 13:0x

> 本文是 `XML_PATH_GUIDE.md` 的**实测判决书**。指南里若干结论已被本轮实测
> 推翻或升级，**冲突时以本文为准**。全部结论都附可复现的证据与脚本名。

---

## TL;DR —— 三个判决

| # | 判决 | 影响 |
|---|---|---|
| 1 | ✅ **XML 路径端到端跑通**：合法 XML → `smimport` → 75 块模型 / 35 个 `File Solid`，19 件质量属性与 SW 内核**逐位一致**（mass 2.9e-12） | 路径 A 作为**外部基准**成立 |
| 2 | ❌ **0 关节不是 bug，是模型本身没有配合关系。** 路径 A **拿不到运动学模型** | 路径 A 的定位要改（见 §7） |
| 3 | ⚠️ 「**改配置名为 ASCII**」这条建议**降级**：根因已确认，但**不用动你的 CAD 文件** —— 已用离线修复替代 | 省掉一次全模型改名 + 重导 |

---

## 1. 0 关节的根因：装配体本来就没有配合（硬证据）

用 SolidWorks COM（`_sw_mates.py` / `_sw_mates2.py`，只读）实测：

| 装配体 | 零部件数 | `IsFixed == True` | `grounded="false"` | XML 关节数 |
|---|---|---|---|---|
| `腿部设计_左.SLDASM` | 35 | **35 / 35** | 0 | 0 |
| `"林-Ⅰ"…V0_1_1.SLDASM`（顶层） | 5 | **5 / 5** | — | — |

**为什么 `IsFixed` 全 True 就等于「没有配合」**：
SolidWorks 里 `IComponent2::IsFixed` 只在零部件被**显式固定**时返回 True；
被配合约束好的零部件是「完全定义/浮动」，`IsFixed` 返回 **False**。
35/35 全部 True ⇒ 所有零件都是"钉在世界系里"，**没有任何一个由配合定位**。

⇒ 插件导出 `<Constraints></Constraints>` 空 + 35 个 `Instance` 全 `grounded="true"`，
是**如实反映**，不是插件缺陷。

**10 秒自查法**（想自己确认）：打开 `腿部设计_左.SLDASM`，
看 FeatureManager 里有没有「**配合**」文件夹、各零部件名后面是否带「**(固定)**」。

> 这一判决与项目自己预留的口径吻合 —— `smimport_from_xml.m` 里早就写着
> `XML_IMPORT_NO_JOINTS`：「装配体里没有配合关系 —— 那是 STEP 导入的『哑实体』，
> 不是原生装配体」。

---

## 2. 垃圾文件名 / 丢件 的根因（**推翻了上一轮假设**）

上一轮的推断是「SW 的配置名本身是坏的（`默认b` / `默认g` / `默认8e-14`）」。
**这是错的** —— 实测 `_sw_cfg_meta.py`：

```
SolidWorks 里开了 20 个文档，逐个读配置：
  腿部_绑缚 / 腿部_轴盖 / 腿部_滑轨_片形 / 腿部_腿杆_片状V5 / 电机_出轴 / 电机_轴 ...
  cfg.Name          = 默认        (UTF-8 e9bb98 e8aea4，干净)
  cfg.AlternateName = ''          (空)
  cfg.Comment       = ''          (空)
  cfg.Description   = 默认
  cfg.IsDerived     = False
  配置自定义属性     = 0 条
```

**「脏件」和「干净件」的元数据完全一模一样**（逐字段对比，见 `_sw_cfg_meta.txt`）。

⇒ 那段垃圾后缀**不是 SW 数据里的任何字段**，而是 **插件把中文配置名转 ANSI 时的
缓冲区 bug**：`默认` 是 2 个 UTF-16 码元 → 按 2 个字符分配缓冲，但 GBK 需要 4 字节
→ 溢出 2 字节 → 后面残留的内存垃圾被当成名字的一部分。

⇒ 所以「改 ASCII 配置名」**理论上确实能修**，但：
- 要改 19 个零件 + 全部引用它的装配体，
- 需要你重新导出一次，
- 而**它挡不住的真正问题（0 关节）改完也还是在**。

**本轮改为离线修复：一个字都不动你的 CAD 文件**（见 §3）。

---

## 3. 离线修复方案（已执行，可复现）

`_xml_fix.py` + `_sw_export_step.py`，三步：

| 步 | 做什么 | 怎么做 |
|---|---|---|
| 1 | 补齐 2 个**丢件** | `腿部_腿杆_片状V5` / `腿部_轴盖` 从 SW 直接导出（COM `SaveAs4`） |
| 2 | 19 个 `GeometryFile` 名归一化 | 全部改成 `<零件名>_Default_sldprt.STEP`，磁盘上同步改名 |
| 3 | 重写 XML | 按**文档序**逐位替换 `GeometryFile` 的 `name` 属性 |

**结果**（`_xml_fix.txt`）：

```
磁盘 STEP 数 = 19          就位 19 / 19
raw 里 GeometryFile name 命中 = 19（期望 19）
XML 解析 : OK    XML 禁止控制字符 = 0
GeometryFile 引用 19 , 磁盘缺失 0
VERDICT: xml_valid=True  step_refs_missing=0  parts=19
```

**安全前提**：动手前已把整个 `装配体\` 目录备份到
`C:\Users\29408\exo_work\_cad_backup_20260919\`（**1520 个文件 / 139.15 MB**）。

### 3.1 SolidWorks COM 自动化的三个坑（写脚本必踩）

| 坑 | 现象 | 正解 |
|---|---|---|
| **晚期绑定下属性当方法调** | `d.GetTitle()` → `TypeError: 'str' object is not callable` | 写个 `getv(o,name,*a)`：`callable` 才调用。**SW 里一半 API 是属性** |
| **`GetFirstDocument` / `GetMates` / `GetTypeName2` 找不到成员** | `com_error -2147352573 找不到成员` | 这些名字不在 IDispatch 名表里。改用 `sw.GetDocuments`（**属性**）遍历；`GetComponents` 可用 |
| **`SaveAs4` 的 `Errors/Warnings` 是 by-ref** | `com_error -2147352571 类型不匹配` | 必须传 `win32com.client.VARIANT(pythoncom.VT_BYREF|VT_I4, 0)` |

**改配置名的正解 API**（虽然这次没用上，留着备用）：
`EditConfiguration3(Name, NewName, Comment, AlternateName, Options) -> bool`
（旧版 `EditConfiguration` 有 9 个参数，已 obsolete）。

---

## 4. XML → smimport 全链路验证（**路径 A 的核心成果**）

```
smimport_from_xml('D:\exo_xml_a\legL_xml.xml','ModelName','sm_exo_legL_xml')
```

**模型结构**（`probe_xmlmodel.m` 体检）：

| MaskType | 数量 | 说明 |
|---|---|---|
| `File Solid` | **35** | 35 个实例，每个一个刚体 |
| `Rigid Transform` | 35 | 实例在装配体里的位姿 |
| `World Frame` / `Mechanism Configuration` / `Solver Configuration` | 1 / 1 / 1 | |
| 关节 | **0** | 见 §1 |
| 合计块数 | 75 | |

**`File Solid` 是怎么拿到质量的**（`probe_xmlmodel2.m`）：

```
ExtGeomFileName  = 电机_片形腿杆连接件_Default_sldprt.STEP   ← 我归一化后的名字被正确引用
InertiaType      = Custom                                   ← 用 XML 的质量属性，不是按几何反算
Mass             = smiData.Solid(1).mass
CenterOfMass     = smiData.Solid(1).CoM
MomentsOfInertia = smiData.Solid(1).MoI
ProductsOfInertia= smiData.Solid(1).PoI
CenterOfMassUnits     = mm
MomentsOfInertiaUnits = kg*mm^2
```

**单位链**：XML 声明 `ModelUnits: kg/mm`、`DataUnits: kg/m`
→ 即 `Mass` 用 kg、`CenterOfMass` 用 **m**、`Inertia` 用 **kg·m²**；
而块用 mm / kg·mm²，`smimport` **自动换算正确**（逐件验证过）。

**逐件对账**（`_sw_ml_xcheck.py`，smiData ↔ SW 内核基准）：

```
SolidWorks 基准条目 = 19     smiData 条目 = 19
逐件全项一致 = 19 / 19
最大偏差: mass rel 2.86e-12 | CoM abs 4.69e-10 mm | MoI rel 3.61e-12
Σ SW mass(19 唯一件) = 0.099084258 kg
Σ ML mass(19 条)     = 0.099084258 kg
VERDICT: SW_ML_XCHECK_OK
```

### 4.1 顺带修掉项目自己的一个**假阴性**

`smimport_from_xml.m` 的 `dump_bodies` 原来只认「有 `Mass` 对话框参数且能
`str2double` 成数字」的块。而 XML 路造的 `File Solid` 里
`Mass` 是**表达式字符串** `smiData.Solid(k).mass` → `str2double` 给 NaN
→ 整块被当"质量 0"滤掉 → 明明摆着 35 个刚体却报 `XML_IMPORT_NO_BODIES`。

已修：对表达式参数回落到 `smiData` 查表（新增局部函数 `smi_lookup` / `pad3`）。
修后判决正确变成 **`XML_IMPORT_NO_JOINTS`，刚体 35**。

### 4.2 几何路径必须绝对化（否则仿真必挂）

`smimport` 由 XML 导入时，`ExtGeomFileName` 只写**裸文件名**；而 `.slx` **没有内嵌几何**
（391.9 KB，STEP 合计约 7 MB）。所以除非 MATLAB 的 pwd 就在 `D:\exo_xml`，一编译就找不到文件。

**新工具 `absolutize_solid_paths.m`**：

```matlab
r = smimport_from_xml('D:\exo_xml_a\legL_xml.xml','ModelName','sm_exo_legL_xml');
n = absolutize_solid_paths('sm_exo_legL_xml','D:\exo_xml');   % 实测改写 35/35
save_system('sm_exo_legL_xml');
```

---

## 5. 新发现：19 件 STEP 与 SW 内核的独立几何复核

`_step_check_all.py`（gmsh/OpenCASCADE 读 STEP，vs XML 声明值；
多实体合成、惯量矩阵用平行轴定理移到质心后比较）：

| 档 | 件数 | 偏差量级 | 零件 |
|---|---|---|---|
| **精确** | 9 | ≤ 1e-10 | 黄铜轴套、各螺丝/顶丝、平头螺丝、法兰注塑螺母、滑轨挡片 |
| **同量级可接受** | 7 | 1e-06 ~ 5.3e-05 | 电机_出轴 1.8e-6、电机_片形腿杆连接件 6.5e-6、电机_轴 1.1e-6、腿部_按键_双 4.6e-6、腿部_滑块_双键 2.1e-6、腿部_滑轨_片形 1.5e-5、腿部_绑缚 5.3e-5 |
| **异常** | 3 | 0.92% / 5.8% / 5.8% | `腿部_腿杆_片状V5` **0.92%**、`腿部设计_左-5` **5.8%**、`腿部设计_左-6` **5.8%** |

- 中间那档的两个代表值（`滑块_双键` **2.06e-06**、`绑缚` **5.32e-05**）
  与上一轮三方探针的 **A~B** 值**完全一致** ⇒ **本次改名/补齐没有引入任何误差**，
  这些偏差是 Parasolid(SW) 与 OpenCASCADE 两个内核的固有差异。
- ★ **新线索**：`腿部设计_左-5 / -6` 是 **SW 内核与自己导出的 B-rep 都不一致**，
  差 5.8%（20.33 vs 21.51 mm³）。这两件各有 **752 KB 的 STEP 文件**却只有
  **20 mm³ 体积** —— 形状/体积严重不匹配（上一轮已标为可疑件）。虽然各只有 20 mg
  对总量无影响，但**值得单独看一眼**（是面体？还是超薄件？）。

---

## 6. 当前状态总表

| # | 判据 | 状态 | 证据 |
|---|---|---|---|
| 1 | XML 合法性 | ✅ 0 控制字符、可解析 | `_xml_fix.txt` |
| 2 | 关节数 | ⛔ **0（模型本身无配合，路径 A 的硬边界）** | `_sw_mates.txt` |
| 3 | 材质 / 真密度 | ❌ **19 件全 = 1.0000 g/cm³**（未赋材质，SW 默认密度） | `_sw_baseline.txt` |
| 4 | 几何引用 | ✅ 19 / 19 命中 | `_xml_fix.txt` |
| 5 | STEP 数 == Part 数 | ✅ **19 == 19** | `_xml_fix.txt` |
| 6 | XML → smimport | ✅ 75 块 / 35 刚体 / 质量属性 19/19 一致 | `sm_exo_legL_xml_log.txt` |
| 7 | STEP vs SW 内核几何 | ⚠️ 16/19 一致，3 件异常 | `_step_check_all.txt` |

**判据 3 的含义**（重要，别误读）：SW 对没赋材质的零件用**默认密度 1000 kg/m³**，
照样输出非零质量。所以这份 XML 的**绝对质量不能当质量基准**；
但 `CenterOfMass` / `Inertia` 是**密度无关的几何量**，可以直接用：

| SW 输出 | XML 单位 | 还原成几何量 |
|---|---|---|
| `Mass` | kg | 体积 [mm³] = `Mass × 1e6` |
| `CenterOfMass` | m | CoM [mm] = `× 1000` |
| `Inertia` | kg·m² | ∫r²dV [mm⁵] = `× 1e12`（与 `exo2609/geometry.py` 的 `inertia_local` 同约定） |

---

## 7. 路径 A 的定位要改（结论）

**原来期望**：XML 路提供运动学模型（关节）＋ 质量属性。
**实测结论**：**关节拿不到**（模型无配合），**质量属性是假的**（无材质）。

⇒ **路径 A 剩下的真实价值 = 一条独立上游的几何/惯量外部基准**：
用 SolidWorks 内核的数字，去交叉验证我们自己 Python STEP 解析链（`exo2609/geometry.py`）
的体积 / 质心 / 惯量。这正是项目里 L1/L2/L2′ 三条证据一直缺的东西
（那三条**全部同源**于我们自己的解析器，同源会同错）。

**运动学仍然走 URDF 路**（`exo_real.urdf` / `sm_exo_real.slx` / `exo2dof.urdf`），
那条路的关节是手工定义+我们的轴拟合，不依赖 mates。

---

## 8. 剩余待办（按优先级）

1. **★ 材料（唯一还能提升「基准」价值的一项）**
   先做**单件实验**：只给 1 个零件赋一个密度鲜明的材料（钢 7850）→ 重导 →
   看该件质量是否变成 **7.85 倍**。
   - 变了 → 插件确实吃 SW 密度 → 批量赋值有意义；
   - 没变 → **别再批量赋值**，密度在外部表里处理。
   （真实密度最终只能靠**称重**或 BOM。）
2. **查 3 个几何异常件**：`腿部_腿杆_片状V5`、`腿部设计_左-5/-6`。
3. ~~改配置名为 ASCII~~ —— **降级**：只在你打算长期频繁导出时才值得做；
   单次用途已被 §3 的离线修复覆盖。
4. Step 2 零成本体积对账（照 `_xml_group_names.txt`）。
5. **L3（真机 q, q̇）** 仍未做。

---

## 9. 本轮文件清单

### 新增（`matlab2609/simscape/`）
| 文件 | 作用 |
|---|---|
| `absolutize_solid_paths.m` | 把 `File Solid` 的几何路径批量改成绝对路径（**必需**，否则仿真找不到 STEP） |
| `probe_xmlmodel.m` / `probe_xmlmodel2.m` / `probe_xmlmodel3.m` | XML 导入模型的体检：块类型分布 / `File Solid` 参数 / `smiData` dump |
| `smiData_dump.csv` | smimport 导入进来的 19 条质量属性（供 Python 对账） |
| `sw_baseline_legL.csv` | **SolidWorks 内核外部基准表**（19 件，量纲写进列名） |

### 修改
| 文件 | 改动 |
|---|---|
| `smimport_from_xml.m` | **bug fix**：`dump_bodies` 支持 `File Solid` 的表达式参数（`smiData`），新增 `smi_lookup` / `pad3` |
| `XML_PATH_GUIDE.md` | 顶部加指向本文的警告 |
| skill `cad-to-simscape-multibody` | 同步三条判决 ✅ **已完成**：①「改配置名」降级为「插件 ANSI buffer 溢出、优先离线修 XML」；②新增 `IsFixed` 全 True = 无 mates 的 0 关节根因实证；③新增 XML 路端到端验通判据表 + `smiData` 回落 + `absolutize_solid_paths`；并新增「离线修复 SOP」「SW COM 自动化坑」两节 + description 扩展 + 验收清单「XML 路专项」 |

### 工作脚本（`C:\Users\29408\exo_work\`）
`_sw_cfgs.py` / `_sw_cfgs2.py` / `_sw_cfgs3.py`（列配置名）、`_sw_cfg_meta.py`（配置元数据）、
`_sw_mates.py` / `_sw_mates2.py`（配合关系）、`_sw_export_step2.py`（补导 STEP）、
`_xml_fix.py`（修复 XML）、`_sw_baseline.py`（基准表）、`_sw_ml_xcheck.py`（对账）、
`_step_check_all.py`（STEP↔内核几何复核）

### 数据产物
- `D:\exo_xml\腿部设计_左.fixed.xml` —— **合法可导入的 XML**（19 件）
- `D:\exo_xml\legL_xml.xml` —— ASCII 名副本（给 MATLAB 用）
- `D:\exo_xml\*_Default_sldprt.STEP` × 19 —— 归一化后的几何
- `D:\exo_xml_a` —— 指向 `D:\exo_xml` 的 ASCII junction
- `matlab2609\simscape\sm_exo_legL_xml.slx` —— **导入成功的 Simscape Multibody 模型**
- `C:\Users\29408\exo_work\_cad_backup_20260919\` —— CAD 全量备份（1520 文件 / 139 MB）

---

## 10. 复现命令

```matlab
cd C:\Users\29408\Desktop\外骨骼\matlab2609\simscape
addpath(pwd)

% 1) 导入 + 几何路径绝对化 + 存盘
r = smimport_from_xml('D:\exo_xml_a\legL_xml.xml','ModelName','sm_exo_legL_xml');
n = absolutize_solid_paths('sm_exo_legL_xml','D:\exo_xml');
save_system('sm_exo_legL_xml');

% 2) 模型体检
probe_xmlmodel            % 块类型分布
probe_xmlmodel2           % File Solid 参数 + 几何文件名
probe_xmlmodel3           % smiData 数值 dump
```

```bat
:: 3) Python 侧对账 / 复核
E:\Anaconda\python.exe C:\Users\29408\exo_work\_sw_baseline.py       :: 重建 SW 基准表
E:\Anaconda\python.exe C:\Users\29408\exo_work\_sw_ml_xcheck.py      :: smiData vs SW 基准
E:\Anaconda\python.exe C:\Users\29408\exo_work\_step_check_all.py    :: STEP vs SW 内核几何
```
