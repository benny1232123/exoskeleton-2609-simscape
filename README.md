# 髋关节外骨骼 · 论文复现 + CAD → Simscape Multibody 多体动力学

复现 **arXiv:2609.15352**（髋关节外骨骼助力）的核心链路——「动力学模型 → 滑窗力矩估计（Eq.6）」
——并把它从论文的 **2-DOF 简化模型**扩展成**由 CAD 直接建立的多体模型**（MATLAB/Simulink
**Simscape Multibody**），在 **MATLAB** 与 **Python** 两条独立实现上交叉验证。

> 本仓库只含**代码 + 文档 + 报告**，不含 CAD 源文件与大二进制（STEP / STL / SLDPRT / .slx / 图）。
> CAD 侧只保留「怎么读它」的脚本与结论；原始装配体由 SolidWorks 导出为 STEP 后离线处理。

---

## 一、结果速览

### 1.1 质量属性（本 CAD 口径 vs 论文 Table II）

| 量 | 本 CAD | 论文 | 比值 |
|---|---|---|---|
| 单腿摆动件质量 `m` | **0.312589452 kg** | — | — |
| 单腿绕髋轴惯量 `I_axis` | **0.016997395 kg·m²** | 0.0156 | **1.090 ×** |
| 重力矩幅值 `G_amp` | **0.558396084 N·m** | 0.879 | **0.635 ×** |
| 重力矩分解 | `τ_g = −0.341436830·sin q − 0.441845084·cos q` | `0.879·sin q` | 零位不在自然下垂 ⇒ `cos` 项非零 |
| 整机质量（4-DOF 全部 link 求和） | **3.329496 kg** | — | — |

### 1.2 验证判据（全部通过）

| 判据 | 命令 / 脚本 | 结果 |
|---|---|---|
| 2-DOF：解析 vs Simscape 逐通道 | `matlab2609/simscape/compare_simscape_vs_analytical.m` | **`COMPARE_OK`** |
| 4-DOF：解析 vs Simscape（4 角 + 4 角速度通道） | `_multidof_compare.py` | **`COMPARE_MULTIDOF_OK`**，worst `rms/range` = **1.94e-06**（阈值 1e-2） |
| 4-DOF 拆段回归（`real` vs `_locked`） | `matlab2609/simscape/smoke_multidof.m` | `max\|Δx\|` = **1.023e-08** → **PASS** |
| 4-DOF 解析侧自检（判据 A/B/C） | `_multidof_dyn.py` | `VERDICT: ALL_OK` |
| 一键重跑（密度一改即全链重放） | `matlab2609/simscape/rerun_after_density.m` | **`RERUN_ALL_OK`** |

> 4-DOF 对照里 `abduct` 摆幅 **0.407 / 0.428 rad**（≈0 则说明退化成 2-DOF、对照没意义）——
> 说明 4 个自由度都真实被激励，对照有效。

---

## 二、数据血缘：两条**互不依赖**的链路

| 链路 | 用途 | 数据流 |
|---|---|---|
| **A · 动力学（无 STL）** | 质量 / 惯量 / 多体动力学 | STEP 实体 B-rep → gmsh/OpenCASCADE 对 `dim=3` 实体做体积分 → 体积/质心/惯量（ρ=1）→ ×密度 → `<inertial>` → URDF → `plant_params.csv` |
| **B · 三维外形（这才是 STL）** | Mechanics Explorer 显示 | STEP → 表面三角化 → `meshes/*.stl` → `<visual>` |

**★ 关键事实：动力学数据全部来自 STEP 的实体 B-rep，STL 只用于显示。**
STL 是三角面片、无体积无惯量，质量必须对**闭合实体**做体积积分。因此 `<visual>` 的有无
**不影响任何动力学结果**。

关节轴同理不是"看"出来的：圆柱面轴线用 `_cyl_faces.json` 拟合（`电机_出轴` r=22 mm，
残差 0.0000 mm），移动副用 `_plane_faces.json`（PLANE 法向 + 足迹）判定。

### ★ CAD 拿不到关节信息

STEP 的 `<Constraints>` 为空、实例全 grounded；原生装配体 35/35 零件的 `IsFixed` 为真，
`GetMates` 抛 `AttributeError`。⇒ **关节自由度只能靠几何反推**（圆柱面共轴、面对面的对齐关系）。

---

## 三、自由度口径（两套并存，别混）

| 口径 | 模型 | 说明 |
|---|---|---|
| **2-DOF（论文口径）** | `exo2609/`、`matlab2609/+exo2609/` | 左右髋屈伸各 1 自由度；严格按论文。**不要动** |
| **4-DOF（扩展）** | `exo_multidof.urdf` → `sm_exo_multidof.slx` | 每腿 3 段：`hip`(revolute) → `abduct`(revolute，⊥髋轴，距髋 52 mm) → `slide`(按 fixed 处理) |

- `abduct` 是**真自由度**（"向内旋转"），轴不平行任何全局轴且随之随 `hip` 转动。
- `slide` 确为移动副（滑块↔导轨 4 组面对面、法向 ⊥ 导程），但**行程仅 2 mm 装配间隙** ⇒ 按 fixed 忽略。
  所以是 **4 DOF**，不是 6。
- 每腿**只有 1 个电机** ⇒ 额外的自由度只能是被动的（自由 / 弹簧）。

### 4-DOF 解析侧为什么不能沿用 2-DOF 的解耦式

2-DOF 里每个关节解耦，可用 `I·q̈ = T + A·sin q + B·cos q`。到 4-DOF **不成立**：
`abduct` 轴不平行全局轴且固定随 `hip` 转 ⇒ 质量矩阵 `M(q)` 非对角。必须解完整形式：

```
M(q)·q̈ + C(q,q̇)·q̇ + G(q) = Q_ext
```

（沿用解耦式粗估会差 ~53%。）

---

## 四、★ 2026-09-20 密度修正（本仓库的关键更正）

### 为什么"密度是唯一软假设"

CAD 里所有零件**都没有赋材质**（SolidWorks `MaterialIdName` 全为空，
密度字段精确 = 1000.0）。体积已被 SW 内核独立核对过（19/19 几何配对，
rel diff −0.055%），⇒ **几何可信，密度是唯一未知量**。
密度表是「精确名覆盖 → 关键字查表 → 兜底 2700 铝」三层。

### 修正内容

`电机_轴` 原**静默落到兜底铝 2700**（关键字表里有"出轴""轴套"却没有裸"轴"），
而同一台电机的 `电机_出轴` 吃钢 7850 ⇒ 两根轴两种材料，**自相矛盾**。
按功能反推（Ø8×32 基本实心轴、且是 Ø8 内孔黄铜轴套的配合轴）改判**钢 7850**。

| 量 | 改前 | 改后 | Δ |
|---|---|---|---|
| 单腿 `m` | 0.304833 kg | **0.312589452 kg** | **+2.545%** |
| 单腿 `I_axis` | 0.016975774 | **0.016997395** | **+0.127%** |
| `\|τ_g\|` | 0.554528 N·m | **0.558396084** | **+0.697%** |
| 整机总质量 | 3.313983 kg | **3.329496 kg** | **+0.468%** |
| 兜底覆盖 | 9/19 件 · 54.87% | **8/19 件 · 52.13%** | −1 件 |

> **传播比值得注意：质量不确定度 ≠ 动力学不确定度。** 出问题的件在轴上 / 近轴处
> ⇒ 质量改了 +2.5%，而 `M` 只动 +0.13%、`G_amp` 只动 +0.70%。
> 反过来，**远离轴的薄长件**（313 mm 腿杆、平均厚仅 1.19 mm）才是 `I_axis` 的真风险源。

### 改一个输入数字之后的**连锁重跑**（最容易漏 Simscape 那条）

`smimport` 把 URDF 的数值**直接烘进 .slx 块参数**（URDF 输入不产生参数数据文件）
⇒ 只重生 URDF 不够，**必须重新 `smimport`**，否则 Simscape 用旧惯量、解析侧用新 CSV，
两侧系统性错开。完整顺序（已脚本化）：

```
密度表 → exo_real.urdf → smimport → harness → compare_simscape_vs_analytical
       → plant_params.csv
       → exo_multidof[_locked].urdf → smimport → smoke(回归) → dump 轨迹
       → quick_multidof_check → test_multidof_analytical
```

一键驱动：`matlab2609/simscape/rerun_after_density.m` → `RERUN_ALL_OK`。

---

## 五、目录结构

```
exo2609/                     Python 侧：论文 2-DOF 模型 + 几何/密度 + 滑窗力矩估计
  geometry.py                ★ 密度表真源（PART_EXACT → 关键字 → 兜底）
  dynamics.py, torque_opt.py, params.py, gait.py
  tests/                     pytest

matlab2609/                  MATLAB 侧
  +exo2609/                  Eq.1/4/5 动力学 + Eq.6 滑窗力矩估计（estimate.m）
  README.md                  ★ 权威说明（实现口径 / 参数 / 判据）
  MULTIDOF_DYNAMICS_GUIDE.md 4-DOF 解析模型用法与判据
  simscape/                  Simscape Multibody + 全部 SOP / 取证文档
    import_*.m, build_harness_*.m, compare_*.m, smoke_multidof.m, rerun_after_density.m
    plant_params.csv         ★ 派生参数真源（m/I/A/B）
    plant_multidof.json, multidof_baseline.json
    *.urdf                   exo_real / exo2dof / exo_multidof[_locked]
    MATERIAL_FUNCTIONAL_INFERENCE.md   ★ 材质按功能反推（19 件全表 + 修正落地表）
    MATERIAL_AND_MASS_RECON.md, XML_PATH_GUIDE.md, XML_PATH_VERDICT.md
    SIMSCAPE_BRIDGE_SOP.md, MULTIDOF_3D_HOWTO.md, VISUAL_ANIM_GUIDE.md
  solidworks/                质量预算工具（mass_budget.py / weighing_sheet.py）

dynamics_model/              早期探索（STEP 解析、双髋动力学、MPC demo）

_stp2urdf.py / _stp2urdf_multidof.py / _stp2stl.py / _stp2plant.py   生成链
_multidof_dyn.py             Python 4-DOF 拉格朗日组装（与 MATLAB 独立实现）
_multidof_export.py          → plant_multidof.json
_multidof_compare.py         4-DOF 判据 D：解析 vs Simscape
_report_2609.py              生成 2609_report.html
_axis_find.py / _plane_faces.py / _joint_candidates.py   关节轴 / 移动副取证
_tidy_root.py / _trash_scratch.py                        仓库整理与回收站工具

2609_report.html             结果报告（可读版）
2609_report.txt / 2609_text.txt
```

---

## 六、复现步骤

### 0) 依赖

- Python 3.12/3.13 + `numpy` / `scipy` / `matplotlib` / **`gmsh`**（OpenCASCADE）
- MATLAB R2024b + **Simulink + Simscape Multibody**

### 1) 只跑 Python 侧（不需要 MATLAB）

```bash
python exo2609/geometry.py          # CAD 质量属性 + 髋轴识别（需要 _reduced3/*.stp）
python _report_2609.py              # 生成 2609_report.html / .txt
python -m pytest exo2609/tests      # 论文 2-DOF 模型单测
```

### 2) 论文 2-DOF 链路

```matlab
cd matlab2609
demo2609_run                        % 正动力学 / 逆问题(Eq.6) / 判据
```

### 3) 4-DOF 多体链路（Simscape）

```matlab
cd matlab2609/simscape              % ★ 工作目录必须是这里（STL 是相对路径）
run_remultidof                      % 或逐个：
import_exo_multidof; smoke_multidof; quick_multidof_check; test_multidof_analytical
rerun_after_density                 % 一键重放全链并核对全部 verdict
```

> ⚠ 3D 视窗（Mechanics Explorer）只在**模型更新或仿真**时创建：
> `open_system` 只弹框图，要看到 3D 得点 **Run** 或按 **Ctrl+D**。详见 `MULTIDOF_3D_HOWTO.md`。

---

## 七、方法学备忘（踩过的坑，已固化为纪律）

1. **判据脚本失败不抛异常**（`compare_*` / `smoke_*` / `quick_*` / `test_*` 只把 FAIL 写进自己的日志）
   ⇒ 驱动脚本必须**回读日志判 verdict**，且 pass/fail token 不可互为前缀
   （`COMPARE_` 是 `COMPARE_OK` 的前缀，会把 PASS 误判成 FAIL）。
2. **URDF `<limit>` 在 Simscape 里是 1e4 N·m/deg 硬弹簧**。拿"原样导入模型"做自由摆动对照时，
   限位会盖住动力学 ⇒ 必须先关硬限位 + 统一重力，否则得到**假 FAIL**
   （实测关掉前 1.68e-03，关掉后 1.02e-08）。
3. **派生数被硬编码的地方都会漂**：测试基准、对账常量、动画/HUD 参数、文档值……
   一律改成**读机器可读产物**（CSV/JSON），并在回退时显式打印 `HARDCODED FALLBACK`。
4. **与结论绑定的叙述性文字也会漂**：改了数值后，脚本 docstring / 打印提示里
   由旧值推出的论断往往还在讲旧故事（本次靠 `grep 结论词` 抓出过一句明确写错的话）。

---

## 八、参考

- 论文：*arXiv:2609.15352*（髋关节外骨骼助力 / 生物力矩控制器）
- Camargo et al. 2021, *A comprehensive, open-source dataset of lower limb biomechanics*
- 相关仓库：[`gait_data`](https://github.com/benny1232123/gait_data)（生物力矩控制器复现，另一条线）

---

_生成：2026-09-20 · 密度修正后（`电机_轴` → 钢 7850）全链重跑通过。_
