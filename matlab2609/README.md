# matlab2609 —— 论文 2609 动力学模型（SolidWorks + MATLAB 实现）

论文：**Assistance Torque Estimation via Dynamics-Aware Optimization for Lower-Limb
Exoskeleton in Complex Environments**（arXiv:2609.15352v1）
复现范围：**严格按论文** —— 左右髋 2-DOF 刚体动力学 + Eq.(6) 滑窗式助力力矩估计。
几何基线：本机 `“林-Ⅰ”髋关节外骨骼机器人开发平台V0_1_1.stp`。

---

## 1. 快速开始

```matlab
cd matlab2609
addpath(pwd)

check_gradient2609    % ① 梯度校验：解析伴随梯度 vs 中心差分      -> 应 PASS
selftest2609          % ② 自检：CSV->M/G_amp、谱分解、离散化       -> 应 ALL PASS
demo2609_run          % ③ 端到端：正向仿真 + Eq.(6) 反解 + 出图   -> out/*.png

% ④ SolidWorks -> Simscape Multibody 多体链路（**真实装配体**，CAD 侧）
addpath(fullfile(pwd,'simscape'))
import_exo2dof            %  exo_real.urdf -> sm_exo_real.slx          （真实装配体导入）
attach_visual_meshes      %  核验 <visual> 的 .stl 真进模型 + 路径改 ASCII 绝对路径
build_harness_exo2dof     %  + 力矩驱动/角度测量 -> sm_exo_real_harness.slx
compare_simscape_vs_analytical   %  多体 vs 解析模型交叉验证 -> 应 COMPARE_OK
validate_energy(3)        %  第三层：自由摆动能量守恒 -> 应 VALIDATE_ENERGY_OK

% ④' 若要重新从 STEP 生成 URDF / 对照参数 / 可视化网格（几何改动后）
%    python _stp2urdf.py    %  STP  -> simscape/exo_real.urdf + exo_real_report.txt
%    python _stp2plant.py   %  URDF -> simscape/plant_params.txt / .csv
%    python _stp2stl.py     %  STP  -> simscape/meshes/{base,leg_L,leg_R}.stl（约 7 min）
%    python _stl_preview.py %  STL  -> out_simscape/exo_real_mesh_preview.png

% ⑤ 一条命令跑完整条链路并回归（推荐；日志 simscape/vis_smoke_log.txt）
vis_smoke

% ⑥ 路径 A：SolidWorks 原生装配体 -> XML（需先装 Simscape Multibody Link 插件）
%    完整分步 SOP 见 simscape/XML_PATH_GUIDE.md
xml_preflight                      %  环境体检（只读）：插件 / SW 注册 / 路径 / 安装包 一次列清
smlink_verify                      %  装完插件后验收（只读）：核对 7 项落盘物 + 注册表
smimport_from_xml('D:\exo_xml\exo_master.xml')   %  导入 XML + 存盘 + dump 刚体 + 与 URDF 路对账
```

以上脚本都用 `E:\2024b-matlab\bin\matlab.exe -sd <ASCII junction> -batch "<脚本名>"`
跑过，全部通过；真实装配体链路最终判定 **`COMPARE_OK`**。

---

## 2. 目录结构

```
matlab2609/
├── +exo2609/                     ← MATLAB 包（函数调用形式 exo2609.xxx）
│   ├── params_paper.m            论文 Table II 标定参数
│   ├── params_from_csv.m         SolidWorks 质量属性 CSV -> M / G_amp      ★接口
│   ├── dyn_f.m                   Eq.(4) 连续状态导数
│   ├── dyn_step.m                Eq.(5) 显式欧拉离散步进
│   ├── dyn_rollout.m             整段前向积分
│   ├── dyn_jac.m                 解析雅可比 A_k, B_k
│   ├── weights_paper.m           Eq.(6) 权重 Q / P（Table II）
│   ├── q_blocks.m                Q_k 分块构造
│   ├── cost_grad.m               Eq.(6) 代价 + **解析伴随梯度**
│   ├── estimate.m                Eq.(6) 滑窗力矩估计（fmincon，无工具箱则退化为投影梯度）
│   ├── spectrum.m                观测 Hessian H=S'QS 谱分解（解释复原偏差）
│   ├── gait_ref.m                参考步态生成（surrogate，非实测）
│   └── discretization_info.m     步长 dt 的阻尼可分辨性诊断
├── check_gradient2609.m          ① 梯度校验
├── selftest2609.m                ② 回归自检（钉在已验证基准上）
├── demo2609_run.m                ③ 端到端 demo + 出图
├── out/                          运行产物（log + 3 张图）
├── solidworks/
│   ├── SOLIDWORKS_质量属性_SOP.md   ★ SolidWorks 侧完整操作流程
│   ├── mass_props_template.csv      填表模板（含本机 STP 的零件清单与质量基准）
│   ├── mass_props_example_from_stp.csv  由本机 STP 生成的可直接跑的样例
│   └── export_mass_props.bas        可选：批量导出宏（未在 GUI 实测，见文件头声明）
└── simscape/                     ← SolidWorks -> Simscape Multibody 链路
    ├── SIMSCAPE_BRIDGE_SOP.md      ★ CAD 导入多体的完整 SOP（XML/URDF/路径D + 12 个坑）
    ├── exo2dof.urdf                手写 2-DOF 孪生模型（隔离验证用；**不是**真实装配体）
    ├── exo_real.urdf               ★ 由 _stp2urdf.py 从真实装配体 STP 生成（含 <visual>）
    ├── exo_real_report.txt         生成报告：分组质量/髋轴/惯量/重力矩/自检
    ├── meshes/{base,leg_L,leg_R}.stl   ★ 由 _stp2stl.py 生成的可视化网格（世界系, m）
    ├── plant_params.txt / .csv     由 _stp2plant.py 生成：解析对照用的 M/A/B/I_axis
    ├── import_exo2dof.m            ※ 名字是历史遗留：实际导入的是 exo_real.urdf
    │                                  -> sm_exo_real.slx（真实装配体）
    ├── build_harness_exo2dof.m     加力矩驱动与状态测量 -> sm_exo_real_harness.slx
    ├── attach_visual_meshes.m      核验 <visual> 的 .stl 真进模型 + 路径改 ASCII 绝对路径
    ├── compare_simscape_vs_analytical.m   多体 vs 解析交叉验证（L2）-> COMPARE_OK
    ├── validate_energy.m           自由摆动能量守恒（L2'）-> VALIDATE_ENERGY_OK
    ├── view_exo_real.m             一条命令：开模型 + 打印配置 + 跑仿真 + 画 q/ω
    ├── vis_smoke.m                 ★ 一键回归：导入→挂网格→重建 harness→L2→L2'
    ├── xml_preflight.m             路径 A 环境体检（只读）：插件/SW/路径/安装包
    ├── smlink_verify.m             路径 A 插件验收（只读）：7 项落盘物 + 注册表
    ├── smimport_from_xml.m         路径 A：导入 SolidWorks XML + 存盘 + dump 刚体 + 对账
    ├── XML_PATH_GUIDE.md           ★ 路径 A（XML）完整分步 SOP（含 12 个 XML 特有的坑）
    ├── probe_ports.m / probe_params.m / probe_torque.m   排障诊断脚本
    ├── sm_exo_real.slx / sm_exo_real_harness.slx        生成的多体模型与仿真 harness
    └── *_log.txt                   运行日志（import / build_harness / compare / vis_smoke）
```

---

## 3. 与 SolidWorks 的接口（两步）

1. **在 SolidWorks 里**：建参考坐标系（原点在髋轴上、**Z 轴沿髋屈伸轴**）→
   `评估 → 质量属性` → 选项里选该坐标系 → 读出 质量 / 质心 / 惯性矩。
   详见 `solidworks/SOLIDWORKS_质量属性_SOP.md`。
2. **在 MATLAB 里**：
   ```matlab
   p = exo2609.params_from_csv('solidworks/mass_props.csv', 'leg_L','leg_R', 0.01);
   % p.M, p.G_amp 就是论文模型的惯量矩阵与重力矩幅值
   ```

折算关系（`params_from_csv.m` 内部）：
```
M_i     = I_hip_i                     绕髋轴惯量（含平行轴项）
G_amp_i = m_i * g * d_i               d_i = hypot(cx, cy) = 髋轴到质心垂距
K1/T0   几何不可反算（人机交互阻尼/摩擦），沿用论文并标注来源
```
**内置自检**：`Izz_O` 必须等于 `Izz_com + m·(cx²+cy²)`，偏差 > 1% 会告警
（说明参考坐标系没设对或零件漏选）。

---

## 4. SolidWorks → Simscape Multibody 多体链路（已在本机验证）

这是「用 SolidWorks + MATLAB 做动力学仿真」的**第二条腿**：解析模型（`+exo2609`）
给出辨识算法，多体模型给出**带真实 CAD 惯量的被控对象**，两者互为独立校核。

### 4.1 关键事实：`smimport` 不接受 STEP

```
smimport('...stp')  ->  sm:import:InvalidFileType
   Invalid file type '.stp'. Can only import Physical Modeling XML files or URDF files.
```
`smimport` 只吃两种输入：

| 路径 | 产出 | 前提 |
|---|---|---|
| **Simscape Multibody Link**（SolidWorks 插件） | 多体 **XML**（+ 参数 .m 数据文件） | 需安装该插件，SolidWorks 里 `Simscape Multibody Link → Export` |
| **URDF**（SW2URDF 插件 / 手写） | `*.urdf` | 无需 MATLAB 插件；`smimport` 直接吃，但**不生成**参数数据文件 |

两条路的操作步骤都在 `simscape/SIMSCAPE_BRIDGE_SOP.md`。
本机已验证的是 **URDF 路径**（不依赖插件、可脚本化、可进版本库）。
XML 路径与之等价，只是必须先装插件。

### 4.2 已验证结论

两条路都跑通了。**路径 D（真实装配体 STEP 直接生成 URDF）是最终采用的那条。**

**(a) 手写简化孪生 `exo2dof.urdf`** —— 只作隔离验证用：

| 核对项 | 结果 |
|---|---|
| `smimport` 导入 URDF | ✅ `URDF_OK`，生成 28 个块、2 个 Revolute Joint |
| 生成的模型编译 | ✅ `compile check: OK` |
| 力矩驱动 + 角度/角速度测量接线 | ✅ 12 条连线全部 `line OK` |
| 多体 vs 解析轨迹偏差 | ✅ rms/range = 9.0e-06 |
| ⚠ 局限 | 每腿参数（`m = 1.657 kg, d = 0.034104 m`）**与真实装配体不符**，只是 `m·g·d` 恰好撞上 |

**(b) 真实装配体 `exo_real.urdf`**（`_stp2urdf.py` 从 STP 生成，**无需 SW 插件**）：

| 核对项 | 结果 |
|---|---|
| `I_axis` vs STP 基准 CSV | ✅ 相对偏差 **6.0e-12 / 1.8e-11** |
| 闭式解 vs Rodrigues 采点自检 | ✅ **1.1e-16 m**（机器精度） |
| 模型编译 + 12 条接线 | ✅ 全部 OK |
| 场景1 自由摆动（`T = 0`） | ✅ **rms/range = 2.6e-06**，`max|q|` 两侧同为 1.8258 |
| 场景2 强制摆动（0.15 N·m @ 0.3 Hz） | ✅ **rms/range = 2.9e-06**，`max|q|` 两侧同为 2.1821 |
| **总判定** | ✅ **`COMPARE_OK`** |

即：STEP →（几何/惯量聚合 + 髋轴拟合）→ URDF → Simscape Multibody
搭出的多体对象，其响应与解析模型一致到 **3e-06（相对量程）**。
→ **CAD→MATLAB 的参数链路是可信的**，解析模型的惯量/重力矩不是拍脑袋来的。

> 真实装配体每腿参数（**2026-09-20 密度修正后**；见 `simscape/exo_real_report.txt`、
> `simscape/plant_params.csv`）：
> `m = 0.312589 kg`、`I_axis = 0.016997395 kg·m²`、`d = 0.182102894 m`、
> `τ_g = −0.341437·sin q − 0.441845·cos q` N·m、`|τ_g| = 0.558396 N·m`。
>
> ⚠ 修正前是 `m = 0.304833` / `I_axis = 0.016975774` / `|τ_g| = 0.554528`。
> 变动来源：`电机_轴` 由兜底铝 2700 改判**钢 7850**（理由：Ø8×32 实心轴，
> 且是 `电机_黄铜轴套8_10_18`（Ø8 内孔）的配合轴）。整腿 **+2.54%**、`I_axis` **+0.13%**、
> `|τ_g|` **+0.70%**。机理与全部 19 件的判决见 `simscape/MATERIAL_FUNCTIONAL_INFERENCE.md`。
> 改一处密度会牵动 `exo_real.urdf` → Simscape `.slx` → 全部 `COMPARE_OK` 基线，
> 重跑脚本：`matlab2609/simscape/rerun_after_density.m`。
>
> **`B ≠ 0`** —— CAD 关节零位不是「腿自然下垂」，详见
> `simscape/SIMSCAPE_BRIDGE_SOP.md` 第 2 节。用教科书 `−G·sin q` 去对照会得到
> 假性失配（`rms/range ≈ 0.95`，轨迹镜像）。

**(c) 可视化外形网格 `exo_real.urdf` + `meshes/*.stl`**（`_stp2stl.py`，让
Mechanics Explorer 显示真实外形，而不是占位体）：

| 刚体 | 三角面 | STL | 面覆盖率 | 说明 |
|---|---|---|---|---|
| `base`（motor_L+R+back） | 748,197 | 35.68 MB | 99.91% | 丢 2 个 OCC 无名小件（vol 135/209） |
| `leg_L` | 47,618 | 2.27 MB | 100% | — |
| `leg_R` | 47,614 | 2.27 MB | 100% | — |
| 合计 | **843,429** | 40.2 MB | — | 耗时 440 s |

顶点 = **世界系坐标（m）**（因为关节 `rpy="0 0 0"` ⇒ link frame ≡ 世界系），
`<visual><origin>` 取 0。自检：`leg_L` 的 STL 包围盒
`x[0.095,0.405] y[0.067,0.218] z[-0.327,-0.029] m` 与「关节原点 + CoM 偏移」
`(0.2606, 0.1617, −0.1651)` 吻合。

> **`<visual>` 与 `<inertial>` 是两条独立链路**：前者来自面网格（会 heal / 会丢件），
> 后者来自 `_mass_*.json`。所以丢那 2 个小件**不可能影响动力学**。
> 想要更小的 STL，调 `_stp2stl.py` 里的 `MESH_SIZE`（当前 base 6.0 mm / 腿 3.5 mm）。

### 4.3 关节物理端口索引（`probe_ports.m` 实测）

Simscape Multibody 关节块的物理信号端口**不是** `Inport/Outport`，而是
`LConn/RConn`，靠索引寻址，脚本化接线必须先探测：

| 端口 | 含义 | hip_L | hip_R |
|---|---|---|---|
| `LConn1` | 机械端口 R（smimport 已接） | 200 | · |
| `LConn2` | **力矩输入**（`TorqueActuationMode='InputTorque'`） | 280 | 283 |
| `RConn1` | 机械端口 C（smimport 已接） | 201 | · |
| `RConn2` | **位置输出 q**（`SensePosition='on'`） | 281 | 284 |
| `RConn3` | **角速度输出 ω**（`SenseVelocity='on'`） | 282 | 285 |

### 4.4 必须记住的坑（前 5 个来自多体链路，后 3 个来自本轮真实装配体对照）

1. **`smimport` 的模型只在内存里**。`-batch` 退出时未存盘的模型会被丢弃，
   必须显式 `save_system(mdl, path)` 才会有 `.slx` 落盘。
2. **XML 注释里不能出现 `--`**。URDF 是 XML，用 `-----------` 画分隔线会直接报
   `'--' sequence is illegal in comment`。分隔线请用 `=====`。
3. **不要用 `exist(p,'file')` 判断 `.slx` 是否存在**。一旦把该目录 `addpath` 进来，
   `exist('...\x.slx','file')` 会返回 **4**（"Simulink model"）而不是 2，
   于是 `~=2` 的判空逻辑会误报文件不存在。**用 `isempty(dir(p))`**。
4. **关节硬限位会毁掉对照实验**。URDF 的 `<limit>` 在 Simscape 里变成刚度
   `1e4 N·m/deg` 的硬弹簧；解析模型没有限位，一旦轨迹走到 ±1.5 rad，两边必然发散。
   纯动力学对照时应 `LowerLimitSpecify/UpperLimitSpecify = 'off'`。
5. **测试工况要避开大信号**。本腿惯量只有 `0.017 kg·m²`，`|τ_g| = 0.554 N·m`
   → 摆的固有频率 `sqrt(|τ_g|/I) = 5.71 rad/s`。给几 N·m 的力矩就是
   `q̈ ≈ 150 rad/s²`，一个周期内就撞限位。
   对照实验用小信号、远离共振：`amp = 0.15 N·m, f = 0.30 Hz`
   （`ω = 1.88 rad/s ≈ 0.33 ωn`）。
6. **`[a; b]` 拼接 + 列向量输入 = 驱动力矩静默归零**（本轮最难查的一个）。
   `@(t) [f(t); g(t)]` 在 `t` 是列向量时返回 `2N×1` 而非 `2×N`，
   于是 `Ta(1,:)` 只剩一个元素 → `From Workspace` 收到「常数 0」，
   **MATLAB 不报任何警告**，两个不同工况的轨迹逐位相同。
   → 用 `t(:).'` 强制行向量，并在调用处断言 `size(Ta) == [2, numel(taus)]`。
   **判据**：两个不同工况的 `max|q|` 完全一致 → 先怀疑力矩没进模型。
7. **仿射项 `C0 = (a·r3)(a·c)` 漏了不影响力矩，但会让自检误报**。
   完整展开是 `z_com(q) = C0 + W cos q + V sin q`，自检对拍必须带 `C0`
   （漏掉时报出的 `closed form mismatch` 数值**恰好等于** `|C0|`）。
8. **重力项符号**：`I·q̈ = T + τ_g`（两个广义力**同为加号**）。
   写成 `T − τ_g` 会让轨迹整体镜像到 `2π − q`，看着像 CAD 模型全错。
   本次修这个符号后，`rms/range` 从 `9.9e-01` 直接降到 `2.9e-06`。

### 4.5 可视化网格（STL）的三个坑

9. **`Impossible to mesh periodic surface` 换算法没用**。`back` / `motor_R` 里
   有**参数域上下界反转**的 Cone 面（`u=[-4.1e-5,-0.141]`、`v=[0.867,0.141]`），
   gmsh 取不到合法 UV 参数化，在 `meshGFace` 里**算法分派之前**就报错 ——
   `Mesh.Algorithm` = 1…6 全部同样失败。删这张面也没用
   （`occ.remove([(2,s)],recursive=False)` 连删 26 轮它都还在）。
   只有 `occ.healShapes([(3,v)], tol)` 有效：它会把判为非法的体积整个清掉
   （实测 `vol_sum` 恰好 −128.086800 mm³ = 该体积自身体积）。
10. **`occ.remove` + `synchronize()` 在大模型上假死**。3 万面的模型上删 1 个体积
    会触发**全模型 re-bind**，十几分钟回不来，看着像脚本挂了。→ 用局部 `healShapes`。
11. **`heal` / `remove` 会重编号体积 tag**。沿用 `_mass_*.json` 的旧 tag 会整体错位，
    而**惯量看着还挺正常**（静默错配零件）。→ 现场取几何指纹，与 `_map_solids`
    的 `SolidProps` 按 `(体积, 局部质心)` 逐位重匹（实测 258/258 命中）。

---

## 5. 已验证结果（MATLAB ↔ Python 交叉核对）

| 核对项 | Python（已独立验证） | MATLAB | 相对偏差 |
|---|---|---|---|
| 解析梯度 vs 中心差分 | ‖Δg‖/‖g‖ = 2.5e-10 | 6.3e-10 | — |
| 观测 Hessian 条件数 κ | 6.8918e+02 | 6.8918e+02 | **1.6e-06** |
| λ_min | 1.2358e-02 | 1.2358e-02 | <1e-6 |
| λ_max | 8.516700 | 8.516671 | **3.5e-06** |
| M（CAD） | 0.016976 | 0.016976 | 2.3e-07 |
| G_amp（CAD） | 0.5544 | 0.5546 | 1.5e-04 * |

\* G_amp 的小差异来自样例 CSV 里质心坐标只保留 4 位小数。

> ⚠ 踩过的坑：MATLAB 的 `(:)` 是**列优先**，numpy 的 `ravel()` 是**行优先**。
> `spectrum.m` 里若直接用 `Y(:)` 展平轨迹，会把权重 `Q` 作用到**错误的行**上，
> 把 κ 从 690 误算成 **9740**。已用局部函数 `rmajor()` 统一为行优先并写进注释。
> 这就是为什么 `selftest2609` 必须核对 λ 的具体数值，而不只是"看起来合理"。

---

## 6. 关键物理结论

### 6.1 本 CAD vs 论文 Table II

| 量 | 本 CAD | 论文 | 比值 |
|---|---|---|---|
| M (kg·m²) | **0.016976** | 0.0156 | 1.088× |
| G_amp (N·m) | **0.5544** | 0.879 | 0.631× |
| K1 | 3（沿用论文） | 3 | 1.000× |

髋轴由左右腿 `电机_出轴`（r=22mm）圆柱面拟合，**残差 0.0000 mm**，方向 ≈ +Y。
腿等效质量 0.3048 kg / 侧，髋轴到质心 0.1854 m，整机 3.314 kg。

> 注：`G_amp` 上表 0.5544 用的是整腿质量 1.657 kg / 垂距 0.0341 m 的 URDF 口径；
> `0.3048 kg / 0.1854 m` 是 STP 里按「摆动件」分组的另一口径。
> 真机标定前必须统一口径 —— **以 SolidWorks 里实际选中参与摆动的零件为准**。

### 6.2 「复原误差 85~89%」不是 bug

注入已知 `T_true` → 只给轨迹 → Eq.(6) 反解，`‖ΔT‖/‖T‖ = 85.2%`，
但**收缩比 = 0.148**：估计力矩是 `T_true` 的**同向等比压缩**（幅值只剩 ~15%），不是噪声。

机理（`spectrum.m` 给出）：窗口线性化后
```
A* = Σ_i [λ_i/(λ_i+Ca)] (v_i' A_true) v_i     ← 收缩因子就是 λ_i/(λ_i+Ca)
```
- `H = S'QS` 病态：本 CAD κ≈689，论文参数 κ≈2194（λ_max 8.5 vs 27.2）
- `Ca = 0.5` 是 `λ_median` 的 **18.4 倍** → `A_true` 约 **99%** 的能量落在 `λ < Ca` 的方向上被抹平
- Ca → 1e-8 时误差收敛到 5%、收缩比 → 1.002（沿真值方向已精确复原）

论文自述 `P` 是「preventing the torque from becoming excessively large」的正则项
→ **这是方法的固有特性，不是复现错误**。`Ca` 应被当作**助力幅值的标定旋钮**。

> 想解耦幅值与正则强度：让权重随观测曲率自适应，如 `Ca = α·λ_median`，
> 此时 `shrink ≈ 1/(1+α)` 不随工况漂移。

### 6.3 步长 dt 必须 ≤ 1.4 ms

内部阻尼时间常数 `M/K1`：CAD **5.66 ms**，论文 **5.20 ms**。

| dt | 极点 1−dt·K1/M | ‖ΔT‖/‖T‖ | 收缩比 |
|---|---|---|---|
| 10 ms | **−0.767**（符号交替） | 0.889 | 0.111 |
| 5 ms | +0.116 | 0.797 | 0.204 |
| 2 ms | +0.647 | 0.476 | 0.539 |
| **1 ms** | **+0.823** | **0.238** | **0.818** |

`dt = 10 ms` 时 `dt·K1/M = 1.77 > 1`，显式欧拉严重欠采样交互阻尼。
**请用 dt ≤ 1.4 ms（≥ 707 Hz）**，或对 `K1·qd` 做隐式/半隐式积分。

---

## 7. 环境与依赖

| 项 | 本机 |
|---|---|
| MATLAB | **R2024b**（`E:\2024b-matlab`） |
| Optimization Toolbox | ✅ 有（`estimate.m` 用 `fmincon`） |
| Simscape / Simscape Multibody | ✅ 有（`smimport` 实测可用） |
| Simscape Multibody Link 插件 | ❌ 未安装（XML 路径需先装） |
| SolidWorks | 2024（v32.5） |

`estimate.m` 在没有 Optimization Toolbox 的机器上会自动退化为
「Barzilai-Borwein 投影梯度 + Armijo 回溯」，结果等价但更慢。

调用方式（中文路径下的可靠做法）：
```
E:\2024b-matlab\bin\matlab.exe -sd C:\Users\29408\exo_work\matlab2609 -batch "<脚本名>"
```
`C:\Users\29408\exo_work` 是指向本目录的 **ASCII junction**，用来绕开
MATLAB CLI 在中文路径下的编码问题；`-sd` 只接受**脚本名**，
以 `_` 开头的文件名不是合法 MATLAB 标识符（`run` 会报 "文本字符无效"）。

---

## 8. 真机标定注意

- **步态轨迹目前是 surrogate**（按正常步态事件点拟合的代理数据），不是实测。
  基于它得到的**助力力矩绝对值**只能验证模型行为，
  **真机标定必须把 `Yd` 换成实测 `(q, qd)`**：
  ```matlab
  out = exo2609.estimate(p, w, Qref, Qdref, L);   % Qref/Qdref = (N x 2) 实测
  ```
- **密度是外部假设**（SolidWorks 材质 / 或手动指定）。`M` 与 `G_amp` 都随密度**线性**缩放。
- 论文**未给出窗口长度 `L` 的数值**（只给预测网络的 `Obs. horizon = 250`）。
  `L` 是一个自由参数，本实现默认 20（0.2 s @ dt=0.01）。
- Simscape 那边若要研究真实机器人，**记得把关节限位打开**
  （`LowerLimitSpecify/UpperLimitSpecify = 'on'`），并加上电机/减速器的
  惯量与摩擦（目前 URDF 里是理想关节，`damping=0`）。
