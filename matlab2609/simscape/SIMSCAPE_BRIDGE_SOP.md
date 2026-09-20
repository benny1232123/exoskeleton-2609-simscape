# SolidWorks → Simscape Multibody 链路 SOP

目标：把 SolidWorks 里的髋关节外骨骼装配体变成 MATLAB 里**可仿真的多体对象**，
并且让它的惯量/质心与 `+exo2609` 的解析模型对得上。

> 本机实测状态：
> - ✅ **路径 D（真实装配体 STEP → URDF → Simscape Multibody）** 已端到端跑通：
>   `COMPARE_OK`，两个工况 `rms/range = 2.6e-06 / 2.9e-06`。**这是最终采用的路径。**
> - ✅ **路径 B（插件导出 / 手写 URDF）** 已跑通（`URDF_OK` → 编译 OK → 偏差 `9.0e-06`）；
>   但那份手写孪生 `exo2dof.urdf` 的每腿参数与真实件不符，仅作隔离验证用（见路径 C）。
> - 🟡 **路径 A（Simscape Multibody XML）** 前提已全部验通、SOP 已另立文档：
>   **SolidWorks 2024 SP5** 与**原生主装配体**（`装配体\“林-Ⅰ”髋关节外骨骼机器人开发平台V0_1_1.SLDASM`
>   + 1058 个 `.SLDASM`）本机都在，唯一缺件是 *Simscape Multibody Link* 插件
>   （需下载 `smlink-r2024b-win64.zip`）。完整步骤见 **`XML_PATH_GUIDE.md`**。
>   体检脚本 `xml_preflight.m` 已跑通，当前输出 `XML_PREFLIGHT_BLOCKED`（2 项待办）。

---

## 0. 先记住这件事：`smimport` 不吃 STEP

```
>> smimport('xxx.stp')
错误: sm:import:InvalidFileType
Invalid file type '.stp'. Can only import Physical Modeling XML files or URDF files.
```

`smimport` 只接受：

| 输入 | 来源 | 是否生成参数数据文件 |
|---|---|---|
| **Simscape Multibody XML** | SolidWorks + *Simscape Multibody Link* 插件 | ✅ 生成 `*.m`（`smiData` 结构） |
| **URDF** | SW2URDF 插件 / 手写 / 其他 CAD 导出 | ❌ 不生成（参数直接烙进块里） |
| `robotics.RigidBodyTree` | Robotics System Toolbox | ❌ |

所以「SolidWorks 直接进 Simscape」必然要经过**插件导出**这一步，没有捷径。

---

## 路径 A：Simscape Multibody Link（XML）

> **详细分步 SOP 见 `XML_PATH_GUIDE.md`**（含本机体检结果、插件下载/安装命令、
> 导出设置、导入验收脚本与 XML 路特有的 9 个坑）。下面只留提纲。

**前提 1 —— 装插件**（插件**不在** MATLAB Add-On Explorer 里，必须手工下载 ZIP）：

```matlab
% 下载 **两个**文件：smlink-r2024b-win64.zip + installaddon.m
%   https://www.mathworks.com/campaigns/offerings/download_smlink_confirmation.html
% ⚠ 不要解压；⚠ 下面的 MATLAB 会话必须是**管理员身份**
% ⚠★ MATLAB 不内置 installaddon（R2024b 实测 which 为空；matlab.addons.install
%     只认 .mltbx）→ installaddon.m 必须一起下载，否则这一步卡死。
addpath('C:\smlink_install')
which installaddon                         % 自查：不能是空
installaddon('smlink-r2024b-win64.zip')   % 不是 install_addon
regmatlabserver                            % 注册 MATLAB 为自动化服务器
smlink_linksw                              % 挂进 SolidWorks（撤销用 smlink_unlinksw）
```

然后在 SolidWorks：`工具 → 插件` 勾选 **Simscape Multibody Link**。

**前提 2 —— 必须是原生装配体，且配合关系完整**

插件把装配体的 **mates** 自动映射成关节（不用你选轴/坐标系）。
STEP 导入得到的"哑实体"**没有 mates ⇒ XML 里没有关节**。
→ 这是路径 A 最常见的失败原因。

**前提 3 —— 每个零件要有材质/密度**

质量属性 = 几何 × 密度。`M` 与 `G_amp` 都随密度线性缩放，材质错了后面全错。
本仓库 Python 侧的密度是**查表假设**（`exo2609/geometry.py: DEFAULT_DENSITIES`），
两边要对账就得对齐口径 —— 详见 `XML_PATH_GUIDE.md` §2。

**SolidWorks 侧**

1. 打开**装配体**（零件不行），先另存为**纯 ASCII 名**（原文件名含中文引号）。
2. `工具 → Simscape Multibody Link → Export → Simscape Multibody`
3. Export settings：勾**导出几何**，选**每零件一个独立 STEP 文件**（别嵌进 XML）；
   目标目录用**新建的空目录**、纯 ASCII、无空格。
4. 产出：`<name>.xml`（拓扑 + 质量属性 + 参考系）+ `_STEP/` 几何目录。
   **目录结构别动**（XML 里是相对路径）。

**MATLAB 侧**

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape     % ASCII junction，绕开中文路径
addpath(pwd)
xml_preflight                                       % 环境体检（只读）
r = smimport_from_xml('D:\exo_xml\exo_master.xml')   % 导入 + 存盘 + dump 刚体 + 对账
```

`smimport_from_xml.m` 会显式 `save_system`（坑 1），统计关节，
dump 所有刚体的质量/质心/惯量，并与 URDF 路的参考数字对账，末尾给 `VERDICT`。

> **为什么路径 A 值得做**：本仓库 L1 / L2 / L2′ 三条证据**全部同源**于我们自己的
> Python STEP 解析（连所谓"STP 基准"也是同一条链产出的）—— 同源会同错。
> XML 路的质量属性来自 **SolidWorks 自己的内核**，是唯一能做**外部对账**的链路。

---

## 路径 B：URDF（开放、可脚本化、可进 git）★ 本机已验证

**前提**：SolidWorks 装 **SW2URDF** 插件
（ROS-Industrial `sw_urdf_exporter`，GitHub release 里的 `.exe` 安装包，免费）。

**SolidWorks 侧**

1. `工具 → Tools → Export as URDF`（或 `SW2URDF → Export as URDF`）。
2. **Link Tree**：把零件分配到 link。规则——
   - 相对躯干**不动的**（背板、电机定子、电池）→ 根 link（本 SOP 里叫 `base`）
   - 相对髋轴**摆动的**（大腿件、电机转子、绑带）→ 子 link（`leg_L` / `leg_R`）
3. **Joint 定义**：对每个父子关系选 joint 类型
   - 髋屈伸 = **revolute**（转动副）
   - `Axis` 一栏选**髋屈伸轴**（务必与解析模型约定一致，见第 2 节）
   - 限位填实际机械限位（例：`-1.5 ~ +1.5 rad`，力矩上限 18 N·m）
4. **Inertial** 页：选 `Automatically calculate`，并确认参考坐标系原点
   落在**关节轴上**。这一步就是质量属性的来源 —— 与
   `SOLIDWORKS_质量属性_SOP.md` 里手搓参考坐标系是同一件事，
   插件只是替你做了。
5. **导出**：得到 `<name>.urdf` + `meshes/`（可视化网格，可不带）。

**MATLAB 侧**（本机实测命令）

```matlab
cd C:\Users\29408\exo_work\matlab2609
addpath(fullfile(pwd,'simscape'))

import_exo2dof;                  % URDF -> sm_exo2dof.slx（并打印 28 块的清单）
build_harness_exo2dof;           % 加力矩输入 + q/ω 测量 -> sm_exo2dof_harness.slx
compare_simscape_vs_analytical;  % 与 +exo2609 解析模型对比 -> 应 COMPARE_OK
```

把 `import_exo2dof.m` 里的 `urdf` 换成你导出的文件路径即可。

---

## 路径 C：手写简化 URDF（仅作「隔离验证」用，**不要当作真实模型**）

`simscape/exo2dof.urdf` 是一个**手写的 2-DOF 简化孪生模型**：
不含真实几何，只含惯量/质心/关节轴。它的用途只有一个 ——
把「CAD 参数是否被正确翻译成动力学」这件事**单独隔离出来验证**
（没有几何/拓扑噪声；若连它都对不上，就不必去查 CAD）。

⚠ **它的每腿参数与真实装配体不一致**（`m = 1.657 kg, d = 0.034104 m`），
只是恰好让 `m·g·d = 0.5544 N·m` 与真实值撞上了，惯量与质心都是错的。
**要真实动力学请走路径 D。**

---

## 路径 D：从真实装配体 STEP 直接生成 URDF ★ 最终采用（无需任何 SW 插件）

关键认识：**STEP 里其实已经包含了动力学建模所需的一切** —— 几何，以及由
CAD 内核算出的完整惯量张量；缺的只是**拓扑语义**（哪些零件属于同一个刚体、
转轴在哪）。补上这一层，就**不需要 Simscape Multibody Link 插件**，
也不需要人工在 SolidWorks 里量坐标或建参考坐标系。

```text
林-Ⅰ…V0_1_1.stp
   │  _stp2urdf.py    (exo2609.geometry 解析 STEP -> 按零件名分组聚合刚体
   │                   用左右「电机_出轴」圆柱面拟合髋轴 -> 吐 URDF)
   ▼
simscape/exo_real.urdf      base = motor_L+motor_R+back ; leg_L / leg_R 绕髋轴
   │  _stp2plant.py    (反解解析对照所需的等效单腿参数 A, B, I_axis)
   ▼
simscape/plant_params.txt   人读
simscape/plant_params.csv   纯数值（MATLAB readmatrix 读，切勿混入文字）
   │  import_exo2dof -> build_harness_exo2dof -> compare_simscape_vs_analytical
   ▼
sm_exo_real.slx / sm_exo_real_harness.slx  +  COMPARE_OK
```

分组约定必须与 `solidworks/mass_props_example_from_stp.csv` 一致：

| 刚体 | 组成零件 | 说明 |
|---|---|---|
| `base` | `motor_L` + `motor_R` + `back` | 相对躯干不动的全部零件 |
| `leg_L` / `leg_R` | 各侧摆动件 | 绕髋轴做 1-DOF 转动 |

### 设计决定：link frame ≡ 世界坐标系（`<origin rpy="0 0 0">`）

第一版给关节写了 `rpy`，结果踩了坐标系约定的坑：URDF 的 `rpy` 语义与
其它地方的「XYZ 欧拉角」很容易不一致，而一旦不一致，**质心就被放错位置、
重力矩就悄悄变了 —— 而且不报错**。所以现在把关节 `rpy` 恒设为 `0 0 0`，
轴改用 3 分量 `<axis xyz>` 显式给出：

```
origin xyz = 髋轴上一点（世界系, m）
origin rpy = 0 0 0                      -> child link frame ≡ 世界系
axis       = 髋轴方向（世界系单位向量）
inertial.origin xyz = 质心相对关节原点的世界系偏移 (m)
inertia             = 绕质心、**世界轴向**的惯量张量 (kg·m²)
```

整条链上只有「世界系」一个坐标系，**没有任何旋转约定需要猜**。

---

## 2. 重力矩为什么不是 `−G_amp·sin q`（最容易误判成「模型全错」）

**关节零位 = CAD 建模时的位姿，不是「腿自然下垂」。**
URDF/Simscape 的关节角 q 从建模位姿起算，所以 CAD 导入的多体模型，
其重力矩通常**不是**一个纯 `sin`，而且**静止位姿下也不为 0**。

记 `a` = 关节轴单位向量（世界系）、`c` = 质心相对关节原点的偏移（世界系）、
`r3 = [0 0 1]`。绕轴转 q 后质心高度的**完整**仿射展开为

```
z_com(q) = C0 + W cos q + V sin q
    C0 = (a·r3)(a·c)            <- 仿射项。对力矩**无**贡献，但影响位姿
    W  = r3·c − (a·r3)(a·c)
    V  = r3·(a×c)
```

于是广义重力力矩（`U = m·g·z_com`，`τ_g = −dU/dq`）

```
τ_g(q) = A sin q + B cos q
    A = m·g·W ,   B = −m·g·V
    |τ_g| = m·g·d·|r3_perp| ,   d = |c − (a·c)a| = 质心到髋轴垂距
```

本机真实装配体实测（见 `exo_real_report.txt`；**2026-09-20 密度修正后**）：

| 量 | leg_L | leg_R |
|---|---|---|
| `I_axis = aᵀIa + m·d²` | 0.016997395 | 0.016997437 kg·m² |
| `d` | 0.182102894 | 0.182104871 m |
| `A` | −0.341436830 | −0.341462603 N·m |
| `B` | **−0.441845083** | **−0.441832833** N·m |
| `|τ_g|` | 0.558396084 | 0.558402150 N·m |
| 相位 | −127.6951° | −127.6980° |

> ⚠ 修正前的值是 `I_axis 0.016975774/0.016975815`、`B −0.438275227/−0.438316813`、
> `|τ_g| 0.554528/0.554511`。变动来源：`电机_轴` 由兜底铝 2700 改判钢 7850
> （`MATERIAL_FUNCTIONAL_INFERENCE.md` §0.1）。

**`B ≈ −0.44 N·m`，远不是 0** —— 静止位姿下本来就挂着 0.44 N·m 的重力矩。

> **误判特征**：若对照脚本仍用教科书形式 `M·q̈ = T − G_amp·sin q`，
> 会得到 `rms/range ≈ 0.95`（看着像模型全错），且 `max|q|` 恰好等于
> `2π −` 正确值（两条轨迹互为镜像）。这不是 CAD 错，是**零位相位**问题。
> `W, V` 在绕关节轴旋转坐标系时不变 —— 所以 `B = 0` 做不到，
> 除非腿质心恰好落在过轴的竖直面内。这个相位由**物理位姿**决定，换坐标系消不掉。

### 2.1 运动方程里的符号（务必写对）

```
I_axis·q̈ = T_applied + τ_g          <- 两项都是广义力，**同为加号**
```

实测校验（`T = 0` 自由摆动）：`B < 0` → `q̈(0) = B/I = −25.82 rad/s²`，
腿往 `−q` 方向摆；转折点解 `U(q) = U(0)` 得 `q = −1.8227 rad`，
与 Simscape 实测 `max|q| = 1.8231` 一致。

> 若误写成 `(T − τ_g)`，轨迹整体镜像到 `+4.4604 rad (= 2π − 1.8227)`。

### 2.2 重力向量

`MechanismConfiguration.GravityVector = [0 0 −9.81]`。
`smimport` 默认给 `−9.80665`；本流程强制改成 `−9.81`，
以便与 `_stp2plant.py` 里的 `G = 9.81` 逐位对齐。

---

## 3. 九个坑（全部实测踩过）

1. **`smimport` 生成的模型只在内存里。**
   `-batch` 退出时未存盘的模型直接丢弃 —— 日志里看着 `URDF_OK`，磁盘上却没有 `.slx`。
   → 必须显式 `save_system(mdl, fullpath)`。

2. **XML 注释里不能出现 `--`。**
   URDF 是 XML，拿 `-----------------` 画分隔线会直接报
   `'--' sequence is illegal in comment`。分隔线请用 `=====`。

3. **不要用 `exist(p,'file') == 2` 判断 `.slx` 是否存在。**
   一旦把该目录 `addpath` 进来，`exist('...\x.slx','file')` 返回 **4**
   （含义是 "Simulink model"）而不是 2，`~=2` 的判空逻辑就会误报"文件不存在"。
   → 用 `isempty(dir(p))`。

4. **关节硬限位会毁掉对照实验。**
   URDF 的 `<limit>` 在 Simscape 里变成刚度 `1e4 N·m/deg` 的硬弹簧。
   解析模型没有限位，一旦轨迹走到限位（本机是 ±1.5 rad）两边必然发散。
   纯动力学对照时应关掉：
   ```matlab
   set_param('sm_exo_real_harness/hip_L','LowerLimitSpecify','off', ...
                                        'UpperLimitSpecify','off');
   ```
   做真实机器人研究时再打开。

5. **测试工况必须避开大信号。**
   本腿惯量只有 `0.017 kg·m²`、`|τ_g| = 0.554 N·m`
   → 摆的固有频率 `sqrt(|τ_g|/I) = 5.71 rad/s`。
   给几 N·m 就是 `q̈ ≈ 150 rad/s²`，一个周期内冲出限位。
   对照实验用小信号、远离共振：`amp = 0.15 N·m, f = 0.30 Hz`
   （`ω = 1.88 rad/s`，约 `0.33 ωn`）。
   真实装配体实测：场景1 `max|q| = 1.8231 rad`，场景2 `max|q| = 2.1832 rad`，
   两侧偏差 `≤ 3e-06`。（手写孪生用同一工况时 `|q|max = 0.41 rad` ——
   因为它的假参数把重力矩相位留在了 0 附近。）

6. **仿射项 `C0 = (a·r3)(a·c)` 千万别漏 —— 漏了不影响力矩，但会让自检误报。**
   绕轴转 q 后完整展开是 `(R c)_z = C0 + W cos q + V sin q`。
   第一版只写了 `const + W cos q + V sin q` 而没把 `C0` 显式算出来，
   自检对拍立刻报 `closed form mismatch 1.4555e-03` —— 而这个数与
   `|C0|` **逐位相同**。
   `W, V` 本身是**对的**，`τ_g = −dU/dq` 对常数不敏感，所以物理没坏；
   但**位姿对拍必须带 `C0`**。
   教训：自检报错时先分清「是物理错了」还是「是自检用的模型不完整」。

7. **`[a; b]` 拼接 + 列向量输入 = 驱动力矩静默归零。**（本轮最难查的一个）
   ```matlab
   sc(2).Tfun = @(t) [ 0.15*sin(2*pi*0.3*t) ; -0.15*sin(2*pi*0.3*t) ];
   taus = (0:1e-4:Tend).';        % 列向量
   Ta   = Tfun(taus);             % => 2N x 1，不是 2 x N ！
   timeseries(Ta(1,:).', taus)    % Ta(1,:) 只剩 1 个元素 = sin(0) = 0
   ```
   于是喂给 `From Workspace` 的是「常数 0」的 timeseries —— **力矩没了**，
   `scenario 2` 的轨迹退化成 `scenario 1`（两者 `max|q|` 逐位相同），
   而 MATLAB **一句警告都不给**。
   → `Tfun` 一律用 `t(:).'` 强制行向量，并在调用处断言
   `size(Ta) == [2, numel(taus)]`。
   **判据**：若两个不同工况的 `max|q|` 完全一致，先怀疑驱动力根本没进模型。

8. **区分「力矩没进去」和「符号写错」，用一次阶跃实验。**
   `probe_torque.m` 给 `T = +1 N·m`（大于 `|τ_g| = 0.554 N·m`）跑 0.6 s：
   `max|q|` 从 `1.8228` 变成 `6.7636` → 力矩通路正常。
   若**不变** → 查坑 7 / 端口接线；若轨迹**镜像**（`2π − q`）→ 查第 2.1 节的符号。

9. **link 子系统里质心偏移不在 `Inertia.CenterOfMass`。**
   smimport 把 CoM 偏移放在独立的 `InertiaOriginTransform`（Rigid Transform）块里，
   参数名是 **`TranslationCartesianOffset`**；而 `Inertia` 块自己的
   `CenterOfMass` **恒为 `[0 0 0]`** —— 读它只会得到误导性的 0（本机踩过）。
   ```matlab
   b = 'sm_exo_real/leg_L';
   get_param([b '/Inertia'],'Mass')                                     % 质量 [kg]
   get_param([b '/InertiaOriginTransform'],'TranslationCartesianOffset') % CoM 偏移 [m]
   get_param([b '/Inertia'],'MomentsOfInertia')                          % [Ixx Iyy Izz]
   get_param([b '/Inertia'],'ProductsOfInertia')                         % [Iyz Ixz Ixy]
   ```
   这是**免 GUI 核对「CAD 参数到底有没有进模型」**最快的一招
   （本机实测与 `exo_real.urdf` 逐位一致）。

---

## 3.9 在 MATLAB 里查看模型的四种方式

| 方式 | 命令 | 用途 |
|---|---|---|
| **免 GUI 核对参数** | 见坑 9 的四条 `get_param` | 确认 CAD 参数真的进了模型（最廉价） |
| **打开模型窗口** | `open_system('sm_exo_real')` | 看拓扑：3 刚体 + 2 关节 |
| **一条命令跑通并画图** | `view_exo_real` | 准备 workspace 变量 → 跑仿真 → 画 q/ω → 打印配置 |
| **离线出动画（MP4/GIF）** | `anim_exo_3d` | 读 STL + q(t) 逐帧渲染成片，无 GUI 也能跑 |

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape    % ASCII junction
addpath(pwd)
view_exo_real                     % 默认：跑 1 s 受驱仿真并画图
view_exo_real('model','import')    % 只看纯 CAD 导入模型的结构
view_exo_real('Tamp',0.2,'f',0.5,'Tend',2)
view_exo_real('compare',true)      % 顺带重跑一致性对照
view_exo_real('video',true)        % 顺带把 Mechanics Explorer 的 3D 录成 MP4

anim_exo_3d                        % 出 out_simscape/exo_real_drop.mp4 + 三联姿态图
anim_exo_3d('preset','gait','view','iso')
```

> ⚠ **在 GUI 里直接点 Run 会失败**：harness 的 `T_L_src/T_R_src` 是
> **From Workspace** 块，需要 base 工作区里先有 `T_L_data` / `T_R_data`，
> 否则**编译期**就报「变量不存在」。`view_exo_real` 就是替你做这件事的。

> ⚠ **必须先 `cd` 到 `simscape` 目录**：STL 外形用的是**相对路径** `meshes/x.stl`，
> 按「当前目录」解析。直接在别的目录打开 `.slx` 再 Run，Mechanics Explorer 里
> 只会剩坐标系/占位体。（`view_exo_real` / `anim_exo_3d` 都已内置 cd。）

> **关于 3D 外形**：URDF 的 `<inertial>` 只给动力学，**不含几何** —— 所以哪怕
> 惯量/质心逐位正确，Mechanics Explorer 里也只有坐标系 + 占位体。
> 现在已补上真实外形：见下面 **3.10 可视化外形网格（STL）**。
>
> **可视化专题（preset 摆幅、物理注意事项、渲染坑位表）另见
> [VISUAL_ANIM_GUIDE.md](VISUAL_ANIM_GUIDE.md)。**

---

## 3.10 可视化外形网格（STL）：让 Mechanics Explorer 显示真实外形

### 为什么必须单独做一步
URDF 的 `<inertial>` 只描述动力学、**不含任何几何**，所以模型再准，
Mechanics Explorer 里也只有占位体。链路是：
`STEP → 面网格 → STL（世界系, m）→ URDF 写 <visual><mesh> → 重新 smimport`。

```bash
E:\Anaconda\python.exe -u _stp2stl.py              # 四组 -> meshes/{base,leg_L,leg_R}.stl
E:\Anaconda\python.exe -u _stp2stl.py --probe      # 只看 tag/零件 映射，不网格化
E:\Anaconda\python.exe -u _stp2stl.py --no-repair  # A/B 对照：不修坏面
```
```matlab
import_exo2dof          % 重新 smimport（URDF 里已带 <visual>）
attach_visual_meshes    % 核验 .stl 真进了模型 + 相对路径改 ASCII 绝对路径
```

### 坐标约定（复用 rpy = 0 的设计决定）
关节 `origin rpy="0 0 0"` ⇒ **child link frame ≡ 世界系**，所以 STL 顶点
**直接写世界系坐标（m）**，`<visual><origin>` 取 0。自检：`leg_L` 的 STL 包围盒
`x[0.095,0.405] y[0.067,0.218] z[-0.327,-0.029] m` 与「关节原点 + CoM 偏移」
= `(0.2606, 0.1617, −0.1651)` 吻合（质心落在盒内）。

### 与动力学的隔离（重要）
`<inertial>` 与 `<visual>` 是**两条互不相干的链路**：
`<inertial>` ← `_mass_*.json`（第一次 gmsh 提取的质量属性，未网格化）；
`<visual>` ← `_stp2stl.py` 的面网格（会 heal / 会丢件）。
所以下面那些「删面 / 丢体积」**不可能漏进动力学**。

### 坑 10：`Impossible to mesh periodic surface` —— OCC 翻译体的参数域反转面
`back` / `motor_R` 是 OCC 重新翻译出的体，里面有**上下界反转**的面：
```
surface 19349  type=Cone
  u = [-4.148e-05, -1.414e-01]      <- u_lo > u_hi
  v = [ 8.667e-01,  1.414e-01]      <- v_lo > v_hi
```
gmsh 取不到合法 UV 参数化，在 `meshGFace` 里**算法分派之前**就报错 ——
实测 `Mesh.Algorithm` = 1…6 全部（含默认 6）**同样失败**。别浪费时间换算法。
`occ.remove([(2,s)], recursive=False)` 删这张面也**无效**：连删 26 轮它都还在，
`synchronize()` 会从父体积的 shell 把它重建回来。
唯一有效的是 `occ.healShapes([(3,v)], tolerance)` —— 它会把判为**非法**的体积
整个清掉（本机实测 `vol_sum` 恰好 −128.086800 mm³ = 该体积自身体积）。
→ 本机代价：丢 2 个 OCC 无名小件（`back` 的 vol 135 / 209），换取其余 258 个体积完整。

### 坑 11：`occ.remove` + `synchronize()` 在大模型上「假死」
3 万面的模型上删 1 个体积会触发**全模型 re-bind**，实测十几分钟回不来，
看着像脚本挂了、其实在算。→ 清体积用**局部 `healShapes`**（20 s），别用 `occ.remove`。

### 坑 12：`heal` / `remove` 会重编号体积 tag ⇒ 映射必须按几何重建
一旦沿用 `_mass_<group>.json` 的旧 `tag`，映射就整体错位，而**惯量看着还挺正常**
（静默错配零件）。→ 改为现场取几何指纹，与 `_map_solids` 的 `SolidProps` 按
`(体积, 局部质心)` 双判据（体积相对 1e-9 + 质心绝对 1e-9 mm）逐位重匹。
本机实测 258/258 命中，未匹配的恰好是被丢掉的那 2 个体积。

### 分级判据（别把「有三角面」当「网格完整」）
`generate(2)` 在第一张坏面处抛异常，但**此前已网格化的面其网格仍在** ——
所以「修不动」不等于「什么都拿不到」。脚本据此分级输出：
1. 修（局部 heal / 删自由面）直到跑通；
2. 同一张面反复出现 3 次 → 停止修复，**收割已网格化的部分**；
3. 如实报 `面覆盖率`（有网格的面 / 总面）与 `丢弃的体积`，不掩盖缺口。
   判定 `STL_OK` 要求**所有保留体积都有三角面 且 面覆盖率 > 99.9%**。

---


## 交叉验证：三层证据链与各自的盲区

交叉验证最容易犯的错，是把「一层通过」当成「整条链都对」。本链路实际是三层，
每层堵的漏洞不同 —— 汇报时**必须分层说**：

| 层 | 手段 | 能证明 | **不能**证明 |
|---|---|---|---|
| **L1** STEP → URDF | `_stp2urdf.py` 自检：`I_axis` 对 STP 基准 CSV（rel **6e-12**）；闭式解 vs Rodrigues 采点（**1.1e-16**） | 几何解析、零件分组、髋轴拟合、惯量聚合无误 | Simscape 会怎么解释 URDF 的坐标系语义 |
| **L2** URDF → 多体 | `compare_simscape_vs_analytical` 轨迹对照（两工况 rms/range **2.6e-06 / 2.9e-06**） | Simscape 搭出的动力学 == URDF 蕴含的单自由度动力学 | URDF 本身对不对（**两边同源，会一起错**） |
| **L2'** 能量守恒 | `validate_energy`：自由摆动 `E = ½Iω² + (A cos q − B sin q)` 漂移 **1.4e-06** | Simscape **内部**的惯量/重力矩/积分器自洽；无隐藏阻尼、限位弹簧等非保守项 | 同上 |
| **L3** 模型 → 真机 | **未做**，需实测 `(q, q̇)` | 参数口径、摩擦、人体交互 | — |

```matlab
addpath(fullfile(pwd,'simscape'))
compare_simscape_vs_analytical    % L2 ：轨迹对照   -> COMPARE_OK
validate_energy(3)                % L2'：自由摆动 3 s 能量守恒 -> VALIDATE_ENERGY_OK
```

### L2' 为什么值得单独做

它**不写第二个模型、不需要解析 ODE**，只用 Simscape 自己输出的 `(q, ω)` 去验证
「单自由度投影 `(I_axis, A, B)` 是否真是那个多体系统的精确投影」。

当年那个 `rpy` 坐标系约定 bug（手推 `q̈(0)=+25.82`、Simscape 实测 `+19.94`）
用这一条会**立刻暴露** —— 因为 Simscape 内部积分的动力学与 URDF 的标量参数
不再匹配，能量必然漂移。而它只要一次仿真，比写第二个模型便宜。

### L2 的固有盲区（务必知道）

L2 的两条证据**都读同一个 URDF**。若 URDF 的 CoM 放错了，
Simscape 的动力学和解析的 `A`、`B` 会**一起错** —— 轨迹照样重合、
能量照样守恒，两边一起蒙对。

所以 **URDF 的正确性只能靠 L1 去堵**。这就是 L1 必须存在、且必须对
**外部基准**（`mass_props_example_from_stp.csv`，来自 SolidWorks/STP 质量属性）
对账，而不是对自身对账的原因。

### 怎么读这些数

| 量 | 值 | 含义 |
|---|---|---|
| 轨迹偏差 rms/range | `2.6e-06 ~ 2.9e-06` | — |
| Simscape 能量漂移 | `1.4e-06` | 与上面**同量级** |
| `ode45` 参考能量漂移 | `3.3e-12` | 求解器精度地板 |

Simscape 的漂移恰好和轨迹偏差同量级、而比 `ode45` 高约 5 个数量级
→ 说明**当前精度已经贴在 `ode23t` 的 `RelTol=1e-6` 容差地板上**，
是数值误差、**不是模型差异**。想再往下压只能收紧求解器容差，
而不是去改模型（改模型是错的方向）。

---

## 4. 关节物理端口索引（脚本化接线必需）

Simscape Multibody 关节块的物理信号端口**不是** `Inport/Outport`，而是
`LConn/RConn`，只能按索引寻址。`probe_ports.m` 实测（本机 smimport 生成的模型）：

| 端口 | 需要先打开的参数 | 含义 | hip_L | hip_R |
|---|---|---|---|---|
| `LConn1` | — | 机械端口 R（smimport 已接） | 200 | — |
| `LConn2` | `TorqueActuationMode='InputTorque'` | **力矩输入** | 280 | 283 |
| `RConn1` | — | 机械端口 C（smimport 已接） | 201 | — |
| `RConn2` | `SensePosition='on'` | **位置输出 q** | 281 | 284 |
| `RConn3` | `SenseVelocity='on'` | **角速度输出 ω** | 282 | 285 |

接线模板（`build_harness_exo2dof.m` 里就是这么做的）：

```matlab
set_param('sm_exo2dof_harness/hip_L', 'TorqueActuationMode','InputTorque', ...
                                      'SensePosition','on','SenseVelocity','on');
ph = get_param('sm_exo2dof_harness/hip_L','PortHandles');
torqueIn = ph.LConn(end);      % 力矩输入
qOut     = ph.RConn(end-1);    % q
wOut     = ph.RConn(end);      % ω
add_line(mdl, psOut, torqueIn, 'autorouting','on');   % Simulink-PS -> 关节
add_line(mdl, qOut,  psIn,     'autorouting','on');   % 关节 -> PS-Simulink
```
> 端口的**顺序**由块内部决定。换 MATLAB 版本或换关节类型后，
> **先跑一遍 `probe_ports.m` 打日志确认索引**，不要凭记忆。

---

## 5. 验收清单

- [ ] SolidWorks 里每个零件都有材质（密度）
- [ ] 反算的总质量与 SolidWorks 的 `质量属性` 总质量一致（本机基准 **3.3140 kg**）
- [ ] `Izz_O = Izz_com + m·(cx²+cy²)` 自检通过（`params_from_csv.m` 内置，偏差 <1%）
- [ ] 屈伸轴与质心方向**从 CAD 反算**，不要抄「轴 = +Y / 质心在 −Z」这种经验说法
      —— 那只对**路径 C 的手写孪生模型**成立。本机真实装配体实测：
      髋轴 `dir = (−0.00213, −0.99996, −0.00906)`（≈ **−Y**），
      `leg_L` 质心偏移 `= (0.14691, 0.16143, −0.11215) m`（**三个分量都不为零**）。
      改用「CoM 是否与 `exo_real_report.txt` 逐位一致」来验收（见坑 9）
- [ ] 若加了 `<visual>` 网格：`attach_visual_meshes` 报 **`VISUAL_MESH_OK`**
- [ ] `smimport` 返回 `URDF_OK`，且 `.slx` **真的落盘**（`dir` 看得到）
- [ ] 生成模型能编译：`set_param(mdl,'SimulationCommand','update')` 不报错
- [ ] `compare_simscape_vs_analytical` → **`COMPARE_OK`**（`rms/range < 1e-2`）
- [ ] 研究真实机器人时：关节限位已打开、电机/减速器惯量与摩擦已补上
      （URDF 里目前是理想关节 `damping=0`）

---

## 6. 相关文件

| 文件 | 作用 |
|---|---|
| `exo2dof.urdf` | 手写 2-DOF 孪生模型，可被 `smimport` 直接吃 |
| `import_exo2dof.m` | URDF → `sm_exo2dof.slx`，附带块清单与端口枚举导出 |
| `build_harness_exo2dof.m` | 加力矩驱动 + q/ω 测量 + 求解器设置，编译校验后存盘 |
| `compare_simscape_vs_analytical.m` | 多体 vs 解析交叉验证，出图 `out_simscape/*.png` |
| `probe_ports.m` | 诊断脚本：打印关节端口索引与可用的枚举值 |
| `../solidworks/SOLIDWORKS_质量属性_SOP.md` | SolidWorks 侧质量属性提取流程 |
| `../solidworks/mass_props_example_from_stp.csv` | 本机 STP 折算出的参数（URDF 数值来源） |
