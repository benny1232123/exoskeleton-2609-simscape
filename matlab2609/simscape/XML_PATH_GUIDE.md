# Simscape Multibody Link（XML 路径）实测 SOP

> 目标：把 `SolidWorks 原生装配体 → Simscape Multibody XML → smimport` 这条**路径 A** 真正跑通并验收。
>
> 为什么值得做：本仓库现有 L1 / L2 / L2′ 三条证据**全部同源**于我们自己写的 STEP 解析，
> 而 XML 路的质量属性来自 **SolidWorks 自己的内核** —— 这是目前唯一能做**外部对账**的链路。

---

## 0. 本机前提体检（2026-09-19 实测，脚本：`xml_preflight.m`）

| 项目 | 实测结果 | 结论 |
|---|---|---|
| MATLAB | R2024b (24.2.0.2712019) @ `E:\2024b-matlab` | ✅ |
| Simscape Multibody | `toolbox\physmod\sm\import\m\smimport.m` 在 | ✅ |
| SolidWorks | **SOLIDWORKS 2024 SP5**（build 32.5.0.0048）@ `C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\SLDWORKS.exe` | ✅ 在官方支持范围（2001Plus–2026）内 |
| 原生装配体 | `装配体\“林-Ⅰ”髋关节外骨骼机器人开发平台V0_1_1.SLDASM`（33.4 MB）+ 1058 个 `.SLDASM` + 462 个 `.SLDPRT` | ✅ **有原生装配体，这是关键** |
| ASCII junction | `C:\Users\29408\exo_work` | ✅ |
| **Simscape Multibody Link 插件** | **已装 + 已注册**（2026-09-19 12:34）：`which smlink_linksw` 有值；`HKLM\SOFTWARE\SolidWorks\AddIns\{2666BDBF-5207-4731-9976-13172BEB124F}` → `Simscape Multibody Link` | ✅ 卡点已消除 |

需要的安装包（版本对应关系由 `ver('matlab').Release` 自动推出）：**`smlink-r2024b-win64.zip`**

先跑一次体检（可随时跑，只读、不改任何东西）：

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape
addpath(pwd)
xml_preflight
```

日志 → `simscape/xml_preflight_log.txt`，末尾给 `XML_PREFLIGHT_READY` 或 `XML_PREFLIGHT_BLOCKED`。

---

## 1. 先理解这条路和已跑通那条路的区别

| | **路径 D（已跑通）** | **路径 A（XML，本文）** |
|---|---|---|
| 上游是谁 | 我们的 Python STEP 解析（`exo2609/geometry.py`） | **SolidWorks 内核** |
| 关节从哪来 | 我们按零件名分组 + **圆柱面拟合**轴 | **装配体 mates 自动映射**（不用你选轴/坐标系） |
| 质量属性从哪来 | 查表密度 × 几何体积 | **SW 的材质/密度 + SW 质量属性** |
| 几何从哪来 | 我们自己导的 STL / 解析体积 | 插件导出的 STEP（每零件一个文件） |
| 是否独立 | ❌ 与 L1/L2/L2′ 同源 | ✅ **独立上游** |

**两个直接推论（决定这条路能不能走通）：**

1. **没有配合关系 = 没有关节。**
   插件是靠装配体的 **mates** 生成关节的。把 STEP 导进 SolidWorks 得到的是"哑实体"，
   **没有 mates**，导出的 XML 关节就是错的/没有。
   → **必须用原生 `.SLDASM`**（我们正好有，见 §0）。

2. **没有材质 = 质量属性不可信。**
   SW 的质量属性 = 几何 × 材质密度。而我们 Python 侧的密度是**查表假设**
   （`exo2609/geometry.py` 的 `DEFAULT_DENSITIES`）：

   | 零件名关键字 | 密度 kg/m³ | 依据 |
   |---|---|---|
   | 黄铜 / 铜 | 8500 | brass |
   | 弹簧钢 / 钢珠 / 轴承 | 7850 | bearing steel |
   | 螺丝 螺钉 螺母 螺柱 顶丝 销 卡簧 垫片 弹垫 平垫 轴套 出轴 | 7850 | steel fastener/shaft |
   | 绑缚 魔术贴 织带 | 1150 | nylon/textile |
   | 麦拉片 按键 导光 泡棉 胶 | 1200 | polymer |
   | PCB 电路板 元件 OPEN_CASCADE USER_LIBRARY SOT23 LQFP HDR1 ACP3225 LEADER | 1500 | electronics |
   | **兜底** | **2700** | aluminium |

   → 想让两边可比，必须**对齐密度口径**，做法见 §2。

---

## 2. Step 2 —— 零成本预对账（**不用插件，先做这个**）

目的：先把密度口径的差异量出来，避免后面把「密度不同」误判成「几何错了」。

### 2.1 用**体积**对账（推荐先做，无任何密度歧义）

SW 的 `工具 → 评估 → 质量属性` 里同时给出**体积**和**质量**。
**体积与密度无关** → 体积对上就说明「几何解析 + 零件分组」无误，这是纯几何的外部证据。

在 SW 里打开主装配体，按零件名选中该组全部零件，读体积与质量，与下表对：

| 分组 | 零件数 | **我方体积 mm³** | 我方质量 kg | 我方 CoM（世界系）mm |
|---|---|---|---|---|
| `leg_L` | 18 | 99 029.547 | 0.312589 | [258.1357, 162.2063, −162.8516] |
| `leg_R` | 18 | 99 029.548 | 0.312589 | [257.4458, −160.3330, −165.7811] |
| `motor_L` | 8 | 131 436.295 | 0.355676 | [108.3377, 195.6374, −48.3544] |
| `motor_R` | 244 | 146 815.540 | 0.392793 | [107.3357, −194.1382, −51.5897] |
| `back` | 196 | 498 103.426 | 1.955848 | [−38.7700, 1.9863, −5.8954] |
| `base`（= motor_L + motor_R + back） | — | 776 355.261 | 2.704317 | [1.7991, −1.0308, −18.1166] |
| **整机** | — | — | **3.329496** | — |

> 质量/CoM 为 **2026-09-20 密度修正后**的值（`leg` 由 0.304833 → 0.312589，
> 因 `电机_轴` 改判钢 7850；见 `MATERIAL_FUNCTIONAL_INFERENCE.md` §0.1）。
> 体积来源：`simscape/stl_export_report.txt` 里各组的 `vol_sum`；
> 质量/CoM 来源：`simscape/exo_real_report.txt`。

⚠ **`leg_L` 组不只是「腿部设计_左」子装配体** —— 它还含 4 个**随腿一起转动的电机件**：
`电机_轴`、`电机_出轴`、`电机_黄铜轴套8_10_18`、`电机_片形腿杆连接件`。
只选子装配体会导致质量偏小。**完整清单**见 `_xml_group_names.txt`
（导出脚本：`_xml_group_names.py`，从 `_targets3.json` 生成）。

### 2.2 判据

| 观察 | 结论 |
|---|---|
| 体积对得上（rel ≲ 1e-3） | ✅ 几何解析 + 分组无误 —— **对外部基准的第一条独立证据** |
| 体积对不上 | ❌ 真问题。回头查分组边界 / 是否漏了零件，**别继续往下走** |
| 体积对得上、但质量差一个固定比例 | 正常 —— 纯粹是密度表差异，按 §1 推论 2 对齐后再比 |

如果要在 SW 里做**质量**对账：给每个零件赋材质，密度取上表同值
（铝 2700 / 钢 7850 / 黄铜 8500 / 尼龙 1150 / 塑料 1200 / PCB 1500），
SW 自带材质库里这些值基本都是整数，直接改密度即可。

---

## 3. Step 1 —— 装插件（约 20 分钟）

### 3.1 下载

打开下载页（需要 MathWorks 账号登录；插件本身不额外收费）：

```
https://www.mathworks.com/campaigns/offerings/download_smlink_confirmation.html
```

找到 **Simscape Multibody Link 24.2 – Release 2024b**，下载**两个**文件：

- `smlink-r2024b-win64.zip` —— 插件本体
- **`installaddon.m`** —— 安装脚本。**这个必须一起下，漏了必卡。**

放到一个**纯 ASCII、无空格**的目录，例如 `C:\smlink_install`（两个文件放同一目录）。

> ⚠ **不要解压这个 zip**。`installaddon` 要的就是压缩包本身。

> ⚠ **为什么必须下 `installaddon.m`**（本机实测，2026-09-19）：
> **R2024b 并不内置 `installaddon`** —— `which('installaddon')` 返回空、`exist` = 0，
> 整份 MATLAB 安装目录里 `**/installaddon*` 零命中。
> 而唯一的通用安装 API `matlab.addons.install` **只认 `.mltbx`**
> （源码里写死 `if ~strcmpi(ext,'.mltbx'), error('...invalidFileType')`），喂 `.zip` 直接报错。
> 所以 `installaddon.m` 是 MathWorks 为这类插件包**单独提供的安装脚本**，
> 不是 MATLAB 自带函数。
>
> 自查：`which installaddon` 应当指向你刚下载的那个 `.m` 文件（而不是空）。

> **备选方案（拿不到 `installaddon.m` 时）**：这个 zip 的内部结构**就是 matlabroot 的镜像**
> （顶层只有 `bin/`、`toolbox/`、`resources/` 三个目录，
> 含 `bin\win64\cl_sldwks2sm.dll`、`resources\physmod\en\smlink\swaddin.xml`、
> `toolbox\local\path\physmod_smlink.phl`）。
> 因此理论上可以手动把这三个目录**合并复制**进 `matlabroot`（本机 `E:\2024b-matlab`，
> 已实测当前用户对其 `bin\` 与 `toolbox\physmod\` **可写**）。
> 但这是**非官方做法**，没有卸载入口，也不做版本/一致性检查 —— 优先还是把 `installaddon.m` 下下来。

### 3.2 以**管理员身份**运行 MATLAB

这一步不能省 —— `installaddon` 要往 MATLAB 安装目录和注册表写东西。

### 3.3 安装 + 注册为自动化服务器 + 挂到 SolidWorks

> 以下步骤已按 **`installaddon.m` 源码 + 插件 zip 内部结构实测**核对（2026-09-19）。
> 本机两个文件都在 `E:\Edge_Download\`：`smlink-r2024b-win64.zip`（8 199 063 B）
> 与 `installaddon.m`（3 419 B）。

在（管理员的、**桌面版**）MATLAB 命令行里依次执行：

```matlab
addpath('E:\Edge_Download')                                    % zip 与 installaddon.m 所在目录
which installaddon                                             % ★ 自查：应指向 E:\Edge_Download\installaddon.m，不能是空
installaddon('E:\Edge_Download\smlink-r2024b-win64.zip')      % ★ 传绝对路径最稳
smlink_verify                                                  % ★ 立刻验收：期望 SMLINK_FILES_OK_SW_NOT_REGISTERED
regmatlabserver                                                % 注册 MATLAB 为 COM 自动化服务器（导出时插件要连回来）
smlink_linksw                                                  % 注册 SolidWorks 插件（会弹 UAC，选"是"）
smlink_verify                                                  % 再验收一次：期望 SMLINK_INSTALL_OK
```

**`installaddon.m` 源码实测行为**（照它做，别照网上教程做）：

| 行为 | 细节 | 对你的影响 |
|---|---|---|
| 参数 | `installaddon(zip_file)`，**只吃 1 个参数** | 不是 `install_addon`（老 RW 名），没有第 2 个参数 |
| **解析文件名** | `strtok(name,'-')` 切出 `smlink` / `r2024b` / `win64`，再剥掉 `r` | ★ **绝对不要重命名 zip**。叫 `... (1).zip` 或去掉连字符 → 直接 error |
| 架构校验 | `computer('arch')` 必须 == `win64` | 本机满足 |
| 版本校验 | `version('-release')` 必须 == `2024b` | 必须下 **R2024b** 版，不能拿 R2023b 的凑 |
| 需要 Java | 开头 `usejava('jvm')` 检查 | **必须桌面版 MATLAB**（`-nojvm`/纯 CLI 会挂在此处） |
| 落盘 | `unzip(zip, matlabroot)` 直接铺进 `E:\2024b-matlab` | zip 内部就是 matlabroot 镜像（顶层 `bin/` `resources/` `toolbox/`） |
| 挂 path | 读 `toolbox/local/path/physmod_smlink.phl`（**只有 1 行**：`toolbox/physmod/smlink/smlink`），`savepath` 写 `pathdef.m` | 若报 `Warning: Unable to save modified path to file` → 本次会话可用，**下次启动就找不到了** |
| 结尾 | 调 Java `BuildSharedDocCommand` 重建文档索引 | ⚠️ 这步可能抛异常，但**解压/挂 path 早已完成** → 见下 |

> ⚠ **`installaddon` 报错 ≠ 安装失败。** 最后一步（文档索引）失败时，前面
> 的解压和加 path 都做完了。此时直接跑 `smlink_verify` 核对文件；若文件齐，
> 补一句 `addpath(fullfile(matlabroot,'toolbox','physmod','smlink','smlink')); savepath;` 即可。

> **`smlink_linksw` 到底干什么**（源码实测，`toolbox\physmod\smlink\smlink\smlink_linksw.m`）：
> 它执行 `system('regsvr32 "<matlabroot>\bin\win64\cl_sldwks2sm.dll"', '-echo', '-runAsAdmin')`。
> 即：**regsvr32 注册那个 559 400 B 的 DLL**，由 DLL 自己的 `DllRegisterServer`
> 写入 `HKLM\SOFTWARE\SolidWorks\AddIns\{CLSID}`。所以：
> - DLL 必须先被 `installaddon` 放到 `E:\2024b-matlab\bin\win64\` —— 顺序不能颠倒；
> - `-runAsAdmin` 会弹 **UAC**，点"是"；MATLAB 本身已是管理员时不弹；
> - ★★ **`smlink_linksw` 不检查 `system` 返回值** → UAC 被漏掉/拒绝时是
>   **静默 no-op**：不报错、不输出、什么都没发生。**这是本路径最容易误判的失败。**
>   2026-09-19 本机真实踩到：7 项落盘物字节数全对、`pathdef.m` 已写、
>   `regmatlabserver` 成功，但 `HKLM\SOFTWARE\SolidWorks\AddIns` 里没有项、
>   `HKCR\CLSID` 搜 `cl_sldwks2sm` **0 命中**。原因就是 UAC 对话框被 MATLAB
>   窗口盖住没被点到。**绝不能用「MATLAB 没报错」判定注册成功。**
> - 客观判定命令（普通 PowerShell 即可，不需要管理员）：
>   ```
>   reg query "HKLM\SOFTWARE\SolidWorks\AddIns" /s | findstr /i "simscape smlink"
>   reg query "HKCR\CLSID" /s /f cl_sldwks2sm /d
>   ```
>   成功时应看到（本机实测值）：
>   `{2666BDBF-5207-4731-9976-13172BEB124F}` / `Title = Simscape Multibody Link` /
>   `InprocServer32 = E:\2024b-matlab\bin\win64\cl_sldwks2sm.dll`
> - ⚠ **`regmatlabserver` 成功 ≠ 提权成功**。它写的是 `HKCU\Software\Classes`
>   （HKCR 的按用户合并视图），**本来就不需要管理员**。所以它成、`smlink_linksw` 败，
>   是完全可能的组合 —— 别拿它当提权证据。MATLAB 里自查提权：
>   `system('net session >nul 2>&1') == 0` 才是管理员。
> - **最确定的修法（绕过 MATLAB 提权链）**：管理员 cmd 里直接
>   ```
>   regsvr32 "E:\2024b-matlab\bin\win64\cl_sldwks2sm.dll"
>   ```
>   → **必须看到弹窗** `DllRegisterServer 在 ... 中成功`。没弹窗就是没成。
> - 撤销注册用 `smlink_unlinksw`。

> 📌 **纠正一处常见误解**：zip 里的 `resources\physmod\en\smlink\swaddin.xml`（236 B）
> **不是** SolidWorks 插件清单。它是个 i18n 消息表（根节点 `<rsccat>`，全文只有一条
> `WindowsOnly` 文案）。插件清单/注册完全由上面那个 DLL 承担。别拿它当验收对象。

**如果某一步报权限错误**（`Access is denied`、`accessDeniedInstallationPath`）：
说明当前 MATLAB 不是管理员 —— **关掉它，右键 `matlab.exe` → 以管理员身份运行**，重来一遍。

> `regmatlabserver` 的作用：每次导出时插件要**连回 MATLAB**。
> 备用做法（管理员 cmd）：`matlab -regserver`。
> 官方要求 **MATLAB 与 CAD 装在同一台机器**上（本机满足）。

### 3.4 在 SolidWorks 里启用

1. 启动 SolidWorks（**注册后要重开一次**，插件列表在启动时读取）
2. 菜单栏 **工具(Tools) → 插件(Add-Ins)**
3. 勾选 **Simscape Multibody Link**（右侧"启动"也一并勾上，省得每次重勾）
4. 关闭对话框

之后**打开装配体**时，菜单栏会出现：
`工具(Tools) → Simscape Multibody Link → Export → Simscape Multibody`

### 3.5 Step 1 验收

**不看"MATLAB 没报错"，看落盘物。** 期望这几个文件真的存在（`matlabroot` = `E:\2024b-matlab`）：

| 期望路径 | 大小 | 作用 |
|---|---|---|
| `bin\win64\cl_sldwks2sm.dll` | 559 400 B | ★ 真载荷，`regsvr32` 的对象 |
| `toolbox\physmod\smlink\smlink\smlink_linksw.m` | 686 B | ★ 注册命令 |
| `toolbox\physmod\smlink\smlink\smlink_unlinksw.m` | 738 B | 撤销注册 |
| `toolbox\physmod\smlink\smlink\smlink_linkinv.m` | 727 B | Inventor 版（本机用不到） |
| `toolbox\physmod\smlink\smlink\smlink_unlinkinv.m` | 788 B | 同上 |
| `toolbox\local\path\physmod_smlink.phl` | 71 B | path 清单（只列 1 个目录） |
| `resources\physmod\en\smlink\swaddin.xml` | 236 B | i18n 消息表（非清单） |

一条命令全查完：

```matlab
r = smlink_verify          % 期望 VERDICT: SMLINK_INSTALL_OK
which smlink_linksw        % 应为 E:\2024b-matlab\toolbox\physmod\smlink\smlink\smlink_linksw.m
```

SolidWorks 里 `工具` 菜单下能看到 `Simscape Multibody Link` 子菜单 → ✅

---

## 4. Step 3 —— 导出 XML

### 4.1 先用**最小装配体**跑一遍（强烈建议）

在 SW 里打开 `装配体\腿部设计_左.SLDASM`（49 KB，18 个零件）→ 导出。

目的：用最小代价把**所有环境坑一次暴露**（管理员权限 / regmatlabserver / 路径编码 / 几何导出设置）。
这一步的产物只用来验证工具链，数字不用管。

### 4.2 完整导出

1. 在 SW 里打开主装配体 `“林-Ⅰ”髋关节外骨骼机器人开发平台V0_1_1.SLDASM`
   > ⚠ 文件名里有**中文引号**。先 **另存为** 一个纯 ASCII 名（如 `exo_master.SLDASM`）再导出，
   > 否则 XML 与 STEP 文件名会带中文，MATLAB 侧读取容易出问题（已知坑）。
2. `工具 → Simscape Multibody Link → Export → Simscape Multibody`
3. **Export settings** 里：
   - 勾 **导出几何（Export geometry）**
   - 选 **每个零件导出为独立的 STEP 文件**（不要"把几何嵌进 XML"）
     → XML 只存拓扑，几何是外部文件，便于后续替换成 STL
   - 目标目录：**新建的空目录**、纯 ASCII、无空格，例如 `D:\exo_xml`
4. 确认 → 等待（主装配体体量大，可能跑几分钟）
5. 预期产物：

   ```
   D:\exo_xml\
       exo_master.xml            ← 拓扑 + 质量属性 + 参考系
       exo_master_STEP\          ← 各零件几何（目录名以实际为准）
           xxx.step ...
   ```

> ⚠ **导出后不要改动目录结构**：XML 里记的是相对路径，挪了就丢几何。

### 4.3 导出后肉眼查两件事

- **XML 文件大小**：完整装配体应当是**几百 KB 到 MB 级**；若只有几 KB，
  说明参数/几何没真正导出（回 §4.2 第 3 步）—— 这个判据不依赖任何格式细节。
- **几何目录里文件数**：应当与装配体零件数量级相当，不是空的。

### 4.4 ★★ 导出后**必查**四件事（2026-09-19 实测，全部踩过）

「文件不为空」远远不够。先跑脚本，再逐条看判据 —— 这四条能挡掉几乎全部无效导出：

```bash
# 合法性 + 结构 + 逐件质量 + 几何引用核对
python _xml_inspect.py  "D:\exo_xml\<模型>.xml"  "matlab2609\simscape\_xml_inspect.txt"
# 关节/装配层次段落原文 + 多口径关节计数
python _xml_joints.py   "D:\exo_xml\<模型>.xml"  "matlab2609\simscape\_xml_joints.txt"
```

| # | 判据 | 通过标准 | 不通过说明什么 |
|---|---|---|---|
| 1 | **XML 是否合法** | `非法 UTF-8 = 0` 且 `XML 禁止的控制字符 = 0` | `smimport` 会直接 `not well-formed (invalid token)`，**连导入都进不去** |
| 2 | **关节数** | `<Constraints>` 内有子元素 / 全文含 `Joint`/`Revolute`/… > 0 | **0 = 白导**。装配体 mates 没映射上，导入得到一堆固定不动的摆件 |
| 3 | **材质（真密度）** | 逐件 `质量 ÷ 我方体积 ≈ 该零件的真实材料密度` | 全部 ≈ 1.000 g/cm³ → SW 用了**默认密度**，质量全是假的（见下） |
| 4 | **几何引用** | 所有 `.STEP` 引用都能在目录里找到 | 引用名与磁盘名不一致 → `smimport` 找不到几何 |

⚠ 注意判据 4：XML 里的 `.SLDPRT` 引用**本来就找不到**（那是插件记录的原生零件回链，
不在导出目录里）—— **只需核对 `.STEP`**，别被 `MISS` 数量吓到。

#### 2026-09-19 本机实测（`腿部设计_左.SLDASM`）—— 四条里**两条没过**

| 判据 | 实测结果 |
|---|---|
| 1 合法性 | ❌ **10 个控制字符全挤在 L455**（`0x0C 0x15 0x19 0x18 0x17 0x1C 0x11 0x04`），出自 `腿部_腿杆_片状V5` 的 `<GeometryFile name="...">` |
| 2 关节 | ❌ **`<Constraints>` 内部只有 1 个换行符**；`Joint/Mate/Revolute/Prismatic/Fixed/...` 全文计数 **全 0**；35 个 `<Instance>` **全部 `grounded="true"`**，无一个 `false` |
| 3 材质 | ❌ 19 件 Σ=`0.099084 kg`，`mass==0` **0 件**（看着像 OK），但 `电机_轴` 1506.0946 mm³ ÷ 0.0015060930 kg → **1.000000 g/cm³**；`电机_出轴` 12342.4015 mm³ → **1.000004 g/cm³** |
| 4 几何引用 | ✅ 17 个 `.STEP` 全部命中（另外 21 个 `MISS` 都是 `.SLDPRT` 回链，属正常） |

**由判据 3 引出的重要结论**：我们 Python 侧 `_mass_leg_L.json` 里
`电机_轴` 体积 1506.0946 mm³、质心 9.79477 mm、惯量与 SW **严格差一个固定倍数**
（同几何同分布），但质量差 **3.077 倍** —— 因为
**我们的查表密度均值 3.078 g/cm³ vs SW 的默认 1.000 g/cm³**。
→ **这份 SW 质量数据当前不能当外部基准用。** 必须先在 SolidWorks 里逐件赋真材料。

#### 三条判据背后的坑（写进 §6）

- **控制字符来自「配置名」**：插件把非 ASCII 配置名拼进 `name=` 时吐原始字节。
  同源问题还有第二种表现：名字里的坏字节被替换成 `U+FFFD`（合法 UTF-8，
  XML 不报错但文件名变乱码，如 `腿部_滑轨_片形_默认(ń�b01)…`）。
  **两种表现同一个根因**；改配置名为纯 ASCII 可一并消除。
- **`grounded="true"` 满盘 = mates 没被识别**。别只看「有几个 Instance」，
  **要看 `grounded="false"` 有几个**（0 个就是全固定）。
- **`mass ≠ 0` ≠ 有材质**。SW 对没赋材质的零件仍会用**默认密度 1000 kg/m³**
  算出一个非零质量——比 `mass == 0` 更难发现，因为它看起来"正常"。
  **唯一可靠判据是 `质量 ÷ 体积` 是否等于该零件应有的密度。**

#### 判据 5：**几何文件数必须等于 `<Part>` 数**（最容易漏的一类丢件）

`_xml_inspect.py` §7 里 `.STEP` 引用出现 `MISS`，**不是脚本 bug，是真丢件**。
2026-09-19 实测：**19 个 `<Part>` 只导出 17 个 `.STEP`** ——
`腿部_腿杆_片状V5`（**32.27 cm³，最大的结构件**）和 `腿部_轴盖` 的几何**根本没写出来**。

原因见下：控制字节在 Windows 文件名里非法 → 写文件直接失败。

> **同一根因的两种后果**（配置名非 ASCII → 插件拼文件名时吐坏字节）：
> | 坏字节形态 | 后果 |
> |---|---|
> | `U+FFFD`（合法字符） | 文件**写出**、名字乱码（`腿部_绑缚_默认\ufffd\ufffde_sldprt.STEP`）、XML 仍合法、引用对得上 → **只是难看，能导入** |
> | 原始控制字节 `0x00-0x1F` | **文件名非法 → 文件写不出来（静默丢件）** + XML 变非法 → **双重致命** |
>
> 一次改配置名为纯 ASCII 可同时消除两种后果。

### 4.5 ★★ 关键方法：**不需要知道材料，也能做独立几何对账**

这把「我没有材料信息」从阻塞项变成非阻塞项 —— **路径 A 的核心价值是几何，不是质量**。

前提（已实测确认到 7 位）：SolidWorks 未赋材质时用**默认密度 1000 kg/m³**，且精确。
于是在这个已知比值下，SW 侧的任何输出都能还原成**密度无关的几何量**：

| SW 侧输出 | 单位 | 还原成几何量 |
|---|---|---|
| `Mass` | kg | **体积 [mm³] = Mass × 1e6** |
| `CenterOfMass` | m | **CoM [mm] = × 1000** |
| `Inertia`（**前 3 个数就是对角**，实测确认） | kg·m² | **∫r²dV [mm⁵] = I × 1e12** |

而我方 `exo2609/geometry.py` 输出的 `inertia_local` **本身就是 `mm⁵`、密度=1**
（见该文件 `SolidProps` 注释）—— **两边可以直接逐件比，不需要任何材料信息。**

工具：**`_xml_vs_py.py`** —— 逐件三方表（SW 体积 / 我方体积 / 相对差）、
质心对比、材料建议表、判定 `GEOM_XCHECK_OK / MISMATCH`。

### 4.6 我方解析器已被**独立验证**（2026-09-19 实测）

三方探针 **`_sw_step_probe.py <零件名前缀>`**：
**A** = gmsh/OCC 读**插件导出的 STEP**；**B** = SW XML 反解；**C** = 我方 `_reduced3` 结果。

| 零件 | A~B（体积） | A~C（体积） |
|---|---|---|
| `腿部_滑块_双键` | **2.06e-4 %** | 1.25e-3 % |
| `腿部_绑缚` | **5.32e-3 %** | 2.05e-2 % |

**A 始终比 C 更贴近 B（4~6 倍）** → 我方解析器（gmsh 4.15.2 / OCC）与 SolidWorks 内核
在**同一份 STEP** 上吻合到 `1e-6 ~ 1e-4` 相对量级（Parasolid vs OCC 两个不同内核，
这点差异属正常）。`_xml_vs_py` 里看到的偏差**主要来自 `_reduced3` 这份裁剪/重写后的几何**，
不是解析器误差。偏差量级分布：

- **解析曲面件**（螺丝/轴套/挡片/螺母…）：`< 1e-6`（本质精确）
- **自由曲面件**（滑轨/绑缚/按键/滑块…）：`1e-3 ~ 2e-2 %`
- ⚠ `腿部_腿杆_片状V5`（`_xml_vs_py` 里差 0.18%）**无法用此探针验证** ——
  它的 STEP 没被导出（见判据 5）

### 4.7 材料怎么办（回答「我只有一个 STP 文件」）

- **STP 里没有材料记录**（实测 `MATERIAL` / `DENSITY` 零命中）→ 只能推断。
- 三档做法，可靠性递增：
  1. **按零件名推断** —— `_xml_vs_py.py` 末尾会打印建议表，真源是
     `exo2609/geometry.py: DEFAULT_DENSITIES`（黄铜 8500 / 钢 7850 / 尼龙织物 1150 /
     聚合物 1200 / 电子件 1500，找不到兜底铝 2700）。
  2. **称重** —— 实物单件质量 ÷ 已知体积 = 真实密度。**这是唯一硬证据**；
     没有实物就找设计者要 BOM。**注意：几何一旦被独立验证，密度就成了唯一的未知量，
     一次称重能定一整类零件。**
  3. **在 SolidWorks 里赋材料**后再导一次，用 XML 的绝对质量（此时基准才真正成立）。
- ★★ **先做单件验证再批量赋材料**：只给 **1 个**零件赋一个密度鲜明的材料（如钢 7850），
  重导，看该件质量是否变成原来的 **7.85 倍**。
  - 变了 → 插件确实读 SW 密度，批量赋材料有效；
  - 没变 → 插件不吃 SW 的密度，**别再花时间批量赋值**，改在外部密度表里处理。

---

## 5. Step 4 —— 导入并验收

### 5.1 准备路径

中文路径是已知坑，统一走 ASCII junction：

```
mklink /J  C:\Users\29408\exo_work  "C:\Users\29408\Desktop\外骨骼"
```

（已存在则跳过。）

### 5.2 导入

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape
addpath(pwd)

r = smimport_from_xml('D:\exo_xml\exo_master.xml')                        % 默认模型名 sm_exo_xml
r = smimport_from_xml('D:\exo_xml\exo_master.xml','ModelName','sm_exo_sw') % 换个名字方便对照
```

脚本会：`smimport` → 显式 `save_system` → 统计关节 → dump 所有刚体 → 与 URDF 路对账 → 出判定。

日志 → `simscape/sm_exo_xml_log.txt`。

### 5.3 判定表

| VERDICT | 含义 | 处置 |
|---|---|---|
| `XML_IMPORT_OK` | 导入 + 存盘成功，刚体 > 0，关节 > 0 | 继续看下面的对账数字 |
| `XML_IMPORT_NO_JOINTS` | **有刚体但一个关节都没有** | 装配体的 mates 没配好（§1 推论 1）。回 SW 检查配合关系 |
| `XML_IMPORT_NO_BODIES` | 一个带质量的刚体都没有 | 几何/材质没导出；或所有零件都没赋材质 |
| `XML_IMPORT_FAILED` | `smimport` 直接报错 | 看日志里的 `id` / `msg`；常见是路径含中文或 XML 结构不完整 |

### 5.4 对账怎么读

脚本自动比 **总质量**（与参考系无关，所以只有它能自动判）：

- 导的是**完整装配体** → 应当 ≈ **3.329496 kg**
- 导的是**子装配体**（§4.1 的小样）→ 不该等于 3.329496，此时只验证工具链

**质心与惯量只打印、不判 PASS/FAIL** —— 因为两条路的 link frame 语义不同：
URDF 路是我们**刻意对齐到世界系**的（关节 `origin rpy="0 0 0"`），
XML 路的参考系由插件按 SolidWorks 坐标系决定。拿两套语义硬比会得到假的失配。

想做**逐组**质量对账（`leg_L` / `motor_L` / `back` …）：
在 SW 里另建一个**三体简化装配体**（base = 背部设计 + 电机设计_左 + 电机设计_右 固定成一体；
再插入 腿部设计_左 / 腿部设计_右，加**同心 + 重合**配合约束成绕髋轴转动），
导出后脚本会因刚体数 ≤ 12 而自动做逐组比对。

### 5.5 与三层证据链的关系

XML 路补的是 **L1 缺的那条外部基准**：

| 层 | 手段 | 上游 | 独立性 |
|---|---|---|---|
| L1 | `I_axis` 对 STP 基准 rel 6e-12 | Python 解析 | ❌ 同源 |
| L2 | 轨迹对照 rms/range 2.6e-06 | 同一个 URDF | ❌ 同源 |
| L2′ | 能量守恒漂移 1.4e-06 | Simscape 自身 | ⚠️ 只验内部自洽 |
| **XML 对账** | **质量 / 体积 / 惯量 vs SolidWorks 内核** | **SolidWorks** | ✅ **真正独立** |
| L3 | 真机 (q, q̇) | 实测 | ✅ 仍未做 |

---

## 6. XML 路特有的坑（与 URDF 路的不通用）

1. **`ver('matlab').Release` 自带圆括号** —— R2024b 返回 `'(R2024b)'`。
   直接 `sprintf('smlink-%s-win64.zip', lower(v.Release))` 会拼出
   `smlink-(r2024b)-win64.zip` 这种不存在的文件名。
   → 用 `regexprep(v.Release,'[^A-Za-z0-9]','')` 先洗一遍（`xml_preflight.m` 已修）。

2. **`installaddon` 不是 `install_addon`** —— 网上老教程（R2020 前后）写的是
   `install_addon('smlink.r2020.win64.zip')`；R2024b 的官方文档是
   `installaddon('smlink-r2024b-win64.zip')`。文件名里的分隔符也从 `.` 变成了 `-`。

3. **插件必须挂在 `Assembly` 上导出**，零件（`.SLDPRT`）没有 mates，导出没有意义。

4. **STEP 导入的装配体不能用来导出 XML** —— 没有 mates ⇒ 没有关节（本文 §1 推论 1）。
   这是最容易踩的一条：看起来"我有 CAD"，但那个 CAD 必须是**原生装配体**。

5. **中文/空格路径**：导出目录、模型文件名、另存的 `.SLDASM` 名都保持纯 ASCII。
   本机已验证 MATLAB CLI 在中文路径下有编码问题（所以才建 junction）。

6. **默认颜色是 `#CAD1EE` 的同一种灰** —— 所有零件一个色，在 Mechanics Explorer 里
   几乎看不出结构。想要分色：改 XML 里的 `rgb` 值，或在 Simulink 里给各
   `Visual` 块换颜色。

7. **`Visual` 块也带一个 `Mass` 参数（恒为 0）** —— 想脚本化统计刚体时，
   只按"有没有 `Mass` 参数"筛会把每个 link 的 Visual 也算成刚体
   （实测 3 个真刚体被数成 6 个）。必须加 `m > 0` 过滤。
   `smimport_from_xml.m` 里已处理并注释。

8. **同一 link 子系统里有多个块带 `TranslationCartesianOffset`**
   （`InertiaOriginTransform` / `VisualOriginTransform` / `hip_L_AxisTransform` /
   `hip_L_OriginTransform` …）。**必须显式挑含 `InertiaOriginTransform` 的那个**，
   靠 `find_system` 返回顺序取第一个是脆弱的。

9. **XML 输入时 `smimport` 的第二个返回值可能非空**（`smiData` 参数数据文件），
   而 URDF 输入时恒为空串 —— 这是两条路一个可观察的差异，别拿它当错误。

10. **`installaddon` 报错 ≠ 安装失败**（源码实测）。它的最后一步调 Java
    `com.mathworks.install.command.doc.BuildSharedDocCommand` 重建文档索引；
    这步挂了，前面的 `unzip(..., matlabroot)` 和 `addpath` + `savepath` 都已完成。
    → 别急着重来，先跑 `smlink_verify` 核对文件是否落地。
    同样，`savepath` 失败只会打印 `Warning: Unable to save modified path to file`：
    本会话能用，**下次启动 MATLAB 就 `which smlink_linksw` 为空了**。

11. **zip 的名字不能被改**（源码实测）。`installaddon.m` 用
    `strtok(name,'-')` 解析出 `smlink` / `r2024b` / `win64`，再拿
    `version('-release')` 和 `computer('arch')` 逐一比对。浏览器另存为
    `smlink-r2024b-win64 (1).zip` 会让解析出 `win64 (1)` → 架构校验直接 error。
    → 下载后**先确认文件名是 `smlink-r2024b-win64.zip`**，再动手。

12. **`swaddin.xml` 不是插件清单**。它在 `resources\physmod\en\smlink\` 下，是
    i18n 消息表（`<rsccat>`，全文一条 `WindowsOnly` 文案）。真正的 SolidWorks
    注册由 `bin\win64\cl_sldwks2sm.dll` 的 `DllRegisterServer` 完成。
    → 验收要看 DLL，不看这个 XML。

---

> ⚠️ **2026-09-19 13:0x 重大更新 —— 本文若干结论已被实测推翻/升级**
>
> 冲突时一律以 **[`XML_PATH_VERDICT.md`](XML_PATH_VERDICT.md)** 为准：
> ① 0 关节的根因是「装配体本身没有配合」（35/35 零部件 `IsFixed=True`），**路径 A 拿不到运动学模型**；
> ② 「改配置名为 ASCII」这条建议已**降级** —— 配置名其实是干净的 `默认`，
>    垃圾来自插件转 ANSI 时的缓冲区 bug，已用离线修复替代，**不必动 CAD 文件**；
> ③ XML → `smimport` 已**端到端验证通过**（35 刚体，19 件质量属性逐位一致）。

## 7. 相关文件

| 文件 | 作用 |
|---|---|
| `simscape/xml_preflight.m` | 环境体检（只读）。已在本机跑通 |
| `simscape/xml_preflight_log.txt` | 体检日志 |
| `simscape/smlink_verify.m` | **装完插件后的验收**（只读）。核对 7 项落盘物 + 注册表 |
| `simscape/smlink_verify_log.txt` | 验收日志 |
| `simscape/smimport_from_xml.m` | 导入 XML + 存盘 + dump 刚体 + 对账 |
| `_xml_inspect.py` | **★ 导出后体检**：合法性/结构/逐件质量/几何引用（§4.4 判据 1·3·4） |
| `_xml_joints.py` | **★ 导出后体检**：关节段落原文 + 多口径关节计数（§4.4 判据 2） |
| `simscape/_xml_inspect.txt` | 上面脚本的输出（在项目根跑时路径不同） |
| `_xml_dump.py` | 早期版本，功能已被上面两个覆盖（保留备查） |
| `_xml_group_names.py` | 从 `_targets3.json` 导出各分组的零件清单 |
| `_xml_group_names.txt` | 分组零件清单（§2.1 对账时照着勾） |
| `simscape/exo_real_report.txt` | URDF 路的参考数字（质量/质心/惯量） |
| `simscape/stl_export_report.txt` | 各组 `vol_sum`（体积对账用） |
| `simscape/SIMSCAPE_BRIDGE_SOP.md` | 总 SOP（路径 A/B/D 的对照在这里） |

**未验证部分**：`smimport_from_xml.m` 里真正的「XML → smimport」那一段
**必须在插件装好、导出 XML 之后才能实测**（本机当前没有插件、没有 XML）。
它的**刚体 dump 逻辑**已在真实的 smimport 产物（`sm_exo_real.slx`）上验证过：
3 个真刚体、剔除 3 个 `Visual` 无质量块、总质量 3.329496 kg，
与 `exo_real_report.txt` 相对偏差 6.07e-08。
