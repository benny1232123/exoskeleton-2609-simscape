# 多维模型的 3D 怎么看（`sm_exo_multidof`）

> **一句话**：`cd` 进 `simscape` 目录 → `smimport` 已生成好的 `sm_exo_multidof.slx` 双击打开 → **点 Run**。
> 它没有 `From Workspace` 依赖，不像 `sm_exo_real_harness` 那样要预置数据。
>
> 🔴 **`open_system` 只弹出框图，3D 要等你点 Run**（或 `Ctrl+D` 更新模型）**才出来**。
> "只看到框图" 不是坏了 —— 见 §2.0。
>
> ⚠️ **唯一的坑**：必须先 `cd` 到 `simscape` 目录。URDF 里 STL 走**相对路径** `meshes/*.stl`，
> 不在这个目录下，模型**直接编译失败**（5 条 `Geometry/File Name is a file that does not exist`），
> 根本跑不到渲染那一步 —— 不是"3D 里只剩坐标系"（这一点 2026-09-20 实测纠正，见 §4.1）。

---

## 0. 先决条件

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape    % ASCII junction，绕开中文路径
addpath(pwd)
```

（本机 MATLAB 是 `E:\2024b-matlab\bin\matlab.exe`。中文路径会让 CLI 出问题，
所以用 junction `C:\Users\29408\exo_work` → `C:\Users\29408\Desktop\外骨骼`。）

### 0.1 先体检（推荐，秒级）

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape
check_3d_open        % -> VERDICT: OPEN3D_READY
```

它会查三件事，并把结果写进 `check_3d_open_log.txt`：

| 检查 | 本机实测 |
|---|---|
| `SimMechanicsOpenEditorOnUpdate` | **on**（三个模型都是）⇒ 3D **在「模型更新或仿真」时自己弹**，光打开不算 |
| File Solid 引用的 STL 路径 | `meshes/*.stl`（**相对路径**）× 5/5 存在 |
| 关节块 | `sm_exo_multidof`：4 Revolute(`hip_L/R`,`abduct_L/R`) + 2 Weld(`slide_L/R`) |

> ★ 这个开关的**官方标签原文**是
> **"Open Mechanics Explorer on model update **or simulation**"**
> （`sm.sli.configParameters.explorer.openEditorOnUpdate.Label`，在
> **模型配置参数 → Simscape Multibody → Explorer** 里）。
> 注意是 `OnUpdate` —— **`open_system` 不触发 update**，所以不会弹。

> ⚠ **参数名是 `ExtGeomFileName`，不是 `FileName`**。
> 写错不会报错、只会抛「取不到」，然后被误判成「STL 找不到」——
> 我第一次就踩了这个坑，差点得出"模型坏了"的错误结论。

---

## 1. 三个模型，别搞混

| 模型 | 是什么 | 自由度 | 文件 |
|---|---|---|---|
| `sm_exo_real` | **论文口径**：整条腿一个刚体 | 2（左右髋屈伸） | `sm_exo_real.slx` |
| `sm_exo_multidof_locked` | 拆成 3 段、但 abduct/slide 都 fixed | 2（同上） | `sm_exo_multidof_locked.slx` |
| `sm_exo_multidof` | **拆段 + abduct 可动** | **4** | `sm_exo_multidof.slx` |

* `_locked` 是**回归孪生**：拓扑一样，新关节写成 `fixed`。
  它跑出来应与 `sm_exo_real` **逐位一致**（实测末态差 `1.261e-07`）——
  这就是「按零件归属拆段没有改变动力学」的证据。
* `sm_exo_multidof` 才是真正的多维模型。**当前 4 DOF = 每腿 hip 屈伸 + abduct 髋部横向**。

### 自由度是怎么定下来的

| 关节 | 类型 | 轴向 | 依据 |
|---|---|---|---|
| `hip_L/R` | revolute | ∥ 世界 Y（髋轴） | 论文口径，原有 |
| `abduct_L/R` | **revolute** | ⊥ 髋轴，`[0.445,−0.009,0.895]` | **Ø8 黄铜衬套轴系**（`电机_轴`→`电机_黄铜轴套8_10_18`→`电机_出轴`），距髋 52 mm |
| `slide_L/R` | **fixed**（曾建 prismatic） | ∥ `[0.423,0.008,−0.906]` | 滑块↔导轨 4 组面对面贴合、法向 ⊥ 此轴 ⇒ 是移动副；**但行程仅 2 mm 装配间隙**，故忽略 |

> `strap`（绕 ∥髋轴 的那根销）**已废止**：它是**横穿滑块的防转键**，
> 键的作用就是阻止转动 —— 建 revolute 等于把被约束掉的自由度当成自由度。
> 信号：转 0.6 rad 只推段质心 3.98 mm。

---

## 2. 看 3D 的几种方式（§2.0 先看，最常见卡点）

### 2.0 ★ FAQ：为什么 `open_system` 之后**只弹出框图**，没有 3D？

**这是正常的，不是坏了。** 3D 视窗（Mechanics Explorer）不是"打开模型"时创建的，
而是 **Simscape Multibody 在模型 *更新/编译* 时**创建的：

| 你做了什么 | 框图 | 3D |
|---|---|---|
| `open_system('sm_exo_multidof')` / 双击 .slx | ✅ 弹 | ❌ **不弹** |
| 点 **Run**（`sim(...)`） | ✅ | ✅ **弹**（因为 Run 会先编译） |
| **`Ctrl+D`** 更新模型 | ✅ | ✅ **弹**（`OnUpdate` 就是这个意思） |

判据不用猜，看 Simulink 窗口**左下角状态栏**：
显示 **`就绪`** = 还没编译过 ⇒ 3D 一定还没出来；点 Run 后会变成仿真中/完成。

> 2026-09-20 实测确认：本机 `sm_exo_multidof` 的
> `SimMechanicsOpenEditorOnUpdate = on`，
> 且模型**能正常编译仿真**（`sim(mdl,'StopTime','0.5')` 在 `-batch` 下成功返回）。
> 所以"只有框图"只剩一个解释 —— **还没点 Run**。

**如果你点了 Run 还是没有 3D**，按这个顺序查（都是有实测指纹的）：

1. **看有没有红色报错弹窗。** 有 `Geometry/File Name is a file that does not exist`
   ⇒ 工作目录不对，见 §4.1。
2. **3D 窗口是不是在别的窗口后面 / 另一块屏幕。** Mechanics Explorer 是**独立顶层窗口**，
   不在 Simulink 窗口里、任务栏里也是单独一项。Windows 上按 `Alt+Tab` 找一下。
3. **仿真模式必须是「普通」。** 工具栏 `仿真` 页里那个下拉框：
   ME 不支持 `Rapid Accelerator` 等模式，官方报错
   `Simscape Multibody visualization using Mechanics Explorer is not supported in the %s
   simulation mode. Only %s and %s modes are supported.`
   本机截图里显示的是 **`普通`** ⇒ 这一条没问题。
4. **MATLAB 必须是带 Java 的桌面会话。** `-nojvm` 起的话会报
   `Mechanics Explorer requires Java to be enabled.`
   ⇒ 用 `E:\2024b-matlab\bin\matlab.exe` 正常启动即可。
5. **配置参数里被关掉了**：模型配置参数 → `Simscape Multibody` → `Explorer`
   → 勾上 **`Open Mechanics Explorer on model update or simulation`**。

### 方式 A：Simulink + Mechanics Explorer（交互，最常用）

**必须用 MATLAB 桌面（有 GUI）**——`-batch` 无显示环境里 `open_system` 只是加载，
不会渲染任何 3D。所以这条要在 MATLAB 窗口里执行，不是在 cmd 里跑。

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape     % ① ★ 这一句不能省，见下面 4.1
open_system('sm_exo_multidof')                     % ② 只弹框图，还没 3D
                                                   % ③ 点工具栏的「运行」← 3D 在这一步才弹
```

* **3D 是第 ③ 步（Run）弹出来的，不是第 ② 步。** 这是最容易卡住的地方，见 §2.0。
* 模型里 `StopTime = 10`（秒）。想看和我那份 GIF 一样的 2.5 s 自由落体，
  先把 StopTime 改成 `2.5` 再 Run —— T=0 时 CAD 零位本身就挂着力矩，
  腿会自己甩下去，10 s 会甩很久还容易撞限位。
* 在 Mechanics Explorer 里可以：拖拽旋转 / 滚轮缩放 / 右键改渲染模式
  （`Update tree` 后能按 link 单独显示或隐藏）。
* **每个段在模型里是一个子系统**，里面并排放着两个块：

  | 块 | MaskType | 管什么 |
  |---|---|---|
  | `leg_L_2/Inertia` | `Inertia` | 质量 + 惯量（**决定动力学**） |
  | `leg_L_2/Visual` | `File Solid` | 指向 `meshes/leg_L_2.stl`（**只管画**） |

  点任意一个 → 3D 里对应那一段高亮，这是核对"哪块是哪块"最快的办法。
  **滑块消失的原因就在这**：`leg_L_3/Inertia` 还在，`leg_L_3/Visual` 没了。

### 方式 B：只看结构、不跑仿真

**`Ctrl+D`（更新模型）** —— 会编译但不仿真，3D 视窗弹出来停在 q=0 的零位，
可以直接拖拽看模型长什么样，不用等 10 s 仿真跑完。

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape
open_system('sm_exo_multidof')
% 然后按 Ctrl+D（等价于命令行 update_diagram(gcs) / set_param(mdl,'SimulationCommand','update')）
```

如果不想要 GUI，导入日志 `import_sm_exo_multidof_log.txt` 里有完整的
**joint / solid 清单**，不用开 GUI 也能核对接线。
**L3 的 Visual 缺失是滑块消失的直接证据**：

```
-- joint blocks = 6 --
  abduct_L  abduct_R  hip_L  hip_R  slide_L  slide_R

-- solid-ish blocks --
  base/Inertia     base/Visual      [File Solid]
  leg_L_1/Inertia  leg_L_1/Visual   [File Solid]
  leg_L_2/Inertia  leg_L_2/Visual   [File Solid]
  leg_L_3/Inertia                   ← 只有 Inertia，没有 Visual！
  leg_R_1/Inertia  leg_R_1/Visual   [File Solid]
  leg_R_2/Inertia  leg_R_2/Visual   [File Solid]
  leg_R_3/Inertia                   ← 同上
```

### 方式 C：离线姿态图（不开 MATLAB GUI 也能看，推荐用于文档）

```matlab
% 不存在批处理入口，直接跑 Python 渲染器（不依赖 MATLAB）：
```

```powershell
C:\Users\29408\.workbuddy\binaries\python\envs\default\Scripts\python.exe `
  C:\Users\29408\Desktop\外骨骼\_multidof_render.py
```

出图 `matlab2609/out_simscape/multidof_poses.png`（6 联：零位 / hip / abduct±/ slide 无效 / 复合）。
这个渲染器**直接从 URDF 做前向运动学**，所以它和 Simscape 的位姿是一致的，
并且**遵守 `<visual>` 声明**——URDF 里没 `<visual>` 的段（= 现在的滑块 L3）它也不画。

> ★ **2026-09-20 改为「着色实体面」渲染**（原来是散点云）。
> 散点云远看像一团噪点，用户会直接说「**没看到 3D 模型**」——
> 要像模型就必须画面。做法：
> 1. **顶点聚类减面**（`decimate_cluster`，无外部依赖）：顶点按 bbox 均分的立方网格吸附、
>    同格合并，再丢退化面/重复面。`base.stl` **74.8 万面 → 565 面**，
>    `leg_L_2` 3.3 万 → 695 面，视觉轮廓保留、面数掉两个数量级。
> 2. **Lambert 着色**（`shade_faces`）：按面法向点乘固定光向，
>    `inten = 0.38 + 0.62*|n·L|`（取绝对值 ⇒ 双面可见，不会出现全黑背面）。
> 3. `Poly3DCollection` 逐面画实心，`antialiased=False`。
>
> 环境里**没有** trimesh / pyvista / vtk / open3d / plotly（只有 matplotlib + numpy + PIL），
> 所以减面只能自己写 —— 顶点聚类是性价比最高的那个（几十行、O(n)、效果够用）。

### 方式 D：离线 3D **动画**（不进 MATLAB，也能"看见它动"）

Mechanics Explorer 只能在 GUI 里看，截不了图、也分享不了。这一段动画补上这个缺口：

```powershell
C:\Users\29408\.workbuddy\binaries\python\envs\default\Scripts\python.exe `
  C:\Users\29408\Desktop\外骨骼\_multidof_anim.py
```

出两个东西：

| 文件 | 内容 |
|---|---|
| `out_simscape/multidof_drop.gif` | 60 帧、2.5 s 自由落体的循环动画 |
| `out_simscape/multidof_drop_strip.png` | 6 格胶片（t = 0 / 0.5 / 1.0 / 1.5 / 2.0 / 2.5 s），静态给文档用 |

**★ 关键点：轨迹来自 Simscape 自己导出的 `multidof_traj.csv`**，
不是另算一遍 —— 所以这动画就是 Mechanics Explorer 里那条运动的可视化，
逐位相同（同一份 `(q, q̇)`）。网格与运动学复用 `_multidof_render.py`
（含上面的**实体面渲染**），同样遵守 `<visual>` 声明（L3 滑块不画）。
动画每段减面目标 700 面（比静态图的 900 更省，因为要逐帧重画）。

动画末尾会打一条**摆幅自检**，用来确认没退化成 2-DOF：

```
hip_L       1.834666 rad  (105.12 deg)   [-1.8320, 0.0026]
abduct_L    0.406874 rad  (23.31 deg)   [0.0000, 0.4069]
hip_R       1.840289 rad  (105.44 deg)   [-1.8358, 0.0045]
abduct_R    0.427279 rad  (24.48 deg)   [-0.4273, 0.0000]
```

`abduct` 有 23°/24° 的实打实摆幅 ⇒ 动画里能看到腿的横向张开，不是只绕髋转。

---

## 3. 滑块（L3）为什么不见了

`slide` 既然按 `fixed` 忽略，那个"抱在腿杆下端的夹紧块"就只剩视觉噪声。
已在生成脚本里把它从渲染中移除：

```python
# _stp2urdf_multidof.py
SHOW_SEG = {"L1": True, "L2": True, "L3": False}
```

它**只删 `<visual>`，不删 `<inertial>`**：

```xml
<link name="leg_L_3">
  <inertial>          <!-- 仍在：m=0.028125985 kg，惯量照旧 -->
    ...
  </inertial>
  <!-- 没有 <visual> 了 -->
</link>
```

后果：

| | 变了吗 |
|---|---|
| 质量 / 惯量 / 关节 | **一个字都没变** |
| 回归 `max\|Δ\|` | `1.261e-07` → **PASS**（与隐藏前完全相同） |
| `sm_exo_multidof` 状态数 | 8（4 DOF），末态逐位不变 |
| Mechanics Explorer | 不再显示滑块 |

**想恢复显示**：把 `SHOW_SEG` 里 `"L3"` 改回 `True`，重跑
`_stp2urdf_multidof.py` → `run_remultidof`。
`meshes/leg_{L,R}_3.stl`（8290 面）一直保留在磁盘上，没删。

---

## 4. 两个必知的坑

### 4.1 一定要 `cd` 到 `simscape`

STL 是相对路径。在别的目录**根本编译不过**（2026-09-20 实测）：

```
sm:sli:setup:compile:ErrorMessages
['sm_exo_multidof']: The following errors were found in the model sm_exo_multidof.
  [1] sm:model:evaluate:FileNotExist
      ['sm_exo_multidof/base/Visual']: The parameter Geometry/File Name is a file that
      does not exist. Resolve this issue in order to simulate the model.
  [2] ... ['sm_exo_multidof/leg_L_1/Visual'] ...
  [3] ... ['sm_exo_multidof/leg_L_2/Visual'] ...
  [4] ... ['sm_exo_multidof/leg_R_1/Visual'] ...
  [5] ... ['sm_exo_multidof/leg_R_2/Visual'] ...
```

**正好 5 条**，对应 5 个 File Solid 块（L3 滑块按 `SHOW_SEG` 隐藏了 visual，所以数不到它）。
看到这 5 条就是工作目录错了，`cd` 回 `simscape` 即可，**不是模型坏了**。

> 之前这里写的是"3D 里只剩坐标系和占位方块"——**那是错的**，纠正于 2026-09-20：
> 相对路径找不到是**硬错误**（编译期就 fail），压根到不了渲染。
> 实测对照：cwd=正确的 `simscape` → `SIM OK` 且无 warning；
> cwd=错误的 `out_simscape` → 上面这 5 条。

### 4.2 `_locked` 与 `real` 的末态**不是**严格相等

是 `1.261e-07` 的浮点级差异。判据是 `< 1e-6`，不是 `== 0`。

---

## 5. 文件清单

| 文件 | 作用 |
|---|---|
| `exo_multidof.urdf` | 生成源（4 DOF，L3 无 visual） |
| `exo_multidof_locked.urdf` | 回归孪生（abduct/slide 全 fixed） |
| `exo_multidof_report.txt` | 守恒校验 + 关节轴位轴向 |
| `check_3d_open.m` | **看 3D 前的体检**（EditorOnUpdate / STL 路径 / 关节块），`check_3d_open_log.txt` |
| `import_exo_multidof.m` | `smimport` 导入 → `.slx` |
| `run_remultidof.m` | **一键**：导入 free + locked + smoke 回归 |
| `smoke_multidof.m` | 三模型各跑 0.5 s + 回归判定 |
| `smoke_multidof_log.txt` | 回归结果（`VERDICT: PASS`） |
| `_stp2urdf_multidof.py`（仓库根） | URDF 生成器（`ENABLE_SLIDE` / `SHOW_SEG` 开关在这） |
| `_multidof_render.py`（仓库根） | **运动学模块 + 离线姿态图**（可 import：`parse_urdf`/`link_fk`/`load_solid_meshes`/`decimate_cluster`/`shade_faces`；只有 `main()` 才出图） |
| `_multidof_anim.py`（仓库根） | **离线 3D 动画渲染器**（读 Simscape 的 `multidof_traj.csv` → GIF + 胶片） |
| `out_simscape/multidof_poses.png` | 成品姿态图（6 联） |
| `out_simscape/multidof_drop.gif` | 成品动画（4-DOF 自由落体，2.5 s 循环） |
| `out_simscape/multidof_drop_strip.png` | 成品胶片（6 格，静态） |

---

## 6. 一句话速查

**在 MATLAB 里交互看**（要 MATLAB 桌面，不能是 `-batch`）。就四步：

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape   % 1) 必须 cd 到这里
check_3d_open                                     % 2) 体检，应回 OPEN3D_READY
open_system('sm_exo_multidof')                    % 3) 只弹框图，正常
                                                  % 4) ★点「运行」← 3D 在这一步才弹
```

> ★ **卡点在第 4 步**：`open_system` 不会创建 3D 视窗。
> 3D 由 `SimMechanicsOpenEditorOnUpdate = on` 在**模型更新或仿真**时创建，
> 所以要么点 **Run**，要么按 **`Ctrl+D`**。状态栏显示 `就绪` 就说明还没编译过。

> 三个模型对应三种看法：`sm_exo_multidof` = 4 DOF（看新自由度，**默认看这个**）；
> `sm_exo_multidof_locked` = 2 DOF 回归孪生；`sm_exo_real` = 论文口径。
> 想复现我那份 GIF 的 2.5 s 自由落体，先 `set_param('sm_exo_multidof','StopTime','2.5')`。

**不进 MATLAB、只看图 / 看动画**（推荐先跑这个，秒级出结果）：

```powershell
$py = 'C:\Users\29408\.workbuddy\binaries\python\envs\default\Scripts\python.exe'
& $py C:\Users\29408\Desktop\外骨骼\_multidof_render.py   # -> multidof_poses.png（6 联姿态）
& $py C:\Users\29408\Desktop\外骨骼\_multidof_anim.py     # -> multidof_drop.gif + 胶片
```

要重建 / 改开关：

```matlab
run_remultidof                     % 重新导入 + 回归验证
```
