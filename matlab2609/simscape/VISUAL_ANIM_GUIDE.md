# 可视化仿真怎么跑（真实装配体 · Simscape Multibody）

> 一句话：**要交互看 3D 用 `view_exo_real`；要一份能分享的离线视频用 `anim_exo_3d`。**
> 两条路都跑同一个 harness（`sm_exo_real_harness.slx`，真 CAD 导入的 plant），
> 都显示 `base + leg_L + leg_R` 三个真实 STL 外形。

---

## 0. 先决条件（只要一次）

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape    % ASCII junction，绕开中文路径
addpath(pwd)
```

`meshes/*.stl` 是**相对路径**，按「当前目录」解析。所以必须先 cd 到 `simscape`——
否则 Mechanics Explorer 里只剩坐标系 / 占位体，看不到外形。

网格接线已核实（`attach_visual_meshes` 判决 = `VISUAL_MESH_OK`）：

| link | STL | 三角面 | ExtGeomFileName |
|---|---|---|---|
| base | `meshes/base.stl` | 748 061 | `meshes/base.stl` |
| leg_L | `meshes/leg_L.stl` | 47 606 | `meshes/leg_L.stl` |
| leg_R | `meshes/leg_R.stl` | 47 602 | `meshes/leg_R.stl` |

两个模型（`sm_exo_real` / `sm_exo_real_harness`）都已接线，
且 `SimMechanicsOpenEditorOnUpdate = on` → **仿真一跑，3D 视窗自己弹出来**。

---

## 1. 交互式：Mechanics Explorer + 时域曲线

```matlab
view_exo_real                            % 跑 1 s 受驱仿真，弹 3D + 画 q/ω
view_exo_real('Tend',4)                  % 时长拉到 4 s
view_exo_real('Tend',4,'video',true)     % 顺带用 smwritevideo 录成 MP4
view_exo_real('model','import')          % 只看纯 CAD 导入模型（结构，不跑仿真）
view_exo_real('compare',true)            % 顺带重跑 Simscape vs 解析对照
```

* 3D：Simulink 模型窗口 + Mechanics Explorer（真实外形）
* 图：`out_simscape/view_exo_real_response.png`（q、ω，左右腿两条线）
* 输出的 `max|q| / max|ω|` 直接打在命令行里
* `video=true` 走 `smwritevideo`，**需要桌面 MATLAB**；`-batch` 下会失败并提示改用 `anim_exo_3d`

---

## 2. 离线渲染：MP4 / GIF / 三联姿态图（推荐）

不依赖 GUI，相机与配色可复现，`-batch` 也能跑。

```matlab
anim_exo_3d                              % preset 'drop'（默认）
anim_exo_3d('preset','drive')
anim_exo_3d('preset','gait')
anim_exo_3d('preset','gait','view','iso')                       % 换视角
anim_exo_3d('preset','custom','Tamp',0.20,'f',0.45,'Tend',5,'fps',30)
anim_exo_3d('preset','drop','formats',{'mp4','gif'},'reduce',0.25)
anim_exo_3d('preset','drive','source','analytic')               % 不跑 Simscape，ode45 积分同一 ODE
```

### 内置 preset（实测摆幅，均已筛过「不翻转」）

| preset | T_L(t) | Tend | max&#124;q_L&#124; | max&#124;q_R&#124; | 帧数@25fps | 看点 |
|---|---|---|---|---|---|---|
| `drop` | 0（自由落摆） | 2.5 s | 1.8228 rad | 1.8231 rad | 64 | 纯重力响应，最能看出惯量/重力矩对不对 |
| `drive` | +0.15 sin(2π·0.3t) N·m | 4.0 s | 2.1167 rad | 2.1832 rad | 101 | 左右反相受驱，腿会交叉 |
| `gait` | +0.20 sin(2π·0.4t) N·m | 6.0 s | 2.4744 rad | 2.5035 rad | 151 | 3 个周期的摆腿，最像"在动"的演示 |

`T_R = -T_L`（左右反相），方便在画面上区分两条腿。

### 参数

| 名 | 说明 | 默认 |
|---|---|---|
| `preset` | `drop` / `drive` / `gait` / `custom` | `drop` |
| `Tamp` `f` `Tend` | 力矩幅值 [N·m] / 频率 [Hz] / 时长 [s] | 随 preset |
| `fps` | 帧率 | 25 |
| `source` | `simscape`（真跑 harness）/ `analytic`（ode45 积分同一 ODE） | `simscape` |
| `view` | `side`（沿髋轴看=钟摆正视）/ `iso` / `front` / `top` | `side` |
| `reduce` | 网格抽稀比例（1.0 = 不抽稀） | 1.0 |
| `formats` | `{'mp4','gif'}` | `{'mp4'}` |
| `pose` | 出三联姿态 PNG | true |

### 两种调用写法（都支持）

```matlab
anim_exo_3d('gait','view','iso')            % 位置简写：第一个参数是 preset
anim_exo_3d('preset','gait','view','iso')   % 名值对（文档推荐）
```

> ⚠ **历史 bug（2026-09-19 已修）**：旧版 `parse_args` 写成
> `if ischar(varargin{1}), preset = varargin{1}; end` —— **无条件吞掉第一个参数**。
> 于是 `('preset','gait')` 里 `'preset'` 被自己吃掉、`'gait'` 变成孤立键，
> 报 `错误使用 anim_exo_3d>parse_args (第 433 行) 未知参数 'gait'`。
> 现在只有第一个参数**恰好是 preset 名**（`drop`/`drive`/`gait`/`custom`）
> 时才按简写处理。顺带补了「名值参数必须成对」与 `preset` 类型校验。

### 输出（`matlab2609/out_simscape/`）

文件名 tag 规则：**`tag = preset`；`view` 不是默认的 `side` 时追加 `_<view>`**。
所以 `anim_exo_3d('preset','gait','view','iso')` 出的是 `exo_real_gait_iso.mp4`，
而侧视的 `exo_real_gait.mp4` 保持不动 —— 各视角互不覆盖。

* `exo_real_<tag>.mp4` / `.gif` — 动画
* `exo_real_<tag>_poses.png` — 起点 / 摆幅极值 / 终点 三联姿态
* `anim_exo_3d_<tag>.txt` — 日志：摆幅、每帧耗时、包围盒、转动符号自检

---

## 3. 两个必须知道的物理事实（别当成 bug）

### 3.1 CAD 零位不是重力平衡位

髋轴水平（世界 Y 向），腿质心到轴 `d = 0.18210 m`。URDF 给出的重力矩是

```
tau_g(q) = A sin q + B cos q ,   A = -0.341437 ,  B = -0.441845 N*m   (leg_L)
```

`B ≠ 0` 意味着**零位本来就挂着 0.442 N·m 的重力矩**。所以哪怕 `T = 0`
（preset `drop`），腿也会自由落到 `q = -1.83 rad`。这是模型的真实行为。
若拿教科书式 `M q̈ = T - G sin q` 去对，会得到 rms/range ≈ 2.17 的假性失配
（详见 `compare_simscape_vs_analytical.m` 顶部注释）。

### 3.2 harness 的关节限位是关的、阻尼是 0

* `LowerLimitSpecify = off`、`UpperLimitSpecify = off`、`DampingCoefficient = 0`
* 驱动频率靠近共振（`f_n = sqrt(|A|/I_axis)/2π ≈ 0.71 Hz`）时能量会被持续泵入
* 实测：`f=0.5 Hz, Tamp=0.35 N·m` → `max|q|` 冲到 **14.57 rad**（翻过顶连续转圈，2.3 圈）
* 内置 preset 已按「`max|q| < π`、不翻转」筛过；脚本每次都会把实测摆幅打进日志

**想要 ±0.5 rad 那种真实步态摆幅**，需要闭环重力补偿
`T = -(A sin q + B cos q) + T_swing`。本 harness 的 T 来自 `From Workspace`（开环），做不到。
两条出路：改模型接线，或离线自己造 T(t) 再喂给 `anim_exo_3d('source','analytic',...)`。

> ⚠ 顺带一个坑：**用常量偏置去"抵消"重力矩是行不通的**。定值力矩给势能加了个
> 线性项（`U_eff = 0.5545 sin(q - 0.6597) - Tdc·q`），周期性被破坏，
> `q ≈ 0` 从稳定平衡点变成**不稳定**平衡点，腿会朝一个方向一直滚走。
> （实测 `Tdc = 0.4383, Tamp = 0.10, f = 0.3` → `max|q| = 180 rad`。）

---

## 4. 渲染管线自检（为什么动画可信）

`anim_exo_3d` 每次都会做**转动符号自检**，不通过就拒绝出片：

* 取腿质心（URDF 给的相对关节原点偏移），按候选符号 `±q` 各转一次，算世界系 z 分量
* 与解析式 `z_com(q) = C0 + W cos q + V sin q` 比对，取误差小的那个符号
* 实测 `s = +1`，`|e|max = 6.368e-10 m`（两个角度 0.7 / -1.2 rad 都查）
* 残差 > `1e-9 m` 直接 `error` —— 这条同时钉死了
  **「STL 世界系顶点 + 关节轴 + 转动符号」三者对齐**

其它已踩过的坑（都写进了脚本注释）：

| 现象 | 真因 |
|---|---|
| `stlread` 报找不到 `base.stl.stl` | 拼扩展名时忘了去掉 `fileparts` 里的 `.stl` |
| `Matrix 属性的值无效` | 帧时刻用了 `round(linspace(...))`，末点 `round(2.5)=3 > Tend`，`interp1` 外插出 `NaN` |
| 腿尖被切出画面 | 扫掠角只取了 `[min(q) max(q)]` 两个端点，漏掉"竖直朝下"的中间位姿 |
| 底座被切掉 | 包围盒只扫了腿，没含 `base.V` |
| 长条 axes 里整体被裁 | `axis equal` + `axis vis3d` 会放大箱体填满；应改 `daspect([1 1 1])` + `pbaspect(包围盒边长比)` |
| `analytic` 报「要串联的数组的维度不一致」 | `Tfun(t).'` 在标量 t 下把 2×1 转成 1×2，与 `A.*sin(y(1:2))` 广播成 2×2 |

---

## 5. 文件清单

| 文件 | 作用 |
|---|---|
| `anim_exo_3d.m` | **离线 3D 动画渲染器**（MP4/GIF/姿态图 + 转动符号自检） |
| `view_exo_real.m` | **交互式查看器**（Mechanics Explorer + q/ω 图 + 可选录屏） |
| `attach_visual_meshes.m` | 核验 / 修正模型里的 `<visual>` 网格接线 |
| `probe_viz_ready.m` | 只看不写：两个模型的 STL 接线 + From/To Workspace + Mechanice Explorer 参数 |
| `probe_anim_inputs.m` | 只看不写：顶层块结构 + 三个 STL 包围盒 |
| `out_simscape/exo_real_*.mp4/.gif/_poses.png` | 成品 |
| `_sweep_drive.py` / `_sweep_dc.py` | 选 preset 参数用的扫参脚本（证明"不翻转"是筛出来的，不是碰运气） |
