# 4-DOF 解析动力学 —— 与 2-DOF 论文口径**并存**

> **一句话**：`+exo2609` 的论文口径（2-DOF、每关节解耦）**一个字都没动**；
> 这里新增的是一套**通用多刚体拉格朗日**解析实现，用来对
> `simscape/exo_multidof.urdf`（每腿 hip + abduct）做交叉验证。
>
> 需要 4-DOF 时用 `exo2609.multidof`（MATLAB）或 `_multidof_dyn.MultiBody`（Python）；
> 需要复现论文时继续用 `exo2609.params_paper` / `exo2609.dyn_f`（2-DOF，原样）。

---

## 1. 为什么不能沿用 2-DOF 那种形式

论文口径（`+exo2609`）是**每关节解耦**的：

```
I_i * qdd_i = T_i + A_i*sin(q_i) + B_i*cos(q_i)
```

这在 2-DOF 下成立：左右髋各绕一条 ∥Y 的水平轴，两腿互不耦合，
单轴旋转的重力矩就是 sin/cos 的线性组合。

但 `exo_multidof.urdf` 加了 `abduct`（髋部横向）自由度，它有两个性质：

1. 轴 `[0.445, −0.009, 0.895]` **不平行任何全局轴**；
2. 它**固定在 L1 段上，随 hip 一起转**。

于是：

* 质量矩阵**不再对角**（实测零位就有 `M(hip,abd) = 7.263e−04` 的耦合）；
* 重力矩**不再**是单个 sin/cos 能表达的。

必须退回通用形式：

```
M(q) qdd + C(q,qd) qd + G(q) = Q_ext
```

> ★ **耦合项不能省**。零位下若用 `qdd_abd ≈ −G_abd/M_abd` 粗略估，
> 得 `+3.12 rad/s²`；而正确值（解 2×2 块）是 `+4.761 rad/s²` —— **差 53%**。
> 这正是"解耦模型在 4-DOF 上不成立"的直接证据。

---

## 2. 怎么组装（不引入任何手写矩阵）

输入**只有 URDF**。流程：

| 步骤 | 公式 |
|---|---|
| 前向运动学 | `T_child = T_parent · [R(axis_j, q_j)  o_j; 0 1]` |
| 雅可比 | 对祖先链上的**可动**关节 j：`Jw[:,j] = a_j^world`，`Jv[:,j] = a_j^world × (p_com − o_j^world)` |
| 质量矩阵 | `M(q) = Σ_k [ m_k Jvᵀ Jv + Jwᵀ (R_k I_k R_kᵀ) Jw ]` |
| 重力广义力 | `U(q) = Σ m_k g z_k(q)`，`G(q) = ∂U/∂q`（中心差分） |
| 科氏项 | `[C qd]_i = Σ ∂M_ij/∂q_k qd_j qd_k − ½ Σ ∂M_jk/∂q_i qd_j qd_k`（Christoffel 的数值版） |

**符号约定**（与 `compare_simscape_vs_analytical.m` 一致）：`G(q) = −tau_g(q)`
⇒ 2-DOF 退化时 `G(q)` 应等于 `−(A sin q + B cos q)`。这条就是下面判据 A 的来源。

---

## 3. 三个**互相独立**的实现

同一套力学，三条互不相干的代码路径 —— 吻合才有说服力：

| 实现 | 位置 | 用什么算 |
|---|---|---|
| **Simscape Multibody** | `sm_exo_multidof.slx` | Simscape 自己的多体引擎 |
| **Python 组装器** | `_multidof_dyn.py`（仓库根） | numpy，Jacobian 组装 |
| **MATLAB 类** | `+exo2609/multidof.m` | MATLAB，同一算法的独立实现 |

Python 与 MATLAB 两侧都从 `plant_multidof.json` 取**同一份刚体参数**
（由 `_multidof_export.py` 从 URDF 派生，**只含 URDF 已有的量**，
不含任何动力学结果）—— 所以"独立"指的是算法，不是数据源。

---

## 4. 怎么用

### 4.1 Python

```python
import sys; sys.path.insert(0, r"C:\Users\29408\Desktop\外骨骼")
from _multidof_dyn import MultiBody

mb = MultiBody(r"...\matlab2609\simscape\exo_multidof.urdf")
mb.q_names            # ['hip_L', 'abduct_L', 'hip_R', 'abduct_R']
M   = mb.M(q)               # 4x4
G   = mb.G(q)               # ∂U/∂q
qdd = mb.qdd(q, qd, T)      # 正动力学
KE, PE, E = mb.energy(q, qd)          # ← Python 侧不存在"只取首个输出"的坑
tau = M @ qdd + mb.coriolis(q, qd) + G   # 逆动力学（见 4.4）
```

自检（三重判据，一次跑完）：

```powershell
C:\Users\29408\.workbuddy\binaries\python\envs\default\Scripts\python.exe `
  C:\Users\29408\Desktop\外骨骼\_multidof_dyn.py
```

### 4.2 MATLAB

```matlab
addpath('C:\Users\29408\exo_work\matlab2609')
mb  = exo2609.multidof();        % 读 simscape/plant_multidof.json
M   = mb.massmatrix(q);
G   = mb.gravity(q);
qdd = mb.accel(q, qd, T);
[~,~,E] = mb.energy(q, qd);      % ⚠ 三个输出 [KE,PE,E]，只写 E=... 会拿到 KE
```

**★ 想先看「这东西到底能干什么」，跑这一个（秒级，五个用法一次演示完）：**

```matlab
cd C:\Users\29408\exo_work\matlab2609\simscape
demo_multidof               % [1]查参数 [2]正动力学 [3]逆动力学 [4]能量 [5]正运动学
```

输出落在 `simscape/demo_multidof_log.txt`，可以直接照着改。

验证（三个脚本，由快到慢）：

```matlab
quick_multidof_check        % 语法 + 加载 + M(0)/G(0) 基准 + 初始加速度（秒级）
test_multidof_analytical    % 上面 + ode45 轨迹 vs Simscape + 能量守恒（分钟级）
dump_multidof_traj          % 重新导出 Simscape 轨迹（改了 Simscape 模型才需要）
```

### 4.3 逆动力学（给定轨迹，算需要多大力矩）

`tau = M(q)*qdd + C(q,qd)*qd + G(q)`，一行：

```matlab
Tau = mb.massmatrix(q)*qdd + mb.coriolis(q, qd) + mb.gravity(q);   % 4x1
```

> **前提**：`(q, qd, qdd)` 必须是**同一条轨迹**在同一时刻的值（自己给 `q(t)` 解析求导，
> 或对测量数据做二阶差分）。只给 `q` 和 `qdd` 而随手填 `qd` 会算出错力矩。
> 实测 `q_i = 0.2 sin(2π·0.5t)` 这条轨迹：`hip` 峰值约 **0.47 N·m**，
> `abduct` 约 **0.04 N·m** —— abduct 的力矩量级比 hip 小一个数量级，符合几何直觉。
> 这个 `tau` 是**净关节力矩**，未含摩擦 / 传动损耗。

### 4.4 完整重建链

```powershell
# 1) URDF -> JSON 参数
python C:\Users\29408\Desktop\外骨骼\_multidof_export.py
```
```matlab
% 2) Simscape 侧导出轨迹 + 解析侧验证
cd C:\Users\29408\exo_work\matlab2609\simscape
dump_multidof_traj
quick_multidof_check
```
```powershell
# 3) Python 解析 vs Simscape 逐通道比对（出图）
python C:\Users\29408\Desktop\外骨骼\_multidof_compare.py
```

---

## 5. 验证结果

### 5.1 判据 A —— 2-DOF 退化一致性 ★ 最强

把**同一个组装器**喂 `exo_real.urdf`（2 个 hip 关节），结果必须与
`_stp2plant.py` 的**闭式解**吻合。两条推导完全独立（一个 Jacobian 组装、
一个 sin/cos 仿射展开）：

| 量 | 结果 |
|---|---|
| `M(0)_ii` vs `I_axis` | 相对 **2.46e−08 / 2.05e−08**（地板由 CSV 的 9 位小数决定 ~3e−8） |
| `G(q)` vs `−(A sin q + B cos q)`，q∈[−π,π] 51 点 | max **2.232e−09** |
| 零位 `G(0)` vs `−B` | **7.01e−10 / 9.80e−11** |

### 5.2 判据 B —— 4-DOF 结构性质

| 项 | 结果 |
|---|---|
| 总质量 | 3.329495573 kg |
| `M(q)` 对称性 | max\|M−Mᵀ\| = **0**（精确） |
| `M(q)` 正定 | 20 随机位姿最小特征值 **2.0307e−03** |
| 零位对角 | `[0.016997395, 0.011527605, 0.016997437, 0.011527226]` |
| hip↔abduct 耦合 | `M(hip,abd) = 7.263e−04`（非零 ⇒ 解耦模型不成立） |
| 零位重力矩 | hip `+0.4418`，abduct `−0.0360 / +0.0434` N·m（**不是 0**） |

### 5.3 判据 C —— 能量守恒

自由落体（T=0，RK4，dt=1e−4，0.5 s）。**两侧独立实现给出同一结果**：

| 量 | Python | MATLAB |
|---|---|---|
| `E0` | −0.083166247686 J | −0.083166247686 J |
| `max\|ΔE\|` | **2.474e−10 J**（相对 2.97e−09） | **2.437e−10 J**（相对 2.93e−09） |

> `E0` 两侧**逐位相同**（−0.083166247686）本身就是一条强交叉验证：
> Python `energy()` 与 MATLAB `energy()` 是两个独立写的函数，
> 势能基准一致 ⇒ 两边质点质量与位置口径完全同源。

> 判据取 1e−7 而非机器精度：`G(q)` 与 `∂M/∂q` 都用中心差分（h=1e−7），
> 舍入 ~`eps·U/h` ≈ 1e−9 相对，每步 `qdd` 都带这个量级的非保守伪力。
> 模型若真写错，漂移会是 O(1)，与这个地板差 9 个数量级 —— 判据依然有效。

### 5.4 判据 D —— 与 Simscape 逐通道对照

`_multidof_compare.py`：Simscape 4763 点，t∈[0, 2.5]，限位已关，g=[0 0 −9.81]。

| channel | max\|err\| | rms(err) | range(sim) | rms/range |
|---|---|---|---|---|
| `hip_L` | 7.4888e−06 | 3.1632e−06 | 1.834666 | 1.7241e−06 |
| `abduct_L` | 2.0715e−06 | 7.8792e−07 | 0.406874 | 1.9365e−06 |
| `hip_R` | 7.5061e−06 | 3.1729e−06 | 1.840289 | 1.7241e−06 |
| `abduct_R` | 1.9728e−06 | 7.7724e−07 | 0.427279 | 1.8190e−06 |
| `d_hip_L` | 4.4655e−05 | 1.8240e−05 | 10.028444 | 1.8189e−06 |
| `d_abduct_L` | 1.6550e−05 | 5.3576e−06 | 2.560752 | **2.0922e−06** ← worst |
| `d_hip_R` | 4.4865e−05 | 1.8302e−05 | 10.063257 | 1.8187e−06 |
| `d_abduct_R` | 1.6152e−05 | 5.2070e−06 | 2.665044 | 1.9538e−06 |

末态 q —— Simscape `[−0.303565, 0.370363, −0.294993, −0.385711]`，
解析 `[−0.303572, 0.370363, −0.295000, −0.385711]`。

**worst rms/range = 2.09e−06**（阈值 1e−2，沿用 2-DOF 那条链）→ **OK**。
图：`out_simscape/compare_multidof.png`（上半 Simscape 红虚线 / 解析黑实线，
下半残差，全程 ≤7.5e−06 rad）。

> ★ 摆幅对照证明自由度**真被激励**：`abduct_L` Simscape 0.406874 / 解析 0.406874 rad，
> `abduct_R` 0.427279 / 0.427279 rad。若 abduct 摆幅≈0 就说明模型已退化成 2-DOF、
> 这个对照毫无意义 —— 现在是 0.41 / 0.43 rad，所以是一次**真实有效的 4-DOF 验证**。

### 5.5 判据 E —— MATLAB 类 vs Simscape 初始加速度

`quick_multidof_check.m` 把 Simscape 轨迹前几个点反推的初始加速度，
与 `exo2609.multidof.accel(0,0,0)` 逐位比：

| joint | analytic | simscape | rel |
|---|---|---|---|
| `hip_L` | −26.198778 | −26.198778 | 8.87e−10 |
| `abduct_L` | +4.771687 | +4.771687 | 6.55e−10 |
| `hip_R` | −26.225580 | −26.225580 | 6.36e−11 |
| `abduct_R` | −5.419238 | −5.419238 | 1.06e−10 |

`checkcode` **0 条**，VERDICT `QUICK_OK`。

### 5.6 总判据一览

| 命令 | 结论 |
|---|---|
| `python _multidof_dyn.py` | `ALL_OK`（判据 A/B/C） |
| `matlab quick_multidof_check` | `QUICK_OK` |
| `matlab test_multidof_analytical` | `TEST_MULTIDOF_ALL_OK` |
| `python _multidof_compare.py` | `COMPARE_MULTIDOF_OK` |
| `matlab smoke_multidof` | `VERDICT: PASS`（拆段回归 real vs locked = **1.023e−08**） |
| `matlab rerun_after_density` | `RERUN_ALL_OK`（模型参数变更后的 8 步重放 + 4 条判据复核） |

> **数值基准日期 = 2026-09-20**（`电机_轴` 由兜底铝 2700 改判钢 7850 之后）。
> 密度表任何改动都会改变本节全部数值 —— 改动后按 §5.6 最后一行整套重跑。

---

## 6. 必须知道的几个坑

### 6.1 Simscape 的状态顺序**不是**「先所有 q 再所有 w」

实测 `sm_exo_multidof` 的 `xout` 是按块名字母序**交错**：

```
col1..8 = abduct_L.q, abduct_L.w, abduct_R.q, abduct_R.w,
          hip_L.q,    hip_L.w,    hip_R.q,    hip_R.w
```

而解析侧的顺序是 `[hip_L, abduct_L, hip_R, abduct_R]`。
**必须按列名映射**，不能假设顺序。

> 踩过的坑：曾从 0.5 s 末态数值倒推顺序，得出 `[hip_L, abduct_L, hip_R, abduct_R]`
> —— 看着自洽，其实是数值巧合，**是错的**。所以 `dump_multidof_traj.m`
> 会把真实列名写进 CSV 表头。

### 6.2 `M(q)` 与 `q_hip` **无关**，别误判成 bug

绕 hip 轴转 `q_hip` 是整条腿绕该轴刚性旋转 —— 沿轴的分量不变，
而 abduct 轴固定在 L1 上、跟着一起转，相对关系也没变。
所以 `M` 对 `q_hip` 是**常量**。

真正让 `M` 变的是 `q_abd`：

| q_abd | M(hip,hip) | M(abd,abd) | M(hip,abd) |
|---|---|---|---|
| 0.00 | 0.016997395 | 0.011527605 | +0.000726348 |
| 0.30 | 0.017578532 | 0.011527605 | −0.000230886 |
| 0.60 | 0.015787266 | 0.011527605 | −0.001167495 |
| 0.90 | 0.012167856 | 0.011527605 | −0.001999815 |

（`M(abd,abd)` 对 `q_abd` 也不变 —— abduct 是绕自身轴。）

### 6.3 两侧重力必须一致

Simscape 的 `MechanismConfiguration` 默认重力是 **`[0 0 −9.80665]`**，
而解析侧用 `9.81`。`dump_multidof_traj.m` 会把模型改成 `[0 0 −9.81]`
—— 不改的话有 0.03% 的系统性偏差。

### 6.4 关节硬限位必须关（否则会造出「假 FAIL」）

URDF 的 `<limit>` 在 Simscape 里是 1e4 N·m/deg 的硬弹簧，解析模型没有它。
轨迹一旦碰到 ±1.5 rad 两边必然发散。
`dump_multidof_traj.m` 会设 `LowerLimitSpecify/UpperLimitSpecify = 'off'`。
（`slide_*` 是 Weld Joint，没有这个参数，不是错误。）

> ★ **2026-09-20 实测教训**：`smoke_multidof.m` 原先**没有**关限位，于是
> 「拆段回归」（real vs locked）报出 `max|ΔxT| = 1.681e−03` → `VERDICT: FAIL`。
> 但同一对模型交给 **Python 组装器**去比，M(0) 差 8e−9、RK4 末态差 **1.699e−09**
> —— 拆分在动力学上是精确的。1e−3 的差全部来自「撞限位后的数值灵敏度」。
> 修法：`smoke_multidof.m` 现在对每个模型先跑 `prep_for_dynamics()`
> （重力 → `[0 0 −9.81]`、关节限位全关、求解器 `ode23t/MaxStep 1e−3/RelTol 1e−6`），
> 之后回归值回到 **1.023e−08 = PASS**。
> ⇒ **凡是要判「动力学是否等价」的仿真，都必须先把硬限位关掉**；
> 需要真实机器人行为时再单独打开。

### 6.4b 判据脚本失败时**不抛异常** —— 驱动脚本必须回读日志

`compare_simscape_vs_analytical` / `smoke_multidof` / `quick_multidof_check` /
`test_multidof_analytical` 失败时**只往自己的 log 里写 FAIL 字样**，不 error。
所以 `rerun_after_density.m` 里 `try/catch` 的「OK」只代表「没抛异常」，
**必须**再回读各 log 核对终端 verdict 串（见该脚本「判据复核」段）。
> 踩过的坑：那段的 fail token 一开始写成 `'COMPARE_'`，而它正是 pass token
> `'COMPARE_OK'` 的**前缀** ⇒ 永远判 FAIL。**pass/fail token 必须互不包含。**

### 6.5 MATLAB 多输出陷阱：`E = mb.energy(q,qd)` 拿到的是动能

`multidof.energy` 与 Python 侧同签名 `[KE, PE, E]`。MATLAB 里
`E = mb.energy(q,qd)` **只取第一个输出**，即 `KE`。把动能当总能量去做
守恒判据必然 FAIL（2026-09-20 实测 drift 0.447 J ≈ 整个摆动过程的动能峰值）。
**一律写 `[~,~,E] = mb.energy(...)`。**

### 6.6 别把 `joint_frames` 的签名写回 `(obj, q, T)`

轴/原点只由**父链**位姿决定，与关节自身转角无关，所以签名是 `(obj, T)`。
好处是 `jacobians` / `massmatrix` / `gravity` 复用同一份 `fk` 结果 ——
`coriolis` 会对 `M` 做 `2*nq` 次中心差分，每次都要过 `jacobians`，
多调一次 `fk` 会让自检从秒级变成分钟级（曾为此杀过一次 3 分钟的 run）。

---

## 7. 文件清单

| 文件 | 作用 |
|---|---|
| `_multidof_dyn.py`（仓库根） | **Python 组装器** + 三重自检（判据 A/B/C）+ 写出基准 JSON |
| `_multidof_export.py`（仓库根） | URDF → `plant_multidof.json` |
| `_multidof_compare.py`（仓库根） | Python 解析 vs Simscape 逐通道比对 + 出图（判据 D） |
| `+exo2609/multidof.m` | **MATLAB 组装器**（classdef） |
| `simscape/demo_multidof.m` | **用法速查 demo**（5 个典型用法，秒级，可直接抄改） |
| `simscape/quick_multidof_check.m` | 快速验证（语法/加载/M(0)/G(0)/初始加速度 = 判据 E） |
| `simscape/test_multidof_analytical.m` | 完整验证（含 ode45 轨迹 vs Simscape） |
| `simscape/smoke_multidof.m` | 三模型短仿真 + **拆段回归**（locked vs 2-DOF） |
| `simscape/dump_multidof_traj.m` | 导出 Simscape 自由落体轨迹（带列名） |
| `simscape/rerun_after_density.m` | ★ 密度表/URDF 变更后**整套重跑** + 回读日志复核 verdict |
| `simscape/plant_multidof.json` | 派生的刚体参数（机器可读） |
| `simscape/multidof_baseline.json` | M(0) 对角 / G(0) 基准（**Python 写出、MATLAB 读**，用来杜绝硬编码副本漂移） |
| `out_simscape/multidof_traj.csv` | Simscape 轨迹 |
| `out_simscape/compare_multidof.png` | 对照图 |

### 不动的文件（论文口径，**不要碰**）

`+exo2609/params_paper.m`、`dyn_f.m`、`dyn_step.m`、`dyn_rollout.m`、`dyn_jac.m`、
`estimate.m`、`spectrum.m`、`weights_paper.m`、`q_blocks.m`、`cost_grad.m`、
`gait_ref.m`、`params_from_csv.m`、`discretization_info.m`
以及 `compare_simscape_vs_analytical.m`。
