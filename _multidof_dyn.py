# -*- coding: utf-8 -*-
"""
_multidof_dyn.py —— 从 URDF 直接组装「多刚体拉格朗日动力学」
================================================================

为什么要这个
------------
`+exo2609` 的解析模型（paper 口径）是**每关节解耦**的：

    I_i * qdd_i = T_i + A_i*sin(q_i) + B_i*cos(q_i)

这在 2-DOF（左右髋各绕一条 ∥Y 的水平轴）下成立，因为两腿没有耦合、
且单轴旋转的重力矩就是 sin/cos 的线性组合。

但 `exo_multidof.urdf` 有 4 个自由度（每腿 hip 屈伸 + abduct 髋部横向），
**abduct 轴不平行于任何全局轴、且它随 hip 一起转** ⇒ 质量矩阵不再对角、
重力矩不再能写成单个 sin/cos。所以必须退回**通用多刚体形式**：

    M(q) qdd + C(q,qd) qd + G(q) = Q_ext

本模块就做这件事：**只以 URDF 为唯一真源**，不引入任何手写矩阵。

怎么组装（全部解析，不用符号工具箱）
------------------------------------
1) 前向运动学   T_child = T_parent · Trans(o_j) · R(axis_j, q_j)
   （与本工程统一口径一致：所有 link frame 与世界系轴对齐，joint rpy = 0）

2) 雅可比       对 link k 的祖先链上每个**可动**关节 j：
                    Jw_k[:, j] = a_j^world                        （角速度）
                    Jv_k[:, j] = a_j^world × (p_k − o_j^world)    （质心线速度）
                 不在链上的列为 0。

3) 质量矩阵     M(q) = Σ_k [ m_k Jv_kᵀ Jv_k + Jw_kᵀ (R_k I_k R_kᵀ) Jw_k ]
                其中 R_k = T_k 的旋转部分，I_k 是 URDF 给的**绕质心**惯量。

4) 重力广义力   U(q) = Σ_k m_k·g·z_k(q)（z 向上为正）
                G(q) = ∂U/∂q  —— 中心差分（解析求导对链式 q 很啰嗦，
                数值差分在这个尺度上是机器精度级，且不会写错符号）。

5) 科氏/离心    直接对 M(q) 数值求 ∂M/∂q，用标准恒等式
                    [C qd]_i = Σ_{j,k} ∂M_ij/∂q_k · qd_j·qd_k
                              − ½ Σ_{j,k} ∂M_jk/∂q_i · qd_j·qd_k
                （即 Christoffel 的数值版；不需要符号导数）

★ 符号约定（与 compare_simscape_vs_analytical.m 对齐）
    M qdd + C qd + G = Q_ext        ← 本模块
    I qdd = T + tau_g               ← compare 的写法
  ⇒  G(q) = −tau_g(q)
  2-DOF 退化自检就用这条：本模块算出的 G 应等于 −(A sin q + B cos q)。
  这是**独立于本模块**的判据（A/B 来自 _stp2plant.py 的闭式解）。

用法
----
    from _multidof_dyn import MultiBody
    mb = MultiBody(urdf_path)
    mb.nq, mb.q_names
    M  = mb.M(q);  G = mb.G(q);  U = mb.U(q)
    qdd = mb.qdd(q, qd, T)        # 正动力学
    E  = mb.energy(q, qd)         # KE + PE，用于守恒自检
"""
import io
import json
import os
import xml.etree.ElementTree as ET

import numpy as np

G_ACC = 9.81          # 必须与 Simscape MechanismConfiguration 的 GravityVector 一致


# ------------------------------------------------------------------ 小工具
def skew(a):
    return np.array([[0.0, -a[2], a[1]],
                     [a[2], 0.0, -a[0]],
                     [-a[1], a[0], 0.0]])


def rot_axis(a, q):
    """Rodrigues 旋转矩阵。a 不必归一（内部归一）。"""
    a = np.asarray(a, float)
    a = a / np.linalg.norm(a)
    K = skew(a)
    return np.eye(3) + np.sin(q) * K + (1.0 - np.cos(q)) * (K @ K)


def trans(v):
    T = np.eye(4)
    T[:3, 3] = v
    return T


def homog(R=np.eye(3), p=(0.0, 0.0, 0.0)):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def inertia_from_urdf(el):
    """<inertia .../> -> 3x3（绕质心，link frame 方向）"""
    g = {k: float(el.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
    return np.array([[g["ixx"], g["ixy"], g["ixz"]],
                     [g["ixy"], g["iyy"], g["iyz"]],
                     [g["ixz"], g["iyz"], g["izz"]]], float)


# ------------------------------------------------------------------ 主类
class MultiBody:
    def __init__(self, urdf_path, gravity=G_ACC, verbose=False):
        self.path = urdf_path
        self.g = float(gravity)
        root = ET.parse(urdf_path).getroot()

        # ---- links ----
        self.links = []          # 拓扑无关的顺序，只为遍历
        self.link = {}           # name -> dict(mass, cog, I)
        for l in root.findall("link"):
            nm = l.get("name")
            ine = l.find("inertial")
            if ine is None:
                raise ValueError("link %s 没有 <inertial>（本工程要求每段都有）" % nm)
            mass = float(ine.find("mass").get("value"))
            cog = np.array([float(x) for x in
                            ine.find("origin").get("xyz").split()], float)
            I = inertia_from_urdf(ine.find("inertia"))
            self.link[nm] = dict(mass=mass, cog=cog, I=I)
            self.links.append(nm)

        # ---- joints ----
        self.joints = []
        for j in root.findall("joint"):
            nm = j.get("name")
            typ = j.get("type")
            par = j.find("parent").get("link")
            chi = j.find("child").get("link")
            o = np.array([float(x) for x in
                          j.find("origin").get("xyz").split()], float)
            ax = j.find("axis")
            axis = None if ax is None else np.array(
                [float(x) for x in ax.get("xyz").split()], float)
            rpy = j.find("origin").get("rpy", "0 0 0").strip()
            assert rpy in ("0 0 0", "", "0.0 0.0 0.0"), \
                "本工程口径要求 joint rpy = 0 0 0（link frame 与世界系轴对齐），实际 %r" % rpy
            movable = typ in ("revolute", "continuous", "prismatic")
            if movable and axis is None:
                raise ValueError("可动关节 %s 没有 <axis>" % nm)
            if typ == "prismatic":
                raise NotImplementedError(
                    "本工程当前把 slide 建成 fixed；若将来启用 prismatic，"
                    "需要在 fk()/jacobians() 里加平移分支（两者都要改）。")
            self.joints.append(dict(name=nm, type=typ, parent=par, child=chi,
                                    o=o, axis=axis, movable=movable, qidx=None))
            lim = j.find("limit")
            if lim is not None:
                self.joints[-1]["lo"] = float(lim.get("lower"))
                self.joints[-1]["hi"] = float(lim.get("upper"))

        # ---- 自由度编号：按 URDF 里 joint 出现的顺序（可动者）----
        q = 0
        for j in self.joints:
            if j["movable"]:
                j["qidx"] = q
                q += 1
        self.nq = q
        self.q_names = [None] * q
        for j in self.joints:
            if j["movable"]:
                self.q_names[j["qidx"]] = j["name"]

        # ---- 拓扑 ----
        self._order = self._topo_order()
        self._anc = self._ancestor_joints()

        if verbose:
            print("MultiBody: %d links, %d joints (%d movable: %s)"
                  % (len(self.links), len(self.joints), self.nq,
                     ", ".join(self.q_names)))

    # -------------------------------------------------------------- 拓扑
    def _topo_order(self):
        """返回关节的处理顺序，保证父 link 的位姿已算好。"""
        children = {j["child"] for j in self.joints}
        roots = [l for l in self.links if l not in children]
        order, placed = [], set(roots)
        pending = list(self.joints)
        while pending:
            progressed = False
            rest = []
            for j in pending:
                if j["parent"] in placed:
                    order.append(j)
                    placed.add(j["child"])
                    progressed = True
                else:
                    rest.append(j)
            pending = rest
            if not progressed:
                raise ValueError("关节不成树（有环或父 link 缺失）: %s"
                                 % [j["name"] for j in pending])
        return order

    def _ancestor_joints(self):
        """child link -> 从根到它的**可动**关节列表（按链序）。"""
        parent_joint = {}
        for j in self.joints:
            parent_joint[j["child"]] = j
        children = {j["child"] for j in self.joints}
        chain = {l: [] for l in self.links if l not in children}

        def build(l):
            if l in chain:
                return chain[l]
            j = parent_joint.get(l)
            chain[l] = [] if j is None else build(j["parent"]) + [j]
            return chain[l]

        for l in self.links:
            build(l)
        return {l: [j for j in ch if j["movable"]] for l, ch in chain.items()}

    # -------------------------------------------------------------- 运动学
    def fk(self, q):
        """返回 {link: 4x4 世界位姿} 与 {joint_name: (o_world, a_world)}。"""
        q = np.asarray(q, float)
        assert q.shape == (self.nq,), "q 应为 %d 维，实际 %s" % (self.nq, q.shape)
        T = {}
        children = {j["child"] for j in self.joints}
        for l in self.links:
            if l not in children:
                T[l] = np.eye(4)          # 根（本工程只有 base），frame ≡ 世界系
        jw = {}
        for j in self._order:
            Tp = T[j["parent"]]
            o_w = Tp[:3, :3] @ j["o"] + Tp[:3, 3]
            if j["movable"]:
                a_w = Tp[:3, :3] @ j["axis"]
                a_w = a_w / np.linalg.norm(a_w)
                Tj = trans(j["o"]) @ homog(rot_axis(j["axis"], q[j["qidx"]]))
            else:
                a_w = None
                Tj = trans(j["o"])
            T[j["child"]] = Tp @ Tj
            jw[j["name"]] = (o_w, a_w)
        return T, jw

    def _jac_from(self, T, jw):
        """由已算好的 fk 结果组装每个 link 的 (Jv, Jw)，各 3 x nq。

        ★ 必须接受 T/jw 而不是自己再调 fk()：M() 里每调用一次 fk 都要重建
        6 个 4x4 矩阵 + 归一化轴向，数值差分的 O(nq²) 次调用下这是唯一瓶颈。
        （第一版就写成了重复调用，25000 步 RK4 跑 3 分钟还没完。）
        """
        out = {}
        for l in self.links:
            Jv = np.zeros((3, self.nq))
            Jw = np.zeros((3, self.nq))
            p = T[l][:3, :3] @ self.link[l]["cog"] + T[l][:3, 3]
            for j in self._anc[l]:
                o_w, a_w = jw[j["name"]]
                Jw[:, j["qidx"]] = a_w
                Jv[:, j["qidx"]] = np.cross(a_w, p - o_w)
            out[l] = (Jv, Jw)
        return out

    def jacobians(self, q):
        T, jw = self.fk(q)
        return self._jac_from(T, jw)

    def com_world(self, q):
        T, _ = self.fk(q)
        return {l: T[l][:3, :3] @ self.link[l]["cog"] + T[l][:3, 3]
                for l in self.links}

    # -------------------------------------------------------------- 动力学
    def M(self, q):
        """质量矩阵 nq x nq（对称正定）。"""
        T, jw = self.fk(q)
        J = self._jac_from(T, jw)
        Mx = np.zeros((self.nq, self.nq))
        for l in self.links:
            d = self.link[l]
            if d["mass"] == 0.0:
                continue
            Jv, Jw = J[l]
            R = T[l][:3, :3]
            Mx += d["mass"] * (Jv.T @ Jv) + Jw.T @ (R @ d["I"] @ R.T) @ Jw
        return 0.5 * (Mx + Mx.T)      # 对称化，压掉浮点不对称

    def U(self, q):
        """重力势能 [J]，z 向上为正。"""
        c = self.com_world(q)
        return float(sum(self.link[l]["mass"] * self.g * c[l][2] for l in self.links))

    def G(self, q, h=1e-7):
        """重力广义力矩 ∂U/∂q（中心差分，二阶精度）。"""
        q = np.asarray(q, float)
        out = np.zeros(self.nq)
        for i in range(self.nq):
            e = np.zeros(self.nq)
            e[i] = h
            out[i] = (self.U(q + e) - self.U(q - e)) / (2.0 * h)
        return out

    def dM_dq(self, q, h=1e-7):
        """返回 list（长度 nq），第 i 项是 ∂M/∂q_i。"""
        q = np.asarray(q, float)
        out = []
        for i in range(self.nq):
            e = np.zeros(self.nq)
            e[i] = h
            out.append((self.M(q + e) - self.M(q - e)) / (2.0 * h))
        return out

    def coriolis(self, q, qd, h=1e-7):
        """[C(q,qd) qd] —— Christoffel 的数值版。"""
        qd = np.asarray(qd, float)
        dM = self.dM_dq(q, h)
        nq = self.nq
        c = np.zeros(nq)
        for i in range(nq):
            s = 0.0
            for j in range(nq):
                for k in range(nq):
                    s += dM[k][i, j] * qd[j] * qd[k]
            for j in range(nq):
                for k in range(nq):
                    s -= 0.5 * dM[i][j, k] * qd[j] * qd[k]
            c[i] = s
        return c

    def qdd(self, q, qd, T):
        """正动力学：M qdd = T − C qd − G。"""
        Mx = self.M(q)
        rhs = np.asarray(T, float) - self.coriolis(q, qd) - self.G(q)
        return np.linalg.solve(Mx, rhs)

    def energy(self, q, qd):
        """返回 (KE, PE, E)。自由落体下 E 应守恒。"""
        T, jw = self.fk(q)
        J = self._jac_from(T, jw)
        KE = 0.0
        for l in self.links:
            d = self.link[l]
            if d["mass"] == 0.0:
                continue
            Jv, Jw = J[l]
            v = Jv @ qd
            w = Jw @ qd
            R = T[l][:3, :3]
            KE += 0.5 * d["mass"] * (v @ v) + 0.5 * w @ (R @ d["I"] @ R.T) @ w
        PE = self.U(q)
        return KE, PE, KE + PE


# ------------------------------------------------------------------ 自检
def _selfcheck():
    SIM = r"C:\Users\29408\Desktop\外骨骼\matlab2609\simscape"
    URDF4 = os.path.join(SIM, "exo_multidof.urdf")
    URDF2 = os.path.join(SIM, "exo_real.urdf")
    OUT = r"C:\Users\29408\Desktop\外骨骼\_multidof_dyn_selfcheck.txt"

    L = []
    def say(s=""):
        L.append(s)
        print(s)

    # ==============================================================
    # ★★★ 判据 A（最强）：2-DOF 退化一致性
    #   同一个组装器跑 exo_real.urdf（2 个 hip 关节），结果必须与
    #   _stp2plant.py 的**闭式解**逐位吻合 —— 两条独立推导链。
    # ==============================================================
    say("=" * 78)
    say("判据 A：2-DOF 退化一致性（本组装器 vs _stp2plant.py 闭式解）")
    say("=" * 78)
    mb2 = MultiBody(URDF2, verbose=True)
    say("  自由度顺序: %s" % mb2.q_names)

    raw = np.loadtxt(os.path.join(SIM, "plant_params.csv"), delimiter=",", skiprows=1)
    # 列: m_kg, I_axis, A, B, amp, phase_deg ; 行: hip_L, hip_R
    say("")
    say("  %-8s %14s %14s %12s" % ("joint", "I_axis(ref)", "M(0)_ii", "rel"))
    okA = True
    for i, nm in enumerate(mb2.q_names):
        I_ref = raw[i, 1]
        M0 = mb2.M(np.zeros(2))
        rel = abs(M0[i, i] - I_ref) / abs(I_ref)
        say("  %-8s %14.9f %14.9f %12.2e" % (nm, I_ref, M0[i, i], rel))
        # 判据地板受 plant_params.csv 影响：它用 "%.9f" 存 I_axis，
        # 而 I_axis ~ 0.017，于是 5e-10 的绝对舍入 = 3e-8 的相对地板。
        # 观测到的 ~2.6e-8 正好在这个地板上，属「完全一致」。
        okA &= rel < 1e-6
    say("  -> I_axis 一致: %s   (判据 1e-6；CSV 只存 9 位小数，地板 ~3e-8)"
        % ("OK" if okA else "FAIL"))

    # G(q) 必须等于 -(A sin q + B cos q)
    say("")
    say("  G(q) vs -(A sin q + B cos q)：扫 q ∈ [-π, π]，51 点")
    qs = np.linspace(-np.pi, np.pi, 51)
    worstG, worstAt = 0.0, ""
    for i, nm in enumerate(mb2.q_names):
        A, B = raw[i, 2], raw[i, 3]
        for qq in qs:
            q = np.zeros(2)
            q[i] = qq
            ref = -(A * np.sin(qq) + B * np.cos(qq))
            got = mb2.G(q)[i]
            e = abs(got - ref)
            if e > worstG:
                worstG, worstAt = e, "%s q=%.4f" % (nm, qq)
    say("  max|G − ref| = %.3e   @ %s" % (worstG, worstAt))
    say("  -> 重力项一致: %s   (判据 < 1e-8)" % ("OK" if worstG < 1e-8 else "FAIL"))
    okA &= worstG < 1e-8

    # 零位重力矩就是 -B
    say("")
    say("  零位重力矩 G(0) 应 = -B（静止位姿本来就挂着重力矩）:")
    G0 = mb2.G(np.zeros(2))
    for i, nm in enumerate(mb2.q_names):
        say("    %-8s  G(0)=%+.9f   -B=%+.9f   |Δ|=%.2e"
            % (nm, G0[i], -raw[i, 3], abs(G0[i] + raw[i, 3])))

    # ==============================================================
    # 判据 B：4-DOF 结构性质
    # ==============================================================
    say("")
    say("=" * 78)
    say("判据 B：4-DOF 结构性质（exo_multidof.urdf）")
    say("=" * 78)
    mb = MultiBody(URDF4, verbose=True)
    say("  自由度顺序: %s" % mb.q_names)
    say("  总质量 = %.9f kg" % sum(mb.link[l]["mass"] for l in mb.links))

    rng = np.random.default_rng(0)
    worst_sym, worst_eig = 0.0, np.inf
    for _ in range(20):
        q = rng.uniform(-1.2, 1.2, mb.nq)
        Mx = mb.M(q)
        worst_sym = max(worst_sym, float(np.max(np.abs(Mx - Mx.T))))
        worst_eig = min(worst_eig, float(np.min(np.linalg.eigvalsh(Mx))))
    say("")
    say("  M(q) 20 随机位姿: max|M−Mᵀ| = %.3e   最小特征值 = %.9f"
        % (worst_sym, worst_eig))
    say("  -> 对称正定: %s" % ("OK" if worst_sym < 1e-12 and worst_eig > 0 else "FAIL"))

    M0 = mb.M(np.zeros(mb.nq))
    say("")
    say("  零位 M(0):")
    say("    对角   = %s" % np.array2string(np.diag(M0), precision=9))
    offmax = np.max(np.abs(M0 - np.diag(np.diag(M0))))
    say("    |非对角|max = %.3e" % offmax)
    say("    -> hip 与 abduct %s"
        % ("**耦合**（这正是不能用解耦模型的理由）" if offmax > 1e-6 else "解耦"))

    # ★ M 到底跟哪个 q 有关？这里反直觉，写清楚免得后人误判：
    #   绕 hip 轴转 q_hip 是**整个腿绕该轴刚性旋转** —— 沿轴的分量不变，
    #   而 abduct 轴固定在 L1 上、跟着一起转，两者相对关系也没变。
    #   于是 M(q) 与 q_hip **完全无关**（下面列出来就是常量；这曾让我误判成 bug）。
    #   真正让 M 变的是 q_abd：abduct 一转，L2/L3 相对 hip 轴的分布就变了。
    say("")
    say("  M 对 q_hip 的依赖（预期 **恒定**：绕 hip 轴旋转不改变任何绕轴惯量）:")
    for qh in (0.0, 0.3, 0.6, 0.9):
        q = np.zeros(4); q[0] = qh
        Mx = mb.M(q)
        say("    q_hip=%+.2f  M(hip,hip)=%+.9f  M(abd,abd)=%+.9f  M(hip,abd)=%+.9f"
            % (qh, Mx[0, 0], Mx[1, 1], Mx[0, 1]))

    say("")
    say("  M 对 q_abd 的依赖（预期 **变化**：姿态相关性都在这里）:")
    for qa in (0.0, 0.3, 0.6, 0.9):
        q = np.zeros(4); q[1] = qa
        Mx = mb.M(q)
        say("    q_abd=%+.2f  M(hip,hip)=%+.9f  M(abd,abd)=%+.9f  M(hip,abd)=%+.9f"
            % (qa, Mx[0, 0], Mx[1, 1], Mx[0, 1]))

    G0 = mb.G(np.zeros(mb.nq))
    say("")
    say("  零位重力矩 G(0) = ∂U/∂q|₀:")
    for i, nm in enumerate(mb.q_names):
        say("    %-10s %+.9f N*m" % (nm, G0[i]))
    say("  -> 与 2-DOF 同源现象：CAD 零位不是重力平衡位，T=0 时自己就会动")

    # ------------------------------------------------------------------
    # ★ 把 M(0) 对角 与 G(0) 写成机器可读基线，供 MATLAB 侧直接读。
    #
    #   为什么要落文件：`test_multidof_analytical.m` 与 `quick_multidof_check.m`
    #   原本把这两组数字**硬编码**在脚本里。密度表一改（改前 hip 项
    #   0.016975773 / 0.438275226 -> 改后 0.016997395 / 0.441845081），
    #   这两个测试就会假 FAIL —— 而且没人会想到是「基线的副本过期了」。
    #   这与密度表 4 处副本漂移是同一类错误，所以这里改成**唯一真源 + 读文件**。
    # ------------------------------------------------------------------
    base = {"source": "_multidof_dyn.py selfcheck (Python assembler)",
            "urdf": os.path.basename(URDF4),
            "q_names": list(mb.q_names),
            "M0_diag": [float(x) for x in np.diag(M0)],
            "G0": [float(x) for x in G0],
            "total_mass": float(sum(mb.link[l]["mass"] for l in mb.links))}
    base_p = os.path.join(SIM, "multidof_baseline.json")
    io.open(base_p, "w", encoding="utf-8").write(
        json.dumps(base, indent=1, ensure_ascii=False))
    say("")
    say("  [基线] WROTE %s" % base_p)
    say("         M0_diag = %s" % np.array2string(np.diag(M0), precision=9))
    say("         G0      = %s" % np.array2string(G0, precision=9))

    # ==============================================================
    # 判据 C：能量守恒（4-DOF 自由落体）
    # ==============================================================
    say("")
    say("=" * 78)
    say("判据 C：能量守恒（4-DOF, T=0, RK4, dt=1e-4）")
    say("=" * 78)

    def rk4(f, y, dt):
        k1 = f(y); k2 = f(y + 0.5*dt*k1); k3 = f(y + 0.5*dt*k2); k4 = f(y + dt*k3)
        return y + dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4)

    zero = np.zeros(mb.nq)
    y = np.zeros(2 * mb.nq)
    E0 = mb.energy(y[:mb.nq], y[mb.nq:])[2]
    dt, T_end = 1e-4, 0.5
    nst = int(round(T_end / dt))
    drift = 0.0
    for k in range(nst):
        y = rk4(lambda yy: np.concatenate(
            [yy[mb.nq:], mb.qdd(yy[:mb.nq], yy[mb.nq:], zero)]), y, dt)
        if (k + 1) % 500 == 0:
            E = mb.energy(y[:mb.nq], y[mb.nq:])[2]
            drift = max(drift, abs(E - E0))
    Ef = mb.energy(y[:mb.nq], y[mb.nq:])[2]
    say("  E0 = %.12f J   E(T=%.2f) = %.12f J" % (E0, T_end, Ef))
    rel_drift = drift / max(abs(E0), 1e-12)
    say("  max|ΔE| = %.3e J   (相对 %.2e)" % (drift, rel_drift))
    say("  判据取 1e-7，因为地板是**数值微分**不是模型：G(q) 与 dM/dq 都用")
    say("  中心差分 h=1e-7，舍入 ~eps·U/h ≈ 1e-9 相对；每步 qdd 都带这个量级的")
    say("  非保守伪力，积分下来漂移自然落在 1e-9。模型若真写错，漂移是 O(1) 量级，")
    say("  与这个地板差 9 个数量级 —— 所以这个判据依然有效。")
    okC = rel_drift < 1e-7
    say("  %s" % ("OK" if okC else "FAIL"))
    say("")
    say("  T=%.2f s 末态（q 与 qd）:" % T_end)
    for i, nm in enumerate(mb.q_names):
        say("    %-10s q=%+.9f  qd=%+.9f" % (nm, y[i], y[mb.nq + i]))

    say("")
    say("=" * 78)
    verdict = "ALL_OK" if (okA and worst_sym < 1e-12 and worst_eig > 0 and okC) \
              else "CHECK_FAILED"
    say("VERDICT: %s" % verdict)
    say("=" * 78)

    io.open(OUT, "w", encoding="utf-8").write("\n".join(L) + "\n")
    return verdict


if __name__ == "__main__":
    _selfcheck()
