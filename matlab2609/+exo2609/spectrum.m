function s = spectrum(p, w, y0, Yd, Atrue)
%SPECTRUM  观测 Hessian H = S'QS 的谱分解 —— 定量解释 Eq.(6) 的复原偏差。
%
%   s = exo2609.spectrum(p, w, y0, Yd, Atrue)
%
%   在一个窗口上线性化：Y(A) ≈ Y(A_true) + S*(A - A_true)，S = dY/dA，
%   记 H = S' Q S（Gauss-Newton Hessian），则 Eq.(6) 的解为
%
%       A* = (H + P)^{-1} H A_true = sum_i [lam_i/(lam_i+Ca)] (v_i' A_true) v_i
%
%   **收缩因子就是 lam_i/(lam_i + Ca)**，(lam_i, v_i) 为 H 的特征对。
%   打靶(shooting)式力矩估计的 H 天然病态：许多 A 方向几乎不改变轨迹
%   （lam_i -> 0），这些方向被 1/(1 + Ca/lam_i) 近乎抹平。
%
%   输入
%       p, w      参数 / 权重
%       y0  (4x1) 窗口起始状态
%       Yd  (Lx4) 该窗口期望轨迹（本诊断只用 L 与 p，Yd 不参与 H 的构造）
%       Atrue (Lxnq) 注入的**真值**力矩（用于算 A_true 的投影能量分布）
%   输出 struct s
%       s.lam (2L x 1) 升序特征值      s.V 特征向量      s.c = V'*A_true(:)
%       s.na2 = ||A_true||^2
%       s.kappa, s.lam_median
%       s.rel_pred, s.shrink_pred  —— 该窗口由谱公式预测的误差 / 收缩比
%       s.lost_frac —— A_true 落在 lam < Ca 方向的能量占比（被抹掉的部分）
%       s.S, s.H, s.Qf
%
%   ⚠ MATLAB 线性索引是列优先，Python 的 ravel() 是行优先；此处 S 的列顺序
%     与 A_true(:) 的展开顺序**保持一致**即可（用 (:) 两处都列优先）。
%     特征值集合与列顺序无关，所以谱结论与 Python 完全可比。
%
%   见 also exo2609.cost_grad, diag_shrinkage2609

L  = size(Atrue, 1);
nq = p.nq;
n  = p.n;

% ---- 分块权重 Qf ----
Q  = exo2609.q_blocks(w, L);
Qf = zeros(4*L, 4*L);
for k = 1:L
    Qf(4*(k-1)+1 : 4*k, 4*(k-1)+1 : 4*k) = Q{k};
end

% ---- 中心差分组装 S = dY/dA ----
% ⚠ 行顺序必须是「按步分块」(step-major)：第 k 块的 4 行对应 y_k 的 4 个状态，
%   这样才能与下面按 4x4 分块放置的 Qf 对齐。
%   MATLAB 的 Y(:) 是列优先（变量优先），直接用它会把 Q 作用到错误的行上
%   —— 曾经因此把 kappa 从 690 误算成 9740。必须用 rmajor() 展平。
eps_ = 1e-6;
S = zeros(4*L, L*nq);
for i = 1:(L*nq)
    Ap = zeros(L, nq);  Ap(i) =  eps_;
    Am = zeros(L, nq);  Am(i) = -eps_;
    Yp = exo2609.dyn_rollout(p, y0, Ap);
    Ym = exo2609.dyn_rollout(p, y0, Am);
    S(:, i) = (rmajor(Yp) - rmajor(Ym)) / (2*eps_);
end

H = S.' * Qf * S;
H = 0.5 * (H + H.');

[V, D] = eig(H);
lam = real(diag(D));
[lam, idx] = sort(lam);
V = V(:, idx);
lam = max(lam, 0);

a = Atrue(:);
c = V.' * a;
na2 = a.' * a;

Ca = w.Ca;
sh = lam ./ (lam + Ca);
Ah = V * (sh .* c);

pos = lam > 1e-30;
s = struct();
s.lam        = lam;
s.V          = V;
s.c          = c;
s.na2        = na2;
s.kappa      = max(lam) / min(lam(pos));
s.lam_median = median(lam);
s.Ca_over_lam_median = Ca / max(eps, s.lam_median);
s.rel_pred    = norm(Ah - a) / max(eps, norm(a));
s.shrink_pred = (c .* sh).' * c / max(eps, na2);
s.lost_frac   = sum(c(lam < Ca).^2) / max(eps, na2);
s.S  = S;
s.H  = H;
s.Qf = Qf;
end

% ==================================================================
function v = rmajor(M)
%RMAJOR  行优先展平（等价于 numpy 的 ravel()；MATLAB 的 M(:) 是列优先）。
%   作用：把 (L x 4) 的轨迹矩阵展成「按步分块」的 4L 向量，
%   使第 k 块的 4 个元素对应 y_k 的 4 个状态。
v = reshape(M.', [], 1);
end
