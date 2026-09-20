function [J, g] = cost_grad(p, w, A, y0, Yd, L)
%COST_GRAD  Eq.(6) 的代价函数与**解析伴随梯度**（无有限差分）。
%
%   [J, g] = exo2609.cost_grad(p, w, A, y0, Yd, L)
%
%   argmin_A  (Yd - Y)' Q (Yd - Y) + A' P A
%             s.t.  y_{k+1} = f(y_k, T_k)
%
%       Yd = [y1; ...; yL] ∈ R^{4L}   期望轨迹
%       Y  = rollout(y0, A)           由动力学模型滚动得到的轨迹
%       A  = [T1; ...; TL] ∈ R^{2L}   待优化助力力矩
%
%   伴随递推（把状态误差对状态的导数记为列车向量）：
%       lam_L   = 2 Q_L (Y_L - Yd_L)
%       lam_{k-1} = A_k' lam_k + 2 Q_{k-1} (Y_{k-1} - Yd_{k-1})
%       dJ/dT_k = B_k' lam_k + 2 P_k T_k
%   其中 A_k = dy_{k+1}/dy_k, B_k = dy_{k+1}/dT_k 由 exo2609.dyn_jac 给出。
%
%   ⚠ 用 check_gradient2609.m 与中心差分核对过：
%     ||dg||/||g_fd|| ~ 1e-10, cos(g_ana, g_fd) = 1.0000000000

nq = p.nq;
n  = p.n;
A  = reshape(A, L, nq);
Q  = exo2609.q_blocks(w, L);

% ---- 前向滚动，缓存每步雅可比 ----
Y  = zeros(L, n);
Aj = zeros(n, n,  L);
Bj = zeros(n, nq, L);
y  = y0(:);
for k = 1:L
    [Aj(:, :, k), Bj(:, :, k)] = exo2609.dyn_jac(p, y, A(k, :));
    y = exo2609.dyn_step(p, y, A(k, :));
    Y(k, :) = y.';
end

% ---- 代价 ----
E = Y - Yd;
J = 0;
for k = 1:L
    J = J + E(k, :) * Q{k} * E(k, :).';
end
Pk = w.Ca * eye(nq);
for k = 1:L
    J = J + A(k, :) * Pk * A(k, :).';
end

% ---- 伴随递推 ----
G = zeros(L, nq);
lam = 2 * (Q{L} * E(L, :).');
G(L, :) = (Bj(:, :, L).' * lam + 2 * Pk * A(L, :).').';
for k = L-1:-1:1
    lam = Aj(:, :, k+1).' * lam + 2 * (Q{k} * E(k, :).');
    G(k, :) = (Bj(:, :, k).' * lam + 2 * Pk * A(k, :).').';
end

g = G(:);
end
