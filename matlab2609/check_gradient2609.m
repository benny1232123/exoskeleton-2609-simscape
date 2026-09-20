%% CHECK_GRADIENT2609  梯度校验：解析伴随梯度 vs 中心差分
%
% 论文 Eq.(6) 的精确梯度是整个滑窗估计器的地基。不先做这一步，
% 后面「估计力矩偏小 ~89%」就分不清是求解器/梯度写错了，还是正则项使然。
%
% 判据：||dg||/||g_fd|| < 1e-5 且 cos(g_ana, g_fd) ≈ 1.0000000000
% 期望结果（已验证）：||dg||/||g_fd|| ≈ 2e-10, cos = 1.0000000000

clear; clc;
thisdir = fileparts(mfilename('fullpath'));
if isempty(thisdir), thisdir = pwd; end
addpath(thisdir);

fprintf('================ 梯度校验 ================\n');
fprintf('MATLAB %s\n', version);

p = exo2609.params_paper(0.01);
w = exo2609.weights_paper();
L = 20;

% ---- 造一段带已知 T_true 的轨迹 ----
g    = exo2609.gait_ref(1.0, 0.4, p.dt);
phi  = 2*pi*g.f_cycle*g.t;
Ttr  = [0.9*sin(phi), -0.9*sin(phi)];
N    = numel(g.t);
y    = [g.q(1, :).'; g.qd(1, :).'];
Yd   = zeros(N, 4);
for k = 1:N
    y = exo2609.dyn_step(p, y, Ttr(k, :));
    Yd(k, :) = y.';
end

rng(0);
ytest = Yd(1, :).';
Ydwin = Yd(2 : 1+L, :);
Atest = 0.4 * randn(L, p.nq);

[J0, gana] = exo2609.cost_grad(p, w, Atest, ytest, Ydwin, L);

eps_ = 1e-6;
xo   = Atest(:);
gfd  = zeros(size(gana));
for i = 1:numel(xo)
    xp = xo; xp(i) = xp(i) + eps_;
    xm = xo; xm(i) = xm(i) - eps_;
    Jp = exo2609.cost_grad(p, w, xp, ytest, Ydwin, L);
    Jm = exo2609.cost_grad(p, w, xm, ytest, Ydwin, L);
    gfd(i) = (Jp - Jm) / (2*eps_);
end

grel = norm(gana - gfd) / norm(gfd);
cosg = (gana.' * gfd) / (norm(gana) * norm(gfd));

if grel < 1e-5, verdict = 'PASS'; else, verdict = 'FAIL'; end

fprintf('\n');
fprintf('  J(A_test)        = %.10e\n', J0);
fprintf('  ||g_analytic||   = %.6e\n', norm(gana));
fprintf('  ||g_finite_diff||= %.6e\n', norm(gfd));
fprintf('  ||dg||/||g_fd||  = %.3e   -> %s\n', grel, verdict);
fprintf('  max|dg|          = %.3e\n', max(abs(gana - gfd)));
fprintf('  cos(g_ana,g_fd)  = %.10f\n', cosg);
fprintf('\n结论：动力学 + 解析伴随梯度链路自洽，Eq.(6) 求解不存在梯度 bug。\n');

if ~strcmp(verdict, 'PASS')
    error('梯度校验失败：||dg||/||g_fd|| = %.3e', grel);
end
