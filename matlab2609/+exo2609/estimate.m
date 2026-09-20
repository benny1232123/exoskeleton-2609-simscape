function out = estimate(p, w, Qref, Qdref, L, opts)
%ESTIMATE  Eq.(6) 滑窗式助力力矩估计（论文 Section III.B）。
%
%   out = exo2609.estimate(p, w, Qref, Qdref, L, opts)
%
%   论文原文：the optimization is performed via a sliding-window scheme with
%   window length L and sliding step size 1. The first torque value of A solved
%   from each window-wise optimization is regarded as the estimated assistance
%   torque corresponding to the current motion state.
%
%   输入
%       p, w            参数 / 权重（exo2609.params_paper / weights_paper）
%       Qref  (N x nq)  实测关节角 [rad]
%       Qdref (N x nq)  实测关节角速度 [rad/s]
%       L               窗口长度（时间步数）
%       opts (可选)     .enforce_tau_limit 默认 true
%                       .display          'off' | 'iter'，默认 'off'
%                       .maxiter          默认 4000
%   输出 struct out
%       out.T_est (N x nq)  估计助力力矩；第 t 行对应「当前运动状态 t」
%                           窗口 t 覆盖参考点 t+1..t+L，故仅前 N-L 行有解，其余 NaN
%       out.info  1xN struct 数组：cost / exitflag / iterations / firstorderopt
%       out.diag  汇总统计
%
%   依赖：Optimization Toolbox 的 fmincon（本机 R2024b 已装）。
%         若缺失，自动退化为带 Barzilai-Borwein 步长的投影梯度 + Armijo 回溯
%         （结果等价但慢）。
%
%   见 also exo2609.cost_grad, check_gradient2609, diag_shrinkage2609

if nargin < 6, opts = struct(); end
if ~isfield(opts, 'enforce_tau_limit'), opts.enforce_tau_limit = true;   end
if ~isfield(opts, 'display'),           opts.display = 'off';            end
if ~isfield(opts, 'maxiter'),           opts.maxiter = 4000;             end

nq = p.nq;
Qref  = reshape(Qref,  [], nq);
Qdref = reshape(Qdref, [], nq);
N = size(Qref, 1);
if N < L + 1
    error('exo2609:estimate:short', '轨迹长度 N=%d 必须 > 窗口长度 L=%d', N, L);
end

Yd_all = [Qref, Qdref];                 % (N x 4)
T_est  = nan(N, nq);
info   = repmat(struct('cost', NaN, 'exitflag', NaN, 'iterations', NaN, ...
                       'firstorderopt', NaN), 1, N);
hasFmincon = exist('fmincon', 'file') == 2;

if hasFmincon
    fopts = optimoptions('fmincon', 'Algorithm', 'sqp', ...
        'SpecifyObjectiveGradient', true, 'Display', opts.display, ...
        'MaxIterations', opts.maxiter, 'MaxFunctionEvaluations', 1e5, ...
        'OptimalityTolerance', 1e-10, 'StepTolerance', 1e-12, ...
        'FunctionTolerance', 1e-12);
end

lb = []; ub = [];
if opts.enforce_tau_limit
    lb = repmat(-p.tau_lim(:), L, 1);
    ub = repmat( p.tau_lim(:), L, 1);
end

Aprev = [];
nFail = 0;
A0first = [];
for t = 0:(N - L - 1)
    y_meas = Yd_all(t+1, :).';            % 窗口起始的「当前状态」
    Yd     = Yd_all(t+2 : t+1+L, :);      % 该窗口的期望轨迹（L x 4）

    if isempty(Aprev)
        A0 = zeros(L, nq);
    else                                  % 热启动：整体前移一步，末位重复
        A0 = [Aprev(2:end, :); Aprev(end, :)];
    end

    fun = @(a) exo2609.cost_grad(p, w, a, y_meas, Yd, L);

    if hasFmincon
        [a, fval, ef, outp] = fmincon(fun, A0(:), [],[],[],[], lb, ub, [], fopts);
        nit = outp.iterations;  gopt = outp.firstorderopt;
    else
        [a, fval, ef, nit, gopt] = pgd_bb(fun, A0(:), lb, ub, opts.maxiter);
    end

    A = reshape(a, L, nq);
    T_est(t+1, :) = A(1, :);              % 取窗口第一个力矩
    info(t+1) = struct('cost', fval, 'exitflag', ef, ...
                       'iterations', nit, 'firstorderopt', gopt);
    if ef <= 0, nFail = nFail + 1; end
    if isempty(A0first), A0first = A; end
    Aprev = A;
end

valid = ~isnan(T_est(:, 1));
out = struct();
out.T_est = T_est;
out.info  = info;
out.diag  = struct( ...
    'n_windows', sum(valid), ...
    'n_failed',  nFail, ...
    'L',         L, ...
    'used_fmincon', hasFmincon, ...
    'tau_peak',  max(abs(T_est(valid, :)), [], 'all'), ...
    'max_firstorderopt', max([info(valid).firstorderopt]));
end

% ==================================================================
function [a, f, ef, it, gopt] = pgd_bb(fun, a0, lb, ub, maxit)
%PGD_BB  投影梯度 + Barzilai-Borwein 步长 + Armijo 回溯（无 Optimization Toolbox 时的兜底）
if isempty(lb), lb = -inf(size(a0)); end
if isempty(ub), ub =  inf(size(a0)); end

proj = @(x) min(max(x, lb), ub);
a = proj(a0);
[f, g] = fun(a);  g0 = max(abs(g));
alpha = 1 / max(1e-8, norm(g));
ef = 0;
for it = 1:maxit
    gn = norm(proj(a - g) - a);                 % 投影梯度范数
    if gn < 1e-9, ef = 1; break; end
    d = -g;
    accepted = false;
    fn = f; gnv = g;
    for ls = 1:25
        an = proj(a + alpha * d);
        [fn, gnv] = fun(an);
        if fn <= f - 1e-4 * g.' * (a - an)      % Armijo
            accepted = true;  break
        end
        alpha = alpha * 0.5;
    end
    if ~accepted, ef = 0; break; end
    s = an - a;  yv = gnv - g;
    sy = s.' * yv;
    if sy > 1e-14, alpha = min(1e3, max(1e-12, (s.'*s) / sy)); end
    a = an;  f = fn;  g = gnv;
end
gopt = norm(proj(a - g) - a);
end
