%% DEMO2609_RUN  2609 复现 · MATLAB 端到端 demo
%
%   1. 论文 Table II 标定参数 -> 前向动力学 + 参考步态
%   2. 注入已知助力力矩 T_true，正向仿真出轨迹
%   3. 只把轨迹喂给 Eq.(6) 滑窗估计器，反解助力力矩
%   4. 观测 Hessian 谱分解，解释复原偏差（不是 bug，是正则项）
%   5. 出图存到 out/
%
% 用法：直接 Run，或在命令行  run('demo2609_run.m')
%
% ⚠ 步态轨迹是 surrogate（代理数据），不是实测。助力力矩绝对值只能用于
%   验证模型行为；真机标定必须把 Yd 换成实测 (q, qd)。

clear; clc; close all;
thisdir = fileparts(mfilename('fullpath'));
if isempty(thisdir), thisdir = pwd; end
addpath(thisdir);
outdir = fullfile(thisdir, 'out');
if ~exist(outdir, 'dir'), mkdir(outdir); end
logf = fullfile(outdir, 'demo2609_log.txt');
diary(logf); diary on;
fprintf('日志写入 %s\n\n', logf);

DT    = 0.01;
DUR   = 1.2;
L     = 20;
SPEED = 1.0;
AMP   = 0.9;

% ================= 1. 参数 =================
p = exo2609.params_paper(DT);
w = exo2609.weights_paper();
fprintf('================ 参数（论文 Table II）================\n');
fprintf('  M     = %.6f I  [kg*m^2]\n', p.M(1,1));
fprintf('  G_amp = %.6f    [N*m]\n', p.G_amp(1));
fprintf('  K1    = %.6f I  [N*m*s]\n', p.K1(1,1));
fprintf('  dt    = %.4g s\n', p.dt);
dsc = exo2609.discretization_info(p);
fprintf('  %s\n', dsc.note);
fprintf('  M/K1 = %.3f ms, 建议 dt <= %.4g s (%.0f Hz)\n', ...
        dsc.tau_damp*1e3, dsc.recommended_dt, dsc.recommended_hz);

g = exo2609.gait_ref(SPEED, DUR, DT);
fprintf('\n参考步态：步频 %.4f Hz，幅值缩放 %.3f，N = %d\n', ...
        g.f_cycle, g.amp_scale, numel(g.t));

% ================= 2. 正向仿真（注入 T_true）=================
phi   = 2*pi*g.f_cycle*g.t;
Ttrue = [AMP*sin(phi), AMP*sin(phi + pi)];    % 左右反相
N     = numel(g.t);
y     = [g.q(1, :).'; g.qd(1, :).'];
Yd    = zeros(N, 4);
for k = 1:N
    y = exo2609.dyn_step(p, y, Ttrue(k, :));
    Yd(k, :) = y.';
end
fprintf('已生成 T_true 驱动的轨迹：幅值 %.1f N*m，时长 %.1f s\n', AMP, DUR);

% ================= 3. Eq.(6) 滑窗估计 =================
fprintf('\n================ Eq.(6) 滑窗估计 ================\n');
tic;
out = exo2609.estimate(p, w, Yd(:, 1:2), Yd(:, 3:4), L);
el = toc;

nv  = N - L;
Te  = out.T_est(1:nv, :);
Tt  = Ttrue(1:nv, :);
err = Te - Tt;
rel    = norm(err) / norm(Tt);
shrink = sum(Te(:) .* Tt(:)) / sum(Tt(:) .^ 2);

fprintf('  求解 %d 个窗口，用时 %.2f s\n', nv, el);
fprintf('  失败窗口 = %d / %d   (fmincon: %d)\n', ...
        out.diag.n_failed, nv, out.diag.used_fmincon);
fprintf('  max|dT| = %.6f N*m   RMS(dT) = %.6f N*m\n', ...
        max(abs(err), [], 'all'), sqrt(mean(err(:).^2)));
fprintf('  ||dT||/||T|| = %.4f%%\n', rel*100);
fprintf('  收缩比 sum(Te.*Tt)/sum(Tt.^2) = %.6f   <-- 估计力矩对真值的幅值比\n', shrink);
fprintf('  估计力矩峰值 = %.4f N*m（真值峰值 %.4f）\n', ...
        max(abs(Te), [], 'all'), max(abs(Tt), [], 'all'));

% ================= 4. 谱分解 =================
fprintf('\n================ 观测 Hessian 谱分解 ================\n');
S = exo2609.spectrum(p, w, Yd(1, :).', Yd(2:1+L, :), Ttrue(1:L, :));
fprintf('  窗口 H = S''QS 为 %dx%d，条件数 kappa = %.4e\n', size(S.H, 1), size(S.H, 2), S.kappa);
fprintf('  lam_min = %.4e   lam_max = %.4e   lam_median = %.4e\n', ...
        min(S.lam), max(S.lam), S.lam_median);
fprintf('  Ca(论文) / lam_median = %.1f 倍\n', S.Ca_over_lam_median);
fprintf('  谱预测 ||dT||/||T|| = %.6f   （实测 %.6f）\n', S.rel_pred, rel);
fprintf('  谱预测收缩比        = %.6f   （实测 %.6f）\n', S.shrink_pred, shrink);
fprintf('  A_true 落在 lam<Ca 方向的能量占比 = %.2f%%\n', S.lost_frac*100);
fprintf('\n  结论：复原偏差由「病态观测 Hessian x Tikhonov 正则」解释。\n');
fprintf('        收缩因子 = lam_i/(lam_i+Ca)；小 lam 方向被 1/(1+Ca/lam_i) 抹平。\n');
fprintf('        论文自述 P 是「防止力矩过大的正则项」=> 方法的固有特性，非复现错误。\n');

% ================= 5. 出图 =================
f1 = figure('Position', [80 80 980 380], 'Color', 'w');
subplot(1,2,1);
plot(g.t, rad2deg(g.q(:,1)), 'LineWidth', 1.4); hold on;
plot(g.t, rad2deg(g.q(:,2)), 'LineWidth', 1.4);
grid on; xlabel('time [s]'); ylabel('hip angle [deg]');
legend({'left hip','right hip'}, 'Location','best');
title('surrogate gait reference');
subplot(1,2,2);
plot(g.t, Ttrue(:,1), 'LineWidth', 1.4); hold on;
plot(g.t, Ttrue(:,2), 'LineWidth', 1.4);
grid on; xlabel('time [s]'); ylabel('torque [N\cdotm]');
legend({'T_{true} left','T_{true} right'}, 'Location','best');
title('injected ground-truth assistance torque');
exportgraphics(f1, fullfile(outdir, 'fig1_gait.png'), 'Resolution', 120);

tv = g.t(1:nv);
f2 = figure('Position', [80 80 980 560], 'Color', 'w');
subplot(2,1,1);
plot(tv, Tt(:,1), '--', 'LineWidth', 2, 'Color', [0.6 0.6 0.6]); hold on;
plot(tv, Te(:,1), '-', 'LineWidth', 1.6);
plot(tv, Te(:,2), ':', 'LineWidth', 1.6);
grid on; legend({'T_{true}','T_{est} left','T_{est} right'}, 'Location','best');
ylabel('torque [N\cdotm]');
title(sprintf('closed-loop recovery: ||\\DeltaT||/||T|| = %.1f%%,  shrink = %.3f', ...
      rel*100, shrink));
subplot(2,1,2);
plot(tv, err(:,1)*1e3, 'LineWidth', 1.2); hold on;
plot(tv, err(:,2)*1e3, 'LineWidth', 1.2);
grid on; xlabel('time [s]'); ylabel('error [mN\cdotm]');
legend({'\DeltaT left','\DeltaT right'}, 'Location','best');
exportgraphics(f2, fullfile(outdir, 'fig2_recovery.png'), 'Resolution', 120);

f3 = figure('Position', [80 80 980 380], 'Color', 'w');
subplot(1,2,1);
semilogy(1:numel(S.lam), S.lam, 'o-', 'MarkerSize', 3); hold on;
yline(w.Ca, '--', 'Ca = 0.5 (paper)'); yline(0.05, '--', 'Ca = 0.05');
grid on; xlabel('eigen index'); ylabel('lambda of H = S^T Q S');
title(sprintf('observation Hessian spectrum (\\kappa = %.0f)', S.kappa));
subplot(1,2,2);
grid_ = logspace(log10(max(1e-4, min(S.lam))), log10(max(S.lam)), 120);
cum = arrayfun(@(g_) sum(S.c(S.lam < g_).^2)/S.na2, grid_);
semilogx(grid_, cum*100, 'LineWidth', 2); hold on;
xline(w.Ca, '--', 'Ca = 0.5'); grid on;
xlabel('lambda threshold'); ylabel('% of ||A_{true}||^2 with \lambda < threshold');
title('torque energy lives in near-null directions');
exportgraphics(f3, fullfile(outdir, 'fig3_hessian.png'), 'Resolution', 120);

fprintf('\n图已保存到 %s\n', outdir);
diary off;
