%% SELFTEST2609  自检：SolidWorks CSV -> 参数；谱分解与 Python 结果交叉核对
%
%   这是把 MATLAB 侧钉在「已验证的 Python 实现」上的回归测试。
%   两处关键交叉核对：
%     (1) CSV -> M / G_amp  必须复现由本机 STP 直算的 0.016976 / 0.5544
%     (2) 观测 Hessian 谱    必须复现 Python 的 kappa ~ 689, lam_max ~ 8.517
%        （这一项抓出过一个大坑：MATLAB 的 (:) 是列优先、numpy 的 ravel() 是行优先，
%          直接展平会把 Q 作用到错误的行上，把 kappa 从 690 误算成 9740。）

clear; clc;
thisdir = fileparts(mfilename('fullpath'));
if isempty(thisdir), thisdir = pwd; end
addpath(thisdir);

fail = 0;
fprintf('=================== SELFTEST 2609 ===================\n');
fprintf('MATLAB %s\n\n', version);

% ============ 1) SolidWorks CSV -> 参数 ============
fprintf('--- 1) SolidWorks CSV -> M / G_amp ---\n');
csvp = fullfile(thisdir, 'solidworks', 'mass_props_example_from_stp.csv');
pcsv = exo2609.params_from_csv(csvp, 'leg_L', 'leg_R', 0.01, true);

expM = 0.016976;
expG = 0.5544;
dM = abs(pcsv.M(1,1) - expM);
dG = abs(pcsv.G_amp(1) - expG);
fprintf('\n  期望（由本机 STP 直算）: M = %.6f,  G_amp = %.4f\n', expM, expG);
fprintf('  实际                    : M = %.6f,  G_amp = %.4f\n', pcsv.M(1,1), pcsv.G_amp(1));
fprintf('  偏差                    : dM = %.3e,  dG = %.3e\n', dM, dG);
if dM < 2e-6 && dG < 5e-4
    fprintf('  -> PASS（CSV 接口与单位换算正确）\n');
else
    fprintf('  -> FAIL\n'); fail = fail + 1;
end

% ============ 2) 谱分解 vs Python ============
fprintf('\n--- 2) 观测 Hessian 谱分解（CAD 参数, dt=0.01, L=20）---\n');
w  = exo2609.weights_paper();
pc = pcsv;

g   = exo2609.gait_ref(1.0, 1.2, pc.dt);
phi = 2*pi*g.f_cycle*g.t;
Ttr = [0.9*sin(phi), -0.9*sin(phi + pi)];
N   = numel(g.t);
y   = [g.q(1, :).'; g.qd(1, :).'];
Yd  = zeros(N, 4);
for k = 1:N
    y = exo2609.dyn_step(pc, y, Ttr(k, :));
    Yd(k, :) = y.';
end

L = 20;
tw = 20;                                        % Python 的 t_w = 20（0-based）
S = exo2609.spectrum(pc, w, Yd(tw+1, :).', Yd(tw+2 : tw+1+L, :), Ttr(tw+1 : tw+L, :));

fprintf('  MATLAB : kappa = %.4e, lam_min = %.4e, lam_max = %.6f\n', ...
        S.kappa, min(S.lam), max(S.lam));
fprintf('  Python : kappa = 6.8918e+02, lam_min = 1.2358e-02, lam_max = 8.516700\n');
kerr = abs(S.kappa - 689.18) / 689.18;
merr = abs(max(S.lam) - 8.5167) / 8.5167;
fprintf('  相对偏差: kappa %.3e, lam_max %.3e\n', kerr, merr);
if kerr < 1e-3 && merr < 1e-3
    fprintf('  -> PASS（谱与 Python 一致，展平顺序正确）\n');
else
    fprintf('  -> FAIL\n'); fail = fail + 1;
end

% ============ 3) 离散化诊断 ============
fprintf('\n--- 3) 离散化诊断 ---\n');
d = exo2609.discretization_info(pc);
fprintf('  %s\n', d.note);
fprintf('  tau = %.4f ms, dt 建议上限 = %.5f s (%.0f Hz)\n', ...
        d.tau_damp*1e3, d.recommended_dt, d.recommended_hz);
fprintf('  （期望：tau ≈ 5.66 ms，建议 dt ≤ 0.001415 s / 707 Hz）\n');
if abs(d.tau_damp*1e3 - 5.659) < 0.01
    fprintf('  -> PASS\n');
else
    fprintf('  -> FAIL\n'); fail = fail + 1;
end

if fail == 0
    verdict = 'ALL PASS';
else
    verdict = sprintf('%d FAILED', fail);
end
fprintf('\n=================== 结果：%s ===================\n', verdict);
if fail > 0
    error('selftest2609: 有 %d 项失败', fail);
end
