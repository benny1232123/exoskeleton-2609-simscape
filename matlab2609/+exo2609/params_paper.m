function p = params_paper(dt)
%PARAMS_PAPER  论文 Table II 标定的 2-DOF 髋关节外骨骼动力学参数。
%
%   p = exo2609.params_paper(dt)
%
%   论文：Assistance Torque Estimation via Dynamics-Aware Optimization for
%         Lower-Limb Exoskeleton in Complex Environments（arXiv:2609.15352v1）
%
%   Table II 的 Torque Estimation 列：M = 0.0156 I, C = 0,
%   G = 0.879 sin(y), K1 = 3 I, Q = diag{10, 0.1} I, P = 0.5 I
%
%   约定（与 Eq.(1) 一致；论文 Eq.(4) 里 G 的符号是笔误）：
%       M*qdd + C*qd + G(q) = T - T_int,   T_int = K1*qd + T0
%   gravity_sign = -1 时前向动力学中重力是阻力。
%
%   输入  dt  离散步长 [s]，默认 0.01
%   输出  p   参数 struct
%
%   见 also exo2609.params_from_csv, exo2609.dyn_f

if nargin < 1 || isempty(dt)
    dt = 0.01;
end

p              = struct();
p.M            = 0.0156 * eye(2);       % 惯量矩阵 [kg*m^2]
p.C_coef       = 0.0;                   % 速度/科氏项系数（论文为 0）
p.G_amp        = [0.879; 0.879];        % 重力矩幅值 [N*m]，G(q) = G_amp.*sin(q)
p.K1           = 3.0 * eye(2);          % 人机交互力矩系数 [N*m*s]
p.T0           = [0; 0];                % 交互力矩常数项 [N*m]
p.dt           = dt;                    % 步长 [s]
p.tau_lim      = [18; 18];              % 电机力矩限幅 [N*m]（AK80-9 手册）
p.q_lim        = [-0.6 1.2; -0.6 1.2];  % 关节限位 [rad]
p.gravity_sign = -1;                    % -1 物理约定；+1 复现 Eq.(4) 字面形式
p.g            = 9.81;                  % 重力加速度 [m/s^2]
p.nq           = 2;                     % 驱动关节数（左髋、右髋）
p.n            = 4;                     % 状态维数 2*nq
p.source       = struct('M', 'paper', 'C_coef', 'paper', 'G_amp', 'paper', ...
                        'K1', 'paper', 'T0', 'assumed', 'dt', 'assumed', ...
                        'tau_lim', 'datasheet');
end
