function g = gait_ref(speed, duration, dt, phaseOffset, lrDelay)
%GAIT_REF  生成双侧髋关节参考轨迹（替代实测步态，用于跑通与验证）。
%
%   g = exo2609.gait_ref(speed, duration, dt, phaseOffset, lrDelay)
%
%   !! 本函数产出的是 surrogate（代理）轨迹，不是实测步态。
%      基于它得到的**助力力矩绝对值**只能用于验证模型行为，
%      不能当作硬件标定结果。真机标定必须换成实测 (q, qd) 轨迹。
%
%   构造：髋屈伸角逐周期取 2 次谐波傅里叶级数，对正常步态关键事件点最小二乘拟合。
%     相位   屈髋角(deg)   事件
%     0.00    25          首次触地
%     0.10    18          承重反应
%     0.30     5          支撑中期
%     0.50   -10          支撑末期（髋后伸最大）
%     0.60     5          预摆动
%     0.75    28          摆动初期（屈髋峰值）
%     0.85    22          摆动中期
%     1.00    25          摆动末期
%   步频随速度线性插值：0.6 m/s -> 0.75 Hz, 1.0 -> 0.89, 1.4 -> 1.02
%   幅值随速度轻微放大：0.85x / 1.00x / 1.12x
%
%   输入
%       speed        行走速度 [m/s]，对应论文 Fig.4 的 0.6 / 1.0 / 1.4
%       duration     时长 [s]
%       dt           采样步长 [s]
%       phaseOffset  整体相位偏移（归一化周期），默认 0
%       lrDelay      左右腿相位差，默认 0.5（对侧步态）
%   输出 struct g
%       g.t (Nx1), g.q (Nx2), g.qd (Nx2), g.qdd (Nx2), g.f_cycle, g.amp_scale
%       g.q(:,1)/g.qd(:,1) = 左髋；(:,2) = 右髋
%
%   见 also exo2609.load_reference

if nargin < 4 || isempty(phaseOffset), phaseOffset = 0.0; end
if nargin < 5 || isempty(lrDelay),     lrDelay = 0.5;    end

% ---- 关键事件点 -> 2 次谐波傅里叶系数（最小二乘） ----
KEYPHI = [0.00; 0.10; 0.30; 0.50; 0.60; 0.75; 0.85; 1.00];
KEYDEG = [25.0; 18.0;  5.0; -10.0; 5.0; 28.0; 22.0; 25.0];
ykey   = deg2rad(KEYDEG);

A = ones(numel(KEYPHI), 5);
for k = 1:2
    A(:, 2*k)   = cos(2*pi*k*KEYPHI);
    A(:, 2*k+1) = sin(2*pi*k*KEYPHI);
end
coef = A \ ykey;              % [a0; a1; b1; a2; b2]

% ---- 步频 / 幅值 ----
vAnchor = [0.6; 1.0; 1.4];
fAnchor = [0.75; 0.89; 1.02];
aAnchor = [0.85; 1.00; 1.12];
f = interp_clamped(speed, vAnchor, fAnchor);
s = interp_clamped(speed, vAnchor, aAnchor);

% ---- 时间序列 ----
N = round(duration / dt);
t = (0:N-1).' * dt;
phiL = mod(f*t + phaseOffset, 1);
phiR = mod(f*t + phaseOffset + lrDelay, 1);

q   = [hipflex(phiL, coef, s), hipflex(phiR, coef, s)];
qd  = [  dhip(phiL, coef, s, f),   dhip(phiR, coef, s, f)];
qdd = [ ddhip(phiL, coef, s, f),  ddhip(phiR, coef, s, f)];

g = struct('t', t, 'q', q, 'qd', qd, 'qdd', qdd, ...
           'f_cycle', f, 'amp_scale', s, 'speed', speed, ...
           'coef', coef, 'source', 'surrogate');
end

% ------------------------------------------------------------------
function y = hipflex(phi, coef, s)
% s 只缩放交流分量（a0 是均值，不缩放）
y = coef(1) * ones(size(phi));
for k = 1:2
    y = y + s * (coef(2*k)*cos(2*pi*k*phi) + coef(2*k+1)*sin(2*pi*k*phi));
end
end

function y = dhip(phi, coef, s, f)
w = 2*pi*f;
y = zeros(size(phi));
for k = 1:2
    y = y + s*w*k*(-coef(2*k)*sin(2*pi*k*phi) + coef(2*k+1)*cos(2*pi*k*phi));
end
end

function y = ddhip(phi, coef, s, f)
w = 2*pi*f;
y = zeros(size(phi));
for k = 1:2
    y = y - s*(w*k)^2*(coef(2*k)*cos(2*pi*k*phi) + coef(2*k+1)*sin(2*pi*k*phi));
end
end

function y = interp_clamped(xq, xp, fp)
% np.interp 语义：超出范围时钳到端点值（MATLAB interp1 默认会给 NaN）
xq = min(max(xq, min(xp)), max(xp));
y  = interp1(xp, fp, xq, 'linear');
end
