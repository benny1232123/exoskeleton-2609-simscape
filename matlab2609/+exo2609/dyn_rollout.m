function Y = dyn_rollout(p, y0, U, dt)
%DYN_ROLLOUT  给定初始状态与整段控制序列，前向积分。
%
%   Y = exo2609.dyn_rollout(p, y0, U, dt)
%
%   输入
%       y0  (4x1)             初始状态 [q; qd]
%       U   (L x nq)          控制序列 [T_1; ...; T_L]
%       dt  (可选)            步长，默认 p.dt
%   输出
%       Y   (L x 4)           每步之后的状态，Y(k,:) 对应论文 Y = [y1; ...; yL]
%
%   见 also exo2609.dyn_step

if nargin < 4 || isempty(dt)
    dt = p.dt;
end

L = size(U, 1);
Y = zeros(L, p.n);
y = y0(:);
for k = 1:L
    y = exo2609.dyn_step(p, y, U(k, :), dt);
    Y(k, :) = y.';
end
end
