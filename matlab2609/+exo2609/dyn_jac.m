function [A, B] = dyn_jac(p, y, T, dt)
%DYN_JAC  离散状态转移的解析雅可比（Eq.(6) 的精确梯度依赖它）。
%
%   [A, B] = exo2609.dyn_jac(p, y, T, dt)
%
%       y_{k+1} = y_k + h * f(y_k, T_k)
%       A = dy_{k+1}/dy_k = I + h * df/dy
%       B = dy_{k+1}/dT_k =     h * df/dT
%
%       df/dy = [        0                  ,   I       ;
%                 Minv*gs*diag(G_amp.*cos q) , -Minv*K1 ]
%
%   ⚠ 左下块必须写成矩阵乘 Minv*diag(...)，不能按元素广播：M 一般非对角。
%
%   注：T 在此处不参与（f 对 T 是仿射的），保留参数只为调用签名对称。

if nargin < 4 || isempty(dt)
    dt = p.dt;
end

y  = y(:);
nq = p.nq;
n  = p.n;
q  = y(1:nq);
Minv = inv(p.M);

dfdy = zeros(n, n);
dfdy(1:nq, nq+1:n)     = eye(nq);
dfdy(nq+1:n, 1:nq)     = p.gravity_sign * (Minv * diag(p.G_amp .* cos(q)));
dfdy(nq+1:n, nq+1:n)   = -(Minv * p.K1);

dfdT = zeros(n, nq);
dfdT(nq+1:n, :)        = Minv;

A = eye(n) + dt * dfdy;
B = dt * dfdT;
end
