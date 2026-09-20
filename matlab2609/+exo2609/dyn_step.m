function ynext = dyn_step(p, y, T, dt)
%DYN_STEP  Eq.(5)：离散状态转移 y_{k+1} = y_k + dt * f(y_k, T_k)（显式欧拉）。
%
%   ynext = exo2609.dyn_step(p, y, T, dt)
%
%   ⚠ 数值注意：本模型的内部交互阻尼时间常数为 M/K1（CAD 约 5.66 ms，
%     论文约 5.20 ms）。显式欧拉要正确分辨它需要 dt << M/K1；
%     若 dt > 0.5*M/K1，离散速度极点 1 - dt*K1/M 会变号（速度逐步符号交替），
%     轨迹对力矩的响应被失真放大。建议 dt <= 1.4 ms（>= 707 Hz），
%     或对 K1*qd 阻尼项改用隐式/半隐式积分。
%     诊断函数见 exo2609.discretization_info。

if nargin < 4 || isempty(dt)
    dt = p.dt;
end

ynext = y(:) + dt * exo2609.dyn_f(p, y, T);
end
