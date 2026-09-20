function d = discretization_info(p)
%DISCRETIZATION_INFO  显式欧拉的离散化诊断（判断 dt 是否分辨得了内部阻尼）。
%
%   d = exo2609.discretization_info(p)
%
%   内部交互阻尼的最快时间常数 tau = min eig(M) / max eig(K1)。
%   连续速度自由度按 exp(-t/tau) 衰减。显式欧拉要正确分辨它需要 dt << tau；
%   若 dt*max eig(K1)/min eig(M) 接近或超过 1，离散速度极点
%       pole = 1 - dt*K1/M
%   会变负 -> 速度逐步**符号交替**（数值伪振荡），轨迹对力矩的响应被失真放大，
%   进而恶化 Eq.(6) 观测 Hessian 的条件数。
%
%   本 CAD / 论文的实际值：
%       CAD  M/K1 = 5.66 ms  -> dt 建议 <= 1.4 ms (>= 707 Hz)
%       论文 M/K1 = 5.20 ms  -> dt 建议 <= 1.3 ms (>= 769 Hz)
%   dt = 10 ms 时 pole = -0.767（欠采样）。降到 dt = 1 ms 后 Eq.(6) 复原误差
%   从 0.889 降到 0.238。详见 2609_report。
%
%   输出 struct d
%       d.tau_damp, d.ratio, d.pole, d.ok, d.recommended_dt, d.recommended_hz, d.note

tau   = min(eig(p.M)) / max(eig(p.K1));
ratio = p.dt / tau;
pole  = 1 - p.dt * max(eig(p.K1)) / min(eig(p.M));
rec   = 0.25 * tau;

d = struct('tau_damp', tau, 'ratio', ratio, 'pole', pole, ...
           'ok', ratio <= 0.5, ...
           'recommended_dt', rec, 'recommended_hz', 1/rec);

if d.ok
    d.note = sprintf(['dt=%.4g s 可分辨内部阻尼时间常数 M/K1=%.4g s ' ...
                      '(dt*K1/M=%.2f, 极点 %+.3f)'], p.dt, tau, ratio, pole);
else
    d.note = sprintf(['[警告] dt=%.4g s 相对内部阻尼时间常数 M/K1=%.4g s 过大 ' ...
        '(dt*K1/M=%.2f, 极点 %+.3f 符号交替)：显式欧拉欠采样阻尼，' ...
        'Eq.(6) 的力矩响应会被失真放大。建议 dt <= %.4g s (>= %.0f Hz)，' ...
        '或对 K1*qd 阻尼项做隐式/半隐式积分。'], ...
        p.dt, tau, ratio, pole, rec, 1/rec);
end
end
