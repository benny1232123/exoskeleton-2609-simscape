function w = weights_paper()
%WEIGHTS_PAPER  Eq.(6) 的权重标量（论文 Table II）。
%
%   Q_k (k = 1..L-1) = diag(Cq*I2, Cv*I2)   ∈ R^{4x4}  角度/角速度误差
%   Q_L              = Cp*I4                ∈ R^{4x4}  终端误差
%   P_k              = Ca*I2                ∈ R^{2x2}  力矩惩罚（正则项）
%
%   Table II: Q = diag{10, 0.1} I -> Cq = 10, Cv = 0.1
%             P = 0.5 I           -> Ca = 0.5
%   Cp 论文未给出，默认取 Cq。
%
%   ⚠ Ca 是「估计助力幅值」的标定旋钮，不是纯数值参数：
%     窗口线性化后 A* = sum_i [lam_i/(lam_i+Ca)] (v_i'*A_true) v_i，
%     收缩因子就是 lam_i/(lam_i+Ca)。观测 Hessian 病态时，
%     Ca=0.5 会把估计力矩压到真值幅值的约 11%。诊断见 exo2609.spectrum。
%
%   见 also exo2609.q_blocks, exo2609.cost_grad

w = struct('Cq', 10.0, 'Cv', 0.1, 'Ca', 0.5, 'Cp', 10.0);
end
