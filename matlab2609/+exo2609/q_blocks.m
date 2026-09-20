function Q = q_blocks(w, L)
%Q_BLOCKS  构造 Eq.(6) 的分块误差权重 Q_k（元胞，k = 1..L）。
%
%   Q = exo2609.q_blocks(w, L)
%
%   Q{k}  = diag([Cq Cq Cv Cv])  (k < L，作用于 [q; qd] 4 维误差)
%   Q{L}  = Cp * eye(4)          (终端误差)
%
%   见 also exo2609.weights_paper, exo2609.cost_grad

Q = cell(L, 1);
qblk = diag([w.Cq, w.Cq, w.Cv, w.Cv]);
for k = 1:L
    if k == L
        Q{k} = w.Cp * eye(4);
    else
        Q{k} = qblk;
    end
end
end
