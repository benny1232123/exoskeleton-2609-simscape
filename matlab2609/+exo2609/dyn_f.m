function ydot = dyn_f(p, y, T)
%DYN_F  Eq.(4)：连续状态导数 ydot = f(y, T)，状态 y = [q; qd]。
%
%   ydot = exo2609.dyn_f(p, y, T)
%
%   qdd = Minv * ( T - C*qd + gs*G(q) - T_int ),  T_int = K1*qd + T0
%   G(q) = G_amp .* sin(q)，gs = p.gravity_sign
%
%   见 also exo2609.dyn_step, exo2609.dyn_jac

q  = y(1:p.nq);
qd = y(p.nq+1:2*p.nq);
T  = T(:);

qdd = p.M \ ( T - p.C_coef*qd ...
              + p.gravity_sign * (p.G_amp .* sin(q)) ...
              - (p.K1*qd + p.T0) );

ydot = [qd; qdd];
end
