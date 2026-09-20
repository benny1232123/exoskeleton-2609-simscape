function info = demo_multidof()
%DEMO_MULTIDOF  4-DOF 解析模型 exo2609.multidof 的典型用法速查（可直接抄改）
%
%   跑法：
%     cd C:\Users\29408\exo_work\matlab2609\simscape
%     demo_multidof
%
%   前提：simscape/plant_multidof.json 已存在（由 python _multidof_export.py 生成）。
%   本脚本**只读不写**模型，不碰论文口径的 2-DOF 链路。
%
%   五个用法：
%     [1] 参数查询   —— 任意姿态下的 M(q) / G(q)，以及零力矩初始加速度
%     [2] 正动力学   —— 给定力矩，积分出轨迹（ode45）
%     [3] 逆动力学   —— 给定轨迹，算需要多大关节力矩
%     [4] 能量自检   —— 一行，守恒性快查
%     [5] 正运动学   —— 各段质心/位姿
%
%   见 also exo2609.multidof, MULTIDOF_DYNAMICS_GUIDE.md

here = fileparts(fileparts(mfilename('fullpath')));   % .../matlab2609
addpath(here);
logf = fullfile(here, 'simscape', 'demo_multidof_log.txt');
fid  = fopen(logf, 'w'); c = onCleanup(@() fclose(fid));

mb = exo2609.multidof();
z  = zeros(mb.nq, 1);

fprintf(fid, '== demo_multidof ==\n%s\n', datestr(now));
fprintf(fid, 'nq = %d    q_names = %s\n', mb.nq, strjoin(mb.q_names, ', '));
fprintf(fid, '总质量 = %.9f kg\n', sum([mb.links.mass]));

% =====================================================================
fprintf(fid, '\n[1] 参数查询：任意姿态下的 M(q) 与 G(q)\n');
q = [0.30; -0.20; 0.30; 0.20];
M = mb.massmatrix(q);
G = mb.gravity(q);
fprintf(fid, '    q      = [%s]\n', num2str(q.', '%.4f '));
fprintf(fid, '    M(q) =\n');
for i = 1:mb.nq
    fprintf(fid, '      %-9s ', mb.q_names{i});
    fprintf(fid, '%15.6e', M(i, :));      % 宽度 >=14 才不会被负号挤掉分隔
    fprintf(fid, '\n');
end
fprintf(fid, '    min eig(M) = %.6e    max|非对角| = %.6e\n', ...
        min(eig(M)), max(max(abs(M - diag(diag(M))))));
fprintf(fid, '    G(q) =\n');
for i = 1:mb.nq
    fprintf(fid, '      %-9s %+ .9f N*m\n', mb.q_names{i}, G(i));
end
fprintf(fid, '    -> 零力矩下的初始角加速度（qdd = M \\ -G）：\n');
qdd0 = mb.accel(q, z, z);
for i = 1:mb.nq
    fprintf(fid, '      %-9s %+ .9f rad/s^2\n', mb.q_names{i}, qdd0(i));
end
fprintf(fid, '    ⚠ G(q) 不是 0：CAD 零位不是重力平衡位，T=0 时腿会自己动。\n');

% =====================================================================
fprintf(fid, '\n[2] 正动力学：给定力矩 -> 轨迹（此处 T=0 自由落体，0.5 s，ode45）\n');
odef = @(t, y) [y(mb.nq+1:end); mb.accel(y(1:mb.nq), y(mb.nq+1:end), z)];
sol  = ode45(odef, [0 0.5], zeros(2*mb.nq, 1), ...
             odeset('RelTol', 1e-10, 'AbsTol', 1e-12));
yf   = deval(sol, 0.5);
fprintf(fid, '      %-9s %14s %14s\n', 'joint', 'q(T)', 'qd(T)');
for i = 1:mb.nq
    fprintf(fid, '      %-9s %+14.6f %+14.6f\n', mb.q_names{i}, yf(i), yf(mb.nq+i));
end
fprintf(fid, '    -> 换成有力矩：把 z 换成 T(t) 即可（T 必须是 %d x 1）。\n', mb.nq);

% =====================================================================
fprintf(fid, '\n[3] 逆动力学：给定轨迹 -> 需要多大关节力矩\n');
fprintf(fid, '    取 q_i(t) = A*sin(2*pi*f*t)，A = 0.2 rad，f = 0.5 Hz，t in [0,1]，1 ms 步长\n');
Aq = 0.2; f = 0.5; w = 2*pi*f;
tt   = (0:1e-3:1).';
qs   = Aq*sin(w*tt);  qds = Aq*w*cos(w*tt);  qdds = -Aq*w^2*sin(w*tt);
Tau  = zeros(numel(tt), mb.nq);
for k = 1:numel(tt)
    qk   = qs(k)*ones(mb.nq,1);
    qdk  = qds(k)*ones(mb.nq,1);
    qddk = qdds(k)*ones(mb.nq,1);
    % tau = M(q)*qdd + C(q,qd)*qd + G(q)      ← 逆动力学就这一行
    Tau(k, :) = (mb.massmatrix(qk)*qddk + mb.coriolis(qk, qdk) + mb.gravity(qk)).';
end
fprintf(fid, '      %-9s %16s %16s\n', 'joint', 'peak|tau| [N*m]', 'rms(tau)');
for i = 1:mb.nq
    fprintf(fid, '      %-9s %16.6f %16.6f\n', mb.q_names{i}, max(abs(Tau(:,i))), rms(Tau(:,i)));
end
[v, k] = max(abs(Tau(:)));
[kk, jj] = ind2sub(size(Tau), k);
fprintf(fid, '    -> 全局峰值 %.6f N*m  @ t = %.3f s, %s\n', v, tt(kk), mb.q_names{jj});
fprintf(fid, '    ⚠ 这是"保持这条轨迹所需的净关节力矩"，未含摩擦/传动损耗。\n');

% =====================================================================
fprintf(fid, '\n[4] 能量自检（一行）\n');
[KE0, PE0, E0] = mb.energy(z, z);
fprintf(fid, '      E(0,0) = %.12f J   (KE0 = %.3e, PE0 = %.12f)\n', E0, KE0, PE0);
fprintf(fid, '      预期与 Python 侧一致：-0.071583229124 J\n');
fprintf(fid, '    ⚠ 必须写 [~,~,E] = mb.energy(...)：\n');
fprintf(fid, '      E = mb.energy(...) 只拿到**第一个**输出 = 动能 KE，拿去做守恒判据必假 FAIL。\n');

% =====================================================================
fprintf(fid, '\n[5] 正运动学：各段 frame 原点 与 质心（单位 m，与 URDF 一致）\n');
T  = mb.fk(z);            % 位姿：T.(link) = 4x4 世界位姿
cw = mb.com_world(z);     % 质心：cw.(link) = 3x1
nm = fieldnames(T);
for i = 1:numel(nm)
    fprintf(fid, '      %-12s frame = [%+ .4f %+ .4f %+ .4f]   CoM = [%+ .4f %+ .4f %+ .4f]\n', ...
            nm{i}, T.(nm{i})(1:3,4), cw.(nm{i}));
end
fprintf(fid, '    -> fk(q) 给 4x4 位姿（可反解 R 看姿态）；com_world(q) 直接给质心。\n');

fprintf(fid, '\n== done ==\n');
fprintf(fid, 'DEMO_MULTIDOF_DONE\n');
info = struct('ok', true, 'nq', mb.nq, 'log', logf);
end
