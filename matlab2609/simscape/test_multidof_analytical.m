function info = test_multidof_analytical()
%TEST_MULTIDOF_ANALYTICAL  验证 exo2609.multidof（MATLAB 侧 4-DOF 解析组装）
%
%   三重核对：
%     A) M(0) / G(0) 与 **Python 侧独立组装器**（_multidof_dyn.py）的基准逐位比
%     B) 自由落体轨迹 vs Simscape 导出的 multidof_traj.csv（第三个独立实现）
%     C) 能量守恒
%
%   前提（跑一次即可）：
%     python _multidof_export.py        -> simscape/plant_multidof.json
%     python _multidof_dyn.py           -> 打印 A) 用的基准值
%     matlab dump_multidof_traj         -> out_simscape/multidof_traj.csv
%
%   ★ 本脚本**不改**任何 2-DOF 论文口径的东西（params_paper / dyn_f /
%     compare_simscape_vs_analytical 都不碰）。

here = fileparts(fileparts(mfilename('fullpath')));   % .../matlab2609
simd = fullfile(here,'simscape');
outd = fullfile(here,'out_simscape');
logf = fullfile(simd,'test_multidof_analytical_log.txt');
fid  = fopen(logf,'w'); c = onCleanup(@() fclose(fid));
fprintf(fid,'== test_multidof_analytical ==\n%s\n\n', datestr(now));

addpath(here);
mb = exo2609.multidof();
fprintf(fid,'model   : exo2609.multidof  (nq = %d)\n', mb.nq);
fprintf(fid,'q_names : %s\n', strjoin(mb.q_names, ', '));
fprintf(fid,'总质量  : %.9f kg\n\n', sum([mb.links.mass]));

okA = true; okB = true; okC = true;

% =====================================================================
% A) 与 Python 侧组装器的基准对比
%    基准 = _multidof_dyn.py 自检输出的 M(0) 对角 与 G(0)
%    两侧是**独立实现**（Python: numpy；MATLAB: 本类），吻合才有意义
%    ★ 基准的唯一真源 = simscape/multidof_baseline.json（由 _multidof_dyn.py 写出）。
%      这里**不再硬编码**：密度表一改，旧硬编码值就会让本测试假 FAIL。
% =====================================================================
Mref = [0.016997395 0.011527605 0.016997437 0.011527226];   % 仅作 json 缺失时的回退
Gref = [0.441845081; -0.035976693; 0.441832832; 0.043426856];
basef = fullfile(simd,'multidof_baseline.json');
if ~isempty(dir(basef))
    b = jsondecode(fileread(basef));
    if numel(b.M0_diag) == numel(Mref), Mref = b.M0_diag(:).'; end
    if numel(b.G0)      == numel(Gref), Gref = b.G0(:);         end
    fprintf(fid,'baseline    : %s  [%s]\n', basef, b.source);
else
    fprintf(fid,'baseline    : *** HARDCODED FALLBACK ***  %s 不存在\n', basef);
    fprintf(fid,'              -> 先跑 python _multidof_dyn.py\n');
end

M0 = mb.massmatrix(zeros(mb.nq,1));
G0 = mb.gravity(zeros(mb.nq,1));

fprintf(fid,'-- A) MATLAB 组装 vs Python 组装（独立实现）--\n');
fprintf(fid,'  %-12s %16s %16s %12s\n','M(0) 对角','matlab','python','rel');
for i = 1:mb.nq
    rel = abs(M0(i,i)-Mref(i))/abs(Mref(i));
    fprintf(fid,'  %-12s %16.9f %16.9f %12.2e\n', mb.q_names{i}, M0(i,i), Mref(i), rel);
    okA = okA && rel < 1e-6;
end
fprintf(fid,'  %-12s %16s %16s %12s\n','G(0)','matlab','python','abs');
for i = 1:mb.nq
    fprintf(fid,'  %-12s %+16.9f %+16.9f %12.2e\n', mb.q_names{i}, G0(i), Gref(i), abs(G0(i)-Gref(i)));
    okA = okA && abs(G0(i)-Gref(i)) < 1e-8;
end
fprintf(fid,'  -> A) %s\n\n', ternary(okA,'OK','FAIL'));

% 顺带把「零位有重力矩」这件事显式打出来
fprintf(fid,'  零位重力矩 G(0) 的物理含义：CAD 零位不是重力平衡位，\n');
fprintf(fid,'  所以 T=0 时腿会自己动（与 2-DOF 侧 B != 0 是同一个现象）。\n\n');

% =====================================================================
% B) 自由落体 vs Simscape
% =====================================================================
csv = fullfile(outd,'multidof_traj.csv');
if isempty(dir(csv))
    fprintf(fid,'-- B) SKIP：%s 不存在（先跑 dump_multidof_traj）--\n\n', csv);
    okB = false;
else
    fh = fopen(csv,'r'); hdr = strsplit(strtrim(fgetl(fh)),','); fclose(fh);
    D  = readmatrix(csv,'NumHeaderLines',1);
    t  = D(:,1);

    % 按**列名**映射（Simscape 的 xout 是按块名字母序交错的，别假设顺序）
    iq = zeros(1,mb.nq); iw = zeros(1,mb.nq);
    for i = 1:mb.nq
        iq(i) = find(strcmp(hdr,[mb.q_names{i} '.q']),1);
        iw(i) = find(strcmp(hdr,[mb.q_names{i} '.w']),1);
        if isempty(iq(i)) || isempty(iw(i))
            error('CSV 里找不到 %s.q / %s.w', mb.q_names{i}, mb.q_names{i});
        end
    end
    qs = D(:,iq); ws = D(:,iw);
    fprintf(fid,'-- B) 自由落体：MATLAB 解析 vs Simscape --\n');
    fprintf(fid,'  CSV %d 点, t in [%.4f, %.4f]\n', size(D,1), t(1), t(end));
    fprintf(fid,'  列映射: %s\n', strjoin(arrayfun(@(k) sprintf('%s<-col%d',mb.q_names{k},iq(k)), ...
            1:mb.nq,'UniformOutput',false),', '));

    opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
    z4   = zeros(mb.nq,1);
    odef = @(tt,y) [y(mb.nq+1:end); mb.accel(y(1:mb.nq), y(mb.nq+1:end), z4)];
    sol  = ode45(odef, [t(1) t(end)], zeros(2*mb.nq,1), opts);

    Yq = deval(sol, t).';                 % nt x 2nq
    names = [mb.q_names, cellfun(@(s) ['d_' s], mb.q_names, 'UniformOutput',false)];
    fprintf(fid,'  %-14s %13s %13s %13s %12s\n', ...
            'channel','max|err|','rms(err)','range(sim)','rms/range');
    worst = 0; worstAt='';
    for i = 1:mb.nq
        for pass = 1:2
            if pass == 1, ref = qs(:,i); got = Yq(:,i);   nm = names{i};
            else,         ref = ws(:,i); got = Yq(:,mb.nq+i); nm = names{mb.nq+i};
            end
            e  = got - ref;
            rg = max(ref) - min(ref);
            rel = sqrt(mean(e.^2)) / max(rg,eps);
            fprintf(fid,'  %-14s %13.4e %13.4e %13.6f %12.4e\n', ...
                    nm, max(abs(e)), sqrt(mean(e.^2)), rg, rel);
            if rel > worst, worst = rel; worstAt = nm; end
        end
    end
    fprintf(fid,'  worst: %s  rms/range = %.4e\n', worstAt, worst);
    okB = worst < 1e-2;
    fprintf(fid,'  -> B) %s   (判据 1e-2，沿用 2-DOF 那条链)\n\n', ternary(okB,'OK','FAIL'));
end

% =====================================================================
% C) 能量守恒
% =====================================================================
fprintf(fid,'-- C) 能量守恒（T=0, dt=1e-4, 0.5 s）--\n');
% ⚠ energy 的**第一个输出是 KE 不是 E**。只写 E0 = mb.energy(...) 拿到的是动能，
%   动能当然不守恒（2026-09-20 曾因此误判 C) FAIL，drift 0.447 J 恰是动能峰值）。
%   必须显式取第三个输出。
z4 = zeros(mb.nq,1);
y  = zeros(2*mb.nq,1);
dt = 1e-4; nst = round(0.5/dt);
[KE0, PE0, E0] = mb.energy(y(1:mb.nq), y(mb.nq+1:end));
drift = 0;
for k = 1:nst
    y = rk4(@(yy) [yy(mb.nq+1:end); mb.accel(yy(1:mb.nq), yy(mb.nq+1:end), z4)], y, dt);
    if mod(k,500)==0
        [~, ~, E] = mb.energy(y(1:mb.nq), y(mb.nq+1:end));
        drift = max(drift, abs(E-E0));
    end
end
[KEf, PEf, Ef] = mb.energy(y(1:mb.nq), y(mb.nq+1:end));
relD = drift / max(abs(E0),eps);
fprintf(fid,'  零位: KE0 = %.12f, PE0 = %.12f, E0 = %.12f J\n', KE0, PE0, E0);
fprintf(fid,'  末态: KEf = %.12f, PEf = %.12f, Ef = %.12f J\n', KEf, PEf, Ef);
fprintf(fid,'  max|dE| = %.3e J  (rel %.2e)   判据 1e-7（地板是数值微分）\n', drift, relD);
okC = relD < 1e-7;
fprintf(fid,'  -> C) %s\n\n', ternary(okC,'OK','FAIL'));

% =====================================================================
verdict = ternary(okA && okB && okC, 'ALL_OK', 'CHECK_FAILED');
fprintf(fid,'VERDICT: %s\n', verdict);
fprintf(fid,'TEST_MULTIDOF_%s\n', verdict);
info = struct('ok',strcmp(verdict,'ALL_OK'),'A',okA,'B',okB,'C',okC,'log',logf);
end

% ---------------------------------------------------------------- helpers
function s = ternary(tf,a,b), if tf, s=a; else, s=b; end, end

function y = rk4(f, y, h)
k1 = f(y); k2 = f(y+0.5*h*k1); k3 = f(y+0.5*h*k2); k4 = f(y+h*k3);
y  = y + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
