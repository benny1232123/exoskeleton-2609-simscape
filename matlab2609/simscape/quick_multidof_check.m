function info = quick_multidof_check()
%QUICK_MULTIDOF_CHECK  快速功能测试（不积分）：语法 + 类加载 + M(0)/G(0) 基准比对
%
%   比 test_multidof_analytical 快得多（不做 ode45 积分），
%   用于改完 exo2609.multidof 后先确认没写坏。

here  = fileparts(mfilename('fullpath'));    % .../matlab2609/simscape
rootd = fileparts(here);                     % .../matlab2609
logf  = fullfile(here,'quick_multidof_check_log.txt');
fid   = fopen(logf,'w'); c = onCleanup(@() fclose(fid));
fprintf(fid,'== quick_multidof_check ==\n%s\n\n', datestr(now));

ok = true;

% ---------------------------------------------------------------- 语法
cls = fullfile(rootd,'+exo2609','multidof.m');
fprintf(fid,'-- 语法检查 %s --\n', cls);
msgs = checkcode(cls,'-struct');
fprintf(fid,'  checkcode: %d 条\n', numel(msgs));
for k = 1:numel(msgs)
    fprintf(fid,'    L%-4d %s\n', msgs(k).line, msgs(k).message);
end
ok = ok && isempty(msgs);

% ---------------------------------------------------------------- 加载
fprintf(fid,'\n-- 构造 exo2609.multidof --\n');
try
    addpath(rootd);
    mb = exo2609.multidof();
    fprintf(fid,'  OK  nq=%d  q_names=%s\n', mb.nq, strjoin(mb.q_names,', '));
    fprintf(fid,'  总质量 = %.9f kg\n', sum([mb.links.mass]));
catch ME
    fprintf(fid,'  FAIL %s | %s\n', ME.identifier, ME.message);
    if ~isempty(ME.stack)
        fprintf(fid,'       @ %s line %d\n', ME.stack(1).name, ME.stack(1).line);
    end
    fprintf(fid,'\nQUICK_MULTIDOF_FAIL\n');
    info = struct('ok',false); return
end

% ---------------------------------------------------------------- 基准
% ★ 唯一真源 = _multidof_dyn.py 自检写出的 multidof_baseline.json。
%   以前这两行是硬编码的，密度表一改（0.016975773 -> 0.016997395、
%   0.438275226 -> 0.441845081）本测试就会假 FAIL —— 属于「基线副本漂移」。
Mref = [0.016997395 0.011527605 0.016997437 0.011527226];   % 仅作 json 缺失时的回退
Gref = [0.441845081; -0.035976693; 0.441832832; 0.043426856];
basef = fullfile(here,'multidof_baseline.json');
if ~isempty(dir(basef))
    b = jsondecode(fileread(basef));
    if numel(b.M0_diag) == numel(Mref), Mref = b.M0_diag(:).'; end
    if numel(b.G0)      == numel(Gref), Gref = b.G0(:);         end
    fprintf(fid,'baseline : %s  [%s]\n', basef, b.source);
else
    fprintf(fid,'baseline : *** HARDCODED FALLBACK ***  %s 不存在\n', basef);
end

M0 = mb.massmatrix(zeros(mb.nq,1));
G0 = mb.gravity(zeros(mb.nq,1));

fprintf(fid,'\n-- M(0) 对角 vs Python 侧基准 --\n');
fprintf(fid,'  %-12s %16s %16s %12s\n','joint','matlab','python','rel');
for i = 1:mb.nq
    rel = abs(M0(i,i)-Mref(i))/abs(Mref(i));
    fprintf(fid,'  %-12s %16.9f %16.9f %12.2e\n', mb.q_names{i}, M0(i,i), Mref(i), rel);
    ok = ok && rel < 1e-6;
end

fprintf(fid,'\n-- G(0) vs Python 侧基准 --\n');
fprintf(fid,'  %-12s %16s %16s %12s\n','joint','matlab','python','abs');
for i = 1:mb.nq
    fprintf(fid,'  %-12s %+16.9f %+16.9f %12.2e\n', ...
            mb.q_names{i}, G0(i), Gref(i), abs(G0(i)-Gref(i)));
    ok = ok && abs(G0(i)-Gref(i)) < 1e-8;
end

% ---------------------------------------------------------------- 结构性质
rng(0);
worstSym = 0; minEig = inf;
for k = 1:20
    q = (rand(mb.nq,1)-0.5)*2.4;
    Mx = mb.massmatrix(q);
    worstSym = max(worstSym, max(abs(Mx-Mx.'),[],'all'));
    minEig = min(minEig, min(eig(Mx)));
end
fprintf(fid,'\n-- M(q) 20 随机位姿：max|M-M''| = %.3e   最小特征值 = %.9f\n', worstSym, minEig);
ok = ok && worstSym < 1e-12 && minEig > 0;

% ---------------------------------------------------------------- 初始加速度
% 与 Simscape CSV 前几个点反推的初始加速度对照（不掺时间积分误差，最干净的判据）
csv = fullfile(rootd,'out_simscape','multidof_traj.csv');
if ~isempty(dir(csv))
    fh = fopen(csv,'r'); hdr = strsplit(strtrim(fgetl(fh)),','); fclose(fh);
    D  = readmatrix(csv,'NumHeaderLines',1);
    fprintf(fid,'\n-- 初始加速度：解析 vs Simscape（由前几个点反推）--\n');
    fprintf(fid,'  %-12s %16s %16s %12s\n','joint','analytic','simscape','rel');
    for i = 1:mb.nq
        col = find(strcmp(hdr,[mb.q_names{i} '.w']),1);
        if isempty(col), continue, end
        % 用最早两个非零点做二次拟合 w(t)=a t + b t^2 并取 a
        idx = find(abs(D(:,col)) > 0, 3, 'first');
        if numel(idx) < 2, continue, end
        t1 = D(idx(2),1); w1 = D(idx(2),col);
        t2 = D(idx(3),1); w2 = D(idx(3),col);
        A = [t1 t1^2; t2 t2^2];
        ab = A \ [w1; w2];
        a_sim = ab(1);
        a_an  = mb.accel(zeros(mb.nq,1), zeros(mb.nq,1), zeros(mb.nq,1));
        a_an  = a_an(i);
        rel = abs(a_an-a_sim)/max(abs(a_sim),eps);
        fprintf(fid,'  %-12s %16.6f %16.6f %12.2e\n', mb.q_names{i}, a_an, a_sim, rel);
        ok = ok && rel < 5e-3;
    end
else
    fprintf(fid,'\n-- 初始加速度：SKIP（%s 不存在）--\n', csv);
end

fprintf(fid,'\nVERDICT: %s\n', ternary(ok,'QUICK_OK','QUICK_FAIL'));
fprintf(fid,'QUICK_MULTIDOF_%s\n', ternary(ok,'OK','FAIL'));
info = struct('ok',ok,'log',logf);
end

function s = ternary(tf,a,b), if tf, s=a; else, s=b; end, end
