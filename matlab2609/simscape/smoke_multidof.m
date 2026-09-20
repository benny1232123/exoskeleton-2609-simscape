function smoke_multidof()
%SMOKE_MULTIDOF  对 3 个模型各跑一次短仿真，并做「拆段回归」
%
%  sm_exo_real              : 原 2-DOF（leg 是一个刚体）
%  sm_exo_multidof_locked   : 拆成 3 段、abduct 也 fixed -> **应与上面逐位一致**
%  sm_exo_multidof          : abduct 可动 / slide 仍 fixed（4 DOF = 8 状态）
%
%  判据：状态维数相同 + 末态差 ~0（浮点级）。若 locked 与 real 不一致，
%  说明「按零件归属拆段」这一步改变了动力学（质量/惯量/关节位置有误）。
%
%  ★ 必须先 prep_for_dynamics()（2026-09-20 修正）
%  ------------------------------------------------
%  本脚本原先把 smimport 的产物**原样**拿去 sim，于是：
%    · URDF 的 <limit> ±1.5 rad 变成 1e4 N*m/deg 的硬弹簧；
%    · 而自由落体 0.5 s 的摆幅会越过 -1.5 rad ⇒ 轨迹被这跟弹簧主导；
%    · 求解器还用导入默认值（容差偏松）。
%  结果是 real vs locked 差 1.68e-03 被报成「拆段改变了动力学」——
%  典型的假 FAIL：比的其实是「撞硬限位后的数值灵敏度」。
%  （同一坑 build_harness_exo2dof.m 早已写明；dump_multidof_traj.m 也已按
%    正常做法关限位。这里补齐。）
%  修好后判定的是纯动力学等价性，与解析侧（无限位）同口径。

simd = fileparts(mfilename('fullpath'));
logf = fullfile(simd,'smoke_multidof_log.txt');
fid = fopen(logf,'w'); c = onCleanup(@() fclose(fid));

mds = {'sm_exo_real','sm_exo_multidof_locked','sm_exo_multidof'};
R = cell(1,numel(mds));

for i = 1:numel(mds)
    mdl = mds{i};
    fprintf(fid,'\n===================== %s =====================\n', mdl);
    try
        if bdIsLoaded(mdl), close_system(mdl,0); end
        load_system(fullfile(simd,[mdl '.slx']));
        prep_for_dynamics(mdl, fid);      % ★ 关硬限位 + 定重力 + 紧求解器
        set_param(mdl,'StopTime','0.5','SaveState','on','SaveOutput','on', ...
                      'SaveFormat','Dataset','ReturnWorkspaceOutputs','on');
        so = sim(mdl);
        names = so.who;
        fprintf(fid,'outputs : %s\n', strjoin(cellstr(names),', '));
        [t,x] = pull_x(so, fid);
        if isempty(x)
            fprintf(fid,'!!! 拿不到状态向量\n');
        else
            fprintf(fid,'sim OK   t_end=%.6f  n_states=%d\n', t(end), size(x,2));
            fprintf(fid,'x0 = %s\n', mat2str(x(1,:),8));
            fprintf(fid,'xT = %s\n', mat2str(x(end,:),8));
            R{i} = struct('ok',true,'x0',x(1,:),'xT',x(end,:));
        end
    catch ME
        fprintf(fid,'sim FAIL : %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf(fid,'   at %s line %d\n', ME.stack(1).name, ME.stack(1).line);
        end
    end
    try, close_system(mdl,0); catch, end
end

% ---------------- 回归 ----------------
fprintf(fid,'\n===================== 回归 =====================\n');
if ~isempty(R{1}) && ~isempty(R{2})
    a = R{1}.xT; b = R{2}.xT;
    if numel(a) == numel(b)
        d = max(abs(a(:)-b(:)));
        fprintf(fid,'real vs locked : 状态数相同 (%d)\n', numel(a));
        fprintf(fid,'max| xT_real - xT_locked | = %.3e\n', d);
        if d < 1e-6
            fprintf(fid,'VERDICT: PASS  —— 拆段+fixed 精确重现原 2-DOF\n');
        else
            fprintf(fid,'VERDICT: FAIL  —— 拆段改变了动力学\n');
        end
    else
        fprintf(fid,'VERDICT: FAIL  —— 状态数不同 real=%d locked=%d\n', numel(a), numel(b));
    end
else
    fprintf(fid,'VERDICT: SKIP (有模型没跑起来)\n');
end
if ~isempty(R{3})
    fprintf(fid,'free  模型跑通, 末态 = %s\n', mat2str(R{3}.xT,6));
else
    fprintf(fid,'free  模型未跑通\n');
end
fprintf(fid,'\nSMOKE_MULTIDOF_DONE\n');
end

% ------------------------------------------------------------------
function [t,x] = pull_x(so, fid)
% 从 SimulationOutput 里尽力取出 (t, x)。x 为 [nt x nstate]。
t = []; x = [];
for nm = {'xout','yout','logsout'}
    try
        v = so.get(nm{1});
    catch
        continue
    end
    if isempty(v), continue, end
    fprintf(fid,'  %s : %s\n', nm{1}, class(v));
    [tt,xx] = flatten(v);
    if ~isempty(xx) && strcmp(nm{1},'xout')
        t = tt; x = xx; return
    elseif isempty(x) && ~isempty(xx)
        t = tt; x = xx;
    end
end
end

function [t,x] = flatten(v)
t = []; x = [];
if isa(v,'timeseries')
    t = v.Time(:); d = v.Data;
    if ndims(d) == 2
        if size(d,1) == numel(t), x = d; else, x = d.'; end
    else
        x = reshape(d, [], numel(t)).';
    end
elseif isa(v,'Simulink.SimulationData.Dataset') || isa(v,'Simulink.SimulationData.DatasetRef')
    ts = {}; xs = {};
    for k = 1:v.numElements
        [tk, xk] = flatten(v.getElement(k).Values);
        if ~isempty(xk), ts{end+1} = tk; xs{end+1} = xk; end %#ok<AGROW>
    end
    if ~isempty(xs)
        t = ts{1};
        x = cat(2, xs{:});
        n = min(size(x,1), numel(t));
        x = x(1:n,:); t = t(1:n);
    end
elseif isstruct(v) && isfield(v,'signals')
    t = v.time(:);
    x = zeros(numel(t),0);
    for k = 1:numel(v.signals)
        d = v.signals(k).values;
        if size(d,1) ~= numel(t), d = d.'; end
        x = [x d]; %#ok<AGROW>
    end
end
end

% ------------------------------------------------------------------
function prep_for_dynamics(mdl, fid)
% 把「CAD 导入模型」摆成可与解析侧对照的状态：
%   1) 重力 [0 0 -9.81]  —— 与 +exo2609 / build_harness_exo2dof 一致
%   2) 关掉所有 Revolute/Prismatic 关节的硬限位
%      （URDF <limit> 在 Simscape 是 1e4 N*m/deg 硬弹簧；解析侧没有它）
%   3) 紧求解器：ode23t / MaxStep 1e-3 / RelTol 1e-6 / AbsTol 1e-8
%      ——沿用 build_harness_exo2dof.m 的设置，否则判定地板由求解器决定
fprintf(fid,'  -- prep_for_dynamics --\n');

mc = find_system(mdl,'MaskType','Mechanism Configuration');
if ~isempty(mc)
    try
        before = get_param(mc{1},'GravityVector');
        set_param(mc{1},'GravityVector','[0 0 -9.81]');
        fprintf(fid,'  gravity  %s -> [0 0 -9.81]\n', before);
    catch ME
        fprintf(fid,'  gravity  FAIL %s\n', ME.message);
    end
end

blks = find_system(mdl,'SearchDepth',Inf,'Type','Block');
nj = 0; noff = 0;
for k = 1:numel(blks)
    mt = '';
    try, mt = get_param(blks{k},'MaskType'); end
    if isempty(strfind(lower(mt),'joint')), continue, end
    nj = nj + 1;
    if ~isempty(strfind(lower(mt),'weld')), continue, end   % fixed 关节：无限位可关
    try
        set_param(blks{k},'LowerLimitSpecify','off');
        set_param(blks{k},'UpperLimitSpecify','off');
        noff = noff + 1;
    catch ME
        fprintf(fid,'  limit    FAIL %s : %s\n', blks{k}, ME.message);
    end
end
fprintf(fid,'  joints   %d 个，其中 %d 个已关限位\n', nj, noff);

try
    set_param(mdl,'SolverType','Variable-step','Solver','ode23t', ...
                  'MaxStep','1e-3','RelTol','1e-6','AbsTol','1e-8');
    fprintf(fid,'  solver   ode23t  MaxStep=1e-3  RelTol=1e-6  AbsTol=1e-8\n');
catch ME
    fprintf(fid,'  solver   FAIL %s\n', ME.message);
end
end
