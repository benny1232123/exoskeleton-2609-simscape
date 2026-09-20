function info = dump_multidof_traj(Tend)
%DUMP_MULTIDOF_TRAJ  从 sm_exo_multidof 导出 4-DOF 自由落体轨迹，供解析侧对照。
%
%   dump_multidof_traj        % 默认 2.5 s
%   dump_multidof_traj(1.0)
%
%   为什么用「自由落体」就够了
%     sm_exo_multidof 是纯 CAD 导入模型（无 From Workspace 依赖、无力矩输入），
%     所以它天然就是 T = 0。而 CAD 零位**不是**重力平衡位（零位挂着约 0.44 N*m
%     的重力矩，即 2-DOF 那边 B != 0 的同一个现象），因此 T=0 就已经是一次
%     充分的自由落体 —— 足以把 M(q) 与 G(q) 同时激励起来，且 abduct 也会动。
%
%   ★ 必须关掉关节硬限位：URDF 的 <limit> 在 Simscape 里变成 1e4 N*m/deg 的
%     硬弹簧，解析模型没有它，一旦轨迹碰到 ±1.5 rad 两边必然发散（README 坑 #4）。
%   ★ 不保存模型：只改内存里的参数，保持 sm_exo_multidof.slx 是 pristine 导入产物。
%
%   输出 matlab2609/out_simscape/multidof_traj.csv（均匀网格，供 Python 侧读）
%        matlab2609/simscape/dump_multidof_traj_log.txt

simd = fileparts(mfilename('fullpath'));
cd(simd);
if nargin < 1 || isempty(Tend), Tend = 2.5; end

mdl  = 'sm_exo_multidof';
slx  = fullfile(simd,[mdl '.slx']);
logf = fullfile(simd,'dump_multidof_traj_log.txt');
outd = fullfile(fileparts(simd),'out_simscape');
if exist(outd,'dir') ~= 7, mkdir(outd); end
csv  = fullfile(outd,'multidof_traj.csv');

fid = fopen(logf,'w'); c = onCleanup(@() fclose(fid));
fprintf(fid,'== dump_multidof_traj ==\n%s\nTend = %.4f s\n\n', datestr(now), Tend);

if isempty(dir(slx))
    fprintf(fid,'FAIL: %s missing - run run_remultidof first\n', slx);
    error('dump_multidof_traj:noSlx','%s missing', slx);
end
if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(slx);

% ---------------------------------------------------------------- gravity
mc = find_system(mdl,'MaskType','Mechanism Configuration');
if ~isempty(mc)
    fprintf(fid,'gravity before : %s\n', get_param(mc{1},'GravityVector'));
    set_param(mc{1},'GravityVector','[0 0 -9.81]');
    fprintf(fid,'gravity after  : %s\n', get_param(mc{1},'GravityVector'));
else
    fprintf(fid,'gravity        : MechanismConfiguration NOT FOUND\n');
end

% ---------------------------------------------------------------- joints
blks = find_system(mdl,'SearchDepth',Inf,'Type','Block');
jnt = {};
for k = 1:numel(blks)
    mt = '';
    try, mt = get_param(blks{k},'MaskType'); end
    if ~isempty(strfind(lower(mt),'joint')), jnt{end+1} = blks{k}; end %#ok<AGROW>
end
fprintf(fid,'\njoint blocks (%d):\n', numel(jnt));
for k = 1:numel(jnt)
    fprintf(fid,'  [%d] %s\n', k, jnt{k});
end

fprintf(fid,'\n-- 关硬限位（解析侧没有限位，必须一致）--\n');
for k = 1:numel(jnt)
    mt = '';
    try, mt = get_param(jnt{k},'MaskType'); end
    if ~isempty(strfind(lower(mt),'weld'))
        % URDF 里 type="fixed" 的关节（本工程的 slide_*）在 Simscape 里是
        % Weld Joint，**没有** LowerLimitSpecify 参数 —— 不是错误，直接跳过。
        fprintf(fid,'  %-46s Weld Joint（fixed 关节）—— 无限位可关\n', jnt{k});
        continue
    end
    try
        lo = get_param(jnt{k},'LowerLimitSpecify');
        up = get_param(jnt{k},'UpperLimitSpecify');
        set_param(jnt{k},'LowerLimitSpecify','off');
        set_param(jnt{k},'UpperLimitSpecify','off');
        fprintf(fid,'  %-46s limits %s/%s -> off\n', jnt{k}, lo, up);
    catch ME
        fprintf(fid,'  %-46s limit FAIL %s\n', jnt{k}, ME.message);
    end
    % 阻尼/摩擦也应当为 0（URDF 里就是 0，这里显式确认）
    try
        d = get_param(jnt{k},'DampingCoefficient');
        fprintf(fid,'  %-46s damping = %s\n', jnt{k}, d);
    catch
    end
end

% ---------------------------------------------------------------- solver
set_param(mdl,'StopTime',num2str(Tend), ...
              'SolverType','Variable-step','Solver','ode23t', ...
              'MaxStep','1e-3','RelTol','1e-8','AbsTol','1e-10', ...
              'SaveState','on','SaveOutput','on','SaveFormat','Dataset', ...
              'ReturnWorkspaceOutputs','on');
fprintf(fid,'\nsolver = %s (%s), MaxStep = %s, RelTol = %s\n', ...
        get_param(mdl,'Solver'), get_param(mdl,'SolverType'), ...
        get_param(mdl,'MaxStep'), get_param(mdl,'RelTol'));

% ---------------------------------------------------------------- run
so = sim(mdl);

% ---- 状态顺序：先 dump 出来，别猜 ----
% ⚠ 实测顺序 **不是**「先所有 q 再所有 w」，而是按块名字母序**交错**：
%     col1..8 = abduct_L.q, abduct_L.w, abduct_R.q, abduct_R.w,
%               hip_L.q,    hip_L.w,    hip_R.q,    hip_R.w
%   第一版曾从 0.5 s 末态数值去倒推顺序，得出 [hip_L,abduct_L,hip_R,abduct_R]
%   ——**是错的**（那个推断建立在数值巧合上）。所以下游必须**按名字**映射。
stNames = {};
if isprop(so,'xout') && ~isempty(so.xout)
    d = so.xout;
    fprintf(fid,'\nxout : %s , numElements = %d\n', class(d), d.numElements);
    for k = 1:d.numElements
        e = d.getElement(k);
        fprintf(fid,'  [%d] name = "%s"\n', k, e.Name);
        stNames{end+1} = char(e.Name); %#ok<AGROW>
    end
end

[t, x] = pull_state(so, fid);
if isempty(x)
    fprintf(fid,'\nFAIL: 拿不到状态向量\nDUMP_MULTIDOF_TRAJ_FAIL\n');
    info = struct('ok',false); return
end
fprintf(fid,'\nraw: %d 点, %d 状态, t = [%.6f .. %.6f]\n', ...
        size(x,1), size(x,2), t(1), t(end));

% ---- 状态短名：sm_exo_multidof.abduct_L.Rz.q  ->  abduct_L.q ----
short = cell(1,size(x,2));
for k = 1:size(x,2)
    if k <= numel(stNames)
        p = strsplit(stNames{k},'.');
        if numel(p) >= 4
            short{k} = sprintf('%s.%s', p{end-2}, p{end});
        else
            short{k} = stNames{k};
        end
    else
        short{k} = sprintf('x%d', k);
    end
end
fprintf(fid,'\n状态顺序（= CSV 列序；请按名字映射，勿假设）:\n');
for k = 1:numel(short)
    fprintf(fid,'  col %d = %s\n', k, short{k});
end

% ---- 直接导出**原始**时间点 ----
% 不做重采样：Simscape 是变步长（ode23t），点只在需要的地方加密；
% 插到均匀网格只会引入额外误差。解析侧自己积分后插到这些 t 上再比。
fid2 = fopen(csv,'w');
fprintf(fid2,'%s\n', strjoin([{'t'} short], ','));
for k = 1:size(x,1)
    fprintf(fid2,'%.12e', t(k));
    fprintf(fid2,',%.12e', x(k,:));
    fprintf(fid2,'\n');
end
fclose(fid2);
fprintf(fid,'\n导出原始 %d 点 -> %s\n', size(x,1), csv);

fprintf(fid,'\n各状态范围:\n');
for k = 1:size(x,2)
    fprintf(fid,'  %-14s min %+12.6f   max %+12.6f   range %12.6f\n', ...
            short{k}, min(x(:,k)), max(x(:,k)), max(x(:,k))-min(x(:,k)));
end
fprintf(fid,'\nDUMP_MULTIDOF_TRAJ_OK\n');
info = struct('ok',true,'csv',csv,'log',logf,'n',size(x,1),'nstate',size(x,2));
end

% ---------------------------------------------------------------- helpers
function [t,x] = pull_state(so, fid)
t = []; x = [];
for nm = {'xout','yout','logsout'}
    try, v = so.get(nm{1}); catch, continue, end
    if isempty(v), continue, end
    fprintf(fid,'  candidate %s : %s\n', nm{1}, class(v));
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
elseif isa(v,'Simulink.SimulationData.Dataset') || ...
       isa(v,'Simulink.SimulationData.DatasetRef')
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
