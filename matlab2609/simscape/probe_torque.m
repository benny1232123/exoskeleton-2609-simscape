function info = probe_torque()
%PROBE_TORQUE  决定性实验：驱动力矩到底进没进关节？
%
% 背景：compare_simscape_vs_analytical 里 scenario 1（T=0）已完美通过
% （rms/range ~2.6e-06），但 scenario 2 的 Simscape 轨迹与 scenario 1
% **逐位相同**（max|q| = 1.8230 vs 1.8230），而解析值不同（2.1832）。
% 这说明 T_L_data / T_R_data 的改动没有生效。
%
% 本探针做两件互不影响的事：
%   1) dump 端口连接关系 + From Workspace 参数 + 关节 actuation 参数
%   2) 同模型跑 T=0 与 T=+1 N*m，比较 qL 轨迹 —— 若完全相同则力矩没进去

here = fileparts(fileparts(mfilename('fullpath')));
simd = fullfile(here,'simscape');
mdl  = 'sm_exo_real_harness';
logf = fullfile(simd,'probe_torque_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== probe_torque ==\n%s\n\n', datestr(now));

if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(fullfile(simd,[mdl '.slx']));

% ---------- 1. hip_L / hip_R 的端口连接关系 ----------
for jn = {'hip_L','hip_R'}
    fprintf(fid,'-- %s PortConnectivity --\n', jn{1});
    pc = get_param([mdl '/' jn{1}],'PortConnectivity');
    fprintf(fid,'   (fields: %s)\n', strjoin(fieldnames(pc)', ', '));
    for i = 1:numel(pc)
        srcs = '';
        if ~isempty(pc(i).SrcBlock)
            for k = 1:numel(pc(i).SrcBlock)
                if pc(i).SrcBlock(k) > 0
                    srcs = [srcs sprintf('%s(port %d) ', ...
                        getfullname(pc(i).SrcBlock(k)), pc(i).SrcPort(k))]; %#ok<AGROW>
                end
            end
        end
        dsts = '';
        if ~isempty(pc(i).DstBlock)
            for k = 1:numel(pc(i).DstBlock)
                if pc(i).DstBlock(k) > 0
                    dsts = [dsts sprintf('-> %s(port %d) ', ...
                        getfullname(pc(i).DstBlock(k)), pc(i).DstPort(k))]; %#ok<AGROW>
                end
            end
        end
        pv = '';                          % R2024b 的 PortConnectivity 未必有 Port 字段
        try, pv = num2str(pc(i).Port); catch, pv = '<n/a>'; end
        fprintf(fid,'  [%d] Type=%-12s Port=%-14s src{%s} dst{%s}\n', ...
            i, pc(i).Type, pv, srcs, dsts);
    end
    fprintf(fid,'\n');
end

% ---------- 2. From Workspace 源块参数 ----------
fprintf(fid,'-- From Workspace 参数 --\n');
for b = {'T_L_src','T_R_src'}
    for p = {'VariableName','SampleTime','Interpolate','FormOutputAfterFinalDataValue', ...
             'ZeroCross','OutputAfterFinalValue','MaxDataPoints'}
        try
            fprintf(fid,'  %-10s . %-28s = %s\n', b{1}, p{1}, ...
                num2str(get_param([mdl '/' b{1}], p{1})));
        catch ME
            fprintf(fid,'  %-10s . %-28s : %s\n', b{1}, p{1}, ME.message);
        end
    end
end
fprintf(fid,'\n');

% ---------- 3. 关节 actuation / sensing 参数 ----------
fprintf(fid,'-- hip_L 关节参数 --\n');
for p = {'TorqueActuationMode','MotionActuationMode','SensePosition','SenseVelocity', ...
         'LowerLimitSpecify','UpperLimitSpecify','JointAxis', ...
         'DampingCoefficient','FrictionCoefficient'}
    try
        fprintf(fid,'  %-24s = %s\n', p{1}, num2str(get_param([mdl '/hip_L'], p{1})));
    catch ME
        fprintf(fid,'  %-24s : %s\n', p{1}, ME.message);
    end
end
fprintf(fid,'\n');

% ---------- 4. 决定性对比 ----------
tt = (0:1e-4:0.6).';
mQ = zeros(2,2);
qLtrace = cell(1,2);
for k = 1:2
    if k == 1
        TL = zeros(size(tt));  tag = 'T_L = 0        (baseline)';
    else
        TL = 1.0*ones(size(tt)); tag = 'T_L = +1.0 N*m  (constant)';
    end
    assignin('base','T_L_data', timeseries(TL, tt));
    assignin('base','T_R_data', timeseries(zeros(size(tt)), tt));
    so = sim(mdl,'StopTime','0.6');
    qL = getts(so,'qL_log');  qR = getts(so,'qR_log');
    mQ(k,:) = [max(abs(qL.Data)) max(abs(qR.Data))];
    qLtrace{k} = qL;
    fprintf(fid,'run %d : %-30s max|qL| = %.9e   max|qR| = %.9e\n', k, tag, mQ(k,1), mQ(k,2));
end

d = mQ(2,1) - mQ(1,1);
fprintf(fid,'\ndelta max|qL|  (T=1 vs T=0) = %+.9e\n', d);

% 端点值也要看（避免 max 掩盖）
fprintf(fid,'qL(0.6 s): T=0 -> %.9e ,  T=1 -> %.9e\n', ...
    qLtrace{1}.Data(end), qLtrace{2}.Data(end));

if abs(d) < 1e-9
    fprintf(fid,'\nVERDICT: TORQUE_NOT_APPLIED  <-- 力矩没进到关节\n');
else
    fprintf(fid,'\nVERDICT: TORQUE_APPLIED  (delta = %.3e)\n', d);
end

fprintf(fid,'\nPROBE_TORQUE_DONE\n');
info = struct('delta',d,'mQ',mQ,'log',logf);
end

function ts = getts(so, name)
ts = [];
try
    if ~isempty(so) && isprop(so,name) && ~isempty(so.(name))
        ts = so.(name);
    end
catch
end
if isempty(ts) && evalin('base',['exist(''' name ''',''var'')'])
    ts = evalin('base',name);
end
if isempty(ts)
    error('probe_torque:noLog','logged signal %s not found', name);
end
if isa(ts,'timeseries'), return, end
if isstruct(ts) && isfield(ts,'time') && isfield(ts,'signals')
    ts = timeseries(ts.signals.values, ts.time);
end
end
