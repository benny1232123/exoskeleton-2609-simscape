function probe_params()
%PROBE_PARAMS  查 smimport 生成的 link 子系统里，质量/质心/惯量到底存在哪个块哪个参数上。
% 背景：view_exo_real 想打印 CoM 偏移，但读 Inertia.CenterOfMass 和
% InertiaOriginTransform.Translation 都得到 [0 0 0]，与 URDF 里的非零偏移不符。

here = fileparts(fileparts(mfilename('fullpath')));
simd = fullfile(here,'simscape');
logf = fullfile(simd,'probe_params_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== probe_params ==\n%s\n\n', datestr(now));

mdl = 'sm_exo_real';
if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(fullfile(simd,[mdl '.slx']));

targets = { [mdl '/leg_L/Inertia'], ...
            [mdl '/leg_L/InertiaOriginTransform'], ...
            [mdl '/leg_L/ReferenceFrame'], ...
            [mdl '/base/Inertia'], ...
            [mdl '/hip_L'] };

for k = 1:numel(targets)
    b = targets{k};
    fprintf(fid,'==== %s ====\n', b);
    if isempty(find_system('SearchDepth',Inf,'Name',get_param(b,'Name')))
        fprintf(fid,'  <block not found>\n\n');
        continue
    end
    try
        dp = get_param(b,'DialogParameters');
        fn = fieldnames(dp);
    catch ME
        fprintf(fid,'  <no DialogParameters: %s>\n\n', ME.message);
        continue
    end
    for j = 1:numel(fn)
        nm = fn{j};
        try
            v = get_param(b, nm);
            if ischar(v)
                vs = v;
            elseif iscell(v)
                vs = strjoin(cellfun(@num2str, v, 'uni', 0), ' | ');
            elseif isnumeric(v)
                vs = mat2str(v);
            else
                vs = class(v);
            end
        catch ME
            vs = ['<err: ' ME.message '>'];
        end
        fprintf(fid,'  %-34s = %s\n', nm, vs);
    end
    fprintf(fid,'\n');
end

fprintf(fid,'PROBE_PARAMS_DONE\n');
fclose(fid);
end
