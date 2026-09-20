function probe_viz_ready()
%PROBE_VIZ_READY  在交付「可视化仿真」之前，先免 GUI 地把三件事钉死：
%   1) 两个模型（sm_exo_real / sm_exo_real_harness）各自的 ExtGeomFileName
%      到底指向哪、文件在不在 —— 决定 Mechanics Explorer 能不能看到外形。
%   2) harness 的输入/记录接线（From Workspace 变量名、To Workspace 变量名）
%      —— 决定脚本要往 base 工作区塞什么、能从 so 里取什么。
%   3) 模型对象上跟 Mechanics Explorer 相关的可写参数名
%      —— 决定用什么 set_param 让 3D 视窗自动弹出来。
% 只读，不 save_system，不改任何参数。

here = fileparts(fileparts(mfilename('fullpath')));
simd = fullfile(here,'simscape');
logf = fullfile(simd,'probe_viz_ready_log.txt');
fid  = fopen(logf,'w');
c    = onCleanup(@() fclose(fid));

fprintf(fid,'== probe_viz_ready ==\n%s\n', datestr(now));
fprintf(fid,'matlabroot: %s\n', matlabroot);
fprintf(fid,'simd: %s\n\n', simd);

% ---------------- 1. 两个模型的 STL 接线 ----------------
for mdl = {'sm_exo_real','sm_exo_real_harness'}
    m = mdl{1};
    slx = fullfile(simd,[m '.slx']);
    fprintf(fid,'---- [%s] ----\n', m);
    if isempty(dir(slx))
        fprintf(fid,'  SLX MISSING: %s\n\n', slx);
        continue
    end
    if bdIsLoaded(m), close_system(m,0); end
    load_system(slx);
    blks = find_system(m,'SearchDepth',Inf,'Type','Block');
    nHit = 0;
    for k = 1:numel(blks)
        b = blks{k};
        try, dp = get_param(b,'DialogParameters'); catch, continue, end
        if ~isstruct(dp), continue, end
        fn = fieldnames(dp);
        for j = 1:numel(fn)
            try, v = get_param(b,fn{j}); catch, continue, end
            if ~ischar(v) || isempty(strfind(lower(v),'.stl')), continue, end
            d = dir(v);
            ex = 'NO';
            if ~isempty(d) && d.bytes>0, ex = 'YES'; end
            fprintf(fid,'  [%d] %s\n        %s = %s\n        exists=%s\n', ...
                nHit+1, b, fn{j}, v, ex);
            nHit = nHit+1;
        end
    end
    fprintf(fid,'  stl-params = %d\n', nHit);

    % 输入 / 输出接线
    fprintf(fid,'  -- From Workspace --\n');
    fw = find_system(m,'SearchDepth',Inf,'BlockType','FromWorkspace');
    for i = 1:numel(fw)
        try, vn = get_param(fw{i},'VariableName'); catch, vn = '?'; end
        fprintf(fid,'    %s   VariableName = %s\n', fw{i}, vn);
    end
    fprintf(fid,'  -- To Workspace --\n');
    tw = find_system(m,'SearchDepth',Inf,'BlockType','ToWorkspace');
    for i = 1:numel(tw)
        try, vn = get_param(tw{i},'VariableName'); catch, vn = '?'; end
        try, sv = get_param(tw{i},'SaveFormat');   catch, sv = '?'; end
        fprintf(fid,'    %s   VariableName = %s   SaveFormat = %s\n', tw{i}, vn, sv);
    end
    try
        fprintf(fid,'  -- 顶层关节块 --\n');
        jn = find_system(m,'SearchDepth',1,'MaskType','Revolute Joint');
        for i = 1:numel(jn)
            fprintf(fid,'    %s\n', jn{i});
            for prm = {'TorqueActuationMode','SensePosition','SenseVelocity', ...
                       'LowerLimitSpecify','UpperLimitSpecify'}
                try
                    fprintf(fid,'        %-20s = %s\n', prm{1}, ...
                        num2str(get_param(jn{i},prm{1})));
                catch
                end
            end
        end
    catch
    end

    % Mechanics Explorer 相关可写参数
    try
        op = get_param(m,'ObjectParameters');
        ks = fieldnames(op);
        hits = ks(~cellfun('isempty', strfind(lower(ks),'mechanic')));
        hits2 = ks(~cellfun('isempty', strfind(lower(ks),'explorer')));
        hits3 = ks(~cellfun('isempty', strfind(lower(ks),'zoom')));
        hits = unique([hits; hits2; hits3]);
        fprintf(fid,'  -- 模型对象上 Mechanics/Explorer 相关参数 --\n');
        for i = 1:numel(hits)
            try
                v = get_param(m,hits{i});
                if ischar(v)
                    fprintf(fid,'    %-42s = %s\n', hits{i}, v);
                else
                    fprintf(fid,'    %-42s = <%s>\n', hits{i}, class(v));
                end
            catch
            end
        end
    catch ME
        fprintf(fid,'  ObjectParameters 查询失败: %s\n', ME.message);
    end
    fprintf(fid,'\n');
    close_system(m,0);
end

% ---------------- 2. 有没有 smwritevideo ----------------
fprintf(fid,'-- 录制/动画 API --\n');
for f = {'smwritevideo','smwritevideo_legacy','mech_ssc_video'}
    fprintf(fid,'  %s : %s\n', f{1}, which(f{1}));
end
fprintf(fid,'\nPROBE_VIZ_READY_DONE\n');
fprintf(fid,'log -> %s\n', logf);
end
