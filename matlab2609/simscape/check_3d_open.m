function info = check_3d_open()
%CHECK_3D_OPEN  「在 MATLAB 里怎么看 3D」的口令体检（无 GUI，秒级）
%
%   检查三件事：
%     1) SimMechanicsOpenEditorOnUpdate —— 决定 3D 视窗**会不会自己弹**。
%        官方标签是 "Open Mechanics Explorer on model update or simulation"，
%        即：**模型更新(Ctrl+D) 或 仿真(Run) 时**才创建；
%        `open_system` 光打开**不会**创建 —— 这是最常见的「只弹出框图」原因。
%     2) File Solid 块引用的 STL 路径是不是**相对路径** —— 决定「必须 cd 到 simscape」
%        这条硬要求，以及引用的文件是否真的存在。
%        （找不到 = 编译期硬错误，压根跑不起来，不是"缺外形"。）
%     3) 模型里到底有哪些关节块 / 世界重力 / 求解器
%
%   跑法：
%     cd C:\Users\29408\exo_work\matlab2609\simscape
%     check_3d_open
%
%   见 also MULTIDOF_3D_HOWTO.md

here = fileparts(fileparts(mfilename('fullpath')));   % .../matlab2609
simd = fullfile(here, 'simscape');
logf = fullfile(simd, 'check_3d_open_log.txt');
fid  = fopen(logf, 'w'); c = onCleanup(@() fclose(fid));
fprintf(fid, '== check_3d_open ==\n%s\n', datestr(now));
fprintf(fid, '当前目录 : %s\n\n', pwd);

mdls = {'sm_exo_multidof', 'sm_exo_multidof_locked', 'sm_exo_real'};
okAll = true;

for m = 1:numel(mdls)
    mdl = mdls{m};
    fprintf(fid, '%s\n%s\n', mdl, repmat('-', 1, numel(mdl)));

    if isempty(dir([mdl '.slx']))
        fprintf(fid, '  ✗ %s.slx 不在当前目录 —— 先 cd 到 simscape\n\n', mdl);
        okAll = false;
        continue
    end

    loadedHere = ~isempty(find_system('SearchDepth', 0, 'Name', mdl));
    if ~loadedHere
        load_system(mdl);
    end
    fprintf(fid, '  （本次由脚本 load：%d；跑完不保存关闭）\n', ~loadedHere);

    fprintf(fid, '  SimMechanicsOpenEditorOnUpdate = %s   <-- on 才会在「更新/仿真」时自动弹 3D\n', ...
            get_param(mdl, 'SimMechanicsOpenEditorOnUpdate'));
    fprintf(fid, '  StopTime = %s    Solver = %s / %s\n', ...
            get_param(mdl, 'StopTime'), get_param(mdl, 'Solver'), get_param(mdl, 'SolverType'));

    % ---- File Solid 块（= 管渲染的那些）----
    blks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'Block');
    nfs = 0; nmiss = 0;
    for i = 1:numel(blks)
        try
            mt = get_param(blks{i}, 'MaskType');
        catch
            mt = '';
        end
        if ~strcmp(mt, 'File Solid'), continue, end
        nfs = nfs + 1;
        % ★ 参数名是 ExtGeomFileName，**不是** FileName。
        %   （2026-09-20 探针实测；写成 FileName 会抛「取不到」，然后被误判成
        %     "STL 找不到" —— 那是我自己的脚本 bug，不是模型的问题。）
        try
            fn = get_param(blks{i}, 'ExtGeomFileName');
        catch
            fn = '(取不到 ExtGeomFileName)';
        end
        rel = isempty(regexp(fn, '^([A-Za-z]:|\\\\|/)', 'once'));   % 不是绝对路径
        exists_here = ~isempty(dir(fn));
        if ~exists_here, nmiss = nmiss + 1; end
        fprintf(fid, '    %-34s %-28s 相对=%d 存在=%d\n', ...
                strrep(blks{i}, [mdl '/'], ''), fn, rel, exists_here);
    end
    fprintf(fid, '  File Solid 块 %d 个，其中在**当前目录**找不到文件的 %d 个\n', nfs, nmiss);
    if nmiss > 0
        % 2026-09-20 实测纠正：相对路径找不到是**编译期硬错误**，
        % 模型直接 fail（sm:sli:setup:compile:ErrorMessages + N 条 FileNotExist），
        % 根本到不了渲染 —— 不是"3D 里少个外形"。
        fprintf(fid, '  ✗ 有 %d 个 STL 在当前目录找不到 ⇒ **编译期直接报错**，跑不起来。\n', nmiss);
        fprintf(fid, '     报错形如: [''%s/base/Visual'']: The parameter Geometry/File Name is a\n', mdl);
        fprintf(fid, '               file that does not exist.  -> cd 到 simscape 即可。\n');
        okAll = false;
    end

    % ---- 关节块 ----
    jl = {};
    for i = 1:numel(blks)
        try
            mt = get_param(blks{i}, 'MaskType');
        catch
            mt = '';
        end
        if any(strcmp(mt, {'Revolute Joint', 'Prismatic Joint', 'Weld Joint', ...
                           'Bearing Joint', 'Custom Joint', 'Spherical Joint'}))
            jl{end+1} = sprintf('%s[%s]', get_param(blks{i}, 'Name'), mt); %#ok<AGROW>
        end
    end
    fprintf(fid, '  关节块 %d 个： %s\n', numel(jl), strjoin(sort(jl), '  '));

    % ---- 世界重力 ----
    mech = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                       'regexp', 'on', 'MaskType', 'Mechanism Configuration');
    for i = 1:numel(mech)
        g = '?';
        for p = {'GravityVector', 'Gravity'}
            try
                g = mat2str(get_param(mech{i}, p{1})); break
            catch
            end
        end
        fprintf(fid, '  MechanismConfiguration 重力 = %s\n', g);
    end

    if ~loadedHere
        close_system(mdl, 0);       % 只关自己 load 的，且不保存
    end
    fprintf(fid, '\n');
end

fprintf(fid, 'VERDICT: %s\n', ternary(okAll, 'OPEN3D_READY', 'OPEN3D_NEEDS_FIX'));
fprintf(fid, 'CHECK_3D_%s\n', ternary(okAll, 'OK', 'FAIL'));

if okAll
    fprintf(fid, '\n--- NEXT（体检过了，接下来怎么看）---\n');
    fprintf(fid, '  1) 确保当前目录 = %s\n', simd);
    fprintf(fid, '  2) open_system(''sm_exo_multidof'')   %% 这一步只会弹**框图**，正常\n');
    fprintf(fid, '  3) 点工具栏「运行」 或按 Ctrl+D        %% ★ 3D 视窗在这一步才弹出来\n');
    fprintf(fid, '  若 3 之后仍无 3D：查是否在别的窗口后面 / 仿真模式是否「普通」/\n');
    fprintf(fid, '  MATLAB 是否带 Java。详见 MULTIDOF_3D_HOWTO.md §2.0。\n');
end

info = struct('ok', okAll, 'log', logf);
end

function s = ternary(tf, a, b), if tf, s = a; else, s = b; end, end
