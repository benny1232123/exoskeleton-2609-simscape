function info = xml_preflight()
%XML_PREFLIGHT  跑 Simscape Multibody Link（XML 路径）之前的环境体检。
%
%   info = xml_preflight()
%
%   不需要插件已安装也能跑 —— 它存在的意义就是：在你动手装插件之前，
%   把「还缺什么、zip 该叫什么名字、下载页在哪」一次列清楚。
%
%   结果同时写到
%     matlab2609/simscape/xml_preflight_log.txt
%
%   检查项
%     1. MATLAB / Simscape Multibody 是否可用（smimport 在不在 path 上）
%     2. Simscape Multibody Link 插件是否已装（smlink_linksw 能否解析到）
%     3. SolidWorks 是否已注册该插件（读注册表 HKLM\SOFTWARE\SolidWorks\AddIns）
%     4. 中英路径：ASCII junction 是否在（MATLAB CLI 在中文路径下会出问题）
%     5. 插件安装包 smlink-<release>-win64.zip + installaddon.m 是否已在本地
%     6. installaddon 能否在 path 上解析到（它是下载页给的脚本，不是内置函数）
%
%   判定
%     XML_PREFLIGHT_READY    可以直接去 SolidWorks 导出了
%     XML_PREFLIGHT_BLOCKED  还缺件，日志末尾列出待办
%
%   本脚本**只读**：不调用 smlink_linksw，不改注册表，不启动 SolidWorks。

here = fileparts(fileparts(mfilename('fullpath')));      % .../matlab2609
simd = fullfile(here,'simscape');
logf = fullfile(simd,'xml_preflight_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== xml_preflight ==\n');
fprintf(fid,'time       : %s\n', datestr(now));
fprintf(fid,'matlabroot : %s\n', matlabroot);

rel = '';
try
    v = ver('matlab');
    % 注意（实测）：R2024b 的 ver('matlab').Release 返回的是 '(R2024b)'
    % —— **自带圆括号**。不去掉就会拼出 'smlink-(r2024b)-win64.zip' 这种
    % 不存在的文件名（本文件第一版就踩了）。统一剥掉非字母数字字符。
    rel = regexprep(v.Release,'[^A-Za-z0-9]','');
    fprintf(fid,'release    : %s  (version %s)\n', rel, v.Version);
catch
    fprintf(fid,'release    : <query failed>\n');
end
fprintf(fid,'\n');

todo = {};

% ---------------------------------------------------------------- 1. smimport
p = which('smimport');
if isempty(p)
    fprintf(fid,'[1] Simscape Multibody : MISSING  (smimport 不在 path 上)\n');
    todo{end+1} = '安装 Simscape Multibody 工具箱 —— XML 与 URDF 两条路都依赖它'; %#ok<AGROW>
else
    vs = '';
    try
        s = ver('sm');
        if ~isempty(s), vs = ['  ' s(1).Version]; end
    catch
    end
    fprintf(fid,'[1] Simscape Multibody : OK%s\n', vs);
    fprintf(fid,'      smimport -> %s\n', p);
end

% ------------------------------------------------- 2. Simscape Multibody Link
lnk = which('smlink_linksw');
unl = which('smlink_unlinksw');
plugin_fns = false;
if ~isempty(lnk) && ~strcmp(lnk,'built-in') && ~strcmp(lnk,'variable')
    plugin_fns = true;
elseif ~isempty(lnk) && strcmp(lnk,'built-in')
    plugin_fns = true;
end
if plugin_fns
    fprintf(fid,'[2] Multibody Link 插件 : OK  (smlink_linksw -> %s)\n', lnk);
else
    fprintf(fid,'[2] Multibody Link 插件 : MISSING  (找不到 smlink_linksw)\n');
    z = sprintf('smlink-%s-win64.zip', lower(rel));
    fprintf(fid,'      需要下载并安装 : %s\n', z);
    fprintf(fid,'      下载页         : https://www.mathworks.com/campaigns/offerings/download_smlink_confirmation.html\n');
    fprintf(fid,'      ★ 下载页会同时给 installaddon.m —— 两个文件都要下，不要解压 zip\n');
    todo{end+1} = sprintf(['下载 %s + installaddon.m（都不要解压/改名）→ 以**管理员身份**运行 MATLAB → ' ...
        'addpath(下载目录); installaddon(''<下载目录>\\%s'') → regmatlabserver → smlink_linksw'], z, z); %#ok<AGROW>
end

% ------------------------------------------ 3. SolidWorks 注册表里有没有它
sw_reg = false; sw_raw = '';
cmds = { 'reg query "HKLM\SOFTWARE\SolidWorks\AddIns" /s'
         'reg query "HKCU\SOFTWARE\SolidWorks\AddIns" /s'
         'reg query "HKCU\SOFTWARE\SolidWorks\AddInsStartup" /s' };
for i = 1:numel(cmds)
    try
        [st,out] = system(cmds{i});
        if st == 0
            sw_raw = [sw_raw out]; %#ok<AGROW>
        end
    catch
    end
end
if isempty(sw_raw)
    fprintf(fid,'[3] SolidWorks 插件注册 : 读不到注册表（可能没装 SolidWorks）\n');
    todo{end+1} = '确认已安装 SolidWorks，并能打开 .SLDASM 装配体'; %#ok<AGROW>
else
    has_sw  = ~isempty(strfind(sw_raw,'SOLIDWORKS')) || ~isempty(strfind(sw_raw,'SolidWorks')); %#ok<STREMP>
    has_lnk = ~isempty(strfind(sw_raw,'Simscape')) || ~isempty(strfind(sw_raw,'smlink')) ... %#ok<STREMP>
              || ~isempty(strfind(sw_raw,'SMLINK')); %#ok<STREMP>
    sw_reg = has_lnk;
    if ~has_sw
        fprintf(fid,'[3] SolidWorks 插件注册 : 没有 SolidWorks 的 AddIns 项\n');
        todo{end+1} = '确认本机装了 SolidWorks'; %#ok<AGROW>
    elseif has_lnk
        fprintf(fid,'[3] SolidWorks 插件注册 : OK  (注册表里已有 Simscape/smlink 项)\n');
    else
        fprintf(fid,'[3] SolidWorks 插件注册 : 未注册\n');
        fprintf(fid,'      下一步：MATLAB 里跑 smlink_linksw（它内部执行 regsvr32 cl_sldwks2sm.dll，\n');
        fprintf(fid,'      会弹 UAC；成功后 DLL 的 DllRegisterServer 才写入 SW 的 AddIns 项）。\n');
        fprintf(fid,'      然后在 SolidWorks 的 工具 → 插件 里勾选 "Simscape Multibody Link"\n');
        todo{end+1} = '跑 smlink_linksw（允许 UAC），并在 SolidWorks 工具→插件 里勾选 Simscape Multibody Link'; %#ok<AGROW>
    end
end

% ------------------------------------------------------- 4. ASCII junction
jx = 'C:\Users\29408\exo_work';
if exist(jx,'dir') == 7
    fprintf(fid,'[4] ASCII junction     : OK  %s\n', jx);
else
    fprintf(fid,'[4] ASCII junction     : MISSING  %s\n', jx);
    fprintf(fid,'      建法（管理员 cmd）：mklink /J "%s" "%s"\n', jx, here(1:end-10));
    todo{end+1} = sprintf('建 ASCII junction：mklink /J "%s" "<项目根>"', jx); %#ok<AGROW>
end

% --------------------------------------------------------- 5. 安装包在不在
% 注意（实测）：下载默认落在 Edge 的下载目录 E:\Edge_Download，不是
% %USERPROFILE%\Downloads —— 早期版本漏了这个目录，会误报「本地没有」。
zipname  = sprintf('smlink-%s-win64.zip', lower(rel));
searchDirs = { 'C:\smlink_install', 'E:\Edge_Download', ...
               'D:\Edge_Download', 'E:\Downloads', ...
               fullfile(getenv('USERPROFILE'),'Downloads'), ...
               'C:\Users\29408\Downloads', ...
               'C:\Users\29408\Desktop', here, simd };
found = '';
for i = 1:numel(searchDirs)
    if isempty(searchDirs{i}) || exist(searchDirs{i},'dir') ~= 7, continue; end
    d = dir(fullfile(searchDirs{i},'smlink*.zip'));
    for k = 1:numel(d)
        found = fullfile(searchDirs{i}, d(k).name);
        fprintf(fid,'[5] 安装包             : 找到 %s  (%.1f MB)\n', ...
            found, d(k).bytes/1048576);
    end
end
if isempty(found)
    fprintf(fid,'[5] 安装包             : 本地没有 %s\n', zipname);
end

% --------------------------------------------------- 6. installaddon 在不在
% ★ installaddon 不是 MATLAB 内置函数（R2024b 实测 which/exist 均为空，
%   matlabroot 下零命中）。它是下载页单独提供的脚本，必须自己下载。
%   另一个通用 API matlab.addons.install 只吃 .mltbx，源码里硬编码
%   `if ~strcmpi(ext,'.mltbx'), error(...)`，所以对 .zip 无效。
ia = which('installaddon');
if ~isempty(ia)
    fprintf(fid,'[6] installaddon       : OK  %s\n', ia);
else
    fprintf(fid,'[6] installaddon       : MISSING  (它不是内置函数，需从下载页单独下 .m)\n');
    todo{end+1} = '从同一下载页下载 installaddon.m，与 zip 放同一目录（addpath 后用）'; %#ok<AGROW>
end

% ------------------------------------------------------------- 判定 / 待办
ready = ~isempty(p) && plugin_fns && sw_reg && exist(jx,'dir') == 7;

fprintf(fid,'\n-- 待办 --\n');
if isempty(todo)
    fprintf(fid,'  （无）\n');
else
    for i = 1:numel(todo)
        fprintf(fid,'  %d) %s\n', i, todo{i});
    end
end

fprintf(fid,'\n-- 下一步（插件装好后）--\n');
fprintf(fid,'  MATLAB    : smlink_verify                       %% 先验收安装是否完整\n');
fprintf(fid,'  SolidWorks: Tools > Simscape Multibody Link > Export > Simscape Multibody\n');
fprintf(fid,'  MATLAB    : r = smimport_from_xml(''<导出目录>\\<模型名>.xml'')\n');

if ready
    fprintf(fid,'\nVERDICT: XML_PREFLIGHT_READY\n');
else
    fprintf(fid,'\nVERDICT: XML_PREFLIGHT_BLOCKED  (%d 项待办)\n', numel(todo));
end

info = struct('ready',ready,'release',rel,'smimport',p,'pluginFns',plugin_fns, ...
              'swRegistered',sw_reg,'junction',jx,'zipName',zipname, ...
              'zipFound',found,'installaddon',ia,'todo',{todo},'log',logf);

fprintf('\n===== xml_preflight =====\n');
fprintf('release        : %s\n', rel);
fprintf('smimport       : %s\n', char(p));
fprintf('插件函数       : %s\n', ternary(plugin_fns,'OK','MISSING'));
fprintf('SW 注册        : %s\n', ternary(sw_reg,'OK','未注册'));
fprintf('ASCII junction : %s\n', ternary(exist(jx,'dir')==7,'OK','MISSING'));
fprintf('需要的安装包   : %s\n', zipname);
fprintf('installaddon   : %s\n', ternary(~isempty(ia),'OK','MISSING'));
for i = 1:numel(todo)
    fprintf('  待办 %d) %s\n', i, todo{i});
end
fprintf('日志 -> %s\n', logf);
fprintf('===== xml_preflight 结束 =====\n\n');
end

function s = ternary(tf,a,b)
if tf, s = a; else, s = b; end
end
