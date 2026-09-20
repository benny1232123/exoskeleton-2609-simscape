function info = smlink_verify(varargin)
%SMLINK_VERIFY  验收 Simscape Multibody Link 插件是否**真的装全了**。
%
%   info = smlink_verify()        快速版（默认）
%   info = smlink_verify(true)    额外扫 HKCR\CLSID 找 DLL 的 COM 注册（慢，约 10–30 s），
%                                 用来区分「regsvr32 压根没跑」与「跑了但 SW 项没写」
%
%   在跑完 installaddon 之后立刻跑这个。它逐项核对 installaddon 到底落盘了
%   什么、有没有落对位置，比「MATLAB 没报错」可信得多。
%
%   为什么需要它：installaddon.m 的最后一步会调用 Java 的
%   com.mathworks.install.command.doc.BuildSharedDocCommand 去重建文档索引。
%   这一步在部分环境下会抛异常 —— 但那时 **解压和加 path 其实已经完成了**。
%   所以「installaddon 报错」!=「安装失败」，必须回头核对文件。
%
%   期望落盘物（源与 zip 内条目一致，matlabroot = E:\2024b-matlab）
%     <matlabroot>\bin\win64\cl_sldwks2sm.dll                       559400 B  ★真载荷
%     <matlabroot>\toolbox\physmod\smlink\smlink\smlink_linksw.m       686 B  ★注册命令
%     <matlabroot>\toolbox\physmod\smlink\smlink\smlink_unlinksw.m     738 B
%     <matlabroot>\toolbox\physmod\smlink\smlink\smlink_linkinv.m      727 B
%     <matlabroot>\toolbox\physmod\smlink\smlink\smlink_unlinkinv.m    788 B
%     <matlabroot>\toolbox\local\path\physmod_smlink.phl                71 B  ★path 清单
%     <matlabroot>\resources\physmod\en\smlink\swaddin.xml             236 B
%
%   判定
%     SMLINK_INSTALL_OK            文件齐 + 在 path 上 → 可以去跑 smlink_linksw
%     SMLINK_INSTALL_INCOMPLETE    缺件，日志列出补法
%     SMLINK_INSTALL_PARTIAL_DOC   文件齐但文档索引那步没过（不影响导出，可忽略）
%
%   本脚本**只读**：不跑 regsvr32，不改 path，不写注册表。
%   （它只调用 reg query 读注册表。）

doDeep = ~isempty(varargin) && logical(varargin{1});

mr   = matlabroot;
here = fileparts(fileparts(mfilename('fullpath')));      % .../matlab2609
simd = fullfile(here,'simscape');
logf = fullfile(simd,'smlink_verify_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== smlink_verify ==\n');
fprintf(fid,'time       : %s\n', datestr(now));
fprintf(fid,'matlabroot : %s\n\n', mr);

% ---------------------------------------------------- 1. 四个 .m 命令函数
swdir = fullfile(mr,'toolbox','physmod','smlink','smlink');
fns = { 'smlink_linksw','smlink_unlinksw','smlink_linkinv','smlink_unlinkinv' };
fnsOk = true;
for i = 1:numel(fns)
    fp = fullfile(swdir, [fns{i} '.m']);
    hasFile = exist(fp,'file') == 2;
    wh = which(fns{i});
    if hasFile
        fprintf(fid,'[1] %-18s : 文件 OK   %s\n', fns{i}, fp);
    else
        fprintf(fid,'[1] %-18s : 文件 MISSING  %s\n', fns{i}, fp);
        fnsOk = false;
    end
    if isempty(wh)
        fprintf(fid,'      which -> <空>  ← 不在 path 上\n');
        fnsOk = false;
    else
        fprintf(fid,'      which -> %s\n', wh);
    end
end

% path 里到底有没有那个目录（physmod_smlink.phl 只加这一个）
onPath = false;
try
    pth = strsplit(path, pathsep);
    onPath = any(strcmpi(pth, swdir));
catch
end
fprintf(fid,'\n[2] path 含 %s : %s\n', swdir, yn(onPath));

% ---------------------------------------------------- 3. 二进制真载荷
dll = fullfile(mr,'bin','win64','cl_sldwks2sm.dll');
dllOk = exist(dll,'file') == 2;
if dllOk
    d = dir(dll);
    fprintf(fid,'[3] cl_sldwks2sm.dll : OK  %.0f B  %s\n', d.bytes, dll);
    if d.bytes ~= 559400
        fprintf(fid,'      ★ 期望 559400 B，实际 %.0f B —— 大小不符，确认下版本\n', d.bytes);
    end
else
    fprintf(fid,'[3] cl_sldwks2sm.dll : MISSING  %s\n', dll);
    fprintf(fid,'      没有它 smlink_linksw 的 regsvr32 必然失败\n');
end

% ---------------------------------------------------- 4. path 清单 + 消息表
phl = fullfile(mr,'toolbox','local','path','physmod_smlink.phl');
xml = fullfile(mr,'resources','physmod','en','smlink','swaddin.xml');
phlOk = exist(phl,'file') == 2;
xmlOk = exist(xml,'file') == 2;
fprintf(fid,'[4] physmod_smlink.phl : %s  %s\n', yn(phlOk), phl);
fprintf(fid,'[5] swaddin.xml        : %s  %s\n', yn(xmlOk), xml);
fprintf(fid,'      ^ 注意：swaddin.xml 只是 i18n 消息表（rsccat，仅一条 WindowsOnly\n');
fprintf(fid,'        文案），**不是** SolidWorks 插件清单。真正的注册靠 DLL 里的\n');
fprintf(fid,'        DllRegisterServer（由 smlink_linksw 调 regsvr32 触发）。\n');

% ---------------------------------------------------- 6. SolidWorks 侧注册表
swHit = false; gr = '';
try
    [st,out] = system('reg query "HKLM\SOFTWARE\SolidWorks\AddIns" /s');
    if st == 0, gr = [gr out]; end
    [st2,out2] = system('reg query "HKCU\SOFTWARE\SolidWorks\AddInsStartup" /s');
    if st2 == 0, gr = [gr out2]; end
catch
end
if isempty(gr)
    fprintf(fid,'\n[6] SW 注册表 : 读不到（没装 SW 或权限不足）\n');
else
    swHit = ~isempty(strfind(gr,'Simscape')) || ~isempty(strfind(gr,'SMLINK')) ... %#ok<STREMP>
            || ~isempty(strfind(gr,'smlink'));                                    %#ok<STREMP>
    fprintf(fid,'\n[6] SW 注册表 : %s\n', yn(swHit));
    if ~swHit
        fprintf(fid,'      还没注册 → 在 MATLAB 里跑 smlink_linksw（会弹 UAC，选"是"）\n');
    end
end

% ---------------------------------------------------- 7. installaddon 在不在
ia = which('installaddon');
fprintf(fid,'[7] installaddon : %s\n', ...
    ternary(~isempty(ia), ia, '<空>（非内置函数，需单独下载，仅安装时用得上）'));

% ------------------------------------------- 8. MATLAB COM 自动化服务器
% regmatlabserver 的产物。⚠ 它写的是 HKCU\Software\Classes（HKCR 的按用户合并视图），
% **不需要提权** → 所以「regmatlabserver 成功」绝不能当成「提权成功」的证据（实测踩过）。
mm = ''; matlabCom = false;
try
    [st,out] = system('reg query "HKCR\MATLAB.Application" /s');
    if st == 0, mm = out; end
catch
end
if isempty(mm)
    fprintf(fid,'[8] MATLAB.Application : MISSING（regmatlabserver 没跑成？）\n');
else
    matlabCom = true;
    cl = regexp(mm,'\{[0-9A-Fa-f\-]{36}\}','match','once');
    if isempty(cl), cl = '<CLSID 未解析到>'; end
    fprintf(fid,'[8] MATLAB.Application : OK  CLSID %s\n', cl);
end

% ------------------------------------------- 9. 当前 MATLAB 是否提权 ★★
% 本脚本最关键的一项诊断。smlink_linksw 内部是
%   system('regsvr32 "<matlabroot>\bin\win64\cl_sldwks2sm.dll"','-echo','-runAsAdmin')
% 它 **不检查返回值** —— UAC 被漏掉/拒绝时就是**静默 no-op**：不报错、不输出。
% 所以「文件都齐但 SW 注册表没项」的头号嫌疑永远是提权，必须单独查。
elevated = false; elevKnown = false;
try
    stE = system('net session >nul 2>&1');
    elevated = (stE == 0); elevKnown = true;
catch
end
if ~elevKnown
    fprintf(fid,'[9] MATLAB 提权 : 无法判定\n');
elseif elevated
    fprintf(fid,'[9] MATLAB 提权 : OK（管理员）—— smlink_linksw 可直接写 HKLM\n');
else
    fprintf(fid,'[9] MATLAB 提权 : NO（非管理员）★\n');
    fprintf(fid,'      → smlink_linksw 的 -runAsAdmin 可能被静默忽略（UAC 漏掉时无任何报错）\n');
    fprintf(fid,'      稳妥做法：管理员 cmd 直接跑 regsvr32 "%s"\n', ...
        fullfile(mr,'bin','win64','cl_sldwks2sm.dll'));
end

% ------------------------------------------- 10. DLL 的 COM 注册（深查，慢）
% 用来区分两种「未注册」：
%   dllCom = 0 → regsvr32 **压根没执行**（UAC 被漏掉 / 命令没跑）
%   dllCom = 1 → regsvr32 跑了、COM 注册成功，但 HKLM\SOFTWARE\SolidWorks\AddIns 没写
dllCom = [];
if doDeep && dllOk
    fprintf(fid,'[10] 深查 HKCR\\CLSID 找 cl_sldwks2sm ...（约 10–30 s）\n');
    try
        [~,out] = system('reg query "HKCR\CLSID" /s /f cl_sldwks2sm /d');
        dllCom = ~isempty(strfind(out,'cl_sldwks2sm')); %#ok<STREMP>
    catch
        dllCom = [];
    end
    fprintf(fid,'[10] DLL 的 COM 注册 : %s\n', yn(dllCom));
end

% ---------------------------------------------------- 判定
filesOk = fnsOk && onPath && dllOk && phlOk && xmlOk;
if filesOk && swHit
    v = 'SMLINK_INSTALL_OK';
elseif filesOk && ~swHit && ~isempty(dllCom) && ~dllCom
    v = 'SMLINK_REG_NEVER_RAN';
elseif filesOk && ~swHit
    v = 'SMLINK_FILES_OK_SW_NOT_REGISTERED';
else
    v = 'SMLINK_INSTALL_INCOMPLETE';
end

fprintf(fid,'\n-- 判定 --\n  %s\n', v);
if strcmp(v,'SMLINK_INSTALL_OK')
    fprintf(fid,'\n-- 下一步 --\n');
    fprintf(fid,'  1) SolidWorks: 工具 → 插件 → 勾选 "Simscape Multibody Link"（两个框都勾）\n');
    fprintf(fid,'  2) 重开 SolidWorks，打开装配体：工具 → Simscape Multibody Link → Export → Simscape Multibody\n');
    fprintf(fid,'  3) MATLAB: xml_preflight   → 期望 XML_PREFLIGHT_READY\n');
    fprintf(fid,'             r = smimport_from_xml(''<导出目录>\\<模型名>.xml'')  → 总质量 ≈ 3.329496 kg\n');
    fprintf(fid,'             （基准真源 = solidworks/mass_props_example_from_stp.csv，密度表一改此数即变）\n');
elseif strcmp(v,'SMLINK_REG_NEVER_RAN')
    fprintf(fid,'\n-- 诊断 --\n');
    fprintf(fid,'  HKCR\\CLSID 里连 DLL 的 COM 注册都没有 → regsvr32 **从未成功执行**。\n');
    fprintf(fid,'  文件都齐了，所以问题不在安装，在**提权**：\n');
    fprintf(fid,'  smlink_linksw 用 -runAsAdmin 弹 UAC，被漏掉时是静默 no-op（不报错）。\n');
    fprintf(fid,'\n-- 修法（最确定，跳过 MATLAB）--\n');
    fprintf(fid,'  1) 开始菜单搜 cmd → 右键 → 以管理员身份运行\n');
    fprintf(fid,'  2) regsvr32 "%s"\n', fullfile(mr,'bin','win64','cl_sldwks2sm.dll'));
    fprintf(fid,'  3) ★ 必须看到弹窗 "DllRegisterServer 在 ... 中成功"，没弹窗就是没成\n');
    fprintf(fid,'  4) 回来重跑本脚本，应变成 SMLINK_INSTALL_OK\n');
elseif strcmp(v,'SMLINK_FILES_OK_SW_NOT_REGISTERED')
    fprintf(fid,'\n-- 下一步 --\n');
    fprintf(fid,'  文件都齐了，只差注册。两种走法：\n');
    if ~elevated
        fprintf(fid,'  ⚠ 当前 MATLAB **不是管理员** → 优先走 (B)\n');
    end
    fprintf(fid,'  (A) MATLAB 里跑 smlink_linksw —— 会弹 UAC，**必须点"是"**（被窗口盖住是常见坑）\n');
    fprintf(fid,'  (B) 管理员 cmd：regsvr32 "%s"\n', fullfile(mr,'bin','win64','cl_sldwks2sm.dll'));
else
    fprintf(fid,'\n-- 补法（非官方兜底）--\n');
    fprintf(fid,'  installaddon 若中途失败，可直接把 zip 解到 matlabroot（E:\\ 可写）：\n');
    fprintf(fid,'    unzip(''<下载目录>\\smlink-r2024b-win64.zip'', matlabroot);\n');
    fprintf(fid,'    addpath(fullfile(matlabroot,''toolbox'',''physmod'',''smlink'',''smlink''));\n');
    fprintf(fid,'    savepath;\n');
    fprintf(fid,'    which smlink_linksw   %% 应指向 <matlabroot>\\toolbox\\physmod\\smlink\\smlink\\\n');
end

info = struct('verdict',v,'fnsOk',fnsOk,'onPath',onPath,'dllOk',dllOk, ...
              'phlOk',phlOk,'xmlOk',xmlOk,'swRegistered',swHit, ...
              'matlabCom',matlabCom,'elevated',elevated,'dllCom',dllCom, ...
              'matlabroot',mr,'log',logf);

fprintf('\n===== smlink_verify =====\n');
fprintf('插件命令文件   : %s\n', yn(fnsOk));
fprintf('在 path 上     : %s\n', yn(onPath));
fprintf('cl_sldwks2sm.dll : %s\n', yn(dllOk));
fprintf('physmod_smlink.phl : %s\n', yn(phlOk));
fprintf('MATLAB.Application : %s\n', yn(matlabCom));
if elevKnown, fprintf('MATLAB 提权    : %s\n', yn(elevated)); end
if ~isempty(dllCom), fprintf('DLL 的 COM 注册 : %s\n', yn(dllCom)); end
fprintf('SW 已注册      : %s\n', yn(swHit));
fprintf('VERDICT        : %s\n', v);
fprintf('日志 -> %s\n', logf);
fprintf('===== smlink_verify 结束 =====\n\n');
end

function s = yn(tf)
if tf, s = 'OK'; else, s = 'NO'; end
end

function s = ternary(tf,a,b)
if tf, s = a; else, s = b; end
end
