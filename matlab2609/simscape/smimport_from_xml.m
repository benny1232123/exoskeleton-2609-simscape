function info = smimport_from_xml(xmlPath, varargin)
%SMIMPORT_FROM_XML  导入 Simscape Multibody Link 导出的 XML，存盘，并把所有
%                   刚体的「质量 / 质心 / 惯量」dump 出来供对账。
%
% 用法
%   r = smimport_from_xml('D:\exo_xml\exo_master.xml')
%   r = smimport_from_xml(xml,'ModelName','sm_exo_xml')
%   r = smimport_from_xml(xml,'NoSave',true)      % 只 dump，不落 .slx
%
% 为什么必须显式存盘
%   smimport 生成的模型**只在内存里**，-batch 退出即丢 —— 日志里写着 OK，
%   磁盘上什么都没有。
%
% 为什么这个对账才有意义
%   本仓库有两条从**同一份 CAD** 出发、但**上游完全不同**的链路：
%     URDF 路 : 我们自己的 Python STEP 解析（exo2609/geometry.py 查表密度）
%     XML  路 : SolidWorks 自己的内核（Simscape Multibody Link 插件导出）
%   现有 L1/L2/L2' 三条证据**全部同源**于 URDF 路（连所谓"STP 基准"也是
%   同一条链产出的）—— 同源会同错。只有 XML 路的数字才算外部对账。
%   参考值取自 exo_real_report.txt，见本文件末尾 REF 表。
%
% 只自动比对**质量**
%   质量与参考系无关；而两条路的 link frame 语义不同（URDF 路是我们刻意
%   对齐到世界系的，XML 路的参考系由插件按 SolidWorks 坐标系决定），
%   所以质心/惯量只打印、不判 PASS/FAIL，避免拿两套语义硬比。
%
% 判定
%   XML_IMPORT_OK          导入 + 存盘成功，刚体 > 0，关节 > 0
%   XML_IMPORT_NO_JOINTS   有刚体但一个关节都没有 -> 装配体 mates 没配好
%                          （插件是靠**配合关系**生成关节的）
%   XML_IMPORT_NO_BODIES   一个带质量的刚体都没有 -> 几何/材质没导出
%   XML_IMPORT_FAILED      smimport 直接报错

here = fileparts(fileparts(mfilename('fullpath')));      % .../matlab2609
simd = fullfile(here,'simscape');

p = struct('ModelName','sm_exo_xml','NoSave',false);
if mod(numel(varargin),2) ~= 0
    error('smimport_from_xml:args','参数必须成对出现');
end
for i = 1:2:numel(varargin)
    k = varargin{i};
    if ~isfield(p,k), error('smimport_from_xml:args','未知参数 ''%s''', k); end
    p.(k) = varargin{i+1};
end

if nargin < 1 || isempty(xmlPath)
    error('smimport_from_xml:noArg', ...
        '必须给出 XML 路径。用法：smimport_from_xml(''D:\\exo_xml\\exo_master.xml'')');
end
xd = dir(xmlPath);
if isempty(xd)
    error('smimport_from_xml:noXml', ...
        ['XML 不存在：%s\n' ...
         '（先在 SolidWorks 里 Tools > Simscape Multibody Link > Export > Simscape Multibody）'], ...
        xmlPath);
end
[~,~,ext] = fileparts(xmlPath);
if ~strcmpi(ext,'.xml')
    error('smimport_from_xml:badExt', ...
        '期望 .xml（插件导出的 Physical Modeling XML），实际扩展名是 %s', ext);
end

mdl  = p.ModelName;
logf = fullfile(simd,[mdl '_log.txt']);
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== smimport_from_xml ==\n');
fprintf(fid,'time  : %s\n', datestr(now));
fprintf(fid,'xml   : %s  (%.1f KB)\n', xmlPath, xd.bytes/1024);
fprintf(fid,'model : %s\n\n', mdl);

if bdIsLoaded(mdl), close_system(mdl,0); end

% ------------------------------------------------------------- 1. 导入
t0 = tic;
try
    [H, dfn] = smimport(xmlPath,'ModelName',mdl,'ModelSimplification','bringJointsToTop');
catch ME
    fprintf(fid,'smimport : FAILED\n  id  = %s\n  msg = %s\n', ME.identifier, ME.message);
    fprintf(fid,'\nVERDICT: XML_IMPORT_FAILED\n');
    fprintf('\nXML_IMPORT_FAILED  —— 详情见 %s\n', logf);
    info = struct('ok',false,'verdict','XML_IMPORT_FAILED','log',logf);
    rethrow(ME);
end
fprintf(fid,'smimport : OK  (%.1f s, handle=%g)\n', toc(t0), H);
fprintf(fid,'  datafile = "%s"\n', char(string(dfn)));
fprintf(fid,'  （URDF 输入时这里恒为空串；XML 输入时是否非空取决于插件的\n');
fprintf(fid,'    几何/参数导出设置 —— 选了 "MATLAB data file" 才会给出 smiData）\n');

% ------------------------------------------------------------- 2. 存盘
slx = fullfile(simd,[mdl '.slx']);
saved = false;
if ~p.NoSave
    save_system(mdl, slx);
    sd = dir(slx);
    if isempty(sd)
        fprintf(fid,'save_system : FAILED（磁盘上没有 %s）\n', slx);
    else
        saved = true;
        fprintf(fid,'save_system : OK  %s  (%.1f KB)\n', slx, sd.bytes/1024);
    end
else
    fprintf(fid,'save_system : skipped (NoSave=true)\n');
end

% ------------------------------------------------------------- 3. 关节
allb = find_system(mdl,'SearchDepth',Inf,'Type','Block');
jnAll = {};
for k = 1:numel(allb)
    mt = '';  try, mt = get_param(allb{k},'MaskType'); end
    if ~isempty(strfind(lower(mt),'joint'))
        jnAll{end+1} = allb{k}; %#ok<AGROW>
    end
end
tops = find_system(mdl,'SearchDepth',1,'Type','Block');
jnTop = {};
for k = 1:numel(tops)
    mt = '';  try, mt = get_param(tops{k},'MaskType'); end
    if ~isempty(strfind(lower(mt),'joint'))
        jnTop{end+1} = tops{k}; %#ok<AGROW>
    end
end

fprintf(fid,'\n-- 关节 --\n');
fprintf(fid,'  全部块里的关节 = %d   顶层关节 = %d\n', numel(jnAll), numel(jnTop));
fprintf(fid,'  （插件把装配体 mates 自动映射成关节。若这里接近 0 而零件上千，\n');
fprintf(fid,'    说明装配体里没有配合关系 —— 那是 STEP 导入的"哑实体"，不是原生装配体）\n');
for k = 1:numel(jnTop)
    mt = '';  try, mt = get_param(jnTop{k},'MaskType'); end
    fprintf(fid,'    %s   [%s]\n', jnTop{k}, mt);
end

% ------------------------------------------------------------- 4. 刚体 dump
B = dump_bodies(mdl, mdl);
fprintf(fid,'\n-- 刚体 (%d) --\n', numel(B));
if ~isempty(B)
    fprintf(fid,'  CoM 是该刚体自己 link frame 里的偏移（XML 路 frame 语义与 URDF 路\n');
    fprintf(fid,'  不同，只作参考，不参与判定）\n');
    for k = 1:numel(B)
        fprintf(fid,'  %-46s m=%11.6f kg  CoM=[% .6f % .6f % .6f]\n', ...
            B(k).name, B(k).m, B(k).com);
    end
end

% ------------------------------------------------------------- 5. 对账
% ★ 基准真源 = matlab2609/solidworks/mass_props_example_from_stp.csv
%   （由 _make_sw_csv.py 从同一条 STEP 解析链生成，含 5 个刚体的 mass_kg）。
%   以前这里硬编码 exo_real_report.txt 的旧值（leg 0.304833 / 总 3.313983）：
%   密度表一改就全过期，而对账照旧打印"相对偏差"，没人看得出基准本身是旧的。
CSV_REF = fullfile(simd,'..','solidworks','mass_props_example_from_stp.csv');
REF_TOTAL = 3.329496;                       % 回退值（= 改后基准）
REF = { 'leg_L',   0.312589
        'leg_R',   0.312589
        'motor_L', 0.355676
        'motor_R', 0.392793
        'back',    1.955848
        'base',    2.704317 };              % 回退值（= 改后基准）
ref_src = 'HARDCODED FALLBACK';
if ~isempty(dir(CSV_REF))
    try
        T = readtable(CSV_REF, 'TextType', 'string');
        gg = string(T.group);  mm = double(T.mass_kg);
        REF_TOTAL = sum(mm(~isnan(mm)));
        for i = 1:size(REF,1)
            if strcmp(REF{i,1},'base'), continue, end   % CSV 无 base（base 是合并口径）
            v = mm(find(gg == REF{i,1}, 1));
            if ~isempty(v) && ~isnan(v), REF{i,2} = v; end
        end
        ref_src = 'mass_props_example_from_stp.csv';
    catch ME
        ref_src = ['HARDCODED FALLBACK (' ME.message ')'];
    end
end

if ~isempty(B)
    mt = sum([B.m]);
    fprintf(fid,'\n-- 总质量对账 --\n');
    fprintf(fid,'  基准来源       = %s\n', ref_src);
    fprintf(fid,'  XML 路(本模型) = %.6f kg\n', mt);
    fprintf(fid,'  URDF 路(参考)  = %.6f kg   <- 仅当导的是**完整装配体**时才应相等\n', REF_TOTAL);
    fprintf(fid,'  相对偏差       = %.3e\n', abs(mt-REF_TOTAL)/REF_TOTAL);

    [~,ord] = sort([B.m],'descend');
    nTop = min(10,numel(B));
    fprintf(fid,'\n-- 最重的 %d 个刚体 --\n', nTop);
    for k = 1:nTop
        i = ord(k);
        fprintf(fid,'  %-46s %.6f kg\n', B(i).name, B(i).m);
    end
end

if ~isempty(B) && numel(B) <= 12
    fprintf(fid,'\n-- 逐刚体质量对账（刚体数 %d <= 12，可一一对应）--\n', numel(B));
    fprintf(fid,'   匹配方式：取质量最接近的刚体（启发式，仅供定位）\n');
    for i = 1:size(REF,1)
        tgt = REF{i,2};
        [~,ix] = min(abs([B.m]-tgt));
        rel = abs(B(ix).m-tgt)/tgt;
        fprintf(fid,'  %-8s 参考 %10.6f  ->  %-40s %10.6f  rel %.2e %s\n', ...
            REF{i,1}, tgt, B(ix).name, B(ix).m, rel, okstr(rel < 5e-3));
    end
elseif ~isempty(B)
    fprintf(fid,'\n-- 逐刚体质量对账：跳过（刚体数 %d > 12，说明导的是完整装配体）\n', numel(B));
    fprintf(fid,'   这时只有**总质量**可比（%.6f kg）。想做逐组对账，\n', REF_TOTAL);
    fprintf(fid,'   请在 SolidWorks 里另建一个"三体简化装配体"再导出（见 XML_PATH_GUIDE.md Step 2）。\n');
end

% ------------------------------------------------------------- 6. 判定
if isempty(B)
    verdict = 'XML_IMPORT_NO_BODIES';
elseif numel(jnAll) == 0
    verdict = 'XML_IMPORT_NO_JOINTS';
else
    verdict = 'XML_IMPORT_OK';
end

fprintf(fid,'\nVERDICT: %s\n', verdict);
fprintf('\n===== smimport_from_xml =====\n');
fprintf('model   : %s\n', mdl);
fprintf('刚体    : %d\n', numel(B));
fprintf('关节    : %d (顶层 %d)\n', numel(jnAll), numel(jnTop));
if ~isempty(B)
    fprintf('总质量  : %.6f kg\n', sum([B.m]));
end
fprintf('verdict : %s\n', verdict);
fprintf('日志 -> %s\n', logf);
fprintf('===== smimport_from_xml 结束 =====\n\n');

info = struct('ok',strcmp(verdict,'XML_IMPORT_OK'),'verdict',verdict, ...
              'model',mdl,'slx',slx,'saved',saved, ...
              'nBodies',numel(B),'nJoints',numel(jnAll),'bodies',B,'log',logf);
end

% ================================================================ helpers
function B = dump_bodies(mdl, root)
% 扫全模型，把每个「带 Mass 参数的块」当成一个刚体。
% 注意（实测踩过）：质心偏移**不在** Inertia 块自己身上，而在同级的
% InertiaOriginTransform（Rigid Transform）块的 TranslationCartesianOffset 里。
% 另外不要把 CenterOfMass 当来源 —— 它恒为 [0 0 0]。
blks = find_system(root,'SearchDepth',Inf,'Type','Block');
B = struct('name',{},'m',{},'com',{},'I',{},'Ip',{});
for k = 1:numel(blks)
    b = blks{k};
    dn = {};
    try, dn = fieldnames(get_param(b,'DialogParameters')); end
    if ~any(strcmp(dn,'Mass')), continue; end
    mRaw = getp(b,'Mass');
    m = numof(mRaw);
    % ★ 实测坑（XML 路，2026-09-19）：Simscape Multibody Link 导入出来的块是
    %   `File Solid`，它的 Mass/CoM/MoI/PoI 不是数字而是**表达式字符串**
    %   `smiData.Solid(k).mass`；str2double 对表达式给 NaN，于是刚体全被当成
    %   "质量 0" 滤掉 —— 模型里明明摆着 35 个 File Solid，却报
    %   XML_IMPORT_NO_BODIES（假阴性，实测踩过）。
    %   这里对表达式参数回落到 smiData 查表。
    smi = [];
    if ~isfinite(m) && ischar(mRaw)
        smi = smi_lookup(b, mRaw);
        if ~isempty(smi) && isfield(smi,'mass'), m = double(smi.mass); end
    end
    % ★ 实测坑：Visual（几何外形）块**也带一个 Mass 参数**，恒为 0。
    %   只按「有没有 Mass 参数」筛选，会把每个 link 的 Visual 也算成刚体
    %   （实测 3 个真刚体被数成 6 个）。用 m > 0 过滤掉。
    if ~isfinite(m) || m <= 0, continue; end

    Iv  = numof(getp(b,'MomentsOfInertia'));
    Ipv = numof(getp(b,'ProductsOfInertia'));

    if ~isempty(smi)
        % ---- XML 路：smiData 就是插件写进 XML 的权威值，直接取 ----
        co  = pad3(smi, 'CoM');
        Iv  = pad3(smi, 'MoI');
        Ipv = pad3(smi, 'PoI');
    else
        % ---- URDF 路：质心偏移不在 Inertia 块自己身上 ----
        % 而在同级的 InertiaOriginTransform（Rigid Transform）块的
        % TranslationCartesianOffset 里。同一 link 子系统里**有多个块**带
        % 这个参数（InertiaOriginTransform / VisualOriginTransform /
        % hip_L_AxisTransform / hip_L_OriginTransform …），必须显式挑名字含
        % InertiaOriginTransform 的那个 —— 靠 find_system 的返回顺序
        % "碰巧取第一个"是脆弱的。
        co  = [NaN NaN NaN];
        par = get_param(b,'Parent');
        sib = find_system(par,'SearchDepth',1,'Type','Block');
        cand = {};
        for j = 1:numel(sib)
            dnj = {};
            try, dnj = fieldnames(get_param(sib{j},'DialogParameters')); end
            if any(strcmp(dnj,'TranslationCartesianOffset'))
                cand{end+1} = sib{j}; %#ok<AGROW>
            end
        end
        pick = '';
        for j = 1:numel(cand)
            if ~isempty(strfind(cand{j},'InertiaOriginTransform'))
                pick = cand{j};
                break
            end
        end
        if isempty(pick) && ~isempty(cand), pick = cand{1}; end
        if ~isempty(pick)
            v = numof(getp(pick,'TranslationCartesianOffset'));
            if numel(v) == 3, co = v(:).'; end
        end
    end

    nm = strrep(b,[mdl '/'],'');
    B(end+1) = struct('name',nm,'m',m, ...            %#ok<AGROW>
                      'com',co, ...
                      'I',Iv, ...
                      'Ip',Ipv);
    if ~strcmp(root,mdl), B(end).name = strrep(b,[root '/'],''); end
end
end

function v = getp(b,prm)
v = [];
try, v = get_param(b,prm); end
end

function x = numof(v)
% get_param 对数值型对话框参数返回 **字符串**（可能是 '0.304833'，
% 也可能是 '[0.1 0.2 0.3]'）—— str2double 对后者会给 NaN，必须分开处理。
x = NaN;
if isempty(v), return; end
if isnumeric(v), x = double(v); return; end
if iscell(v), x = cellfun(@(z) numof(z), v); return; end
if ischar(v)
    s = strtrim(v);
    if isempty(s), return; end
    if s(1) == '['
        y = str2num(s); %#ok<ST2NM>
        if ~isempty(y), x = y; end
    else
        x = str2double(s);
    end
end
end

function s = okstr(tf)
if tf, s = 'OK'; else, s = 'DIFF'; end
end

function s = smi_lookup(blk, exprstr)
%SMI_LOOKUP  从 'smiData.Solid(k).mass' 这类表达式里取第 k 条 smiData 记录。
%   XML 路（Simscape Multibody Link 导入）的参数是表达式而非数值，
%   必须回模型工作区 / 基础工作区把 smiData 取出来。
s = [];
tok = regexp(exprstr, 'Solid\((\d+)\)', 'tokens', 'once');
if isempty(tok), return; end
k = str2double(tok{1});
if ~isfinite(k), return; end

S = [];
try
    mdlName = bdroot(blk);
    mw = get_param(mdlName, 'ModelWorkspace');
    if mw.hasVariable('smiData')
        S = mw.getVariable('smiData');
    end
end
if isempty(S)
    try, S = evalin('base', 'smiData'); end %#ok<TRYNC>
end
if isempty(S), return; end

if isfield(S, 'Solid'), sol = S.Solid; else, sol = S; end
if k >= 1 && k <= numel(sol), s = sol(k); end
end

function v = pad3(s, f)
%PAD3  从 smiData 记录里取一个字段，统一成 1x3；取不到返回 [NaN NaN NaN]。
v = [NaN NaN NaN];
if ~isstruct(s) || ~isfield(s, f), return; end
x = s.(f);
if ~isnumeric(x), return; end
x = x(:).';
if numel(x) >= 3, v = x(1:3); end
end
