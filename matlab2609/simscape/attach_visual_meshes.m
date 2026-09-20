function attach_visual_meshes()
%ATTACH_VISUAL_MESHES  核验（并必要时修正）smimport 是否真把 URDF 的
%                      <visual><mesh> 吃进了模型，让 Mechanics Explorer 有外形。
%
%   背景
%   ----
%   URDF 的 <inertial> 只给动力学，**不含几何**。所以哪怕惯量/质心逐位正确，
%   Mechanics Explorer 里也只有占位体。要看到真实外形，必须在 URDF 里写
%   <visual><geometry><mesh filename="..."/>，由 _stp2stl.py 产出 STL。
%
%   本脚本做两件事：
%   1) 证据：逐个块扫描所有**字符型** DialogParameters，找出值里含 '.stl' 的项
%      —— 这证明 <visual> 确实进了模型（而不是被静默忽略）。
%      这跟「读 InertiaOriginTransform.TranslationCartesianOffset 来证明惯性
%      参数真的进模型了」是同一类免 GUI 证据。
%   2) 修正：把相对路径 'meshes/x.stl' 改写成 ASCII junction 下的**绝对路径**，
%      绕开中文路径在 Simscape STL 读取时的编码风险，然后 save_system。
%      （ASCII junction：C:\Users\29408\exo_work -> 项目目录）
%
%   不做几何/动力学计算，因此不需要 sim，也不改任何动力学参数。

here = fileparts(fileparts(mfilename('fullpath')));       % .../matlab2609
simd = fullfile(here,'simscape');
mdl  = 'sm_exo_real';
slx  = fullfile(simd,[mdl '.slx']);
logf = fullfile(simd,'attach_visual_meshes_log.txt');
fid  = fopen(logf,'w');
c    = onCleanup(@() fclose(fid));

fprintf(fid,'== attach_visual_meshes ==\n%s\n', datestr(now));
fprintf(fid,'model : %s\n\n', slx);
if isempty(dir(slx))
    fprintf(fid,'VERDICT: FAIL  model not found\n');
    return
end

% 项目目录的 ASCII 别名（junction）。没有就退回真实路径。
ascii_root = 'C:\Users\29408\exo_work';
if isempty(dir(ascii_root))
    ascii_root = here;
    fprintf(fid,'note: ASCII junction not found -> fall back to %s\n\n', ascii_root);
end
ascii_simd = fullfile(ascii_root,'matlab2609','simscape');

if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(slx);

blks = find_system(mdl,'SearchDepth',Inf,'Type','Block');
fprintf(fid,'scanning %d blocks for character parameters containing ".stl" ...\n\n', numel(blks));

nHit = 0; nOk = 0; nFix = 0; nBad = 0; nPredirty = 0;
for k = 1:numel(blks)
    b = blks{k};
    try
        dp = get_param(b,'DialogParameters');
    catch
        continue
    end
    if ~isstruct(dp), continue; end
    fn = fieldnames(dp);
    for j = 1:numel(fn)
        nm = fn{j};
        try
            v = get_param(b,nm);
        catch
            continue
        end
        if ~ischar(v) || isempty(strfind(lower(v),'.stl')), continue; end

        nHit = nHit + 1;
        rel = v;
        fprintf(fid,'[%d] %s\n      param  = %s\n      value  = %s\n', nHit, b, nm, rel);

        % 候选解析：相对 simscape 目录 / 相对模型所在目录 / 原始绝对路径
        cands = { rel, ...
                  fullfile(simd,rel), ...
                  fullfile(fileparts(simd),rel), ...
                  fullfile(ascii_simd,rel) };
        use = ''; how = '';
        for q = 1:numel(cands)
            d = dir(cands{q});
            if ~isempty(d) && d.bytes > 0
                use = cands{q}; how = sprintf('candidate #%d',q); break
            end
        end
        if isempty(use)
            % 相对路径解不出来 -> 用 ASCII 绝对路径兜底
            [~,nmf,ext] = fileparts(rel);
            use = fullfile(ascii_simd, [nmf ext]);
            how = 'fallback: ascii simscape dir + basename';
        end

        d = dir(use);
        if isempty(d) || d.bytes == 0
            fprintf(fid,'      -> RESOLVE FAIL: %s (not found)\n\n', use);
            nBad = nBad + 1;
            continue
        end
        fprintf(fid,'      -> resolved (%s) : %s  [%.2f MB]\n', how, use, d.bytes/1048576);

        if ~strcmp(rel,use)
            try
                set_param(b,nm,use);
                nFix = nFix + 1;
                nPredirty = 1;
                fprintf(fid,'      -> rewritten to absolute path\n');
            catch ME
                fprintf(fid,'      -> set_param failed: %s\n', ME.message);
                nBad = nBad + 1;
            end
        else
            fprintf(fid,'      -> unchanged\n');
        end
        nOk = nOk + 1;
        fprintf(fid,'\n');
    end
end

fprintf(fid,'hit=%d  resolved=%d  rewritten=%d  failed=%d\n\n', nHit, nOk, nFix, nBad);
for nm = {'base','leg_L','leg_R'}
    p = fullfile(ascii_simd,'meshes',[nm{1} '.stl']);
    d = dir(p);
    if isempty(d)
        fprintf(fid,'  mesh %-6s MISSING  %s\n', nm{1}, p);
    else
        fprintf(fid,'  mesh %-6s %8.2f MB  %s\n', nm{1}, d.bytes/1048576, p);
    end
end

if nHit == 0
    fprintf(fid,'\nVERDICT: NO_MESH_IN_MODEL  <visual> 没进模型（URDF 里没写，或 smimport 忽略了）\n');
elseif nBad > 0
    fprintf(fid,'\nVERDICT: PARTIAL  %d 项无法解析\n', nBad);
else
    fprintf(fid,'\nVERDICT: VISUAL_MESH_OK  共 %d 项，全部指向存在的 STL\n', nHit);
end

if nPredirty
    save_system(mdl, slx);
    d = dir(slx);
    fprintf(fid,'save_system: OK  (%d bytes)\n', d.bytes);
end
if ~usejava('desktop')
    close_system(mdl,0);
end
fprintf(fid,'ATTACH_VISUAL_MESHES_DONE\n');
fprintf(fid,'log -> %s\n', logf);
end
