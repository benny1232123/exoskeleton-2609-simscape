function n = absolutize_solid_paths(mdl, geomDir)
%ABSOLUTIZE_SOLID_PATHS  把模型里所有 File Solid 的 ExtGeomFileName 改成绝对路径。
%
% 为什么需要
%   smimport 由 XML 导入时，File Solid 的 ExtGeomFileName 只写了**裸文件名**
%   （实测：'电机_片形腿杆连接件_Default_sldprt.STEP'）。Simscape 在编译期
%   按当前工作目录解析它 —— 也就是说，除非你把 MATLAB 的 pwd 切到那个目录，
%   模型一仿真就报找不到几何文件。 .slx 里也**没有内嵌**几何（实测 391.9 KB，
%   而 STEP 合计约 7 MB）。
%
% 用法
%   absolutize_solid_paths('sm_exo_legL_xml','D:\exo_xml')
%
% 返回被改写的块数。改完记得 save_system。

if nargin < 1 || isempty(mdl),   error('需要模型名'); end
if nargin < 2 || isempty(geomDir), error('需要几何文件目录'); end

wasLoaded = bdIsLoaded(mdl);
if ~wasLoaded, load_system(mdl); end

b = find_system(mdl, 'SearchDepth', Inf, 'Type', 'Block', 'MaskType', 'File Solid');
n = 0; miss = {};
for k = 1:numel(b)
    fn = '';
    try, fn = get_param(b{k}, 'ExtGeomFileName'); end
    if ~ischar(fn) || isempty(fn), continue; end
    if isempty(fileparts(fn))                       % 裸文件名 -> 补绝对目录
        full = fullfile(geomDir, fn);
    else
        full = fn;                                  % 已是路径，只做存在性核对
    end
    if isempty(dir(full))
        miss{end+1} = fn; %#ok<AGROW>
        continue
    end
    try
        set_param(b{k}, 'ExtGeomFileName', full);
        n = n + 1;
    catch ME
        warning('设置失败 %s : %s', b{k}, ME.message);
    end
end

fprintf('absolutize_solid_paths: 改写 %d / %d 个 File Solid\n', n, numel(b));
if ~isempty(miss)
    fprintf('  以下几何文件没找到（保持原样）：\n');
    for k = 1:numel(miss)
        fprintf('    %s\n', miss{k});
    end
end
end
