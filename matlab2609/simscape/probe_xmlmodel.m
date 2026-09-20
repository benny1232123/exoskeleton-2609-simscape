function probe_xmlmodel(mdl)
%PROBE_XMLMODEL  体检 smimport 由 XML 生成的模型：块类型分布 / 质量参数 / DataFile
if nargin < 1, mdl = 'sm_exo_legL_xml'; end
here = fileparts(mfilename('fullpath'));
logf = fullfile(here, 'probe_xmlmodel_log.txt');
fid  = fopen(logf, 'w');
c = onCleanup(@() fclose(fid));

fprintf(fid, '== probe_xmlmodel ==\nmodel = %s\n', mdl);
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

b = find_system(mdl, 'SearchDepth', Inf, 'Type', 'Block');
fprintf(fid, 'blocks total = %d\n', numel(b));

mtl = cell(1, numel(b)); btl = cell(1, numel(b));
for k = 1:numel(b)
    mt = ''; try, mt = get_param(b{k}, 'MaskType'); end
    bt = ''; try, bt = get_param(b{k}, 'BlockType'); end
    mtl{k} = mt; btl{k} = bt;
end

fprintf(fid, '\n-- MaskType 统计 --\n');
u = unique(mtl);
for k = 1:numel(u)
    fprintf(fid, '  [%s]  %d\n', u{k}, sum(strcmp(mtl, u{k})));
end

fprintf(fid, '\n-- BlockType 统计 --\n');
u = unique(btl);
for k = 1:numel(u)
    fprintf(fid, '  [%s]  %d\n', u{k}, sum(strcmp(btl, u{k})));
end

fprintf(fid, '\n-- 顶层块 (%d) --\n', numel(find_system(mdl,'SearchDepth',1,'Type','Block')));
t = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
for k = 1:numel(t)
    fprintf(fid, '  %s\n', t{k});
end

fprintf(fid, '\n-- 前 30 个块明细 --\n');
for k = 1:min(30, numel(b))
    dn = {}; try, dn = fieldnames(get_param(b{k}, 'DialogParameters')); end
    hasMass = any(strcmp(dn, 'Mass'));
    mv = NaN; mi = [NaN NaN NaN];
    if hasMass
        try
            mv = numof(get_param(b{k}, 'Mass'));
        end
    end
    try, mi = numof(get_param(b{k}, 'MomentsOfInertia')); end
    fprintf(fid, '  %-64s mask=[%s] nP=%d Mass=%s I=[%s]\n', ...
        b{k}, mtl{k}, numel(dn), mat2str(mv), mat2str(mi));
end

fprintf(fid, '\n-- DataFile --\n');
v = evalin('base', 'who');
fprintf(fid, 'base vars: %s\n', strjoin(v, ', '));
d = dir(fullfile(here, '*.mat'));
for k = 1:numel(d)
    fprintf(fid, '  mat(simscape): %s (%d bytes)\n', d(k).name, d(k).bytes);
end
d2 = dir('D:\exo_xml\*.mat');
for k = 1:numel(d2)
    fprintf(fid, '  mat(exo_xml) : %s (%d bytes)\n', d2(k).name, d2(k).bytes);
end

fprintf(fid, '\nDONE\n');
end

function x = numof(v)
x = NaN;
if isempty(v), return; end
if isnumeric(v), x = double(v); return; end
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
