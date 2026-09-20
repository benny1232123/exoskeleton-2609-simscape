function probe_xmlmodel3(mdl)
%PROBE_XMLMODEL3  找到 smiData，dump 成 CSV 供 Python 对账；并 dump 35 个 Solid 的几何文件名。
if nargin < 1, mdl = 'sm_exo_legL_xml'; end
here = fileparts(mfilename('fullpath'));
logf = fullfile(here, 'probe_xmlmodel3_log.txt');
csvf = fullfile(here, 'smiData_dump.csv');
fid  = fopen(logf, 'w');
c = onCleanup(@() fclose(fid));

if ~bdIsLoaded(mdl), load_system(mdl); end
fprintf(fid, 'model = %s\n', mdl);

% ---- 1. 找 smiData ----
S = [];
src = '';
try
    mw = get_param(mdl, 'ModelWorkspace');
    if mw.hasVariable('smiData')
        S = mw.getVariable('smiData'); src = 'model workspace';
    end
end
if isempty(S)
    try
        S = evalin('base', 'smiData'); src = 'base workspace';
    end
end
if isempty(S)
    d = dir(fullfile(here, '**', '*DataFile*.mat'));
    fprintf(fid, '发现候选数据文件 %d 个\n', numel(d));
    for k = 1:numel(d)
        fprintf(fid, '   %s\n', fullfile(d(k).folder, d(k).name));
        try
            L = load(fullfile(d(k).folder, d(k).name));
            f = fieldnames(L);
            for j = 1:numel(f)
                if isstruct(L.(f{j})) && isfield(L.(f{j}), 'Solid')
                    S = L.(f{j}); src = sprintf('%s [%s]', d(k).name, f{j});
                end
            end
        end
    end
end
fprintf(fid, 'smiData 来源 = %s\n', src);
if isempty(S)
    fprintf(fid, '!!! 没找到 smiData\n');
    fprintf(fid, '\n== 模型工作区变量 ==\n');
    try
        mw = get_param(mdl, 'ModelWorkspace');
        w = mw.whos;
        for k = 1:numel(w)
            fprintf(fid, '   %-24s %s %s\n', w(k).name, w(k).class, mat2str(w(k).size));
        end
    end
    fprintf(fid, '\nDONE\n');
    return;
end

if isfield(S, 'Solid'), sol = S.Solid; else, sol = S; end
n = numel(sol);
fprintf(fid, 'Solid 条目数 = %d\n', n);
fprintf(fid, '\n字段名: %s\n', strjoin(fieldnames(sol(1)), ', '));

f2 = fopen(csvf, 'w');
fprintf(f2, 'idx,name,mass_kg,comx_mm,comy_mm,comz_mm,Ixx,Iyy,Izz,Ixy,Ixz,Iyz\n');
for k = 1:n
    s = sol(k);
    nm = '';
    if isfield(s, 'name'), nm = char(string(s.name)); end
    m  = getf(s, 'mass');
    co = getf(s, 'CoM');
    I  = getf(s, 'MoI');
    P  = getf(s, 'PoI');
    co = pad3(co); I = pad3(I); P = pad3(P);
    fprintf(f2, '%d,%s,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n', ...
        k, nm, m, co(1), co(2), co(3), I(1), I(2), I(3), P(1), P(2), P(3));
end
fclose(f2);
fprintf(fid, 'CSV -> %s\n', csvf);

fprintf(fid, '\n-- 前 12 条 --\n');
for k = 1:min(12, n)
    s = sol(k);
    fprintf(fid, '  [%2d] m=%.9f kg  CoM=[%s]  MoI=[%s]\n', k, getf(s,'mass'), ...
        mat2str(pad3(getf(s,'CoM')), 8), mat2str(pad3(getf(s,'MoI')), 8));
end
fprintf(fid, '  Σ mass = %.9f kg\n', sum(arrayfun(@(z) getf(z,'mass'), sol)));

% ---- 2. 35 个 Solid 的几何文件名 ----
b = find_system(mdl, 'SearchDepth', Inf, 'Type', 'Block', 'MaskType', 'File Solid');
fprintf(fid, '\n-- File Solid (%d) 的 ExtGeomFileName --\n', numel(b));
for k = 1:numel(b)
    fn = ''; try, fn = get_param(b{k}, 'ExtGeomFileName'); end
    fprintf(fid, '  %-40s %s\n', b{k}, fn);
end
fprintf(fid, '\nDONE\n');
end

function v = getf(s, f)
v = NaN;
if isfield(s, f)
    v = s.(f);
    if ~isnumeric(v)
        try, v = double(string(v)); end
    end
end
end

function v = pad3(x)
v = [NaN NaN NaN];
x = x(:).';
if numel(x) >= 3, v = x(1:3); end
end
