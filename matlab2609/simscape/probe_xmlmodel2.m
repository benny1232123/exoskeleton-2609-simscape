function probe_xmlmodel2(mdl)
%PROBE_XMLMODEL2  把 File Solid 的真实参数全部 dump 出来，
%   并核对每个 Solid 引用的 STEP 文件能否解析。
if nargin < 1, mdl = 'sm_exo_legL_xml'; end
here = fileparts(mfilename('fullpath'));
fid  = fopen(fullfile(here, 'probe_xmlmodel2_log.txt'), 'w');
c = onCleanup(@() fclose(fid));

if ~bdIsLoaded(mdl), load_system(mdl); end
b = find_system(mdl, 'SearchDepth', Inf, 'Type', 'Block', 'MaskType', 'File Solid');
fprintf(fid, 'File Solid count = %d\n', numel(b));
if isempty(b)
    fprintf(fid, 'none found\n');
    return;
end

blk = b{1};
dn = fieldnames(get_param(blk, 'DialogParameters'));
fprintf(fid, '\n== %s 全部 %d 个参数 ==\n', blk, numel(dn));
for k = 1:numel(dn)
    v = '';
    try, v = get_param(blk, dn{k}); end
    if ~ischar(v)
        try, v = mat2str(v); end
    end
    if numel(v) > 150, v = [v(1:150) '...']; end
    fprintf(fid, '  %-36s = %s\n', dn{k}, v);
end

fprintf(fid, '\n== 全部 File Solid 引用的几何文件 ==\n');
nmiss = 0; names = {};
for k = 1:numel(b)
    fn = '';
    for cand = {'FileName', 'File Name', 'STEPFile', 'File'}
        try
            fn = get_param(b{k}, cand{1});
            if ~isempty(fn), break; end
        end
    end
    if isempty(fn)
        fn = '<无该参数>';
    end
    ex = 0;
    if ischar(fn) && ~strcmp(fn, '<无该参数>')
        ex = exist(fn, 'file');
    end
    if ex == 0, nmiss = nmiss + 1; end
    fprintf(fid, '  %-34s exist=%d  %s\n', b{k}, ex, fn);
    names{end+1} = fn; %#ok<AGROW>
end
fprintf(fid, '\n文件不可解析 = %d / %d\n', nmiss, numel(b));
fprintf(fid, '\n== 参数名里含 Mass/Density/Inertia 的 ==\n');
for k = 1:numel(dn)
    if ~isempty(strfind(lower(dn{k}), 'mass')) || ...
       ~isempty(strfind(lower(dn{k}), 'density')) || ...
       ~isempty(strfind(lower(dn{k}), 'inertia'))
        v = ''; try, v = get_param(blk, dn{k}); end
        if ~ischar(v), try, v = mat2str(v); end, end
        if numel(v) > 150, v = [v(1:150) '...']; end
        fprintf(fid, '  %-36s = %s\n', dn{k}, v);
    end
end
fprintf(fid, '\nDONE\n');
end
