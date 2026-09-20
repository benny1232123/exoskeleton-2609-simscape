function probe_anim_inputs()
%PROBE_ANIM_INPUTS  给「离线 3D 动画渲染器」铺路：确认底座是否焊死在 World，
%   并量出三个 STL 的包围盒，用来定坐标轴范围/相机。只读。

here = fileparts(fileparts(mfilename('fullpath')));
simd = fullfile(here,'simscape');
logf = fullfile(simd,'probe_anim_inputs_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== probe_anim_inputs ==\n%s\n\n', datestr(now));

mdl = 'sm_exo_real_harness';
slx = fullfile(simd,[mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(slx);

fprintf(fid,'---- 顶层块（BlockType / MaskType / 名字）----\n');
tops = find_system(mdl,'SearchDepth',1,'Type','Block');
for i = 1:numel(tops)
    bt = ''; mt = '';
    try, bt = get_param(tops{i},'BlockType'); catch, end
    try, mt = get_param(tops{i},'MaskType'); catch, end
    fprintf(fid,'  %-46s | %-22s | %s\n', tops{i}, bt, mt);
end

fprintf(fid,'\n---- 全部 Weld Joint / World / Mechanism Config ----\n');
for mt = {'World','Weld Joint','Mechanism Configuration','Solver Configuration'}
    hits = find_system(mdl,'SearchDepth',Inf,'MaskType',mt{1});
    for i = 1:numel(hits)
        fprintf(fid,'  %s\n', hits{i});
    end
end

fprintf(fid,'\n---- STL 包围盒 (world coords, m) ----\n');
for nm = {'base','leg_L','leg_R'}
    p = fullfile(simd,'meshes',[nm{1} '.stl']);
    if isempty(dir(p))
        fprintf(fid,'  %-6s MISSING\n', nm{1});
        continue
    end
    t0 = tic;
    TR = stlread(p);
    V  = TR.Points;
    fprintf(fid,'  %-6s tris=%d  verts=%d  read=%.1f s\n', nm{1}, ...
        size(TR.ConnectivityList,1), size(V,1), toc(t0));
    fprintf(fid,'         x[%.4f %.4f]  y[%.4f %.4f]  z[%.4f %.4f]\n', ...
        min(V(:,1)),max(V(:,1)), min(V(:,2)),max(V(:,2)), min(V(:,3)),max(V(:,3)));
end

% 关节轴线是否真穿过装配体
o = [0.113697402 0.000239573 -0.052968076];
a = [-0.002126698679 -0.999956730125 -0.009056215036];
fprintf(fid,'\n---- 髋轴 ----\n  origin = [%.9f %.9f %.9f]\n  axis   = [%.9f %.9f %.9f]  norm=%.12f\n', ...
    o, a, norm(a));

close_system(mdl,0);
fprintf(fid,'\nPROBE_ANIM_INPUTS_DONE\nlog -> %s\n', logf);
end
