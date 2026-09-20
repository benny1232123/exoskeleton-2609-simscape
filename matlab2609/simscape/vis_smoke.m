function vis_smoke()
%VIS_SMOKE  加了 <visual> 网格之后，把整条链路重跑一遍，验证两件事：
%           (a) STL 网格真的进了 Simscape 模型（不是被 smimport 静默忽略）；
%           (b) 动力学**一点没被改坏**（视觉网格不该影响 <inertial>）。
%
%   为什么必须回归
%   --------------
%   重跑 `smimport` 会**整个重建** sm_exo_real.slx，连带 harness 也要重建。
%   只要 URDF 里多了一段 <visual>，就有"顺手动到别处"的风险。
%   所以这里按顺序跑：导入 -> 挂网格 -> 重建 harness -> L2 轨迹对照 -> L2' 能量守恒，
%   每一步都记进日志，任何一步失败都能立刻定位。
%
%   跑法（本机 ASCII junction，绕开中文路径编码）：
%     cd C:\Users\29408\exo_work\matlab2609\simscape
%     matlab -batch "vis_smoke"

here = fileparts(fileparts(mfilename('fullpath')));     % .../matlab2609
simd = fullfile(here, 'simscape');
addpath(simd);
logf = fullfile(simd, 'vis_smoke_log.txt');
fid  = fopen(logf, 'w');
c    = onCleanup(@() fclose(fid));

fprintf(fid, '== vis_smoke ==\n%s\nmatlabroot: %s\n\n', datestr(now), matlabroot);

stages = { ...
    'import_exo2dof',              @() import_exo2dof(); ...
    'attach_visual_meshes',        @() attach_visual_meshes(); ...
    'build_harness_exo2dof',       @() build_harness_exo2dof(); ...
    'compare_simscape_vs_analytical', @() compare_simscape_vs_analytical(); ...
    'validate_energy(3)',          @() validate_energy(3); ...
};

nfail = 0;
for k = 1:size(stages, 1)
    nm = stages{k, 1};
    fprintf(fid, '==== [%d/%d] %s ====\n', k, size(stages, 1), nm);
    t0 = tic;
    try
        stages{k, 2}();
        fprintf(fid, '---- %s : OK  (%.1f s)\n\n', nm, toc(t0));
    catch ME
        nfail = nfail + 1;
        fprintf(fid, '---- %s : FAILED  (%.1f s)\n%s\n%s\n\n', ...
                nm, toc(t0), ME.message, getReport(ME, 'basic', 'hyperlinks', 'off'));
    end
end

fprintf(fid, '================================\n');
if nfail == 0
    fprintf(fid, 'VERDICT: VIS_SMOKE_OK\n');
else
    fprintf(fid, 'VERDICT: VIS_SMOKE_FAILED  (%d/%d 步失败)\n', nfail, size(stages, 1));
end
fprintf(fid, 'VIS_SMOKE_DONE\n');
fprintf('log -> %s\n', logf);
end
