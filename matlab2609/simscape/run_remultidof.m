function run_remultidof()
%RUN_REMULTIDOF  一次性把「移动副版」多自由度模型重建完
%
%   1) import_exo_multidof('free')     -> sm_exo_multidof.slx
%   2) import_exo_multidof('locked')   -> sm_exo_multidof_locked.slx
%   3) smoke_multidof()                -> 跑三模型 + 拆段回归
%
%   ⚠ 必须 cd 进 simscape 目录再跑：URDF 里 STL 用相对路径 meshes/*.stl。
%   ⚠ smimport 只在内存建模型，-batch 下必须显式 save_system（脚本里已做）。

simd = fileparts(mfilename('fullpath'));
cd(simd);
fprintf('cwd = %s\n', pwd);

try
    a = import_exo_multidof('free');
    fprintf('import free   : ok=%d  njoints=%d\n', a.ok, a.njoints);
catch ME
    fprintf('import free   : FAIL %s\n', ME.message);
end

try
    b = import_exo_multidof('locked');
    fprintf('import locked : ok=%d  njoints=%d\n', b.ok, b.njoints);
catch ME
    fprintf('import locked : FAIL %s\n', ME.message);
end

try
    smoke_multidof();
    fprintf('smoke         : done\n');
catch ME
    fprintf('smoke         : FAIL %s\n', ME.message);
end

fprintf('RUN_REMULTIDOF_DONE\n');
end
