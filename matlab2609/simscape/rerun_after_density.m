function info = rerun_after_density()
%RERUN_AFTER_DENSITY  密度表/URDF 变更后，一次性把 Simscape 侧全部基线重跑一遍。
%
%   为什么需要这个脚本
%   ------------------
%   URDF 是 Simscape 模型参数的唯一来源，而 `smimport` 把**块参数烤进 .slx**
%   （import_exo2dof.m 注释原文：for URDF input no parameter data file is produced,
%    the block values are baked in）。所以只要密度表动了 ->
%   `exo_real.urdf` / `exo_multidof.urdf` 变了 -> 下面这些 **必须** 重建，
%   否则 Simscape 侧还留着旧惯量，而解析侧读的是新 CSV，两侧会系统性地错开：
%
%     1) import_exo2dof()                  -> sm_exo_real.slx
%     2) build_harness_exo2dof()           -> sm_exo_real_harness.slx
%     3) compare_simscape_vs_analytical()  -> COMPARE_OK
%     4) import_exo_multidof('free'|'locked') -> sm_exo_multidof[_locked].slx
%     5) smoke_multidof()                  -> 拆段回归 PASS（locked vs 2-DOF）
%     6) dump_multidof_traj()              -> out_simscape/multidof_traj.csv
%     7) quick_multidof_check()            -> QUICK_OK
%     8) test_multidof_analytical()        -> TEST_MULTIDOF_ALL_OK
%
%   本脚本**不改任何模型参数**，只是按正确顺序重放既有脚本。
%   之后还要在仓库根跑一次 python _multidof_compare.py（判据 D）。
%
%   ⚠ 必须 cd 进 simscape：URDF 里 STL 用相对路径 meshes/*.stl。
%   ⚠ 本脚本是 ASCII 路径（junction）友好写法：路径全部从 mfilename 推。

simd  = fileparts(mfilename('fullpath'));       % .../matlab2609/simscape
rootd = fileparts(simd);                        % .../matlab2609
addpath(rootd); addpath(simd);
cd(simd);

logf = fullfile(simd,'rerun_after_density_log.txt');
fid  = fopen(logf,'w'); c = onCleanup(@() fclose(fid));
fprintf(fid,'== rerun_after_density ==\n%s\ncwd = %s\n\n', datestr(now), pwd);

R = struct('step',{},'ok',{},'msg',{});
    function run1(name, f)
        fprintf(fid,'\n----- %s -----\n', name);
        fprintf('%s ...\n', name);
        t0 = tic;
        try
            f();
            fprintf(fid,'%s : OK  (%.1f s)\n', name, toc(t0));
            fprintf('%s : OK\n', name);
            R(end+1) = struct('step',name,'ok',true,'msg','OK'); %#ok<AGROW>
        catch ME
            fprintf(fid,'%s : FAIL  %s | %s\n', name, ME.identifier, ME.message);
            if ~isempty(ME.stack)
                fprintf(fid,'    @ %s line %d\n', ME.stack(1).name, ME.stack(1).line);
            end
            fprintf('%s : FAIL %s\n', name, ME.message);
            R(end+1) = struct('step',name,'ok',false,'msg',ME.message); %#ok<AGROW>
        end
    end

% ---------------- 2-DOF ----------------
run1('import_exo2dof',              @() import_exo2dof());
run1('build_harness_exo2dof',       @() build_harness_exo2dof());
run1('compare_simscape_vs_analytical', @() compare_simscape_vs_analytical());

% ---------------- 4-DOF ----------------
run1('import_multidof_free',        @() import_exo_multidof('free'));
run1('import_multidof_locked',      @() import_exo_multidof('locked'));
run1('smoke_multidof',              @() smoke_multidof());
run1('dump_multidof_traj',          @() dump_multidof_traj());
run1('quick_multidof_check',        @() quick_multidof_check());
run1('test_multidof_analytical',    @() test_multidof_analytical());

% ---------------- 判据复核 ----------------
% ★ 上面 run1 只保证「脚本没抛异常」。但本工程的判据脚本**失败时也不抛异常**，
%   只是往自己的 log 里写 FAIL —— 所以必须回到日志里核对 verdict 字样，
%   否则会出现「step 全 OK，其实判据是 FAIL」的假绿。（本次就踩过。）
fprintf(fid,'\n=================== 判据复核 ===================\n');
V = { ...
    'compare(2-DOF)', 'COMPARE_OK',            'COMPARE_FAIL',               fullfile(simd,'compare_log.txt'); ...
    'smoke(拆段回归)', 'VERDICT: PASS',         'VERDICT: FAIL',              fullfile(simd,'smoke_multidof_log.txt'); ...
    'quick(4-DOF)',   'QUICK_MULTIDOF_OK',     'QUICK_MULTIDOF_FAIL',        fullfile(simd,'quick_multidof_check_log.txt'); ...
    'test4(A/B/C)',   'TEST_MULTIDOF_ALL_OK',  'TEST_MULTIDOF_CHECK_FAILED', fullfile(simd,'test_multidof_analytical_log.txt')};
% ★ pass/fail 必须是**互不包含**的完整 token。踩过的坑：上一版把 fail 写成
%   'COMPARE_'，而它正是 pass 的 'COMPARE_OK' 的前缀 -> 永远报 FAIL。
%   这里统一用各脚本终端 fprintf 的完整 verdict 串（见各自源码末段）。
chk = true(1,size(V,1));
for k = 1:size(V,1)
    txt = '';
    if ~isempty(dir(V{k,4}))
        txt = fileread(V{k,4});
    end
    chk(k) = ~isempty(strfind(txt, V{k,2})) && isempty(strfind(txt, V{k,3}));
    fprintf(fid,'  %-18s %s   (pass=%s  fail=%s)\n', V{k,1}, ternary(chk(k),'PASS','FAIL'), V{k,2}, V{k,3});
end

% ---------------- 汇总 ----------------
fprintf(fid,'\n=================== SUMMARY ===================\n');
for k = 1:numel(R)
    fprintf(fid,'  %-34s %s\n', R(k).step, ternary(R(k).ok,'OK','FAIL'));
end
allok = all([R.ok]) && all(chk);
fprintf(fid,'VERDICT: %s\n', ternary(allok,'RERUN_ALL_OK','RERUN_HAS_FAILURE'));
fprintf(fid,'RERUN_AFTER_DENSITY_%s\n', ternary(allok,'OK','FAIL'));

info = struct('ok',allok,'steps',{R},'verdicts',{V},'chk',chk,'log',logf);
end

function s = ternary(tf,a,b), if tf, s=a; else, s=b; end, end
