function info = import_exo_multidof(which)
%IMPORT_EXO_MULTIDOF  smimport 把 exo_multidof.urdf 导入成 Simscape Multibody 模型
%
%   info = import_exo_multidof()           % 'free'   -> sm_exo_multidof.slx
%   info = import_exo_multidof('locked')   % 新关节按 fixed 写 -> sm_exo_multidof_locked.slx
%
%   为什么要两个模型：
%     free   = 每腿 3 段 3 关节（hip 屈伸 + abduct 髋部横向 + slide 导轨接口）
%              ★ 真正可动的只有 hip + abduct（4 DOF）；slide 经复查判定为「2mm 装配
%                间隙、非设计行程」且工作时被双键锁住，已写成 fixed 忽略。
%     locked = 同样拓扑但 abduct 也 fixed -> 整体应与原 2-DOF 模型**逐位一致**，
%              用来回归验证「拆段本身没有改变动力学」（质量/惯量守恒）。
%
%   ★ 3D 渲染：leg_*_3（滑块+按键）的 <visual> 已按 SHOW_SEG 隐藏，但 <inertial>
%     保留 -> Mechanics Explorer 里看不到滑块，动力学一个字节都没变。
%
%   注意（沿用 import_exo2dof.m 的既有经验）：
%     * smimport 只吃 Simscape Multibody XML / URDF，不吃 STEP。
%     * smimport 只在内存里建模型，-batch 下退出即丢 -> 必须显式 save_system。
%     * URDF 输入不产生参数数据文件，第二个输出恒为 ''。

if nargin < 1, which = 'free'; end
switch lower(which)
    case 'free',   uf = 'exo_multidof.urdf';        mdl = 'sm_exo_multidof';
    case 'locked', uf = 'exo_multidof_locked.urdf'; mdl = 'sm_exo_multidof_locked';
    otherwise, error('import_exo_multidof:badArg','which must be free|locked');
end

here = fileparts(fileparts(mfilename('fullpath')));      % .../matlab2609
simd = fullfile(here,'simscape');
urdf = fullfile(simd,uf);
logf = fullfile(simd,['import_' mdl '_log.txt']);

fid = fopen(logf,'w');
c = onCleanup(@() fclose(fid));
fprintf(fid,'== import_exo_multidof(%s) ==\n', which);
fprintf(fid,'time : %s\n', datestr(now));
fprintf(fid,'urdf : %s\n', urdf);
if isempty(dir(urdf))
    fprintf(fid,'FAIL: URDF not found\nIMPORT_MULTIDOF_FAIL\n');
    info = struct('ok',false); return
end

if bdIsLoaded(mdl), close_system(mdl,0); end

% ---------- 1. import ----------
try
    [H, dfn] = smimport(urdf,'ModelName',mdl,'ModelSimplification','bringJointsToTop');
    fprintf(fid,'smimport : OK  handle=%g  datafile="%s"\n', H, char(string(dfn)));
catch ME
    fprintf(fid,'smimport : FAIL  %s\n', ME.message);
    fprintf(fid,'IMPORT_MULTIDOF_FAIL\n');
    info = struct('ok',false); return
end

% ---------- 2. persist ----------
slx = fullfile(simd,[mdl '.slx']);
save_system(mdl, slx);
d = dir(slx);
fprintf(fid,'save_system : %s  (%d bytes)\n', logical_str(~isempty(d)), ...
        (~~isempty(d))*d(1).bytes);

% ---------- 3. joint inventory ----------
blks = find_system(mdl,'SearchDepth',Inf,'Type','Block');
fprintf(fid,'\n-- blocks = %d --\n', numel(blks));
joints = {};
for k = 1:numel(blks)
    bt=''; mt='';
    try, bt = get_param(blks{k},'BlockType'); end
    try, mt = get_param(blks{k},'MaskType');  end
    if ~isempty(strfind(lower(mt),'joint')) || ~isempty(strfind(lower(bt),'joint'))
        joints{end+1} = blks{k}; %#ok<AGROW>
    end
end
fprintf(fid,'\n-- joint blocks = %d --\n', numel(joints));
for k = 1:numel(joints)
    ax = ''; ty = '';
    try, ax = get_param(joints{k},'JointAxis');  end
    try, ty = get_param(joints{k},'JointType');  end
    fprintf(fid,'  [%d] %s\n        JointType=%s  JointAxis=%s\n', k, joints{k}, ty, ax);
end

% ---------- 4. 数一数每段的 Solid 块（确认网格/外形也进来了）----------
fprintf(fid,'\n-- solid-ish blocks --\n');
for k = 1:numel(blks)
    mt=''; try, mt = get_param(blks{k},'MaskType'); end
    if ~isempty(strfind(lower(mt),'solid')) || ~isempty(strfind(lower(mt),'inertia'))
        fprintf(fid,'  %s  [%s]\n', blks{k}, mt);
    end
end

tops = find_system(mdl,'SearchDepth',1,'Type','Block');
fprintf(fid,'\n-- top level (%d) --\n', numel(tops));
for k = 1:numel(tops), fprintf(fid,'  %s\n', tops{k}); end

fprintf(fid,'\nIMPORT_MULTIDOF_OK\n');
info = struct('ok',true,'slx',slx,'model',mdl,'njoints',numel(joints), ...
              'joints',{joints},'log',logf);
end

function s = logical_str(tf)
if tf, s = 'OK'; else, s = 'MISSING'; end
end
