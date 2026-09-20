function info = view_exo_real(varargin)
%VIEW_EXO_REAL  在 MATLAB 里打开、运行并查看「真实装配体」多体模型。
%
% 用法（交互式 MATLAB 命令行里）：
%
%   cd C:\Users\29408\exo_work\matlab2609\simscape     % ASCII junction，绕开中文路径
%   addpath(pwd)
%
%   view_exo_real                     % 默认：打开 harness，跑 1 s 受驱仿真并画图
%   view_exo_real('model','import')    % 只打开纯 CAD 导入模型（看结构，不跑仿真）
%   view_exo_real('Tamp',0.15,'f',0.3,'Tend',2)
%   view_exo_real('plot',false)        % 只跑不看图
%   view_exo_real('compare',true)      % 顺带重跑一致性对照并弹出误差图
%   view_exo_real('video',true)        % 顺带把 Mechanics Explorer 里的 3D 录成 MP4
%   view_exo_real('Tend',4,'Tamp',0.15,'video',true)   % 长一点、录下来
%
% 为什么需要这个脚本，而不是直接双击 .slx 点 Run：
%   harness 里的 T_L_src / T_R_src 是 **From Workspace** 块，需要 base 工作区里
%   先存在 T_L_data / T_R_data（timeseries）。直接点 Run 会在**编译期**报错
%   （变量不存在）。本脚本负责先把这两个变量按你要的力矩曲线准备好。
%
% 两个模型分别看什么：
%   sm_exo_real.slx          ← 纯 smimport 产物：World + 机构配置 + Solver 配置
%                              + base / leg_L / leg_R 三个刚体 + hip_L / hip_R 两个转动副
%                              关节是「原始状态」：TorqueActuationMode=NoTorque、
%                              限位 on(±1.5 rad)、sensing 全 off
%   sm_exo_real_harness.slx  ← 在上一版基础上加：力矩输入、q/ω 测量、
%                              限位 off（纯动力学对照用）。**要跑仿真用这个。**
%
% 关于 3D 动画：**外形已经接进去了**。exo_real.urdf 里写了
%   <visual><geometry><mesh filename="meshes/<link>.stl"/>，base/leg_L/leg_R
%   三个 STL（35.7/2.3/2.3 MB）都在 simscape/meshes/ 下，并且两个模型
%   （sm_exo_real 与 sm_exo_real_harness）的 Visual 块 ExtGeomFileName 都指向它们
%   （attach_visual_meshes 核验 = VISUAL_MESH_OK），SimMechanicsOpenEditorOnUpdate
%   也已打开 —— 所以 sim 一跑，Mechanics Explorer 里就是**真实装配体外形**。
%
%   ⚠ 但这些路径是**相对**的（'meshes/base.stl'），按「当前目录」解析。
%     所以本脚本开头会 cd 进 simscape 目录，结束再切回来。若你直接双击 .slx
%     再点 Run，当前目录不对，Mechanics Explorer 就只剩坐标系/占位体。
%
% 想要一份能分享的**离线视频**（不依赖 GUI、相机与配色可复现），用：
%   anim_exo_3d('preset','drop')     % 出 MP4/GIF + 三联姿态图，见该文件头注释

    p = parse_args(varargin{:});

    here = fileparts(fileparts(mfilename('fullpath')));   % .../matlab2609
    simd = fullfile(here,'simscape');
    outd = fullfile(here,'out_simscape');
    if isempty(dir(outd)), mkdir(outd); end
    addpath(simd);

    % STL 外形用的是相对路径 'meshes/x.stl'，按当前目录解析。
    % 不切进去的话 Mechanics Explorer 找不到网格，只显示坐标系/占位体。
    oldcd = cd(simd);
    restoreCd = onCleanup(@() cd(oldcd));

    hasDesktop = usejava('desktop');

    switch lower(p.model)
        case 'import'
            mdl = 'sm_exo_real';
            doSim = false;
        case {'harness','sim'}
            mdl = 'sm_exo_real_harness';
            doSim = true;
        otherwise
            error('view_exo_real:model','model 只能是 ''import'' 或 ''harness''');
    end
    if ~isempty(getenv('EXO2609_MODEL'))
        mdl = getenv('EXO2609_MODEL');
        if doSim, mdl = [mdl '_harness']; end
    end
    slx = fullfile(simd,[mdl '.slx']);
    if isempty(dir(slx))
        error('view_exo_real:noSlx', ...
            '%s 不存在。先跑 import_exo2dof / build_harness_exo2dof。', slx);
    end

    fprintf('\n===== view_exo_real =====\n');
    fprintf('model : %s\n', mdl);
    fprintf('slx   : %s\n', slx);
    fprintf('desktop: %s\n\n', ternary(hasDesktop,'yes (GUI)','no (-batch)'));

    % ---------- 1. 打开 / 载入 ----------
    if bdIsLoaded(mdl), close_system(mdl,0); end
    load_system(slx);
    if hasDesktop
        open_system(mdl);      % 弹出 Simulink 模型窗口
    end

    % ---------- 2. 把模型的关键配置打出来（省得逐个双击看） ----------
    print_config(mdl, simd);

    % ---------- 3. 跑仿真 + 画图 ----------
    if doSim
        Tamp = p.Tamp;  f = p.f;  Tend = p.Tend;
        tt   = (0:1e-4:Tend).';
        TL   =  Tamp*sin(2*pi*f*tt);
        TR   = -Tamp*sin(2*pi*f*tt);      % 左右反相，便于区分两条腿
        assignin('base','T_L_data', timeseries(TL, tt));
        assignin('base','T_R_data', timeseries(TR, tt));

        fprintf('-- 仿真 --\n');
        fprintf('  关节力矩 T_L(t) = %+.3f*sin(2*pi*%g*t) N*m\n', Tamp, f);
        fprintf('  关节力矩 T_R(t) = %+.3f*sin(2*pi*%g*t) N*m\n', -Tamp, f);
        fprintf('  时长 %.3f s，求解器 %s\n', Tend, get_param(mdl,'Solver'));

        set_param(mdl,'SaveOutput','on','SaveTime','on');
        so = sim(mdl,'StopTime',num2str(Tend));

        qL = getws('qL_log',so); qR = getws('qR_log',so);
        wL = getws('wL_log',so); wR = getws('wR_log',so);

        fprintf('  max|q_L| = %.6f rad      max|q_R| = %.6f rad\n', ...
            max(abs(qL.Data)), max(abs(qR.Data)));
        fprintf('  max|w_L| = %.6f rad/s    max|w_R| = %.6f rad/s\n', ...
            max(abs(wL.Data)), max(abs(wR.Data)));
        if max([max(abs(qL.Data)) max(abs(qR.Data))]) > 1.5
            fprintf('  注: |q| 超过 1.5 rad -> 限位已关闭（纯动力学对照配置）\n');
        end

        if p.plot
            fh = figure('Name','exo_real : hip joint response','NumberTitle','off');
            if ~hasDesktop, set(fh,'Visible','off'); end
            tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

            nexttile;
            plot(qL.Time,qL.Data,'b-','LineWidth',1.2); hold on;
            plot(qR.Time,qR.Data,'r-','LineWidth',1.2);
            grid on; ylabel('q [rad]'); legend('hip\_L','hip\_R','Location','best');
            title(sprintf('关节角（T = %.2f N·m @ %.2f Hz）', Tamp, f),'Interpreter','none');

            nexttile;
            plot(wL.Time,wL.Data,'b-','LineWidth',1.2); hold on;
            plot(wR.Time,wR.Data,'r-','LineWidth',1.2);
            grid on; xlabel('t [s]'); ylabel('\omega [rad/s]');
            legend('hip\_L','hip\_R','Location','best');

            png = fullfile(outd,'view_exo_real_response.png');
            try
                exportgraphics(fh, png, 'Resolution', 120);
                fprintf('  图 -> %s\n', png);
            catch
            end
        end

        % ---------- 3b. 可选：把 Mechanics Explorer 的 3D 录成 MP4 ----------
        % smwritevideo 要求「仿真已跑完 + 可视化结果在 Mechanics Explorer 标签页里」，
        % 所以只能在桌面模式下用；-batch 会失败（那时请用 anim_exo_3d 离线渲染）。
        if p.video
            mp4 = fullfile(outd,['view_exo_real_' p.model '.mp4']);
            try
                smwritevideo(mdl, mp4, 'VideoFormat','mpeg-4', 'FrameRate',30, ...
                             'PlaybackSpeedRatio',1, 'FrameSize','auto');
                fprintf('  视频 -> %s\n', mp4);
            catch ME
                fprintf('  !! 录制失败：%s\n', ME.message);
                fprintf('     smwritevideo 需要桌面 MATLAB + Mechanics Explorer 已打开；\n');
                fprintf('     离线出片请用 anim_exo_3d(''preset'',''drop'')\n');
            end
        end
    else
        fprintf('（import 模式只打开模型，不跑仿真。要看结构请点开 base / leg_L / leg_R）\n');
    end

    % ---------- 4. 可选：重跑一致性对照 ----------
    if p.compare
        fprintf('\n-- 重跑 compare_simscape_vs_analytical --\n');
        out = compare_simscape_vs_analytical();
        fprintf('verdict = %s，worst rms/range = %.3e\n', out.verdict, out.worst);
        if hasDesktop
            try, winopen(out.png); catch, end
        end
    end

    fprintf('\n查看现有对照图：\n  winopen(''%s'')\n', ...
        fullfile(outd,'compare_simscape_vs_analytical.png'));
    fprintf('===== view_exo_real 结束 =====\n\n');

    % -batch 退出前把（被 set_param 标脏的）模型关掉，免得 MATLAB 报
    % "无法关闭模型 'xxx'，因为它已被修改"。GUI 模式**不要**关，留给用户看。
    if ~hasDesktop
        try, close_system(mdl,0); catch, end
    end

    info = struct('model',mdl,'slx',slx,'outdir',outd);
end

% ---------------------------------------------------------------- helpers
function print_config(mdl, simd)
    fprintf('-- 机构配置 --\n');
    mc = find_system(mdl,'SearchDepth',1,'MaskType','Mechanism Configuration');
    for i = 1:numel(mc)
        try
            fprintf('  %s\n    GravityVector = %s\n', mc{i}, ...
                get_param(mc{i},'GravityVector'));
        catch ME
            fprintf('  %s : %s\n', mc{i}, ME.message);
        end
    end

    fprintf('-- 关节 --\n');
    jn = find_system(mdl,'SearchDepth',1,'MaskType','Revolute Joint');
    for i = 1:numel(jn)
        fprintf('  %s\n', jn{i});
        for prm = {'TorqueActuationMode','MotionActuationMode','SensePosition', ...
                   'SenseVelocity','LowerLimitSpecify','UpperLimitSpecify', ...
                   'DampingCoefficient'}
            try
                fprintf('      %-22s = %s\n', prm{1}, num2str(get_param(jn{i},prm{1})));
            catch
            end
        end
    end

    % 注意（实测踩过）：smimport 把质心偏移放在独立的 **InertiaOriginTransform**
    % （Rigid Transform）块里，参数名是 **TranslationCartesianOffset**；
    % Inertia 块自身的 CenterOfMass 恒为 [0 0 0] —— 读它会得到误导性的 0。
    fprintf('-- 刚体 --\n');
    for nm = {'base','leg_L','leg_R'}
        b = [mdl '/' nm{1}];
        if isempty(find_system(mdl,'SearchDepth',1,'Name',nm{1})), continue; end
        mstr = ''; cstr = ''; imom = '';
        try, mstr = num2str(get_param([b '/Inertia'],'Mass')); catch, end
        try
            cstr = num2str(get_param([b '/InertiaOriginTransform'], ...
                                     'TranslationCartesianOffset'));
        catch
        end
        try
            imom = num2str(get_param([b '/Inertia'],'MomentsOfInertia'));
        catch
        end
        fprintf('  %-8s m = %s kg\n', nm{1}, mstr);
        fprintf('           CoM offset(相对关节原点) = %s  [m]\n', cstr);
        fprintf('           MomentsOfInertia [Ixx Iyy Izz] = %s  [kg*m^2]\n', imom);
    end

    pp = fullfile(simd,'plant_params.csv');
    if ~isempty(dir(pp))
        fprintf('-- 解析对照参数（plant_params.csv）--\n');
        raw = readmatrix(pp,'NumHeaderLines',1);
        legNames = {'leg_L','leg_R'};
        for i = 1:size(raw,1)
            nm = legNames{min(i,numel(legNames))};
            fprintf('  %-6s: m=%.6f  I_axis=%.9f  A=%+.6f  B=%+.6f  |tau_g|=%.6f N*m\n', ...
                nm, raw(i,1), raw(i,2), raw(i,3), raw(i,4), raw(i,5));
        end
    end
    fprintf('\n');
end

function p = parse_args(varargin)
    p = struct('model','harness','Tamp',0.15,'f',0.30,'Tend',1.0, ...
               'plot',true,'compare',false,'video',false);
    if mod(numel(varargin),2) ~= 0
        error('view_exo_real:args','参数必须成对出现：view_exo_real(''Tamp'',0.15)');
    end
    for i = 1:2:numel(varargin)
        k = varargin{i};
        if ~isfield(p,k)
            error('view_exo_real:args','未知参数 ''%s''', k);
        end
        p.(k) = varargin{i+1};
    end
end

function ts = getws(name, so)
    ts = [];
    try
        if ~isempty(so) && isprop(so,name) && ~isempty(so.(name))
            ts = so.(name);
        end
    catch
    end
    if isempty(ts) && evalin('base',['exist(''' name ''',''var'')'])
        ts = evalin('base',name);
    end
    if isempty(ts)
        error('view_exo_real:noLog','未找到记录信号 %s', name);
    end
    if isa(ts,'timeseries'), return, end
    if isstruct(ts) && isfield(ts,'time') && isfield(ts,'signals')
        ts = timeseries(ts.signals.values, ts.time);
    end
end

function s = ternary(tf,a,b), if tf, s = a; else, s = b; end, end
