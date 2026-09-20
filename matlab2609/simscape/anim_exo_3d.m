function info = anim_exo_3d(varargin)
%ANIM_EXO_3D  把真实装配体（base + 左右腿）连 STL 外形渲成动画：MP4 / GIF / 三联姿态图。
%
%   为什么需要它
%   ------------
%   Mechanics Explorer 能看 3D，但它依赖 GUI，相机/配色/导出都不好复现。
%   本脚本走**离线渲染**：直接读 meshes/*.stl + 仿真出来的 q(t)，
%   用刚体变换把两条腿摆对位置，逐帧渲染。于是：
%     - 无 GUI 也能出片（-batch 可跑，远程/CI 都行）
%     - 视频可复现、可分享、可进报告
%     - 相机可选「沿髋轴看」= 钟摆正视图，摆角一眼可读
%
%   用法
%   ----
%     cd C:\Users\29408\exo_work\matlab2609\simscape ; addpath(pwd)
%
%     anim_exo_3d                            % 默认 preset='drop'（自由落摆，2.5 s）
%     anim_exo_3d('preset','drive')          % 受驱往复 ±0.15 N*m @ 0.3 Hz，4 s
%     anim_exo_3d('preset','gait')           % 左右反相摆动 ±0.20 N*m @ 0.4 Hz，6 s
%
%   ⚠ 关于摆幅，务必知道的两件事
%   ---------------------------
%   1) CAD 零位不是重力平衡位。髋轴水平（沿世界 Y），腿质心到轴 0.1854 m，
%      URDF 给的重力矩是 tau_g(q) = A sin q + B cos q，A=-0.3397、B=-0.4383 N*m。
%      即「零位本来挂着 0.438 N*m 的重力力矩」——所以哪怕 T=0（preset 'drop'），
%      腿也会自由落到 q=-1.82 rad。这是模型的真实行为，不是 bug。
%   2) harness 里关节限位是**关**的（LowerLimitSpecify=off），且没有阻尼。
%      摆幅一大、或者驱动频率靠近共振（f_n = sqrt(|A|/I_axis)/2pi ≈ 0.71 Hz），
%      腿会翻过顶、甚至连续转圈（实测 f=0.5 Hz / 0.35 N*m 时 max|q| 冲到 14.6 rad）。
%      内置 preset 已按「不翻转」筛过：drop 1.82 rad、drive 2.12 rad、gait 2.47 rad。
%      想要 ±0.5 rad 那种真实步态摆幅，需要**闭环重力补偿**
%      T = -(A sin q + B cos q) + T_swing，而本 harness 的 T 来自
%      From Workspace（开环），做不到——要么改模型接线，要么离线用本脚本的
%      'source','analytic' 自己造 T(t)（提示：常量偏置补偿**不行**，
%      定值力矩与周期势能叠加会把 q≈0 变成不稳定平衡点，腿会一直向一个方向滚走）。
%
%     anim_exo_3d('Tend',1.2,'Tamp',0.3,'f',0.6,'fps',30)
%     anim_exo_3d('view','iso')              % 等轴测视角
%     anim_exo_3d('source','analytic')       % 不跑 Simscape，用 plant_params.csv 解析积分
%     anim_exo_3d('formats',{'mp4','gif'},'reduce',0.25)
%
%   参数（名值对）
%   -------------
%     preset   'drop' | 'drive' | 'gait' | 'custom'      默认 'drop'
%     Tamp/f/Tend   力矩幅值[N*m]/频率[Hz]/时长[s]（给了就覆盖 preset 的时序）
%     fps      帧率，默认 25
%     source   'simscape'（默认，真跑 harness）| 'analytic'（ode45 积分同一 ODE）
%     view     'side'（默认，沿髋轴看=钟摆视角）| 'iso' | 'front' | 'top'
%     reduce   网格抽稀比例，默认 1.0（不抽稀）。0.25 明显更快，外形基本不变
%     formats  输出格式 cell，默认 {'mp4'}，可含 'gif'
%     pose     true/false 出三联姿态 PNG，默认 true
%
%   输出目录 matlab2609/out_simscape/
%     exo_real_<tag>.mp4 / .gif       动画
%     exo_real_<tag>_poses.png        t=起点/中点/终点 三联姿态
%     anim_exo_3d_<tag>.txt           日志（含每帧耗时、姿态一致性校核）
%   tag 规则：tag = preset；若 view 不是默认的 'side' 就追加 '_<view>'。
%     例：anim_exo_3d('preset','gait','view','iso') -> exo_real_gait_iso.mp4
%     （各视角互不覆盖，也不会盖掉已出的侧视成品 exo_real_gait.mp4）
%
%   自检（重要）
%   -----------
%   渲染用的旋转方向不能靠猜。脚本把腿质心按候选符号 ±q 各转一次，算出世界系
%   质心高度，再跟 URDF 解析式 z_com(q) = C0 + W cos q + V sin q 比对，取误差
%   小的那个符号，并把残差写进日志（应 ~1e-16 m）。残差超 1e-9 m 直接报错、
%   拒绝出片。这条自检同时钉死「STL 世界顶点 + 关节轴 + 转动符号」三者对齐。
%
%   见 SIMSCAPE_BRIDGE_SOP.md / MATERIAL_AND_MASS_RECON.md

    p    = parse_args(varargin{:});
    here = fileparts(fileparts(mfilename('fullpath')));   % .../matlab2609
    simd = fullfile(here,'simscape');
    outd = fullfile(here,'out_simscape');
    if isempty(dir(outd)), mkdir(outd); end

    % 相对网格路径 meshes/*.stl 是相对「当前目录」解析的，先切进去
    old = cd(simd);
    c   = onCleanup(@() cd(old));

    % 文件名标签：默认视角(side)沿用 exo_real_<preset>.* 不变（保持旧文档有效）；
    % 换了视角就加后缀，免得 iso/front/top 互相覆盖、也免得覆盖掉已出的侧视成品。
    tag  = p.preset;
    if ~strcmpi(p.view,'side')
        tag = [tag '_' lower(p.view)];
    end
    logf = fullfile(outd,['anim_exo_3d_' tag '.txt']);
    fid  = fopen(logf,'w');
    cl   = onCleanup(@() fclose(fid));

    fprintf(fid,'== anim_exo_3d ==\n%s\n', datestr(now));
    fprintf(fid,'preset=%s  source=%s  view=%s  fps=%d  reduce=%.2f\n', ...
        tag, p.source, p.view, p.fps, p.reduce);
    fprintf(fid,'outdir=%s\n', outd);
    if ~usejava('desktop')
        fprintf(fid,'note: 无桌面(-batch) -> 全部离屏渲染，不经 Mechanics Explorer\n');
    end
    fprintf(fid,'\n');

    % ------------------------------------------------ 1. 时序 T(t) 与 q(t)
    [tt, qL, qR, TL, TR, srcName] = get_motion(p, simd, fid);
    fprintf(fid,'motion : %s\n', srcName);
    fprintf(fid,'  N=%d   t in [%.4f %.4f] s\n', numel(tt), tt(1), tt(end));
    fprintf(fid,'  max|q_L| = %.6f rad    max|q_R| = %.6f rad\n', max(abs(qL)), max(abs(qR)));
    fprintf(fid,'  max|w_L| = %.6f rad/s  max|w_R| = %.6f rad/s\n\n', ...
        max(abs(gradient(qL,tt))), max(abs(gradient(qR,tt))));

    % ------------------------------------------------ 2. URDF 几何/关节
    [o, ax, meshNames] = read_urdf_kinematics(fullfile(simd,'exo_real.urdf'));
    fprintf(fid,'-- hip joint (world frame, m) --\n');
    fprintf(fid,'  origin = [%.9f %.9f %.9f]\n', o);
    fprintf(fid,'  axis   = [%.9f %.9f %.9f]   norm %.12f\n', ax, norm(ax));
    fprintf(fid,'  meshes = %s\n\n', strjoin(meshNames,', '));

    % ------------------------------------------------ 3. 读网格
    fprintf(fid,'-- meshes --\n');
    G = struct();
    for nm = meshNames
        f = fullfile(simd,'meshes',[nm{1} '.stl']);
        t0 = tic; TR = stlread(f);
        V = TR.Points; F = double(TR.ConnectivityList); nTri0 = size(F,1);
        if p.reduce < 1
            T2 = reducepatch(TR, p.reduce);
            if isa(T2,'triangulation'), V = T2.Points; F = double(T2.ConnectivityList);
            else,                        V = T2.vertices; F = double(T2.faces); end
        end
        G.(nm{1}) = struct('V',V,'F',F,'nTri0',nTri0);
        fprintf(fid,'  %-6s tris %7d -> %7d   verts %7d   read+dec %.1f s\n', ...
            nm{1}, nTri0, size(F,1), size(V,1), toc(t0));
    end
    fprintf(fid,'\n');

    % ------------------------------------------------ 4. 转动符号自检
    % 注意量纲口径：URDF 里 leg_L 的质心是「相对关节原点」的偏移（世界系分量），
    % 而解析式 z_com(q) = C0 + W cos q + V sin q 给的是**同一个相对偏移**的 z 分量。
    % 所以两边都取相对量，不要一边加关节原点、一边不加（踩过：混用会得到恒定的
    % |e| = |o_z| 残差，看着像几何不一致，其实是口径错）。
    comL = [0.146909793 0.161426527 -0.112150864];   % URDF leg_L 质心(相对关节原点)
    cC0  = +0.001455481; cW = -0.113606345; cV = +0.146560131;
    qs   = [0.7 -1.2];
    resid = inf; sgn = +1;
    for si = 1:2
        s = 3 - 2*si;                                % si=1 -> +1, si=2 -> -1
        e = 0; detail = '';
        for q = qs
            zrel  = rotv(comL, ax, s*q);  zrel = zrel(3);   % 渲染：R(s*q)*com 的 z
            zdes  = cC0 + cW*cos(q) + cV*sin(q);            % 解析
            e = max(e, abs(zrel - zdes));
            detail = [detail sprintf(' q=%+.1f: render %+.9f vs analytic %+.9f', ...
                q, zrel, zdes)];                             %#ok<AGROW>
        end
        fprintf(fid,'  sign trial s=%+d :%s   |e|max = %.3e m\n', s, detail, e);
        if e < resid, resid = e; sgn = s; end
    end
    fprintf(fid,'  => 采用 s = %+d   residual = %.3e m\n', sgn, resid);
    if resid > 1e-9
        fprintf(fid,'  SGN_CHECK_FAIL 残差过大：STL / 关节轴 / URDF 不自洽，拒绝出片\n');
        error('anim_exo_3d:sign', ...
            '转动符号自检残差 %.3e m 过大（阈值 1e-9）。详见 %s', resid, logf);
    end
    fprintf(fid,'  SGN_CHECK_OK\n\n');

    % ------------------------------------------------ 5. 画布 / 相机
    fh = figure('Color',p.bg,'Position',[80 60 p.W p.H],'Visible','off');
    if usejava('desktop'), set(fh,'Visible','on'); end
    axh = axes('Parent',fh,'Position',[0 0 1 1]); hold(axh,'on');
    set(axh,'Color',p.bg,'XColor','none','YColor','none','ZColor','none');

    LIGHT = struct('FaceLighting','flat','EdgeColor','none','FaceAlpha',1, ...
                   'AmbientStrength',0.55,'DiffuseStrength',0.70);
    patch('Parent',axh,'Faces',G.base.F,'Vertices',G.base.V, ...
        'FaceColor',p.colBase, LIGHT);
    tfL = hgtransform('Parent',axh);  tfR = hgtransform('Parent',axh);
    hL = patch('Parent',axh,'Faces',G.leg_L.F,'Vertices',G.leg_L.V,'FaceColor',p.colL, LIGHT);
    hR = patch('Parent',axh,'Faces',G.leg_R.F,'Vertices',G.leg_R.V,'FaceColor',p.colR, LIGHT);
    set(hL,'Parent',tfL); set(hR,'Parent',tfR);
    camlight(axh,'headlight');
    light('Parent',axh,'Position',[0.6 1.0 0.8],'Style','infinite');

    % 包围盒：**必须含底座**，否则底座顶部会被切掉（踩过：只扫了腿，
    % z 上限取到 0.008 而底座实际到 0.062，画面上看不到腰架）。
    % 扫掠角也**必须细采样**：只取 [min(q) max(q)] 两个端点会漏掉「腿正好
    % 竖直朝下」的中间位姿，最小值被低估 -> 腿尖切出画面外（踩过）。
    qsw = linspace(min([min(qL) min(qR)]), max([max(qL) max(qR)]), 61);
    VsL = G.leg_L.V(1:7:end,:);  VsR = G.leg_R.V(1:7:end,:);
    Vs = [ G.base.V ; VsL ; rotvSweep(VsL, ax, o, qsw) ; ...
                      VsR ; rotvSweep(VsR, ax, o, qsw) ; o ];
    lo = min(Vs,[],1); hi = max(Vs,[],1);
    pad = 0.04*max(hi-lo); lo = lo - pad; hi = hi + pad;

    % 铺满且**不裁切**的正确配法：daspect=[1 1 1] 保证等比，
    % pbaspect=包围盒边长比 保证箱体比例跟数据一致 -> MATLAB 自动 letterbox 进 axes。
    % （踩过：`axis equal` + `axis vis3d` 在长条形 axes 里会把箱体放大填满，
    %   摆幅大的帧直接把腿切出画面外。）
    dbox = [hi(1)-lo(1), hi(2)-lo(2), hi(3)-lo(3)];
    daspect(axh,[1 1 1]); pbaspect(axh, dbox/max(dbox));
    xlim(axh,[lo(1) hi(1)]); ylim(axh,[lo(2) hi(2)]); zlim(axh,[lo(3) hi(3)]);
    switch lower(p.view)
        case 'side',  view(axh,0,0);        % 沿 -Y 看：钟摆正视（推荐）
        case 'iso',   view(axh,-37.5,22);
        case 'front', view(axh,90,0);
        case 'top',   view(axh,0,90);
        otherwise,    error('anim_exo_3d:view','view 只能是 side/iso/front/top');
    end
    axis(axh,'off');
    fprintf(fid,'bbox x[%.4f %.4f]  y[%.4f %.4f]  z[%.4f %.4f] m   view=%s\n\n', ...
        lo(1), hi(1), lo(2), hi(2), lo(3), hi(3), p.view);

    % ------------------------------------------------ 6. 逐帧渲染
    tLbl = title(axh,'','FontName','Consolas','FontSize',12,'Color',p.fg);
    tHud = text(axh,0.014,0.035,'','Units','normalized','FontName','Consolas', ...
        'FontSize',9,'Color',p.fg,'HorizontalAlignment','left','VerticalAlignment','bottom');

    % 帧时刻直接在 [t0,tend] 上等分，**不要 round()**：
    % linspace 的末点常正好落在 x.5 上，round(2.5)=3 > tend=2.5，
    % interp1 外插返回 NaN -> hgtransform 的 Matrix 变成 NaN，
    % 报「Matrix 属性的值无效」（踩过，很容易误判成矩阵构造写错）。
    nF  = max(2, round((tt(end)-tt(1))*p.fps)+1);
    tw  = linspace(tt(1),tt(end),nF);
    qLi = interp1(tt,qL,tw,'linear');  qRi = interp1(tt,qR,tw,'linear');
    if any(~isfinite(qLi)) || any(~isfinite(qRi))
        error('anim_exo_3d:nanQ','插值出非有限角度，帧时刻越界：t in [%g %g], t_end=%g', ...
            min(tw), max(tw), tt(end));
    end
    if p.Tamp == 0
        hudT = 'free swing : T_L = T_R = 0 N*m';
    else
        hudT = sprintf('T_L = %+0.3f*sin(2*pi*%g*t) N*m   T_R = -T_L', p.Tamp, p.f);
    end
    hudG = sprintf(['preset %s   |   %s\n' ...
        'plant from exo_real.urdf (real CAD)   |   d_cg %.4f m   I_axis %.6f kg*m^2'], ...
        tag, hudT, p.lcg, p.Iax);
    set(tHud,'String',hudG);

    base = fullfile(outd,['exo_real_' tag]);
    vw = [];
    if any(strcmpi(p.formats,'mp4'))
        vw = VideoWriter([base '.mp4'],'MPEG-4');
        vw.FrameRate = p.fps; vw.Quality = 92;
        open(vw);
        cv = onCleanup(@() close(vw));
    end
    gifOn = any(strcmpi(p.formats,'gif'));

    drawnow;
    fms = zeros(numel(tw),1);
    for k = 1:numel(tw)
        t0 = tic;
        set(tfL,'Matrix',hinge_matrix(ax,o,sgn*qLi(k)));
        set(tfR,'Matrix',hinge_matrix(ax,o,sgn*qRi(k)));
        set(tLbl,'String',sprintf('t = %5.2f s     q_L = %+6.3f rad     q_R = %+6.3f rad', ...
            tw(k), qLi(k), qRi(k)));
        drawnow;
        F = getframe(fh);
        if ~isempty(vw), writeVideo(vw, F); end
        if gifOn
            [A,map] = rgb2ind(F.cdata,200,'nodither');
            if k == 1
                imwrite(A,map,[base '.gif'],'gif','LoopCount',Inf,'DelayTime',1/p.fps);
            else
                imwrite(A,map,[base '.gif'],'gif','WriteMode','append','DelayTime',1/p.fps);
            end
        end
        fms(k) = toc(t0)*1000;
        if mod(k,20)==0 || k==numel(tw)
            fprintf(fid,'  frame %3d/%3d   %.0f ms\n', k, numel(tw), fms(k));
        end
    end
    if ~isempty(vw), close(vw); end
    fprintf(fid,'\nrender : %d frames, mean %.0f ms/frame, max %.0f ms\n', ...
        numel(tw), mean(fms), max(fms));

    % ------------------------------------------------ 7. 三联姿态图
    posePng = '';
    if p.pose
        % 三联姿态取「起点 / 摆幅极值 / 终点」——取 t 的 1/2、2/3 处经常都在
        % 小角度段，三张几乎一样，看不出摆幅（踩过）。
        [~,kmax] = max(abs(qLi));
        kk = unique([1, kmax, numel(tw)]);
        sub = figure('Color',p.bg,'Position',[80 60 round(p.W*1.7) p.H],'Visible','off');
        for i = 1:numel(kk)
            pos = [0.01 + (i-1)*(0.98/numel(kk)), 0.02, 0.96/numel(kk)-0.012, 0.90];
            a = axes('Parent',sub,'Position',pos); hold(a,'on');
            set(a,'Color',p.bg,'XColor','none','YColor','none','ZColor','none');
            patch('Parent',a,'Faces',G.base.F,'Vertices',G.base.V, ...
                'FaceColor',p.colBase, LIGHT);
            VL = (rotm(ax,sgn*qLi(kk(i)))*(G.leg_L.V - o).').' + o;
            VR = (rotm(ax,sgn*qRi(kk(i)))*(G.leg_R.V - o).').' + o;
            patch('Parent',a,'Faces',G.leg_L.F,'Vertices',VL,'FaceColor',p.colL, LIGHT);
            patch('Parent',a,'Faces',G.leg_R.F,'Vertices',VR,'FaceColor',p.colR, LIGHT);
            camlight(a,'headlight');
            daspect(a,[1 1 1]); pbaspect(a, dbox/max(dbox));
            xlim(a,[lo(1) hi(1)]); ylim(a,[lo(2) hi(2)]); zlim(a,[lo(3) hi(3)]);
            view(a,0,0); axis(a,'off');
            title(a,sprintf('t = %.2f s    q_L = %+.3f rad', tw(kk(i)), qLi(kk(i))), ...
                'FontName','Consolas','FontSize',10,'Color',p.fg);
        end
        posePng = [base '_poses.png'];
        exportgraphics(sub, posePng, 'Resolution',120, 'BackgroundColor',p.bg);
        close(sub);
        fprintf(fid,'poses  -> %s\n', posePng);
    end

    mp4 = ''; gif = '';
    if any(strcmpi(p.formats,'mp4')), mp4 = [base '.mp4']; end
    if gifOn, gif = [base '.gif']; end
    fprintf(fid,'mp4    -> %s\n', mp4);
    fprintf(fid,'gif    -> %s\n', gif);
    fprintf(fid,'\nANIM_EXO_3D_DONE\nlog -> %s\n', logf);

    info = struct('mp4',mp4,'gif',gif,'poses',posePng,'log',logf,'tag',tag, ...
        'nFrames',numel(tw),'msPerFrame',mean(fms),'sign',sgn,'residual',resid, ...
        'maxqL',max(abs(qL)),'maxqR',max(abs(qR)));
end

% ================================================================ 运动时序
function [tt,qL,qR,TL,TR,srcName] = get_motion(p, simd, fid)
    if strcmpi(p.source,'analytic')
        raw = readmatrix(fullfile(simd,'plant_params.csv'),'NumHeaderLines',1);
        M = raw(1:2,2); A = raw(1:2,3); B = raw(1:2,4);
        % Tfun 的形状约定：t 是**标量**时返回 2x1，t 是**向量**时返回 2xN。
        % 所以 t(:).' 把输入强制成行向量 —— 这样既满足 ode45 的标量调用，
        % 也满足后来给 timeseries 用的向量调用。
        Tfun = @(t) [ p.Tamp*sin(2*pi*p.f*t(:).') ; -p.Tamp*sin(2*pi*p.f*t(:).') ];
        % ⚠ 这里**不能**写 Tfun(t).'：t 标量时 Tfun(t) 已是 2x1，再转置成 1x2
        %   跟 A.*sin(y(1:2)) 的 2x1 广播相乘会得到 2x2，拼进 [y(3:4); ...] 就报
        %   「要串联的数组的维度不一致」（踩过，和 compare_simscape_vs_analytical
        %   里记的是同一类形状坑）。
        ode  = @(t,y) [ y(3:4) ; (Tfun(t) + A.*sin(y(1:2)) + B.*cos(y(1:2)))./M ];
        sol  = ode45(ode,[0 p.Tend],[0;0;0;0],odeset('RelTol',1e-10,'AbsTol',1e-12));
        tt   = (0:1/p.fps:p.Tend).';
        Y    = deval(sol,tt).';
        TT   = Tfun(tt).';
        qL = Y(:,1); qR = Y(:,2); TL = TT(:,1); TR = TT(:,2);
        srcName = 'analytic : ode45 on plant_params.csv (no Simscape, same ODE)';
        return
    end
    mdl = 'sm_exo_real_harness';
    slx = fullfile(simd,[mdl '.slx']);
    if isempty(dir(slx)), error('anim_exo_3d:noSlx','%s 不存在', slx); end
    % 已经加载就用现成的；**不要** close_system(mdl,0) 再 load——
    % 那会把用户手上没保存的改动丢掉。
    wasLoaded = bdIsLoaded(mdl);
    if ~wasLoaded, load_system(slx); end
    opened = get_param(mdl,'SimMechanicsOpenEditorOnUpdate');
    % 离线渲染不需要 3D 视窗；桌面下它会弹出来挡着。跑完必须还原，
    % 并且**我们自己加载的模型要自己关掉**——否则 set_param 把模型标脏，
    % MATLAB 退出时会刷一屏「无法关闭模型 xxx，因为它已被修改」。
    set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
    cl = onCleanup(@() restore_model(mdl, opened, wasLoaded));

    taus = (0:1e-4:p.Tend).';
    Ta   = [ p.Tamp*sin(2*pi*p.f*taus), -p.Tamp*sin(2*pi*p.f*taus) ];
    assignin('base','T_L_data', timeseries(Ta(:,1), taus));
    assignin('base','T_R_data', timeseries(Ta(:,2), taus));
    so = sim(mdl,'StopTime',num2str(p.Tend));

    qLt = getws('qL_log',so); qRt = getws('qR_log',so);
    tt  = qLt.Time(:); qL = qLt.Data(:); qR = qRt.Data(:);
    TL  = interp1(taus,Ta(:,1),tt,'linear','extrap');
    TR  = interp1(taus,Ta(:,2),tt,'linear','extrap');
    fprintf(fid,'  Simscape sim : %d samples, solver %s, StopTime %.3f s\n', ...
        numel(tt), get_param(mdl,'Solver'), p.Tend);
    srcName = sprintf('Simscape Multibody %s (real CAD plant)', mdl);
end

% ================================================================ 几何
function [o, ax, names] = read_urdf_kinematics(urdfFile)
    if isempty(dir(urdfFile)), error('anim_exo_3d:noUrdf','%s 不存在', urdfFile); end
    s  = fileread(urdfFile);
    jm = regexp(s,'<joint\s+name="hip_L".*?</joint>','match','once');
    if isempty(jm), error('anim_exo_3d:urdfParse','URDF 里找不到 hip_L'); end
    om = regexp(jm,'<origin\s+xyz="([^"]+)"','tokens','once');
    am = regexp(jm,'<axis\s+xyz="([^"]+)"','tokens','once');
    o  = sscanf(om{1},'%f').';
    ax = sscanf(am{1},'%f').'; ax = ax/norm(ax);
    mm = regexp(s,'<mesh\s+filename="([^"]+)"','tokens');
    names = cellfun(@(c) mesh_base(c{1}), mm, 'UniformOutput',false);
end
function n = mesh_base(f)
    % 只取文件名主干：调用处以 [name '.stl'] 重新拼扩展名，
    % 所以这里**不能**再带 .stl（否则拼出 'base.stl.stl'，stlread 直接报文件不存在）。
    [~,b] = fileparts(f); n = b;
end
function R = rotm(a, q)
    a = a(:)/norm(a);
    K = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
    R = eye(3) + sin(q)*K + (1-cos(q))*(K*K);        % Rodrigues
end
function V = rotv(V, a, q)
    V = (rotm(a,q)*V.').';
end
function M = hinge_matrix(a, o, q)
    R = rotm(a,q);
    M = [R, (o.' - R*o.'); 0 0 0 1];                 % 绕「过 o 点、方向 a」的轴转 q
end
function Vout = rotvSweep(V, a, o, qs)
    Vout = zeros(0,3);
    for q = qs(:).'
        Vout = [Vout; (rotm(a,q)*(V - o).').' + o];  %#ok<AGROW>
    end
end

% ================================================================ 杂项
function restore_model(mdl, opened, wasLoaded)
%RESTORE_MODEL  还原被 set_param 改过的模型参数；自己加载的模型顺手关掉。
    try, set_param(mdl,'SimMechanicsOpenEditorOnUpdate',opened); catch, end
    if ~wasLoaded
        try, close_system(mdl,0); catch, end
    end
end

function ts = getws(name, so)
    ts = [];
    try
        if ~isempty(so) && isprop(so,name) && ~isempty(so.(name)), ts = so.(name); end
    catch
    end
    if isempty(ts) && evalin('base',['exist(''' name ''',''var'')'])
        ts = evalin('base',name);
    end
    if isempty(ts), error('anim_exo_3d:noLog','未找到记录信号 %s', name); end
    if isa(ts,'timeseries'), return, end
    if isstruct(ts) && isfield(ts,'time') && isfield(ts,'signals')
        ts = timeseries(ts.signals.data, ts.time);
    end
end

function p = parse_args(varargin)
    % 两种写法都支持：
    %   位置简写   anim_exo_3d('gait','view','iso')
    %   名值对     anim_exo_3d('preset','gait','view','iso')      <- 文档推荐
    % 关键：**只有第一个参数恰好是 preset 名时**才按简写吞掉它。
    % （旧版写成 `if ischar(varargin{1})` 无条件吞掉第一个参数，
    %   于是 'preset' 被自己吃掉、'gait' 变成孤立键 -> "未知参数 'gait'"。）
    preset = 'drop';
    if ~isempty(varargin) && ischar(varargin{1}) && ...
            any(strcmpi(varargin{1}, {'drop','drive','gait','custom'}))
        preset = varargin{1}; varargin = varargin(2:end);
    end
    % ★ d_cg / I_axis 只用于 HUD 文字，但同样不该硬编码：
    %   从 plant_params.csv（_stp2plant.py 由 exo_real.urdf 派生）取。
    %   旧硬编码 0.185443 / 0.016975774 是密度表改动**之前**的值，已过期。
    [lcg0, Iax0] = leg_params_from_csv();
    p = struct('preset',preset, 'Tamp',[], 'f',[], 'Tend',[], 'fps',25, ...
               'source','simscape', 'view','side', 'reduce',1.0, ...
               'formats',{{'mp4'}}, 'pose',true, 'W',900, 'H',620, ...
               'bg',[0.95 0.95 0.96], 'fg',[0.10 0.10 0.12], ...
               'colBase',[0.62 0.65 0.70], 'colL',[0.16 0.44 0.85], 'colR',[0.88 0.36 0.10], ...
               'lcg',lcg0, 'Iax',Iax0);
    if mod(numel(varargin),2) ~= 0
        error('anim_exo_3d:args','名值参数必须成对出现，收到 %d 个参数', numel(varargin));
    end
    for i = 1:2:numel(varargin)
        k = varargin{i};
        if ~ischar(k) || ~isfield(p,k)
            error('anim_exo_3d:args','未知参数（第 %d 个）：%s', i, num2str(k));
        end
        p.(k) = varargin{i+1};
    end
    if ~ischar(p.preset)
        error('anim_exo_3d:preset','preset 必须是字符串 drop/drive/gait/custom');
    end
    switch lower(p.preset)
        case 'drop',   d = struct('Tamp',0.00, 'f',0.0, 'Tend',2.5);
        case 'drive',  d = struct('Tamp',0.15, 'f',0.3, 'Tend',4.0);
        case 'gait',   d = struct('Tamp',0.20, 'f',0.4, 'Tend',6.0);
        case 'custom', d = struct('Tamp',0.15, 'f',0.3, 'Tend',2.0);
        otherwise, error('anim_exo_3d:preset','preset 只能是 drop/drive/gait/custom');
    end
    if isempty(p.Tamp), p.Tamp = d.Tamp; end
    if isempty(p.f),    p.f    = d.f;    end
    if isempty(p.Tend), p.Tend = d.Tend; end
end

function [lcg, Iax] = leg_params_from_csv()
% 从 simscape/plant_params.csv 取 hip_L 的 I_axis 与等效垂距 d_cg。
%   d_cg 由 d = amp/(m*g) 反算（amp = m*g*sqrt(W^2+V^2)）。
%   ★ 严格说真实垂距还要除以 |r3_perp| = sqrt(1-gamma^2)，本例 gamma=-0.009056
%     -> 相对差 ~4e-5；HUD 只印 4 位小数，看不出来。要做精确值请用 exo_real_report.txt。
simd = fileparts(mfilename('fullpath'));
pp   = fullfile(simd,'plant_params.csv');
lcg  = 0.1821;  Iax = 0.017;                 % 兜底（文件缺失时）
if isempty(dir(pp)), return, end
try
    T = readtable(pp, 'NumHeaderLines', 1);  % 纯数值，无表头
    m   = T{1,1};  Iax = T{1,2};  amp = T{1,5};
    lcg = amp / (m * 9.81);
catch
end
end
