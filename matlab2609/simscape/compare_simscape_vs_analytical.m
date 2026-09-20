function out = compare_simscape_vs_analytical()
%COMPARE_SIMSCAPE_VS_ANALYTICAL
%   Independent cross-check of the CAD -> Simscape Multibody plant against an
%   analytical model, driven by the SAME torques from the SAME state.
%
%   Plant parameters are NOT hardcoded: they are read from
%   simscape/plant_params.csv, which _stp2plant.py derives from exo_real.urdf.
%   So this script can never drift away from the CAD.
%
%   *** 为什么重力矩不是 -G*sin(q) ***
%   URDF/Simscape 的关节零位 = CAD 建模时的位姿，不是「腿自然下垂」。
%   记 a = 关节轴单位向量（世界系），c = 质心相对关节原点的偏移（世界系），
%   r3 = 世界 z 单位向量。绕轴转 q 后质心高度的**完整**仿射展开为
%       z_com(q) = C0 + W cos q + V sin q
%           C0 = (a·r3)(a·c)                 <- 仿射项，对力矩无贡献
%           W  = r3·c - (a·r3)(a·c)
%           V  = r3·(a x c)
%   =>  tau_g(q) = -dU/dq = A sin q + B cos q
%           A = m*g*W ,   B = -m*g*V ,   |tau_g| = m*g*d*|r3_perp|
%   本装配体实测 a ≈ [0,-1,0]（髋屈伸轴沿 Y）、d = 0.18544 m，
%   B = -0.4383 N*m，**远不是 0** —— 静止位姿下本来就挂着 0.44 N*m 的重力矩。
%   若仍拿教科书形式 M*qdd = T - G_amp*sin(q) 去对照，会得到
%   rms/range = 2.17 的「假性失配」——看着像模型全错，其实就是零位相位问题。
%   （W,V 在绕关节轴旋转坐标系时不变，所以 B=0 做不到，
%     除非腿质心恰好落在过轴的竖直面内。）
%
%   另注：link frame 恒与世界系对齐（URDF 里 joint origin rpy = 0 0 0，
%   轴靠 <axis xyz> 显式给出），所以这里不需要任何欧拉角约定假设，
%   也不该再用「r3 = R_link 第 3 行」那种依赖 rpy 约定的写法。
%   见 _stp2urdf.py 顶部的「设计决定」注释。
%
%   Writes simscape/compare_log.txt and out_simscape/compare_simscape_vs_analytical.png

here = fileparts(fileparts(mfilename('fullpath')));       % .../matlab2609
simd = fullfile(here,'simscape');
outd = fullfile(here,'out_simscape');
if exist(outd,'dir') ~= 7, mkdir(outd); end
mdl  = 'sm_exo_real_harness';
if ~isempty(getenv('EXO2609_MODEL')), mdl = [getenv('EXO2609_MODEL') '_harness']; end
slx  = fullfile(simd,[mdl '.slx']);
logf = fullfile(simd,'compare_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== compare_simscape_vs_analytical ==\n%s\n\n', datestr(now));

% ---------------------------------------------------- 1. plant from the URDF
pp = fullfile(simd,'plant_params.csv');            % 纯数值，由 _stp2plant.py 生成
if isempty(dir(pp))
    error('compare:noParams','%s missing - run _stp2plant.py first', pp);
end
raw = readmatrix(pp,'NumHeaderLines',1);
if size(raw,1) < 2
    error('compare:badParams','expected 2 rows (hip_L, hip_R) in %s', pp);
end
M_ax = raw(1:2,2);        % I_axis  per leg   [kg*m^2]
Ac   = raw(1:2,3);        % A  coefficient    [N*m]
Bc   = raw(1:2,4);        % B  coefficient    [N*m]
ampG = raw(1:2,5);
fprintf(fid,'plant from plant_params.csv   (row order = hip_L, hip_R)\n');
fprintf(fid,'  M_axis = [%.9f  %.9f] kg*m^2\n', M_ax(1), M_ax(2));
fprintf(fid,'  tau_g  = A sin q + B cos q\n');
fprintf(fid,'           A = [%+.9f  %+.9f]\n', Ac(1), Ac(2));
fprintf(fid,'           B = [%+.9f  %+.9f] N*m\n', Bc(1), Bc(2));
fprintf(fid,'  |tau_g| amplitude = [%.6f  %.6f] N*m\n', ampG(1), ampG(2));
fprintf(fid,'  B/amplitude = [%.4f  %.4f]  -> 相位偏移不可忽略\n\n', ...
        Bc(1)/ampG(1), Bc(2)/ampG(2));

% ------------------------------------------------------- 2. scenarios
% 注意 Tfun 的**形状约定**：必须对任意形状的 t 返回 2 x numel(t)（每行一个关节）。
% 踩过的坑：写成 @(t) [a(t); b(t)]，当 t 是**列向量**时 `[x;y]` 是竖向拼接，
% 返回的是 2N x 1 而不是 2 x N；于是 Ta(1,:) 只剩一个元素，
% From Workspace 收到「常数 0 的 timeseries」-> 驱动力矩**静默变 0**，
% scenario 2 的 Simscape 轨迹与 scenario 1 逐位相同（max|q| 都是 1.8230）。
% 所以这里统一用 t(:).' 强制行向量，并在调用处加形状断言。
sc = struct('name',{},'Tfun',{},'Tend',{});
sc(1).name = 'free swing (T = 0) - isolates the gravity term';
sc(1).Tfun = @(t) zeros(2, numel(t));
sc(1).Tend = 1.0;
sc(2).name = 'forced swing 0.15 N*m @ 0.3 Hz - stresses M and gravity together';
sc(2).Tfun = @(t) [  0.15*sin(2*pi*0.3*t(:).') ; ...
                    -0.15*sin(2*pi*0.3*t(:).') ];
sc(2).Tend = 1.0;

if bdIsLoaded(mdl), close_system(mdl,0); end
if isempty(dir(slx))
    error('compare:noSlx','Run build_harness_exo2dof() first (%s missing)', slx);
end
load_system(slx);
fprintf(fid,'simscape : %s   solver=%s\n\n', mdl, get_param(mdl,'Solver'));

names = {'q_L','q_R','qd_L','qd_R'};
worstAll = 0; worstTag = '';
YaAll = cell(1,numel(sc)); YsAll = cell(1,numel(sc)); ttAll = cell(1,numel(sc));

for i = 1:numel(sc)
    Tfun = sc(i).Tfun;  Tend = sc(i).Tend;  tt = (0:1e-3:Tend).';

    % ---- analytical: exact integration of the SAME ode ----
    odef = @(t,y) plant_ode(t, y, M_ax, Ac, Bc, Tfun);
    sol  = ode45(odef, [0 Tend], [0;0;0;0], odeset('RelTol',1e-10,'AbsTol',1e-12));
    Ya   = deval(sol, tt).';

    % ---- simscape ----
    taus = (0:1e-4:Tend).';
    Ta   = Tfun(taus);
    assert(size(Ta,1) == 2 && size(Ta,2) == numel(taus), ...
        'compare:badTfun', ...
        ['Tfun 必须返回 2 x numel(t)，实际是 %d x %d。' ...
         '若 t 是列向量而 Tfun 用 [a; b] 拼接，会静默得到 2N x 1，' ...
         '驱动力矩随之变成 0（本文件曾因此误报 FAIL）。'], size(Ta,1), size(Ta,2));
    assignin('base','T_L_data', timeseries(Ta(1,:).', taus));
    assignin('base','T_R_data', timeseries(Ta(2,:).', taus));
    so = sim(mdl,'StopTime',num2str(Tend));

    qLs = getws('qL_log',so); wLs = getws('wL_log',so);
    qRs = getws('qR_log',so); wRs = getws('wR_log',so);
    Ys  = [ interp1(qLs.Time,qLs.Data,tt,'linear'), ...
            interp1(qRs.Time,qRs.Data,tt,'linear'), ...
            interp1(wLs.Time,wLs.Data,tt,'linear'), ...
            interp1(wRs.Time,wRs.Data,tt,'linear') ];

    fprintf(fid,'-- scenario %d : %s --\n', i, sc(i).name);
    fprintf(fid,'   %-6s %12s %12s %12s %11s\n','sig','maxabs','rms','range','rms/range');
    for k = 1:4
        e  = Ys(:,k) - Ya(:,k);
        rg = max(Ya(:,k)) - min(Ya(:,k));
        ma = max(abs(e)); rms_e = sqrt(mean(e.^2)); rel = rms_e/max(rg,eps);
        fprintf(fid,'   %-6s %12.3e %12.3e %12.3e %11.3e\n', names{k}, ma, rms_e, rg, rel);
        if rel > worstAll, worstAll = rel; worstTag = sprintf('sc%d/%s', i, names{k}); end
    end
    qmA = max(abs(Ya(:,1:2)),[],'all');  qmS = max(abs(Ys(:,1:2)),[],'all');
    fprintf(fid,'   max|q| : analytical %.4f rad, simscape %.4f rad\n', qmA, qmS);
    if max(qmA,qmS) > 1.5
        fprintf(fid,'   note   : |q| > 1.5 rad -> harness 已关关节限位，不影响对照\n');
    end
    fprintf(fid,'\n');

    YaAll{i} = Ya; YsAll{i} = Ys; ttAll{i} = tt;
end

if worstAll < 1e-2,        verdict = 'OK';
elseif worstAll < 5e-2,    verdict = 'MARGINAL';
else,                      verdict = 'FAIL';
end
fprintf(fid,'worst channel overall: %s   rms/range = %.3e\n', worstTag, worstAll);
fprintf(fid,'verdict: %s  (threshold 1e-2)\n', verdict);

% -------------------------------------------------------------- figure
fh = figure('Visible','off','Position',[100 100 1050 700]);
for i = 1:numel(sc)
    subplot(numel(sc),2,2*i-1);
    plot(ttAll{i},YaAll{i}(:,1),'k-','LineWidth',1.6); hold on;
    plot(ttAll{i},YsAll{i}(:,1),'r--','LineWidth',1.2);
    grid on; xlabel('t [s]'); ylabel('q_L [rad]');
    legend('analytical','simscape','Location','best');
    title(sprintf('scenario %d: %s', i, sc(i).name),'Interpreter','none');
    subplot(numel(sc),2,2*i);
    plot(ttAll{i},YsAll{i}(:,1)-YaAll{i}(:,1),'b-'); hold on;
    plot(ttAll{i},YsAll{i}(:,2)-YaAll{i}(:,2),'m-');
    grid on; xlabel('t [s]'); ylabel('error [rad]');
    legend('q_L','q_R','Location','best'); title('mismatch');
end
pngf = fullfile(outd,'compare_simscape_vs_analytical.png');
print(fh,'-dpng','-r120',pngf); close(fh);
fprintf(fid,'\nfigure: %s\n', pngf);
fprintf(fid,'\nCOMPARE_%s\n', verdict);

out = struct('verdict',verdict,'worst',worstAll,'worstTag',worstTag, ...
             'png',pngf,'log',logf,'M_axis',M_ax,'A',Ac,'B',Bc);
end

% ---------------------------------------------------------------- helpers
function yd = plant_ode(t, y, M_ax, Ac, Bc, Tfun)
nq = numel(M_ax);
q  = y(1:nq);  qd = y(nq+1:end);
Tg = Ac.*sin(q) + Bc.*cos(q);      % 广义重力力矩 = -dU/dq（含相位，不是 -G*sin q）
T  = Tfun(t);
% 拉格朗日：I_axis*qdd = T_applied + tau_g —— 两项都是广义力，**同为加号**。
% 实测（自由摆动，T=0）：B = -0.4383 N*m < 0
%   -> qdd(0) = B/I = -25.82 rad/s^2，腿往 -q 方向摆，
%      转折点 q = -1.8227 rad，与 Simscape 实测 max|q| = 1.8231 一致。
% 若误写成 (T - Tg)，轨迹整体镜像到 +4.4604 rad (= 2*pi - 1.8227)，
%   max|q| 对不上、rms/range ~0.95 —— 看着像 CAD 模型错，其实是这一处符号错。
yd = [qd; (T + Tg)./M_ax];
end

function ts = getws(name, so)
ts = [];
try
    if ~isempty(so) && isprop(so,name) && ~isempty(so.(name))
        ts = so.(name);
    end
catch
end
if isempty(ts)
    if evalin('base',['exist(''' name ''',''var'')'])
        ts = evalin('base',name);
    end
end
if isempty(ts)
    error('compare:noLog','logged signal %s not found', name);
end
if isa(ts,'timeseries'), return, end
if isstruct(ts) && isfield(ts,'time') && isfield(ts,'signals')
    ts = timeseries(ts.signals.values, ts.time);
end
end
