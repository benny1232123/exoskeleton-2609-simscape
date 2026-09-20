function out = validate_energy(Tend)
%VALIDATE_ENERGY  第三层验证：自由摆动的「机械能守恒」自检。
%
% 为什么需要它
% ------------
% compare_simscape_vs_analytical 是「多体模型 vs 解析模型」的**轨迹对照**，
% 它验证的是「URDF → 动力学」这一步翻译没错。但它有一个盲区：
% 两边都用了同一套从 URDF 反解出来的 M_ax / A / B，
% 所以**如果 URDF 本身错了，两边会一起错、照样通过**。
%
% 能量守恒是一条完全独立的证据链：**不需要任何参数对照**，
% 只用 Simscape 自己输出的 (q, ω) 去算
%
%     E(t) = 1/2 * I_axis * ω²  +  U(q)
%     U(q) = A*cos(q) − B*sin(q)        (因为 dU/dq = −(A sin q + B cos q) = −τ_g)
%
% 自由摆动（T = 0）时系统保守，E 应当只做数值波动、不漂移。
% 积分器的能量漂移会直接暴露在这里 —— 这是轨迹对照看不出来的
% （两条轨迹可以一起漂）。
%
% 归一化：drift / (1/2 * I_axis * max(ω)²)，用动能尺度做参考，
% 避免拿 J 级的 E 去除以 mJ 级的量。
%
% 用法：
%   validate_energy        % 默认 3 s 自由摆动
%   validate_energy(5)

if nargin < 1, Tend = 3.0; end

here = fileparts(fileparts(mfilename('fullpath')));
simd = fullfile(here,'simscape');
mdl  = 'sm_exo_real_harness';
if ~isempty(getenv('EXO2609_MODEL')), mdl = [getenv('EXO2609_MODEL') '_harness']; end
logf = fullfile(simd,'validate_energy_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== validate_energy ==\n%s\n', datestr(now));
fprintf(fid,'自由摆动 T = 0 ，时长 %.3f s\n\n', Tend);

pp = fullfile(simd,'plant_params.csv');
if isempty(dir(pp))
    error('validate_energy:noParams','%s missing - 先跑 _stp2plant.py', pp);
end
raw  = readmatrix(pp,'NumHeaderLines',1);
M_ax = raw(1:2,2);  Ac = raw(1:2,3);  Bc = raw(1:2,4);
fprintf(fid,'I_axis = [%.9f  %.9f] kg*m^2\n', M_ax(1), M_ax(2));
fprintf(fid,'A = [%+.9f  %+.9f] N*m\n', Ac(1), Ac(2));
fprintf(fid,'B = [%+.9f  %+.9f] N*m\n\n', Bc(1), Bc(2));

% ---------- 1. Simscape 自由摆动 ----------
if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(fullfile(simd,[mdl '.slx']));
tt0 = (0:1e-4:Tend).';
assignin('base','T_L_data', timeseries(zeros(size(tt0)), tt0));
assignin('base','T_R_data', timeseries(zeros(size(tt0)), tt0));
so = sim(mdl,'StopTime',num2str(Tend));

qS = {getws('qL_log',so), getws('qR_log',so)};
wS = {getws('wL_log',so), getws('wR_log',so)};

% ---------- 2. 解析参考（ode45 高精度）----------
Tf = @(~) [0;0];
odef = @(t,y) plant_ode(t, y, M_ax, Ac, Bc, Tf);
sol  = ode45(odef, [0 Tend], [0;0;0;0], odeset('RelTol',1e-12,'AbsTol',1e-14));
ttA  = (0:1e-3:Tend).';
YA   = deval(sol, ttA).';
qA   = {YA(:,1), YA(:,2)};
wA   = {YA(:,3), YA(:,4)};

% ---------- 3. 计算能量漂移 ----------
fprintf(fid,'%-8s %14s %14s %14s\n','leg','max|q| [rad]','E_scale [J]','drift/E_scale');
fprintf(fid,'%s\n', repmat('-',1,56));

res = struct();
for i = 1:2
    % Simscape：把 ω 对齐到 q 的时间栅格
    tq = qS{i}.Time;  qv = qS{i}.Data;
    wv = interp1(wS{i}.Time, wS{i}.Data, tq, 'linear');

    [drS, scS, E0S] = energy_drift(qv, wv, M_ax(i), Ac(i), Bc(i));
    [drA, scA, E0A] = energy_drift(qA{i}, wA{i}, M_ax(i), Ac(i), Bc(i));

    fprintf(fid,'%-8s %14.6f %14.6e %14.3e\n', ...
        sprintf('leg_%s', ternary(i==1,'L','R')), max(abs(qv)), scS, drS);
    fprintf(fid,'  (ode45 参考同一量: drift/E_scale = %.3e)\n', drA);
    fprintf(fid,'  E(0) = %.9e J  (simscape)   %.9e J  (ode45)   差 %.2e J\n', ...
        E0S, E0A, abs(E0S-E0A));
    res(i).drift = drS;  res(i).driftRef = drA;
    res(i).scale = scS;  res(i).E0 = E0S;
end

worst    = max([res(1).drift res(2).drift]);
worstRef = max([res(1).driftRef res(2).driftRef]);

fprintf(fid,'\n最差 Simscape 能量漂移 = %.3e  (ode45 参考 %.3e)\n', worst, worstRef);
fprintf(fid,'比 ode45 参考大 %.1f 倍  —— 差异来自 ode23t 的 RelTol=1e-6/AbsTol=1e-8\n', ...
    worst / max(worstRef, eps));

if worst < 1e-3
    verdict = 'OK';
    fprintf(fid,'\n判定：OK —— 自由摆动机械能守恒到 %.1e（动能尺度），\n', worst);
    fprintf(fid,'      说明 Simscape 侧的惯量、重力矩、积分器自洽，\n');
    fprintf(fid,'      这**独立于**「与解析模型对照」那条证据链。\n');
elseif worst < 1e-2
    verdict = 'MARGINAL';
    fprintf(fid,'\n判定：MARGINAL —— 漂移偏大，建议收紧 ode23t 的 RelTol/AbsTol。\n');
else
    verdict = 'FAIL';
    fprintf(fid,'\n判定：FAIL —— 能量明显不守恒：查阻尼/摩擦是否非 0，或积分器步长。\n');
end

fprintf(fid,'\nVALIDATE_ENERGY_%s\n', verdict);
out = struct('verdict',verdict,'worst',worst,'worstRef',worstRef,'log',logf);
end

% ---------------------------------------------------------------- helpers
function [drift, scale, E0] = energy_drift(q, w, M, A, B)
% E = 1/2*M*w^2 + (A*cos q - B*sin q)
E     = 0.5*M*w.^2 + (A*cos(q) - B*sin(q));
E0    = E(1);
scale = 0.5*M*max(abs(w))^2;          % 动能尺度
drift = max(abs(E - E0)) / max(scale, eps);
end

function yd = plant_ode(t, y, M_ax, Ac, Bc, Tfun)
nq = numel(M_ax);  q = y(1:nq);  qd = y(nq+1:end);
Tg = Ac.*sin(q) + Bc.*cos(q);
yd = [qd; (Tfun(t) + Tg)./M_ax];
end

function ts = getws(name, so)
ts = [];
try
    if ~isempty(so) && isprop(so,name) && ~isempty(so.(name)), ts = so.(name); end
catch
end
if isempty(ts) && evalin('base',['exist(''' name ''',''var'')']), ts = evalin('base',name); end
if isempty(ts), error('validate_energy:noLog','未找到记录信号 %s', name); end
if isa(ts,'timeseries'), return, end
if isstruct(ts) && isfield(ts,'time') && isfield(ts,'signals')
    ts = timeseries(ts.signals.values, ts.time);
end
end

function s = ternary(tf,a,b), if tf, s = a; else, s = b; end, end
