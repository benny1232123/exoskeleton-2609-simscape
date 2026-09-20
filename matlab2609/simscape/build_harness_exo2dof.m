function info = build_harness_exo2dof()
%BUILD_HARNESS_EXO2DOF  Add torque actuation + state logging to the imported
%                       Simscape Multibody plant and save a simulation harness.
%
%   Creates matlab2609/simscape/sm_exo2dof_harness.slx from sm_exo2dof.slx:
%
%       T_L_data ─▶ [S2PS] ─▶ hip_L.t
%       T_R_data ─▶ [S2PS] ─▶ hip_R.t
%       hip_L.q ─▶ [PS2S] ─▶ qL_log   hip_L.w ─▶ [PS2S] ─▶ wL_log
%       hip_R.q ─▶ [PS2S] ─▶ qR_log   hip_R.w ─▶ [PS2S] ─▶ wR_log
%
%   Gravity is forced to [0 0 -9.81] so the plant matches +exo2609 exactly.
%   Solver: ode23t, MaxStep 1e-3, tolerances tightened (see the under-sampling
%   note in +exo2609/discretization_info -- the damping pole M/K1 is ~5.7 ms).
%
%   Run compare_simscape_vs_analytical() afterwards to check the plant.

here = fileparts(fileparts(mfilename('fullpath')));
simd = fullfile(here,'simscape');
srcName = 'sm_exo_real';   dstName = 'sm_exo_real_harness';
if ~isempty(getenv('EXO2609_MODEL'))          % 可用环境变量切换到别的导入模型
    srcName = getenv('EXO2609_MODEL');
    dstName = [srcName '_harness'];
end
src  = fullfile(simd,[srcName '.slx']);
dst  = fullfile(simd,[dstName '.slx']);
mdl  = dstName;
logf = fullfile(simd,['build_harness_' srcName '_log.txt']);
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== build_harness_exo2dof ==\n%s\n', datestr(now));

% ---------- 1. start from a clean copy of the pristine import ----------
if bdIsLoaded(srcName),        close_system(srcName,0);        end
if bdIsLoaded(mdl),            close_system(mdl,0);            end
ok = copyfile(src, dst, 'f');
fprintf(fid,'copyfile: %d  %s\n', ok, dst);
if ~ok, error('build_harness_exo2dof:copy','copyfile failed'); end
load_system(dst);
fprintf(fid,'loaded %s\n', mdl);

% ---------- 2. joint actuators + sensors ------------------------------
jnt = {[mdl '/hip_L'],[mdl '/hip_R']};
for k = 1:numel(jnt)
    set_param(jnt{k},'TorqueActuationMode','InputTorque');
    set_param(jnt{k},'MotionActuationMode','ComputedMotion');
    set_param(jnt{k},'SensePosition','on');
    set_param(jnt{k},'SenseVelocity','on');
    % The analytical plant in +exo2609 has no hard stops, so a pure dynamics
    % comparison must not have them either. The URDF limits (+/-1.5 rad) become
    % a 1e4 N*m/deg stiff spring that completely dominates any trajectory which
    % reaches them. Re-enable these for realistic robot studies.
    set_param(jnt{k},'LowerLimitSpecify','off');
    set_param(jnt{k},'UpperLimitSpecify','off');
end
fprintf(fid,'joints configured: %s | %s\n', jnt{1}, jnt{2});

% ---------- 3. gravity -------------------------------------------------
mc = find_system(mdl,'MaskType','Mechanism Configuration');
if ~isempty(mc)
    set_param(mc{1},'GravityVector','[0 0 -9.81]');
    fprintf(fid,'gravity %s -> %s\n', mc{1}, get_param(mc{1},'GravityVector'));
end

% ---------- 4. discover the joint PS port indices ---------------------
% mechanical R/C are LConn1 / RConn1 (already connected by smimport).
% signal ports appear afterwards: LConn2 = torque input, RConn2/RConn3 = q,w
portInfo = struct();
for k = 1:numel(jnt)
    ph = get_param(jnt{k},'PortHandles');
    portInfo(k).blk   = jnt{k};
    portInfo(k).torq  = ph.LConn(end);
    portInfo(k).q     = ph.RConn(end-1);
    portInfo(k).w     = ph.RConn(end);
    fprintf(fid,'%s : LConn n=%d, RConn n=%d  -> torque=%.0f q=%.0f w=%.0f\n', ...
        jnt{k}, numel(ph.LConn), numel(ph.RConn), ...
        double(portInfo(k).torq), double(portInfo(k).q), double(portInfo(k).w));
end

% ---------- 5. add helper blocks ---------------------------------------
function h = addb(lib, name, pos)
    h = add_block(lib, [mdl '/' name], 'Position', pos);
    fprintf(fid,'  + %-42s %s\n', name, lib);
end
y0 = 40;
addb('simulink/Sources/From Workspace',      'T_L_src',[ 60 y0     170 y0+40 ]);
addb('simulink/Sources/From Workspace',      'T_R_src',[ 60 y0+80  170 y0+120]);
addb('nesl_utility/Simulink-PS Converter',   'T_L_S2PS',[230 y0     280 y0+40 ]);
addb('nesl_utility/Simulink-PS Converter',   'T_R_S2PS',[230 y0+80  280 y0+120]);
addb('nesl_utility/PS-Simulink Converter',   'q_L_PS2S',[430 y0+170 480 y0+210]);
addb('nesl_utility/PS-Simulink Converter',   'w_L_PS2S',[430 y0+230 480 y0+270]);
addb('nesl_utility/PS-Simulink Converter',   'q_R_PS2S',[430 y0+290 480 y0+330]);
addb('nesl_utility/PS-Simulink Converter',   'w_R_PS2S',[430 y0+350 480 y0+390]);
addb('simulink/Sinks/To Workspace',          'qL_log',   [560 y0+170 640 y0+200]);
addb('simulink/Sinks/To Workspace',          'wL_log',   [560 y0+230 640 y0+260]);
addb('simulink/Sinks/To Workspace',          'qR_log',   [560 y0+290 640 y0+320]);
addb('simulink/Sinks/To Workspace',          'wR_log',   [560 y0+350 640 y0+380]);

% ---------- 6. configure helper blocks ---------------------------------
set_param([mdl '/T_L_src'],'VariableName','T_L_data','Interpolate','on','SampleTime','0');
set_param([mdl '/T_R_src'],'VariableName','T_R_data','Interpolate','on','SampleTime','0');
for b = {'T_L_S2PS','T_R_S2PS'}
    try, set_param([mdl '/' b{1}],'Unit','N*m'); catch ME, fprintf(fid,'unit set %s: %s\n',b{1},ME.message); end
end
for b = {'q_L_PS2S','q_R_PS2S'}
    try, set_param([mdl '/' b{1}],'Unit','rad'); catch ME, fprintf(fid,'unit set %s: %s\n',b{1},ME.message); end
end
for b = {'w_L_PS2S','w_R_PS2S'}
    try, set_param([mdl '/' b{1}],'Unit','rad/s'); catch ME, fprintf(fid,'unit set %s: %s\n',b{1},ME.message); end
end
tw = {'qL_log','q_L_PS2S'; 'wL_log','w_L_PS2S'; 'qR_log','q_R_PS2S'; 'wR_log','w_R_PS2S'};
for i = 1:size(tw,1)
    try, set_param([mdl '/' tw{i,1}],'VariableName',tw{i,1},'SaveFormat','Timeseries');
    catch ME, fprintf(fid,'toworkspace %s: %s\n',tw{i,1},ME.message); end
end

% ---------- 7. connect ------------------------------------------------
function do_line(a, b, tag)
    try
        add_line(mdl, a, b, 'autorouting','on');
        fprintf(fid,'  line OK   %s\n', tag);
    catch ME
        fprintf(fid,'  line FAIL %s : %s\n', tag, ME.message);
    end
end

% helper-block PS port handles (log them once so failures are diagnosable)
for b = {'T_L_S2PS','T_R_S2PS','q_L_PS2S','w_L_PS2S','q_R_PS2S','w_R_PS2S'}
    dump_ph(fid, [mdl '/' b{1}]);
end

do_line(out_h([mdl '/T_L_src']),  in_h([mdl '/T_L_S2PS']),  'T_L_src -> T_L_S2PS');
do_line(out_h([mdl '/T_R_src']),  in_h([mdl '/T_R_S2PS']),  'T_R_src -> T_R_S2PS');
do_line(ps_out([mdl '/T_L_S2PS']), portInfo(1).torq,        'T_L_S2PS -> hip_L.t');
do_line(ps_out([mdl '/T_R_S2PS']), portInfo(2).torq,        'T_R_S2PS -> hip_R.t');
do_line(portInfo(1).q, ps_in([mdl '/q_L_PS2S']),            'hip_L.q -> q_L_PS2S');
do_line(portInfo(1).w, ps_in([mdl '/w_L_PS2S']),            'hip_L.w -> w_L_PS2S');
do_line(portInfo(2).q, ps_in([mdl '/q_R_PS2S']),            'hip_R.q -> q_R_PS2S');
do_line(portInfo(2).w, ps_in([mdl '/w_R_PS2S']),            'hip_R.w -> w_R_PS2S');
do_line(out_h([mdl '/q_L_PS2S']), in_h([mdl '/qL_log']),    'q_L_PS2S -> qL_log');
do_line(out_h([mdl '/w_L_PS2S']), in_h([mdl '/wL_log']),    'w_L_PS2S -> wL_log');
do_line(out_h([mdl '/q_R_PS2S']), in_h([mdl '/qR_log']),    'q_R_PS2S -> qR_log');
do_line(out_h([mdl '/w_R_PS2S']), in_h([mdl '/wR_log']),    'w_R_PS2S -> wR_log');

% ---------- 8. solver -------------------------------------------------
set_param(mdl,'StopTime','1.0','SolverType','Variable-step', ...
              'Solver','ode23t','MaxStep','1e-3','RelTol','1e-6','AbsTol','1e-8', ...
              'SaveOutput','off','SaveTime','on');

% ---------- 9. compile check + save -----------------------------------
% From Workspace sources need their variables to exist before compiling.
tt = (0:1e-4:1).';
assignin('base','T_L_data', timeseries(2*sin(2*pi*0.8*tt)+0.5, tt));
assignin('base','T_R_data', timeseries(-2*sin(2*pi*0.8*tt),     tt));
fprintf(fid,'workspace data: T_L_data / T_R_data defined (dry-run profile)\n');

compileOK = false;
try
    set_param(mdl,'SimulationCommand','update');
    compileOK = true;
    fprintf(fid,'compile check: OK\n');
catch ME
    fprintf(fid,'compile check: FAIL %s | %s\n', ME.identifier, ME.message);
    log_causes(fid, ME, 1);
end
try
    save_system(mdl, dst);
    d = dir(dst);
    fprintf(fid,'save_system: OK (%d bytes)\n', d.bytes);
catch ME
    fprintf(fid,'save_system: FAIL %s\n', ME.message);
end

fprintf(fid,'\nBUILD_HARNESS_%s\n', ternary_str(compileOK,'OK','PARTIAL'));
info = struct('ok',compileOK,'model',mdl,'slx',dst,'log',logf);
end

% ------------------------------------------------------------- helpers
function h = out_h(b), ph = get_param(b,'PortHandles'); h = ph.Outport(1); end
function h = in_h(b),  ph = get_param(b,'PortHandles'); h = ph.Inport(1);  end
function h = ps_out(b)
ph = get_param(b,'PortHandles');
if ~isempty(ph.Outport), h = ph.Outport(1);
elseif ~isempty(ph.RConn), h = ph.RConn(1);
else, h = ph.LConn(1); end
end
function h = ps_in(b)
ph = get_param(b,'PortHandles');
if ~isempty(ph.Inport), h = ph.Inport(1);
elseif ~isempty(ph.LConn), h = ph.LConn(1);
else, h = ph.RConn(1); end
end
function dump_ph(fid, b)
ph = get_param(b,'PortHandles'); f = fieldnames(ph); s = {};
for i = 1:numel(f)
    v = ph.(f{i});
    if ~isempty(v), s{end+1} = sprintf('%s(n=%d)', f{i}, numel(v)); end %#ok<AGROW>
end
fprintf(fid,'  ports %-38s : %s\n', b, strjoin(s,', '));
end
function s = ternary_str(tf,a,b), if tf, s=a; else, s=b; end, end

function log_causes(fid, ME, depth)
% Simulink wraps compile failures in a "MultipleErrors" MException; walk the
% cause chain so the real message is visible in the log.
pad = repmat(' ',1,4*depth);
if isempty(ME.cause)
    return
end
for i = 1:numel(ME.cause)
    c = ME.cause{i};
    fprintf(fid,'%s(cause %d) %s | %s\n', pad, i, c.identifier, c.message);
    log_causes(fid, c, depth+1);
end
end
