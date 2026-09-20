function probe_plant()
%PROBE_PLANT  Directly measure the imported plant's initial angular acceleration
%             under gravity alone, and compare with the prediction from the URDF.
%             This settles "does the Simscape plant have the gravity term we think".
%             Writes simscape/probe_plant_log.txt
here = fileparts(fileparts(mfilename('fullpath')));
simd = fullfile(here,'simscape');
mdl  = 'sm_exo_real_harness';
logf = fullfile(simd,'probe_plant_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== probe_plant ==\n%s\n\n', datestr(now));

% prediction from plant_params.csv
raw = readmatrix(fullfile(simd,'plant_params.csv'),'NumHeaderLines',1);
M_ax = raw(1:2,2);  Ac = raw(1:2,3);  Bc = raw(1:2,4);
fprintf(fid,'from URDF:  M = [%.9f %.9f]\n', M_ax(1), M_ax(2));
fprintf(fid,'            tau_g(0) = B = [%+.9f %+.9f] N*m\n', Bc(1), Bc(2));
fprintf(fid,'  => predicted qdd(0) = -B/M = [%+.6f %+.6f] rad/s^2\n\n', ...
        -Bc(1)/M_ax(1), -Bc(2)/M_ax(2));

if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(fullfile(simd,[mdl '.slx']));

% ---------- free swing, T = 0 ----------
taus = (0:1e-4:1).';
assignin('base','T_L_data', timeseries(zeros(numel(taus),1), taus));
assignin('base','T_R_data', timeseries(zeros(numel(taus),1), taus));
so = sim(mdl,'StopTime','1');

qL = getts(so,'qL_log');  wL = getts(so,'wL_log');
qR = getts(so,'qR_log');  wR = getts(so,'wR_log');

fprintf(fid,'-- free swing (T = 0) --\n');
dumpreport(fid, 'qL', qL); dumpreport(fid, 'qR', qR);

% initial acceleration by finite difference on the dense early samples
for k = 1:2
    if k==1, t = qL.Time; d = qL.Data; nm='qL'; else, t = qR.Time; d = qR.Data; nm='qR'; end
    i = find(t >= 0.02, 1);
    if ~isempty(i) && t(i) > 0
        qdd0 = 2*(d(i) - d(1))/(t(i)^2);
        fprintf(fid,'   measured qdd(0) [%s] = %+.6f rad/s^2   (pred %+.6f)\n', ...
                nm, qdd0, -Bc(k)/M_ax(k));
    end
end

% ---------- tiny horizon: does it move at all? ----------
fprintf(fid,'\n-- short horizon sweep (T = 0) --\n');
for Te = [0.01 0.05 0.2 0.5 1.0]
    assignin('base','T_L_data', timeseries(zeros(numel(taus),1), taus));
    assignin('base','T_R_data', timeseries(zeros(numel(taus),1), taus));
    s2 = sim(mdl,'StopTime',num2str(Te));
    q2 = getts(s2,'qL_log');
    fprintf(fid,'   StopTime %5.2f : samples=%5d  tspan=[%.6f %.6f]  qL=[%+.6f %+.6f]\n', ...
            Te, numel(q2.Time), q2.Time(1), q2.Time(end), min(q2.Data), max(q2.Data));
end

% ---------- constant torque: does the plant respond to T at all? ----------
fprintf(fid,'\n-- constant torque step --\n');
for Tc = [0 1 5]
    assignin('base','T_L_data', timeseries(Tc*ones(numel(taus),1), taus));
    assignin('base','T_R_data', timeseries(Tc*ones(numel(taus),1), taus));
    s3 = sim(mdl,'StopTime','0.2');
    q3 = getts(s3,'qL_log');
    fprintf(fid,'   T = %+g N*m : qL(end)=%+.9f  max|qL|=%+.9f  samples=%d\n', ...
            Tc, q3.Data(end), max(abs(q3.Data)), numel(q3.Time));
end

fprintf(fid,'\nPROBE_PLANT_OK\n');
end

% ---------------------------------------------------------------- helpers
function ts = getts(so, name)
ts = [];
try
    if isprop(so,name), ts = so.(name); end
catch
end
if isempty(ts)
    if evalin('base',['exist(''' name ''',''var'')']), ts = evalin('base',name); end
end
if isempty(ts), error('probe:noLog','%s not found', name); end
if isa(ts,'timeseries'), return, end
if isstruct(ts) && isfield(ts,'time') && isfield(ts,'signals')
    ts = timeseries(ts.signals.values, ts.time);
end
end

function dumpreport(fid, nm, ts)
fprintf(fid,'   %s : samples=%d  tspan=[%.6f %.6f]  min=%+.9f  max=%+.9f\n', ...
        nm, numel(ts.Time), ts.Time(1), ts.Time(end), min(ts.Data), max(ts.Data));
end
