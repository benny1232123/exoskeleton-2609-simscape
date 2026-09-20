function probe_harness()
%PROBE_HARNESS  Why does the Simscape run stop at |q| = 1.3146 regardless of torque?
%               Writes simscape/probe_harness_log.txt
here = fileparts(fileparts(mfilename('fullpath')));
simd = fullfile(here,'simscape');
mdl  = 'sm_exo_real_harness';
logf = fullfile(simd,'probe_harness_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== probe_harness ==\n%s\n\n', datestr(now));

if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(fullfile(simd,[mdl '.slx']));

% ---- 1. are the joint limits really off? ----
for j = {'hip_L','hip_R'}
    b = [mdl '/' j{1}];
    fprintf(fid,'%s\n', b);
    for p = {'TorqueActuationMode','SensePosition','SenseVelocity', ...
             'LowerLimitSpecify','LowerLimitBound','UpperLimitSpecify','UpperLimitBound'}
        fprintf(fid,'   %-22s = %s\n', p{1}, get_param(b,p{1}));
    end
end

% ---- 2. solver / model settings ----
cs = getActiveConfigSet(mdl);
for p = {'SolverName','SolverType','StopTime','MaxStep','RelTol','AbsTol'}
    try, fprintf(fid,'\n%-12s = %s', p{1}, get_param(cs,p{1})); catch, end
end
fprintf(fid,'\n');

% ---- 3. scenario comparison: sample count and span ----
fprintf(fid,'\n-- scenarios --\n');
for k = 1:2
    Tend = 1.0;
    taus = (0:1e-4:Tend).';
    if k == 1
        Ta = [zeros(size(taus)), zeros(size(taus))];
        tag = 'T = 0';
    else
        Ta = [ 0.15*sin(2*pi*0.3*taus), -0.15*sin(2*pi*0.3*taus) ];
        tag = '0.15 N*m @ 0.3 Hz';
    end
    assignin('base','T_L_data', timeseries(Ta(:,1), taus));
    assignin('base','T_R_data', timeseries(Ta(:,2), taus));
    so = sim(mdl,'StopTime',num2str(Tend));

    qL = evalin('base','qL_log');
    fprintf(fid,'\n[%d] %s\n', k, tag);
    fprintf(fid,'   samples   = %d\n', numel(qL.Time));
    fprintf(fid,'   t span    = [%.6f  %.6f]  (requested StopTime %.1f)\n', ...
            qL.Time(1), qL.Time(end), Tend);
    fprintf(fid,'   qL range  = [%+.6f  %+.6f] rad\n', min(qL.Data), max(qL.Data));
    fprintf(fid,'   qL(1)     = %+.9f ; qL(end) = %+.9f\n', qL.Data(1), qL.Data(end));
    fprintf(fid,'   qL min/max time = %.6f / %.6f s\n', ...
            qL.Time(find(qL.Data==min(qL.Data),1)), qL.Time(find(qL.Data==max(qL.Data),1)));
    try
        fprintf(fid,'   sim out   : %s\n', class(so));
    catch
    end
end

fprintf(fid,'\nPROBE_HARNESS_OK\n');
end
