function info = import_exo2dof()
%IMPORT_EXO2DOF  SolidWorks/CAD -> Simscape Multibody bridge for the 2-DOF
%                hip exoskeleton, and dump an inventory of what was created.
%
%   info = import_exo2dof()
%
%   Reads  matlab2609/simscape/exo2dof.urdf  (URDF export of the SolidWorks
%   assembly -- see SOLIDWORKS_SIMSCAPE_BRIDGE_SOP.md) and generates the
%   Simscape Multibody model  sm_exo2dof.slx  next to it.
%
%   Also writes a block inventory to
%   matlab2609/simscape/import_exo2dof_log.txt  so downstream scripts
%   (build_harness_exo2dof.m) can find the joint block paths and their
%   actuation / sensing dialog parameters without guessing.
%
%   Notes
%   -----
%   * smimport() accepts ONLY Simscape Multibody XML (from the Simscape
%     Multibody Link CAD plug-in) or URDF. It does NOT accept STEP/IGES/SLDPRT.
%   * smimport() creates the model in memory only. Under -batch the unsaved
%     model is discarded on exit, so we must save_system() explicitly.
%   * For URDF input no parameter data file is produced (the block values are
%     baked in). The second output of smimport is always '' in that case.

here = fileparts(fileparts(mfilename('fullpath')));      % .../matlab2609
simd = fullfile(here,'simscape');
urdf = fullfile(simd,'exo_real.urdf');   % 由 _stp2urdf.py 从真实装配体 STEP 生成
mdl  = 'sm_exo_real';
logf = fullfile(simd,'import_exo_real_log.txt');

fid = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== import_exo2dof ==\n');
fprintf(fid,'time       : %s\n', datestr(now));
fprintf(fid,'matlabroot : %s\n', matlabroot);
fprintf(fid,'urdf       : %s\n', urdf);
if isempty(dir(urdf))
    error('import_exo2dof:noUrdf','URDF not found: %s', urdf);
end

if bdIsLoaded(mdl)
    close_system(mdl,0);
end

% ---------- 1. import -------------------------------------------------
[H, dfn] = smimport(urdf,'ModelName',mdl,'ModelSimplification','bringJointsToTop');
fprintf(fid,'smimport   : OK  (handle=%g, datafile="%s")\n', H, char(string(dfn)));

% ---------- 2. persist ------------------------------------------------
slx = fullfile(simd,[mdl '.slx']);
save_system(mdl, slx);
d = dir(slx);
if isempty(d)
    fprintf(fid,'save_system: FAILED\n');
    info = struct('ok',false,'slx',slx);
    return
end
fprintf(fid,'save_system: OK  %s  (%d bytes)\n', slx, d.bytes);

% ---------- 3. solver / model settings --------------------------------
try
    cs = getActiveConfigSet(mdl);
    fprintf(fid,'solver     : %s  StartTime=%s StopTime=%s\n', ...
        get_param(cs,'SolverName'), get_param(cs,'StartTime'), get_param(cs,'StopTime'));
catch ME
    fprintf(fid,'solver     : <query failed: %s>\n', ME.message);
end

% ---------- 4. block inventory ----------------------------------------
blks = find_system(mdl,'SearchDepth',Inf,'Type','Block');
fprintf(fid,'\n-- block inventory (%d blocks) --\n', numel(blks));
jointPaths = {};
for k = 1:numel(blks)
    bt = ''; mt = '';
    try, bt = get_param(blks{k},'BlockType'); end
    try, mt = get_param(blks{k},'MaskType');  end
    fprintf(fid,'%s | BlockType=%s | MaskType=%s\n', blks{k}, bt, mt);
    if ~isempty(strfind(lower(mt),'joint')) || ~isempty(strfind(lower(bt),'joint'))
        jointPaths{end+1} = blks{k};  %#ok<AGROW>
    end
end

% ---------- 5. joint dialog parameters --------------------------------
fprintf(fid,'\n-- joint candidates (%d) --\n', numel(jointPaths));
for k = 1:numel(jointPaths)
    fprintf(fid,'[%d] %s\n', k, jointPaths{k});
    try
        dp = get_param(jointPaths{k},'DialogParameters');
        fn = fieldnames(dp);
        for j = 1:numel(fn)
            nm = fn{j};
            if isempty(strfind(lower(nm),'actuat')) && ...
               isempty(strfind(lower(nm),'sens'))   && ...
               isempty(strfind(lower(nm),'axis'))   && ...
               isempty(strfind(lower(nm),'limit'))  && ...
               isempty(strfind(lower(nm),'damp'))   && ...
               isempty(strfind(lower(nm),'friction'))
                continue
            end
            try
                v = get_param(jointPaths{k}, nm);
                if ischar(v)
                    vs = v;
                elseif iscell(v)
                    vs = strjoin(cellfun(@num2str,v,'uni',0),', ');
                else
                    vs = mat2str(v);
                end
                fprintf(fid,'      %-28s = %s\n', nm, vs);
            catch
            end
        end
    catch ME
        fprintf(fid,'      <DialogParameters failed: %s>\n', ME.message);
    end
end

% ---------- 6. top-level layout --------------------------------------
tops = find_system(mdl,'SearchDepth',1,'Type','Block');
fprintf(fid,'\n-- top level (%d) --\n', numel(tops));
for k = 1:numel(tops)
    fprintf(fid,'  %s\n', tops{k});
end

fprintf(fid,'\nIMPORT_EXO2DOF_OK\n');

info = struct('ok',true,'slx',slx,'model',mdl,'joints',{jointPaths},'log',logf);
end
