function probe_ports()
%PROBE_PORTS  Discover the dialog enums and physical ports we need to build a
%             Simulink harness around the imported sm_exo2dof plant.
%             Writes simscape/probe_ports_log.txt
here = fileparts(fileparts(mfilename('fullpath')));
simd = fullfile(here,'simscape');
mdl  = 'sm_exo2dof';
logf = fullfile(simd,'probe_ports_log.txt');
fid  = fopen(logf,'w');
c = onCleanup(@() fclose(fid));

fprintf(fid,'== probe_ports ==\n%s\n', datestr(now));
load_system(fullfile(simd,[mdl '.slx']));
fprintf(fid,'loaded %s\n', mdl);

jnts = {'sm_exo2dof/hip_L','sm_exo2dof/hip_R'};

% ---- 1. enums we care about -----------------------------------------
keys = {'TorqueActuationMode','MotionActuationMode','SensePosition','SenseVelocity', ...
        'SenseAcceleration','SenseTorqueForce','DampingCoefficient'};
fprintf(fid,'\n-- dialog enums on %s --\n', jnts{1});
dp = get_param(jnts{1},'DialogParameters');
fn = fieldnames(dp);
for i = 1:numel(fn)
    f = fn{i};
    if isempty(find(strcmp(f,keys),1)), continue; end
    ent = dp.(f);
    fprintf(fid,'  %-22s Type=%-12s Enum={%s}\n', f, ent.Type, strjoin(cellstr(ent.Enum),' | '));
end

% ---- 2. port handles BEFORE -----------------------------------------
fprintf(fid,'\n-- PortHandles BEFORE (hip_L) --\n');
ph0 = get_param(jnts{1},'PortHandles');
dump_ph(fid, ph0);

% ---- 3. enable actuation + sensing ----------------------------------
% pick the torque-on value from the enum (first entry that contains 'input')
tv = pick_enum(dp.TorqueActuationMode.Enum,'InputTorque');
fprintf(fid,'\nTorqueActuationMode -> "%s"\n', tv);
for k = 1:numel(jnts)
    set_param(jnts{k},'TorqueActuationMode',tv);
    set_param(jnts{k},'SensePosition','on');
    set_param(jnts{k},'SenseVelocity','on');
end
fprintf(fid,'hip_L/hip_R: TorqueActuationMode=%s SensePosition=%s SenseVelocity=%s\n', ...
    get_param(jnts{1},'TorqueActuationMode'), get_param(jnts{1},'SensePosition'), ...
    get_param(jnts{1},'SenseVelocity'));

% ---- 4. port handles AFTER ------------------------------------------
fprintf(fid,'\n-- PortHandles AFTER (hip_L) --\n');
ph1 = get_param(jnts{1},'PortHandles');
dump_ph(fid, ph1);

% also print the port connectivity table (labels live here)
fprintf(fid,'\n-- PortConnectivity AFTER (hip_L) --\n');
try
    pc = get_param(jnts{1},'PortConnectivity');
    for i = 1:numel(pc)
        fprintf(fid,'  [%d] Type=%-8s Name=%-14s Src=%s Dst=%s\n', i, ...
            pc(i).Type, safe_field(pc(i),'Name'), ...
            safe_src(pc(i).SrcBlock), safe_dst(pc(i).DstBlock));
    end
catch ME
    fprintf(fid,'  <PortConnectivity failed: %s>\n', ME.message);
end

% ---- 5. MechanismConfiguration gravity ------------------------------
fprintf(fid,'\n-- MechanismConfiguration params --\n');
mc = find_system(mdl,'MaskType','Mechanism Configuration');
fprintf(fid,'  found %d: %s\n', numel(mc), strjoin(mc,', '));
if ~isempty(mc)
    dpm = get_param(mc{1},'DialogParameters');
    fm = fieldnames(dpm);
    for i = 1:numel(fm)
        if ~isempty(strfind(lower(fm{i}),'grav'))
            fprintf(fid,'  %-22s Type=%-12s = %s\n', fm{i}, dpm.(fm{i}).Type, ...
                safe_get(mc{1},fm{i}));
        end
    end
end

% ---- 6. helper blocks available? ------------------------------------
fprintf(fid,'\n-- helper block availability --\n');
for b = {'nesl_utility/Simulink-PS Converter','nesl_utility/PS-Simulink Converter', ...
         'simulink/Sources/From Workspace','simulink/Sinks/To Workspace', ...
         'simulink/Sources/Inport','simulink/Sinks/Outport'}
    fprintf(fid,'  %-45s load_system=%d\n', b{1}, exist_blocklib(b{1}));
end

% ---- 7. Simulink-PS / PS-Simulink dialog ----------------------------
fprintf(fid,'\n-- units params --\n');
tmp = [mdl '_tmp'];
if bdIsLoaded(tmp), close_system(tmp,0); end
new_system(tmp); load_system(tmp);
try
    a = add_block('nesl_utility/Simulink-PS Converter',[tmp '/S2PS']);
    dps = get_param(a,'DialogParameters');
    for i = 1:numel(keys_unit(dps))
        f = keys_unit(dps); f = f{i};
        fprintf(fid,'  S2PS.%-22s = %s\n', f, safe_get(a,f));
    end
    b2 = add_block('nesl_utility/PS-Simulink Converter',[tmp '/PS2S']);
    dpb = get_param(b2,'DialogParameters'); fb = fieldnames(dpb);
    for i = 1:numel(fb)
        if ~isempty(strfind(lower(fb{i}),'unit')) || ~isempty(strfind(lower(fb{i}),'out'))
            fprintf(fid,'  PS2S.%-22s = %s\n', fb{i}, safe_get(b2,fb{i}));
        end
    end
catch ME
    fprintf(fid,'  <unit probe failed: %s>\n', ME.message);
end
close_system(tmp,0);

fprintf(fid,'\nPROBE_PORTS_OK\n');
end

% ---------------------------------------------------------------- utils
function dump_ph(fid, ph)
f = fieldnames(ph);
for i = 1:numel(f)
    v = ph.(f{i});
    if isempty(v)
        fprintf(fid,'  %-8s : (empty)\n', f{i});
    else
        s = arrayfun(@(x) sprintf('%g',double(x)), v, 'UniformOutput', false);
        fprintf(fid,'  %-8s : n=%d  [%s]\n', f{i}, numel(v), strjoin(s,', '));
    end
end
end

function s = safe_field(st, fn)
if isfield(st,fn) && ~isempty(st.(fn)), s = char(string(st.(fn))); else, s = '-'; end
end
function s = safe_src(h), if isempty(h), s='-'; else, s='*'; end, end
function s = safe_dst(h), if isempty(h), s='-'; else, s='*'; end, end

function s = safe_get(b, p)
try, v = get_param(b,p); if ischar(v), s=v; else, s=mat2str(v); end
catch, s='<n/a>'; end
end

function v = pick_enum(en, want)
v = want;
if ischar(en), en = cellstr(en); end
for i = 1:numel(en)
    if ~isempty(strfind(lower(en{i}), lower(want)))
        v = en{i}; return
    end
end
end

function k = keys_unit(dp)
k = {};
f = fieldnames(dp);
for i = 1:numel(f)
    if ~isempty(strfind(lower(f{i}),'unit')) || ~isempty(strfind(lower(f{i}),'filter'))
        k{end+1} = f{i}; %#ok<AGROW>
    end
end
end

function r = exist_blocklib(b)
try, load_system(b); r = 1; catch, r = 0; end
end
