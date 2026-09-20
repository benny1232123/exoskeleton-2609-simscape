function p = params_from_csv(csvpath, groupL, groupR, dt, verbose)
%PARAMS_FROM_CSV  读取 SolidWorks 导出的质量属性 CSV，反算 M 与 G_amp。
%
%   p = exo2609.params_from_csv(csvpath, groupL, groupR, dt, verbose)
%
%   ==== 对 SolidWorks 一侧的接口约定（详见 solidworks/SOLIDWORKS_质量属性_SOP.md）====
%   CSV 必须由**髋关节参考坐标系**输出，该坐标系满足：
%       * 原点落在髋屈伸轴线上（任意一点即可）
%       * Z 轴沿髋屈伸轴（正方向任选，惯量不受影响）
%       * 单位 mm / kg（SolidWorks 文档单位设为 MMKS 或自定义 mm-kg）
%   这时 CSV 里「关于输出坐标系原点」的 Izz_O 就**直接是绕髋轴的转动惯量**。
%
%   列（表头，大小写不敏感、允许前后空格）：
%       group                                                   分组名（如 leg_L / leg_R）
%       part, material                                          仅作留档
%       mass_kg                                                 质量 [kg]
%       cx_mm, cy_mm, cz_mm                                     质心在参考系中的坐标 [mm]
%       Ixx_com ... Iyz_com   关于**质心**的惯性张量分量 [kg*mm^2]
%       Ixx_O   ... Iyz_O     关于**输出坐标系原点**的同一组分量 [kg*mm^2]
%   只有 I__O 或只有 I__com 都能跑（会自动补算），但**两者都给才能自检**。
%
%   折算关系
%       M_i     = I_hip_i   （绕髋轴的转动惯量）
%       G_amp_i = m_i * g * d_i,  d_i = 髋轴到该组质心的垂距 = hypot(cx, cy)
%   K1 / T0 无法由几何得到（人机交互阻尼/摩擦），沿用论文标定值并标注来源。
%
%   自检：关于原点（Izz_O）与「关于质心 + 平行轴」(Izz_com + m*(cx^2+cy^2))
%         必须一致；两者偏差 > 1% 说明参考坐标系没设对或零件漏了 —— 会告警。
%
%   见 also exo2609.params_paper, solidworks/SOLIDWORKS_质量属性_SOP.md

if nargin < 2 || isempty(groupL), groupL = 'leg_L'; end
if nargin < 3 || isempty(groupR), groupR = 'leg_R'; end
if nargin < 4 || isempty(dt),     dt = 0.01;       end
if nargin < 5 || isempty(verbose), verbose = true; end

if ~exist(csvpath, 'file')
    error('exo2609:params_from_csv:missing', '找不到 CSV：%s', csvpath);
end

% 中文零件名是常态，显式按 UTF-8 读，失败再退回默认编码。
try
    T = readtable(csvpath, 'VariableNamingRule', 'preserve', 'Encoding', 'UTF-8');
catch
    T = readtable(csvpath, 'VariableNamingRule', 'preserve');
end
vn = T.Properties.VariableNames;

% ---------------- 取列（大小写不敏感、去空格） ----------------
mass = col(T, vn, {'mass_kg', 'mass', '质量kg'}, true);
cx   = col(T, vn, {'cx_mm', 'cx'}, true);
cy   = col(T, vn, {'cy_mm', 'cy'}, true);
cz   = col(T, vn, {'cz_mm', 'cz'}, false);
if isempty(cz), cz = zeros(size(mass)); end

IzzC = col(T, vn, {'izz_com'}, false);      % 关于质心
IzzO = col(T, vn, {'izz_o', 'izz_origin'}, false);   % 关于输出坐标系原点

% ---------------- 分组 ----------------
grp = colstr(T, vn, {'group', '分组'});
if isempty(grp)
    grp = repmat("all", height(T), 1);
    warning('exo2609:params_from_csv:nogroup', 'CSV 无 group 列，全部当作一组。');
end

if verbose
    fprintf('读入 %s：%d 行\n', csvpath, height(T));
    fprintf('  分组：%s\n', strjoin(cellstr(unique(grp)).', ', '));
end

bad = mass <= 0;
if any(bad)
    warning('exo2609:params_from_csv:zeromass', ...
        '有 %d 行质量 <= 0（SolidWorks 里可能没指定材质），已计入但请核对。', sum(bad));
end

% ---------------- 逐组汇总 ----------------
G_ACC = 9.81;   % 重力加速度，与 exo2609.params_paper 的 p.g 保持一致
names = cellstr(unique(grp, 'stable'));
info  = struct('group', {}, 'm_kg', {}, 'cog_mm', {}, 'd_perp_m', {}, ...
               'I_axis', {}, 'G_amp', {}, 'I_origin', {}, 'I_parallel', {}, ...
               'rel_diff_pct', {}, 'n_parts', {});

for gi = 1:numel(names)
    g  = names{gi};
    m  = grp == string(g);
    m_ = mass(m);
    Mtot = sum(m_);
    if Mtot <= 0, continue; end
    C = [cx(m), cy(m), cz(m)];
    cog = (m_.' * C) / Mtot;                    % mm
    d_perp_mm = hypot(cog(1), cog(2));          % 到髋轴（=Z 轴）的垂距
    d_perp_m  = d_perp_mm * 1e-3;

    Ipar = [];  Iorig = [];
    if ~isempty(IzzC)
        Ipar = sum(IzzC(m) + m_ .* (C(:,1).^2 + C(:,2).^2));   % kg*mm^2
    end
    if ~isempty(IzzO)
        Iorig = sum(IzzO(m));                                   % kg*mm^2
    end
    if ~isempty(Iorig)
        I_used = Iorig;
    elseif ~isempty(Ipar)
        I_used = Ipar;
    else
        error('exo2609:params_from_csv:noinertia', ...
              'CSV 缺少 Izz_com 与 Izz_O，无法得到绕髋轴惯量。');
    end

    if ~isempty(Iorig) && ~isempty(Ipar) && Iorig > 0
        rel = abs(Iorig - Ipar) / Iorig * 100;
    else
        rel = NaN;
    end

    info(end+1) = struct('group', g, 'm_kg', Mtot, 'cog_mm', cog, ...
        'd_perp_m', d_perp_m, 'I_axis', I_used*1e-6, 'G_amp', Mtot*G_ACC*d_perp_m, ...
        'I_origin', Iorig, 'I_parallel', Ipar, 'rel_diff_pct', rel, ...
        'n_parts', sum(m));                                    %#ok<AGROW>
end

if verbose
    fprintf('\n%-10s %10s %10s %12s %10s %8s %8s\n', ...
            'group', 'm[kg]', 'd[m]', 'I_axis', 'G_amp', '自检%', '零件数');
    for k = 1:numel(info)
        fprintf('%-10s %10.4f %10.4f %12.6f %10.4f %8.2f %8d\n', ...
            info(k).group, info(k).m_kg, info(k).d_perp_m, info(k).I_axis, ...
            info(k).G_amp, info(k).rel_diff_pct, info(k).n_parts);
    end
end

% ---------------- 组装参数 struct ----------------
iq = @(nm) find(strcmp({info.group}, nm), 1);
iL = iq(groupL);  iR = iq(groupR);
if isempty(iL) || isempty(iR)
    error('exo2609:params_from_csv:groupmissing', ...
        'CSV 里找不到分组 %s / %s。可用分组：%s', ...
        groupL, groupR, strjoin({info.group}, ', '));
end

p = exo2609.params_paper(dt);
p.M     = diag([info(iL).I_axis, info(iR).I_axis]);
p.G_amp = [info(iL).G_amp; info(iR).G_amp];
p.source.M     = 'cad/solidworks';
p.source.G_amp = 'cad/solidworks';
p.source.C_coef = 'cad/solidworks';
p.geometry = info;
p.geometry_csv = csvpath;

% ---- 自检告警 ----
for k = 1:numel(info)
    if ~isnan(info(k).rel_diff_pct) && info(k).rel_diff_pct > 1
        warning('exo2609:params_from_csv:inconsistent', ...
            ['分组 %s 的绕轴惯量自检偏差 %.2f%%（关于原点 %g vs 平行轴 %g kg*mm^2）。' ...
             '多半是：（a）SolidWorks 参考坐标系原点不在髋轴上/ Z 轴没对准髋轴，' ...
             '或（b）该分组的零件没选全。'], ...
            info(k).group, info(k).rel_diff_pct, info(k).I_origin, info(k).I_parallel);
    end
end

if verbose
    fprintf('\n折算结果（绕髋轴）\n');
    fprintf('  M     = [%.6f %.6f] kg*m^2\n', p.M(1,1), p.M(2,2));
    fprintf('  G_amp = [%.6f %.6f] N*m\n', p.G_amp(1), p.G_amp(2));
    fprintf('  K1    = %.4g I（沿用论文，几何不可反算）\n', p.K1(1,1));
end
end

% ==================================================================
function v = col(T, vn, cands, required)
% 按候选名（大小写不敏感、忽略空格/下划线差异）取数值列，取不到返回 []
key = @(s) lower(regexprep(char(s), '[\s_]', ''));
vnk = cellfun(key, vn, 'UniformOutput', false);
v = [];
for c = cands
    j = find(strcmp(vnk, key(c{1})), 1);
    if ~isempty(j)
        v = T.(vn{j});
        v = double(v(:));
        return
    end
end
if required
    error('exo2609:params_from_csv:colmissing', ...
        'CSV 缺少必需列（候选：%s）。已有列：%s', ...
        strjoin(cands, ' / '), strjoin(vn, ', '));
end
end

function s = colstr(T, vn, cands)
key = @(x) lower(regexprep(char(x), '[\s_]', ''));
vnk = cellfun(key, vn, 'UniformOutput', false);
s = [];
for c = cands
    j = find(strcmp(vnk, key(c{1})), 1);
    if ~isempty(j)
        s = string(T.(vn{j}));
        s = s(:);
        return
    end
end
end
