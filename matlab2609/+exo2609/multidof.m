classdef multidof
%MULTIDOF  4-DOF 多刚体解析动力学（每腿 hip 屈伸 + abduct 髋部横向）
%
%   ★ 与**论文口径的 2-DOF 模型**（exo2609.params_paper / exo2609.dyn_f）**并存**，
%     互不影响：论文复现链路一律走 2-DOF 那一套，本类专门服务
%     `simscape/exo_multidof.urdf`（真实装配体拆段 + 加 abduct 自由度）的交叉验证。
%     不要拿本类去替换 params_paper —— 那会破坏「严格按论文」的复现范围。
%
%   为什么不能沿用 2-DOF 那种解耦形式
%     `+exo2609` 的 2-DOF 模型是 I_i*qdd_i = T_i + A_i*sin(q_i) + B_i*cos(q_i)，
%     每关节独立。但 abduct 轴既不平行任何全局轴、又**固定在 L1 上随 hip 一起转**，
%     于是质量矩阵不再对角、重力矩也不再是单个 sin/cos。必须退回通用形式
%         M(q) qdd + C(q,qd) qd + G(q) = Q_ext
%   —— 本类就按 Jacobian 组装 M(q)、按势能梯度求 G(q)。符号约定 G = -tau_g，
%      与 compare_simscape_vs_analytical.m 一致。
%
%   用法
%     mb  = exo2609.multidof();            % 读 simscape/plant_multidof.json
%     M   = mb.massmatrix(q);              % 4x4，对称正定
%     G   = mb.gravity(q);                 % 4x1 = dU/dq（零位不为 0！）
%     U   = mb.potential(q);               % 重力势能
%     qdd = mb.accel(q, qd, T);            % 正动力学
%     [~,~,E] = mb.energy(q, qd);          % 总机械能（守恒自检用）
%     %   ⚠ energy 有**三个**输出 [KE,PE,E]：只写 E = mb.energy(...) 会拿到 KE
%     %      而不是总能量 —— 守恒判据必须取第三个输出（2026-09-20 踩过）。
%     T   = mb.fk(q);                      % struct: link 名 -> 4x4 世界位姿
%
%   参数来源：`_multidof_export.py` 从 URDF 派生，**只含 URDF 已有的量**
%   （质量/质心/惯量/关节几何），不含任何动力学结果 —— 组装算法在本文件里，
%   与 Python 侧 `_multidof_dyn.py` 是两份**独立实现**，两者应当互相印证。
%
%   见 also exo2609.params_multidof, exo2609.dyn_f, simscape/dump_multidof_traj.m

    properties (SetAccess = private)
        p          % jsondecode 的原始结构体
        nq         % 自由度数
        q_names    % 1 x nq cellstr
        links      % struct array: name / mass / cog / I
        joints     % struct array: name / type / parent / child / o / axis / movable / qidx
        order      % 关节处理顺序（拓扑序，父先于子）
        g          % 重力加速度
    end

    methods
        function obj = multidof(jsonPath)
            if nargin < 1 || isempty(jsonPath)
                here = fileparts(fileparts(mfilename('fullpath')));  % .../matlab2609
                jsonPath = fullfile(here, 'simscape', 'plant_multidof.json');
            end
            if isempty(dir(jsonPath))
                error('exo2609:multidof:noParams', ...
                      ['%s 不存在 —— 先跑\n' ...
                       '  python _multidof_export.py'], jsonPath);
            end
            obj.p   = jsondecode(fileread(jsonPath));
            obj.nq  = obj.p.nq;
            obj.g   = obj.p.gravity;
            obj.links  = obj.p.links;
            obj.joints = obj.p.joints;
            if iscell(obj.p.order)
                obj.order = obj.p.order(:)';
            else
                obj.order = cellstr(obj.p.order)';
            end
            if ischar(obj.p.q_names)
                obj.q_names = {obj.p.q_names};
            else
                obj.q_names = obj.p.q_names(:)';
            end

            % ---- 预计算每个 link 的祖先可动关节链（见 ancestors 的说明）----
            obj.p.anc = struct();
            for i = 1:numel(obj.links)
                nm = obj.links(i).name;
                A = {}; cur = nm;
                while true
                    found = false;
                    for k = 1:numel(obj.joints)
                        if strcmp(obj.joints(k).child, cur)
                            if obj.joints(k).movable
                                A = [{obj.joints(k).name}, A]; %#ok<AGROW>
                            end
                            cur = obj.joints(k).parent;
                            found = true;
                            break
                        end
                    end
                    if ~found, break, end
                end
                obj.p.anc.(nm) = A;
            end
        end

        % ------------------------------------------------------------ 运动学
        function T = fk(obj, q)
        %FK  前向运动学。T.(linkName) = 4x4 世界位姿。
        %   本工程口径：link frame 与世界系轴对齐，joint rpy = 0 0 0，
        %   故 M_child = M_parent · [R(axis,q)  o; 0 1]
            q = q(:);
            assert(numel(q) == obj.nq, 'exo2609:multidof:badq', ...
                   'q 应为 %d 维，实际 %d', obj.nq, numel(q));
            T = struct();
            kids = {obj.joints.child};
            for i = 1:numel(obj.links)
                nm = obj.links(i).name;
                if ~any(strcmp(nm, kids))
                    T.(nm) = eye(4);            % 根（base），frame ≡ 世界系
                end
            end
            for k = 1:numel(obj.order)
                j  = obj.jnamed(obj.order{k});
                Tp = T.(j.parent);
                o  = j.o(:);
                if j.movable
                    Tj = [rotm(j.axis(:), q(j.qidx+1)), o; 0 0 0 1];
                else
                    Tj = [eye(3), o; 0 0 0 1];  % fixed：只平移
                end
                T.(j.child) = Tp * Tj;
            end
        end

        function c = com_world(obj, q)
        %COM_WORLD  各段质心世界坐标。c.(linkName) = 3x1
            T = obj.fk(q);
            c = struct();
            for i = 1:numel(obj.links)
                nm = obj.links(i).name;
                c.(nm) = T.(nm)(1:3,1:3) * obj.links(i).cog(:) + T.(nm)(1:3,4);
            end
        end

        function [Jv, Jw] = jacobians(obj, q)
        %JACOBIANS  各段质心的线速度/角速度雅可比。Jv.(nm) / Jw.(nm) = 3 x nq
        %
        %   对 link 的祖先链上每个**可动**关节 j：
        %       Jw(:,j) = a_j^world
        %       Jv(:,j) = a_j^world × (p_com − o_j^world)
        %   不在链上的列为 0。
            T  = obj.fk(q);
            Jv = struct(); Jw = struct();
            % 先算每个可动关节的世界轴/原点（用父链位姿即可，不需要 q）
            [ow, aw] = obj.joint_frames(T);
            for i = 1:numel(obj.links)
                nm = obj.links(i).name;
                A  = obj.ancestors(nm);
                Jv.(nm) = zeros(3, obj.nq);
                Jw.(nm) = zeros(3, obj.nq);
                % 变量名用 pc 不用 p：类里有个属性叫 obj.p，
                % 同名会被 checkcode 提示「是否想引用属性」（良性但吵）
                pc = T.(nm)(1:3,1:3) * obj.links(i).cog(:) + T.(nm)(1:3,4);
                for k = 1:numel(A)
                    jn = A{k};
                    qi = obj.jnamed(jn).qidx + 1;
                    Jw.(nm)(:,qi) = aw.(jn);
                    Jv.(nm)(:,qi) = cross(aw.(jn), pc - ow.(jn));
                end
            end
        end

        % ------------------------------------------------------------ 动力学
        function M = massmatrix(obj, q)
        %MASSMATRIX  M(q) = Σ_k [ m_k Jv'Jv + Jw'(R I R')Jw ]
            [Jv, Jw] = obj.jacobians(q);
            T = obj.fk(q);
            M = zeros(obj.nq);
            for i = 1:numel(obj.links)
                nm = obj.links(i).name;
                m  = obj.links(i).mass;
                if m == 0, continue, end
                R  = T.(nm)(1:3,1:3);
                Iw = R * obj.links(i).I * R.';
                M  = M + m * (Jv.(nm).' * Jv.(nm)) + Jw.(nm).' * Iw * Jw.(nm);
            end
            M = (M + M.') / 2;      % 对称化
        end

        function U = potential(obj, q)
        %POTENTIAL  重力势能 [J]，z 向上为正
            c = obj.com_world(q);
            U = 0;
            for i = 1:numel(obj.links)
                U = U + obj.links(i).mass * obj.g * c.(obj.links(i).name)(3);
            end
        end

        function G = gravity(obj, q, h)
        %GRAVITY  G(q) = dU/dq（中心差分）。零位 G(0) ≠ 0 —— CAD 零位不是重力平衡位。
            if nargin < 3 || isempty(h), h = 1e-7; end
            q = q(:);
            G = zeros(obj.nq,1);
            for i = 1:obj.nq
                e = zeros(obj.nq,1); e(i) = h;
                G(i) = (obj.potential(q+e) - obj.potential(q-e)) / (2*h);
            end
        end

        function Cq = coriolis(obj, q, qd, h)
        %CORIOLIS  [C(q,qd) qd] —— Christoffel 的数值版（对 M 求中心差分）
            if nargin < 4 || isempty(h), h = 1e-7; end
            q = q(:); qd = qd(:);
            dM = cell(1, obj.nq);
            for i = 1:obj.nq
                e = zeros(obj.nq,1); e(i) = h;
                dM{i} = (obj.massmatrix(q+e) - obj.massmatrix(q-e)) / (2*h);
            end
            Cq = zeros(obj.nq,1);
            for i = 1:obj.nq
                s = 0;
                for j = 1:obj.nq
                    for k = 1:obj.nq
                        s = s + dM{k}(i,j) * qd(j) * qd(k);
                    end
                end
                for j = 1:obj.nq
                    for k = 1:obj.nq
                        s = s - 0.5 * dM{i}(j,k) * qd(j) * qd(k);
                    end
                end
                Cq(i) = s;
            end
        end

        function qdd = accel(obj, q, qd, T)
        %ACCEL  正动力学：M(q) qdd = T − C(q,qd) qd − G(q)
            q = q(:); qd = qd(:); T = T(:);
            rhs = T - obj.coriolis(q, qd) - obj.gravity(q);
            qdd = obj.massmatrix(q) \ rhs;
        end

        function [KE, PE, E] = energy(obj, q, qd)
        %ENERGY  总机械能 E = KE + PE（自由落体下应守恒）
            q = q(:); qd = qd(:);
            [Jv, Jw] = obj.jacobians(q);
            T = obj.fk(q);
            KE = 0;
            for i = 1:numel(obj.links)
                nm = obj.links(i).name;
                m  = obj.links(i).mass;
                if m == 0, continue, end
                v  = Jv.(nm) * qd;
                w  = Jw.(nm) * qd;
                R  = T.(nm)(1:3,1:3);
                KE = KE + 0.5*m*(v.'*v) + 0.5*w.'*(R*obj.links(i).I*R.')*w;
            end
            PE = obj.potential(q);
            E  = KE + PE;
        end
    end

    % ---------------------------------------------------------------- private
    methods (Access = private)
        function j = jnamed(obj, nm)
        %按名字取关节 struct
            for i = 1:numel(obj.joints)
                if strcmp(obj.joints(i).name, nm)
                    j = obj.joints(i);
                    if isempty(j.axis), j.axis = [0 0 0]; end
                    return
                end
            end
            error('exo2609:multidof:noJoint','找不到关节 %s', nm);
        end

        function [ow, aw] = joint_frames(obj, T)
        %每个可动关节的世界原点/轴向（轴向取归一）。
        %   只需已算好的 fk 结果 T，与 q 无关（轴/原点由**父链**位姿决定，
        %   关节自身的转角不影响自身轴线）—— 所以签名里不要 q。
        %   这样 jacobians / massmatrix / gravity 可以复用同一份 T，省一次 fk。
            ow = struct(); aw = struct();
            for k = 1:numel(obj.order)
                j  = obj.jnamed(obj.order{k});
                if ~j.movable, continue, end
                Tp = T.(j.parent);
                ow.(j.name) = Tp(1:3,1:3)*j.o(:) + Tp(1:3,4);
                a = Tp(1:3,1:3)*j.axis(:);
                aw.(j.name) = a / norm(a);
            end
        end

        function A = ancestors(obj, linkName)
        %从根到该 link 的**可动**关节名（按链序）。
        %   constructor 里已预计算到 obj.p.anc —— 热路径（coriolis 会对 M 做
        %   2*nq 次中心差分，每次都过 jacobians）里不能反复做字符串比较。
        %   ⚠ 不要用 persistent 缓存：多个对象（不同 URDF）会串味。
            A = obj.p.anc.(linkName);
        end
    end
end

% ------------------------------------------------------------------ 局部函数
function R = rotm(a, q)
%ROTM  Rodrigues 旋转矩阵（a 内部归一）
a = a(:);
a = a / norm(a);
K = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
R = eye(3) + sin(q)*K + (1-cos(q))*(K*K);
end
