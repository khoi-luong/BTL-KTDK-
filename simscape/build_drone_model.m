%% build_drone_model.m
% Tu dong tao model Simscape Multibody cho quadcopter (cau hinh X):
%   World -> (do cao ban dau) -> 6-DOF Joint -> Body
%   Body  -> 4 Arm, 4 Motor, 4 Propeller (Revolute Joint), 2 Leg
%   Luc day 4 canh quat (External Force and Torque, truc z cua motor)
%   + momen phan luc quanh truc z (yaw).
%
% Yeu cau: MATLAB + Simulink + Simscape + Simscape Multibody.
% Cach dung: dat file nay cung thu muc voi drone_params.m, roi chay
%   >> build_drone_model
% Ket qua: SimpleDrone_sim.slx. Nhan Run -> Mechanics Explorer hien drone.
%
% Dau vao dieu khien: khoi "Thrust_cmd" (vector 4x1, don vi N). Mac dinh
% = T_hover cho ca 4 canh -> drone bay treo. Thay khoi nay bang bo dieu
% khien (PID/LQR...) cua ban.

clear; clc;
drone_params;

mdl = 'SimpleDrone_sim';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);
set_param(mdl, 'PreLoadFcn', 'drone_params;', 'InitFcn', 'drone_params;', ...
    'Solver', 'ode23t', 'StopTime', '10', 'MaxStep', '1e-2');

%% ---------- Khoi he thong ----------
W  = addb(mdl, 'sm_lib/Frames and Transforms/World Frame',      'World',        [30  300]);
MC = addb(mdl, 'sm_lib/Utilities/Mechanism Configuration',      'Mech Config',  [30  200]);
SC = addb(mdl, 'nesl_utility/Solver Configuration',             'Solver Config',[30  100]);
setp(MC, 'GravityVector', '[0 0 -g]');

pconnect(mdl, MC, W, '', '');
pconnect(mdl, SC, W, '', '');

% Do cao ban dau
T0 = addb(mdl, 'sm_lib/Frames and Transforms/Rigid Transform', 'Init Height', [150 300]);
rigid(T0, '[0 0 init.z]', 0);
pconnect(mdl, W, T0, '', 'L1');

% 6-DOF
J6 = addb(mdl, 'sm_lib/Joints/6-DOF Joint', '6-DOF Joint', [270 300]);
pconnect(mdl, T0, J6, 'R1', 'L1');

% Than
B = addb(mdl, 'sm_lib/Body Elements/Brick Solid', 'Body', [420 300]);
brick(B, '[body.L body.W body.H]', 'body.m', 'body.color');
pconnect(mdl, J6, B, 'R1', '');

%% ---------- Lenh luc day ----------
Tc = addb(mdl, 'simulink/Sources/Constant', 'Thrust_cmd', [270 650], [60 30]);
set_param(Tc, 'Value', 'T_hover*ones(4,1)');
Dm = addb(mdl, 'simulink/Signal Routing/Demux', 'Demux', [370 620], [10 120]);
set_param(Dm, 'Outputs', '4');
add_line(mdl, 'Thrust_cmd/1', 'Demux/1', 'autorouting', 'on');

%% ---------- 4 canh tay + motor + canh quat ----------
for i = 1:4
    y0 = 60 + (i-1)*230;
    x0 = 600;
    a = sprintf('arm.angle(%d)', i);

    % Canh tay: xoay quanh z theo goc, tinh tien den giua canh tay
    Ta = addb(mdl, 'sm_lib/Frames and Transforms/Rigid Transform', sprintf('Arm%d Frame', i), [x0 y0]);
    rigid(Ta, sprintf('[(motor.r-arm.len/2)*cosd(%s) (motor.r-arm.len/2)*sind(%s) motor.z-arm.h/2]', a, a), a);
    pconnect(mdl, B, Ta, '', 'L1');
    A = addb(mdl, 'sm_lib/Body Elements/Brick Solid', sprintf('Arm%d', i), [x0+120 y0]);
    brick(A, '[arm.len arm.w arm.h]', 'arm.m', 'arm.color');
    pconnect(mdl, Ta, A, 'R1', '');

    % Khung motor (truc z huong len), dat o dau canh tay
    Tm = addb(mdl, 'sm_lib/Frames and Transforms/Rigid Transform', sprintf('Motor%d Frame', i), [x0 y0+90]);
    rigid(Tm, sprintf('[motor.r*cosd(%s) motor.r*sind(%s) motor.z+motor.len/2]', a, a), 0);
    pconnect(mdl, B, Tm, '', 'L1');
    M = addb(mdl, 'sm_lib/Body Elements/Cylindrical Solid', sprintf('Motor%d', i), [x0+120 y0+90]);
    cyl(M, 'motor.R', 'motor.len', 'motor.m', 'motor.color');
    pconnect(mdl, Tm, M, 'R1', '');

    % Canh quat tren dinh motor
    Tp = addb(mdl, 'sm_lib/Frames and Transforms/Rigid Transform', sprintf('Prop%d Offset', i), [x0+240 y0+90]);
    rigid(Tp, '[0 0 motor.len/2+prop.t]', 0);
    pconnect(mdl, Tm, Tp, 'R1', 'L1');
    Jr = addb(mdl, 'sm_lib/Joints/Revolute Joint', sprintf('Prop%d Joint', i), [x0+360 y0+90]);
    pconnect(mdl, Tp, Jr, 'R1', 'L1');
    P = addb(mdl, 'sm_lib/Body Elements/Brick Solid', sprintf('Prop%d', i), [x0+480 y0+90]);
    brick(P, '[prop.D prop.chord prop.t]', 'prop.m', 'prop.color');
    pconnect(mdl, Jr, P, 'R1', '');

    % Luc day Fz + momen phan luc Tz tai khung motor
    F = addb(mdl, 'sm_lib/Forces and Torques/External Force and Torque', sprintf('Thrust%d', i), [x0+240 y0+170]);
    set_param(F, 'EnableForceZ', 'on', 'EnableTorqueZ', 'on');

    Cf = addb(mdl, 'nesl_utility/Simulink-PS Converter', sprintf('PS_F%d', i), [480 y0+160], [30 30]);
    Ct = addb(mdl, 'nesl_utility/Simulink-PS Converter', sprintf('PS_Q%d', i), [480 y0+200], [30 30]);
    setp(Cf, 'Unit', 'N');
    setp(Ct, 'Unit', 'N*m');
    % Momen phan luc: Q = -dir * (kQ/kT) * T
    G = addb(mdl, 'simulink/Math Operations/Gain', sprintf('Yaw%d', i), [420 y0+200], [30 30]);
    set_param(G, 'Gain', sprintf('-prop.dir(%d)*kQ/kT', i));

    add_line(mdl, sprintf('Demux/%d', i), sprintf('PS_F%d/1', i), 'autorouting', 'on');
    add_line(mdl, sprintf('Demux/%d', i), sprintf('Yaw%d/1', i), 'autorouting', 'on');
    add_line(mdl, sprintf('Yaw%d/1', i), sprintf('PS_Q%d/1', i), 'autorouting', 'on');

    pconnect(mdl, Tm, F, 'R1', 'R1');            % khung F cua luc -> khung motor
    psconnect(mdl, Cf, F);                       % fz
    psconnect(mdl, Ct, F);                       % tz
end

%% ---------- 2 chan ----------
s = [1 -1];
for k = 1:2
    Tl = addb(mdl, 'sm_lib/Frames and Transforms/Rigid Transform', sprintf('Leg%d Frame', k), [420 400 + 80*k]);
    rigid(Tl, sprintf('[0 %d*leg.y -(body.H/2+leg.h/2)]', s(k)), 0);
    pconnect(mdl, B, Tl, '', 'L1');
    L = addb(mdl, 'sm_lib/Body Elements/Brick Solid', sprintf('Leg%d', k), [520 400 + 80*k]);
    brick(L, '[leg.L leg.w leg.h]', 'leg.m', 'leg.color');
    pconnect(mdl, Tl, L, 'R1', '');
end

%% ---------- Do vi tri drone (x, y, z) ----------
try
    S = addb(mdl, 'sm_lib/Frames and Transforms/Transform Sensor', 'Pose Sensor', [270 450]);
    set_param(S, 'SenseX', 'on', 'SenseY', 'on', 'SenseZ', 'on');
    n = 3;
    pconnect(mdl, W, S, '', 'L1');
    pconnect(mdl, B, S, '', 'R1');
    ph = get_param(S, 'PortHandles');
    outs = ph.RConn(2:end);
    Mx = addb(mdl, 'simulink/Signal Routing/Mux', 'MuxPos', [520 560], [10 80]);
    set_param(Mx, 'Inputs', num2str(max(n,1)));
    for k = 1:numel(outs)
        C = addb(mdl, 'nesl_utility/PS-Simulink Converter', sprintf('SL_pos%d', k), [420 540 + 40*k], [30 30]);
        setp(C, 'Unit', 'm');
        pcC = get_param(C, 'PortHandles');
        add_line(mdl, outs(k), pcC.LConn(1), 'autorouting', 'on');
        add_line(mdl, sprintf('SL_pos%d/1', k), sprintf('MuxPos/%d', k), 'autorouting', 'on');
    end
    Sc = addb(mdl, 'simulink/Sinks/Scope', 'Position xyz', [580 580], [40 40]);
    add_line(mdl, 'MuxPos/1', 'Position xyz/1', 'autorouting', 'on');
catch err
    warning('Khong tu tao duoc khoi Transform Sensor (%s). Hay them bang tay neu can.', err.message);
end

%% ---------- Luu ----------
try, Simulink.BlockDiagram.arrangeSystem(mdl); catch, end
save_system(mdl);
fprintf('\nDa tao %s.slx  (m_total = %.3f kg, T_hover = %.3f N/canh)\n', mdl, m_total, T_hover);
fprintf('Nhan Run de mo Mechanics Explorer.\n');

%% =================== Ham phu tro ===================
function h = addb(mdl, lib, name, xy, sz)
    if nargin < 5, sz = [60 50]; end
    h = [mdl '/' name];
    add_block(lib, h, 'Position', [xy(1) xy(2) xy(1)+sz(1) xy(2)+sz(2)]);
end

function setp(blk, varargin)
    for k = 1:2:numel(varargin)
        try
            set_param(blk, varargin{k}, varargin{k+1});
        catch err
            warning('%s: khong dat duoc %s (%s)', blk, varargin{k}, err.message);
        end
    end
end

function rigid(blk, trans, angleDeg)
    if ~(ischar(angleDeg) || (isnumeric(angleDeg) && angleDeg == 0))
        angleDeg = num2str(angleDeg);
    end
    if ischar(angleDeg)
        setp(blk, 'RotationMethod', 'StandardAxis', 'RotationStandardAxis', '+Z', ...
                  'RotationAngle', angleDeg, 'RotationAngleUnits', 'deg');
    end
    setp(blk, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', trans, ...
              'TranslationCartesianOffsetUnits', 'm');
end

function brick(blk, dims, mass, color)
    setp(blk, 'BrickDimensions', dims, 'BrickDimensionsUnits', 'm', ...
              'InertiaType', 'CalculateFromGeometry', 'BasedOnType', 'Mass', ...
              'Mass', mass, 'MassUnits', 'kg', 'GraphicDiffuseColor', color);
end

function cyl(blk, R, len, mass, color)
    setp(blk, 'CylinderRadius', R, 'CylinderRadiusUnits', 'm', ...
              'CylinderLength', len, 'CylinderLengthUnits', 'm', ...
              'InertiaType', 'CalculateFromGeometry', 'BasedOnType', 'Mass', ...
              'Mass', mass, 'MassUnits', 'kg', 'GraphicDiffuseColor', color);
end

function enableCheck(blk, rx)
    % Bat checkbox co ten khop regex (ten tham so khong duoc MathWorks cong bo)
    dp = get_param(blk, 'DialogParameters'); nm = fieldnames(dp);
    hit = nm(~cellfun(@isempty, regexp(nm, rx, 'once')));
    hit = hit(cellfun(@(f) strcmp(dp.(f).Type, 'checkbox'), hit));
    if isempty(hit)
        warning('%s: khong tim thay checkbox khop "%s". Cac tham so: %s', blk, rx, strjoin(nm', ', '));
    else
        set_param(blk, hit{1}, 'on');
    end
end

function pconnect(sys, a, b, pa, pb)
    % Noi 2 cong vat ly. Thu cong uu tien truoc, sau do cac cong con trong.
    ca = cands(a, pa); cb = cands(b, pb);
    for x = ca
        for y = cb
            try
                add_line(sys, x, y, 'autorouting', 'on');
                return;
            catch
            end
        end
    end
    error('Khong noi duoc %s <-> %s', a, b);
end

function psconnect(sys, conv, blk)
    % Noi dau ra PS cua Simulink-PS Converter vao cong PS con trong cua blk
    pc = get_param(conv, 'PortHandles');
    src = pc.RConn(1);
    ph = get_param(blk, 'PortHandles');
    all = [ph.LConn ph.RConn];
    for y = all
        if get_param(y, 'Line') ~= -1, continue; end
        try
            add_line(sys, src, y, 'autorouting', 'on');
            return;
        catch
        end
    end
    error('Khong noi duoc %s -> %s', conv, blk);
end

function c = cands(blk, pref)
    ph = get_param(blk, 'PortHandles');
    all = [ph.LConn ph.RConn];
    c = [];
    if ~isempty(pref)
        if pref(1) == 'L', p = ph.LConn(str2double(pref(2:end)));
        else,               p = ph.RConn(str2double(pref(2:end))); end
        c = p;                                   % cong uu tien (cho phep re nhanh)
        all(all == p) = [];
    elseif numel(all) == 1
        c = all; all = [];
    end
    for h = all                                  % cac cong con lai: chi cong trong
        if get_param(h, 'Line') == -1, c(end+1) = h; end %#ok<AGROW>
    end
    if isempty(c), c = [ph.LConn ph.RConn]; end
end
