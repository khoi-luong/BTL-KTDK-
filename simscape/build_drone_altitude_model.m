%% build_drone_altitude_model.m
% Ghep bo dieu khien do cao PID 2 bac tu do + actuator BLDC (bldc_altitude_sim.m)
% vao model 3D Simscape Multibody cua drone.
%
%   H_ref --> [PID + chong windup + A^-1] --n_c--> [1/(tau s+1)] --n--> [A(n)] --T--> 4 canh quat
%                ^                                                                   |
%                +---------- z, v  (Transform Sensor World -> Body) <---- Drone 6-DOF+ nhieu -8 N (70-72 s)
%
% Cach dung:
%   >> build_drone_altitude_model      % tao SimpleDrone_altitude.slx
%   >> H = 75;  sim('SimpleDrone_altitude')   % hoac nhan Run (mac dinh H = 50)
%   >> run_altitude_compare            % chay 50/75/100 m va so voi bldc_altitude_sim

build_drone_model;                     % tao SimpleDrone_sim.slx (co lenh clear ben trong)
drone_params;

src = 'SimpleDrone_sim';
mdl = 'SimpleDrone_altitude';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
save_system(src, mdl);                 % "Save As" -> model dang mo doi ten thanh mdl
set_param(mdl, 'PreLoadFcn', 'drone_params;', ...
               'InitFcn',    'drone_params;', ...
               'StopTime', '150', 'ReturnWorkspaceOutputs', 'on');

%% 1. Bo khoi lenh luc day hang so cu
lh = get_param([mdl '/Thrust_cmd'], 'LineHandles');
delete_line(lh.Outport);
delete_block([mdl '/Thrust_cmd']);

%% 2. Cam bien z, v (World -> Body)
B = [mdl '/Body'];  W = [mdl '/World'];
S = addb(mdl, 'sm_lib/Frames and Transforms/Transform Sensor', 'Z Sensor', [270 1000]);
set_param(S, 'MeasurementFrame', 'World', 'SenseZ', 'on', 'SenseZDot', 'on');
haveVel = true;
pconnect(mdl, W, S, '', 'L1');
pconnect(mdl, B, S, '', 'R1');
ph = get_param(S, 'PortHandles');  outs = ph.RConn(2:end);

Cz = addb(mdl, 'nesl_utility/PS-Simulink Converter', 'SL_z', [400 1000], [30 30]);  setp(Cz, 'Unit', 'm');
pc = get_param(Cz, 'PortHandles');  add_line(mdl, outs(1), pc.LConn(1), 'autorouting', 'on');
Zh = addb(mdl, 'simulink/Discrete/Zero-Order Hold', 'ZOH_z', [470 1000], [30 30]);  set_param(Zh, 'SampleTime', 'P.dt');
add_line(mdl, 'SL_z/1', 'ZOH_z/1', 'autorouting', 'on');

if haveVel
    Cv = addb(mdl, 'nesl_utility/PS-Simulink Converter', 'SL_v', [400 1060], [30 30]);  setp(Cv, 'Unit', 'm/s');
    pc = get_param(Cv, 'PortHandles');  add_line(mdl, outs(2), pc.LConn(1), 'autorouting', 'on');
    vSrc = 'SL_v/1';
else
    % Khong co checkbox van toc -> dao ham loc cua z
    Dv = addb(mdl, 'simulink/Continuous/Transfer Fcn', 'dz_dt', [400 1060], [60 30]);
    set_param(Dv, 'Numerator', '[1 0]', 'Denominator', '[0.01 1]');
    add_line(mdl, 'SL_z/1', 'dz_dt/1', 'autorouting', 'on');
    vSrc = 'dz_dt/1';
    warning('Dung dao ham loc cua z thay cho cam bien van toc.');
end
Vh = addb(mdl, 'simulink/Discrete/Zero-Order Hold', 'ZOH_v', [470 1060], [30 30]);  set_param(Vh, 'SampleTime', 'P.dt');
add_line(mdl, vSrc, 'ZOH_v/1', 'autorouting', 'on');

%% 3. Bo dieu khien PID 2 bac tu do (MATLAB Function, roi rac dt = 0.005 s)
Hc = addb(mdl, 'simulink/Sources/Constant', 'H_ref', [470 900], [50 30]);   set_param(Hc, 'Value', 'H');
Gc = addb(mdl, 'simulink/Sources/Constant', 'PID_gains', [470 940], [80 30]); set_param(Gc, 'Value', 'PID.vec');
Tt = addb(mdl, 'simulink/Sources/Constant', 'T_tab', [470 1120], [60 30]);   set_param(Tt, 'Value', 'P.T_tab');
Rt = addb(mdl, 'simulink/Sources/Constant', 'rpm_tab', [470 1160], [60 30]); set_param(Rt, 'Value', 'P.rpm_tab');
Pc = addb(mdl, 'simulink/Sources/Constant', 'plant', [470 1200], [100 30]);  set_param(Pc, 'Value', '[P.m P.g P.Tmax P.dt]');

C = addb(mdl, 'simulink/User-Defined Functions/MATLAB Function', 'PID Altitude', [620 940], [140 220]);
rt = sfroot;  ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', C);
ch.Script = strjoin({
'function [nc, Tcmd] = pid_alt(H, z, v, G, Ttab, rtab, par)'
'% PID hai bac tu do + chong windup co dieu kien + bang nguoc A^-1 (bldc_altitude_sim)'
'persistent xi'
'if isempty(xi), xi = 0; end'
'Kp = G(1); Ki = G(2); Kd = G(3); b = G(4);'
'm = par(1); g = par(2); Tmax = par(3); dt = par(4);'
'e  = H - z;'
'ac = Kp*(b*H - z) + Ki*xi - Kd*v;'
'Tu = m*(g + ac)/4;'
'Tcmd = min(max(Tu, 0), Tmax);'
'satHi = Tu > Tmax;  satLo = Tu < 0;'
'if (~satHi && ~satLo) || (satHi && e < 0) || (satLo && e > 0)'
'    xi = xi + dt*e;'
'end'
'% A^-1: toc do can de dat luc day Tcmd'
'if Tcmd < Ttab(1)'
'    nc = rtab(1)*sqrt(max(Tcmd, 0)/Ttab(1));'
'elseif Tcmd >= Ttab(end)'
'    nc = rtab(end);'
'else'
'    k = 1;'
'    for j = 1:numel(Ttab)-1'
'        if Tcmd >= Ttab(j), k = j; end'
'    end'
'    nc = rtab(k) + (rtab(k+1) - rtab(k))/(Ttab(k+1) - Ttab(k))*(Tcmd - Ttab(k));'
'end'
}, newline);

add_line(mdl, 'H_ref/1',     'PID Altitude/1', 'autorouting', 'on');
add_line(mdl, 'ZOH_z/1',     'PID Altitude/2', 'autorouting', 'on');
add_line(mdl, 'ZOH_v/1',     'PID Altitude/3', 'autorouting', 'on');
add_line(mdl, 'PID_gains/1', 'PID Altitude/4', 'autorouting', 'on');
add_line(mdl, 'T_tab/1',     'PID Altitude/5', 'autorouting', 'on');
add_line(mdl, 'rpm_tab/1',   'PID Altitude/6', 'autorouting', 'on');
add_line(mdl, 'plant/1',     'PID Altitude/7', 'autorouting', 'on');

%% 4. Actuator BLDC: tau*dn/dt = n_c - n, n(0) = n_h ; T = A(n)
addb(mdl, 'simulink/Math Operations/Sum', 'Sum_n', [820 960], [20 20]);
set_param([mdl '/Sum_n'], 'Inputs', '+-', 'IconShape', 'round');
addb(mdl, 'simulink/Math Operations/Gain', 'Inv_tau', [860 955], [40 30]);
set_param([mdl '/Inv_tau'], 'Gain', '1/P.tau');
addb(mdl, 'simulink/Continuous/Integrator', 'Motor speed n', [930 955], [40 30]);
set_param([mdl '/Motor speed n'], 'InitialCondition', 'P.nh');
add_line(mdl, 'PID Altitude/1', 'Sum_n/1', 'autorouting', 'on');
add_line(mdl, 'Sum_n/1', 'Inv_tau/1', 'autorouting', 'on');
add_line(mdl, 'Inv_tau/1', 'Motor speed n/1', 'autorouting', 'on');
add_line(mdl, 'Motor speed n/1', 'Sum_n/2', 'autorouting', 'on');

LT = addb(mdl, 'simulink/Lookup Tables/1-D Lookup Table', 'Thrust A(n)', [1010 955], [60 40]);
set_param(LT, 'Table', 'P.T_bp', 'BreakpointsForDimension1', 'P.rpm_bp', 'ExtrapMethod', 'Clip');
add_line(mdl, 'Motor speed n/1', 'Thrust A(n)/1', 'autorouting', 'on');

Mx = addb(mdl, 'simulink/Signal Routing/Mux', 'Mux_T', [1100 930], [10 90]);
set_param(Mx, 'Inputs', '4');
for i = 1:4, add_line(mdl, 'Thrust A(n)/1', sprintf('Mux_T/%d', i), 'autorouting', 'on'); end
add_line(mdl, 'Mux_T/1', 'Demux/1', 'autorouting', 'on');      % vao 4 khoi Thrust1..4 co san

%% 5. Nhieu xung -8 N (70 s -> 72 s) tac dung len than, theo truc z
s1 = addb(mdl, 'simulink/Sources/Step', 'd_on',  [270 1250], [30 30]);
set_param(s1, 'Time', 'P.t1', 'Before', '0', 'After', 'P.d_amp');
s2 = addb(mdl, 'simulink/Sources/Step', 'd_off', [270 1300], [30 30]);
set_param(s2, 'Time', 'P.t2', 'Before', '0', 'After', '-P.d_amp');
addb(mdl, 'simulink/Math Operations/Sum', 'Sum_d', [340 1270], [20 20]);
set_param([mdl '/Sum_d'], 'Inputs', '++', 'IconShape', 'round');
add_line(mdl, 'd_on/1',  'Sum_d/1', 'autorouting', 'on');
add_line(mdl, 'd_off/1', 'Sum_d/2', 'autorouting', 'on');
Cd = addb(mdl, 'nesl_utility/Simulink-PS Converter', 'PS_d', [400 1270], [30 30]);  setp(Cd, 'Unit', 'N');
add_line(mdl, 'Sum_d/1', 'PS_d/1', 'autorouting', 'on');
Fd = addb(mdl, 'sm_lib/Forces and Torques/External Force and Torque', 'Disturbance', [480 1270]);
set_param(Fd, 'EnableForceZ', 'on');
setp(Fd, 'ForceResolutionFrame', 'World');
pconnect(mdl, B, Fd, '', 'R1');
psconnect(mdl, Cd, Fd);

%% 6. Ghi du lieu + Scope
F4 = addb(mdl, 'simulink/Math Operations/Gain', 'F_total', [1100 1060], [40 30]);  set_param(F4, 'Gain', '4');
add_line(mdl, 'Thrust A(n)/1', 'F_total/1', 'autorouting', 'on');
logw(mdl, 'z_sim',  'SL_z/1',          [560 1340]);
logw(mdl, 'v_sim',  vSrc,              [560 1390]);
logw(mdl, 'n_sim',  'Motor speed n/1', [1010 1060]);
logw(mdl, 'F_sim',  'F_total/1',       [1180 1060]);
logw(mdl, 'Tc_sim', 'PID Altitude/2',  [820 1100]);

Mz = addb(mdl, 'simulink/Signal Routing/Mux', 'Mux_zH', [620 1240], [10 50]);  set_param(Mz, 'Inputs', '2');
add_line(mdl, 'SL_z/1',  'Mux_zH/1', 'autorouting', 'on');
add_line(mdl, 'H_ref/1', 'Mux_zH/2', 'autorouting', 'on');
addb(mdl, 'simulink/Sinks/Scope', 'Altitude z & H', [680 1245], [40 40]);
add_line(mdl, 'Mux_zH/1', 'Altitude z & H/1', 'autorouting', 'on');
addb(mdl, 'simulink/Sinks/Scope', 'Motor speed (rpm)', [1180 1110], [40 40]);
add_line(mdl, 'Motor speed n/1', 'Motor speed (rpm)/1', 'autorouting', 'on');

try, Simulink.BlockDiagram.arrangeSystem(mdl); catch, end
save_system(mdl);
% Model trung gian SimpleDrone_sim chi dung de dung model nay -> xoa
if bdIsLoaded(src), close_system(src, 0); end
if exist([src '.slx'], 'file'), delete([src '.slx']); end
fprintf('\nDa tao %s.slx  (H = %g m, Kp = %.5f, Ki = %.6f, Kd = %.5f, b = %.5f)\n', ...
        mdl, H, PID.Kp, PID.Ki, PID.Kd, PID.b);
fprintf('Doi do cao: H = 75; roi nhan Run.  So sanh: run_altitude_compare\n');

%% =================== Ham phu tro ===================
function logw(mdl, var, src, xy)
    h = addb(mdl, 'simulink/Sinks/To Workspace', var, xy, [70 25]);
    set_param(h, 'VariableName', var, 'SaveFormat', 'Structure With Time');
    add_line(mdl, src, [var '/1'], 'autorouting', 'on');
end

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

function enableCheck(blk, rx)
    dp = get_param(blk, 'DialogParameters'); nm = fieldnames(dp);
    hit = nm(~cellfun(@isempty, regexp(nm, rx, 'once')));
    hit = hit(cellfun(@(f) strcmp(dp.(f).Type, 'checkbox'), hit));
    if isempty(hit)
        warning('%s: khong tim thay checkbox "%s". Tham so: %s', blk, rx, strjoin(nm', ', '));
    else
        set_param(blk, hit{1}, 'on');
    end
end

function setPopup(blk, rx, val)
    dp = get_param(blk, 'DialogParameters'); nm = fieldnames(dp);
    hit = nm(~cellfun(@isempty, regexp(nm, rx, 'once')));
    hit = hit(cellfun(@(f) strcmp(dp.(f).Type, 'enum') || strcmp(dp.(f).Type, 'popup'), hit));
    for k = 1:numel(hit)
        try, set_param(blk, hit{k}, val); return; catch, end
    end
    warning('%s: khong dat duoc frame = %s (than drone gan nhu khong nghieng nen anh huong nho).', blk, val);
end

function pconnect(sys, a, b, pa, pb)
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
    pc = get_param(conv, 'PortHandles');
    src = pc.RConn(1);
    ph = get_param(blk, 'PortHandles');
    for y = [ph.LConn ph.RConn]
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
        c = p;  all(all == p) = [];
    elseif numel(all) == 1
        c = all; all = [];
    end
    for h = all
        if get_param(h, 'Line') == -1, c(end+1) = h; end %#ok<AGROW>
    end
    if isempty(c), c = [ph.LConn ph.RConn]; end
end
