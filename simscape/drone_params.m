%% drone_params.m
% Tat ca thong so cho model Simscape (SimpleDrone_altitude.slx):
%   Phan 1 - hinh hoc / khoi luong drone
%   Phan 2 - bo dieu khien do cao + actuator BLDC (truoc day la altitude_params.m)
% Thong so hinh hoc/khoi luong drone (don vi SI) cho model Simscape.
% Da chinh theo drone 5 kg, canh 16x5.4 CF, dong co U5 KV400 (bldc_altitude_sim.m).
% Kich thuoc la uoc luong - chinh lai theo SolidWorks (Evaluate > Mass Properties).
% File nay duoc goi tu dong khi mo/chay model (PreLoadFcn + InitFcn).

%% ===== PHAN 1: HINH HOC / KHOI LUONG =====

g = 9.80665;                    % gia toc trong truong [m/s^2] (giong bldc_altitude_sim)
m_target = 5;                   % tong khoi luong cat canh [kg]

% ----- Than (Body.SLDPRT) -----
body.L = 0.22;                  % dai theo x [m]
body.W = 0.22;                  % rong theo y [m]
body.H = 0.08;                  % cao theo z [m]
body.color = [0.25 0.25 0.30];

% ----- Canh tay (Arm.SLDPRT) x4 -----
arm.len = 0.32;                 % chieu dai [m]
arm.w   = 0.03;                 % be rong [m]
arm.h   = 0.02;                 % be day [m]
arm.m   = 0.10;                 % khoi luong 1 canh tay [kg]
arm.color = [0.10 0.10 0.10];
arm.angle = [45 135 225 315];   % goc dat canh tay (cau hinh X) [deg]

% ----- Vi tri motor / truc canh quat -----
motor.r = 0.40;                 % tam drone -> truc motor [m] (wheelbase 800 mm)
motor.z = body.H/2;             % do cao mat tren canh tay so voi tam [m]
motor.R = 0.026;                % ban kinh motor U5 [m]
motor.len = 0.03;               % chieu cao motor [m]
motor.m = 0.195;                % khoi luong 1 motor [kg]
motor.color = [0.7 0.7 0.7];

% ----- Canh quat (Propeller.SLDPRT) x4 -----
prop.D = 16*0.0254;             % duong kinh 16 inch [m]
prop.chord = 0.035;             % be rong canh [m]
prop.t = 0.006;                 % be day [m]
prop.m = 0.03;                  % khoi luong [kg]
prop.color = [0.85 0.35 0.10];
prop.dir = [1 -1 1 -1];         % chieu quay: +1 CCW, -1 CW

% ----- Chan (Leg.SLDPRT) x2 -----
leg.L = 0.30;                   % dai theo x [m]
leg.w = 0.02;                   % rong [m]
leg.h = 0.20;                   % cao [m]
leg.m = 0.10;                   % khoi luong [kg]
leg.y = 0.12;                   % vi tri 2 chan theo +/- y [m]
leg.color = [0.15 0.15 0.15];

% ----- Than = phan con lai de tong khoi luong dung m_target -----
body.m = m_target - (4*(arm.m + motor.m + prop.m) + 2*leg.m);

% ----- Luc day / momen canh quat (quanh diem treo) -----
kQ = 1.0979e-6;                 % N.m/(rad/s)^2 (tu bang thu, bldc_altitude_sim)
kT = 6.57e-5;                   % N/(rad/s)^2  (~12.26 N tai 4123 vong/phut)

% ----- Tong khoi luong & luc day hover -----
m_total = body.m + 4*(arm.m + motor.m + prop.m) + 2*leg.m;
T_hover = m_total*g/4;          % luc day moi canh quat de bay treo [N]
w_hover = sqrt(T_hover/kT);     % toc do quay hover [rad/s]

% ----- Dieu kien dau -----
init.z = 0;                     % do cao ban dau [m] (cat canh tu mat dat)

%% ===== PHAN 2: BO DIEU KHIEN DO CAO + ACTUATOR BLDC (lay tu bldc_altitude_sim.m) =====
% Doi do cao dat: gan bien H trong workspace truoc khi chay, vd: H = 75;

if ~exist('H', 'var') || ~isnumeric(H) || isempty(H), H = 50; end
clear P PID   % tranh trung ten bien

P.g  = 9.80665;   P.gf = P.g/1000;
P.m  = 5;
P.Vbus = 22.2;

% Bang thu tinh U5 KV400 + 16x5.4 CF, 22,2 V (moi dong co)
P.thr_tab = [50 65 75 85 100];
P.rpm_tab = [4050 4850 5400 5850 6250];
P.T_gf    = [1200 1750 2120 2650 2850];
P.I_tab   = [5.2 9.4 13.0 16.9 20.0];
P.T_tab   = P.T_gf*P.gf;
P.Tmax    = P.T_tab(end);

% Actuator
P.tau = 0.14;                               % s
P.dt  = 0.005;                              % s, chu ky bo dieu khien
P.Tend = 150;

% Nhieu xung -8 N tu 70 s den 72 s
P.d_amp = -8;  P.t1 = 70;  P.t2 = 72;

% Bang A(n) mo rong cho khoi 1-D Lookup: duoi vung thu noi ve goc theo n^2,
% tren vung thu giu Tmax (giong thrustOf trong code goc)
rb = linspace(0, P.rpm_tab(1), 41);  rb = rb(1:end-1);
P.rpm_bp = [rb, P.rpm_tab];
P.T_bp   = [P.T_tab(1)*(rb/P.rpm_tab(1)).^2, P.T_tab];

% Toc do treo n_h = A^-1(m g / 4)
Th = P.m*P.g/4;
k = find(Th >= P.T_tab, 1, 'last');
P.nh = P.rpm_tab(k) + (P.rpm_tab(k+1) - P.rpm_tab(k))/(P.T_tab(k+1) - P.T_tab(k))*(Th - P.T_tab(k));

% He so PID dat cuc: D(s) = tau*(s+p)^3*(s+lam)  (p theo H nhu code goc)
p_alt = interp1([50 75 100], [0.25 0.20 0.18], min(max(H, 50), 100));
lam   = 1/P.tau - 3*p_alt;
PID.Kd = P.tau*(3*p_alt^2 + 3*p_alt*lam);
PID.Kp = P.tau*(p_alt^3 + 3*p_alt^2*lam);
PID.Ki = P.tau*p_alt^3*lam;
PID.b  = P.tau*p_alt^3/PID.Kp;
PID.vec = [PID.Kp PID.Ki PID.Kd PID.b];

clear rb k Th lam
