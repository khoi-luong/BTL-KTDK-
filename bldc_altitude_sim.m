function R = bldc_altitude_sim(doPlot, savePng)
%BLDC_ALTITUDE_SIM  Mo hinh va mo phong: BLDC (ESC + dong co + canh quat) lam actuator
%                   giu do cao quadcopter 5 kg, bo dieu khien PID hai bac tu do.
%
%   R = bldc_altitude_sim              chay 3 do cao 50/75/100 m, in bang, ve 4 hinh
%   R = bldc_altitude_sim(false)       chi chay va in bang, khong ve
%   R = bldc_altitude_sim(true, true)  ve va luu PNG (hinh_1.png ... hinh_4.png) vao thu muc results\
%
%   Dau ra R(k), k = 1..3 ung voi H = 50, 75, 100 m:
%       R(k).t, .z, .v, .n, .F, .Tc, .thr, .I, .P   chuoi thoi gian
%       R(k).G (he so PID), R(k).metrics (chi tieu)
%   Ve hoat hinh 3-D:  animate_drone(R(1).t, R(1).z)
%
%   CHI CO MOT DANG NHIEU: xung luc -8 N tu 70 s den truoc 72 s.
%   Ca ba do cao deu dung cung dang nhieu nay; khong co nhieu bac thang,
%   nhieu doc, doublet hoac sin trong ban nay.
%
%   KHONG can toolbox nao (chi dung ham co ban cua MATLAB).
%
% ----------------------------------------------------------------------------------
%  MO HINH (bam theo phan mo phong drone trong bao cao)
%    Actuator (ESC + BLDC + canh quat), mot dong co:
%        tau * dn/dt = n_c - n          (khau quan tinh bac nhat, do loi tinh = 1)
%        T = A(n)                       (luc day mot dong co, tra bang thu tinh cua hang)
%        tau = J / (Kt*Ke/R + Ba),  Ba = 2*kQ*w0   (tinh o phan 1, dung 0,14 s trong mo phong)
%    Doi tuong (chi chuyen dong thang dung):
%        m * dv/dt = F - m*g + d_z(t),   dz/dt = v,   F = 4*T
%    Bo dieu khien (tinh trong code, khong phai phan cung):
%        a_c = Kp*(b*H - z) + Ki*xi - Kd*v,   dxi/dt = H - z   (co chong windup)
%        T_u = m*(g + a_c)/4,  T_c = kep(T_u, 0, Tmax),  n_c = A^-1(T_c)
%    He so PID tu dat cuc:  D(s) = tau*s^4 + s^3 + Kd*s^2 + Kp*s + Ki = tau*(s+p)^3*(s+lam)
%
%  GIA DINH (can noi ro khi trinh bay): tau, J, L, I0 la gia dinh; do loi tinh n_c -> n bang 1;
%  4 dong co giong het, dien ap 22,2 V co dinh; bo qua luc can khi dong theo phuong dung;
%  cam bien do cao va van toc ly tuong.
% ----------------------------------------------------------------------------------

if nargin < 1, doPlot = true;  end
if nargin < 2, savePng = false; end

P = params();

%% 1. ACTUATOR: tinh hang so thoi gian tau tu thong so dong co -------------------------
Act = actuatorInfo(P);
printActuator(P, Act);

%% 2. DIEM TREO ----------------------------------------------------------------------
Th = P.m*P.g/4;
fprintf('\n=== DIEM TREO ===\n');
fprintf('Luc day moi dong co  T_h = m*g/4 = %g*%.5f/4 = %.4f N = %.1f gf\n', P.m, P.g, Th, Th/P.gf);
fprintf('Toc do moi dong co   n_h = %.2f vong/phut\n', P.nh);
fprintf('Ga (thang bang thu)  %.2f %%\n', thrOf(Th, P));
Ih = currentOf(Th, P);
fprintf('Dong phia nguon/nhanh %.3f A ; cong suat he day %.1f W\n', Ih, 4*P.Vbus*Ih);
fprintf('F_max / (m*g) = %.3f / %.3f = %.2f\n', 4*P.Tmax, P.m*P.g, 4*P.Tmax/(P.m*P.g));

%% 3. BA LAN CHAY CHINH: H = 50, 75, 100 m (nhieu xung -8 N tai giay 70-72) -------------
Hs = [50 75 100];
ps = [0.25 0.20 0.18];
dPulse = @(t) P.d_amp*(t >= P.t1 && t < P.t2);

fprintf('\n=== HE SO PID (dat cuc) ===\n');
fprintf('%-6s %-6s %-10s %-11s %-13s %-10s %-9s\n', 'H(m)', 'p', 'lambda', 'Kp', 'Ki', 'Kd', 'b');
for k = 1:3
    G = pidGains(ps(k), P.tau);
    S = runSim(Hs(k), G, P, dPulse, P.dt, P.Tend);
    S.I = arrayfun(@(T) currentOf(T/4, P), S.F);          % dong phia nguon moi nhanh (quy doi tinh)
    S.P = 4*P.Vbus*S.I;                                   % cong suat he day
    S.H = Hs(k);  S.G = G;
    S.metrics = calcMetrics(S, Hs(k), P);
    if k == 1, R = S; else, R(k) = S; end %#ok<AGROW>
    fprintf('%-6g %-6.2f %-10.6f %-11.6f %-13.8f %-10.6f %-9.6f\n', Hs(k), G.p, G.lam, G.Kp, G.Ki, G.Kd, G.b);
end

printMetrics(R, P);

%% 4. KIEM TRA ON DINH: Bode vong ho, cuc va bang Routh --------------
G50 = R(1).G;
w = logspace(-3, 3, 60000);
Fr = cell(3, 1);
fprintf('\n=== BIEN ON DINH VONG HO  L(s) = (Kd s^2 + Kp s + Ki)/(s^3 (tau s + 1)) ===\n');
fprintf('%-6s %-12s %-12s %-18s %-10s\n', 'H(m)', 'wc (rad/s)', 'PM (do)', 'w pha=-180 (rad/s)', '|L| o do');
for k = 1:3
    Fr{k} = loopFreq(R(k).G, P, w);
    fprintf('%-6g %-12.3f %-12.1f %-18.3f %-10.2f\n', Hs(k), Fr{k}.wc, Fr{k}.PM, Fr{k}.wpc, Fr{k}.Lpc);
end
fprintf('\nNghiem da thuc D(s) (p = 0,25): ');
Dc = [P.tau 1 G50.Kd G50.Kp G50.Ki];
fprintf('%.4f ', sort(real(roots(Dc)))); fprintf('\n');
b1 = G50.Kd - P.tau*G50.Kp;  c1 = G50.Kp - G50.Ki/b1;
fprintf('Bang Routh cot 1: %.5f  %.5f  %.5f  %.5f  %.6f (toan duong -> on dinh)\n', P.tau, 1, b1, c1, G50.Ki);

%% 5. VE DO THI -----------------------------------------------------------------------
if doPlot
    figActuator(P, Act, savePng);
    figAltitude(R, P, savePng);
    figElectrical(R, P, savePng);
    figBode(Fr, Hs, savePng);
end
end

% =====================================================================================
%  THAM SO
% =====================================================================================
function P = params()
P.g = 9.80665;   P.gf = P.g/1000;          % m/s^2 ; N tren gam-luc
P.m = 5;                                    % kg, khoi luong cat canh
P.Vbus = 22.2;                              % V, dien ap bus (dieu kien bang thu)

% Bang thu tinh cua hang: U5 KV400 + canh 16x5.4 CF, 22,2 V (moi hang = 1 dong co)
P.thr_tab = [50 65 75 85 100];              % % ga
P.rpm_tab = [4050 4850 5400 5850 6250];     % vong/phut
P.T_gf    = [1200 1750 2120 2650 2850];     % gam-luc
P.I_tab   = [5.2 9.4 13.0 16.9 20.0];       % A, dong phia nguon moi nhanh
P.T_tab   = P.T_gf*P.gf;                    % N
P.Tmax    = P.T_tab(end);                   % N, luc toi da moi dong co

% Dong co / canh quat
P.Kv = 400;                                 % vong/phut/V (hang)
P.Ke = 60/(2*pi*P.Kv);                      % V.s/rad
P.Kt = P.Ke;                                % N.m/A  (Kt = Ke theo don vi SI)
P.R  = 0.3288;                              % ohm, dien tro tuong duong (tinh tu hang 100 % ga)
P.L  = 60e-6;                               % H     GIA DINH
P.J  = 3.69e-4;                             % kg.m2 GIA DINH (roto + canh quat)
P.kQ = 1.0979e-6;                           % N.m/(rad/s)^2  (tinh tu bang thu)
P.I0 = 0.3;                                 % A     GIA DINH (ton hao khong tai)

% Mo phong
P.tau = 0.14;                               % s  hang so thoi gian actuator DUNG TRONG MO PHONG (gia dinh)
P.dt = 0.005;  P.Tend = 150;                % s
P.d_amp = -8;  P.t1 = 70;  P.t2 = 72;       % N, s, s : xung nhieu goc

P.nh = rpmOf(P.m*P.g/4, P);                 % toc do treo, vong/phut
end

% =====================================================================================
%  BANG TRA: A(n), A^-1(T), ga %, dong
% =====================================================================================
function T = thrustOf(n, P)
% Luc day mot dong co (N) theo toc do n (vong/phut). Duoi vung thu: noi ve goc theo n^2.
x = P.rpm_tab;  y = P.T_tab;
if n < x(1)
    T = y(1)*(n/x(1))^2;
elseif n >= x(end)
    T = y(end);
else
    k = find(n >= x, 1, 'last');
    T = y(k) + (y(k+1) - y(k))/(x(k+1) - x(k))*(n - x(k));
end
end

function n = rpmOf(T, P)
% Ham nguoc A^-1: toc do (vong/phut) can co de dat luc day T (N) moi dong co.
x = P.T_tab;  y = P.rpm_tab;
if T < x(1)
    n = y(1)*sqrt(max(T, 0)/x(1));
elseif T >= x(end)
    n = y(end);
else
    k = find(T >= x, 1, 'last');
    n = y(k) + (y(k+1) - y(k))/(x(k+1) - x(k))*(T - x(k));
end
end

function q = thrOf(T, P)
% Ga (%) theo thang bang thu ung voi luc day T (N) moi dong co.
x = T/P.gf;
if x >= P.T_gf(end)
    q = P.thr_tab(end);
elseif x >= P.T_gf(1)
    k = find(x >= P.T_gf, 1, 'last');
    q = P.thr_tab(k) + (P.thr_tab(k+1) - P.thr_tab(k))/(P.T_gf(k+1) - P.T_gf(k))*(x - P.T_gf(k));
else
    q = P.thr_tab(1)*x/P.T_gf(1);     % duoi bang: noi tuyen tinh ve 0 (chi de ve do thi)
end
end

function I = currentOf(T, P)
% Dong phia nguon moi nhanh (A) theo luc day moi dong co T (N): noi suy bang thu, giu nguyen o hai dau.
x = T/P.gf;
if x <= P.T_gf(1)
    I = P.I_tab(1);
elseif x >= P.T_gf(end)
    I = P.I_tab(end);
else
    k = find(x >= P.T_gf, 1, 'last');
    I = P.I_tab(k) + (P.I_tab(k+1) - P.I_tab(k))/(P.T_gf(k+1) - P.T_gf(k))*(x - P.T_gf(k));
end
end

% =====================================================================================
%  BO PID: dat cuc
% =====================================================================================
function G = pidGains(p, tau)
% D(s) = tau*s^4 + s^3 + Kd*s^2 + Kp*s + Ki  =  tau*(s+p)^3*(s+lam),  lam = 1/tau - 3p > 0
lam = 1/tau - 3*p;
G.p = p;  G.lam = lam;
G.Kd = tau*(3*p^2 + 3*p*lam);
G.Kp = tau*(p^3 + 3*p^2*lam);
G.Ki = tau*p^3*lam;
G.b  = tau*p^3/G.Kp;            % de tu so cua ham truyen kin = tau*p^3*(s+lam) khu duoc cuc -lam
end

% =====================================================================================
%  MO PHONG VONG KIN (Euler, thu tu cap nhat nhu luu do mo phong trong bao cao)
% =====================================================================================
function S = runSim(H, G, P, dfun, dt, Tend)
N = round(Tend/dt);
t = zeros(N+1, 1);  z = t;  v = t;  n = t;  F = t;  Tc = t;  thr = t;
zk = 0;  vk = 0;  xi = 0;  nk = P.nh;
clamped = false;
for k = 0:N
    tk = k*dt;
    e  = H - zk;
    ac = G.Kp*(G.b*H - zk) + G.Ki*xi - G.Kd*vk;     % PID hai bac tu do, dao ham theo van toc do
    Tu = P.m*(P.g + ac)/4;                          % bu trong luc, chia 4 dong co
    Tcmd = min(max(Tu, 0), P.Tmax);                 % kep luc
    nc = rpmOf(Tcmd, P);                            % tra bang nguoc
    t(k+1) = tk;  z(k+1) = zk;  v(k+1) = vk;  n(k+1) = nk;
    F(k+1) = 4*thrustOf(nk, P);  Tc(k+1) = Tcmd;  thr(k+1) = thrOf(Tcmd, P);
    if k == N, break; end
    satHi = Tu > P.Tmax;  satLo = Tu < 0;
    if satHi || satLo, clamped = true; end
    if (~satHi && ~satLo) || (satHi && e < 0) || (satLo && e > 0)   % chong windup co dieu kien
        xi = xi + dt*e;
    end
    nk = nk + dt/P.tau*(nc - nk);                   % quan tinh actuator
    Fk = 4*thrustOf(nk, P);                         % tra bang luc
    vk = vk + dt/P.m*(Fk - P.m*P.g + dfun(tk));     % dong luc hoc drone
    zk = zk + dt*vk;                                % dung van toc MOI v_{k+1}
    if zk < 0, zk = 0;  vk = 0; end                 % mat dat
end
S = struct('t', t, 'z', z, 'v', v, 'n', n, 'F', F, 'Tc', Tc, 'thr', thr, ...
           'I', [], 'P', [], 'H', H, 'G', G, 'metrics', [], 'clamped', clamped);
end

function M = calcMetrics(S, H, P)
t = S.t;  z = S.z;
pre = t < P.t1;
tpre = t(pre);  idx = find(abs(H - z(pre)) > 0.5);
if isempty(idx), M.t_band = 0; else, M.t_band = tpre(idx(end) + 1); end   % vao +-0,5 m va giu den truoc nhieu
M.vmax = max(S.v);
post = (t >= P.t1) & (t <= 100);
M.sag = max(H - z(post));
M.ovs = max(z(t > P.t2) - H);
tp = t(t >= P.t1);  idx2 = find(abs(H - z(t >= P.t1)) > 0.5);
if isempty(idx2), M.t_rec = 0; else, M.t_rec = tp(idx2(end) + 1) - P.t2; end
M.thr_max = max(S.thr);
M.z_end = z(end);
M.T_peak_gf = max(S.Tc)/P.gf;
% giai tich cho mo hinh tuyen tinh danh dinh: z(t) = H[1 - e^{-pt}(1 + pt + (pt)^2/2)]
p = S.G.p;
M.vmax_an = 2*H*p*exp(-2);
x = fzero(@(x) exp(-x)*(1 + x + x^2/2) - 0.5/H, [1 30]);
M.t_band_an = x/p;
end

% =====================================================================================
%  ACTUATOR: hang so thoi gian, mo hinh day du va rut gon
% =====================================================================================
function A = actuatorInfo(P)
A.w0   = (round(P.nh*100)/100)*2*pi/60;      % rad/s, n_h lam tron 2 chu so thap phan (4122,73) nhu trong tai lieu
A.TL   = P.kQ*A.w0^2;                        % mo men can canh quat
A.i    = A.TL/P.Kt + P.I0;                   % dong mo hinh mot chieu tuong duong
A.va   = P.Ke*A.w0 + P.R*A.i;
A.u    = A.va/P.Vbus;
A.Ba   = 2*P.kQ*A.w0;                        % he so can tuyen tinh hoa cua canh quat
A.hd   = P.Kt*P.Ke/P.R;                      % "ham dien" Kt*Ke/R
A.tau_e = P.L/P.R;
A.tau  = P.J/(A.hd + A.Ba);                  % hang so thoi gian cua mo hinh rut gon
A.ratio = A.tau/A.tau_e;
% mo hinh day du, trang thai x = [i; w]:  L di/dt = v - R i - Ke w ;  J dw/dt = Kt i - Ba w
A.Am = [-P.R/P.L, -P.Ke/P.L; P.Kt/P.J, -A.Ba/P.J];
A.Bm = [1/P.L; 0];
A.poles = sort(eig(A.Am));
end

function printActuator(P, A)
fprintf('=== ACTUATOR: VI DU SO TINH TAU (Muc 3.3 cua tai lieu) ===\n');
fprintf('Ke = Kt = 60/(2*pi*%g) = %.6f V.s/rad\n', P.Kv, P.Ke);
fprintf('w0 = %.2f*2*pi/60 = %.4f rad/s\n', P.nh, A.w0);
fprintf('T_L = kQ*w0^2 = %.4e*(%.4f)^2 = %.4e*%.1f = %.5f N.m\n', P.kQ, A.w0, P.kQ, A.w0^2, A.TL);
fprintf('i = T_L/Kt + I0 = %.5f/%.6f + %.1f = %.3f A\n', A.TL, P.Kt, P.I0, A.i);
fprintf('v_a = Ke*w0 + R*i = %.3f + %.4f*%.3f = %.3f V ;  u = %.3f\n', P.Ke*A.w0, P.R, A.i, A.va, A.u);
fprintf('tau_e = L/R = %.4e s = %.4f ms\n', A.tau_e, A.tau_e*1e3);
fprintf('Ba = 2*kQ*w0 = %.4e N.m.s ;  Kt*Ke/R = (%.6f)^2/%.4f = %.4e\n', A.Ba, P.Kt, P.R, A.hd);
fprintf('tau = J/(Kt*Ke/R + Ba) = %.3e/(%.4e + %.4e) = %.4f s  (mo phong dung %.2f s)\n', P.J, A.hd, A.Ba, A.tau, P.tau);
fprintf('tau/tau_e = %.0f ;  cuc rut gon = %.2f rad/s ;  cuc mo hinh day du = %.2f va %.1f rad/s\n', ...
        A.ratio, -1/A.tau, A.poles(end), A.poles(1));
end

function y = stepFull(A, t)
% Dap ung bac thang chuan hoa cua mo hinh day du (omega theo dien ap), tinh bang ma tran mu.
xss = -(A.Am\A.Bm);
y = zeros(size(t));
for q = 1:numel(t)
    x = A.Am\((expm(A.Am*t(q)) - eye(2))*A.Bm);
    y(q) = x(2)/xss(2);
end
end

% =====================================================================================
%  PHAN TICH TAN SO
% =====================================================================================
function Fr = loopFreq(G, P, w)
s  = 1i*w;
L  = (G.Kd*s.^2 + G.Kp*s + G.Ki)./(s.^3.*(P.tau*s + 1));
mag = abs(L);
ph  = angle(L)*180/pi;  ph(ph > 0) = ph(ph > 0) - 360;      % (-360, 0]
i1  = find(mag < 1, 1);
Fr.w = w;  Fr.mag = mag;  Fr.ph = ph;
Fr.wc = w(i1);  Fr.PM = 180 + ph(i1);
j = find(ph(1:end-1) < -180 & ph(2:end) >= -180, 1);          % pha cat -180 do di len (on dinh co dieu kien)
Fr.wpc = w(j);  Fr.Lpc = mag(j);
end

function printMetrics(R, P)
ref = [33.6 44.5 51.5; 3.38 4.06 4.87; 3.18 3.94 4.36; 1.13 1.38 1.52; 21.9 29.4 33.6; 56.64 55.98 55.67; 50.000 75.000 100.001];
names = {'Thoi gian vao vung +-0,5 m (s)', 'Van toc leo lon nhat (m/s)', 'Sut sau nhat sau nhieu (m)', ...
         'Vot lo sau khi het nhieu (m)', 'Thoi gian ve lai +-0,5 m (s)', 'Ga lon nhat (%)', 'Do cao cuoi 150 s (m)'};
fprintf('\n=== CHI TIEU DAP UNG: mo phong [gia tri trong bao cao] ===\n');
fprintf('%-34s %-18s %-18s %-18s\n', '', '50 m', '75 m', '100 m');
for r = 1:7
    fprintf('%-34s', names{r});
    for k = 1:3
        M = R(k).metrics;
        vals = [M.t_band, M.vmax, M.sag, M.ovs, M.t_rec, M.thr_max, M.z_end];
        if r == 7, fprintf(' %9.3f [%7.3f]  ', vals(r), ref(r, k)); else, fprintf(' %9.2f [%7.2f]  ', vals(r), ref(r, k)); end
    end
    fprintf('\n');
end
fprintf('Giai tich (mo hinh tuyen tinh): t_vao_vung = ');  fprintf('%.2f  ', arrayfun(@(s) s.metrics.t_band_an, R));
fprintf('s ;  v_max = 2*H*p*e^-2 = ');                     fprintf('%.3f  ', arrayfun(@(s) s.metrics.vmax_an, R));
fprintf('m/s\n');
fprintf('Luc day yeu cau lon nhat moi dong co: %.0f / %.0f / %.0f gf  (gioi han %.0f gf)\n', ...
        R(1).metrics.T_peak_gf, R(2).metrics.T_peak_gf, R(3).metrics.T_peak_gf, P.T_gf(end));
end

function c = pal()
c = [0.12 0.35 0.65; 0.80 0.30 0.15; 0.25 0.55 0.30];
end

function shadeDist(ax, P)
yl = get(ax, 'YLim');
patch([P.t1 P.t2 P.t2 P.t1], [yl(1) yl(1) yl(2) yl(2)], [1.0 0.85 0.3], 'FaceAlpha', 0.35, ...
      'EdgeColor', 'none', 'Parent', ax);
ch = get(ax, 'Children');  set(ax, 'Children', [ch(2:end); ch(1)]);     % dua dai nhieu ra sau
set(ax, 'YLim', yl);
end

function saveFig(fig, name, savePng)
% Luu PNG vao thu muc results\ (canh file nay)
if savePng
    d = fullfile(fileparts(mfilename('fullpath')), 'results');
    if ~exist(d, 'dir'), mkdir(d); end
    print(fig, fullfile(d, name), '-dpng', '-r150');
end
end

function figActuator(P, A, savePng)
fig = figure('Name', 'Hinh 1 - Actuator: dap ung buoc', 'NumberTitle', 'off', 'Color', 'w', 'Position', [60 60 1000 420]);
col = pal();
t = linspace(0, 0.8, 400)';
yred = 1 - exp(-t/A.tau);
subplot(1, 2, 1);
plot(t, stepFull(A, t), 'LineWidth', 2.2, 'Color', col(1, :)); hold on;
plot(t, yred, '--', 'LineWidth', 1.6, 'Color', col(2, :));
plot(A.tau, 1 - exp(-1), 'ko', 'MarkerFaceColor', 'k');
text(A.tau + 0.02, 0.55, sprintf('t = tau = %.3f s\n63,2 %%', A.tau));
grid on; xlabel('Thoi gian (s)'); ylabel('\Delta\omega / \Delta\omega_{xac lap}');
title('Dap ung buoc cua actuator');
legend('mo hinh day du (bac 2)', 'rut gon 1/(\tau s+1)', 'Location', 'southeast');
subplot(1, 2, 2);
t2 = linspace(0, 2e-3, 400)';
plot(t2*1e3, stepFull(A, t2), 'LineWidth', 2.2, 'Color', col(1, :)); hold on;
plot(t2*1e3, 1 - exp(-t2/A.tau), '--', 'LineWidth', 1.6, 'Color', col(2, :));
grid on; xlabel('Thoi gian (ms)'); ylabel('\Delta\omega / \Delta\omega_{xac lap}');
title(sprintf('Phong to 2 ms dau (tau_e = %.2f ms)', A.tau_e*1e3));
saveFig(fig, 'hinh_1.png', savePng);
end

function figAltitude(R, P, savePng)
fig = figure('Name', 'Hinh 2 - Do cao, toc do dong co, ga', 'NumberTitle', 'off', 'Color', 'w', 'Position', [80 40 900 760]);
col = pal();  lab = {'50 m', '75 m', '100 m'};
ax1 = subplot(3, 1, 1); hold on;
for k = 1:3
    plot(R(k).t, R(k).z, 'LineWidth', 1.8, 'Color', col(k, :));
    tb = R(k).metrics.t_band;  zb = interp1(R(k).t, R(k).z, tb);
    plot(tb, zb, 'o', 'MarkerFaceColor', col(k, :), 'MarkerEdgeColor', 'k');
    text(tb + 1, zb - 6, sprintf('%.1f s', tb), 'Color', col(k, :));
end
ylabel('Do cao (m)'); title('Do cao drone (cham tron: vao vung \pm0,5 m)'); grid on; shadeDist(ax1, P);
legend(lab, 'Location', 'southeast');
ax2 = subplot(3, 1, 2); hold on;
for k = 1:3, plot(R(k).t, R(k).n, 'LineWidth', 1.6, 'Color', col(k, :)); end
plot([0 P.Tend], [P.nh P.nh], ':k');
ylabel('Toc do moi dong co (vong/phut)'); title('Toc do dong co'); grid on; shadeDist(ax2, P);
ax3 = subplot(3, 1, 3); hold on;
for k = 1:3, plot(R(k).t, R(k).thr, 'LineWidth', 1.6, 'Color', col(k, :)); end
plot([0 P.Tend], [thrOf(P.m*P.g/4, P) thrOf(P.m*P.g/4, P)], ':k');
xlabel('Thoi gian (s)'); ylabel('Ga (%)'); title('Lenh ga gui ESC (thang bang thu)'); grid on; shadeDist(ax3, P);
saveFig(fig, 'hinh_2.png', savePng);
end

function figElectrical(R, P, savePng)
fig = figure('Name', 'Hinh 3 - Dong, cong suat, luc day', 'NumberTitle', 'off', 'Color', 'w', 'Position', [100 40 900 760]);
col = pal();
ax1 = subplot(3, 1, 1); hold on;
for k = 1:3, plot(R(k).t, R(k).I, 'LineWidth', 1.6, 'Color', col(k, :)); end
ylabel('Dong (A)'); title('Dong phia nguon moi nhanh (quy doi tinh tu luc day)'); grid on; shadeDist(ax1, P);
legend('50 m', '75 m', '100 m', 'Location', 'northeast');
ax2 = subplot(3, 1, 2); hold on;
for k = 1:3, plot(R(k).t, R(k).P, 'LineWidth', 1.6, 'Color', col(k, :)); end
ylabel('Cong suat (W)'); title('Cong suat bon nhanh'); grid on; shadeDist(ax2, P);
ax3 = subplot(3, 1, 3); hold on;
for k = 1:3, plot(R(k).t, R(k).F, 'LineWidth', 1.6, 'Color', col(k, :)); end
plot([0 P.Tend], [P.m*P.g P.m*P.g], ':k');
xlabel('Thoi gian (s)'); ylabel('Luc day (N)'); title('Tong luc day bon canh (net dut: m g)'); grid on; shadeDist(ax3, P);
saveFig(fig, 'hinh_3.png', savePng);
end

function figBode(Fr, Hs, savePng)
fig = figure('Name', 'Hinh 4 - Bode vong ho, bien on dinh', 'NumberTitle', 'off', 'Color', 'w', 'Position', [140 40 900 700]);
col = pal();
subplot(2, 1, 1); hold on;
for k = 1:3, semilogx(Fr{k}.w, 20*log10(Fr{k}.mag), 'LineWidth', 1.8, 'Color', col(k, :)); end
set(gca, 'XScale', 'log'); plot([1e-3 1e3], [0 0], ':k');
xlim([1e-2 1e2]); ylim([-80 60]); grid on; ylabel('|L| (dB)');
title('Vong ho L(s) = C(s)*G_a(s)');
legend(arrayfun(@(h) sprintf('%g m', h), Hs, 'UniformOutput', false), 'Location', 'southwest');
subplot(2, 1, 2); hold on;
for k = 1:3, semilogx(Fr{k}.w, Fr{k}.ph, 'LineWidth', 1.8, 'Color', col(k, :)); end
set(gca, 'XScale', 'log'); plot([1e-3 1e3], [-180 -180], ':k');
xlim([1e-2 1e2]); ylim([-300 -100]); grid on; xlabel('\omega (rad/s)'); ylabel('Pha (do)');
for k = 1:3
    plot(Fr{k}.wc, -180 + Fr{k}.PM, 'o', 'MarkerFaceColor', col(k, :), 'MarkerEdgeColor', 'k');
    text(Fr{k}.wc*1.15, -180 + Fr{k}.PM - 6 - 9*(k - 1), sprintf('PM = %.1f do', Fr{k}.PM), 'Color', col(k, :));
end
saveFig(fig, 'hinh_4.png', savePng);
end
