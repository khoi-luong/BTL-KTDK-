function animate_drone_3d(H, varargin)
%ANIMATE_DRONE_3D  3-D animation of the altitude simulation (bldc_altitude_sim.m)
%   using the real SolidWorks drone geometry (SimpleDrone.SLDASM, exported to .\stl).
%
%   animate_drone_3d            % H = 50 m, 10x real time
%   animate_drone_3d(75)        % H = 75 m
%   animate_drone_3d(100, 'Speed', 20, 'SaveVideo', true)   % also writes an .mp4
%
%   Options
%     'Speed'      playback speed vs real time (default 10)
%     'FPS'        frames per second (default 25)
%     'SaveVideo'  true -> results\drone_animation_H<H>.mp4 (default false)
%     't', 'z', 'n'  time [s], altitude [m], motor speed [rpm] from another run
%                    (e.g. the Simscape model) instead of bldc_altitude_sim
%
%   Geometry: CAD\stl (made by CAD\export_drone_stl.m from the SolidWorks files).
%   Without it, the drone is drawn from the dimensions in simscape\drone_params.m.
%
%   The axes show X, Y, Z in metres like a standard drone animation. Because the
%   drone (~1 m) is small next to the flight height (50-100 m), the Z axis is
%   compressed; the drone itself is drawn with a matching vertical stretch so it
%   keeps its true shape. The Z tick values are the real altitude.

if nargin < 1 || isempty(H), H = 50; end
op = struct('Speed', 10, 'FPS', 25, 'SaveVideo', false, 't', [], 'z', [], 'n', []);
for k = 1:2:numel(varargin), op.(varargin{k}) = varargin{k+1}; end

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end

%% 1. Simulation data ---------------------------------------------------------------
if isempty(op.t)
    R = bldc_altitude_sim(false);
    k = find([R.H] == H, 1);
    if isempty(k)
        error('H must be one of %s (the heights in bldc_altitude_sim).', mat2str([R.H]));
    end
    t = R(k).t;  z = R(k).z;  n = R(k).n;
else
    t = op.t(:);  z = op.z(:);  n = op.n(:);
    if isempty(n), n = 4123*ones(size(t)); end
end
nh = 4122.73;                                    % hover speed [rpm], for the spin rate

%% 2. Drone geometry -------------------------------------------------------------------
G = loadDroneGeometry(here);

%% 3. Figure and axes --------------------------------------------------------------------
L    = max(1.5, ceil(1.2*G.halfSpan*2)/2);       % half-width of the X/Y box [m]
Ztop = ceil((max(z)*1.1 + 5)/10)*10;             % top of the Z axis [m]
boxH = 1.6;                                      % plot box: height / width
a    = Ztop/(2*L*boxH);                          % z data units per x data unit

fig = figure('Name', sprintf('Drone animation - H = %g m', H), 'Color', 'w', ...
             'Position', [100 60 760 820]);
try, theme(fig, 'light'); catch, end          % white plot even if MATLAB uses the dark theme
ax = axes(fig, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'GridColor', [0.15 0.15 0.15]);
hold(ax, 'on');  grid(ax, 'on');
set(ax, 'XLim', [-L L], 'YLim', [-L L], 'ZLim', [0 Ztop], ...
        'DataAspectRatio', [1 1 a], 'FontSize', 10, 'Projection', 'orthographic');
xlabel(ax, 'X[m]');  ylabel(ax, 'Y[m]');  zlabel(ax, 'Z[m]');
view(ax, 25, 40);
ttl = title(ax, '', 'FontWeight', 'bold', 'Color', 'k');

% Reference altitude: dashed square at z = H
plot3(ax, [-L L L -L -L], [-L -L L L -L], H*ones(1, 5), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
text(ax, L, -L, H, sprintf('  H = %g m', H), 'Color', [0.85 0.33 0.10], 'FontWeight', 'bold');

% Ground shadow
th = linspace(0, 2*pi, 60);
shadow = patch(ax, G.halfSpan*cos(th), G.halfSpan*sin(th), zeros(size(th)), [0 0 0], ...
               'FaceAlpha', 0.12, 'EdgeColor', 'none');

% Drone body: one transform for the whole drone, one per propeller for the spin
hDrone = hgtransform('Parent', ax);
for i = 1:numel(G.parts)
    p = G.parts(i);
    if p.isProp, continue; end
    patch('Parent', hDrone, 'Faces', p.F, 'Vertices', p.V, 'FaceColor', p.color, ...
          'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.45);
end
hProp = gobjects(0);  propC = zeros(0, 2);  propDir = [];
for i = 1:numel(G.parts)
    p = G.parts(i);
    if ~p.isProp, continue; end
    hp = hgtransform('Parent', hDrone);
    patch('Parent', hp, 'Faces', p.F, 'Vertices', p.V, 'FaceColor', p.color, ...
          'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.45);
    % translucent rotor disc, like a standard quadcopter animation
    c = p.center;  r = p.radius;
    patch('Parent', hDrone, 'XData', c(1) + r*cos(th), 'YData', c(2) + r*sin(th), ...
          'ZData', p.zTop*ones(size(th)), 'FaceColor', p.discColor, 'FaceAlpha', 0.35, ...
          'EdgeColor', p.discColor*0.7);
    hProp(end+1) = hp;  propC(end+1, :) = c;  propDir(end+1) = p.dir; %#ok<AGROW>
end
camlight(ax, 'headlight');  lighting(ax, 'gouraud');

%% 4. Play ------------------------------------------------------------------------------
tf = 0:op.Speed/op.FPS:t(end);
zf = interp1(t, z, tf);
nf = interp1(t, n, tf);
S  = makehgtform('scale', [1 1 a]);              % keeps the drone's true shape

if op.SaveVideo
    vw = VideoWriter(fullfile(here, 'results', sprintf('drone_animation_H%g.mp4', H)), 'MPEG-4');
    vw.FrameRate = op.FPS;  open(vw);
end
ang = zeros(1, numel(hProp));
for f = 1:numel(tf)
    if ~isvalid(fig), break; end
    set(hDrone, 'Matrix', makehgtform('translate', [0 0 zf(f)])*S);
    ang = ang + propDir*0.9*nf(f)/nh;            % visual spin, faster when n is higher
    for i = 1:numel(hProp)
        c = propC(i, :);
        set(hProp(i), 'Matrix', makehgtform('translate', [c 0])*makehgtform('zrotate', ang(i)) ...
                                *makehgtform('translate', [-c 0]));
    end
    set(shadow, 'FaceAlpha', 0.12*max(0.15, 1 - zf(f)/Ztop));
    ttl.String = sprintf('Drone altitude control (H = %g m)\nt = %5.1f s    z = %6.2f m    n = %4.0f rpm', ...
                         H, tf(f), zf(f), nf(f));
    drawnow;
    if op.SaveVideo
        writeVideo(vw, getframe(fig));
    else
        pause(max(0, 1/op.FPS - 0.005));
    end
end
if op.SaveVideo
    close(vw);
    fprintf('Saved %s\n', vw.Filename);
end
end

% =====================================================================================
function G = loadDroneGeometry(here)
% Returns G.parts (V, F, color, isProp, center, radius, zTop, dir, discColor) in the
% drone frame: z up, centred on the body, lowest point (feet) at z = 0.
listFile = fullfile(here, 'CAD', 'stl', 'drone_parts.txt');
if isfile(listFile)
    G = fromStl(here, listFile);
else
    warning('No CAD\\stl\\drone_parts.txt - drawing the drone from drone_params.m. Run export_drone_stl to use the SolidWorks shape.');
    G = fromParams(here);
end
% rotor data
P = G.parts;
for i = 1:numel(P)
    if P(i).isProp
        V = P(i).V;
        c = [mean([min(V(:,1)) max(V(:,1))]) mean([min(V(:,2)) max(V(:,2))])];
        P(i).center = c;
        P(i).radius = max(hypot(V(:,1) - c(1), V(:,2) - c(2)));
        P(i).zTop   = max(V(:,3));
        q = atan2d(c(2), c(1));  q = mod(q, 360);          % 45/135/225/315 -> CCW/CW/CCW/CW
        P(i).dir = 1 - 2*mod(floor(q/90), 2);
        if P(i).dir > 0, P(i).discColor = [0.35 0.80 0.85]; else, P(i).discColor = [0.45 0.45 0.90]; end
    end
end
G.parts = P;
allV = vertcat(P.V);
G.halfSpan = max(hypot(allV(:,1), allV(:,2)));
end

function G = fromStl(here, listFile)
txt = strtrim(splitlines(fileread(listFile)));
txt = txt(~cellfun(@isempty, txt));
parts = struct('name', {}, 'V', {}, 'F', {}, 'color', {}, 'isProp', {}, ...
               'center', {}, 'radius', {}, 'zTop', {}, 'dir', {}, 'discColor', {});
for k = 1:numel(txt)
    c = strtrim(split(txt{k}, '|'));
    R = reshape(str2num(c{3}), 3, 3)';  %#ok<ST2NM>
    t = str2num(c{4});                  %#ok<ST2NM>
    box = str2num(c{5});                %#ok<ST2NM>
    TR = stlread(fullfile(here, 'CAD', 'stl', c{2}));
    V = double(TR.Points);  F = TR.ConnectivityList;
    % STL units -> metres: match the STL size to the CAD bounding box (metres)
    eS = max(V) - min(V);  eB = box(4:6) - box(1:3);
    ok = eS > 1e-9 & eB > 1e-9;
    s  = median(eB(ok)./eS(ok));
    cand = [1 0.1 0.01 0.001 0.0254];
    [~, j] = min(abs(log(cand/s)));  s = cand(j);
    V = (R*(V*s)')' + t;
    nm = lower(c{1});
    parts(end+1) = struct('name', c{1}, 'V', V, 'F', F, 'color', colorOf(nm), ...
        'isProp', contains(nm, 'prop'), 'center', [], 'radius', [], 'zTop', [], ...
        'dir', 1, 'discColor', [0 0 0]); %#ok<AGROW>
end
% Orientation: the drone is flat, so "up" is the axis with the smallest extent;
% the propellers sit above the centre, which gives the sign.
allV = vertcat(parts.V);
[~, up] = min(max(allV) - min(allV));
mid = (max(allV(:, up)) + min(allV(:, up)))/2;
pr = [parts.isProp];
if any(pr)
    PV = vertcat(parts(pr).V);
    sgn = sign(mean(PV(:, up)) - mid);
else
    sgn = 1;
end
if sgn == 0, sgn = 1; end
switch up
    case 1, Rup = [0 0 -1; 0 1 0; 1 0 0];   % x -> z
    case 2, Rup = [1 0 0; 0 0 -1; 0 1 0];   % y -> z
    otherwise, Rup = eye(3);
end
if sgn < 0, Rup = diag([1 -1 -1])*Rup; end
allV = (Rup*allV')';
isBody = contains(lower({parts.name}), 'body');
if any(isBody), cV = (Rup*vertcat(parts(isBody).V)')'; else, cV = allV; end
ctr = [(max(cV(:,1)) + min(cV(:,1)))/2, (max(cV(:,2)) + min(cV(:,2)))/2, min(allV(:,3))];
for k = 1:numel(parts)
    parts(k).V = (Rup*parts(k).V')' - ctr;
end
G.parts = parts;
end

function col = colorOf(nm)
if contains(nm, 'prop'),     col = [0.85 0.35 0.10];
elseif contains(nm, 'arm'),  col = [0.15 0.15 0.15];
elseif contains(nm, 'leg'),  col = [0.25 0.25 0.28];
else,                        col = [0.35 0.35 0.40];
end
end

function G = fromParams(here)
run(fullfile(here, 'simscape', 'drone_params.m'));
parts = struct('name', {}, 'V', {}, 'F', {}, 'color', {}, 'isProp', {}, ...
               'center', {}, 'radius', {}, 'zTop', {}, 'dir', {}, 'discColor', {});
add = @(parts, nm, V, F, col, isP) [parts, struct('name', nm, 'V', V, 'F', F, 'color', col, ...
        'isProp', isP, 'center', [], 'radius', [], 'zTop', [], 'dir', 1, 'discColor', [0 0 0])];
[V, F] = boxMesh([0 0 0], [body.L body.W body.H], 0);
parts = add(parts, 'Body', V, F, body.color, false);
for i = 1:4
    q = arm.angle(i);  rm = motor.r - arm.len/2;
    [V, F] = boxMesh([rm*cosd(q) rm*sind(q) motor.z - arm.h/2], [arm.len arm.w arm.h], q);
    parts = add(parts, 'Arm', V, F, arm.color, false);
    [V, F] = cylMesh([motor.r*cosd(q) motor.r*sind(q) motor.z], motor.R, motor.len);
    parts = add(parts, 'Motor', V, F, motor.color, false);
    [V, F] = boxMesh([motor.r*cosd(q) motor.r*sind(q) motor.z + motor.len + prop.t/2], ...
                     [prop.D prop.chord prop.t], 30*i);
    parts = add(parts, 'Propeller', V, F, prop.color, true);
end
for k = [-1 1]
    [V, F] = boxMesh([0 k*leg.y -(body.H/2 + leg.h/2)], [leg.L leg.w leg.h], 0);
    parts = add(parts, 'Leg', V, F, leg.color, false);
end
zmin = min(vertcat(parts.V));  zmin = zmin(3);
for k = 1:numel(parts), parts(k).V(:, 3) = parts(k).V(:, 3) - zmin; end
G.parts = parts;
end

function [V, F] = boxMesh(c, d, yaw)
[x, y, zz] = ndgrid([-1 1]*d(1)/2, [-1 1]*d(2)/2, [-1 1]*d(3)/2);
V = [x(:) y(:) zz(:)];
Rz = [cosd(yaw) -sind(yaw) 0; sind(yaw) cosd(yaw) 0; 0 0 1];
V = (Rz*V')' + c;
F = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
F = [F(:, [1 2 3]); F(:, [1 3 4])];
end

function [V, F] = cylMesh(c, r, h)
m = 24;  q = linspace(0, 2*pi, m + 1);  q(end) = [];
bot = [r*cos(q') r*sin(q') zeros(m, 1)];  top = bot;  top(:, 3) = h;
V = [bot; top; 0 0 0; 0 0 h] + c;
F = zeros(0, 3);
for k = 1:m
    k2 = mod(k, m) + 1;
    F = [F; k k2 m+k2; k m+k2 m+k; 2*m+1 k2 k; 2*m+2 m+k m+k2]; %#ok<AGROW>
end
end
