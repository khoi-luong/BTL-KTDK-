%% main_drone.m  -  Diem bat dau cho thu muc Drone (xem README.md)
% Chay tung phan bang "Run Section" (Ctrl+Enter), hoac Run de chay het.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'CAD'), fullfile(here, 'simscape'));
cd(here);
resDir = fullfile(here, 'results');
if ~exist(resDir, 'dir'), mkdir(resDir); end

%% 1. Mo phong 3 do cao (50 / 75 / 100 m): in bang chi tieu, ve hinh 1-4
%     savePng = true -> luu results\hinh_1.png ... hinh_4.png
R = bldc_altitude_sim(true, true);

%% 2. Luu so lieu 3 do cao: results\altitude_results.mat va altitude_H50/75/100.csv
save(fullfile(resDir, 'altitude_results.mat'), 'R');
for k = 1:numel(R)
    T = table(R(k).t, R(k).z, R(k).v, R(k).n, R(k).F, R(k).thr, ...
        'VariableNames', {'t_s', 'z_m', 'v_mps', 'n_rpm', 'F_N', 'throttle_pct'});
    writetable(T, fullfile(resDir, sprintf('altitude_H%g.csv', R(k).H)));
end
fprintf('Da luu so lieu vao %s\n', resDir);

%% 3. Hoat hinh 3-D, xem nhanh 1 do cao (doi 50 -> 75 hoac 100)
animate_drone_3d(50, 'Speed', 10);

%% 4. Luu video 3 do cao: results\drone_animation_H50/75/100.mp4
for H = [50 75 100]
    animate_drone_3d(H, 'Speed', 10, 'SaveVideo', true);
    close(gcf);
end

%% 5. (Tuy chon) Model Simscape 3-D, so sanh voi mo phong goc -> results\so_sanh_simscape.png
% run_altitude_compare

%% 6. (Tuy chon) Sau khi sua file SolidWorks: xuat lai STL bang Inventor
% export_drone_stl
