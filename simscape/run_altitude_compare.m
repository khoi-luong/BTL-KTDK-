%% run_altitude_compare.m
% Chay SimpleDrone_altitude.slx (Simscape 3D) o H = 50/75/100 m va so sanh voi
% mo phong goc bldc_altitude_sim.m (Euler, chi phuong dung).

here = fileparts(mfilename('fullpath'));
addpath(here, fileparts(here));        % model + bldc_altitude_sim.m
resDir = fullfile(fileparts(here), 'results');

Hs = [50 75 100];
mdl = 'SimpleDrone_altitude';
if ~exist(fullfile(here, [mdl '.slx']), 'file'), build_drone_altitude_model; end
load_system(mdl);

fprintf('Chay mo phong goc bldc_altitude_sim ...\n');
Rref = bldc_altitude_sim(false);

col = [0.12 0.35 0.65; 0.80 0.30 0.15; 0.25 0.55 0.30];
fig = figure('Name', 'Simscape 3D vs bldc_altitude_sim', 'Color', 'w', 'Position', [80 40 950 780]);
ax1 = subplot(3,1,1); hold on; grid on; ylabel('Do cao z (m)');
title('Do cao: net lien = Simscape 3D, net dut = bldc\_altitude\_sim');
ax2 = subplot(3,1,2); hold on; grid on; ylabel('Toc do dong co (vong/phut)');
ax3 = subplot(3,1,3); hold on; grid on; ylabel('Tong luc day (N)'); xlabel('Thoi gian (s)');

fprintf('\n%-6s %-22s %-22s %-22s\n', 'H(m)', 'z cuoi (Simscape/goc)', 'sut max sau nhieu', 'v leo max (m/s)');
for k = 1:numel(Hs)
    H = Hs(k);  assignin('base', 'H', H);
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('H', H);
    try, in = in.setModelParameter('SimMechanicsOpenEditorOnUpdate', 'off'); catch, end
    fprintf('Simscape: H = %g m ...\n', H);
    out = sim(in);

    zs = out.get('z_sim');  ns = out.get('n_sim');  Fs = out.get('F_sim');  vs = out.get('v_sim');
    t  = zs.time;  z = zs.signals.values;  v = vs.signals.values;

    plot(ax1, t, z, 'LineWidth', 1.8, 'Color', col(k,:));
    plot(ax1, Rref(k).t, Rref(k).z, '--', 'LineWidth', 1.2, 'Color', col(k,:));
    plot(ax2, ns.time, ns.signals.values, 'LineWidth', 1.5, 'Color', col(k,:));
    plot(ax2, Rref(k).t, Rref(k).n, '--', 'LineWidth', 1.0, 'Color', col(k,:));
    plot(ax3, Fs.time, Fs.signals.values, 'LineWidth', 1.5, 'Color', col(k,:));
    plot(ax3, Rref(k).t, Rref(k).F, '--', 'LineWidth', 1.0, 'Color', col(k,:));

    post = t >= 70 & t <= 100;
    sag  = max(H - z(post));
    fprintf('%-6g %8.3f / %-11.3f %8.2f / %-11.2f %8.2f / %-11.2f\n', H, z(end), Rref(k).z(end), ...
            sag, Rref(k).metrics.sag, max(v), Rref(k).metrics.vmax);
end
for ax = [ax1 ax2 ax3]
    yl = get(ax, 'YLim');
    patch(ax, [70 72 72 70], [yl(1) yl(1) yl(2) yl(2)], [1 0.85 0.3], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
    set(ax, 'YLim', yl);
end
legend(ax1, {'50 m Simscape','50 m goc','75 m Simscape','75 m goc','100 m Simscape','100 m goc'}, ...
       'Location', 'southeast');
print(fig, fullfile(resDir, 'so_sanh_simscape.png'), '-dpng', '-r150');
fprintf('\nDa luu results\\so_sanh_simscape.png\n');
