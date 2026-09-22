function plot_results(result)
%PLOT Render a stored mission result without re-running the simulation.
if ~isfield(result,'entry')
    figure('Name','Orbital Rendezvous: Phases 1-2','Color','w');
    tiledlayout(2,2);
    nexttile;
    plot3(result.phasing.history.pos(1,:)/1e3,result.phasing.history.pos(2,:)/1e3, ...
        result.phasing.history.pos(3,:)/1e3); hold on;
    plot3(result.proximity.history.pos(1,:)/1e3,result.proximity.history.pos(2,:)/1e3, ...
        result.proximity.history.pos(3,:)/1e3);
    axis equal; grid on; xlabel('ECI X (km)'); ylabel('ECI Y (km)'); zlabel('ECI Z (km)');
    title('Orbital trajectory'); legend('Phasing / homing','Proximity');
    nexttile;
    r=result.proximity.relative_position;
    plot(r(2,:),r(1,:)); hold on; plot(0,0,'r+'); axis equal; grid on;
    xlabel('V (m)'); ylabel('R (m)'); title('Proximity in LVLH');
    nexttile;
    plot(result.phasing.history.time/60,result.phasing.history.mass); grid on;
    xlabel('Phase 1 time (min)'); ylabel('Stack mass (kg)'); title('Phasing mass budget');
    nexttile;
    plot(result.proximity.history.time/60,vecnorm(r)); grid on;
    xlabel('Phase 2 time (min)'); ylabel('Separation (m)'); title('Approach to standoff');
    sgtitle('ORBIT ONLY — atmospheric entry not executed');
    return;
end
sys = result.config.system;
hist_p1 = result.phasing.history;
hist_p2 = result.proximity.history;
hist_p3 = result.deorbit.history;
hist_reentry = result.entry.history;
hist_pos = result.proximity.relative_position;
phase2_targets = result.proximity.targets;
phase2_names = result.proximity.names;
phase2_target_times = result.proximity.target_times;
reentry_atmo_info = result.entry.summary;
% Plotting R-bar trajectory
figure('Name','Proximity Operations','Color','w');
plot(hist_pos(2,:), hist_pos(1,:), 'b-', 'LineWidth', 2); hold on;
plot(phase2_targets(2,:), phase2_targets(1,:), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'w');
plot(0,0,'r^','MarkerSize',10,'MarkerFaceColor','r');

% Waypoint time labels
for kk = 1:size(phase2_targets, 2)
    x_wp = phase2_targets(2, kk);   % V-bar
    y_wp = phase2_targets(1, kk);   % R-bar

    label_txt = sprintf('%s, %.1f min', ...
        char(phase2_names(kk)), phase2_target_times(kk)/60);

    text(x_wp, y_wp, ['  ' label_txt], ...
        'FontSize', 8, ...
        'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'left', ...
        'BackgroundColor', 'w', ...
        'Margin', 1);
end

grid on;
xlabel('V-bar (m)');
ylabel('R-bar (m)');
title('Proximity Approach Trajectory (LVLH)');
legend('Chaser Trajectory', 'Commanded Waypoints', 'Target', 'Location', 'best');
set(gca, 'XDir', 'reverse'); % Flight direction to the left

%% 6. Mission Profile Visualization
fprintf('\nGenerating visualization dashboard...\n');

figure('Name', 'Mission Comprehensive Dashboard', 'Color', 'w', 'Position', [100, 100, 1400, 800]);

% --- (1) 3D Mission Trajectory (ECI Frame) ---
subplot(2, 2, [1, 3]);
% Earth reference sphere.
[XE, YE, ZE] = sphere(50);
surf(XE*sys.Re/1e3, YE*sys.Re/1e3, ZE*sys.Re/1e3, 'FaceColor', [0.2 0.5 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
hold on; grid on; axis equal;

% Trajectories in km.
plot3(hist_p1.pos(1,:)/1e3, hist_p1.pos(2,:)/1e3, hist_p1.pos(3,:)/1e3, 'g-', 'LineWidth', 1.5); % Phase 1
plot3(hist_p2.pos(1,:)/1e3, hist_p2.pos(2,:)/1e3, hist_p2.pos(3,:)/1e3, 'b-', 'LineWidth', 1.5); % Phase 2
plot3(hist_p3.pos(1,:)/1e3, hist_p3.pos(2,:)/1e3, hist_p3.pos(3,:)/1e3, 'r-', 'LineWidth', 1.5); % Phase 3
plot3(hist_reentry.rv_pos(1,:)/1e3, hist_reentry.rv_pos(2,:)/1e3, hist_reentry.rv_pos(3,:)/1e3, 'm-', 'LineWidth', 1.5);
plot3(hist_reentry.chaser_pos(1,:)/1e3, hist_reentry.chaser_pos(2,:)/1e3, hist_reentry.chaser_pos(3,:)/1e3, 'c--', 'LineWidth', 1.0);

% Target-orbit reference line.
theta = linspace(0, 2*pi, 100);
plot3((sys.Re+sys.h_target)/1e3*cos(theta), zeros(1,100), (sys.Re+sys.h_target)/1e3*sin(theta), 'k--', 'LineWidth', 1);

title('3D Mission Trajectory (Earth Centered Inertial)');
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
legend('Earth', 'Phase 1: Phasing (Ascent)', 'Phase 2: R-bar Prox Ops', ...
       sprintf('Phase 3: De-orbit to %.0fkm', sys.h_entry_interface/1000), 'Phase 4: Atmospheric Entry RV', ...
       'Orbiting Target / Relay', sprintf('Target Orbit (%.0fkm)', sys.h_target/1000), 'Location', 'best');
view(45, 30);

% --- (2) Altitude vs Time Profile ---
subplot(2, 2, 2);
alt_p1 = vecnorm(hist_p1.pos) - sys.Re;
alt_p2 = vecnorm(hist_p2.pos) - sys.Re;
alt_p3 = vecnorm(hist_p3.pos) - sys.Re;
time_p1 = hist_p1.time / 3600;
time_p2 = hist_p2.time / 3600;
time_p3 = hist_p3.time / 3600;
time_reentry = hist_reentry.time / 3600;
entry_time_offset = time_p1(end) + time_p2(end) + time_p3(end);

plot(time_p1, alt_p1/1e3, 'g-', 'LineWidth', 2); hold on;
plot(time_p2 + time_p1(end), alt_p2/1e3, 'b-', 'LineWidth', 2);
% Continue Phase 3/4 time axes after Phase 1/2.
plot(time_p3 + time_p1(end) + time_p2(end), alt_p3/1e3, 'r-', 'LineWidth', 2);
plot(time_reentry + entry_time_offset, hist_reentry.altitude/1e3, 'm-', 'LineWidth', 2);
yline(sys.h_insert/1e3, 'k:', sprintf('Insertion (%.0fkm)', sys.h_insert/1000));
yline(sys.h_target/1e3, 'k--', sprintf('Target (%.0fkm)', sys.h_target/1000));
yline(sys.h_entry_interface/1e3, 'm:', sprintf('Entry Interface (%.0fkm)', sys.h_entry_interface/1000));
if reentry_atmo_info.altitude_termination_enabled
    yline(sys.reentry_vehicle.terminal_altitude/1e3, 'm--', 'Entry Stop');
elseif isfinite(reentry_atmo_info.safety_floor_altitude_m)
    yline(reentry_atmo_info.safety_floor_altitude_m/1e3, 'r:', 'Ground Safety');
end

title('Altitude Profile');
xlabel('Mission Time (Hours)'); ylabel('Altitude (km)');
legend('Phasing Maneuver', 'R-bar Prox Ops', sprintf('De-orbit to %.0fkm', sys.h_entry_interface/1000), 'Atmospheric Entry');
grid on;

% --- (3) Mass Depletion Profile ---
subplot(2, 2, 4);
plot(time_p1, hist_p1.mass, 'g-', 'LineWidth', 2); hold on;
plot(time_p2 + time_p1(end), hist_p2.mass, 'b-', 'LineWidth', 2);
plot(time_p3 + time_p1(end) + time_p2(end), hist_p3.mass, 'r-', 'LineWidth', 2);
plot(time_reentry + entry_time_offset, hist_reentry.mass, 'm-', 'LineWidth', 2);

title('Spacecraft Mass Depletion (Fuel Consumption)');
xlabel('Mission Time (Hours)'); ylabel('Mass (kg)');
legend('Fuel used in Phasing', 'Fuel used in Prox Ops', 'Fuel used in De-orbit', 'Atmospheric Entry RV');
grid on;

figure('Name', 'Atmospheric Re-entry Diagnostics', 'Color', 'w', 'Position', [150, 120, 1300, 820]);
entry_min = hist_reentry.time / 60;

subplot(3, 2, 1);
plot(entry_min, hist_reentry.altitude/1e3, 'm-', 'LineWidth', 2); grid on;
xlabel('Entry Time (min)'); ylabel('Altitude (km)');
title('Re-entry Altitude');

subplot(3, 2, 2);
plot(entry_min, hist_reentry.speed_rel/1e3, 'k-', 'LineWidth', 2); grid on;
xlabel('Entry Time (min)'); ylabel('Relative Speed (km/s)');
title('Atmosphere-relative Speed');

subplot(3, 2, 3);
plot(entry_min, hist_reentry.dynamic_pressure/1e3, 'b-', 'LineWidth', 2); grid on;
xlabel('Entry Time (min)'); ylabel('q (kPa)');
title('Dynamic Pressure');

subplot(3, 2, 4);
plot(entry_min, hist_reentry.heat_flux/1e4, 'r-', 'LineWidth', 2); grid on;
xlabel('Entry Time (min)'); ylabel('Heat Flux (W/cm^2)');
title('Sutton-Graves Surrogate Heat Flux');

subplot(3, 2, 5);
plot(entry_min, hist_reentry.g_load, 'Color', [0.2 0.5 0.2], 'LineWidth', 2); grid on;
xlabel('Entry Time (min)'); ylabel('Aero Accel (g)');
title('Aerodynamic g-load');

subplot(3, 2, 6);
yyaxis left;
plot(entry_min, hist_reentry.los_clearance/1e3, 'c-', 'LineWidth', 2); grid on;
ylabel('Earth-limb Clearance (km)');
yyaxis right;
plot(entry_min, hist_reentry.los_elevation_deg, 'Color', [0.4 0.2 0.8], 'LineWidth', 2);
ylabel('RV-to-Relay Elevation (deg)');
xlabel('Entry Time (min)');
title('Relay-RV Line of Sight');

end

