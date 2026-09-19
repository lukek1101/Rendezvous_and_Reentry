function result = run(cfg)
%RUN Execute the four mission phases using an already resolved configuration.
sys = cfg.proximity_system;
custom_params = cfg.phase1;
phasing_mode = cfg.phase1_mode;
% Budget Tracking Table
Budget = table('Size',[0 5], ...
    'VariableTypes',{'string','double','double','double','double'}, ...
    'VariableNames',{'Phase','DeltaV_ms','Fuel_Consumed_kg', ...
                     'Remaining_Mass_kg','Total_Accounted_Mass_kg'});

m_current = sys.Chaser_Mass_Init;
capsule_mass_added_to_initial_stack = false;
if upper(string(sys.reentry_vehicle.vehicle_mode)) == "CAPSULE" && ...
        sys.reentry_vehicle.capsule.add_to_chaser_initial_mass
    m_current = m_current + sys.reentry_vehicle.capsule.mass_kg;
    capsule_mass_added_to_initial_stack = true;
    fprintf('Capsule stack mass: base chaser %.3f kg + capsule %.3f kg = %.3f kg\n', ...
            sys.Chaser_Mass_Init, sys.reentry_vehicle.capsule.mass_kg, m_current);
end
initial_chaser_angle = deg2rad(sys.initial_chaser_angle_deg);
initial_phase_angle = deg2rad(sys.initial_phase_angle_deg);

% Initial chaser state.
[x_insert, v_insert_vec] = circular_polar_state(sys.Re + sys.h_insert, initial_chaser_angle, sys);
X_chaser_init = [x_insert;  v_insert_vec;  0;0;0;1;  0;0;0;  m_current];
target_radius = sys.Re + sys.h_target;

% Initial target state.
[x_target, v_target_vec] = circular_polar_state(sys.Re + sys.h_target, initial_chaser_angle + initial_phase_angle, sys);
X_target = [x_target; v_target_vec];


fprintf('[Phase 1] 3-DOF phasing propagation start (J2 included)...\n');
[X_chaser, dV_p1, fuel_p1, hist_p1, X_target] = Phasing_Propagator(sys, X_chaser_init, target_radius, phasing_mode, custom_params, X_target, 1);
m_current = X_chaser(14);

Budget = [Budget; {"Phase 1: "+phasing_mode, dV_p1, fuel_p1, m_current, m_current}];

if isfield(hist_p1, 'maneuver_duration') && ~isempty(hist_p1.maneuver_duration)
    max_burn_duration_p1 = max(hist_p1.maneuver_duration);
    max_burn_delta_v_p1 = max(hist_p1.maneuver_delta_v);
    fprintf('   Phase 1 max burn duration: %.3f s\n', max_burn_duration_p1);
    fprintf('   Phase 1 max single-burn delta-V: %.6f m/s\n', max_burn_delta_v_p1);
end

[rel_p1_lvlh, ~] = orbit_core.relative_state(X_chaser, X_target);
miss_p1 = norm(rel_p1_lvlh - custom_params.desired_rel_lvlh);

fprintf('   Phase 1 terminal LVLH relative position: [%+.3f, %+.3f, %+.3f] m\n', rel_p1_lvlh(1), rel_p1_lvlh(2), rel_p1_lvlh(3));
fprintf('   desired_rel_lvlh error: %.6f m\n', miss_p1);
fprintf('   Phase 1 terminal altitude: %.2f km\n', (norm(X_chaser(1:3)) - sys.Re)/1000);


result.config = cfg;
result.phasing = struct('history', hist_p1, 'delta_v', dV_p1, 'fuel', fuel_p1, ...
    'chaser', X_chaser, 'target', X_target, 'position_error', miss_p1);
[X_chaser, X_target, result.proximity] = mission.proximity(sys, X_chaser, X_target, cfg.phase2);
result.proximity.chaser = X_chaser;
result.proximity.target = X_target;
if ~result.proximity.reached_standoff
    error('mission:StandoffNotReached','Proximity failed; downstream deorbit is inhibited.');
end
m_current = X_chaser(14);
Budget = [Budget; {"Phase 2: "+result.proximity.execution_model, result.proximity.delta_v, ...
    result.proximity.fuel, m_current, m_current}];

sys = cfg.system;
fprintf('\n[Phase 3] De-orbit to %.1f km interface...\n', sys.h_entry_interface/1e3);
X_orbiting_entry_relay0 = [X_target(1:6); 0;0;0;1; 0;0;0; sys.Target_Mass];
[X_chaser, X_target, dV_p3, fuel_p3, hist_p3, reentry_info] = ...
    mission.deorbit(sys, X_chaser, X_target, cfg.deorbit);
m_current = X_chaser(14);
deorbit_label=cfg.phase3.mode;
if isfield(reentry_info,'mode'), deorbit_label=reentry_info.mode; end
Budget = [Budget; {"Phase 3: "+deorbit_label, dV_p3, fuel_p3, m_current, m_current}];
[X_entry_interface, entry_interface_info] = mission.entry_interface( ...
    hist_p3, X_chaser, sys, sys.h_entry_interface);
result.deorbit = struct('history', hist_p3, 'delta_v', dV_p3, 'fuel', fuel_p3, ...
    'chaser', X_chaser, 'target', X_target, 'info', reentry_info, ...
    'interface_state', X_entry_interface, 'interface', entry_interface_info);
phase3_elapsed_to_interface = entry_interface_info.time_s;
fprintf('   entry interface %.3f km, FPA %.3f deg, speed %.3f m/s\n', ...
    entry_interface_info.altitude_m/1000, entry_interface_info.fpa_deg, entry_interface_info.velocity_m_s);
mission_elapsed_to_entry_s = hist_p1.time(end) + result.proximity.duration + phase3_elapsed_to_interface;
[X_reentry_vehicle, hist_reentry, reentry_atmo_info, X_entry_interface] = mission.entry( ...
    sys, X_entry_interface, X_orbiting_entry_relay0, phase3_elapsed_to_interface, ...
    mission_elapsed_to_entry_s, capsule_mass_added_to_initial_stack);
result.entry = struct('history', hist_reentry, 'summary', reentry_atmo_info, ...
    'vehicle', X_reentry_vehicle, 'initial_state', X_entry_interface, ...
    'mission_start_time', mission_elapsed_to_entry_s);
Budget = [Budget; {"Phase 4: Atmospheric Entry", 0, 0, ...
    reentry_atmo_info.active_vehicle_mass_kg, reentry_atmo_info.total_accounted_mass_kg}];
result.budget = Budget;
mission.report(result);
end

function [r, v] = circular_polar_state(radius, u_rad, sys)
    r = radius * [cos(u_rad); 0; sin(u_rad)];
    z_over_r = sin(u_rad);
    v_mag = sqrt(sys.mu / radius * (1 - sys.J2 * (sys.Re / radius)^2 * (3*z_over_r^2 - 1)));
    v = v_mag * [-sin(u_rad); 0; cos(u_rad)];
end
