function [X_chaser, X_target, dV_used, fuel_used, hist, info] = deorbit(sys, X_chaser, X_target, custom_params)
    dV_used = 0;
    fuel_used = 0;
    hist = init_phase3_hist();
    info = struct();

    if should_use_drag_deorbit_design(sys, custom_params)
        design = load_drag_deorbit_design(custom_params, sys);
        dV_entry = design.delta_v_m_s;

        fprintf('   HOHMANN drag-aware deorbit: %s retrograde %.3f m/s from Python design...\n', ...
                char(design.burn_model), dV_entry);
        if isfield(design, 'predicted_entry_fpa_deg') && isfinite(design.predicted_entry_fpa_deg)
            fprintf('      Python prediction: %.1f km interface after %.2f min total, FPA %.3f deg\n', ...
                    sys.h_entry_interface/1000, design.predicted_total_time_s/60, design.predicted_entry_fpa_deg);
        end
        if design.burn_model == "FINITE_BURN"
            fprintf('      finite burn: %.1f N, Isp %.1f s, predicted duration %.2f min, steering %s\n', ...
                    design.finite_burn_thrust_N, design.finite_burn_isp_s, ...
                    design.predicted_burn_duration_s/60, char(design.burn_steering));
        end

        [X_chaser, X_target, dV_entry, fuel_entry, hist_burn, burn_info] = ...
            apply_retrograde_deorbit_burn(X_chaser, X_target, dV_entry, sys, design);
        dV_used = dV_used + dV_entry;
        fuel_used = fuel_used + fuel_entry;
        info.entry_injection_dV = dV_entry;
        info.entry_injection_fuel = fuel_entry;
        info.drag_deorbit_design = design;
        info.drag_deorbit_burn = burn_info;

        hist = append_phase3_hist(hist, hist_burn);

        if burn_info.interface_reached
            actual_fpa = flight_path_angle_deg(X_chaser(1:3), X_chaser(4:6));
            fprintf('      %.1f km interface reached during burn after %.2f min; actual FPA %.3f deg; deorbit fuel charged: %.4f kg.\n', ...
                    sys.h_entry_interface/1000, burn_info.duration_s/60, actual_fpa, fuel_entry);
            info.reentry_coast_time = 0;
            return;
        end

        dt_reentry = get_phase3_param(custom_params, 'dt_reentry_coast', 2);
        default_reentry_time = max(1.5 * design.predicted_coast_time_s, design.predicted_coast_time_s + 600);
        max_reentry_time = get_phase3_param(custom_params, 'max_reentry_coast_time', default_reentry_time);
        max_reentry_time = get_phase3_param(custom_params, 'drag_deorbit_max_coast_time_s', max_reentry_time);

        [X_chaser, X_target, hist_entry, coast_time] = ...
            propagate_until_altitude(X_chaser, X_target, sys, sys.h_entry_interface, max_reentry_time, dt_reentry, hist.time_end);
        hist = append_phase3_hist(hist, hist_entry);
        info.reentry_coast_time = coast_time;

        actual_fpa = flight_path_angle_deg(X_chaser(1:3), X_chaser(4:6));
        fprintf('      %.1f km interface reached after burn %.2f min + coast %.2f min; actual FPA %.3f deg; deorbit fuel charged: %.4f kg.\n', ...
                sys.h_entry_interface/1000, burn_info.duration_s/60, coast_time/60, actual_fpa, fuel_entry);
        return;
    end

    [target_reentry_r, fpa_calc] = compute_reentry_target_radius(norm(X_chaser(1:3)), sys);
    info.fpa_calc = fpa_calc;
    info.reentry_target_radius = target_reentry_r;

    fprintf('   HOHMANN leg: injection toward %.1f km / %.2f deg FPA...\n', ...
            sys.h_entry_interface/1000, rad2deg(sys.reentry_flight_path_angle));
    [X_chaser, dV_entry, fuel_entry] = apply_reentry_departure_impulse(X_chaser, target_reentry_r, sys);
    dV_used = dV_used + dV_entry;
    fuel_used = fuel_used + fuel_entry;
    info.entry_injection_dV = dV_entry;
    info.entry_injection_fuel = fuel_entry;

    hist = log_phase3_state(hist, X_chaser, X_target, 0);

    r_start = norm(X_chaser(1:3));
    a_trans = 0.5 * (r_start + target_reentry_r);
    default_reentry_time = 1.2 * pi * sqrt(a_trans^3 / sys.mu);
    dt_reentry = get_phase3_param(custom_params, 'dt_reentry_coast', 2);
    max_reentry_time = get_phase3_param(custom_params, 'max_reentry_coast_time', default_reentry_time);

    [X_chaser, X_target, hist_entry, coast_time] = ...
        propagate_until_altitude(X_chaser, X_target, sys, sys.h_entry_interface, max_reentry_time, dt_reentry, hist.time_end);
    hist = append_phase3_hist(hist, hist_entry);
    info.reentry_coast_time = coast_time;

    fprintf('      %.1f km interface reached after %.2f min; deorbit fuel charged: %.4f kg.\n', ...
            sys.h_entry_interface/1000, coast_time/60, fuel_entry);
end

function [target_r, details] = compute_reentry_target_radius(start_r, sys)
    interface_r = sys.Re + sys.h_entry_interface;
    gamma = sys.reentry_flight_path_angle;
    phi = 2 * atan(start_r / (start_r - interface_r) * tan(gamma));
    denom = sin(phi) * cos(gamma) - cos(phi) * sin(gamma);

    if abs(denom) < 1e-12
        error('Re-entry FPA geometry became singular. Check start radius and flight-path angle.');
    end

    ecc = sin(gamma) / denom;
    target_r = (1 - ecc) / (1 + ecc) * start_r;

    if ~isfinite(target_r) || target_r <= 0
        error('Computed invalid re-entry target radius %.6g m.', target_r);
    end

    details = struct();
    details.start_radius = start_r;
    details.interface_radius = interface_r;
    details.flight_path_angle = gamma;
    details.phi = phi;
    details.eccentricity = ecc;
    details.target_radius = target_r;
end

function [X_chaser, dV_mag, fuel_used] = apply_reentry_departure_impulse(X_chaser, target_r, sys)
    r_current = norm(X_chaser(1:3));
    v_current = norm(X_chaser(4:6));
    a_trans = 0.5 * (r_current + target_r);
    sqrt_arg = sys.mu * (2/r_current - 1/a_trans - sys.J2 * sys.Re^2 / r_current^3 * (3 * (X_chaser(3)/r_current)^2 - 1));

    if sqrt_arg <= 0
        error('Computed invalid re-entry injection speed. sqrt argument = %.6g.', sqrt_arg);
    end

    v_trans1 = sqrt(sqrt_arg);
    dV_cmd = v_trans1 - v_current;
    dV_mag = abs(dV_cmd);

    X_chaser(4:6) = X_chaser(4:6) + dV_cmd * X_chaser(4:6) / v_current;

    m0 = X_chaser(14);
    m1 = m0 * exp(-dV_mag / (sys.Isp * sys.g0));
    fuel_used = m0 - m1;
    X_chaser(14) = m1;
end

function [X_chaser, dV_mag, fuel_used] = apply_retrograde_deorbit_impulse(X_chaser, delta_v_m_s, sys)
    if delta_v_m_s <= 0 || ~isfinite(delta_v_m_s)
        error('Drag-aware deorbit delta-V must be positive and finite.');
    end

    v_current = norm(X_chaser(4:6));
    if v_current <= eps
        error('Cannot apply retrograde deorbit impulse with near-zero velocity.');
    end

    dV_mag = delta_v_m_s;
    X_chaser(4:6) = X_chaser(4:6) - dV_mag * X_chaser(4:6) / v_current;

    m0 = X_chaser(14);
    m1 = m0 * exp(-dV_mag / (sys.Isp * sys.g0));
    fuel_used = m0 - m1;
    X_chaser(14) = m1;
end

function [X_chaser, X_target, dV_mag, fuel_used, hist, burn_info] = ...
    apply_retrograde_deorbit_burn(X_chaser, X_target, delta_v_m_s, sys, design)

    burn_model = normalize_deorbit_burn_model(get_phase3_param(design, 'burn_model', "IMPULSIVE"));
    hist = init_phase3_hist();
    burn_info = struct();
    burn_info.burn_model = burn_model;
    burn_info.commanded_delta_v_m_s = delta_v_m_s;
    burn_info.interface_reached = false;

    if burn_model == "IMPULSIVE"
        [X_chaser, dV_mag, fuel_used] = apply_retrograde_deorbit_impulse(X_chaser, delta_v_m_s, sys);
        burn_info.duration_s = 0;
        burn_info.delivered_delta_v_m_s = dV_mag;
        burn_info.propellant_used_kg = fuel_used;
        hist = log_phase3_state(hist, X_chaser, X_target, 0);
        return;
    end

    if delta_v_m_s <= 0 || ~isfinite(delta_v_m_s)
        error('Finite drag-aware deorbit delta-V must be positive and finite.');
    end

    thrust_N = get_phase3_param(design, 'finite_burn_thrust_N', sys.maneuver.finite_burn_thrust);
    Isp_s = get_phase3_param(design, 'finite_burn_isp_s', sys.maneuver.finite_burn_isp);
    dt_burn = get_phase3_param(design, 'finite_burn_dt_s', sys.maneuver.finite_burn_dt);
    steering = normalize_deorbit_burn_steering(get_phase3_param(design, 'burn_steering', "VELOCITY_RETROGRADE"));

    if thrust_N <= 0
        error('Finite drag-aware deorbit requires positive thrust.');
    end
    if Isp_s <= 0
        error('Finite drag-aware deorbit requires positive Isp.');
    end
    if dt_burn <= 0
        error('Finite drag-aware deorbit requires positive dt.');
    end

    v_norm = norm(X_chaser(4:6));
    if v_norm <= eps
        error('Cannot start finite retrograde deorbit burn with near-zero velocity.');
    end

    fixed_retrograde_dir = -X_chaser(4:6) / v_norm;
    m0 = X_chaser(14);
    exhaust_velocity = Isp_s * sys.g0;
    mf_commanded = m0 * exp(-delta_v_m_s / exhaust_velocity);
    burn_duration_s = (m0 - mf_commanded) * exhaust_velocity / thrust_N;

    burn_info.thrust_N = thrust_N;
    burn_info.Isp_s = Isp_s;
    burn_info.dt_burn_s = dt_burn;
    burn_info.steering = steering;
    burn_info.predicted_duration_s = burn_duration_s;
    burn_info.initial_mass_kg = m0;
    burn_info.commanded_final_mass_kg = mf_commanded;
    burn_info.initial_burn_direction_eci = fixed_retrograde_dir;

    design_mass_kg = get_phase3_param(design, 'initial_mass_kg', NaN);
    if isfinite(design_mass_kg) && design_mass_kg > 0 && abs(m0 - design_mass_kg) / design_mass_kg > 0.02
        fprintf('      note: JSON finite-burn design mass %.1f kg, actual Phase 3 mass %.1f kg; duration/fuel recomputed with actual mass.\n', ...
                design_mass_kg, m0);
    end

    hist = log_phase3_state(hist, X_chaser, X_target, 0);
    elapsed = 0;
    while elapsed < burn_duration_s - 1e-12
        dt_step = min(dt_burn, burn_duration_s - elapsed);
        [X_chaser, X_target] = rk4_pair_step_deorbit_burn( ...
            X_chaser, X_target, dt_step, sys, thrust_N, Isp_s, steering, fixed_retrograde_dir, elapsed);
        elapsed = elapsed + dt_step;

        if X_chaser(14) <= 0
            error('Chaser mass depleted during finite drag-aware deorbit burn.');
        end

        hist = log_phase3_state(hist, X_chaser, X_target, elapsed);
        if norm(X_chaser(1:3)) - sys.Re <= sys.h_entry_interface
            burn_info.interface_reached = true;
            break;
        end
    end

    dV_mag = exhaust_velocity * log(m0 / X_chaser(14));
    fuel_used = m0 - X_chaser(14);

    burn_info.duration_s = elapsed;
    burn_info.delivered_delta_v_m_s = dV_mag;
    burn_info.final_mass_kg = X_chaser(14);
    burn_info.propellant_used_kg = fuel_used;
end

function dX = deorbit_burn_dynamics(X, sys, thrust_N, Isp_s, steering, fixed_retrograde_dir)
    r = X(1:3);
    v = X(4:6);
    m = X(14);

    if m <= 0
        error('Finite burn dynamics received non-positive mass.');
    end

    a_gravity = orbit_core.gravity_j2(r, sys);
    a_drag = Atmospheric_Drag_Acceleration(r, v, m, sys, "chaser");

    if steering == "FIXED_INITIAL_RETROGRADE"
        thrust_dir = fixed_retrograde_dir(:) / norm(fixed_retrograde_dir);
    else
        v_norm = norm(v);
        if v_norm <= eps
            thrust_dir = fixed_retrograde_dir(:) / norm(fixed_retrograde_dir);
        else
            thrust_dir = -v / v_norm;
        end
    end

    a_thrust = thrust_dir * (thrust_N / m);
    dm = -thrust_N / (Isp_s * sys.g0);
    dX = [v; a_gravity + a_drag + a_thrust; zeros(7,1); dm];
end

function [X_chaser, X_target] = rk4_pair_step_deorbit_burn( ...
    X_chaser, X_target, dt_step, sys, thrust_N, Isp_s, steering, fixed_retrograde_dir, t_abs)

    if ~isempty(X_target)
        X_t_state = [X_target; zeros(7,1); sys.Target_Mass];
        k1_t = Env_EOM(t_abs,             X_t_state,                [0;0;0], [0;0;0], sys, false);
        k2_t = Env_EOM(t_abs+dt_step/2,   X_t_state+k1_t*dt_step/2, [0;0;0], [0;0;0], sys, false);
        k3_t = Env_EOM(t_abs+dt_step/2,   X_t_state+k2_t*dt_step/2, [0;0;0], [0;0;0], sys, false);
        k4_t = Env_EOM(t_abs+dt_step,     X_t_state+k3_t*dt_step,   [0;0;0], [0;0;0], sys, false);
        X_target = X_target + (dt_step/6)*(k1_t(1:6) + 2*k2_t(1:6) + 2*k3_t(1:6) + k4_t(1:6));
    end

    k1 = deorbit_burn_dynamics(X_chaser,             sys, thrust_N, Isp_s, steering, fixed_retrograde_dir);
    k2 = deorbit_burn_dynamics(X_chaser+k1*dt_step/2,sys, thrust_N, Isp_s, steering, fixed_retrograde_dir);
    k3 = deorbit_burn_dynamics(X_chaser+k2*dt_step/2,sys, thrust_N, Isp_s, steering, fixed_retrograde_dir);
    k4 = deorbit_burn_dynamics(X_chaser+k3*dt_step,  sys, thrust_N, Isp_s, steering, fixed_retrograde_dir);
    X_chaser = X_chaser + (dt_step/6)*(k1 + 2*k2 + 2*k3 + k4);

    if norm(X_chaser(7:10)) > 0
        X_chaser(7:10) = X_chaser(7:10) / norm(X_chaser(7:10));
    end
end

function model = normalize_deorbit_burn_model(value)
    model = upper(strtrim(string(value)));
    if model == "FINITE" || model == "FINITE_BURN" || model == "FINITE_IMPULSE" || model == "CONTINUOUS"
        model = "FINITE_BURN";
    elseif model == "IMPULSIVE" || model == "INSTANT" || model == "INSTANTANEOUS" || model == "CUSTOM_IMPULSE"
        model = "IMPULSIVE";
    else
        error('Unsupported drag-aware deorbit burn model: %s', char(model));
    end
end

function steering = normalize_deorbit_burn_steering(value)
    steering = upper(strrep(strtrim(string(value)), '-', '_'));
    if steering == "RETROGRADE" || steering == "VELOCITY" || steering == "VELOCITY_FOLLOWING" || steering == "VELOCITY_RETROGRADE"
        steering = "VELOCITY_RETROGRADE";
    elseif steering == "FIXED" || steering == "FIXED_START" || steering == "FIXED_RETROGRADE" || steering == "FIXED_INITIAL_RETROGRADE"
        steering = "FIXED_INITIAL_RETROGRADE";
    else
        error('Unsupported drag-aware deorbit burn steering: %s', char(steering));
    end
end

function tf = should_use_drag_deorbit_design(sys, custom_params)
    if ~orbital_drag_enabled(sys)
        tf = false;
        return;
    end

    mode = upper(string(get_phase3_param(custom_params, 'drag_deorbit_design_mode', "AUTO")));
    tf = ~(mode == "OFF" || mode == "NONE" || mode == "DISABLED");
end

function design = load_drag_deorbit_design(custom_params, sys)
    manual_dv = get_phase3_param(custom_params, 'drag_deorbit_delta_v_m_s', []);
    if ~isempty(manual_dv)
        design = struct();
        design.delta_v_m_s = manual_dv;
        design.predicted_coast_time_s = get_phase3_param(custom_params, 'drag_deorbit_max_coast_time_s', 2*pi*sqrt((sys.Re + sys.h_target)^3/sys.mu));
        design.predicted_total_time_s = design.predicted_coast_time_s;
        design.predicted_burn_duration_s = 0;
        design.predicted_entry_fpa_deg = NaN;
        design.burn_model = normalize_deorbit_burn_model(get_phase3_param(custom_params, 'burn_model', sys.maneuver.default_burn_model));
        design.burn_steering = "VELOCITY_RETROGRADE";
        design.finite_burn_thrust_N = get_phase3_param(custom_params, 'finite_burn_thrust', sys.maneuver.finite_burn_thrust);
        design.finite_burn_isp_s = get_phase3_param(custom_params, 'finite_burn_isp', sys.maneuver.finite_burn_isp);
        design.finite_burn_dt_s = get_phase3_param(custom_params, 'dt_burn', sys.maneuver.finite_burn_dt);
        design.source = "manual Mission_Run_Config.m override";
        return;
    end

    design_file = string(get_phase3_param(custom_params, 'drag_deorbit_design_file', "configs/latest_drag_deorbit_solution.json"));
    if strlength(design_file) == 0
        error('Drag-aware HOHMANN deorbit requires a drag_deorbit_design_file or manual delta_v_m_s.');
    end

    design_path = resolve_project_path(design_file);
    if ~isfile(design_path)
        error(['Drag-aware deorbit design JSON not found: %s\n' ...
               'Generate it with: python DragDeorbitDesigner.py --matlab-config-out %s'], ...
               char(design_path), char(design_file));
    end

    cfg = jsondecode(fileread(design_path));
    if ~get_json_bool(cfg, {'phase3','drag_deorbit','enabled'}, false)
        error('Drag-aware deorbit design JSON is not enabled: %s', char(design_path));
    end

    entry_alt_km = get_json_number(cfg, {'phase3','entry_interface_altitude_km'}, NaN);
    if isfinite(entry_alt_km) && abs(entry_alt_km - sys.h_entry_interface/1000) > 1e-3
        error('Drag deorbit JSON entry interface %.6f km does not match current %.6f km. Regenerate the design JSON.', ...
              entry_alt_km, sys.h_entry_interface/1000);
    end

    fpa_deg = get_json_number(cfg, {'phase3','flight_path_angle_deg'}, NaN);
    if isfinite(fpa_deg) && abs(abs(fpa_deg) - abs(rad2deg(sys.reentry_flight_path_angle))) > 1e-3
        error('Drag deorbit JSON FPA %.6f deg does not match current %.6f deg. Regenerate the design JSON.', ...
              fpa_deg, rad2deg(sys.reentry_flight_path_angle));
    end

    design = struct();
    design.delta_v_m_s = get_json_number(cfg, {'phase3','drag_deorbit','delta_v_m_s'}, NaN);
    design.burn_model = normalize_deorbit_burn_model(get_json_string(cfg, {'phase3','drag_deorbit','burn_model'}, "IMPULSIVE"));
    design.burn_steering = normalize_deorbit_burn_steering(get_json_string(cfg, {'phase3','drag_deorbit','burn_steering'}, "VELOCITY_RETROGRADE"));
    design.predicted_burn_duration_s = get_json_number(cfg, {'phase3','drag_deorbit','predicted_burn_duration_s'}, 0);
    design.predicted_burn_time_s = get_json_number(cfg, {'phase3','drag_deorbit','predicted_burn_time_s'}, design.predicted_burn_duration_s);
    design.initial_mass_kg = get_json_number(cfg, {'phase3','drag_deorbit','initial_mass_kg'}, ...
        get_json_number(cfg, {'maneuver','initial_mass_kg'}, NaN));
    design.finite_burn_thrust_N = get_json_number(cfg, {'phase3','drag_deorbit','finite_burn_thrust_N'}, ...
        get_json_number(cfg, {'maneuver','finite_burn_thrust_N'}, sys.maneuver.finite_burn_thrust));
    design.finite_burn_isp_s = get_json_number(cfg, {'phase3','drag_deorbit','finite_burn_isp_s'}, ...
        get_json_number(cfg, {'maneuver','finite_burn_isp_s'}, sys.maneuver.finite_burn_isp));
    design.finite_burn_dt_s = get_json_number(cfg, {'phase3','drag_deorbit','finite_burn_dt_s'}, ...
        get_json_number(cfg, {'maneuver','finite_burn_dt_s'}, sys.maneuver.finite_burn_dt));
    design.predicted_coast_time_s = get_json_number(cfg, {'phase3','drag_deorbit','predicted_coast_time_s'}, NaN);
    design.predicted_total_time_s = get_json_number(cfg, {'phase3','drag_deorbit','predicted_total_time_s'}, design.predicted_coast_time_s + design.predicted_burn_time_s);
    design.predicted_entry_fpa_deg = get_json_number(cfg, {'phase3','drag_deorbit','predicted_entry_fpa_deg'}, NaN);
    design.predicted_entry_speed_m_s = get_json_number(cfg, {'phase3','drag_deorbit','predicted_entry_speed_m_s'}, NaN);
    design.source = string(get_json_string(cfg, {'source'}, "DragDeorbitDesigner.py"));
    design.path = string(design_path);

    if ~isfinite(design.delta_v_m_s) || design.delta_v_m_s <= 0
        error('Drag deorbit JSON has invalid delta_v_m_s: %s', char(design_path));
    end
    if ~isfinite(design.predicted_coast_time_s) || design.predicted_coast_time_s <= 0
        design.predicted_coast_time_s = 2*pi*sqrt((sys.Re + sys.h_target)^3/sys.mu);
    end
    if ~isfinite(design.predicted_total_time_s) || design.predicted_total_time_s <= 0
        design.predicted_total_time_s = design.predicted_coast_time_s + max(0, design.predicted_burn_time_s);
    end
end

function path = resolve_project_path(path_value)
    path_text = string(path_value);
    if is_absolute_path(path_text)
        path = path_text;
    else
        root_dir = mission.project_root();
        path = string(fullfile(root_dir, char(path_text)));
    end
end

function [X_chaser, X_target, hist, elapsed] = propagate_until_altitude(X_chaser, X_target, sys, target_alt, max_time, dt, t0)
    hist = init_phase3_hist();
    elapsed = 0;
    alt_prev = norm(X_chaser(1:3)) - sys.Re;

    if alt_prev <= target_alt
        hist = log_phase3_state(hist, X_chaser, X_target, t0);
        return;
    end

    while elapsed < max_time - 1e-12
        dt_step = min(dt, max_time - elapsed);
        X_prev = X_chaser;
        T_prev = X_target;

        [X_next, T_next] = rk4_pair_step_full(X_prev, T_prev, dt_step, sys, t0 + elapsed);
        alt_next = norm(X_next(1:3)) - sys.Re;

        if alt_next <= target_alt
            [X_chaser, X_target, t_cross] = refine_altitude_crossing(X_prev, T_prev, target_alt, dt_step, sys, t0 + elapsed);
            elapsed = elapsed + t_cross;
            hist = log_phase3_state(hist, X_chaser, X_target, t0 + elapsed);
            return;
        end

        elapsed = elapsed + dt_step;
        X_chaser = X_next;
        X_target = T_next;
        hist = log_phase3_state(hist, X_chaser, X_target, t0 + elapsed);
    end

    error('Re-entry coast did not reach %.1f km altitude within %.2f min.', target_alt/1000, max_time/60);
end

function [X_cross, T_cross, t_cross] = refine_altitude_crossing(X0, T0, target_alt, dt_window, sys, t_abs0)
    lo = 0;
    hi = dt_window;
    X_cross = X0;
    T_cross = T0;
    t_cross = 0;

    for iter = 1:40
        mid = 0.5 * (lo + hi);
        [X_mid, T_mid] = propagate_pair_state_only_full(X0, T0, mid, min(1, max(mid/10, 0.05)), sys, t_abs0);
        alt_mid = norm(X_mid(1:3)) - sys.Re;

        if alt_mid > target_alt
            lo = mid;
        else
            hi = mid;
            X_cross = X_mid;
            T_cross = T_mid;
            t_cross = mid;
        end
    end
end

function [X_chaser, X_target] = propagate_pair_state_only_full(X_chaser, X_target, duration, dt, sys, t_abs0)
    if duration <= 0
        return;
    end

    elapsed = 0;
    while elapsed < duration - 1e-12
        dt_step = min(dt, duration - elapsed);
        [X_chaser, X_target] = rk4_pair_step_full(X_chaser, X_target, dt_step, sys, t_abs0 + elapsed);
        elapsed = elapsed + dt_step;
    end
end

function [X_chaser, X_target] = rk4_pair_step_full(X_chaser, X_target, dt_step, sys, t_abs)
    X_t_state = [X_target; zeros(7,1); sys.Target_Mass];
    k1_t = Env_EOM(t_abs,             X_t_state,                [0;0;0], [0;0;0], sys, false);
    k2_t = Env_EOM(t_abs+dt_step/2,   X_t_state+k1_t*dt_step/2, [0;0;0], [0;0;0], sys, false);
    k3_t = Env_EOM(t_abs+dt_step/2,   X_t_state+k2_t*dt_step/2, [0;0;0], [0;0;0], sys, false);
    k4_t = Env_EOM(t_abs+dt_step,     X_t_state+k3_t*dt_step,   [0;0;0], [0;0;0], sys, false);
    X_target = X_target + (dt_step/6)*(k1_t(1:6) + 2*k2_t(1:6) + 2*k3_t(1:6) + k4_t(1:6));

    k1 = Env_EOM(t_abs,             X_chaser,             [0;0;0], [0;0;0], sys, true);
    k2 = Env_EOM(t_abs+dt_step/2,   X_chaser+k1*dt_step/2,[0;0;0], [0;0;0], sys, true);
    k3 = Env_EOM(t_abs+dt_step/2,   X_chaser+k2*dt_step/2,[0;0;0], [0;0;0], sys, true);
    k4 = Env_EOM(t_abs+dt_step,     X_chaser+k3*dt_step,  [0;0;0], [0;0;0], sys, true);
    X_chaser = X_chaser + (dt_step/6)*(k1 + 2*k2 + 2*k3 + k4);

    if norm(X_chaser(7:10)) > 0
        X_chaser(7:10) = X_chaser(7:10) / norm(X_chaser(7:10));
    end
end

function value = get_phase3_param(s, name, default_value)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        value = s.(name);
    else
        value = default_value;
    end
end

function hist = init_phase3_hist()
    hist.pos = [];
    hist.vel = [];
    hist.mass = [];
    hist.time = [];
    hist.time_end = 0;
    hist.target_pos = [];
    hist.target_vel = [];
    hist.rel_pos = [];
    hist.rel_pos_lvlh = [];
    hist.rel_vel_lvlh = [];
end

function hist = log_phase3_state(hist, X_chaser, X_target, t)
    if ~isfield(hist, 'time')
        hist = init_phase3_hist();
    end

    hist.pos = [hist.pos, X_chaser(1:3)];
    hist.vel = [hist.vel, X_chaser(4:6)];
    hist.mass = [hist.mass, X_chaser(14)];
    hist.time = [hist.time, t];
    hist.time_end = t;

    if ~isempty(X_target)
        hist.target_pos = [hist.target_pos, X_target(1:3)];
        hist.target_vel = [hist.target_vel, X_target(4:6)];
        hist.rel_pos = [hist.rel_pos, X_chaser(1:3) - X_target(1:3)];
        [rel_lvlh, rel_vel_lvlh] = orbit_core.relative_state(X_chaser, X_target);
        hist.rel_pos_lvlh = [hist.rel_pos_lvlh, rel_lvlh];
        hist.rel_vel_lvlh = [hist.rel_vel_lvlh, rel_vel_lvlh];
    end
end

function hist = append_phase3_hist(hist, sub_hist)
    if isempty(sub_hist) || ~isfield(sub_hist, 'time') || isempty(sub_hist.time)
        return;
    end

    if ~isfield(hist, 'time') || isempty(hist.time)
        hist = sub_hist;
        if ~isfield(hist, 'time_end')
            hist.time_end = hist.time(end);
        end
        return;
    end

    fields = {'pos','vel','mass','time','target_pos','target_vel','rel_pos','rel_pos_lvlh','rel_vel_lvlh'};
    for ii = 1:numel(fields)
        f = fields{ii};
        if ~isfield(hist, f)
            hist.(f) = [];
        end
        if isfield(sub_hist, f) && ~isempty(sub_hist.(f))
            hist.(f) = [hist.(f), sub_hist.(f)];
        end
    end
    hist.time_end = hist.time(end);
end
