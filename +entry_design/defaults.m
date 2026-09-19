function c = defaults()
%DEFAULTS Reachability study controls; endpoint is explicit, not implicit landing.
    c.terminal_altitude_m = 20000;
    c.max_time_s = 5000;
    c.max_step_s = 5;
    c.relative_tolerance = 1e-8;
    c.earth_angle_at_epoch_deg = 0;
    c.entry_epoch_s = 0;
    c.earth_rotation_rad_s = 7.2921159e-5;
    c.bank_limit_deg = 60;
    c.bank_rate_deg_s = 10;
    c.initial_bank_deg = 0;
    c.bank_response_s = 2;
    c.speed_fractions = [1 0.75 0.4];
    c.bank_profiles_deg = [-60 -60 -60; -30 -30 -30; 0 0 0; ...
        30 30 30; 60 60 60; -60 60 60; 60 -60 -60; -60 -60 60; 60 60 -60];
    c.max_dynamic_pressure_Pa = inf;
    c.max_g_load = inf;
    c.max_heat_flux_W_m2 = inf;
    c.max_heat_load_J_m2 = inf;
    c.target_tolerance_m = 10000;
    c.target_max_iterations = 35;
    c.target_max_evaluations = 80;
end
