function a = apogee_burn_defaults()
%APOGEE_BURN_DEFAULTS Demonstration constraints, not HTV engine specifications.
    a.enabled = false;
    a.total_delta_v_m_s = 150;
    a.fractions = [0.2 0.2 0.6];
    a.thrust_N = 300;
    a.isp_s = 200;
    a.max_burn_duration_s = 900;
    a.cooldown_s = 600;
    a.minimum_mass_kg = 1000;
    a.max_burns = 20;
    a.max_elapsed_s = 172800;
    a.first_burn = "CURRENT"; % CURRENT or NEXT_APOGEE; circular apogee undefined.
    a.max_step_s = 10;
end
