function a = proximity_defaults()
%PROXIMITY_DEFAULTS Research assumptions, not flight-qualified HTV parameters.
    a.handoff_dwell_s = 60;
    a.approach_mode = "-R";
    a.insertion_range_m = 500;
    a.hold_range_m = 250;
    a.hold_time_s = 60;
    a.final_hold_s = 60;
    a.transfer_times_s = [1800 2700 3600 4500 5400];
    % Quintic peak speed = 1.875*distance/time, below ~10 m/min nominally.
    a.approach_times_s = [3200 2800];
    a.max_force_N = 300;
    a.control_period_s = 1;
    a.natural_frequency_rad_s = 0.02;
    a.damping_ratio = 1;
    a.capture_speed_m_s = 0.01;
    a.gate_position_tol_m = 1;
    a.corridor_half_angle_deg = 10;
    a.corridor_floor_m = 1;
    a.keep_out_radius_m = 20;
    a.closing_min_range_m = 250;
    a.max_closing_speed_m_s = 10;
    a.max_approach_speed_m_s = 0.17;
    a.minimum_mass_kg = 1000;
end
