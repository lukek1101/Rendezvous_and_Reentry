function validate_configuration(cfg)
%VALIDATE_CONFIGURATION Reject invalid loop bounds before running any phase.
    sys = cfg.system;
    for name = {'mu','Re','g0','Isp','Target_Mass','Chaser_Mass_Init'}
        positive(sys.(name{1}), ['system.' name{1}]);
    end
    p1 = cfg.phase1;
    for name = {'time_step','dt_phase','dt_capture','max_wait','max_capture_time', ...
                'event_time_tol','phase_tol'}
        positive(p1.(name{1}), ['phase1.' name{1}]);
    end
    p2 = cfg.phase2;
    if ~isscalar(p2.mode) || ~any(p2.mode == ["LEGACY_IMPULSIVE" "HYBRID_AUTONOMOUS"])
        error('mission:InvalidProximityMode','Unknown Phase 2 execution mode.');
    end
    a = p2.autonomous;
    names = fieldnames(a);
    for j = 1:numel(names)
        value = a.(names{j});
        validateattributes(value, {'numeric'}, {'real','finite','nonempty','nonnegative'}, ...
            mfilename, ['phase2.autonomous.' names{j}]);
        if ~any(strcmp(names{j},{'transfer_times_s','approach_times_s'}))
            validateattributes(value, {'numeric'}, {'scalar'});
        end
    end
    validateattributes(a.transfer_times_s, {'numeric'}, {'vector','positive'});
    validateattributes(a.approach_times_s, {'numeric'}, {'vector','numel',2,'positive'});
    for name = {'control_period_s','max_force_N','natural_frequency_rad_s','damping_ratio', ...
            'hold_time_s','final_hold_s','capture_speed_m_s','gate_position_tol_m', ...
            'max_approach_speed_m_s','max_closing_speed_m_s','minimum_mass_kg'}
        positive(a.(name{1}), ['phase2.autonomous.' name{1}]);
    end
    if ~(a.insertion_range_m>a.hold_range_m && a.hold_range_m>p2.S4_R_abs && ...
            p2.S4_R_abs>a.keep_out_radius_m && a.closing_min_range_m<a.insertion_range_m && ...
            a.corridor_half_angle_deg>0 && a.corridor_half_angle_deg<90)
        error('mission:InvalidProximityGeometry','Invalid hold ranges, corridor, or exclusion radius.');
    end
    for name = {'dt','S4_R_abs','tof_initial_s2','delta_R_cycloid', ...
                'max_cycloid_orbits','tof_hop','tof_terminal_refine','Isp_fallback_s'}
        positive(p2.(name{1}), ['phase2.' name{1}]);
    end
    for name = {'initial_S2_tol','vbar_cross_tol','capture_pos_tol'}
        validateattributes(p2.(name{1}), {'numeric'}, ...
            {'scalar','real','finite','nonnegative'}, mfilename, ['phase2.' name{1}]);
    end
    validateattributes(p2.S2, {'numeric'}, {'size',[3 1],'real','finite'}, mfilename, 'phase2.S2');
    validateattributes(p2.rbar_hop_count, {'numeric'}, ...
        {'scalar','integer','positive','finite'}, mfilename, 'phase2.rbar_hop_count');
    validateattributes(p2.max_terminal_refines, {'numeric'}, ...
        {'scalar','integer','nonnegative','finite'}, mfilename, 'phase2.max_terminal_refines');
    if ~isscalar(p2.vbar_burn_sign) || ~any(p2.vbar_burn_sign == [-1 1])
        error('mission:InvalidBurnSign', 'phase2.vbar_burn_sign must be -1 or +1.');
    end
    positive(cfg.phase3.dt_reentry_coast_s, 'phase3.dt_reentry_coast_s');
    validateattributes(cfg.phase3.apogee_burns.enabled,{'logical'},{'scalar'});
    mission.validate_apogee_burns(cfg.phase3.apogee_burns);
    if ~isempty(cfg.phase3.max_reentry_coast_time_s)
        positive(cfg.phase3.max_reentry_coast_time_s, 'phase3.max_reentry_coast_time_s');
    end
end

function positive(value, name)
    validateattributes(value, {'numeric'}, {'scalar','real','finite','positive'}, mfilename, name);
end
