function results = Validate_Deorbit_Execution()
%VALIDATE_DEORBIT_EXECUTION Exercise both drag-aware burn execution paths.
% High thrust below is a fast numerical test fixture, not vehicle sizing.
    sys = Mission_Config();
    sys.environment.atmospheric_drag.enabled = true;
    sys.environment.atmospheric_drag.use_matlab_atmosisa = false;
    sys.h_entry_interface = 120e3;
    radius = sys.Re + 500e3;
    speed = sqrt(sys.mu/radius * (1+sys.J2*(sys.Re/radius)^2));
    target = [radius;0;0;0;0;speed];
    initial_mass = 2000;
    chaser = [target;0;0;0;1;0;0;0;initial_mass];
    params = struct('drag_deorbit_design_mode',"AUTO", ...
        'drag_deorbit_delta_v_m_s',275, 'drag_deorbit_max_coast_time_s',6000, ...
        'finite_burn_thrust',20000, 'finite_burn_isp',200, 'dt_burn',0.2, ...
        'dt_reentry_coast',2);
    modes = ["IMPULSIVE","FINITE_BURN"];
    for k = 1:numel(modes)
        params.burn_model = modes(k);
        [final, ~, delta_v, fuel, hist, info] = mission.deorbit(sys,chaser,target,params);
        assert(abs(norm(final(1:3))-sys.Re-sys.h_entry_interface)<1e-3);
        assert(abs(delta_v-275)<1e-7);
        expected_mass = initial_mass*exp(-275/(200*sys.g0));
        assert(abs(final(14)-expected_mass)<1e-6);
        assert(abs(fuel-(initial_mass-final(14)))<1e-9);
        assert(all(diff(hist.time)>=0) && all(diff(hist.mass)<=1e-9));
        assert(info.drag_deorbit_burn.burn_model == modes(k));
        results.(char(lower(modes(k)))) = struct('delta_v',delta_v,'fuel',fuel);
    end
    results.passed = true;
    fprintf('Drag-aware deorbit execution: PASS (impulsive and finite burn)\n');
end
