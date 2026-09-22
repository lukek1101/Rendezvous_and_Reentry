function out = propagate(sys,vehicle,initial_state,bank_profile,c)
%PROPAGATE Bank-rate-limited 3-DOF entry; uses the same atmospheric physics as mission.
% Endpoint at requested altitude. No parachute, runway flare, or touchdown model.
    validateattributes(initial_state,{'numeric'},{'vector','numel',6,'finite','real'});
    initial_state=initial_state(:);
    validateattributes(vehicle.mass_kg,{'numeric'},{'scalar','finite','positive'});
    for name={'max_time_s','max_step_s','relative_tolerance','bank_rate_deg_s','bank_response_s'}
        validateattributes(c.(name{1}),{'numeric'},{'scalar','finite','positive'});
    end
    validateattributes(c.terminal_altitude_m,{'numeric'},{'scalar','finite','nonnegative'});
    for name={'earth_angle_at_epoch_deg','entry_epoch_s','earth_rotation_rad_s','initial_bank_deg'}
        validateattributes(c.(name{1}),{'numeric'},{'scalar','finite','real'});
    end
    for name={'max_dynamic_pressure_Pa','max_g_load','max_heat_flux_W_m2','max_heat_load_J_m2'}
        validateattributes(c.(name{1}),{'numeric'},{'scalar','real','positive','nonnan'});
    end
    validateattributes(c.target_tolerance_m,{'numeric'},{'scalar','finite','positive'});
    validateattributes(c.target_max_iterations,{'numeric'},{'scalar','integer','positive'});
    validateattributes(c.target_max_evaluations,{'numeric'},{'scalar','integer','positive'});
    validateattributes(c.bank_limit_deg,{'numeric'},{'scalar','finite','positive','<=',180});
    validateattributes(c.speed_fractions,{'numeric'},{'vector','finite','positive','nonempty'});
    if numel(c.speed_fractions)<2
        error('entry_design:SpeedKnots','At least two decreasing speed knots are required.');
    end
    validateattributes(bank_profile,{'numeric'},{'vector','finite','numel',numel(c.speed_fractions)});
    if any(abs(bank_profile)>c.bank_limit_deg) || abs(c.initial_bank_deg)>c.bank_limit_deg || ...
            any(diff(c.speed_fractions)>=0) || any(c.speed_fractions<=0)
        error('entry_design:BankSchedule','Invalid bank bounds or decreasing speed knots.');
    end
    if norm(initial_state(1:3))-sys.Re<=c.terminal_altitude_m
        error('entry_design:InitialAltitude','Initial state must be above the endpoint altitude.');
    end
    local_sys=sys;
    local_sys.reentry_vehicle.spaceplane.communication.earth_fixed_to_eci_angle_at_mission_epoch_deg=c.earth_angle_at_epoch_deg;
    if sys.environment.atmospheric_drag.earth_rotation_rad_s~=c.earth_rotation_rad_s
        error('entry_design:RotationMismatch','Study and atmospheric rotation must agree.');
    end
    y0=[initial_state(:);vehicle.mass_kg;c.initial_bank_deg];
    initial_aux=evaluate(y0,0);
    reference_speed=initial_aux.speed_rel;
    opts=odeset('RelTol',c.relative_tolerance,'AbsTol',1e-7, ...
        'MaxStep',c.max_step_s,'Events',@events);
    [time,state,~,~,event]=ode45(@rhs,[0 c.max_time_s],y0,opts);
    count=numel(time); heat=zeros(count,1); pressure=heat; loads=heat;
    aoa=heat; profile_held=false(count,1);
    for k=1:count
        aux=evaluate(state(k,:)',time(k));
        heat(k)=aux.heat_flux; pressure(k)=aux.dynamic_pressure; loads(k)=aux.g_load;
        aoa(k)=aux.aoa_deg; profile_held(k)=aux.aoa_profile.boundary_held;
    end
    terminal=any(event==1);
    constraints=max(pressure)<=c.max_dynamic_pressure_Pa && max(loads)<=c.max_g_load && ...
        max(heat)<=c.max_heat_flux_W_m2 && trapz(time,heat)<=c.max_heat_load_J_m2;
    origin=entry_design.ecef(initial_state(1:3),c.entry_epoch_s,c);
    omega=[0;0;c.earth_rotation_rad_s];
    if ~sys.environment.atmospheric_drag.co_rotate_atmosphere, omega=zeros(3,1); end
    heading=entry_design.ecef(initial_state(4:6)-cross(omega,initial_state(1:3)),c.entry_epoch_s,c);
    position=entry_design.ecef(state(end,1:3)',c.entry_epoch_s+time(end),c);
    [latlon,xy]=entry_design.geography(position,origin,heading,sys.Re);
    out=struct('time_s',time,'state',state,'terminal_reached',terminal, ...
        'constraints_satisfied',constraints,'feasible',terminal && constraints, ...
        'latlon_deg',latlon,'range_m',xy,'final_ecef_m',position, ...
        'max_dynamic_pressure_Pa',max(pressure),'max_g_load',max(loads), ...
        'max_heat_flux_W_m2',max(heat),'heat_load_J_m2',trapz(time,heat), ...
        'bank_profile_deg',bank_profile,'terminal_altitude_m',norm(state(end,1:3))-sys.Re, ...
        'termination',"TIMEOUT_OR_SKIP");
    if terminal, out.termination="ALTITUDE_ENDPOINT"; end
    out.initial_conditions=mission.state_record([initial_state;vehicle.mass_kg], ...
        local_sys,c.entry_epoch_s,"CALLER_SUPPLIED_ENTRY_STATE");
    out.vehicle_provenance=vehicle;
    out.aoa_deg=aoa;
    out.aoa_profile_boundary_held=profile_held;
    if terminal && ~constraints, out.termination="PATH_CONSTRAINT_FAILED"; end
    out.limits_enforced=isfinite([c.max_dynamic_pressure_Pa c.max_g_load ...
        c.max_heat_flux_W_m2 c.max_heat_load_J_m2]);
    function aux=evaluate(y,t)
        aux=reentry_core.evaluate_state(y(1:7),local_sys,vehicle.shape,vehicle.aoa_deg,y(8),true,vehicle.heat_coefficient,t);
    end
    function dy=rhs(t,y)
        aux=evaluate(y,t);
        fraction=aux.speed_rel/reference_speed;
        fraction=min(max(fraction,c.speed_fractions(end)),c.speed_fractions(1));
        commanded=interp1(fliplr(c.speed_fractions(:)'),fliplr(bank_profile(:)'),fraction,'linear');
        bank_rate=max(-c.bank_rate_deg_s,min(c.bank_rate_deg_s,(commanded-y(8))/c.bank_response_s));
        dy=[y(4:6);aux.a_gravity+aux.a_aero;0;bank_rate];
    end
    function [value,stop,direction]=events(~,y)
        value=norm(y(1:3))-sys.Re-c.terminal_altitude_m; stop=1; direction=-1;
    end
end
