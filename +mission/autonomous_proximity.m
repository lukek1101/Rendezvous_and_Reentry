function [Xc, Xt, result] = autonomous_proximity(sys, Xc, Xt, p2)
%AUTONOMOUS_PROXIMITY Hybrid research model: impulse closing, finite-force final.
    a = p2.autonomous;
    y = [Xc(1:6);Xt(1:6);Xc(14)]; initial_mass = y(13);
    [r,v] = orbit_core.relative_state(Xc,Xt);
    if norm(r-p2.S2)>p2.initial_S2_tol
        error('mission:HandoffPosition','Phase 1 missed the S2 handoff position gate.');
    end
    history = y; times = 0; phase_names = "HANDOFF";
    [y, handoff_dv] = impulse(y,-v);
    handoff_mass = y(13); history(:,1)=y;
    [y,dwell_states,dwell_times] = coast(y,a.handoff_dwell_s,0);
    history = [history dwell_states]; times = [times dwell_times];
    phase_names = [phase_names repmat("DWELL",1,numel(dwell_times))];
    plan = mission.plan_closing(y,sys,a);
    [y, departure_dv] = impulse(y,plan.departure_lvlh);
    history(:,end)=y;
    [y,closing_states,closing_times] = coast(y,plan.duration_s,times(end));
    history = [history closing_states]; times = [times closing_times];
    phase_names = [phase_names repmat("CLOSING",1,numel(closing_times))];
    [r,v] = orbit_core.relative_state(y(1:6),y(7:12));
    if norm(r-[-a.insertion_range_m;0;0])>a.gate_position_tol_m
        error('mission:ProximityGate','Nonlinear closing arrival missed the R-bar gate.');
    end
    [y, arrival_dv] = impulse(y,-v); history(:,end)=y;
    closing_mass = y(13); acquisition_time = times(end);
    [y,control] = mission.track_rbar(y,sys,p2);
    history = [history control.states(:,2:end)];
    times = [times acquisition_time+control.time(2:end)];
    phase_names = [phase_names control.mode(2:end)];
    count = numel(times); relative = zeros(3,count); velocity = relative;
    for k = 1:count
        [relative(:,k),velocity(:,k)] = orbit_core.relative_state(history(1:6,k),history(7:12,k));
    end
    Xc(1:6)=y(1:6); Xc(14)=y(13); Xt=y(7:12);
    p2.S3 = [-a.insertion_range_m;0;0]; p2.S4=[-p2.S4_R_abs;0;0];
    h = struct('pos',history(1:3,:),'time',times,'mass',history(13,:), ...
        'rel_pos_lvlh',relative,'rel_vel_lvlh',velocity,'mode',phase_names);
    target_times = [0 a.handoff_dwell_s acquisition_time ...
        acquisition_time+control.gate_times([1 2 3 4])];
    targets = [p2.S2 relative(:,numel(dwell_times)+1) p2.S3 ...
        [-a.hold_range_m;0;0] [-a.hold_range_m;0;0] p2.S4 p2.S4];
    result = struct('history',h,'relative_position',relative,'mass',history(13,:), ...
        'targets',targets,'target_times',target_times,'transfer_times',diff(target_times), ...
        'names',["S2 stop" "S2 depart" "R-bar 500 m" "250 m" "250 m depart" "30 m" "30 m verified"], ...
        'config',p2,'delta_v',handoff_dv+departure_dv+arrival_dv+control.delta_v_m_s, ...
        'fuel',initial_mass-y(13),'duration',times(end), ...
        'final_position_error',control.final_error_m,'final_relative_velocity',velocity(:,end), ...
        'reached_standoff',control.final_error_m<=p2.capture_pos_tol && ...
            control.final_speed_m_s<=a.capture_speed_m_s, ...
        'plan',plan,'control',control,'execution_model',"HYBRID_IMPULSE_CLOSING_FINITE_FINAL");
    result.handoff = struct('delta_v',handoff_dv,'fuel',initial_mass-handoff_mass, ...
        'dwell_s',a.handoff_dwell_s,'departure_relative_position',targets(:,2));
    result.closing = struct('delta_v',departure_dv+arrival_dv,'fuel',handoff_mass-closing_mass);
    result.final_approach = struct('delta_v',control.delta_v_m_s,'fuel',closing_mass-y(13));
    fprintf(['Phase 2 HYBRID: handoff %.4f m/s, closing %.4f m/s (%g s), ' ...
        'finite final %.4f m/s; terminal %.4f m, %.6f m/s\n'], ...
        handoff_dv,result.closing.delta_v,plan.duration_s,control.delta_v_m_s, ...
        control.final_error_m,control.final_speed_m_s);

    function [s,dv_mag] = impulse(s,dv)
        [~,~,C] = orbit_core.relative_state(s(1:6),s(7:12));
        dv_mag = norm(dv); s(4:6)=s(4:6)+C'*dv;
        s(13)=s(13)*exp(-dv_mag/(sys.Isp*sys.g0));
        if s(13)<a.minimum_mass_kg
            error('mission:ProximityConstraint','Impulse exceeds configured mass budget.');
        end
    end

    function [s,states,t] = coast(s,T,t0)
        steps = ceil(T/p2.dt); states=zeros(13,steps); t=zeros(1,steps); elapsed=0;
        for j = 1:steps
            dt = min(p2.dt,T-elapsed);
            s=mission.proximity_step(s,zeros(3,1),dt,p2.dt,sys);
            elapsed=elapsed+dt; states(:,j)=s; t(j)=t0+elapsed;
            [rr,vv]=orbit_core.relative_state(s(1:6),s(7:12));
            if norm(rr)<a.closing_min_range_m || norm(vv)>a.max_closing_speed_m_s
                error('mission:ProximityConstraint','Closing coast violates range/speed limits.');
            end
        end
    end
end
