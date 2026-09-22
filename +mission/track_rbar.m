function [y, out] = track_rbar(y, sys, p2)
%TRACK_RBAR Sampled CW feedforward + PD with nonlinear ECI propagation.
% Perfect navigation; ideal direction/throttle; no attitude or RCS allocation.
    a = p2.autonomous;
    axis=mission.approach_axis(a.approach_mode);
    ranges = [a.insertion_range_m a.hold_range_m a.hold_range_m p2.S4_R_abs p2.S4_R_abs];
    durations = [a.approach_times_s(1) a.hold_time_s a.approach_times_s(2) a.final_hold_s];
    labels = ["APPROACH_HOLD" "HOLD" "APPROACH_STANDOFF" "HOLD_STANDOFF"];
    capacity = sum(ceil(durations/a.control_period_s))+1;
    states = zeros(13,capacity); time = zeros(1,capacity);
    force = zeros(3,capacity); relative = zeros(3,capacity); velocity = relative;
    reference = relative; mode = strings(1,capacity);
    states(:,1) = y;
    [relative(:,1),velocity(:,1)] = orbit_core.relative_state(y(1:6),y(7:12));
    reference(:,1) = axis*ranges(1); mode(1) = "AXIS_ACQUIRED";
    sample = 1; elapsed = 0; saturation_time = 0;
    gate_times = nan(1,4);
    violations=struct('corridor',false,'side',false,'keep_out',false,'speed',false,'mass',false,'gate',false);
    termination="COMPLETED"; requested_force=force;
    kp = a.natural_frequency_rad_s^2;
    kd = 2*a.damping_ratio*a.natural_frequency_rad_s;
    for segment = 1:4
        T = durations(segment); local = 0;
        start = axis*ranges(segment); delta = axis*(ranges(segment+1)-ranges(segment));
        while local < T
            h = min(a.control_period_s,T-local);
            q = local/T;
            rref = start+delta*(10*q^3-15*q^4+6*q^5);
            vref = delta*(30*q^2-60*q^3+30*q^4)/T;
            aref = delta*(60*q-180*q^2+120*q^3)/T^2;
            [r,v,C] = orbit_core.relative_state(y(1:6),y(7:12));
            if ~check_limits(r,v,y(13)), termination="CONSTRAINT_VIOLATION"; break; end
            n = norm(cross(y(7:9),y(10:12)))/norm(y(7:9))^2;
            natural = [3*n^2*r(1)+2*n*v(2); -2*n*v(1); -n^2*r(3)];
            % Evaluate CW terms in the physical R,V,H frame for every mode.
            % In particular V motion creates radial Coriolis acceleration;
            % do not rotate a previously computed R-bar force vector.
            requested = y(13)*(aref-natural+kp*(rref-r)+kd*(vref-v));
            magnitude = norm(requested);
            requested_force(:,sample)=requested;
            command = requested*min(1,a.max_force_N/max(magnitude,eps));
            saturation_time = saturation_time+h*(magnitude>a.max_force_N);
            force(:,sample) = command; % Held command for the interval starting here.
            y = mission.proximity_step(y,C'*command,h,p2.dt,sys);
            local = local+h; elapsed = elapsed+h; sample = sample+1;
            states(:,sample) = y; time(sample) = elapsed;
            [relative(:,sample),velocity(:,sample)] = orbit_core.relative_state(y(1:6),y(7:12));
            q = local/T;
            reference(:,sample) = start+delta*(10*q^3-15*q^4+6*q^5);
            mode(sample) = labels(segment);
            if ~check_limits(relative(:,sample),velocity(:,sample),y(13))
                termination="CONSTRAINT_VIOLATION"; break;
            end
        end
        if termination~="COMPLETED", break; end
        goal = axis*ranges(segment+1);
        if norm(relative(:,sample)-goal)>a.gate_position_tol_m || ...
                norm(velocity(:,sample))>a.capture_speed_m_s
            violations.gate=true; termination="GATE_VIOLATION"; break;
        end
        gate_times(segment) = elapsed;
    end
    out = struct('states',states(:,1:sample),'time',time(1:sample), ...
        'relative',relative(:,1:sample),'velocity',velocity(:,1:sample), ...
        'reference',reference(:,1:sample),'force_lvlh',force(:,1:sample), ...
        'mode',mode(1:sample),'gate_times',gate_times,'saturation_time_s',saturation_time);
    out.max_tracking_error_m = max(vecnorm(out.relative-out.reference));
    out.max_force_N = max(vecnorm(out.force_lvlh));
    out.requested_force_lvlh=requested_force(:,1:sample);
    out.max_requested_force_N=max(vecnorm(out.requested_force_lvlh));
    out.termination=termination; out.completed=termination=="COMPLETED";
    out.max_speed_m_s = max(vecnorm(out.velocity));
    out.final_error_m = norm(out.relative(:,end)-axis*p2.S4_R_abs);
    out.final_speed_m_s = norm(out.velocity(:,end));
    out.delta_v_m_s = sys.Isp*sys.g0*log(states(13,1)/y(13));
    out.approach_mode=string(a.approach_mode);
    out.actuator_assumption="IDEAL_ECI_FORCE_DIRECTION_AND_THROTTLE_WITH_NORM_LIMIT";
    out.force_metric_scope="CONTROLLED_APPROACH_ONLY_IMPULSE_PEAK_FORCE_UNDEFINED";
    out.axis_lvlh=axis;
    axial=axis'*out.relative;
    out.min_corridor_margin_m=min(a.corridor_floor_m+max(0,axial)*tand(a.corridor_half_angle_deg)- ...
        vecnorm(out.relative-axis*axial));
    out.min_keep_out_margin_m=min(vecnorm(out.relative))-a.keep_out_radius_m;
    out.constraint_violations=violations;

    function valid=check_limits(r,v,mass)
        axial=dot(axis,r); lateral = norm(r-axis*axial);
        allowed = a.corridor_floor_m+max(0,axial)*tand(a.corridor_half_angle_deg);
        violations.side=violations.side || axial<=0;
        violations.keep_out=violations.keep_out || norm(r)<a.keep_out_radius_m;
        violations.corridor=violations.corridor || lateral>allowed;
        violations.speed=violations.speed || norm(v)>a.max_approach_speed_m_s;
        violations.mass=violations.mass || mass<a.minimum_mass_kg;
        valid=~any(structfun(@(x)x,violations));
    end
end
