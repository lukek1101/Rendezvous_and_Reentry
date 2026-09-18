function [y, out] = track_rbar(y, sys, p2)
%TRACK_RBAR Sampled CW feedforward + PD with nonlinear ECI propagation.
% Perfect navigation; ideal direction/throttle; no attitude or RCS allocation.
    a = p2.autonomous;
    ranges = [a.insertion_range_m a.hold_range_m a.hold_range_m p2.S4_R_abs p2.S4_R_abs];
    durations = [a.approach_times_s(1) a.hold_time_s a.approach_times_s(2) a.final_hold_s];
    labels = ["APPROACH_250" "HOLD_250" "APPROACH_30" "HOLD_30"];
    capacity = sum(ceil(durations/a.control_period_s))+1;
    states = zeros(13,capacity); time = zeros(1,capacity);
    force = zeros(3,capacity); relative = zeros(3,capacity); velocity = relative;
    reference = relative; mode = strings(1,capacity);
    states(:,1) = y;
    [relative(:,1),velocity(:,1)] = orbit_core.relative_state(y(1:6),y(7:12));
    reference(:,1) = [-ranges(1);0;0]; mode(1) = "R_BAR_ACQUIRED";
    sample = 1; elapsed = 0; saturation_time = 0;
    gate_times = zeros(1,4);
    kp = a.natural_frequency_rad_s^2;
    kd = 2*a.damping_ratio*a.natural_frequency_rad_s;
    for segment = 1:4
        T = durations(segment); local = 0;
        start = [-ranges(segment);0;0]; delta = [ranges(segment)-ranges(segment+1);0;0];
        while local < T
            h = min(a.control_period_s,T-local);
            q = local/T;
            rref = start+delta*(10*q^3-15*q^4+6*q^5);
            vref = delta*(30*q^2-60*q^3+30*q^4)/T;
            aref = delta*(60*q-180*q^2+120*q^3)/T^2;
            [r,v,C] = orbit_core.relative_state(y(1:6),y(7:12));
            check_limits(r,v,y(13));
            n = norm(cross(y(7:9),y(10:12)))/norm(y(7:9))^2;
            natural = [3*n^2*r(1)+2*n*v(2); -2*n*v(1); -n^2*r(3)];
            requested = y(13)*(aref-natural+kp*(rref-r)+kd*(vref-v));
            magnitude = norm(requested);
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
            check_limits(relative(:,sample),velocity(:,sample),y(13));
        end
        goal = [-ranges(segment+1);0;0];
        if norm(relative(:,sample)-goal)>a.gate_position_tol_m || ...
                norm(velocity(:,sample))>a.capture_speed_m_s
            error('mission:ProximityGate','%s gate failed; position %.3f m, speed %.4f m/s.', ...
                labels(segment),norm(relative(:,sample)-goal),norm(velocity(:,sample)));
        end
        gate_times(segment) = elapsed;
    end
    out = struct('states',states(:,1:sample),'time',time(1:sample), ...
        'relative',relative(:,1:sample),'velocity',velocity(:,1:sample), ...
        'reference',reference(:,1:sample),'force_lvlh',force(:,1:sample), ...
        'mode',mode(1:sample),'gate_times',gate_times,'saturation_time_s',saturation_time);
    out.max_tracking_error_m = max(vecnorm(out.relative-out.reference));
    out.max_force_N = max(vecnorm(out.force_lvlh));
    out.max_speed_m_s = max(vecnorm(out.velocity));
    out.final_error_m = norm(out.relative(:,end)-[-p2.S4_R_abs;0;0]);
    out.final_speed_m_s = norm(out.velocity(:,end));
    out.delta_v_m_s = sys.Isp*sys.g0*log(states(13,1)/y(13));

    function check_limits(r,v,mass)
        lateral = norm(r(2:3));
        allowed = a.corridor_floor_m+max(0,-r(1))*tand(a.corridor_half_angle_deg);
        if r(1)>=0 || norm(r)<a.keep_out_radius_m || lateral>allowed || ...
                norm(v)>a.max_approach_speed_m_s || mass<a.minimum_mass_kg
            error('mission:ProximityConstraint', ...
                'Approach limit violated: range %.3f m, lateral %.3f m, speed %.4f m/s, mass %.2f kg.', ...
                norm(r),lateral,norm(v),mass);
        end
    end
end
