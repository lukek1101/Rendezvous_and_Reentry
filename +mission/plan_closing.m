function plan = plan_closing(y, sys, a)
%PLAN_CLOSING Screen a finite time grid; no global-optimality claim.
% CW seed, nonlinear differential correction, sampled nonlinear constraints.
    [r,v,C] = orbit_core.relative_state(y(1:6),y(7:12));
    n = norm(cross(y(7:9),y(10:12)))/norm(y(7:9))^2;
    goal = mission.approach_axis(a.approach_mode)*a.insertion_range_m;
    count = numel(a.transfer_times_s);
    row = struct('duration_s',0,'delta_v_m_s',inf,'position_error_m',inf, ...
        'min_range_m',NaN,'max_speed_m_s',NaN,'feasible',false,'reason',"", ...
        'departure_lvlh',zeros(3,1));
    candidates = repmat(row,count,1);
    settings = odeset('RelTol',1e-11,'AbsTol',1e-8,'MaxStep',30);
    for k = 1:count
        T = a.transfer_times_s(k);
        candidates(k).duration_s = T;
        try
            dv = mission.cw_departure(r,v,goal,T,n);
            for iteration = 1:7
                [rf,~,~] = shoot(dv,T);
                residual = rf-goal;
                if norm(residual) < 0.01
                    break;
                end
                jac = zeros(3);
                for j = 1:3
                    d = zeros(3,1); d(j) = 1e-4;
                    rp = shoot(dv+d,T);
                    jac(:,j) = (rp-rf)/d(j);
                end
                if rcond(jac) < 1e-10
                    error('mission:SingularTransfer','Nonlinear correction is singular.');
                end
                dv = dv-jac\residual;
            end
            [rf,vf,states] = shoot(dv,T);
            ranges = zeros(size(states,1),1); speeds = ranges;
            for j = 1:numel(ranges)
                [rr,vv] = orbit_core.relative_state(states(j,1:6)',states(j,7:12)');
                ranges(j)=norm(rr); speeds(j)=norm(vv);
            end
            candidates(k).position_error_m = norm(rf-goal);
            candidates(k).delta_v_m_s = norm(dv)+norm(vf);
            candidates(k).min_range_m = min(ranges);
            candidates(k).max_speed_m_s = max(speeds);
            candidates(k).departure_lvlh = dv;
            candidates(k).feasible = norm(rf-goal)<0.05 && ...
                min(ranges)>=a.closing_min_range_m && max(speeds)<=a.max_closing_speed_m_s;
            candidates(k).reason = "sampled constraints failed";
            if norm(rf-goal)>=0.05, candidates(k).reason="terminal targeting did not converge"; end
            if candidates(k).feasible
                candidates(k).reason = "feasible";
            end
        catch exception
            if ~strcmp(exception.identifier,'mission:SingularTransfer')
                rethrow(exception);
            end
            candidates(k).reason = string(exception.message);
        end
    end
    costs = [candidates.delta_v_m_s];
    costs(~[candidates.feasible]) = inf;
    [cost,index] = min(costs);
    if ~isfinite(cost)
        error('mission:NoClosingTransfer','No candidate satisfies closing constraints.');
    end
    plan = candidates(index);
    plan.candidates = candidates;
    plan.selected_index = index;
    plan.method = "nonlinear-corrected two-impulse time-grid search";

    function [rf,vf,states] = shoot(dv,T)
        initial = y;
        initial(4:6) = initial(4:6)+C'*dv;
        initial(13) = initial(13)*exp(-norm(dv)/(sys.Isp*sys.g0));
        times = linspace(0,T,ceil(T/10)+1);
        [~,states] = ode45(@(t,s) mission.proximity_dynamics(t,s,zeros(3,1),sys),times,initial,settings);
        [rf,vf] = orbit_core.relative_state(states(end,1:6)',states(end,7:12)');
    end
end
