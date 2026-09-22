function out=execute_corrected_nominal(sys,Xc,Xt,goal,plan,p,c,max_burn)
% Execute a frozen nominal schedule with reproducible errors and optional TCMs.
% Perfect current-state knowledge; future actuator errors are not predicted.
started=tic;
validateattributes(c.seed,{'numeric'},{'scalar','integer','nonnegative','<=',2^32-1});
for name={'enabled','corrections_enabled','terminal_velocity_cleanup'}
    validateattributes(c.(name{1}),{'logical'},{'scalar'});
end
for name={'position_sigma_m','velocity_sigma_m_s'}
    validateattributes(c.(name{1}),{'numeric'},{'size',[3 1],'finite','nonnegative'});
end
for name={'burn_gain_sigma','burn_pointing_sigma_rad','max_total_correction_delta_v_m_s','max_propellant_kg'}
    validateattributes(c.(name{1}),{'numeric'},{'scalar','finite','nonnegative'});
end
for name={'max_correction_delta_v_m_s','max_nominal_burn_m_s','minimum_mass_kg', ...
        'max_elapsed_s','minimum_remaining_transfer_s','max_runtime_s'}
    validateattributes(c.(name{1}),{'numeric'},{'scalar','finite','positive'});
end
for name={'max_corrections','max_iterations','max_evaluations'}
    validateattributes(c.(name{1}),{'numeric'},{'scalar','integer','finite','nonnegative'});
end
validateattributes(c.opportunity_fractions,{'numeric'},{'vector','finite','>',0,'<',1});
if any(diff(c.opportunity_fractions)<=0) || string(c.state_knowledge)~="PERFECT_INSTANTANEOUS" || ...
        ~plan.success || sys.environment.atmospheric_drag.enabled
    error('mission:CorrectionScope','Require a valid drag-free nominal plan, ordered opportunities and perfect state knowledge.');
end
stream=RandStream('mt19937ar','Seed',c.seed); slots=numel(c.opportunity_fractions)+2;
initial_error=[c.position_sigma_m.*randn(stream,3,1);c.velocity_sigma_m_s.*randn(stream,3,1)];
% Draws indexed by scheduled role, never by how many burns were executed.
gain=c.burn_gain_sigma*randn(stream,1,slots+1);
pointing=c.burn_pointing_sigma_rad*randn(stream,3,slots+1);
y=[Xc(1:6)+initial_error;Xt(1:6);Xc(14)];
time=0; states=y'; times=0; burns=struct([]); opportunities=struct([]);
count=0; correction_dv=0; total_dv=0; terminal_time=plan.wait_s+plan.transfer_s;
status="EXECUTED"; arrival_retargeted=false;
abstol=[repmat(p.position_absolute_tolerance_m,3,1);repmat(p.velocity_absolute_tolerance_m_s,3,1)];
opts=odeset('RelTol',p.relative_tolerance,'AbsTol',[abstol;abstol;1e-9],'MaxStep',p.max_step_s);
try
    if terminal_time>c.max_elapsed_s
        status="ELAPSED_TIME_LIMIT";
    else
        coast(plan.wait_s);
        if burn(plan.departure_delta_v_eci_m_s,1,"DEPARTURE",false)
            for k=1:numel(c.opportunity_fractions)
                coast(plan.wait_s+c.opportunity_fractions(k)*plan.transfer_s);
                remaining=terminal_time-time;
                [before,~]=predict(y,remaining,zeros(3,1));
                entry=struct('time_s',time,'before',before,'after',before, ...
                    'status',"DISABLED",'evaluations',1,'delta_v_command_eci_m_s',zeros(3,1));
                if c.corrections_enabled && norm(before.position_error_lvlh_m)>p.position_tolerance_m
                    if count>=c.max_corrections
                        entry.status="CORRECTION_COUNT_LIMIT";
                    elseif remaining<c.minimum_remaining_transfer_s
                        entry.status="REMAINING_TIME_LIMIT";
                    else
                        [dv,entry.status,entry.evaluations]=retarget(remaining,before.position_error_lvlh_m);
                        entry.delta_v_command_eci_m_s=dv;
                        if entry.status=="CONVERGED"
                            if burn(dv,k+1,"MIDCOURSE",true)
                                [entry.after,~]=predict(y,remaining,zeros(3,1));
                                entry.status="APPLIED";
                            else
                                entry.status=status;
                            end
                        end
                    end
                elseif c.corrections_enabled
                    entry.status="POSITION_WITHIN_TOLERANCE_VELOCITY_DEFERRED";
                end
                opportunities=[opportunities;entry]; %#ok<AGROW>
                if status~="EXECUTED", break; end
            end
            if status=="EXECUTED"
                coast(terminal_time);
                arrival_command=plan.arrival_delta_v_eci_m_s;
                if c.corrections_enabled
                    [~,v,C]=orbit_core.relative_state(y(1:6),y(7:12));
                    arrival_command=C'*(p.desired_velocity_lvlh_m_s-v);
                    arrival_retargeted=true;
                end
                burn(arrival_command,slots,"ARRIVAL",false);
                [~,v,C]=orbit_core.relative_state(y(1:6),y(7:12));
                if status=="EXECUTED" && c.corrections_enabled && c.terminal_velocity_cleanup && ...
                        norm(v-p.desired_velocity_lvlh_m_s)>p.velocity_tolerance_m_s
                    burn(C'*(p.desired_velocity_lvlh_m_s-v),slots+1,"TERMINAL_VELOCITY_CLEANUP",true);
                end
            end
        end
    end
catch e
    if ~strcmp(e.identifier,'mission:CorrectionWorkLimit'), rethrow(e); end
    status="NUMERICAL_WORK_LIMIT";
end
[r,v]=orbit_core.relative_state(y(1:6),y(7:12));
out=struct('success',status=="EXECUTED" && norm(r-goal)<=p.position_tolerance_m && ...
    norm(v-p.desired_velocity_lvlh_m_s)<=p.velocity_tolerance_m_s, ...
    'execution_status',status,'settings',c,'state_knowledge',c.state_knowledge, ...
    'future_execution_errors_in_prediction',false,'additional_phasing_leg',"NOT_USED", ...
    'arrival_retargeted',arrival_retargeted, ...
    'position_error_m',norm(r-goal),'velocity_error_m_s',norm(v-p.desired_velocity_lvlh_m_s), ...
    'achieved_position_lvlh_m',r,'achieved_velocity_lvlh_m_s',v, ...
    'delta_v_m_s',total_dv,'correction_delta_v_m_s',correction_dv, ...
    'fuel_kg',Xc(14)-y(13),'correction_count',count,'elapsed_s',time, ...
    'runtime_s',toc(started),'burns',burns,'opportunities',opportunities, ...
    'initial_error_eci_si',initial_error,'gain_draws',gain,'pointing_draws_rad',pointing);
out.chaser=Xc; out.chaser(1:6)=y(1:6); out.chaser(14)=y(13); out.target=y(7:12);
out.handoff_status="NOT_REACHED";
if time==terminal_time
    out.handoff_status="TOLERANCE_VIOLATION";
    if out.position_error_m<=p.position_tolerance_m && out.velocity_error_m_s<=p.velocity_tolerance_m_s
        out.handoff_status="TOLERANCES_MET";
    end
end
out.phasing_leg_assessment="NOT_ESTABLISHED_FAILURE_IS_NOT_PROOF_OF_NECESSITY";
if out.success, out.phasing_leg_assessment="NOT_REQUIRED_FOR_THIS_REALIZATION"; end
h=struct('pos',states(:,1:3)','vel',states(:,4:6)','mass',states(:,13)', ...
    'target_pos',states(:,7:9)','target_vel',states(:,10:12)', ...
    'time',times','time_end',times(end),'rel_pos',(states(:,1:3)-states(:,7:9))', ...
    'rel_pos_lvlh',zeros(3,numel(times)),'rel_vel_lvlh',zeros(3,numel(times)), ...
    'maneuver_delta_v',[],'maneuver_duration',[],'maneuver_name',strings(1,0));
for k=1:numel(times)
    [h.rel_pos_lvlh(:,k),h.rel_vel_lvlh(:,k)]=orbit_core.relative_state(states(k,1:6)',states(k,7:12)');
end
if ~isempty(burns)
    h.maneuver_delta_v=[burns.delivered_delta_v_m_s];
    h.maneuver_duration=zeros(size(h.maneuver_delta_v)); h.maneuver_name=[burns.role];
end
out.history=h;
    function dy=rhs(t,state)
        if toc(started)>c.max_runtime_s, error('mission:CorrectionWorkLimit','Execution/prediction time budget exhausted.'); end
        dy=mission.proximity_dynamics(t,state,zeros(3,1),sys);
    end
    function coast(until)
        if until<=time, return; end
        [tt,yy]=ode45(@rhs,[time until],y,opts);
        y=yy(end,:)'; time=tt(end); states=[states;yy(2:end,:)]; times=[times;tt(2:end)];
    end
    function allowed=burn(command,slot,role,is_correction)
        allowed=false;
        limit=min(max_burn,c.max_nominal_burn_m_s);
        if is_correction
            limit=min(limit,c.max_correction_delta_v_m_s);
            if count>=c.max_corrections, status="CORRECTION_COUNT_LIMIT"; return; end
            if correction_dv+norm(command)>c.max_total_correction_delta_v_m_s
                status="CORRECTION_DV_LIMIT"; return;
            end
        end
        predicted_mass=y(13)*exp(-norm(command)/(sys.Isp*sys.g0));
        if norm(command)>limit, status="MANEUVER_CAPABILITY_LIMIT"; return; end
        if Xc(14)-predicted_mass>c.max_propellant_kg || predicted_mass<c.minimum_mass_kg
            status="PROPELLANT_LIMIT"; return;
        end
        actual=(1+gain(slot))*(command+cross(pointing(:,slot),command));
        before=y; y(4:6)=y(4:6)+actual; y(13)=y(13)*exp(-norm(actual)/(sys.Isp*sys.g0));
        total_dv=total_dv+norm(actual);
        if is_correction, count=count+1; correction_dv=correction_dv+norm(actual); end
        record=struct('time_s',time,'slot',slot,'role',role,'is_correction',is_correction, ...
            'command_eci_m_s',command,'delivered_eci_m_s',actual,'delivered_delta_v_m_s',norm(actual), ...
            'state_before',before,'state_after',y,'fuel_kg',before(13)-y(13));
        burns=[burns;record]; states=[states;y']; times=[times;time];
        % A realized execution error may violate a bound: retain the actual
        % burn/state and flag it, never clip or undo an applied impulse.
        if norm(actual)>limit || correction_dv>c.max_total_correction_delta_v_m_s
            status="DELIVERED_MANEUVER_LIMIT_VIOLATION"; return;
        end
        if Xc(14)-y(13)>c.max_propellant_kg || y(13)<c.minimum_mass_kg
            status="DELIVERED_PROPELLANT_LIMIT_VIOLATION"; return;
        end
        allowed=true;
    end
    function [info,final]=predict(initial,remaining,command)
        initial(4:6)=initial(4:6)+command;
        [~,yy]=ode45(@rhs,[0 remaining],initial,opts); final=yy(end,:)';
        if c.corrections_enabled
            [~,vv,CC]=orbit_core.relative_state(final(1:6),final(7:12));
            final(4:6)=final(4:6)+CC'*(p.desired_velocity_lvlh_m_s-vv);
        else
            final(4:6)=final(4:6)+plan.arrival_delta_v_eci_m_s;
        end
        [rr,vv]=orbit_core.relative_state(final(1:6),final(7:12));
        info=struct('position_error_lvlh_m',rr-goal, ...
            'velocity_error_lvlh_m_s',vv-p.desired_velocity_lvlh_m_s);
    end
    function [command,reason,evaluations]=retarget(remaining,residual)
        command=zeros(3,1); reason="ITERATION_LIMIT"; evaluations=0;
        for iteration=1:c.max_iterations
            if evaluations+4>c.max_evaluations, reason="EVALUATION_LIMIT"; return; end
            J=zeros(3);
            for axis=1:3
                delta=zeros(3,1); delta(axis)=p.difference_step_m_s;
                pp=predict(y,remaining,command+delta); evaluations=evaluations+1;
                J(:,axis)=(pp.position_error_lvlh_m-residual)/delta(axis);
            end
            if rcond(J)<1e-10, reason="SINGULAR_JACOBIAN"; return; end
            command=command-J\residual;
            if norm(command)>c.max_correction_delta_v_m_s, reason="PREDICTED_CAPABILITY_LIMIT"; return; end
            pp=predict(y,remaining,command); evaluations=evaluations+1;
            residual=pp.position_error_lvlh_m;
            if norm(residual)<=p.position_tolerance_m/4, reason="CONVERGED"; return; end
        end
    end
end
