function plan=plan_nominal(sys,Xc,Xt,goal,max_wait,p,max_burn)
%PLAN_NOMINAL Existing two-impulse Hohmann architecture, fixed analytic time.
% Solve terminal position, then charge the existing arrival impulse to match
% rotating-LVLH velocity. No delta-V minimization or additional correction burn.
if nargin<7, max_burn=inf; end
started=tic; evaluations=0;
names=fieldnames(mission.nominal_defaults());
for k=1:numel(names)
    if strcmp(names{k},'desired_velocity_lvlh_m_s'), continue; end
    validateattributes(p.(names{k}),{'numeric'},{'scalar','finite','positive'});
end
validateattributes(p.max_iterations,{'numeric'},{'integer'});
validateattributes(p.max_evaluations,{'numeric'},{'integer'});
validateattributes(goal,{'numeric'},{'size',[3 1],'finite'});
validateattributes(p.desired_velocity_lvlh_m_s,{'numeric'},{'size',[3 1],'finite'});
validateattributes(max_wait,{'numeric'},{'scalar','finite','positive'});
validateattributes(max_burn,{'numeric'},{'scalar','positive','nonnan'});
if sys.inc~=pi/2 || any(abs([Xc(2);Xc(5);Xt(2);Xt(5);goal(3);p.desired_velocity_lvlh_m_s(3)])>1e-8) || ...
        norm(Xc(1:3))>=norm(Xt(1:3)) || sys.environment.atmospheric_drag.enabled
    error('mission:NominalScope','Nominal planner supports ascending coplanar X-Z polar, drag-free impulsive missions only.');
end
plan=struct('success',false,'numerical_status',"NOT_RUN",'constraint_status',"NOT_EVALUATED", ...
    'method',"ANALYTIC_HOHMANN_SEED_BOUNDED_POSITION_SHOOTING",'optimization',"NOT_REQUESTED", ...
    'settings',p,'evaluations',0,'iterations',0,'runtime_s',0);
r1=norm(Xc(1:3)); r2=norm(Xt(1:3)); n1=sqrt(sys.mu/r1^3); n2=sqrt(sys.mu/r2^3);
T=pi*sqrt(((r1+r2)/2)^3/sys.mu);
phase=atan2(dot(cross(Xc(1:3),Xt(1:3)),cross(Xc(1:3),Xc(4:6)))/ ...
    norm(cross(Xc(1:3),Xc(4:6))),dot(Xc(1:3),Xt(1:3)));
desired_phase=pi-n2*T-atan2(goal(2),r2+goal(1));
wait=mod(phase-desired_phase,2*pi)/(n1-n2);
plan.wait_s=wait; plan.transfer_s=T;
if wait>max_wait
    plan.constraint_status="WAIT_BOUND_VIOLATED"; finish(); return;
end
abstol=[repmat(p.position_absolute_tolerance_m,3,1);repmat(p.velocity_absolute_tolerance_m_s,3,1)];
opts=odeset('RelTol',p.relative_tolerance,'AbsTol',[abstol;abstol;1e-9],'MaxStep',p.max_step_s);
y0=[Xc(1:6);Xt(1:6);Xc(14)];
try
    if wait>0, [tw,yw]=ode45(@rhs,[0 wait],y0,opts); else, tw=0; yw=y0'; end
catch e
    if ~strcmp(e.identifier,'mission:NominalWorkLimit'), rethrow(e); end
    plan.numerical_status="WORK_LIMIT"; finish(); return;
end
depart=yw(end,:)';
radial=depart(1:3)/norm(depart(1:3)); normal=cross(depart(1:3),depart(4:6)); normal=normal/norm(normal);
tangent=cross(normal,radial); basis=[radial tangent];
vtrans=sqrt(sys.mu*(2/norm(depart(1:3))-2/(norm(depart(1:3))+r2)));
u=basis'*(vtrans*tangent-depart(4:6));
status="ITERATION_LIMIT";
try
    [res,tf,yf]=shoot(u);
    for iteration=1:p.max_iterations
        plan.iterations=iteration;
        if norm(res)<=p.position_tolerance_m, status="CONVERGED"; break; end
        J=zeros(2);
        for j=1:2
            delta=zeros(2,1); delta(j)=p.difference_step_m_s;
            pert=shoot(u+delta); J(:,j)=(pert(1:2)-res(1:2))/delta(j);
        end
        if rcond(J)<1e-10, status="SINGULAR_JACOBIAN"; break; end
        update=-J\res(1:2); update=update*min(1,p.max_update_m_s/norm(update));
        accepted=false;
        for scale=[1 .5 .25]
            candidate=u+scale*update; [rr,tt,yy]=shoot(candidate);
            if norm(rr)<norm(res)
                u=candidate; res=rr; tf=tt; yf=yy; accepted=true; break;
            end
        end
        if ~accepted, status="NO_DESCENT"; break; end
    end
    if norm(res)<=p.position_tolerance_m, status="CONVERGED"; end
catch e
    if ~strcmp(e.identifier,'mission:NominalWorkLimit'), rethrow(e); end
    status="WORK_LIMIT";
end
plan.numerical_status=status;
if ~exist('yf','var'), finish(); return; end
terminal=yf(end,:)'; [~,vp,C]=orbit_core.relative_state(terminal(1:6),terminal(7:12));
dv1=basis*u; dv2=C'*(p.desired_velocity_lvlh_m_s-vp);
terminal(4:6)=terminal(4:6)+dv2;
terminal(13)=terminal(13)*exp(-norm(dv2)/(sys.Isp*sys.g0));
[rp,vp]=orbit_core.relative_state(terminal(1:6),terminal(7:12));
plan.position_error_m=norm(rp-goal); plan.velocity_error_m_s=norm(vp-p.desired_velocity_lvlh_m_s);
plan.target_tolerances_met=plan.position_error_m<=p.position_tolerance_m && ...
    plan.velocity_error_m_s<=p.velocity_tolerance_m_s;
plan.achieved_position_lvlh_m=rp; plan.achieved_velocity_lvlh_m_s=vp;
plan.departure_delta_v_eci_m_s=dv1; plan.arrival_delta_v_eci_m_s=dv2;
plan.delta_v_m_s=norm(dv1)+norm(dv2);
plan.sampled_min_altitude_m=min(vecnorm([yw(:,1:3);yf(:,1:3)],2,2))-sys.Re;
plan.constraint_status="CHECKED_BOUNDS_SATISFIED";
if max([norm(dv1),norm(dv2)])>max_burn || plan.sampled_min_altitude_m<=0
    plan.constraint_status="DEMONSTRATED_CONSTRAINT_VIOLATION";
end
plan.success=status=="CONVERGED" && plan.velocity_error_m_s<=p.velocity_tolerance_m_s && ...
    plan.constraint_status=="CHECKED_BOUNDS_SATISFIED";
plan.chaser=Xc; plan.chaser(1:6)=terminal(1:6); plan.chaser(14)=terminal(13);
plan.target=terminal(7:12); plan.fuel_kg=Xc(14)-terminal(13);
% Preserve both sides of the two instantaneous velocity/mass changes.
states=[yw;yf;terminal']; times=[tw;wait+tf;wait+T];
h=struct('pos',states(:,1:3)','vel',states(:,4:6)','mass',states(:,13)', ...
    'target_pos',states(:,7:9)','target_vel',states(:,10:12)', ...
    'time',times','time_end',times(end),'rel_pos',(states(:,1:3)-states(:,7:9))', ...
    'rel_pos_lvlh',zeros(3,numel(times)),'rel_vel_lvlh',zeros(3,numel(times)), ...
    'maneuver_delta_v',[norm(dv1) norm(dv2)],'maneuver_duration',[0 0], ...
    'maneuver_name',["nominal_departure" "nominal_arrival"]);
for k=1:numel(times)
    [h.rel_pos_lvlh(:,k),h.rel_vel_lvlh(:,k)]=orbit_core.relative_state(states(k,1:6)',states(k,7:12)');
end
plan.history=h; finish();
    function dy=rhs(t,y)
        if toc(started)>p.max_runtime_s
            error('mission:NominalWorkLimit','Nominal wall-time budget exhausted.');
        end
        dy=mission.proximity_dynamics(t,y,zeros(3,1),sys);
    end
    function [residual,t,y]=shoot(command)
        if evaluations>=p.max_evaluations || toc(started)>p.max_runtime_s
            error('mission:NominalWorkLimit','Nominal correction work budget exhausted.');
        end
        evaluations=evaluations+1; initial=depart;
        initial(4:6)=initial(4:6)+basis*command;
        initial(13)=initial(13)*exp(-norm(command)/(sys.Isp*sys.g0));
        [t,y]=ode45(@rhs,[0 T],initial,opts);
        rr=orbit_core.relative_state(y(end,1:6)',y(end,7:12)'); residual=rr-goal;
    end
    function finish()
        plan.evaluations=evaluations; plan.runtime_s=toc(started);
    end
end
