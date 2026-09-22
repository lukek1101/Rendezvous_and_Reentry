function result=Validate_Nominal_Planning()
% Targeting acceptance is independent of cost optimization and integration error.
s=Mission_Config('ARD'); r=Mission_Run_Config(s); r.python_config.mode="NONE";
r.phase1.mode="HOHMANN"; cfg=mission.configure(s,r); s=cfg.proximity_system;
radius=s.Re+s.h_insert; target=s.Re+s.h_target;
x=[radius;0;0;0;0;sqrt(s.mu/radius*(1+s.J2*(s.Re/radius)^2));0;0;0;1;0;0;0;4800];
t=[0;0;target;-sqrt(s.mu/target*(1-2*s.J2*(s.Re/target)^2));0;0];
p=cfg.phase1.nominal; goal=[0;-5000;0];
a=mission.plan_nominal(s,x,t,goal,cfg.phase1.max_wait,p);
assert(a.success && a.evaluations<=p.max_evaluations);
assert(a.position_error_m<p.position_tolerance_m && a.velocity_error_m_s<p.velocity_tolerance_m_s);
assert(numel(a.history.maneuver_delta_v)==2);
assert(abs(a.chaser(14)-x(14)*exp(-a.delta_v_m_s/(s.Isp*s.g0)))<1e-9);
assert(isequal(a.chaser(1:6),[a.history.pos(:,end);a.history.vel(:,end)]));
assert(isequal(a.history.pos(:,end),a.history.pos(:,end-1))); % arrival impulse, no position reset
tight=p; tight.relative_tolerance=p.relative_tolerance/10;
tight.position_absolute_tolerance_m=p.position_absolute_tolerance_m/10;
tight.velocity_absolute_tolerance_m_s=p.velocity_absolute_tolerance_m_s/10;
b=mission.plan_nominal(s,x,t,goal,cfg.phase1.max_wait,tight);
difference=norm(a.chaser(1:3)-b.chaser(1:3)); assert(b.success && difference<.05);
limited=p; limited.max_evaluations=1;
c=mission.plan_nominal(s,x,t,goal,cfg.phase1.max_wait,limited);
assert(~c.success && c.numerical_status=="WORK_LIMIT" && ~c.target_tolerances_met);
assert(c.constraint_status=="CHECKED_BOUNDS_SATISFIED"); % not proof of infeasibility
c=mission.plan_nominal(s,x,t,goal,cfg.phase1.max_wait,p,1);
assert(~c.success && c.numerical_status=="CONVERGED" && c.constraint_status=="DEMONSTRATED_CONSTRAINT_VIOLATION");
c=mission.plan_nominal(s,x,t,goal,1,p);
assert(~c.success && c.constraint_status=="WAIT_BOUND_VIOLATED");
limited=p; limited.max_runtime_s=1e-6;
c=mission.plan_nominal(s,x,t,goal,cfg.phase1.max_wait,limited);
assert(~c.success && c.numerical_status=="WORK_LIMIT");
result=struct('passed',true,'position_error_m',a.position_error_m, ...
    'velocity_error_m_s',a.velocity_error_m_s,'integration_refinement_difference_m',difference, ...
    'evaluations',a.evaluations);
fprintf('Nominal targeting: PASS (integration refinement %.4g m)\n',difference);
end
