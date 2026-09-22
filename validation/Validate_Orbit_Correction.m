function result=Validate_Orbit_Correction()
% Same realized errors, no state resets; all delivered impulses/fuel accounted.
s=Mission_Config('ARD'); r=Mission_Run_Config(s); r.python_config.mode="NONE";
r.phase1.mode="HOHMANN"; cfg=mission.configure(s,r); s=cfg.proximity_system;
radius=s.Re+s.h_insert; target=s.Re+s.h_target;
x=[radius;0;0;0;0;sqrt(s.mu/radius*(1+s.J2*(s.Re/radius)^2));0;0;0;1;0;0;0;4800];
t=[0;0;target;-sqrt(s.mu/target*(1-2*s.J2*(s.Re/target)^2));0;0];
p=cfg.phase1.nominal; goal=[0;-5000;0];
plan=mission.plan_nominal(s,x,t,goal,cfg.phase1.max_wait,p);
c=mission.correction_defaults(); c.enabled=true;
caller_rng=rng;
on=mission.execute_corrected_nominal(s,x,t,goal,plan,p,c,inf);
assert(isequal(caller_rng,rng));
c.corrections_enabled=false;
off=mission.execute_corrected_nominal(s,x,t,goal,plan,p,c,inf);
assert(on.success && ~off.success);
assert(isequal(on.initial_error_eci_si,off.initial_error_eci_si));
assert(isequal(on.gain_draws,off.gain_draws) && isequal(on.pointing_draws_rad,off.pointing_draws_rad));
assert(isequal(on.burns(1).delivered_eci_m_s,off.burns(1).delivered_eci_m_s));
assert(on.elapsed_s==off.elapsed_s && on.correction_count<=c.max_corrections);
assert(norm(on.history.pos(:,1)-x(1:3)-on.initial_error_eci_si(1:3))<1e-9);
assert(on.correction_delta_v_m_s<=c.max_total_correction_delta_v_m_s && on.fuel_kg<=c.max_propellant_kg);
assert(abs(sum([on.burns.fuel_kg])-on.fuel_kg)<1e-9);
assert(abs(sum([on.burns.delivered_delta_v_m_s])-on.delta_v_m_s)<1e-12);
for burn=on.burns'
    assert(isequal(burn.state_before(1:3),burn.state_after(1:3)));
    assert(isequal(burn.state_before(7:12),burn.state_after(7:12)));
    assert(norm(burn.state_after(4:6)-burn.state_before(4:6)-burn.delivered_eci_m_s)<1e-9);
    assert(abs(burn.state_after(13)-burn.state_before(13)*exp(-burn.delivered_delta_v_m_s/(s.Isp*s.g0)))<1e-9);
end
assert(norm(on.chaser(1:3)-on.history.pos(:,end))==0);
c.corrections_enabled=true; tight=p; tight.relative_tolerance=p.relative_tolerance/10;
tight.position_absolute_tolerance_m=p.position_absolute_tolerance_m/10;
tight.velocity_absolute_tolerance_m_s=p.velocity_absolute_tolerance_m_s/10;
refined=mission.execute_corrected_nominal(s,x,t,goal,plan,tight,c,inf);
assert(refined.success && norm(refined.chaser(1:3)-on.chaser(1:3))<.05);
limited=c; limited.max_corrections=0;
e=mission.execute_corrected_nominal(s,x,t,goal,plan,p,limited,inf);
assert(~e.success && e.correction_count==0 && e.execution_status=="CORRECTION_COUNT_LIMIT");
limited=c; limited.max_total_correction_delta_v_m_s=0;
e=mission.execute_corrected_nominal(s,x,t,goal,plan,p,limited,inf);
assert(~e.success && e.correction_count==0 && e.execution_status=="CORRECTION_DV_LIMIT");
limited=c; limited.max_propellant_kg=0;
e=mission.execute_corrected_nominal(s,x,t,goal,plan,p,limited,inf);
assert(e.execution_status=="PROPELLANT_LIMIT" && isempty(e.burns) && e.fuel_kg==0);
limited=c; limited.max_elapsed_s=1;
e=mission.execute_corrected_nominal(s,x,t,goal,plan,p,limited,inf);
assert(e.execution_status=="ELAPSED_TIME_LIMIT" && e.elapsed_s==0 && isempty(e.burns));
limited=c; limited.max_runtime_s=1e-6;
e=mission.execute_corrected_nominal(s,x,t,goal,plan,p,limited,inf);
assert(e.execution_status=="NUMERICAL_WORK_LIMIT" && ~e.success);
limited=c; limited.max_correction_delta_v_m_s=.00001;
e=mission.execute_corrected_nominal(s,x,t,goal,plan,p,limited,inf);
assert(~e.success && e.correction_count==0 && e.execution_status=="MANEUVER_CAPABILITY_LIMIT");
limited=c; limited.max_evaluations=0;
e=mission.execute_corrected_nominal(s,x,t,goal,plan,p,limited,inf);
assert(~e.success && all([e.opportunities.status]=="EVALUATION_LIMIT"));
zero=c; zero.position_sigma_m=zeros(3,1); zero.velocity_sigma_m_s=zeros(3,1);
zero.burn_gain_sigma=0; zero.burn_pointing_sigma_rad=0;
e=mission.execute_corrected_nominal(s,x,t,goal,plan,p,zero,inf);
assert(e.success && e.correction_count==0 && numel(e.burns)==2);
result=struct('passed',true,'off_position_error_m',off.position_error_m, ...
    'on_position_error_m',on.position_error_m,'on_velocity_error_m_s',on.velocity_error_m_s, ...
    'correction_delta_v_m_s',on.correction_delta_v_m_s, ...
    'integration_refinement_m',norm(refined.chaser(1:3)-on.chaser(1:3)));
fprintf('Bounded orbit correction: PASS (%.6g m, %.6g m/s; refinement %.6g m)\n', ...
    on.position_error_m,on.velocity_error_m_s,result.integration_refinement_m);
end
