function result=Validate_Apollo7()
%VALIDATE_APOLLO7 Source knots, trim contract, default selection and propagation.
s=Mission_Config(); assert(s.reference.preset=="APOLLO7_PREFLIGHT_TRIM");
v=entry_design.vehicle(s,'CAPSULE'); d=reference_vehicle.apollo7_data();
assert(abs(v.mass_kg-5608.261421917)<1e-8);
assert(abs(v.shape.reference_area_m2-12.021653376)<1e-10);
assert(v.shape.nose_radius_m==4.694);
assert(numel(d.mach)==12 && all(diff(d.mach)>0));
assert(max(abs(d.cl./d.cd-d.published_ld))<5e-5); % source rounding
for k=1:numel(d.mach)
    alpha=reentry_core.resolve_commands(v.shape,0,0,NaN,d.mach(k),NaN,0);
    [cd,cl]=reentry_core.aerodynamic_coefficients(v.shape,alpha,d.mach(k));
    assert(abs(alpha+d.apollo_alpha_deg(k)-180)<1e-12);
    assert(abs(cd-d.cd(k))<1e-12 && abs(cl-d.cl(k))<1e-12);
end
% Independent source anchors from Table IIb, not from the interpolation input.
[cd,cl]=reentry_core.aerodynamic_coefficients(v.shape,25.24,1.1);
assert(cd==1.1684 && cl==.49540);
[cd,cl]=reentry_core.aerodynamic_coefficients(v.shape,(12.83+15.47)/2,.55);
assert(abs(cd-(.8531+.9852)/2)<1e-12 && abs(cl-(.24399+.26368)/2)<1e-12);
fails(@()reentry_core.aerodynamic_coefficients(v.shape,20,.39),'reference_vehicle:Domain');
fails(@()reentry_core.aerodynamic_coefficients(v.shape,20,28),'reference_vehicle:Domain');
fails(@()reentry_core.aerodynamic_coefficients(v.shape,20,1.1),'reference_vehicle:TrimOnly');
custom=entry_design.vehicle(s,'CAPSULE',struct('aoa_deg',20));
assert(~isfield(custom.shape,'active_aoa_profile'));
fails(@()reentry_core.aerodynamic_coefficients(custom.shape,20,1.1),'reference_vehicle:TrimOnly');
ard=Mission_Config('ARD'); assert(ard.reentry_vehicle.capsule.mass_kg==2800);
horus=Mission_Config('HORUS_2B'); assert(horus.reentry_vehicle.vehicle_mode=="SPACEPLANE");
other=entry_design.vehicle(horus,'CAPSULE'); assert(other.shape.aero_model==s.reference.preset);
run=Mission_Run_Config(s); run.python_config.mode="NONE"; run.phase1.mode="HOHMANN";
cfg=mission.configure(s,run); assert(cfg.system.reference.preset==s.reference.preset);
ac=mission.physics_contract(ard,Mission_Run_Config(ard));
assert(~isequaln(cfg.contract,ac) && isfield(cfg.contract.physics_source_sha256,'apollo7_data'));
old=struct('compatibility',ac,'optimizer',struct('success',true));
fails(@()mission.validate_optimizer_compatibility(old,cfg.contract,run,s.reference.preset), ...
    'mission:IncompatibleOptimizer');
% Actual-state adapter and both propagators preserve a deliberately nonreference mass.
s.environment.atmospheric_drag.use_matlab_atmosisa=false;
c=entry_design.defaults(); c.max_time_s=20; c.max_step_s=.25; c.relative_tolerance=1e-10;
x=entry_design.initial_state(s,[25 40],65000,4500,-3,70,c);
v.mass_kg=v.mass_kg-17;
a=entry_design.propagate(s,v,x,[0 0 0],c);
X=[x;0;0;0;1;0;0;0;v.mass_kg];
[fine,~,history]=Reentry_Propagator(s,X,[],0,struct('dt',.25,'max_time',20,'separation_mode','ATTACHED'));
assert(isequal(history.rv_pos(:,1),x(1:3)) && isequal(history.rv_vel(:,1),x(4:6)));
assert(all(history.mass==v.mass_kg) && all(a.state(:,7)==v.mass_kg));
stack=X; stack(14)=d.mass_kg+2000;
[~,~,separated]=Reentry_Propagator(s,stack,[],0,struct('dt',.25,'max_time',.25));
assert(isequal(separated.rv_pos(:,1),x(1:3)) && isequal(separated.rv_vel(:,1),x(4:6)));
assert(separated.mass(1)==d.mass_kg && stack(14)-separated.mass(1)==2000);
event=stack; event(1:6)=entry_design.initial_state(s,[25 40],s.h_entry_interface,7400,-3,70,c);
eh=struct('pos',event(1:3),'vel',event(4:6),'mass',event(14),'time',123);
[accepted,info]=mission.entry_interface(eh,event,s,s.h_entry_interface);
assert(isequal(event,accepted) && abs(info.fpa_error_deg)>.1);
result.entry_interface=info;
result.cross_integrator_position_m=norm(fine(1:3)-a.state(end,1:3)');
assert(result.cross_integrator_position_m<1e-3);
% Complete atmospheric-entry segment to a declared pre-parachute study endpoint.
% Project initial conditions, not a claimed Apollo flight replay.
c.max_time_s=1800; c.max_step_s=2; c.relative_tolerance=1e-9;
c.terminal_altitude_m=7620; c.initial_bank_deg=30;
x=entry_design.initial_state(s,[25 40],121920,7400,-2,70,c);
v=entry_design.vehicle(s,'CAPSULE');
nominal=entry_design.propagate(s,v,x,[30 30 30],c);
assert(nominal.terminal_reached && all(isfinite(nominal.state(:))));
mach=zeros(size(nominal.time_s));
for k=1:numel(mach)
    aux=reentry_core.evaluate_state(nominal.state(k,1:7)',s,v.shape,0, ...
        nominal.state(k,8),true,v.heat_coefficient,nominal.time_s(k));
    mach(k)=aux.mach;
end
assert(min(mach)>=.4 && max(mach)<=27.72);
result.nominal=nominal; result.mach_range=[min(mach) max(mach)];
result.status="PASS";
end
function fails(f,id)
try
    f();
catch e
    assert(strcmp(e.identifier,id),e.message);
    return;
end
error('validation:ExpectedFailure','Expected %s',id);
end
