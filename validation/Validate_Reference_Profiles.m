function results=Validate_Reference_Profiles()
% Bounded profile precedence, domain, transcription and independent integrator checks.
root=fileparts(fileparts(mfilename('fullpath')));
s=Mission_Config('ARD'); a=entry_design.vehicle(s,'CAPSULE');
assert(a.shape.active_aoa_profile.classification=="REFERENCE_PROFILE_FALLBACK");
[alpha,~,info]=reentry_core.resolve_commands(a.shape,0,0,4500,15,65000,0);
assert(abs(alpha-19.26)<.15 && ~info.boundary_held);
[~,~,info]=reentry_core.resolve_commands(a.shape,0,0,4500,15,90000,0);
assert(info.boundary_held);
v=entry_design.vehicle(s,'CAPSULE',struct('aoa_deg',22));
assert(reentry_core.resolve_commands(v.shape,22,0,4500,15,65000,0)==22);
p=struct('axis',"ALTITUDE_M",'grid',[60000 70000],'values_deg',[18 22]);
v=entry_design.vehicle(s,'CAPSULE',struct('aoa_profile',p));
assert(reentry_core.resolve_commands(v.shape,0,0,4500,15,65000,0)==20);
fails(@()reentry_core.resolve_commands(v.shape,0,0,4500,15,90000,0),'reference_vehicle:ProfileDomain');
fails(@()entry_design.vehicle(s,'CAPSULE',struct('aoa_deg',20,'aoa_profile',p)), ...
    'reference_vehicle:AmbiguousAoA');
p.grid=[70000 60000];
fails(@()entry_design.vehicle(s,'CAPSULE',struct('aoa_profile',p)),'reference_vehicle:Profile');
custom=a.shape; custom=rmfield(custom,{'reference_aoa_profile','active_aoa_profile'});
v=entry_design.vehicle(s,'CAPSULE',struct('shape',custom));
assert(v.shape.active_aoa_profile.classification=="REFERENCE_PROFILE_FALLBACK");
table=jsondecode(fileread(fullfile(root,'configs','reference_profiles','ard_cfd_conditions_2016.json')));
assert(numel(table.altitude_m)==10 && table.altitude_m(1)==85000 && table.altitude_m(end)==40000);
% Changing a profile changes the compatibility record even when the name is unchanged.
r=Mission_Run_Config(s); c1=mission.physics_contract(s,r);
s.reentry_vehicle.reference_aoa_profile.values_deg(1)=21;
c2=mission.physics_contract(s,r); assert(~isequaln(c1,c2));
% A nonconstant HORUS command tests RK stages and mission elapsed-time plumbing.
s=Mission_Config('HORUS_2B'); s.environment.atmospheric_drag.use_matlab_atmosisa=false;
p=reference_vehicle.aoa_profile('HORUS_2B_PUBLISHED_TIME'); p.time_offset_s=1100;
v=entry_design.vehicle(s,'SPACEPLANE',struct('aoa_profile',p));
c=entry_design.defaults(); c.max_time_s=20; c.max_step_s=.25;
c.relative_tolerance=1e-10; c.terminal_altitude_m=20000; c.initial_bank_deg=0;
x=entry_design.initial_state(s,[25 40],65000,4500,-3,70,c);
out=entry_design.propagate(s,v,x,zeros(size(c.speed_fractions)),c);
params=struct('dt',.25,'max_time',20,'bank_angle_deg',0,'aoa_profile',p);
[~,~,hist,summary]=Reentry_Propagator(s,[x;0;0;0;1;0;0;0;v.mass_kg],[],0,params);
position_error=norm(hist.rv_pos(:,end)-out.state(end,1:3)');
assert(position_error<.01 && abs(hist.aoa_deg(1)-26.39)<.01);
assert(hist.aoa_deg(end)<hist.aoa_deg(1)-1);
assert(max(abs(interp1(out.time_s,out.aoa_deg,hist.time)-hist.aoa_deg))<1e-8);
assert(~summary.aoa_profile_boundary_held && all(hist.mass==v.mass_kg));
results=struct('status',"PASS",'duration_s',20,'position_disagreement_m',position_error, ...
    'initial_aoa_deg',hist.aoa_deg(1),'final_aoa_deg',hist.aoa_deg(end));
fprintf('Reference profiles: PASS (RK4/ODE45 position disagreement %.3g m)\n',position_error);
% Active user-selected speed schedule: knots, interpolation and actual air speed.
v=entry_design.vehicle(s,'SPACEPLANE');
assert(v.shape.active_aoa_profile.axis=="AIR_SPEED_M_S");
for pair=[1000 2000 3500 5000 7000;15 15 27.5 40 40]
    assert(abs(reentry_core.resolve_commands(v.shape,0,0,pair(1),15,65000,1100)-pair(2))<1e-12);
end
out=entry_design.propagate(s,v,x,zeros(size(c.speed_fractions)),c);
params=rmfield(params,'aoa_profile');
[~,~,hist]=Reentry_Propagator(s,[x;0;0;0;1;0;0;0;v.mass_kg],[],0,params);
speed_error=norm(hist.rv_pos(:,end)-out.state(end,1:3)');
expected=15+25*(hist.speed_rel-2000)/3000;
assert(speed_error<.01 && max(abs(hist.aoa_deg-expected))<1e-10);
results.speed_profile=struct('position_disagreement_m',speed_error, ...
    'initial_aoa_deg',hist.aoa_deg(1),'final_aoa_deg',hist.aoa_deg(end));
fprintf('HORUS speed profile: PASS (RK4/ODE45 position disagreement %.3g m)\n',speed_error);
end

function fails(f,id)
try
    f();
catch e
    assert(strcmp(e.identifier,id),e.message); return;
end
error('validation:ExpectedFailure','Expected %s',id);
end
