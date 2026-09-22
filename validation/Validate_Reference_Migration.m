function results=Validate_Reference_Migration()
% Bounded transcription, domain, frame, override, stale-artifact and coast tests.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root);
ard=Mission_Config('ARD'); horus=Mission_Config('HORUS_2B');
assert(ard.reentry_vehicle.capsule.mass_kg==2800);
assert(abs(ard.reentry_vehicle.shapes.CAPSULE.reference_area_m2-6.157521601035993)<1e-12);
assert(ard.reentry_vehicle.shapes.CAPSULE.nose_radius_m==3.36);
assert(horus.reference.entry_mass_kg==26029 && horus.reentry_vehicle.shapes.HORUS_2B.reference_area_m2==110);
assert(~any(isfield(ard.reentry_vehicle.shapes,{'COMPROMISE','HEATLOAD_MIN','PAYLOAD_MAX','TPS_MIN'})));
h=entry_design.vehicle(horus,'SPACEPLANE',struct('aoa_deg',40));
a=entry_design.vehicle(ard,'CAPSULE',struct('aoa_deg',20));
[cd,cl]=reentry_core.aerodynamic_coefficients(h.shape,40,20);
assert(cd==.70 && cl==.77); % independently read M-692 high-M/high-alpha corner
[cd,cl]=reentry_core.aerodynamic_coefficients(h.shape,20,10);
assert(cd==.18 && cl==.36);
[cd,cl]=reentry_core.aerodynamic_coefficients(h.shape,0,1.2);
assert(cd==.10 && cl==-.02);
[cd,cl]=reentry_core.aerodynamic_coefficients(h.shape,22.5,7.5);
assert(abs(cd-.245)<1e-12 && abs(cl-.45)<1e-12);
data=reference_vehicle.horus_data();
assert(nnz(isnan(data.cd))==4 && nnz(isnan(data.cl))==7);
fails(@() reentry_core.aerodynamic_coefficients(h.shape,40,1.2),'reference_vehicle:MissingData');
fails(@() reentry_core.aerodynamic_coefficients(h.shape,40,21),'reference_vehicle:Domain');
fails(@() reentry_core.aerodynamic_coefficients(a.shape,14,20),'reference_vehicle:Domain');
[cd,cl]=reentry_core.aerodynamic_coefficients(a.shape,20,15);
assert(abs(cd-(1.36*cosd(20)+.07*sind(20)))<1e-12);
assert(abs(cl-(1.36*sind(20)-.07*cosd(20)))<1e-12);
r=Mission_Run_Config(ard); r.python_config.mode="NONE"; r.phase3.flight_path_angle_deg=3.7;
r.phase1.mode="HOHMANN";
cfg=mission.configure(ard,r); assert(abs(rad2deg(cfg.system.reentry_flight_path_angle)-3.7)<1e-12);
assert(cfg.system.Chaser_Mass_Init==2000);
old=jsondecode(fileread(fullfile(root,'configs','latest_python_solution.json')));
fails(@()mission.validate_optimizer_compatibility(old,cfg.contract,r,"ARD"),'mission:MissingOptimizerContract');
fresh=struct('compatibility',jsondecode(jsonencode(cfg.contract)),'optimizer',struct('success',true));
mission.validate_optimizer_compatibility(fresh,cfg.contract,r,"ARD");
bad=cfg.contract; bad.initial_stack_mass_kg=bad.initial_stack_mass_kg+1;
fails(@()mission.validate_optimizer_compatibility(fresh,bad,r,"ARD"),'mission:IncompatibleOptimizer');
bad=cfg.contract; bad.initial_phase_angle_deg=91;
fails(@()mission.validate_optimizer_compatibility(fresh,bad,r,"ARD"),'mission:IncompatibleOptimizer');
% Earth-fixed geometry round trip with nonzero Earth angle and nonpolar state.
c=entry_design.defaults(); c.entry_epoch_s=123;
[x,prescribed]=entry_design.initial_state(ard,[25 40],65e3,4500,-3,70,c);
record=mission.state_record([x;2800],ard,123,'STANDALONE_PRESCRIBED');
assert(abs(record.fpa_air_relative_deg+3)<1e-10 && abs(record.speed_air_relative_m_s-4500)<1e-9);
assert(prescribed.source=="STANDALONE_PRESCRIBED");
% Short 20 s propagation, well inside both declared aerodynamic domains.
for k=1:2
    if k==1, sys=ard; v=a; alpha=20; else, sys=horus; v=h; alpha=40; end
    sys.environment.atmospheric_drag.use_matlab_atmosisa=false;
    X=[x;v.mass_kg]; coarse=X; fine=X;
    for j=1:40, coarse=reentry_core.rk4_step(coarse,sys,v.shape,alpha,0,true,v.heat_coefficient,.5); end
    for j=1:80, fine=reentry_core.rk4_step(fine,sys,v.shape,alpha,0,true,v.heat_coefficient,.25); end
    assert(norm(coarse(1:3)-fine(1:3))<.01 && coarse(7)==X(7));
    results.propagation(k)=struct('preset',sys.reference.preset, ...
        'duration_s',20,'position_refinement_m',norm(coarse(1:3)-fine(1:3)), ...
        'final_state_si',fine);
end
% Integrated interface accepts only an actual propagated descending endpoint.
s=Mission_Config('LEGACY_CAPSULE_60KG');
X=[s.Re+120e3;0;0;-200;7500;0;0;0;0;1;0;0;0;1800];
hist=struct('pos',X(1:3),'vel',X(4:6),'mass',X(14),'time',100);
[entry,info]=mission.entry_interface(hist,X,s,120e3);
assert(isequal(entry,X) && info.crossing_verified);
assert(abs(info.fpa_error_deg)>.1); % requested != achieved; state preserved
fails(@()mission.entry_interface(struct(),X,s,120e3),'mission:MissingEntryHistory');
wrong=X; wrong(1)=wrong(1)+10;
fails(@()mission.entry_interface(hist,wrong,s,120e3),'mission:NoEntryCrossing');
results.status="PASS";
end

function fails(f,id)
try
    f();
catch e
    assert(strcmp(e.identifier,id),e.message); return;
end
error('validation:ExpectedFailure','Expected %s',id);
end
