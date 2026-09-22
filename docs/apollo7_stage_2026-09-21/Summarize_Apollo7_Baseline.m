function report=Summarize_Apollo7_Baseline()
% Summarize saved bounded checks and inspect actual nominal deorbit handoff.
folder=fileparts(mfilename('fullpath'));
root=fileparts(fileparts(folder)); addpath(root);
checks=load(fullfile(folder,'validation.mat'),'results');
saved=load(fullfile(folder,'nominal.mat'),'nominal','nominal_runtime_s');
n=saved.nominal; a=checks.results.apollo7;
assert(checks.results.passed && n.proximity.reached_standoff);
report=struct('matlab_validation_passed',true,'validation_runtime_s',checks.results.elapsed_s, ...
    'nominal_orbit_runtime_s',saved.nominal_runtime_s,'nominal_standoff_reached',n.proximity.reached_standoff, ...
    'phase1_delta_v_m_s',n.phasing.delta_v,'phase2_delta_v_m_s',n.proximity.delta_v, ...
    'initial_stack_mass_kg',n.interfaces.initial.mass_kg, ...
    'post_proximity_mass_kg',n.interfaces.phase2_to_phase3.mass_kg, ...
    'entry_duration_s',a.nominal.time_s(end),'entry_endpoint_altitude_m',a.nominal.terminal_altitude_m, ...
    'entry_mach_range',a.mach_range,'entry_peak_dynamic_pressure_Pa',a.nominal.max_dynamic_pressure_Pa, ...
    'entry_peak_g',a.nominal.max_g_load,'entry_peak_heat_flux_W_m2',a.nominal.max_heat_flux_W_m2, ...
    'entry_heat_surrogate_only',true,'path_limits_specified',false, ...
    'short_cross_integrator_position_difference_m',a.cross_integrator_position_m);
sys=n.config.system;
t=tic;
[X,~,dv,fuel,hist]=mission.deorbit(sys,n.proximity.chaser,n.proximity.target,n.config.deorbit);
[accepted,interface]=mission.entry_interface(hist,X,sys,sys.h_entry_interface);
assert(isequal(accepted,X));
interface.achieved=mission.state_record(X,sys, ...
    n.interfaces.phase2_to_phase3.time_since_mission_epoch_s+interface.time_s, ...
    "PROPAGATED_DEORBIT_TERMINAL");
v=entry_design.vehicle(sys,'CAPSULE');
[~,~,~,sound]=Standard_Atmosphere_Density(interface.altitude_m,sys.environment.atmospheric_drag);
report.integrated_handoff=interface;
report.integrated_handoff_mach=interface.achieved.speed_air_relative_m_s/sound;
report.deorbit_delta_v_m_s=dv; report.deorbit_fuel_kg=fuel;
report.deorbit_runtime_s=toc(t);
report.integrated_entry_domain_status="SUPPORTED_AT_HANDOFF";
try
    reentry_core.evaluate_state([X(1:6);v.mass_kg],sys,v.shape,v.aoa_deg,0,true,v.heat_coefficient,0);
catch e
    if ~any(string(e.identifier)==["reference_vehicle:Domain","reference_vehicle:ProfileDomain"])
        rethrow(e);
    end
    report.integrated_entry_domain_status="OUTSIDE_PUBLISHED_DOMAIN_STATE_NOT_CHANGED";
    report.integrated_entry_domain_error=string(e.message);
end
save(fullfile(folder,'handoff.mat'),'X','hist','interface');
fid=fopen(fullfile(folder,'summary.json'),'w');
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
disp(report);
end
