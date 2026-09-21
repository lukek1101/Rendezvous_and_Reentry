% Audit-only driver. No optimizer or study-output regeneration.
audit_dir = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(audit_dir));
addpath(root); addpath(fullfile(root,'validation'));
diary(fullfile(audit_dir,'matlab_checks.log'));
fprintf('MATLAB %s; %s\n',version,computer);
disp(ver);
checks.orbit = Validate_Orbit_Core();
checks.entry_core = Validate_Reentry_Core_Equivalence();
checks.deorbit = Validate_Deorbit_Execution();
checks.paper_audit = Run_Paper_Reproduction_Suite();
checks.legacy = Validate_Mission_Architecture();
settings.runtime.allow_environment_overrides = false;
settings.python_config.mode = "FILE";
settings.python_config.file = "configs/python_runs/impulsive_drag-off_h300.0-500.0_phase90.0_31c1e45c_410e0069_20260630_134559Z.json";
settings.maneuver.burn_model = "IMPULSIVE";
settings.maneuver.use_thrust_noise = false;
settings.environment.atmospheric_drag.enabled = false;
settings.phase2.mode = "HYBRID_AUTONOMOUS";
options = struct('plot',false,'verbose',false,'seed',123);
result = Run_Mission(settings,options);
disp(result.budget);
summary = struct('matlab_version',version,'phase1_error_m',result.phasing.position_error, ...
    'phase1_duration_s',result.phasing.history.time(end), ...
    'phase2_duration_s',result.proximity.duration, ...
    'phase2_error_m',result.proximity.final_position_error, ...
    'phase2_speed_m_s',norm(result.proximity.final_relative_velocity), ...
    'phase2_delta_v_m_s',result.proximity.delta_v, ...
    'entry_interface',result.deorbit.interface, ...
    'entry_termination',result.entry.summary.termination_reason, ...
    'budget',table2struct(result.budget), ...
    'elapsed_s',result.metadata.elapsed_seconds);
disp(summary);
save(fullfile(audit_dir,'baseline.mat'),'checks','settings','options','result','summary');
fid=fopen(fullfile(audit_dir,'hybrid_summary.json'),'w');
fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true)); fclose(fid);
fprintf('AUDIT_BASELINE_COMPLETE\n');
diary off;
