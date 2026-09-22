% Bounded migration verification; no optimization and no old output overwrite.
folder=fileparts(mfilename('fullpath')); root=fileparts(fileparts(folder));
addpath(root); addpath(fullfile(root,'validation'));
diary(fullfile(folder,'matlab_checks.log'));
checks=Run_All_Validations();
settings=struct(); settings.runtime.allow_environment_overrides=false;
settings.python_config.mode="FILE";
settings.python_config.allow_legacy_replay=true;
settings.python_config.file="configs/python_runs/impulsive_drag-off_h300.0-500.0_phase90.0_31c1e45c_410e0069_20260630_134559Z.json";
options=struct('preset',"LEGACY_CAPSULE_60KG",'seed',123,'verbose',false);
result=Run_Mission(settings,options);
assert(isequal(result.deorbit.chaser,result.deorbit.interface_state));
assert(isequal(result.entry.initial_state(1:13),result.deorbit.interface_state(1:13)));
assert(result.entry.initial_state(14)==60);
assert(abs(result.entry.summary.total_accounted_mass_kg-result.deorbit.chaser(14))<1e-9);
assert(abs(result.interfaces.phase2_to_phase3.mass_kg-result.proximity.chaser(14))<1e-9);
assert(abs(result.entry.achieved_conditions.fpa_inertial_deg-result.entry.requested_conditions.fpa_inertial_deg)>.01);
summary=struct('reference_checks',checks.reference_migration, ...
    'hybrid_entry_requested',result.entry.requested_conditions, ...
    'hybrid_entry_achieved',result.entry.achieved_conditions, ...
    'hybrid_phase2_error_m',result.proximity.final_position_error, ...
    'hybrid_phase2_delta_v_m_s',result.proximity.delta_v, ...
    'hybrid_interface_preserved',true,'matlab_version',version);
save(fullfile(folder,'verification.mat'),'checks','settings','options','summary');
fid=fopen(fullfile(folder,'summary.json'),'w');
fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true)); fclose(fid);
fprintf('MIGRATION_VERIFICATION_PASS\n'); diary off;
