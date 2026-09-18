% Interactive entry point. The reusable API is Run_Mission(overrides, options).
% No clear/close-all: existing workspace data and figures belong to the caller.
mission_result = Run_Mission(struct(), struct('plot', true));

% Compatibility names for interactive analysis of earlier mission scripts.
Budget = mission_result.budget;
sys = mission_result.config.system;
run_cfg = mission_result.config.run;
mission_cfg = mission_result.config.optimizer;
custom_params = mission_result.config.deorbit;
phasing_mode = mission_result.config.phase1_mode;
reentry_mode = mission_result.config.phase3.mode;
p2 = mission_result.proximity.config;
hist_p1 = mission_result.phasing.history;
hist_p2 = mission_result.proximity.history;
hist_p3 = mission_result.deorbit.history;
hist_reentry = mission_result.entry.history;
hist_pos = mission_result.proximity.relative_position;
hist_mass = mission_result.proximity.mass;
phase2_targets = mission_result.proximity.targets;
phase2_target_times = mission_result.proximity.target_times;
phase2_tofs = mission_result.proximity.transfer_times;
phase2_names = mission_result.proximity.names;
phase2_time = mission_result.proximity.duration;
X_chaser = mission_result.deorbit.chaser;
X_target = mission_result.deorbit.target;
X_entry_interface = mission_result.entry.initial_state;
X_reentry_vehicle = mission_result.entry.vehicle;
entry_interface_info = mission_result.deorbit.interface;
reentry_info = mission_result.deorbit.info;
reentry_atmo_info = mission_result.entry.summary;
dV_p1 = mission_result.phasing.delta_v;
dV_p2 = mission_result.proximity.delta_v;
dV_p3 = mission_result.deorbit.delta_v;
fuel_p1 = mission_result.phasing.fuel;
fuel_p2 = mission_result.proximity.fuel;
fuel_p3 = mission_result.deorbit.fuel;
