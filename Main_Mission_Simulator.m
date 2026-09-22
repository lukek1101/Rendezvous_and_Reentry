% 기본 실행: orbital rendezvous부터 30 m standoff까지.
% FULL_MISSION은 실제 entry state가 aerodynamic 범위를 벗어나면 중단됩니다.
simulation_scope = "ORBIT_ONLY"; % "ORBIT_ONLY" / "FULL_MISSION"
mission_overrides = struct('python_config',struct('mode',"NONE"), ...
    'phase1',struct('mode',"HOHMANN",'hohmann_method',"NOMINAL_TARGET"));
% 다른 설정은 Mission_Run_Config.m을 따릅니다. 위 세 항목은 여기서 덮어씁니다.
if simulation_scope == "ORBIT_ONLY"
    fprintf('ORBIT_ONLY: Phases 1-2 to 30 m standoff; deorbit/entry are not executed.\n');
    mission_result = Run_Nominal_Orbit(mission_overrides, struct('plot', true));
elseif simulation_scope == "FULL_MISSION"
    mission_result = Run_Mission(mission_overrides, struct('plot', true));
else
    error('mission:RunScope','Use ORBIT_ONLY or FULL_MISSION.');
end
mission_result.metadata.requested_scope = simulation_scope;

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
hist_p3 = []; hist_reentry = [];
hist_pos = mission_result.proximity.relative_position;
hist_mass = mission_result.proximity.mass;
phase2_targets = mission_result.proximity.targets;
phase2_target_times = mission_result.proximity.target_times;
phase2_tofs = mission_result.proximity.transfer_times;
phase2_names = mission_result.proximity.names;
phase2_time = mission_result.proximity.duration;
X_chaser = mission_result.proximity.chaser;
X_target = mission_result.proximity.target;
X_entry_interface = []; X_reentry_vehicle = [];
entry_interface_info = []; reentry_info = []; reentry_atmo_info = [];
dV_p1 = mission_result.phasing.delta_v;
dV_p2 = mission_result.proximity.delta_v;
dV_p3 = [];
fuel_p1 = mission_result.phasing.fuel;
fuel_p2 = mission_result.proximity.fuel;
fuel_p3 = [];
if isfield(mission_result,'entry')
    hist_p3 = mission_result.deorbit.history;
    hist_reentry = mission_result.entry.history;
    X_chaser = mission_result.deorbit.chaser;
    X_target = mission_result.deorbit.target;
    X_entry_interface = mission_result.entry.initial_state;
    X_reentry_vehicle = mission_result.entry.vehicle;
    entry_interface_info = mission_result.deorbit.interface;
    reentry_info = mission_result.deorbit.info;
    reentry_atmo_info = mission_result.entry.summary;
    dV_p3 = mission_result.deorbit.delta_v;
    fuel_p3 = mission_result.deorbit.fuel;
end
