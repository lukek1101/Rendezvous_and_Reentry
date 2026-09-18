function results = Validate_Mission_Architecture()
%VALIDATE_MISSION_ARCHITECTURE Full mission regression and runner isolation.
% Frozen orbital values were captured before the September 2026 extraction.
% This verifies software/numerical equivalence, not mission safety or physics.
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    caller_dir = pwd;
    caller_rng = rng;
    cleanup = onCleanup(@() restore_caller(caller_dir, caller_rng));
    options = struct('verbose',false,'seed',123);
    options.system = struct('mu',3.986004418e14,'Re',6378137,'J2',1.08263e-3, ...
        'g0',9.80665,'Isp',200,'Target_Mass',2000,'Chaser_Mass_Init',2000, ...
        'h_insert',300e3,'h_target',500e3,'initial_chaser_angle_deg',0,'initial_phase_angle_deg',90);
    overrides.runtime.allow_environment_overrides = false;
    overrides.python_config.mode = "FILE";
    overrides.python_config.file = "configs/python_runs/impulsive_drag-off_h300.0-500.0_phase90.0_31c1e45c_410e0069_20260630_134559Z.json";
    overrides.maneuver.burn_model = "IMPULSIVE";
    overrides.maneuver.use_thrust_noise = false;
    overrides.environment.atmospheric_drag.enabled = false;
    overrides.reentry.vehicle_mode = "CAPSULE";
    overrides.reentry.capsule.mass_kg = 60;
    overrides.reentry.capsule.add_to_chaser_initial_mass = true;
    overrides.reentry.capsule.separation_mode = "ENTRY_INTERFACE";
    overrides.reentry.capsule.use_paper_entry_conditions = true;
    overrides.reentry.capsule.altitude_termination_enabled = false;
    overrides.phase2 = struct('dt_s',1,'S2_m',[0;-5000;0],'S4_R_abs_m',30, ...
        'initial_S2_tol_m',50,'tof_initial_s2_s',1200,'delta_R_cycloid_m',400, ...
        'vbar_burn_sign',-1,'vbar_cross_tol_m',1,'max_cycloid_orbits',4, ...
        'rbar_hop_count',8,'tof_hop_s',300,'capture_pos_tol_m',0.25, ...
        'max_terminal_refines',4,'tof_terminal_refine_s',180,'Isp_fallback_s',220);
    overrides.phase3.mode = "HOHMANN";
    overrides.phase2.mode = "LEGACY_IMPULSIVE";
    overrides.phase3.dt_reentry_coast_s = 2;
    overrides.phase3.max_reentry_coast_time_s = [];
    overrides.phase1.phase_angle_deg = [];
    overrides.phase1.delta_v_m_s = [];
    overrides.phase1.gamma_deg = [];

    % A different working directory must not change JSON lookup or create figures.
    figures_before = findall(groot, 'Type','figure');
    cd(tempdir);
    test_dir = pwd;
    result = Run_Mission(overrides, options);
    assert(isequal(rng,caller_rng), 'A seeded run changed the caller RNG.');
    assert(isequal(findall(groot,'Type','figure'),figures_before), 'Headless run created figures.');
    assert(strcmp(pwd,test_dir), 'Runner changed the working directory.');

    expected = [-5409479.464207225;0;3600460.581243640; ...
                -4270.932858436175;0;-6682.012366061912];
    assert(max(abs(result.deorbit.chaser(1:6)-expected)) < 1e-4, 'Orbital handoff regression changed.');
    expected_masses = [1999.981205554297;1938.608942624277;1821.636469201103];
    assert(max(abs(result.budget.Remaining_Mass_kg(1:3)-expected_masses)) < 1e-7);
    assert(abs(result.phasing.history.time(end)-32996.731705843835) < 1e-5);
    assert(result.proximity.duration == 16778 && result.proximity.reached_standoff);
    assert(abs(result.deorbit.interface.time_s-2173.238658595189) < 1e-5);
    assert(abs(result.deorbit.interface.altitude_m-120e3) < 1e-3);
    assert(result.entry.summary.termination_reason == "PARACHUTE_SPEED");
    assert(result.entry.vehicle(14) == 60);
    assert(abs(result.budget.Total_Accounted_Mass_kg(4)-expected_masses(3)) < 1e-7);
    assert(strlength(result.log)>0 && ~isfield(result.config.system,'h_reentry'));

    % Typos and zero timesteps must fail before entering a propagation loop;
    % the seed must also be restored on these error paths.
    invalid = overrides;
    invalid.phase2.dt_s = 0;
    assert_fails(@() Run_Mission(invalid,options));
    assert(isequal(rng,caller_rng));
    invalid = overrides;
    invalid.phase2.typo_dt = 1;
    assert_fails(@() Run_Mission(invalid,options), 'mission:UnknownSetting');
    invalid = overrides;
    invalid.phase3.mode = "REMOVED_MODE";
    assert_fails(@() Run_Mission(invalid,options), 'mission:UnsupportedDeorbitMode');

    results.passed = true;
    results.mission_elapsed_seconds = result.metadata.elapsed_seconds;
    results.maximum_orbital_state_error = max(abs(result.deorbit.chaser(1:6)-expected));
    fprintf('Mission architecture: PASS (headless, external cwd, RNG, regression, input errors)\n');
    clear cleanup
end

function assert_fails(action, identifier)
    try
        action();
    catch err
        if nargin > 1
            assert(strcmp(err.identifier,identifier), 'Unexpected error: %s',err.message);
        end
        return;
    end
    error('Expected an invalid configuration to fail.');
end

function restore_caller(folder, state)
    cd(folder);
    rng(state);
end
