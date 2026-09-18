function report = Study_Proximity_Options(output_dir)
%STUDY_PROXIMITY_OPTIONS Reproducible mission comparison and small dispersion pilot.
% Not a statistical reliability demonstration or a flight safety certification.
    root = fileparts(fileparts(mfilename('fullpath'))); addpath(root);
    if nargin < 1
        output_dir = fullfile(root,'output','proximity');
    end
    if ~isfolder(output_dir), mkdir(output_dir); end
    old_rng = rng; cleanup = onCleanup(@() rng(old_rng)); rng(1809,'twister');
    o.runtime.allow_environment_overrides = false;
    o.python_config.mode = "FILE";
    o.python_config.file = "configs/python_runs/impulsive_drag-off_h300.0-500.0_phase90.0_31c1e45c_410e0069_20260630_134559Z.json";
    o.maneuver.burn_model = "IMPULSIVE";
    o.phase2.mode = "HYBRID_AUTONOMOUS";
    mission_result = Run_Mission(o,struct('verbose',false));
    p = mission_result.proximity; cfg=mission_result.config;
    sys=cfg.proximity_system; settings=cfg.phase2;
    legacy_settings=settings; legacy_settings.mode="LEGACY_IMPULSIVE";
    evalc('[~,~,legacy]=mission.proximity(sys,mission_result.phasing.chaser,mission_result.phasing.target,legacy_settings);');
    assert(p.reached_standoff && legacy.reached_standoff);
    report = struct();
    report.scope = "Hybrid autonomous 3-DOF research prototype; perfect navigation and ideal vector force";
    report.config = cfg.run.phase2;
    report.candidates = rmfield(p.plan.candidates,'departure_lvlh');
    report.selected_closing_s = p.plan.duration_s;
    [~,arrival_v] = orbit_core.relative_state(mission_result.phasing.chaser,mission_result.phasing.target);
    handoff = norm(arrival_v);
    report.comparison = table(["Legacy cycloid/hops";"Hybrid R-bar"], ...
        [legacy.duration;p.duration]/60,[legacy.delta_v;p.delta_v], ...
        [legacy.delta_v-handoff;p.delta_v-p.handoff.delta_v], ...
        [legacy.final_position_error;p.final_position_error], ...
        [norm(legacy.final_relative_velocity);norm(p.final_relative_velocity)], ...
        'VariableNames',{'mode','duration_min','total_delta_v_m_s','post_handoff_delta_v_m_s', ...
        'terminal_error_m','terminal_speed_m_s'});
    report.handoff = p.handoff;
    report.control = rmfield(p.control,{'states','time','relative','velocity','reference','force_lvlh','mode'});
    initial = p.control.states(:,1);
    refined=settings; refined.dt=settings.dt/2;
    refined.autonomous.control_period_s=settings.autonomous.control_period_s/2;
    [fine_y,fine] = mission.track_rbar(initial,sys,refined);
    report.convergence = struct('position_difference_m',norm(fine_y(1:3)-p.control.states(1:3,end)), ...
        'delta_v_difference_m_s',abs(fine.delta_v_m_s-p.control.delta_v_m_s));
    assert(report.convergence.position_difference_m<0.02);
    assert(report.convergence.delta_v_difference_m_s<0.02);
    assert(p.control.max_force_N<=settings.autonomous.max_force_N);
    assert(p.control.max_speed_m_s<=settings.autonomous.max_approach_speed_m_s);
    assert(all(diff(p.history.time)>0));
    assert(abs(p.delta_v-sys.Isp*sys.g0*log(mission_result.phasing.chaser(14)/p.chaser(14)))<1e-7);

    % Initial delivery dispersion only: isotropic 1 m and 0.01 m/s, seed fixed.
    trials=10; errors=NaN(trials,1); speeds=errors; passed=false(trials,1);
    initial_errors=zeros(6,trials); peak_errors=errors;
    reasons=strings(trials,1);
    [~,~,C]=orbit_core.relative_state(initial(1:6),initial(7:12));
    omega=cross(initial(7:9),initial(10:12))/norm(initial(7:9))^2;
    for k=1:trials
        y=initial; dr=C'*randn(3,1); dv=C'*(0.01*randn(3,1));
        initial_errors(:,k)=[C*dr;C*dv];
        y(1:3)=y(1:3)+dr; y(4:6)=y(4:6)+dv+cross(omega,dr);
        try
            [~,sample]=mission.track_rbar(y,sys,settings);
            errors(k)=sample.final_error_m; speeds(k)=sample.final_speed_m_s;
            peak_errors(k)=sample.max_tracking_error_m;
            passed(k)=errors(k)<=settings.capture_pos_tol && speeds(k)<=settings.autonomous.capture_speed_m_s;
            reasons(k)="completed";
        catch exception
            if ~startsWith(string(exception.identifier),"mission:Proximity")
                rethrow(exception);
            end
            reasons(k)=string(exception.identifier);
        end
    end
    report.dispersion = struct('seed',1809,'trials',trials,'position_sigma_m',1, ...
        'velocity_sigma_m_s',0.01,'passed',sum(passed),'terminal_errors_m',errors, ...
        'terminal_speeds_m_s',speeds,'termination_reasons',reasons, ...
        'initial_errors_lvlh',initial_errors,'peak_tracking_errors_m',peak_errors);
    assert(all(passed),'Initial-delivery dispersion pilot failed.');

    % Safety monitors must reject off-corridor and insufficient-thrust cases.
    bad=initial; bad(1:3)=bad(1:3)+C'*[0;200;0];
    expect_failure(@() mission.track_rbar(bad,sys,settings),'mission:ProximityConstraint');
    weak=settings; weak.autonomous.max_force_N=0.01;
    expect_failure(@() mission.track_rbar(initial,sys,weak),'mission:ProximityConstraint');
    report.failure_tests = "off-corridor and 0.01 N authority rejected";

    % Sampled 10-minute zero-thrust continuations: a diagnostic, not a guarantee.
    indices=unique(round(linspace(1,size(p.control.states,2),9)));
    drift_min=zeros(size(indices));
    for k=1:numel(indices)
        y=p.control.states(:,indices(k));
        drift_min(k)=norm(p.control.relative(:,indices(k)));
        for j=1:120
            y=mission.proximity_step(y,zeros(3,1),5,1,sys);
            rr=orbit_core.relative_state(y(1:6),y(7:12));
            drift_min(k)=min(drift_min(k),norm(rr));
        end
    end
    report.drift_diagnostic=struct('horizon_s',600,'sample_interval_s',5, ...
        'departure_times_s',p.control.time(indices),'minimum_ranges_m',drift_min, ...
        'all_above_research_exclusion_radius',all(drift_min>=settings.autonomous.keep_out_radius_m));
    report.full_mission_completed = mission_result.entry.summary.completed_nominally;
    report.passed = true;
    file=fopen(fullfile(output_dir,'study_results.json'),'w');
    if file<0, error('mission:OutputFile','Cannot open study output.'); end
    file_cleanup=onCleanup(@() fclose(file));
    printable=report; printable.comparison=table2struct(report.comparison);
    fprintf(file,'%s',jsonencode(printable,PrettyPrint=true));
    clear file_cleanup;
    plot_comparison(p,legacy,output_dir);
    disp(report.comparison);
    fprintf('Proximity study: PASS; dispersion %d/%d; step position difference %.6g m\n', ...
        sum(passed),trials,report.convergence.position_difference_m);
end

function expect_failure(action, identifier)
    try
        action();
    catch exception
        assert(strcmp(exception.identifier,identifier),'Unexpected failure: %s',exception.message);
        return;
    end
    error('mission:MissingFailure','Expected failure %s was not raised.',identifier);
end

function plot_comparison(p,legacy,output_dir)
    fig=figure('Visible','off','Color','w','Position',[50 50 1150 800]);
    cleanup=onCleanup(@() close(fig));
    tiledlayout(2,2,'TileSpacing','compact');
    nexttile;
    plot(legacy.relative_position(2,:),legacy.relative_position(1,:),'Color',[.55 .55 .55]); hold on;
    plot(p.relative_position(2,:),p.relative_position(1,:),'b','LineWidth',1.3);
    plot(0,0,'r+'); grid on; xlabel('V-bar (m)'); ylabel('R-bar (m)');
    legend('Legacy','Hybrid','Target','Location','best'); title('Closing: same Phase 1 handoff');
    nexttile;
    plot(p.control.time/60,-p.control.relative(1,:),'b'); hold on;
    plot(p.control.time/60,-p.control.reference(1,:),'k--'); grid on;
    xlabel('Time since R-bar acquisition (min)'); ylabel('Distance below target (m)');
    title('Finite-force approach and active holds');
    nexttile;
    plot(p.control.time/60,vecnorm(p.control.velocity),'b'); hold on;
    yline(p.config.autonomous.max_approach_speed_m_s,'r--'); grid on;
    xlabel('Time since R-bar acquisition (min)'); ylabel('Relative speed (m/s)');
    title('Approach speed');
    nexttile;
    stairs(p.control.time(1:end-1)/60,vecnorm(p.control.force_lvlh(:,1:end-1)),'b'); grid on;
    xlabel('Time since R-bar acquisition (min)'); ylabel('Force (N)');
    title('Ideal vector actuator command');
    exportgraphics(fig,fullfile(output_dir,'proximity_comparison.png'),'Resolution',160);
end
