function report = Validate_Entry_Design()
%VALIDATE_ENTRY_DESIGN Physics/API checks, witness search and real burn-window scan.
    sys=Mission_Config(); sys.environment.atmospheric_drag.enabled=false;
    radius=sys.Re+500e3; target=[radius;0;0;0;0;sqrt(sys.mu/radius)];
    initial=[target;0;0;0;1;0;0;0;2000];
    schedule=mission.apogee_burn_defaults(); schedule.enabled=true;
    [entry,~,dv,fuel,history,info]=mission.apogee_deorbit(sys,initial,target,schedule);
    assert(numel(info.burns)==3);
    assert(abs(dv-schedule.total_delta_v_m_s)<1e-6);
    assert(abs(fuel-(initial(14)-entry(14)))<1e-7);
    assert(max(abs([info.burns(2:end).start_radial_speed_m_s]))<1e-5);
    assert(all([info.burns.duration_s]<=schedule.max_burn_duration_s+1e-6));
    gaps=[info.burns(2:end).start_s]-[info.burns(1:end-1).start_s]-[info.burns(1:end-1).duration_s];
    assert(all(gaps>=schedule.cooldown_s));
    assert(abs(norm(entry(1:3))-sys.Re-sys.h_entry_interface)<0.01);
    assert(all(diff(history.time)>0) && all(diff(history.mass)<=1e-7));
    report.apogee=info;
    % Restrictive duration splits a requested portion over more apogees.
    restricted=schedule; restricted.max_burn_duration_s=210;
    restricted.total_delta_v_m_s=80; restricted.max_burns=2;
    expect_failure(@() mission.apogee_deorbit(sys,initial,target,restricted),'mission:BurnLimit');
    report.burn_limit_checked=true;
    capped=schedule; capped.total_delta_v_m_s=120; capped.fractions=[0.4 0.6];
    capped.max_burn_duration_s=300;
    [~,~,capped_dv,~,~,capped_info]=mission.apogee_deorbit(sys,initial,target,capped);
    assert(numel(capped_info.burns)>numel(capped.fractions));
    assert(max([capped_info.burns.duration_s])<=300+1e-6);
    assert(abs(capped_dv-120)<1e-6);
    report.duration_capped_burns=capped_info.burns;

    c=entry_design.defaults(); c.entry_epoch_s=history.time(end);
    vehicle=entry_design.vehicle(sys,"CAPSULE");
    c.bank_profiles_deg=[-60 -60 -60;0 0 0;60 60 60];
    fp=entry_design.footprint(sys,vehicle,entry(1:6),c);
    assert(any(fp.feasible));
    selected=find(fp.feasible,1); witness=fp.trajectories{selected};
    assert(abs(witness.terminal_altitude_m-c.terminal_altitude_m)<0.01);
    assert(max(abs(diff(witness.state(:,8))./diff(witness.time_s)))<=c.bank_rate_deg_s+0.01);
    fine=c; fine.max_step_s=c.max_step_s/2; fine.relative_tolerance=c.relative_tolerance/10;
    refined=entry_design.propagate(sys,vehicle,entry(1:6),witness.bank_profile_deg,fine);
    report.endpoint_convergence_m=norm(refined.final_ecef_m-witness.final_ecef_m);
    assert(report.endpoint_convergence_m<100);
    strict=c; strict.max_dynamic_pressure_Pa=1;
    rejected=entry_design.propagate(sys,vehicle,entry(1:6),witness.bank_profile_deg,strict);
    assert(~rejected.feasible && rejected.terminal_reached);
    % Zero lift makes bank choice irrelevant, a physical rather than self-referential test.
    ballistic=vehicle; ballistic.shape.ld_scale=0;
    left=entry_design.propagate(sys,ballistic,entry(1:6),[-60 -60 -60],c);
    right=entry_design.propagate(sys,ballistic,entry(1:6),[60 60 60],c);
    report.zero_lift_bank_difference_m=norm(left.final_ecef_m-right.final_ecef_m);
    assert(report.zero_lift_bank_difference_m<1);
    known=entry_design.target(sys,vehicle,entry(1:6),witness.latlon_deg,c,false);
    assert(known.status=="REACHABLE_WITNESS" && known.miss_distance_m<1);
    far=entry_design.target(sys,vehicle,entry(1:6),[-witness.latlon_deg(1) witness.latlon_deg(2)+180],c,false);
    assert(far.status=="UNRESOLVED_BY_SEARCH");

    u=struct('trials',2,'seed',981,'initial_covariance',diag([10 10 10 .01 .01 .01].^2), ...
        'density_coefficient_of_variation',0.02);
    before=rng;
    spread=entry_design.dispersion(sys,vehicle,entry(1:6),witness.bank_profile_deg,c,u);
    assert(isequal(before,rng));
    repeat=entry_design.dispersion(sys,vehicle,entry(1:6),witness.bank_profile_deg,c,u);
    assert(isequal(spread.latlon_deg,repeat.latlon_deg));
    report.dispersion=spread;
    % Actual stack propagation and actual entry epoch, not a translated footprint.
    window_config=c; window_config.entry_epoch_s=0;
    window_config.bank_profiles_deg=witness.bank_profile_deg;
    params=struct('apogee_burns',schedule);
    opportunities=entry_design.window(sys,initial,target,vehicle,[0 120 240], ...
        witness.latlon_deg,window_config,params,false);
    assert(opportunities.samples(1).status=="REACHABLE_WITNESS");
    assert(opportunities.samples(1).miss_distance_m<1);
    assert(all([opportunities.samples.entry_epoch_s]>[opportunities.samples.delay_s]));
    report.window=rmfield(opportunities,'witnesses');
    plane=entry_design.vehicle(sys,"SPACEPLANE");
    plane_window=entry_design.window(sys,initial,target,plane,0, ...
        witness.latlon_deg,window_config,params,false);
    assert(abs(plane_window.samples.entry_vehicle_mass_kg-entry(14))<1e-6);
    assert(opportunities.samples(1).entry_vehicle_mass_kg==vehicle.mass_kg);
    report.mass_policy_checked=true;
    j2000=datetime(2000,1,1,12,0,0,'TimeZone','UTC');
    assert(abs(entry_design.earth_angle(j2000)-280.46061837)<1e-7);
    rotation_config=entry_design.defaults(); rotation_config.earth_angle_at_epoch_deg=90;
    rotated=entry_design.ecef([1;0;0],0,rotation_config);
    assert(norm(rotated-[0;-1;0])<1e-12);
    report.passed=true;
    root=fileparts(fileparts(mfilename('fullpath'))); directory=fullfile(root,'output','footprint');
    if ~isfolder(directory), mkdir(directory); end
    file=fopen(fullfile(directory,'validation_results.json'),'w');
    if file<0, error('entry_design:Output','Cannot write validation result.'); end
    cleanup=onCleanup(@() fclose(file));
    fprintf(file,'%s',jsonencode(report,PrettyPrint=true));
    fprintf('Entry design: PASS, %d apogee burns, endpoint refinement %.3f m\n',numel(info.burns),report.endpoint_convergence_m);
    disp(struct2table(opportunities.samples));
end

function expect_failure(action,identifier)
    try
        action();
    catch exception
        assert(strcmp(exception.identifier,identifier),'Unexpected error: %s',exception.message);
        return;
    end
    error('entry_design:MissingFailure','Expected error %s.',identifier);
end
