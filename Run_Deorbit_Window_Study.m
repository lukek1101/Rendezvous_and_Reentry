function result = Run_Deorbit_Window_Study(mission_result, overrides)
%RUN_DEORBIT_WINDOW_STUDY Search ignition opportunities for a geographic target.
% w = Run_Deorbit_Window_Study(); % default target [36.5 130.5], 20 km endpoint
% w = Run_Deorbit_Window_Study(mission_result, struct('target_latlon_deg',[lat lon]));
% Uses the real Phase 2 terminal state and resolved deorbit configuration.
% Waiting is free coast, not docked station keeping. Results are sampled witnesses.
    if nargin<1, mission_result=[]; end
    if nargin<2, overrides=struct(); end
    root=fileparts(mfilename('fullpath'));
    entry_defaults=entry_design.defaults();
    entry_defaults.max_step_s=20;
    entry_defaults.target_max_evaluations=24;
    defaults=struct('target_latlon_deg',[36.5 130.5], ...
        'delays_s',0:600:86400,'refinement_step_s',10,'refinement_basins',2, ...
        'time_refinement_evaluations',12, ...
        'vehicle_mode',"",'mission_epoch_utc',[], ...
        'entry',entry_defaults,'output_dir',fullfile(root,'output','deorbit_window'));
    options=mission.merge_settings(defaults,overrides);
    validateattributes(options.target_latlon_deg,{'numeric'},{'real','finite','vector','numel',2});
    if abs(options.target_latlon_deg(1))>90 || abs(options.target_latlon_deg(2))>180
        error('entry_design:Target','Use latitude [-90,90] and longitude [-180,180] degrees.');
    end
    validateattributes(options.delays_s,{'numeric'},{'real','finite','vector','nonempty','nonnegative'});
    options.delays_s=options.delays_s(:)';
    options.target_latlon_deg=options.target_latlon_deg(:)';
    if any(diff(options.delays_s)<=0), error('entry_design:Delays','Delays must increase.'); end
    validateattributes(options.refinement_step_s,{'numeric'},{'scalar','finite','positive'});
    validateattributes(options.refinement_basins,{'numeric'},{'scalar','integer','nonnegative','finite'});
    validateattributes(options.time_refinement_evaluations,{'numeric'},{'scalar','integer','positive','finite'});
    if ~isempty(options.mission_epoch_utc)
        angle=entry_design.earth_angle(options.mission_epoch_utc);
    end
    if isempty(mission_result)
        fprintf('Running the configured mission through Phase 2...\n');
        mission_result=Run_Mission(struct(),struct('stop_after_proximity',true));
    end
    if ~mission_result.proximity.reached_standoff
        error('mission:StandoffNotReached','Window study requires successful Phase 2.');
    end
    sys=mission_result.config.system;
    start_s=mission_result.phasing.history.time(end)+mission_result.proximity.duration;
    mode=upper(string(options.vehicle_mode));
    if strlength(mode)==0, mode=string(sys.reentry_vehicle.vehicle_mode); end
    vehicle=entry_design.vehicle(sys,mode);
    c=options.entry;
    % Keep a single time origin: mission t=0, including all preceding phases.
    c.entry_epoch_s=start_s;
    if isempty(options.mission_epoch_utc)
        c.earth_angle_at_epoch_deg=sys.reentry_vehicle.spaceplane.communication.earth_fixed_to_eci_angle_at_mission_epoch_deg;
        epoch_description="Mission_Config Greenwich angle; no calendar UTC assigned";
    else
        c.earth_angle_at_epoch_deg=angle;
        epoch_description="User-supplied mission t=0 UTC; approximate GMST";
    end
    fprintf('Target %.6f N, %.6f E; %s; endpoint %.1f km, tolerance %.1f km\n', ...
        options.target_latlon_deg,mode,c.terminal_altitude_m/1000,c.target_tolerance_m/1000);
    fprintf('Coarse scan: %d samples. %s\n',numel(options.delays_s),epoch_description);
    % A cheap zero-bank screen locates promising orbital passes. Only the
    % original-accuracy, full bank search can certify a candidate below.
    screen=c; screen.bank_profiles_deg=zeros(1,numel(c.speed_fractions));
    screen.max_step_s=max(c.max_step_s,30);
    screen.relative_tolerance=max(c.relative_tolerance,1e-6);
    coarse=scan(options.delays_s,false,screen);
    costs=[coarse.samples.miss_distance_m];
    minima=find(isfinite(costs) & costs<=[inf costs(1:end-1)] & costs<=[costs(2:end) inf]);
    [~,order]=sort(costs(minima));
    minima=minima(order(1:min(numel(order),options.refinement_basins)));
    fine_delays=[];
    for j=minima
        low=options.delays_s(max(1,j-1)); high=options.delays_s(min(numel(costs),j+1));
        center=options.delays_s(j);
        if high>low
            center=fminbnd(@screen_cost,low,high,optimset('Display','off', ...
                'TolX',min(1,options.refinement_step_s/10), ...
                'MaxFunEvals',options.time_refinement_evaluations));
        end
        neighbors=center+(-3:3)*options.refinement_step_s;
        fine_delays=[fine_delays neighbors(neighbors>=low & neighbors<=high) center]; %#ok<AGROW>
    end
    fine_delays=[fine_delays options.delays_s(costs<=c.target_tolerance_m)];
    if options.refinement_basins==0
        % Explicit grid-only mode still checks every requested time accurately.
        fine_delays=options.delays_s;
    end
    fine_delays=unique(fine_delays);
    result=coarse;
    result.entry_config=c;
    for j=1:numel(result.samples)
        if result.samples(j).status~="DEORBIT_FAILED", result.samples(j).status="SCREEN_ONLY"; end
    end
    if ~isempty(fine_delays)
        fprintf('Refined scan: %d times around %d local minima; bank optimization enabled.\n',numel(fine_delays),numel(minima));
        fine=scan(fine_delays,true,c);
        % Full-accuracy results replace the screen where grids overlap.
        for j=1:numel(fine.samples)
            index=find([result.samples.delay_s]==fine.samples(j).delay_s,1);
            if isempty(index)
                result.samples(end+1)=fine.samples(j); result.witnesses{end+1}=fine.witnesses{j};
            else
                result.samples(index)=fine.samples(j); result.witnesses{index}=fine.witnesses{j};
            end
        end
    end
    [~,order]=sort([result.samples.delay_s]);
    result.samples=result.samples(order); result.witnesses=result.witnesses(order);
    rows=result.samples; n=numel(rows);
    first_burn=nan(n,1); endpoint=nan(n,2);
    for j=1:n
        if ~isempty(rows(j).ignition_times_s), first_burn(j)=rows(j).ignition_times_s(1); end
        if ~isempty(result.witnesses{j}), endpoint(j,:)=result.witnesses{j}.trajectory.latlon_deg; end
    end
    result.table=table([rows.delay_s]',start_s+[rows.delay_s]',first_burn, ...
        [rows.entry_epoch_s]',[rows.miss_distance_m]',[rows.status]',endpoint(:,1),endpoint(:,2), ...
        'VariableNames',{'Delay_s','ScheduleStartMission_s','FirstBurnMission_s','EntryMission_s', ...
        'Miss_m','Status','EndpointLatitude_deg','EndpointLongitude_deg'});
    if ~isempty(options.mission_epoch_utc)
        epoch=options.mission_epoch_utc; epoch.TimeZone='UTC';
        epoch.Format='yyyy-MM-dd HH:mm:ss.SSS';
        result.table.FirstBurnUTC=epoch+seconds(first_burn);
    end
    verified=result.table.Status=="REACHABLE_WITNESS";
    result.candidates=result.table(verified,:);
    result.verified_delays_s=result.table.Delay_s(verified)';
    result.vehicle=vehicle; result.system=sys; result.options=options;
    result.coarse_samples=coarse.samples;
    result.initial_chaser=mission_result.proximity.chaser;
    result.initial_target=mission_result.proximity.target;
    result.scan_start_mission_s=start_s; result.epoch_description=epoch_description;
    result.endpoint_definition="Altitude surface on spherical Earth; not touchdown";
    if c.terminal_altitude_m==0
        result.endpoint_definition="Ground intersection surrogate; no parachute or landing dynamics";
    end
    result.search_limitations="Fixed deorbit law; heuristic time refinement and local bank optimization; unresolved is not unreachable";
    directory=options.output_dir;
    if ~isfolder(directory), mkdir(directory); end
    save(fullfile(directory,'window_results.mat'),'result','-v7.3');
    writetable(result.table,fullfile(directory,'all_samples.csv'));
    writetable(result.candidates,fullfile(directory,'candidates.csv'));
    fig=figure('Visible','off','Color','w','Position',[100 100 1100 450]);
    cleanup=onCleanup(@() close(fig)); tiledlayout(1,2); nexttile;
    plot(result.table.Delay_s/3600,result.table.Miss_m/1000,'.'); hold on;
    scatter(result.table.Delay_s(verified)/3600,result.table.Miss_m(verified)/1000,30,'filled');
    yline(c.target_tolerance_m/1000,'--','Tolerance');
    xlabel('Delay after Phase 2 (h)'); ylabel('Best propagated miss distance (km)'); grid on;
    title('Start time search');
    ax=gca; ax.YAxis.Exponent=0;
    nexttile;
    scatter(endpoint(:,2),endpoint(:,1),12,'.'); hold on;
    scatter(endpoint(verified,2),endpoint(verified,1),30,'filled');
    plot(options.target_latlon_deg(2),options.target_latlon_deg(1),'rp','MarkerSize',13,'MarkerFaceColor','r');
    xlabel('Longitude (deg E)'); ylabel('Latitude (deg N)'); grid on;
    xlim([-180 180]); ylim([-90 90]); title('Propagated endpoints; red star = target');
    sgtitle(sprintf('%s: target %.2f N, %.2f E; endpoint %.1f km',mode,options.target_latlon_deg,c.terminal_altitude_m/1000));
    exportgraphics(fig,fullfile(directory,'window_search.png'),'Resolution',160);
    fprintf('Verified %d / %d sampled candidates. Saved to %s\n',sum(verified),n,directory);
    if any(verified), disp(result.candidates); else, fprintf('No witness found: widen/refine the search; this is not proof of impossibility.\n'); end
    function cost=screen_cost(delay)
        sample=scan(delay,false,screen);
        cost=sample.samples.miss_distance_m;
        if ~isfinite(cost), cost=pi*sys.Re; end
    end
    function out=scan(delays,refine,entry_config)
        out=entry_design.window(sys,mission_result.proximity.chaser,mission_result.proximity.target, ...
            vehicle,delays,options.target_latlon_deg,entry_config,mission_result.config.deorbit,refine,true);
    end
end
