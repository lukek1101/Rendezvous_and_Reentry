function result = window(sys,Xc,Xt,vehicle,delays_s,target_latlon_deg,entry_overrides,deorbit_params,refine,verbose)
%WINDOW Propagate actual wait + deorbit + entry at each candidate ignition delay.
% Grid samples only: no interpolation across holes, no continuous window claim.
    if nargin<9, refine=false; end
    if nargin<10, verbose=false; end
    Xc=Xc(:); Xt=Xt(:);
    validateattributes(delays_s,{'numeric'},{'vector','nonempty','real','finite','nonnegative'});
    if any(diff(delays_s)<=0), error('entry_design:Delays','Ignition delays must strictly increase.'); end
    c=mission.merge_settings(entry_design.defaults(),entry_overrides);
    sys.environment.atmospheric_drag.earth_rotation_rad_s=c.earth_rotation_rad_s;
    if isfield(vehicle,'deorbit_mass_policy') && ...
            (~isscalar(string(vehicle.deorbit_mass_policy)) || ...
             ~any(string(vehicle.deorbit_mass_policy)==["REMAINING_STACK" "FIXED_SEPARATED"]))
        error('entry_design:MassPolicy','Unknown deorbit mass policy.');
    end
    rows=struct('delay_s',{},'entry_epoch_s',{},'status',{},'miss_distance_m',{}, ...
        'ignition_times_s',{},'deorbit_delta_v_m_s',{},'entry_vehicle_mass_kg',{},'bank_profile_deg',{},'reason',{});
    witnesses=cell(numel(delays_s),1);
    initial=[Xc(1:6);Xt(1:6);Xc(14)];
    y=initial; previous_delay=0;
    for k=1:numel(delays_s)
        delay=delays_s(k);
        if delay>previous_delay
            [~,states]=ode45(@(t,s) mission.proximity_dynamics(t,s,zeros(3,1),sys), ...
                [previous_delay delay],y,odeset('RelTol',1e-10,'AbsTol',1e-8,'MaxStep',30));
            y=states(end,:)';
        end
        previous_delay=delay;
        chaser=Xc; chaser(1:6)=y(1:6); chaser(14)=y(13); target=y(7:12);
        row=struct('delay_s',delay,'entry_epoch_s',NaN,'status',"DEORBIT_FAILED", ...
            'miss_distance_m',inf,'ignition_times_s',[],'deorbit_delta_v_m_s',NaN, ...
            'entry_vehicle_mass_kg',NaN,'bank_profile_deg',[],'reason',"");
        try
            if verbose
                evalc('[chaser,~,dv,~,history,info]=mission.deorbit(sys,chaser,target,deorbit_params);');
            else
                [chaser,~,dv,~,history,info]=mission.deorbit(sys,chaser,target,deorbit_params);
            end
            [entry,interface]=mission.entry_interface(history,chaser,sys,sys.h_entry_interface);
            row.entry_epoch_s=c.entry_epoch_s+delay+interface.time_s;
            row.deorbit_delta_v_m_s=dv;
            row.ignition_times_s=c.entry_epoch_s+delay;
            if isfield(info,'burns'), row.ignition_times_s=c.entry_epoch_s+delay+[info.burns.start_s]; end
            entry_vehicle=vehicle;
            if isfield(vehicle,'deorbit_mass_policy') && vehicle.deorbit_mass_policy=="REMAINING_STACK"
                entry_vehicle.mass_kg=entry(14);
            end
            if entry_vehicle.mass_kg>entry(14)
                error('entry_design:Mass','Entry vehicle mass exceeds remaining stack mass.');
            end
            row.entry_vehicle_mass_kg=entry_vehicle.mass_kg;
            trial=c; trial.entry_epoch_s=row.entry_epoch_s;
            candidate=entry_design.target(sys,entry_vehicle,entry(1:6),target_latlon_deg,trial,refine);
            row.status=candidate.status; row.miss_distance_m=candidate.miss_distance_m;
            row.bank_profile_deg=candidate.trajectory.bank_profile_deg;
            witnesses{k}=candidate;
        catch exception
            expected=["mission:EarlyEntry" "mission:NoEntry" "mission:NoApogee" ...
                "mission:DeorbitTimeout" "mission:BurnLimit" "mission:FuelLimit"];
            if ~any(string(exception.identifier)==expected), rethrow(exception); end
            row.reason=string(exception.message);
        end
        rows(k)=row;
        if verbose
            fprintf('  [%d/%d] delay %.1f min: %s, miss %.3f km\n', ...
                k,numel(delays_s),delay/60,row.status,row.miss_distance_m/1000);
        end
    end
    result=struct('samples',rows,'witnesses',{witnesses},'target_latlon_deg',target_latlon_deg, ...
        'entry_config',c,'deorbit_config',deorbit_params, ...
        'interpretation',"SAMPLED_IGNITION_OPPORTUNITIES_NOT_CONTINUOUS_WINDOWS");
    result.verified_delays_s=[rows([rows.status]=="REACHABLE_WITNESS").delay_s];
end
