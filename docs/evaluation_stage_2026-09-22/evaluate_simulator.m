function results=evaluate_simulator()
% Bounded evaluation only; does not change production physics or controllers.
folder=fileparts(mfilename('fullpath')); root=fileparts(fileparts(folder));
addpath(root,fullfile(root,'validation'));
diary(fullfile(folder,'evaluation.log')); cleanup=onCleanup(@()diary('off'));
caller_rng=rng; started=tic;
results.scope="CONDITIONAL_COMPONENT_AND_INTEGRATED_DESIGN_EVALUATION_NOT_RELIABILITY";
results.matlab_version=version;
results.design=struct([]); designs=cell(1,8);
presets=[repmat("APOLLO7_PREFLIGHT_TRIM",1,6),"ARD","HORUS_2B"];
phases=[60 90 120 90 90 90 90 90]; modes=["-R","-R","-R","+R","+V","-V","-R","-R"];
for k=1:numel(phases)
    row=struct('id',k,'preset',presets(k),'phase_deg',phases(k),'mode',modes(k), ...
        'classification',"NOT_RUN",'runtime_s',NaN,'phase1_dv_m_s',NaN,'phase2_dv_m_s',NaN, ...
        'deorbit_dv_m_s',NaN,'fuel_through_deorbit_kg',NaN,'duration_to_entry_s',NaN, ...
        'remaining_stack_mass_kg',NaN,'entry_mach',NaN,'peak_control_force_N',NaN, ...
        'saturation_s',NaN,'tracking_error_m',NaN,'handoff_error_m',NaN,'message',"");
    timer=tic;
    try
        overrides=struct('runtime',struct('allow_environment_overrides',false), ...
            'phase2',struct('autonomous',struct('approach_mode',modes(k))));
        n=Run_Nominal_Orbit(overrides,struct('verbose',false,'seed',22,'preset',presets(k), ...
            'system',struct('initial_phase_angle_deg',phases(k))));
        designs{k}=n; sys=n.config.system;
        [X,~,dv,fuel,hist]=mission.deorbit(sys,n.proximity.chaser,n.proximity.target,n.config.deorbit);
        [accepted,interface]=mission.entry_interface(hist,X,sys,sys.h_entry_interface);
        assert(isequal(accepted,X));
        interface.achieved=mission.state_record(X,sys, ...
            n.interfaces.phase2_to_phase3.time_since_mission_epoch_s+interface.time_s,"PROPAGATED_DEORBIT_TERMINAL");
        [~,~,~,sound]=Standard_Atmosphere_Density(interface.altitude_m,sys.environment.atmospheric_drag);
        row.phase1_dv_m_s=n.phasing.delta_v; row.phase2_dv_m_s=n.proximity.delta_v;
        row.deorbit_dv_m_s=dv; row.fuel_through_deorbit_kg=n.phasing.fuel+n.proximity.fuel+fuel;
        row.remaining_stack_mass_kg=X(14); row.duration_to_entry_s=interface.achieved.time_since_mission_epoch_s;
        row.entry_mach=interface.achieved.speed_air_relative_m_s/sound;
        row.peak_control_force_N=n.proximity.control.max_force_N;
        row.saturation_s=n.proximity.control.saturation_time_s;
        row.tracking_error_m=n.proximity.control.max_tracking_error_m;
        row.handoff_error_m=n.proximity.final_position_error;
        assert(abs(n.interfaces.initial.mass_kg-X(14)-row.fuel_through_deorbit_kg)<1e-7);
        v=entry_design.vehicle(sys,sys.reentry_vehicle.vehicle_mode);
        mass=X(14); if sys.reentry_vehicle.vehicle_mode=="CAPSULE", mass=v.mass_kg; end
        row.classification="ENTRY_HANDOFF_DOMAIN_SUPPORTED_NOT_DESCENT_PROVEN";
        try
            reentry_core.evaluate_state([X(1:6);mass],sys,v.shape,v.aoa_deg,0,true,v.heat_coefficient,0);
        catch e
            row.classification=classify_exception(e); row.message=string(e.message);
        end
        save(fullfile(folder,sprintf('design_%02d.mat',k)),'n','X','hist','interface');
    catch e
        row.classification=classify_exception(e); row.message=string(e.message);
    end
    row.runtime_s=toc(timer); results.design=append_row(results.design,row);
    fprintf('Design %d %s phase %g %s: %s\n',k,presets(k),phases(k),modes(k),row.classification);
end
% Freeze one nominal Apollo 90-degree design. No plan call inside robustness loops.
n=designs{2}; assert(~isempty(n)); sys=n.config.proximity_system;
plan=n.phasing.history.planning; frozen=plan;
ic=n.interfaces.initial; Xc=[ic.position_eci_m;ic.velocity_eci_m_s;0;0;0;1;0;0;0;ic.mass_kg];
radius=sys.Re+sys.h_target; u=deg2rad(sys.initial_phase_angle_deg+sys.initial_chaser_angle_deg);
Xt=[radius*[cos(u);0;sin(u)];sqrt(sys.mu/radius*(1-sys.J2*(sys.Re/radius)^2*(3*sin(u)^2-1)))*[-sin(u);0;cos(u)]];
p=n.config.phase1.nominal; goal=n.config.phase1.desired_rel_lvlh;
c0=mission.correction_defaults(); c0.enabled=true;
% Same propellant fraction as the prior 400 kg / 4800 kg reference allocation.
c0.max_propellant_kg=400*Xc(14)/4800;
save(fullfile(folder,'frozen_nominal.mat'),'plan','sys','Xc','Xt','p','goal','c0');
results.robustness=struct([]); trial=0;
for initial_scale=[0 1 3]
    for burn_scale=[0 1 3]
        for seed=[7 42]
            c=c0; c.seed=seed; c.position_sigma_m=c0.position_sigma_m*initial_scale;
            c.velocity_sigma_m_s=c0.velocity_sigma_m_s*initial_scale;
            c.burn_gain_sigma=c0.burn_gain_sigma*burn_scale;
            c.burn_pointing_sigma_rad=c0.burn_pointing_sigma_rad*burn_scale;
            pair=cell(1,2);
            for enabled=[false true]
                c.corrections_enabled=enabled;
                e=mission.execute_corrected_nominal(sys,Xc,Xt,goal,plan,p,c,inf);
                trial=trial+1; pair{1+enabled}=e;
                results.robustness=append_row(results.robustness,robust_row(e,initial_scale,burn_scale,trial));
                verify_execution(e,Xc,sys);
                save(fullfile(folder,sprintf('robust_%02d.mat',trial)),'e');
            end
            assert(isequal(pair{1}.initial_error_eci_si,pair{2}.initial_error_eci_si));
            assert(isequal(pair{1}.gain_draws,pair{2}.gain_draws));
            assert(isequal(pair{1}.pointing_draws_rad,pair{2}.pointing_draws_rad));
        end
    end
end
assert(isequaln(frozen,plan));
results.budget_controls=struct([]);
for budget=[400 c0.max_propellant_kg]
    c=c0; c.max_propellant_kg=budget; c.position_sigma_m=zeros(3,1); c.velocity_sigma_m_s=zeros(3,1);
    c.burn_gain_sigma=0; c.burn_pointing_sigma_rad=0;
    e=mission.execute_corrected_nominal(sys,Xc,Xt,goal,plan,p,c,inf);
    results.budget_controls=append_row(results.budget_controls,robust_row(e,0,0,0)); verify_execution(e,Xc,sys);
end
% Fixed acquisition state/reference and existing online final-approach controller.
% No closing trajectory planner is called in these component robustness cases.
results.proximity=struct([]); p2=n.config.phase2; acquisition=n.proximity.control.states(:,1);
[~,~,C]=orbit_core.relative_state(acquisition(1:6),acquisition(7:12));
for lateral=[0 1 5]
    for velocity=[0 .02]
        y=acquisition; y(1:3)=y(1:3)+C'*[0;0;lateral]; y(4:6)=y(4:6)+C'*[0;0;velocity];
        [~,control]=mission.track_rbar(y,sys,p2);
        row=struct('lateral_offset_m',lateral,'lateral_velocity_m_s',velocity, ...
            'completed',control.completed,'termination',control.termination, ...
            'dv_m_s',control.delta_v_m_s,'fuel_kg',y(13)-control.states(13,end), ...
            'peak_force_N',control.max_force_N,'saturation_s',control.saturation_time_s, ...
            'tracking_error_m',control.max_tracking_error_m,'terminal_error_m',control.final_error_m, ...
            'duration_s',control.time(end),'violations',control.constraint_violations);
        results.proximity=append_row(results.proximity,row);
        save(fullfile(folder,sprintf('proximity_%02d.mat',numel(results.proximity))),'control','y','p2');
    end
end
% Standalone fixed-bank sensitivity, explicitly disconnected from orbital handoff.
s=Mission_Config(); s.environment.atmospheric_drag.use_matlab_atmosisa=false;
v0=entry_design.vehicle(s,'CAPSULE'); settings=entry_design.defaults();
settings.max_time_s=1800; settings.max_step_s=2; settings.relative_tolerance=1e-9;
settings.terminal_altitude_m=7620; settings.initial_bank_deg=30;
cases=[];
for fpa=[-2.5 -2 -1.5]
    for density=[.9 1 1.1], cases=[cases;fpa density 1]; end %#ok<AGROW>
end
cases=[cases;-2 1 .9;-2 1 1.1];
results.entry=struct([]); baseline=[];
for k=1:size(cases,1)
    v=v0; v.shape.density_scale=cases(k,2); v.shape.ld_scale=cases(k,3);
    x=entry_design.initial_state(s,[25 40],121920,7400,cases(k,1),70,settings);
    row=struct('id',k,'fpa_deg',cases(k,1),'density_scale',cases(k,2),'ld_scale',cases(k,3), ...
        'classification',"NOT_RUN",'duration_s',NaN,'peak_q_Pa',NaN,'peak_g',NaN, ...
        'peak_heat_W_m2',NaN,'terminal_ecef_m',[NaN NaN NaN],'endpoint_shift_m',NaN,'message',"");
    try
        entry=entry_design.propagate(s,v,x,[30 30 30],settings);
        row.classification="ENDPOINT_REACHED_NO_PATH_OR_TARGET_ACCEPTANCE";
        if ~entry.terminal_reached, row.classification="BOUNDED_PROPAGATION_NO_ENDPOINT"; end
        row.duration_s=entry.time_s(end); row.peak_q_Pa=entry.max_dynamic_pressure_Pa;
        row.peak_g=entry.max_g_load; row.peak_heat_W_m2=entry.max_heat_flux_W_m2;
        row.terminal_ecef_m=entry.final_ecef_m(:)';
        if all(cases(k,:)==[-2 1 1]), baseline=entry; end
        save(fullfile(folder,sprintf('entry_%02d.mat',k)),'entry','x','v','settings');
    catch e
        row.classification=classify_exception(e); row.message=string(e.message);
    end
    results.entry=append_row(results.entry,row);
    fprintf('Entry %d FPA %g density %g LD %g: %s\n',k,cases(k,:),row.classification);
end
assert(~isempty(baseline));
for k=1:numel(results.entry)
    results.entry(k).endpoint_shift_m=norm(results.entry(k).terminal_ecef_m-baseline.final_ecef_m(:)');
end
% One precision check with identical physics, policy and initial condition.
tight=settings; tight.max_step_s=1; tight.relative_tolerance=1e-10;
x=entry_design.initial_state(s,[25 40],121920,7400,-2,70,tight);
refined=entry_design.propagate(s,v0,x,[30 30 30],tight);
results.precision=struct('entry_endpoint_difference_m',norm(refined.final_ecef_m-baseline.final_ecef_m), ...
    'entry_time_difference_s',abs(refined.time_s(end)-baseline.time_s(end)),'model_validation',false);
results.verification.orbit=Validate_Orbit_Core();
results.verification.profiles=Validate_Reference_Profiles();
results.verification.apollo=Validate_Apollo7();
assert(isequal(caller_rng,rng));
results.elapsed_s=toc(started); results.verification.fixed_plan_and_paired_draws=true;
save(fullfile(folder,'evaluation.mat'),'results','settings','cases');
% Omit the large validation trajectory from the readable summary; retain in MAT.
readable=results; readable.verification.apollo=rmfield(readable.verification.apollo,'nominal');
fid=fopen(fullfile(folder,'results.json'),'w'); fprintf(fid,'%s\n',jsonencode(readable,PrettyPrint=true)); fclose(fid);
writetable(struct2table(results.design),fullfile(folder,'design.csv'));
writetable(struct2table(results.robustness),fullfile(folder,'robustness.csv'));
writetable(struct2table(results.entry),fullfile(folder,'entry.csv'));
fprintf('Evaluation complete in %.1f s; frozen-plan, ledger and paired-draw assertions passed.\n',results.elapsed_s);
end

function row=robust_row(e,initial_scale,burn_scale,id)
classification="HANDOFF_TOLERANCE_VIOLATION";
if e.success, classification="CONDITIONAL_PHASE1_SUCCESS";
elseif e.execution_status=="NUMERICAL_WORK_LIMIT", classification="NUMERICAL_NONCOMPLETION";
elseif e.execution_status~="EXECUTED", classification="RESOURCE_OR_POLICY_LIMIT";
elseif ~isempty(e.opportunities) && any([e.opportunities.status]=="PREDICTED_CAPABILITY_LIMIT")
    classification="CORRECTION_CAPABILITY_REJECTED_WITH_OBSERVED_MISS";
elseif ~isempty(e.opportunities) && any(ismember([e.opportunities.status], ...
        ["ITERATION_LIMIT","EVALUATION_LIMIT","SINGULAR_JACOBIAN"]))
    classification="NUMERICAL_NONCONVERGENCE_WITH_OBSERVED_MISS";
end
row=struct('id',id,'initial_scale',initial_scale,'burn_scale',burn_scale,'seed',e.settings.seed, ...
    'corrections_enabled',e.settings.corrections_enabled,'accepted',e.success,'classification',classification, ...
    'execution_status',e.execution_status,'handoff_status',e.handoff_status, ...
    'position_error_m',e.position_error_m,'velocity_error_m_s',e.velocity_error_m_s, ...
    'delta_v_m_s',e.delta_v_m_s,'correction_delta_v_m_s',e.correction_delta_v_m_s,'fuel_kg',e.fuel_kg, ...
    'propellant_budget_kg',e.settings.max_propellant_kg,'correction_count',e.correction_count, ...
    'elapsed_s',e.elapsed_s,'runtime_s',e.runtime_s,'opportunity_statuses',"");
if ~isempty(e.opportunities), row.opportunity_statuses=join([e.opportunities.status],"|"); end
end

function verify_execution(e,Xc,s)
if ~isempty(e.burns)
    assert(abs(sum([e.burns.fuel_kg])-e.fuel_kg)<1e-7);
    assert(abs(sum([e.burns.delivered_delta_v_m_s])-e.delta_v_m_s)<1e-9);
    for b=e.burns'
        assert(isequal(b.state_before(1:3),b.state_after(1:3)));
        assert(abs(b.state_after(13)-b.state_before(13)*exp(-b.delivered_delta_v_m_s/(s.Isp*s.g0)))<1e-8);
    end
end
assert(norm(e.history.pos(:,1)-Xc(1:3)-e.initial_error_eci_si(1:3))<1e-8);
assert(abs(Xc(14)-e.chaser(14)-e.fuel_kg)<1e-8);
end

function c=classify_exception(e)
id=string(e.identifier);
if startsWith(id,"reference_vehicle:")
    c="MODEL_DOMAIN_UNSUPPORTED";
elseif contains(id,"Scope") || contains(id,"Unsupported")
    c="IMPLEMENTATION_UNSUPPORTED";
elseif contains(id,"Plan") || contains(id,"Search") || contains(id,"Target")
    c="PLANNING_NONCOMPLETION_NOT_INFEASIBILITY";
elseif contains(id,"Constraint") || contains(id,"Gate")
    c="OBSERVED_CONSTRAINT_VIOLATION";
else
    rethrow(e); % Programming faults must not become scientific results.
end
end

function rows=append_row(rows,row)
if isempty(rows), rows=row; else, rows(end+1)=row; end
end
