function rows=benchmark()
% Two representative phases; bounded old search, no new corrective manoeuvres.
root=fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(root);
folder=fileparts(mfilename('fullpath')); diary(fullfile(folder,'matlab_benchmark.log'));
cleanup=onCleanup(@()diary('off')); rows=struct([]);
for phase=[90 60]
    s=Mission_Config('ARD'); s.initial_phase_angle_deg=phase;
    r=Mission_Run_Config(s); r.python_config.mode="NONE"; r.phase1.mode="HOHMANN";
    cfg=mission.configure(s,r); sys=cfg.proximity_system;
    [rc,vc]=initial(sys.Re+sys.h_insert,0,sys); [rt,vt]=initial(sys.Re+sys.h_target,deg2rad(phase),sys);
    Xc=[rc;vc;0;0;0;1;0;0;0;4800]; Xt=[rt;vt];
    p=cfg.phase1;
    for method=["NOMINAL_TARGET","GRID_SEARCH"]
        p.hohmann_method=method; sys.capture.r_rel0=p.desired_rel_lvlh;
        if method=="GRID_SEARCH"
            p.max_wait=plan.wait_s*1.1; p.dt_scan=60;
            p.refine_span=180; p.refine_step=2;
        end
        clock=tic;
        [xc,dv,~,history,xt]=Phasing_Propagator(sys,Xc,sys.Re+sys.h_target,"HOHMANN",p,Xt,1);
        elapsed=toc(clock);
        if method=="NOMINAL_TARGET", plan=history.planning; end
        candidate=assess(method,xc,xt,dv,elapsed,history);
        if isempty(rows), rows=candidate; else, rows(end+1)=candidate; end %#ok<AGROW>
    end
    file=fullfile(folder,sprintf('python_%d.json',phase));
    if isfile(file)
        py=jsondecode(fileread(file));
        if isfield(py,'optimized_parameters')
            p=cfg.phase1; q=py.optimized_parameters;
            p.phase_angle=q.phase_angle_deg; p.phase_angle_unit="deg";
            p.delta_v=q.delta_v_1_m_s; p.gamma=q.gamma_deg; p.gamma_unit="deg";
            clock=tic;
            [xc,dv,~,history,xt]=Phasing_Propagator(sys,Xc,sys.Re+sys.h_target,"CUSTOM_IMPULSE",p,Xt,1);
            elapsed=toc(clock)+py.runtime_s;
            candidate=assess("PYTHON_BOUNDED_COMPARISON",xc,xt,dv,elapsed,history);
            candidate.optimizer_success=py.success; candidate.optimizer_message=string(py.message);
            rows(end+1)=candidate; %#ok<AGROW>
        end
    end
    % Explicit historical-parameter comparison, freshly propagated. This is
    % not an accepted optimizer artifact or a newly optimized candidate.
    archive=fullfile(root,'configs','python_runs','impulsive_drag-off_h300.0-500.0_phase90.0_31c1e45c_410e0069_20260630_134559Z.json');
    old=jsondecode(fileread(archive)); q=old.phase1; p=cfg.phase1;
    p.phase_angle=q.phase_angle_deg; p.phase_angle_unit="deg";
    p.delta_v=q.delta_v_m_s; p.gamma=q.gamma_deg; p.gamma_unit="deg";
    clock=tic;
    [xc,dv,~,history,xt]=Phasing_Propagator(sys,Xc,sys.Re+sys.h_target,"CUSTOM_IMPULSE",p,Xt,1);
    elapsed=toc(clock);
    candidate=assess("HISTORICAL_PARAMETER_REPLAY",xc,xt,dv,elapsed,history);
    candidate.optimizer_message="HISTORICAL_ITERATION_LIMIT; REPLAY_RUNTIME_ONLY; "+mission.file_sha256(archive);
    rows(end+1)=candidate; %#ok<AGROW>
    save(fullfile(folder,'benchmark.mat'),'rows','cfg');
    fid=fopen(fullfile(folder,'benchmark.json'),'w'); fprintf(fid,'%s\n',jsonencode(rows,PrettyPrint=true)); fclose(fid);
end
    function row=assess(method,xc,xt,dv,elapsed,history)
        [rr,vv]=orbit_core.relative_state(xc,xt);
        row=struct('phase_deg',phase,'method',method,'planning_execution_runtime_s',elapsed, ...
            'position_error_m',norm(rr-p.desired_rel_lvlh),'relative_velocity_m_s',vv, ...
            'position_lvlh_m',rr,'phase1_delta_v_m_s',dv,'phase1_mass_kg',xc(14), ...
            'handoff_correction_m_s',NaN,'phase2_delta_v_m_s',NaN,'total_through_phase2_m_s',NaN, ...
            'downstream_status',"NOT_RUN_HANDOFF_GATE_FAILED",'phase2_runtime_s',NaN, ...
            'optimizer_success',false,'optimizer_message',"NOT_REQUESTED");
        if row.position_error_m<=cfg.phase2.initial_S2_tol
            clock=tic; [~,~,down]=mission.proximity(sys,xc,xt,cfg.phase2);
            row.phase2_runtime_s=toc(clock);
            row.handoff_correction_m_s=down.handoff.delta_v;
            row.phase2_delta_v_m_s=down.delta_v;
            row.total_through_phase2_m_s=dv+down.delta_v;
            row.downstream_status="PROPAGATED_EXISTING_PHASE2";
        end
        if isfield(history,'planning')
            row.optimizer_message="NO_OPTIMIZATION_TARGETING_"+history.planning.numerical_status;
        end
        fprintf('%s phase%d: %.3fs, miss %.6gm, phase1 %.6f m/s, phase2 %.6f m/s\n', ...
            method,phase,elapsed,row.position_error_m,dv,row.phase2_delta_v_m_s);
    end
end
function [r,v]=initial(radius,u,s)
r=radius*[cos(u);0;sin(u)];
v=sqrt(s.mu/radius*(1-s.J2*(s.Re/radius)^2*(3*sin(u)^2-1)))*[-sin(u);0;cos(u)];
end
