function rows=benchmark()
% Paired disturbances use exactly the same initial/departure/arrival draws.
root=fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(root);
folder=fileparts(mfilename('fullpath')); diary(fullfile(folder,'benchmark.log'));
cleanup=onCleanup(@()diary('off')); rows=struct([]); trials=cell(2,4);
for k=1:4
    phase=[90 90 60 60]; seeds=[42 7 42 7];
    s=Mission_Config('ARD'); s.initial_phase_angle_deg=phase(k);
    r=Mission_Run_Config(s); r.python_config.mode="NONE"; r.phase1.mode="HOHMANN";
    cfg=mission.configure(s,r); sys=cfg.proximity_system;
    [rc,vc]=initial(sys.Re+sys.h_insert,0,sys);
    [rt,vt]=initial(sys.Re+sys.h_target,deg2rad(phase(k)),sys);
    Xc=[rc;vc;0;0;0;1;0;0;0;4800]; Xt=[rt;vt];
    p=cfg.phase1.nominal; goal=cfg.phase1.desired_rel_lvlh;
    plan=mission.plan_nominal(sys,Xc,Xt,goal,cfg.phase1.max_wait,p);
    c=mission.correction_defaults(); c.enabled=true; c.seed=seeds(k);
    for enabled=[false true]
        c.corrections_enabled=enabled;
        e=mission.execute_corrected_nominal(sys,Xc,Xt,goal,plan,p,c,inf);
        trials{1+enabled,k}=e;
        row=struct('phase_deg',phase(k),'seed',c.seed,'correction_enabled',enabled, ...
            'accepted',e.success,'status',e.execution_status,'position_error_m',e.position_error_m, ...
            'velocity_error_m_s',e.velocity_error_m_s,'correction_count',e.correction_count, ...
            'correction_delta_v_m_s',e.correction_delta_v_m_s,'total_phase1_delta_v_m_s',e.delta_v_m_s, ...
            'fuel_kg',e.fuel_kg,'elapsed_s',e.elapsed_s,'runtime_s',e.runtime_s);
        if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
        fprintf('phase%d seed%d correction%d: %s accepted%d; %.6g m / %.6g m/s; corrections%d, %.6g m/s\n', ...
            phase(k),c.seed,enabled,e.execution_status,e.success,e.position_error_m,e.velocity_error_m_s,e.correction_count,e.correction_delta_v_m_s);
    end
    assert(isequal(trials{1,k}.initial_error_eci_si,trials{2,k}.initial_error_eci_si));
    assert(isequal(trials{1,k}.gain_draws,trials{2,k}.gain_draws));
    save(fullfile(folder,sprintf('case_%d.mat',k)),'rows','trials','cfg');
    fid=fopen(fullfile(folder,'benchmark.json'),'w'); fprintf(fid,'%s\n',jsonencode(rows,PrettyPrint=true)); fclose(fid);
end
end
function [r,v]=initial(radius,u,s)
r=radius*[cos(u);0;sin(u)];
v=sqrt(s.mu/radius*(1-s.J2*(s.Re/radius)^2*(3*sin(u)^2-1)))*[-sin(u);0;cos(u)];
end
