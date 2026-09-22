function rows=benchmark()
% Matched actual upstream state, requirements, candidate grid and controller.
root=fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(root);
folder=fileparts(mfilename('fullpath')); diary(fullfile(folder,'benchmark.log'));
cleanup=onCleanup(@()diary('off'));
baseline=load(fullfile(root,'docs','nominal_stage_2026-09-21','nominal_api.mat'),'r');
s=baseline.r.config.proximity_system; p=baseline.r.config.phase2;
upstream=baseline.r.phasing; rows=struct([]); results=cell(1,4); modes=["-R" "+R" "+V" "-V"];
for k=1:4
    p.autonomous.approach_mode=modes(k); started=tic;
    [~,~,q]=mission.proximity(s,upstream.chaser,upstream.target,p);
    runtime=toc(started); results{k}=q;
    row=struct('mode',modes(k),'passed',q.reached_standoff,'delta_v_m_s',q.delta_v, ...
        'closing_delta_v_m_s',q.closing.delta_v,'control_delta_v_m_s',q.control.delta_v_m_s, ...
        'duration_s',q.duration,'closing_duration_s',q.plan.duration_s,'runtime_s',runtime, ...
        'peak_force_N',q.control.max_force_N,'peak_requested_force_N',q.control.max_requested_force_N, ...
        'saturation_time_s',q.control.saturation_time_s,'max_tracking_error_m',q.control.max_tracking_error_m, ...
        'final_position_error_m',q.final_position_error,'final_speed_m_s',q.control.final_speed_m_s, ...
        'constraint_violations',q.control.constraint_violations, ...
        'corridor_margin_m',q.control.min_corridor_margin_m,'keep_out_margin_m',q.control.min_keep_out_margin_m);
    if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
    assert(isequal(q.upstream.chaser,upstream.chaser) && isequal(q.upstream.target,upstream.target));
    if k==1
        old=baseline.r.proximity;
        assert(abs(q.delta_v-old.delta_v)<1e-9);
        assert(max(abs(q.control.states-old.control.states),[],'all')<1e-8);
    end
    fprintf('%s: %.6f m/s, %.1f s, %.6f N, saturation %.1f s, tracking %.6f m, accepted%d\n', ...
        modes(k),q.delta_v,q.duration,q.control.max_force_N,q.control.saturation_time_s,q.control.max_tracking_error_m,q.reached_standoff);
end
save(fullfile(folder,'benchmark.mat'),'rows','results','upstream','s','p');
fid=fopen(fullfile(folder,'benchmark.json'),'w'); fprintf(fid,'%s\n',jsonencode(rows,PrettyPrint=true)); fclose(fid);
end
