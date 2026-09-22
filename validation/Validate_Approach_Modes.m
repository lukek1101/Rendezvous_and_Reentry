function result=Validate_Approach_Modes()
% Matched upstream data and baseline are independently frozen before this stage.
root=fileparts(fileparts(mfilename('fullpath')));
data=load(fullfile(root,'docs','approach_stage_2026-09-21','benchmark.mat'));
assert(all([data.rows.passed]));
modes=["-R" "+R" "+V" "-V"];
for k=1:4
    p=data.p; p.autonomous.approach_mode=modes(k);
    [~,~,q]=mission.proximity(data.s,data.upstream.chaser,data.upstream.target,p);
    assert(abs(q.delta_v-data.results{k}.delta_v)<1e-9);
    assert(max(abs(q.control.states-data.results{k}.control.states),[],'all')<1e-8);
    u=mission.approach_axis(modes(k));
    assert(isequal(q.config.S3,u*500) && isequal(q.config.S4,u*30));
    assert(norm(q.control.reference(:,end)-u*30)<1e-12);
    assert(q.control.min_corridor_margin_m>=0 && q.control.min_keep_out_margin_m>=0);
    assert(q.control.max_speed_m_s<=q.config.autonomous.max_approach_speed_m_s);
    assert(q.control.saturation_time_s==0 && ~any(structfun(@(x)x,q.control.constraint_violations)));
    assert(abs(q.delta_v-data.s.Isp*data.s.g0*log(data.upstream.chaser(14)/q.mass(end)))<1e-9);
end
% R and V have different physical feedforward: observed force histories must
% not be mere axis-permutations, even with identical ranges and timing.
assert(data.rows(1).peak_force_N>4*data.rows(4).peak_force_N);
% Force-limited counterexample: retain actual trajectory and violation flags.
q=data.results{1}; p=q.config; p.autonomous.max_force_N=.01;
initial=q.control.states(:,1);
[~,failure]=mission.track_rbar(initial,data.s,p);
assert(~failure.completed && failure.saturation_time_s>0);
assert(any(structfun(@(x)x,failure.constraint_violations)));
assert(failure.max_force_N<=.01*(1+1e-12));
% Invalid side must be diagnosed on the actual state, without state reflection.
p=q.config; p.autonomous.approach_mode="+R";
[last,failure]=mission.track_rbar(initial,data.s,p);
assert(~failure.completed && failure.constraint_violations.side && isequal(last,initial));
s=Mission_Config('ARD'); r=Mission_Run_Config(s); r.python_config.mode="NONE";
r.phase1.mode="HOHMANN"; r.phase2.terminal_standoff_m=60;
r.phase2.autonomous.approach_mode="+V";
cfg=mission.configure(s,r); assert(cfg.phase2.S4_R_abs==60);
result=struct('passed',true,'modes',modes,'negative_R_frozen_baseline',true, ...
    'saturation_counterexample',true,'wrong_side_counterexample',true);
fprintf('Signed-axis proximity: PASS (four modes, frozen -R, saturation and wrong-side failures)\n');
end
