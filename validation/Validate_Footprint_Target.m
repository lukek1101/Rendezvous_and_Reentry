function result = Validate_Footprint_Target()
%VALIDATE_FOOTPRINT_TARGET Reach a target absent from the initial bank library.
    sys=Mission_Config(); sys.environment.atmospheric_drag.enabled=false;
    radius=sys.Re+500e3;
    target=[radius;0;0;0;0;sqrt(sys.mu/radius)];
    initial=[target;0;0;0;1;0;0;0;2000];
    [entry,~,~,~,history]=mission.apogee_deorbit(sys,initial,target,mission.apogee_burn_defaults());
    c=entry_design.defaults(); c.entry_epoch_s=history.time(end);
    c.bank_profiles_deg=[-60 -60 -60;0 0 0;60 60 60];
    c.target_max_evaluations=24; c.target_tolerance_m=100;
    vehicle=entry_design.vehicle(sys,"CAPSULE");
    known=entry_design.propagate(sys,vehicle,entry(1:6),[-45 -45 -45],c);
    answer=entry_design.target(sys,vehicle,entry(1:6),known.latlon_deg,c,true);
    assert(answer.status=="REACHABLE_WITNESS");
    assert(answer.miss_distance_m<=c.target_tolerance_m);
    result=struct('status',answer.status,'miss_distance_m',answer.miss_distance_m, ...
        'tolerance_m',c.target_tolerance_m,'bank_profile_deg',answer.trajectory.bank_profile_deg, ...
        'refinement_evaluations',answer.refinement_evaluations,'passed',true);
    root=fileparts(fileparts(mfilename('fullpath')));
    directory=fullfile(root,'output','footprint');
    if ~isfolder(directory), mkdir(directory); end
    file=fopen(fullfile(directory,'target_refinement.json'),'w');
    if file<0, error('entry_design:Output','Cannot write target validation.'); end
    cleanup=onCleanup(@() fclose(file));
    fprintf(file,'%s',jsonencode(result,PrettyPrint=true));
    fprintf('Off-grid target refinement: PASS, %.3f m, %d propagations\n', ...
        answer.miss_distance_m,answer.refinement_evaluations);
end
