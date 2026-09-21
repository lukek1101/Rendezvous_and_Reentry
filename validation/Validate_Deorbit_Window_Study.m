function Validate_Deorbit_Window_Study()
%VALIDATE_DEORBIT_WINDOW_STUDY Check time origin, actual witnesses and exports.
    root=fileparts(fileparts(mfilename('fullpath'))); addpath(root);
    sys=Mission_Config(); sys.environment.atmospheric_drag.enabled=false;
    radius=sys.Re+500e3; Xt=[radius;0;0;0;0;sqrt(sys.mu/radius)];
    Xc=[Xt;0;0;0;1;0;0;0;2000];
    a=mission.apogee_burn_defaults(); a.enabled=true;
    params=struct('apogee_burns',a);
    c=entry_design.defaults(); c.bank_profiles_deg=[0 0 0];
    vehicle=entry_design.vehicle(sys,"CAPSULE");
    probe=entry_design.window(sys,Xc,Xt,vehicle,0,[36.5 130.5],c,params,false);
    goal=probe.witnesses{1}.trajectory.latlon_deg;
    m=struct('config',struct('system',sys,'deorbit',params), ...
        'phasing',struct('history',struct('time',0)), ...
        'proximity',struct('chaser',Xc,'target',Xt,'duration',0,'reached_standoff',true));
    o=struct('delays_s',0,'refinement_basins',0,'vehicle_mode',"CAPSULE", ...
        'target_latlon_deg',goal,'entry',c, ...
        'output_dir',fullfile(root,'output','deorbit_window_validation'));
    w=Run_Deorbit_Window_Study(m,o);
    assert(height(w.candidates)==1 && w.candidates.Miss_m<1);
    assert(w.table.FirstBurnMission_s==0);
    assert(numel(w.samples.ignition_times_s)==3);
    assert(isfile(fullfile(o.output_dir,'candidates.csv')));
    % Same ECEF trajectory with a shifted time origin and matching Greenwich angle.
    offset=1000; m.proximity.duration=offset;
    m.config.system.reentry_vehicle.spaceplane.communication.earth_fixed_to_eci_angle_at_mission_epoch_deg= ...
        -rad2deg(c.earth_rotation_rad_s*offset);
    shifted=Run_Deorbit_Window_Study(m,o);
    assert(shifted.candidates.Miss_m<1);
    assert(shifted.table.FirstBurnMission_s==offset);
    assert(abs(shifted.table.EntryMission_s-w.table.EntryMission_s-offset)<1e-5);
    % UTC conversion is explicitly tied to mission t=0, not the scan start.
    epoch=datetime(2026,9,21,0,0,0,'TimeZone','UTC');
    angle=entry_design.earth_angle(epoch);
    o.mission_epoch_utc=epoch;
    o.target_latlon_deg=[goal(1),mod(goal(2)-angle-rad2deg(c.earth_rotation_rad_s*offset)+180,360)-180];
    utc=Run_Deorbit_Window_Study(m,o);
    assert(utc.candidates.Miss_m<1);
    assert(utc.table.FirstBurnUTC==epoch+seconds(offset));
    % Negative result remains unresolved, including after local time/bank refinement.
    o.mission_epoch_utc=[]; o.target_latlon_deg=[-goal(1) mod(goal(2)+360,360)-180];
    o.refinement_basins=1; o.entry.target_max_evaluations=4;
    rejected=Run_Deorbit_Window_Study(m,o);
    assert(isempty(rejected.candidates));
    assert(rejected.table.Status=="UNRESOLVED_BY_SEARCH");
    fprintf('Deorbit window study: PASS (witness, epoch invariance, UTC, unresolved, exports)\n');
end
