function validate_optimizer_compatibility(artifact,contract,run,preset)
%VALIDATE_OPTIMIZER_COMPATIBILITY No score-based choice can bypass this gate.
if isempty(fieldnames(artifact)), return; end
if isfield(artifact,'compatibility')
    if ~same(artifact.compatibility,jsondecode(jsonencode(contract)))
        error('mission:IncompatibleOptimizer','Python compatibility contract differs from resolved mission physics. Regenerate the design.');
    end
    if ~isfield(artifact,'optimizer') || ~isfield(artifact.optimizer,'success') || ~artifact.optimizer.success
        error('mission:UnconvergedOptimizer','Normal execution requires a successful, compatible optimizer result.');
    end
    return;
end
% Historical regression only; never accepted by AUTO or for a reference model.
if run.python_config.allow_legacy_replay && upper(string(run.python_config.mode))=="FILE" && ...
        preset=="LEGACY_CAPSULE_60KG"
    s=artifact.scenario;
    pinned_hash="0898fc109c2050eb33245723a68d83569093c14c53279d31911a97622005c82b";
    if ~isfield(artifact,'import_record') || artifact.import_record.sha256~=pinned_hash
        error('mission:UnpinnedLegacyArchive','Legacy replay accepts only the content-pinned audited impulsive archive.');
    end
    atmosphere=artifact.environment.atmospheric_drag;
    atmosphere=rmfield(atmosphere,'target_mass_kg');
    if contract.insertion_altitude_m~=s.h_chaser_km*1000 || ...
            contract.target_altitude_m~=s.h_target_km*1000 || ...
            contract.initial_chaser_angle_deg~=s.initial_chaser_angle_deg || ...
            contract.initial_phase_angle_deg~=s.initial_phase_angle_deg || ...
            contract.base_chaser_wet_mass_kg~=artifact.maneuver.initial_mass_kg || ...
            contract.burn_model~=string(artifact.phase1.burn_model) || ...
            contract.atmospheric_drag.enabled || contract.inclination_rad~=pi/2 || ...
            contract.initial_stack_mass_kg~=2060 || contract.mu_m3_s2~=3.986004418e14 || ...
            contract.J2~=1.08263e-3 || contract.earth_radius_m~=6378137 || ...
            contract.impulse_isp_s~=200 || contract.target_mass_kg~=2000 || ...
            ~same(atmosphere,contract.atmospheric_drag) || contract.use_thrust_noise || ...
            ~isequal(contract.desired_relative_position_m,[0;-5000;0])
        error('mission:IncompatibleOptimizer','Legacy replay is restricted to the original baseline scenario.');
    end
    warning('mission:LegacyOptimizerReplay','Explicit legacy replay: archive is nonconverged and lacks a full physical contract; not reference-vehicle validation.');
    return;
end
error('mission:MissingOptimizerContract', ...
    'Python artifact lacks a verified physics contract. Use a new compatible design, explicit manoeuvre inputs with mode NONE, or the pinned legacy replay.');
end

function tf=same(a,b)
if isstruct(a) && isstruct(b)
    names=sort(fieldnames(a)); tf=isequal(names,sort(fieldnames(b)));
    if ~tf, return; end
    for k=1:numel(names)
        if ~same(a.(names{k}),b.(names{k})), tf=false; return; end
    end
elseif (ischar(a)||isstring(a)) && (ischar(b)||isstring(b))
    tf=isequal(string(a),string(b));
elseif (isnumeric(a)||islogical(a)) && (isnumeric(b)||islogical(b))
    tf=isequaln(double(a(:)),double(b(:)));
else
    tf=isequaln(a,b);
end
end
