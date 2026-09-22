function p=aoa_profile(name)
%AOA_PROFILE Reference schedules, not aerodynamic validity or feedback guidance.
root=fileparts(fileparts(mfilename('fullpath')));
name=upper(string(name));
if name=="APOLLO7_PREFLIGHT_TRIM"
    d=reference_vehicle.apollo7_data();
    p=struct('axis',"MACH",'grid',d.mach(:),'values_deg',d.alpha_deg(:), ...
        'source',d.source);
elseif name=="ARD"
    d=jsondecode(fileread(fullfile(root,'configs','reference_profiles','ard_mean_aoa_2007.json')));
    p=struct('axis',"ALTITUDE_M",'grid',flipud(d.altitude_m(:)), ...
        'values_deg',flipud(d.simulator_aoa_magnitude_deg(:)), ...
        'source',string(d.source));
elseif name=="HORUS_2B"
    d=jsondecode(fileread(fullfile(root,'configs','reference_profiles','horus_shuttle_inspired_speed.json')));
    p=struct('axis',"AIR_SPEED_M_S",'grid',d.air_speed_m_s(:), ...
        'values_deg',d.aoa_deg(:),'source',string(d.source));
elseif name=="HORUS_2B_PUBLISHED_TIME"
    d=jsondecode(fileread(fullfile(root,'configs','reference_profiles','horus_aoa_2024.json')));
    p=struct('axis',"ENTRY_TIME_S",'grid',d.time_since_entry_s(:), ...
        'values_deg',d.aoa_deg(:),'source',string(d.source));
else
    error('reference_vehicle:Profile','Unknown reference AoA profile.');
end
p.boundary_policy="HOLD_ENDPOINT";
if name=="APOLLO7_PREFLIGHT_TRIM", p.boundary_policy="ERROR"; end
p.time_offset_s=0;
p.classification="REFERENCE_PROFILE_FALLBACK";
if name=="HORUS_2B"
    p.provenance_class="USER_SUPPLIED_SHUTTLE_INSPIRED_ASSUMPTION";
end
end
