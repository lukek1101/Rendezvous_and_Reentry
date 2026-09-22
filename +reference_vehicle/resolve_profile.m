function shape=resolve_profile(shape,settings,overrides)
%RESOLVE_PROFILE Explicit user profile > explicit constant > reference fallback.
p=struct();
if isfield(settings,'aoa_profile'), p=settings.aoa_profile; end
if isfield(overrides,'aoa_profile'), p=overrides.aoa_profile; end
explicit_profile=isfield(p,'axis') && strlength(string(p.axis))>0;
explicit_constant=(isfield(overrides,'aoa_deg') && ~isempty(overrides.aoa_deg)) || ...
    (isfield(settings,'aoa_deg') && ~isempty(settings.aoa_deg));
if explicit_profile
    if explicit_constant
        error('reference_vehicle:AmbiguousAoA','Specify a user AoA profile or a constant, not both.');
    end
    p.classification="USER_PROFILE";
elseif explicit_constant
    shape.aoa_profile_mode="CONSTANT_OVERRIDE";
    if isfield(shape,'active_aoa_profile'), shape=rmfield(shape,'active_aoa_profile'); end
    return;
elseif isfield(shape,'reference_aoa_profile')
    p=shape.reference_aoa_profile;
elseif isfield(settings,'reference_aoa_profile')
    p=settings.reference_aoa_profile;
else
    return;
end
if ~isfield(p,'boundary_policy'), p.boundary_policy="ERROR"; end
if ~isfield(p,'time_offset_s'), p.time_offset_s=0; end
if ~isfield(p,'source'), p.source="USER_SUPPLIED"; end
validateattributes(p.grid,{'numeric'},{'vector','finite','real','nonempty'});
validateattributes(p.values_deg,{'numeric'},{'vector','finite','real','nonempty'});
if numel(p.grid)<2 || numel(p.grid)~=numel(p.values_deg) || any(diff(p.grid)<=0) || ...
        ~any(string(p.axis)==["ALTITUDE_M","ENTRY_TIME_S","MACH","AIR_SPEED_M_S"]) || ...
        ~any(string(p.boundary_policy)==["ERROR","HOLD_ENDPOINT"])
    error('reference_vehicle:Profile','Invalid AoA profile axis/grid/boundary policy.');
end
validateattributes(p.time_offset_s,{'numeric'},{'scalar','finite','real'});
shape.active_aoa_profile=p;
shape.aoa_profile_mode="EXPLICIT_PROFILE";
end
