function record=state_record(X,sys,time_s,source)
%STATE_RECORD Explicit phase state, shared by integrated and prescribed entry.
validateattributes(X,{'numeric'},{'vector','real','finite'});
if numel(X)==14, m=X(14); elseif numel(X)==7, m=X(7); else, error('mission:StateSize','Expected 7 or 14 state elements.'); end
if m<=0, error('mission:StateMass','Phase mass must be positive.'); end
r=X(1:3); v=X(4:6); r=r(:); v=v(:);
if ~isfield(sys,'contract')
    legacy=Mission_Config("LEGACY_CAPSULE_60KG"); sys.contract=legacy.contract;
end
atm=sys.environment.atmospheric_drag;
omega=atm.earth_rotation_rad_s;
if isscalar(omega), omega=[0;0;omega]; end
va=v;
if atm.co_rotate_atmosphere, va=v-cross(omega(:),r); end
record=struct('schema_version',2,'source',string(source),'time_since_mission_epoch_s',time_s, ...
    'epoch',sys.contract,'position_eci_m',r,'velocity_eci_m_s',v,'mass_kg',m, ...
    'altitude_spherical_m',norm(r)-sys.Re,'speed_inertial_m_s',norm(v), ...
    'fpa_inertial_deg',reentry_core.flight_path_angle_deg(r,v), ...
    'velocity_air_relative_m_s',va,'speed_air_relative_m_s',norm(va), ...
    'fpa_air_relative_deg',reentry_core.flight_path_angle_deg(r,va));
record.earth_angle_at_mission_epoch_deg= ...
    sys.reentry_vehicle.spaceplane.communication.earth_fixed_to_eci_angle_at_mission_epoch_deg;
end
