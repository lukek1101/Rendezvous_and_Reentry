function [state,conditions] = initial_state(sys,latlon_deg,altitude_m,speed_m_s,fpa_deg,heading_deg,c)
%INITIAL_STATE Air-relative speed/FPA; heading clockwise from local north.
    lat=latlon_deg(1); lon=latlon_deg(2);
    up=[cosd(lat)*cosd(lon);cosd(lat)*sind(lon);sind(lat)];
    east=[-sind(lon);cosd(lon);0]; north=cross(up,east);
    rf=(sys.Re+altitude_m)*up;
    vf=speed_m_s*(sind(fpa_deg)*up+cosd(fpa_deg)*(cosd(heading_deg)*north+sind(heading_deg)*east));
    rotation=entry_design.ecef(eye(3),c.entry_epoch_s,c);
    r=rotation'*rf;
    v=rotation'*vf;
    if sys.environment.atmospheric_drag.co_rotate_atmosphere
        if c.earth_rotation_rad_s~=sys.environment.atmospheric_drag.earth_rotation_rad_s
            error('entry_design:RotationMismatch','Entry constructor and atmospheric rotation must agree.');
        end
        v=v+cross([0;0;c.earth_rotation_rad_s],r);
    end
    state=[r;v];
    conditions=struct('source',"STANDALONE_PRESCRIBED", ...
        'altitude_m',altitude_m,'speed_air_relative_m_s',speed_m_s, ...
        'fpa_air_relative_deg',fpa_deg,'heading_clockwise_from_north_deg',heading_deg, ...
        'latitude_geocentric_deg',lat,'longitude_earth_fixed_deg',lon, ...
        'entry_epoch_s',c.entry_epoch_s,'earth_angle_at_epoch_deg',c.earth_angle_at_epoch_deg, ...
        'state_eci_si',state);
end
