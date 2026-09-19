function state = initial_state(sys,latlon_deg,altitude_m,speed_m_s,fpa_deg,heading_deg,c)
%INITIAL_STATE Air-relative speed/FPA; heading clockwise from local north.
    lat=latlon_deg(1); lon=latlon_deg(2);
    up=[cosd(lat)*cosd(lon);cosd(lat)*sind(lon);sind(lat)];
    east=[-sind(lon);cosd(lon);0]; north=cross(up,east);
    rf=(sys.Re+altitude_m)*up;
    vf=speed_m_s*(sind(fpa_deg)*up+cosd(fpa_deg)*(cosd(heading_deg)*north+sind(heading_deg)*east));
    rotation=entry_design.ecef(eye(3),c.entry_epoch_s,c);
    r=rotation'*rf;
    v=rotation'*vf+cross([0;0;c.earth_rotation_rad_s],r);
    state=[r;v];
end
