function r = ecef(r_eci,epoch_s,c)
%ECEF Spherical Earth, explicit Greenwich angle and elapsed epoch (not full EOP).
    angle=deg2rad(c.earth_angle_at_epoch_deg)+c.earth_rotation_rad_s*epoch_s;
    rotation=[cos(angle) sin(angle) 0;-sin(angle) cos(angle) 0;0 0 1];
    r=rotation*r_eci;
end
