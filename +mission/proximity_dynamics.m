function dy = proximity_dynamics(~, y, force_eci, sys)
%PROXIMITY_DYNAMICS Ideal 3-axis translational actuator, independent of attitude.
% y = [chaser r,v; target r,v; chaser mass]. Force held in ECI per control tick.
    if y(13) <= 0
        error('mission:InvalidMass','Nonpositive propagated chaser mass.');
    end
    ac = orbit_core.gravity_j2(y(1:3),sys) + ...
        Atmospheric_Drag_Acceleration(y(1:3),y(4:6),y(13),sys,"chaser");
    at = orbit_core.gravity_j2(y(7:9),sys) + ...
        Atmospheric_Drag_Acceleration(y(7:9),y(10:12),sys.Target_Mass,sys,"target");
    dy = [y(4:6); ac+force_eci/y(13); y(10:12); at; ...
        -norm(force_eci)/(sys.Isp*sys.g0)];
end
