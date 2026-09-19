function degrees = earth_angle(epoch_utc)
%EARTH_ANGLE Approximate Greenwich mean sidereal angle for an explicit UTC epoch.
% UTC approximates UT1; no polar motion/nutation or precision Earth orientation.
    if ~isdatetime(epoch_utc) || ~isscalar(epoch_utc) || isnat(epoch_utc) || isempty(epoch_utc.TimeZone)
        error('entry_design:Epoch','Supply a scalar timezone-aware datetime.');
    end
    epoch_utc.TimeZone='UTC'; jd=juliandate(epoch_utc);
    centuries=(jd-2451545)/36525;
    degrees=mod(280.46061837+360.98564736629*(jd-2451545)+ ...
        0.000387933*centuries^2-centuries^3/38710000,360);
end
