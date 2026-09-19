function [latlon,xy] = geography(r_ecef,origin,heading,earth_radius)
%GEOGRAPHY Geocentric lat/lon and spherical along/cross-range relative to entry.
    u=r_ecef/norm(r_ecef); up=origin/norm(origin);
    along=heading-dot(heading,up)*up;
    if norm(along)<1e-10
        error('entry_design:Heading','Downrange requires nonzero horizontal entry velocity.');
    end
    along=along/norm(along);
    right=cross(along,up); right=right/norm(right);
    latlon=[asind(max(-1,min(1,u(3)))),atan2d(u(2),u(1))];
    xy=earth_radius*[atan2(dot(u,along),dot(u,up)),asin(max(-1,min(1,dot(u,right))))];
end
