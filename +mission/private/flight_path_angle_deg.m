function fpa_deg = flight_path_angle_deg(r, v)
    rhat = r / norm(r);
    v_radial = dot(v, rhat);
    v_horizontal = norm(v - v_radial * rhat);
    fpa_deg = rad2deg(atan2(v_radial, v_horizontal));
end
