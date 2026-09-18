function a = gravity_acceleration(r, sys, shape)
    r_norm = norm(r);
    a_g = -sys.mu / r_norm^3 * r;
    gravity_model = upper(string(get_field(shape, 'gravity_model', "CENTRAL_SPHERICAL")));
    if gravity_model == "CENTRAL_SPHERICAL" || gravity_model == "CENTRAL"
        a = a_g;
        return;
    end

    a = orbit_core.gravity_j2(r, sys);
end
