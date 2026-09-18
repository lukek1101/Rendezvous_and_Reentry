function a = gravity_j2(r, sys)
%GRAVITY_J2 Central gravity plus the existing zonal J2 model, in SI / ECI.
    r_norm = norm(r);
    a_g = -sys.mu / r_norm^3 * r;
    z2 = (r(3)/r_norm)^2;
    factor = 1.5 * sys.J2 * (sys.mu/r_norm^2) * (sys.Re/r_norm)^2;
    a_j2 = factor * [(r(1)/r_norm)*(5*z2 - 1); ...
                    (r(2)/r_norm)*(5*z2 - 1); ...
                    (r(3)/r_norm)*(5*z2 - 3)];
    a = a_g + a_j2;
end
