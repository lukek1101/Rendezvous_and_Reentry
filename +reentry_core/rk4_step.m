function X_next = rk4_step(X, sys, shape, aoa_deg, bank_angle_deg, lift_enabled, heat_k, dt, entry_time_s)
    if nargin<9, entry_time_s=0; end
    validateattributes(dt, {'numeric'}, ...
        {'real','scalar','finite','nonnegative'}, mfilename, 'dt');
    X = X(:);
    f = @(X_now,t) reentry_core.dynamics( ...
        X_now, sys, shape, aoa_deg, bank_angle_deg, lift_enabled, heat_k, t);
    k1 = f(X,entry_time_s);
    k2 = f(X + 0.5 * dt * k1,entry_time_s+dt/2);
    k3 = f(X + 0.5 * dt * k2,entry_time_s+dt/2);
    k4 = f(X + dt * k3,entry_time_s+dt);
    X_next = X + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end
