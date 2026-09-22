function dX = dynamics(X, sys, shape, aoa_deg, bank_angle_deg, lift_enabled, heat_k, entry_time_s)
    if nargin<8, entry_time_s=0; end
    X = X(:);
    aux = reentry_core.evaluate_state( ...
        X, sys, shape, aoa_deg, bank_angle_deg, lift_enabled, heat_k, entry_time_s);
    dX = [X(4:6); aux.a_gravity + aux.a_aero; 0];
end
