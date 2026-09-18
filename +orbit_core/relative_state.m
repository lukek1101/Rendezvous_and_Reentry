function [r_rel, v_rel, C_I2L] = relative_state(X_chaser, X_target)
%RELATIVE_STATE Target-centered [R,V,H] state with rotating-frame velocity.
% Preserves the mission's h/r^2 frame-rate approximation. A full frame rate
% for a precessing/nonplanar target is a separate physical-model decision.
    r_t = X_target(1:3);
    v_t = X_target(4:6);
    h_vec = cross(r_t, v_t);
    i_u = r_t / norm(r_t);
    k_u = h_vec / norm(h_vec);
    j_u = cross(k_u, i_u);
    C_I2L = [i_u'; j_u'; k_u'];
    rho_eci = X_chaser(1:3) - r_t;
    r_rel = C_I2L * rho_eci;
    omega_eci = h_vec / norm(r_t)^2;
    v_rel = C_I2L * (X_chaser(4:6) - v_t - cross(omega_eci, rho_eci));
end
