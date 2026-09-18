function [X_chaser, X_target, result] = proximity(sys, X_chaser, X_target, p2)
%PROXIMITY Waypoint-impulse baseline, ending at a standoff (not docking).
fprintf('\n[Phase 2] Starting waypoint-impulsive R-bar approach...\n');

% -------------------------------------------------------------------------
% Phase 2 concept
%   - S2:  -V-bar 5 km hold point, inherited from Phase 1 when possible.
%   - S2 -> S3: one retrograde/-V-bar impulse, then free cycloidal drift.
%       * v0 is selected from |delta_R|max = 4*v0/n.
%       * S3 is not pre-fixed. S3 is the first point where V-bar coordinate
%         crosses zero after the cycloidal drift starts.
%   - S3 -> S4: short two-impulse R-bar hops. Each hop uses CW targeting for
%     the departure impulse, nonlinear Env_EOM propagation, then a small
%     braking impulse to hold before the next hop.
%   - No state overwriting: all corrections are impulses + free propagation.
% -------------------------------------------------------------------------

% Current actual relative state after Phase 1
[rel0_lvlh, vrel0_lvlh] = orbit_core.relative_state(X_chaser, X_target);
fprintf('   Phase 2 initial LVLH rel-pos = [%+.2f, %+.2f, %+.2f] m\n', ...
        rel0_lvlh(1), rel0_lvlh(2), rel0_lvlh(3));
fprintf('   Phase 2 initial LVLH rel-vel = [%+.4f, %+.4f, %+.4f] m/s\n', ...
        vrel0_lvlh(1), vrel0_lvlh(2), vrel0_lvlh(3));

% Initialize Phase 2 history containers
hist_pos = rel0_lvlh;
hist_mass = X_chaser(14);
hist_p2 = struct();
hist_p2.pos = X_chaser(1:3);
hist_p2.time = 0;
hist_p2.mass = X_chaser(14);
phase2_time = 0;
dV_p2 = 0;
fuel_p2 = 0;

% Keep a list of meaningful Phase 2 points for plotting.
phase2_targets = [];
phase2_target_times = [];      % [s] elapsed time from Phase 2 start
phase2_tofs = [];
phase2_names = strings(1,0);

% -------------------------------------------------------------------------
% 2-0. Optional cleanup to S2, then brake to make S2 a true hold point.
% -------------------------------------------------------------------------
if norm(rel0_lvlh - p2.S2) > p2.initial_S2_tol
    tof_seg = p2.tof_initial_s2;
    [r_rel, v_rel] = orbit_core.relative_state(X_chaser, X_target);
    n_now = target_mean_motion(X_target);
    dv_lvlh = cw_delta_v_to_waypoint(r_rel, v_rel, p2.S2, tof_seg, n_now);

    [X_chaser, fuel_seg] = apply_impulse_lvlh(X_chaser, X_target, dv_lvlh, sys, p2);
    dV_p2 = dV_p2 + norm(dv_lvlh);
    fuel_p2 = fuel_p2 + fuel_seg;

    fprintf('   cleanup_to_S2     : target [%+7.1f,%+7.1f,%+6.1f] m, TOF %5.0f s, dV %8.4f m/s\n', ...
            p2.S2(1), p2.S2(2), p2.S2(3), tof_seg, norm(dv_lvlh));

    [X_chaser, X_target, seg_hist] = propagate_pair_free(X_chaser, X_target, tof_seg, p2.dt, sys, phase2_time);
    phase2_time = seg_hist.time(end);

    [hist_pos, hist_mass, hist_p2] = append_phase2_segment(hist_pos, hist_mass, hist_p2, seg_hist);
end

% S2 hold trim: cancel any remaining LVLH relative velocity before the
% cycloidal free drift. This is necessary because the 4*v0/n relation assumes
% a clean S2 starting condition except for the deliberate -V-bar impulse.
[~, v_s2] = orbit_core.relative_state(X_chaser, X_target);
dv_hold_s2_lvlh = -v_s2;
if norm(dv_hold_s2_lvlh) > 1e-6
    [X_chaser, fuel_seg] = apply_impulse_lvlh(X_chaser, X_target, dv_hold_s2_lvlh, sys, p2);
    dV_p2 = dV_p2 + norm(dv_hold_s2_lvlh);
    fuel_p2 = fuel_p2 + fuel_seg;
    hist_mass(end) = X_chaser(14);
    hist_p2.mass(end) = X_chaser(14);
    fprintf('   S2_hold_trim      : residual rel-vel canceled, dV %8.4f m/s\n', norm(dv_hold_s2_lvlh));
end

% Record S2 waypoint time after optional cleanup and hold trim.
[phase2_targets, phase2_target_times, phase2_tofs, phase2_names] = ...
    append_phase2_waypoint(phase2_targets, phase2_target_times, phase2_tofs, phase2_names, p2.S2, phase2_time, "S2", []);

% -------------------------------------------------------------------------
% 2-1. S2 -> S3 natural cycloidal drift using one -V-bar impulse.
% -------------------------------------------------------------------------
n_now = target_mean_motion(X_target);
v0_cycloid = n_now * p2.delta_R_cycloid / 4;
dv_cycloid_lvlh = [0; p2.vbar_burn_sign * v0_cycloid; 0];
expected_delta_R_code = 4 * dv_cycloid_lvlh(2) / n_now;

[X_chaser, fuel_seg] = apply_impulse_lvlh(X_chaser, X_target, dv_cycloid_lvlh, sys, p2);
dV_p2 = dV_p2 + norm(dv_cycloid_lvlh);
fuel_p2 = fuel_p2 + fuel_seg;
hist_mass(end) = X_chaser(14);
hist_p2.mass(end) = X_chaser(14);

fprintf('   S2_to_S3_cycloid  : single %+.4f m/s V-bar impulse, |delta_R|max %.1f m', ...
        dv_cycloid_lvlh(2), p2.delta_R_cycloid);
fprintf(' (current LVLH signed delta_R %.1f m)\n', expected_delta_R_code);

% Propagate freely until the chaser first reaches the target R-bar line,
% i.e., until V-bar coordinate crosses zero.
max_cycloid_time = p2.max_cycloid_orbits * 2*pi / n_now;
cycloid_elapsed = 0;
[r_prev, ~] = orbit_core.relative_state(X_chaser, X_target);

initial_vbar_sign = sign(r_prev(2));
if initial_vbar_sign == 0
    initial_vbar_sign = -1;
end
crossed_rbar = false;
% Allocate the long cycloid history once instead of copying it every step.
sample_capacity = ceil(max_cycloid_time / p2.dt) + 1;
cycloid_hist = struct('rel', zeros(3,sample_capacity), ...
    'pos', zeros(3,sample_capacity), 'mass', zeros(1,sample_capacity), ...
    'time', zeros(1,sample_capacity));
sample_count = 0;

while cycloid_elapsed < max_cycloid_time
    [X_chaser, X_target, seg_hist] = propagate_pair_free(X_chaser, X_target, p2.dt, p2.dt, sys, phase2_time);
    phase2_time = seg_hist.time(end);
    cycloid_elapsed = cycloid_elapsed + p2.dt;

    sample_count = sample_count + 1;
    cycloid_hist.rel(:,sample_count) = seg_hist.rel;
    cycloid_hist.pos(:,sample_count) = seg_hist.pos;
    cycloid_hist.mass(sample_count) = seg_hist.mass;
    cycloid_hist.time(sample_count) = seg_hist.time;

    [r_now, ~] = orbit_core.relative_state(X_chaser, X_target);
    vbar_now = r_now(2);

    if abs(vbar_now) <= p2.vbar_cross_tol || sign(vbar_now) ~= initial_vbar_sign
        crossed_rbar = true;
        break;
    end

end

fields = fieldnames(cycloid_hist);
for field_index = 1:numel(fields)
    field = fields{field_index};
    cycloid_hist.(field) = cycloid_hist.(field)(:,1:sample_count);
end
[hist_pos, hist_mass, hist_p2] = append_phase2_segment(hist_pos, hist_mass, hist_p2, cycloid_hist);

if ~crossed_rbar
    error('Phase 2 cycloidal drift did not reach the R-bar line within %.2f orbits. Check S2 offset, delta_R_cycloid, and sign convention.', p2.max_cycloid_orbits);
end

[r_s3, v_s3] = orbit_core.relative_state(X_chaser, X_target);
p2.S3 = [r_s3(1); 0; 0];
[phase2_targets, phase2_target_times, phase2_tofs, phase2_names] = ...
    append_phase2_waypoint(phase2_targets, phase2_target_times, phase2_tofs, phase2_names, p2.S3, phase2_time, "S3", cycloid_elapsed);

fprintf('      S3 detected at first V-bar crossing after %.2f min\n', cycloid_elapsed/60);
fprintf('      S3 actual LVLH rel-pos = [%+.3f, %+.3f, %+.3f] m\n', r_s3(1), r_s3(2), r_s3(3));
fprintf('      S3 actual LVLH rel-vel = [%+.5f, %+.5f, %+.5f] m/s\n', v_s3(1), v_s3(2), v_s3(3));

% Brake at S3 before starting the controlled R-bar hop sequence. Without this,
% the next S3->S4 hop starts with a large natural-drift velocity and can become
% an unsafe fly-by rather than an approach.
dv_hold_s3_lvlh = -v_s3;
if norm(dv_hold_s3_lvlh) > 1e-6
    [X_chaser, fuel_seg] = apply_impulse_lvlh(X_chaser, X_target, dv_hold_s3_lvlh, sys, p2);
    dV_p2 = dV_p2 + norm(dv_hold_s3_lvlh);
    fuel_p2 = fuel_p2 + fuel_seg;
    hist_mass(end) = X_chaser(14);
    hist_p2.mass(end) = X_chaser(14);
    fprintf('   S3_hold_brake     : cycloid rel-vel canceled, dV %8.4f m/s\n', norm(dv_hold_s3_lvlh));
end

% -------------------------------------------------------------------------
% 2-2. S3 -> S4 R-bar approach.
% -------------------------------------------------------------------------
% Use the same signed R-bar side as the actual S3. This avoids commanding the
% chaser to cross through the target just because of a sign-convention mismatch.
approach_R_sign = sign(p2.S3(1));
if approach_R_sign == 0
    approach_R_sign = sign(expected_delta_R_code);
end
if approach_R_sign == 0
    approach_R_sign = 1;
end
p2.S4 = [approach_R_sign * p2.S4_R_abs; 0; 0];

if abs(p2.S3(1)) <= abs(p2.S4(1))
    warning('S3 radial distance %.2f m is already inside or near S4 %.2f m. Skipping R-bar hops and using terminal refine only.', p2.S3(1), p2.S4(1));
    rbar_hops = p2.S4(1);
else
    rbar_hops = linspace(p2.S3(1), p2.S4(1), p2.rbar_hop_count + 1);
    rbar_hops = rbar_hops(2:end);
end

for ii = 1:numel(rbar_hops)
    r_goal = [rbar_hops(ii); 0; 0];
    tof_seg = p2.tof_hop;

    [r_rel, v_rel] = orbit_core.relative_state(X_chaser, X_target);
    n_now = target_mean_motion(X_target);
    dv_lvlh = cw_delta_v_to_waypoint(r_rel, v_rel, r_goal, tof_seg, n_now);

    [X_chaser, fuel_seg] = apply_impulse_lvlh(X_chaser, X_target, dv_lvlh, sys, p2);
    dV_p2 = dV_p2 + norm(dv_lvlh);
    fuel_p2 = fuel_p2 + fuel_seg;

    fprintf('   Rbar_hop_%02d      : target [%+7.1f,%+7.1f,%+6.1f] m, TOF %5.0f s, depart dV %8.4f m/s\n', ...
            ii, r_goal(1), r_goal(2), r_goal(3), tof_seg, norm(dv_lvlh));

    [X_chaser, X_target, seg_hist] = propagate_pair_free(X_chaser, X_target, tof_seg, p2.dt, sys, phase2_time);
    phase2_time = seg_hist.time(end);

    [hist_pos, hist_mass, hist_p2] = append_phase2_segment(hist_pos, hist_mass, hist_p2, seg_hist);

    [~, v_arrive] = orbit_core.relative_state(X_chaser, X_target);

    % Make each hop a real hold point by braking the residual relative velocity.
    % This is more physical/safe than only doing one final brake at S4.
    dv_brake_lvlh = -v_arrive;
    [X_chaser, fuel_brake] = apply_impulse_lvlh(X_chaser, X_target, dv_brake_lvlh, sys, p2);
    dV_p2 = dV_p2 + norm(dv_brake_lvlh);
    fuel_p2 = fuel_p2 + fuel_brake;
    hist_mass(end) = X_chaser(14);
    hist_p2.mass(end) = X_chaser(14);

    if ii == numel(rbar_hops)
        waypoint_name = "S4";
    else
        waypoint_name = "WP" + string(ii);
    end
    [phase2_targets, phase2_target_times, phase2_tofs, phase2_names] = ...
        append_phase2_waypoint(phase2_targets, phase2_target_times, phase2_tofs, phase2_names, r_goal, phase2_time, waypoint_name, tof_seg);

    [r_hold, v_hold] = orbit_core.relative_state(X_chaser, X_target);
    fprintf('      arrival error: pos %.3f m, brake dV %.4f m/s, post-brake rel-vel %.5f m/s\n', ...
            norm(r_hold - r_goal), norm(dv_brake_lvlh), norm(v_hold));
end

% Nonlinear/J2 residual cleanup at S4. This is still done by impulses and
% free propagation, not by overwriting the state.
for refine = 1:p2.max_terminal_refines
    [r_rel, v_rel] = orbit_core.relative_state(X_chaser, X_target);
    pos_err = norm(r_rel - p2.S4);
    if pos_err <= p2.capture_pos_tol
        break;
    end

    n_now = target_mean_motion(X_target);
    dv_lvlh = cw_delta_v_to_waypoint(r_rel, v_rel, p2.S4, p2.tof_terminal_refine, n_now);
    [X_chaser, fuel_seg] = apply_impulse_lvlh(X_chaser, X_target, dv_lvlh, sys, p2);
    dV_p2 = dV_p2 + norm(dv_lvlh);
    fuel_p2 = fuel_p2 + fuel_seg;

    fprintf('   terminal_refine_%d : remaining pos %.3f m, TOF %5.0f s, dV %8.4f m/s\n', ...
            refine, pos_err, p2.tof_terminal_refine, norm(dv_lvlh));

    [X_chaser, X_target, seg_hist] = propagate_pair_free(X_chaser, X_target, p2.tof_terminal_refine, p2.dt, sys, phase2_time);
    phase2_time = seg_hist.time(end);

    [hist_pos, hist_mass, hist_p2] = append_phase2_segment(hist_pos, hist_mass, hist_p2, seg_hist);
end

% Final braking impulse: cancel residual LVLH relative velocity at S4.
[~, v_final] = orbit_core.relative_state(X_chaser, X_target);
dv_stop_lvlh = -v_final;
[X_chaser, fuel_stop] = apply_impulse_lvlh(X_chaser, X_target, dv_stop_lvlh, sys, p2);
dV_p2 = dV_p2 + norm(dv_stop_lvlh);
fuel_p2 = fuel_p2 + fuel_stop;
hist_mass(end) = X_chaser(14);
hist_p2.mass(end) = X_chaser(14);

[r_final, v_final] = orbit_core.relative_state(X_chaser, X_target);
fprintf('   terminal braking dV: %.4f m/s\n', norm(dv_stop_lvlh));
fprintf('   Phase 2 final LVLH rel-pos = [%+.3f, %+.3f, %+.3f] m\n', r_final(1), r_final(2), r_final(3));
fprintf('   Phase 2 final LVLH rel-vel = [%+.5f, %+.5f, %+.5f] m/s\n', v_final(1), v_final(2), v_final(3));
fprintf('   Phase 2 total dV: %.4f m/s, fuel: %.4f kg, elapsed: %.2f min\n', dV_p2, fuel_p2, phase2_time/60);


hist_p2.rel_pos_lvlh = hist_pos;
result = struct('history', hist_p2, 'relative_position', hist_pos, ...
    'mass', hist_mass, 'targets', phase2_targets, 'target_times', phase2_target_times, ...
    'transfer_times', phase2_tofs, 'names', phase2_names, 'config', p2, ...
    'delta_v', dV_p2, 'fuel', fuel_p2, 'duration', phase2_time, ...
    'final_position_error', norm(r_final-p2.S4), 'final_relative_velocity', v_final);
result.reached_standoff = result.final_position_error <= p2.capture_pos_tol;
end

function [hist_pos, hist_mass, hist_p2] = append_phase2_segment(hist_pos, hist_mass, hist_p2, seg_hist)
    hist_pos = [hist_pos, seg_hist.rel];
    hist_mass = [hist_mass, seg_hist.mass];
    hist_p2.pos = [hist_p2.pos, seg_hist.pos];
    hist_p2.time = [hist_p2.time, seg_hist.time];
    hist_p2.mass = [hist_p2.mass, seg_hist.mass];
end

function [targets, target_times, tofs, names] = append_phase2_waypoint(targets, target_times, tofs, names, point, time_s, name, tof_s)
    targets = [targets, point];
    target_times = [target_times, time_s];
    names(end+1) = string(name);
    if ~isempty(tof_s)
        tofs = [tofs, tof_s];
    end
end

function n = target_mean_motion(X_target)
    r_t = X_target(1:3);
    v_t = X_target(4:6);
    h_vec = cross(r_t, v_t);
    n = norm(h_vec) / norm(r_t)^2;
end

function dv_lvlh = cw_delta_v_to_waypoint(r0, v0, rf, tof, n)
    % CW/Hill single-impulse targeting: choose post-impulse v0+dv so that
    % the linearized relative trajectory reaches rf after tof.
    nt = n * tof;
    s = sin(nt);
    c = cos(nt);

    Phi_rr = [4 - 3*c,        0, 0; ...
              6*(s - nt),     1, 0; ...
              0,              0, c];

    Phi_rv = [s/n,              2*(1-c)/n,       0; ...
              -2*(1-c)/n,      (4*s - 3*nt)/n,  0; ...
              0,               0,               s/n];

    if rcond(Phi_rv) < 1e-10
        error('Phase 2 CW targeting became ill-conditioned. Change the segment TOF away from singular transfer times.');
    end

    v_req = Phi_rv \ (rf - Phi_rr*r0);
    dv_lvlh = v_req - v0;
end

function [X_chaser, fuel_used] = apply_impulse_lvlh(X_chaser, X_target, dv_lvlh, sys, p2)
    [~, ~, C_I2L] = orbit_core.relative_state(X_chaser, X_target);
    dv_eci = C_I2L' * dv_lvlh;
    dv_mag = norm(dv_eci);

    X_chaser(4:6) = X_chaser(4:6) + dv_eci;

    m0 = X_chaser(14);
    Isp = impulse_isp(sys, p2);
    g0 = 9.80665;
    if Isp > 0 && dv_mag > 0
        m1 = m0 * exp(-dv_mag/(Isp*g0));
        fuel_used = m0 - m1;
        X_chaser(14) = m1;
    else
        fuel_used = 0;
    end
end

function Isp = impulse_isp(sys, p2)
    candidates = {'Isp_Impulsive', 'Isp_RCS', 'Isp_Thruster', 'Isp'};
    Isp = NaN;
    for ii = 1:numel(candidates)
        if isfield(sys, candidates{ii}) && isnumeric(sys.(candidates{ii})) && isscalar(sys.(candidates{ii}))
            Isp = sys.(candidates{ii});
            break;
        end
    end
    if isnan(Isp)
        Isp = p2.Isp_fallback_s;
    end
end

function [X_chaser, X_target, hist] = propagate_pair_free(X_chaser, X_target, tof, dt, sys, t0)
    n_steps = ceil(tof/dt);
    hist.rel = zeros(3, n_steps);
    hist.pos = zeros(3, n_steps);
    hist.mass = zeros(1, n_steps);
    hist.time = zeros(1, n_steps);

    t_elapsed = 0;
    for kk = 1:n_steps
        dt_step = min(dt, tof - t_elapsed);
        t_abs = t0 + t_elapsed;

        % Target propagation: 6-DOF wrapper with zero force/torque.
        X_t_state = [X_target; zeros(7,1); sys.Target_Mass];
        k1_t = Env_EOM(t_abs,             X_t_state,               [0;0;0], [0;0;0], sys, false);
        k2_t = Env_EOM(t_abs+dt_step/2,   X_t_state+k1_t*dt_step/2,[0;0;0], [0;0;0], sys, false);
        k3_t = Env_EOM(t_abs+dt_step/2,   X_t_state+k2_t*dt_step/2,[0;0;0], [0;0;0], sys, false);
        k4_t = Env_EOM(t_abs+dt_step,     X_t_state+k3_t*dt_step,  [0;0;0], [0;0;0], sys, false);
        X_target = X_target + (dt_step/6)*(k1_t(1:6) + 2*k2_t(1:6) + 2*k3_t(1:6) + k4_t(1:6));

        % Chaser free-flight propagation after the impulse.
        k1 = Env_EOM(t_abs,             X_chaser,             [0;0;0], [0;0;0], sys, true);
        k2 = Env_EOM(t_abs+dt_step/2,   X_chaser+k1*dt_step/2,[0;0;0], [0;0;0], sys, true);
        k3 = Env_EOM(t_abs+dt_step/2,   X_chaser+k2*dt_step/2,[0;0;0], [0;0;0], sys, true);
        k4 = Env_EOM(t_abs+dt_step,     X_chaser+k3*dt_step,  [0;0;0], [0;0;0], sys, true);
        X_chaser = X_chaser + (dt_step/6)*(k1 + 2*k2 + 2*k3 + k4);

        if norm(X_chaser(7:10)) > 0
            X_chaser(7:10) = X_chaser(7:10) / norm(X_chaser(7:10));
        end

        t_elapsed = t_elapsed + dt_step;
        [r_rel, ~] = orbit_core.relative_state(X_chaser, X_target);
        hist.rel(:,kk) = r_rel;
        hist.pos(:,kk) = X_chaser(1:3);
        hist.mass(kk) = X_chaser(14);
        hist.time(kk) = t0 + t_elapsed;
    end
end
