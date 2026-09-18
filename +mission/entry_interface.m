function [X_entry, info] = entry_interface(hist, X_fallback, sys, target_altitude)
    X_entry = X_fallback;
    
    % Initialize info struct with velocity magnitude and vector from the fallback state
    info = struct('time_s', 0, ...
                  'altitude_m', norm(X_fallback(1:3)) - sys.Re, ...
                  'velocity_m_s', norm(X_fallback(4:6)), ...
                  'fpa_deg', NaN);
                  
    if ~isfield(hist, 'pos') || isempty(hist.pos) || ~isfield(hist, 'vel') || isempty(hist.vel)
        info.fpa_deg = flight_path_angle_deg(X_entry(1:3), X_entry(4:6));
        return;
    end
    
    alt = vecnorm(hist.pos, 2, 1) - sys.Re;
    idx_cross = find(alt(1:end-1) >= target_altitude & alt(2:end) <= target_altitude, 1, 'first');
    
    if ~isempty(idx_cross)
        i0 = idx_cross;
        i1 = idx_cross + 1;
        denom = alt(i1) - alt(i0);
        if abs(denom) <= eps
            alpha = 0;
        else
            alpha = (target_altitude - alt(i0)) / denom;
        end
        alpha = min(1, max(0, alpha));
        
        r_entry = hist.pos(:,i0) + alpha * (hist.pos(:,i1) - hist.pos(:,i0));
        v_entry = hist.vel(:,i0) + alpha * (hist.vel(:,i1) - hist.vel(:,i0));
        
        if isfield(hist, 'mass') && numel(hist.mass) >= i1
            mass_entry = hist.mass(i0) + alpha * (hist.mass(i1) - hist.mass(i0));
        else
            mass_entry = X_fallback(14);
        end
        
        if isfield(hist, 'time') && numel(hist.time) >= i1
            time_entry = hist.time(i0) + alpha * (hist.time(i1) - hist.time(i0));
        else
            time_entry = 0;
        end
    else
        [~, idx] = min(abs(alt - target_altitude));
        r_entry = hist.pos(:,idx);
        v_entry = hist.vel(:,idx);
        
        if isfield(hist, 'mass') && numel(hist.mass) >= idx
            mass_entry = hist.mass(idx);
        else
            mass_entry = X_fallback(14);
        end
        
        if isfield(hist, 'time') && numel(hist.time) >= idx
            time_entry = hist.time(idx);
        else
            time_entry = 0;
        end
    end
    
    X_entry = zeros(14,1);
    X_entry(1:6) = [r_entry; v_entry];
    X_entry(7:10) = [0;0;0;1];
    X_entry(11:13) = [0;0;0];
    X_entry(14) = mass_entry;
    
    % Update the info struct with the interpolated entry state
    info.time_s = time_entry;
    info.altitude_m = norm(r_entry) - sys.Re;
    info.velocity_m_s = norm(v_entry);
    info.fpa_deg = flight_path_angle_deg(r_entry, v_entry);
end

