function [aoa_deg, bank_deg, profile_info] = resolve_commands(shape, aoa_fallback_deg, bank_fallback_deg, speed_rel, mach, altitude_m, entry_time_s)
    if nargin<6, altitude_m=NaN; end
    if nargin<7, entry_time_s=0; end
    profile_mode = upper(string(get_field(shape, 'aoa_profile_mode', "CONSTANT")));
    bank_deg = bank_fallback_deg;
    profile_info=struct('source',"EXISTING_COMMAND",'boundary_held',false,'query',NaN);

    if profile_mode=="EXPLICIT_PROFILE"
        p=shape.active_aoa_profile;
        switch string(p.axis)
            case "ALTITUDE_M", query=altitude_m;
            case "ENTRY_TIME_S", query=entry_time_s+p.time_offset_s;
            case "MACH", query=mach;
            case "AIR_SPEED_M_S", query=speed_rel;
            otherwise, error('reference_vehicle:Profile','Unknown profile axis.');
        end
        if ~isfinite(query), error('reference_vehicle:ProfileQuery','Profile requires a finite query.'); end
        outside=query<p.grid(1) || query>p.grid(end);
        if outside && string(p.boundary_policy)=="ERROR"
            error('reference_vehicle:ProfileDomain','AoA query outside the user profile domain.');
        end
        aoa_deg=interp1(p.grid,p.values_deg,min(max(query,p.grid(1)),p.grid(end)),'linear');
        profile_info=struct('source',string(p.source),'boundary_held',outside,'query',query);
    elseif profile_mode == "SPEED_SCHEDULE"
        speed_grid = get_field(shape, 'aoa_speed_grid_m_s', [0 8000]);
        aoa_grid = get_field(shape, 'aoa_values_deg', [aoa_fallback_deg aoa_fallback_deg]);
        speed_query = min(max(speed_rel, speed_grid(1)), speed_grid(end));
        aoa_deg = interp1(speed_grid, aoa_grid, speed_query, 'linear');
    elseif profile_mode == "MACH_SCHEDULE"
        mach_grid = get_field(shape, 'aoa_mach_grid', [0 30]);
        aoa_grid = get_field(shape, 'aoa_values_deg', [aoa_fallback_deg aoa_fallback_deg]);
        mach_query = min(max(mach, mach_grid(1)), mach_grid(end));
        aoa_deg = interp1(mach_grid, aoa_grid, mach_query, 'linear');
    else
        aoa_deg = aoa_fallback_deg;
    end
end
