function [X_reentry_vehicle, hist_reentry, reentry_atmo_info, X_entry_interface] = ...
    entry(sys, X_entry_interface, X_orbiting_entry_relay0, phase3_elapsed_to_interface, ...
          mission_elapsed_to_entry_s, capsule_mass_added_to_initial_stack)
%ENTRY Separate the configured entry vehicle and propagate atmospheric entry.
fprintf('\n[Phase 4] Atmospheric re-entry from %.0f km interface...\n', sys.h_entry_interface/1000);
entry_params = struct();
entry_params.vehicle_mode = sys.reentry_vehicle.vehicle_mode;
entry_params.shape_name = sys.reentry_vehicle.selected_shape;
entry_params.paper_tdrs_entry_epoch_s = mission_elapsed_to_entry_s;

entry_stack_mass_kg = X_entry_interface(14);
entry_capsule_mass_kg = NaN;
entry_carrier_mass_after_kg = NaN;
carrier_post_separation_status = "NOT_APPLICABLE";
if upper(string(entry_params.vehicle_mode)) == "CAPSULE"
    entry_params.shape_name = "CAPSULE";
    entry_params.capsule_mass_kg = sys.reentry_vehicle.capsule.mass_kg;
    entry_params.separation_mode = sys.reentry_vehicle.capsule.separation_mode;
    entry_capsule_mass_kg = entry_params.capsule_mass_kg;
    if ~isfinite(entry_capsule_mass_kg) || entry_capsule_mass_kg <= 0
        error('Capsule mass must be a finite positive value.');
    end
    if entry_capsule_mass_kg > entry_stack_mass_kg
        error(['Capsule mass %.3f kg exceeds the %.3f kg entry stack. ' ...
               'The initial chaser mass ledger must include the capsule.'], ...
              entry_capsule_mass_kg, entry_stack_mass_kg);
    end
    if upper(string(entry_params.separation_mode)) == "ENTRY_INTERFACE"
        entry_carrier_mass_after_kg = entry_stack_mass_kg - entry_capsule_mass_kg;
        X_entry_interface(14) = entry_params.capsule_mass_kg;
        fprintf('   capsule separation at entry interface: stack %.3f kg -> capsule %.3f kg (zero separation impulse assumed)\n', ...
                entry_stack_mass_kg, X_entry_interface(14));
        fprintf('   residual carrier mass: %.3f kg (carrier atmospheric trajectory NOT PROPAGATED)\n', ...
                entry_carrier_mass_after_kg);
        carrier_post_separation_status = "NOT_PROPAGATED";
    elseif upper(string(entry_params.separation_mode)) ~= "ATTACHED"
        error('Unknown capsule separation_mode: %s. Use ENTRY_INTERFACE or ATTACHED.', ...
              char(string(entry_params.separation_mode)));
    else
        carrier_post_separation_status = "ATTACHED_STACK_PROPAGATED";
    end
end

[X_reentry_vehicle, ~, hist_reentry, reentry_atmo_info] = ...
    Reentry_Propagator(sys, X_entry_interface, X_orbiting_entry_relay0, phase3_elapsed_to_interface, entry_params);
reentry_atmo_info.stack_mass_before_separation_kg = entry_stack_mass_kg;
reentry_atmo_info.capsule_mass_kg = entry_capsule_mass_kg;
reentry_atmo_info.carrier_mass_after_separation_kg = entry_carrier_mass_after_kg;
reentry_atmo_info.carrier_post_separation_status = carrier_post_separation_status;
reentry_atmo_info.capsule_mass_added_to_initial_stack = capsule_mass_added_to_initial_stack;

active_vehicle_mass_kg = X_reentry_vehicle(14);
if isfinite(entry_carrier_mass_after_kg)
    total_accounted_mass_kg = active_vehicle_mass_kg + entry_carrier_mass_after_kg;
else
    total_accounted_mass_kg = active_vehicle_mass_kg;
end
reentry_atmo_info.active_vehicle_mass_kg = active_vehicle_mass_kg;
reentry_atmo_info.total_accounted_mass_kg = total_accounted_mass_kg;

end

