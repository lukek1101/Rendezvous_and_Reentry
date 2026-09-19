function validate_apogee_burns(a)
    for name={'total_delta_v_m_s','thrust_N','isp_s','max_burn_duration_s', ...
            'minimum_mass_kg','max_elapsed_s','max_step_s'}
        validateattributes(a.(name{1}),{'numeric'},{'scalar','real','finite','positive'});
    end
    validateattributes(a.cooldown_s,{'numeric'},{'scalar','real','finite','nonnegative'});
    validateattributes(a.max_burns,{'numeric'},{'scalar','integer','positive'});
    validateattributes(a.fractions,{'numeric'},{'vector','nonempty','finite','positive'});
    if ~isscalar(string(a.first_burn)) || ~any(string(a.first_burn)==["CURRENT" "NEXT_APOGEE"])
        error('mission:FirstBurn','first_burn must be CURRENT or NEXT_APOGEE.');
    end
end
