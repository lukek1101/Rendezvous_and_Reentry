function value = get_run_bool_field(s, path, default_value)
    raw = get_run_value(s, path, default_value);
    if islogical(raw)
        value = raw;
    elseif isnumeric(raw) && isscalar(raw)
        value = raw ~= 0;
    else
        text_value = lower(strtrim(string(raw)));
        if any(text_value == ["1", "true", "yes", "y", "on"])
            value = true;
        elseif any(text_value == ["0", "false", "no", "n", "off"])
            value = false;
        else
            value = default_value;
        end
    end
end
