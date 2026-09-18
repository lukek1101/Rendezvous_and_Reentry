function value = parse_bool_setting(raw_value)
    if islogical(raw_value)
        value = raw_value;
        return;
    end

    if isnumeric(raw_value) && isscalar(raw_value)
        value = raw_value ~= 0;
        return;
    end

    text_value = lower(strtrim(string(raw_value)));
    if any(text_value == ["1", "true", "yes", "y", "on"])
        value = true;
    elseif any(text_value == ["0", "false", "no", "n", "off"])
        value = false;
    else
        error('Boolean setting must be true/false, on/off, yes/no, or 1/0. Got: %s', string(raw_value));
    end
end
