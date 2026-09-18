function value = get_json_bool(cfg, path, default_value)
    value = get_json_value(cfg, path, default_value);
    if islogical(value)
        return;
    end
    if isnumeric(value)
        value = value ~= 0;
        return;
    end
    txt = lower(strtrim(string(value)));
    value = txt == "true" || txt == "1" || txt == "yes" || txt == "on";
end
