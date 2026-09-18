function value = get_json_number(cfg, path, default_value)
    value = get_json_value(cfg, path, default_value);
    if ischar(value) || isstring(value)
        value = str2double(value);
    end
    if isempty(value) || ~isnumeric(value) || ~isscalar(value) || isnan(value)
        value = default_value;
    end
end
