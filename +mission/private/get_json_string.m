function value = get_json_string(cfg, path, default_value)
    value = get_json_value(cfg, path, default_value);
    value = string(value);
end
