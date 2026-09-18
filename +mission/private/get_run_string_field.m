function value = get_run_string_field(s, path, default_value)
    value = get_run_value(s, path, default_value);
    if isempty(value)
        value = default_value;
    end
    value = string(value);
end
