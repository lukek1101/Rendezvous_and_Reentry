function value = get_json_value(cfg, path, default_value)
    if ~has_json_path(cfg, path)
        value = default_value;
        return;
    end

    value = cfg;
    for ii = 1:numel(path)
        value = value.(path{ii});
    end
end
