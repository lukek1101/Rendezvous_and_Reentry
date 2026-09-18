function value = get_run_value(s, path, default_value)
    value = default_value;
    if ~has_run_path(s, path)
        return;
    end

    node = s;
    for ii = 1:numel(path)
        node = node.(path{ii});
    end
    if ~isempty(node)
        value = node;
    end
end
