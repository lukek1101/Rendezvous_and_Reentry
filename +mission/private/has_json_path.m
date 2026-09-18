function tf = has_json_path(cfg, path)
    tf = true;
    node = cfg;
    for ii = 1:numel(path)
        name = path{ii};
        if ~isstruct(node) || ~isfield(node, name)
            tf = false;
            return;
        end
        node = node.(name);
    end
end
