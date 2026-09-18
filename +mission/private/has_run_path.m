function tf = has_run_path(s, path)
    if ~isstruct(s)
        tf = false;
        return;
    end
    node = s;
    for ii = 1:numel(path)
        key = path{ii};
        if ~isstruct(node) || ~isfield(node, key)
            tf = false;
            return;
        end
        node = node.(key);
    end
    tf = true;
end
