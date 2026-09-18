function merged = merge_settings(defaults, overrides, path)
%MERGE_SETTINGS Recursively overlay known settings; reject misspelled fields.
    if nargin < 3
        path = 'settings';
    end
    if ~isstruct(overrides) || ~isscalar(overrides)
        error('mission:InvalidSettings', '%s must be a scalar struct.', path);
    end
    merged = defaults;
    names = fieldnames(overrides);
    for k = 1:numel(names)
        name = names{k};
        field_path = [path '.' name];
        if ~isfield(defaults, name)
            error('mission:UnknownSetting', 'Unknown setting: %s', field_path);
        end
        if isstruct(defaults.(name))
            merged.(name) = mission.merge_settings(defaults.(name), overrides.(name), field_path);
        else
            merged.(name) = overrides.(name);
        end
    end
end
