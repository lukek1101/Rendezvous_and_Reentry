function tf = orbital_drag_enabled(sys)
    tf = false;
    if ~isfield(sys, 'environment') || ~isfield(sys.environment, 'atmospheric_drag')
        return;
    end

    drag = sys.environment.atmospheric_drag;
    if isfield(drag, 'enabled')
        tf = parse_bool_setting(drag.enabled);
    end
    if tf && isfield(drag, 'model')
        model = upper(string(drag.model));
        tf = ~(model == "OFF" || model == "NONE" || model == "DISABLED" || model == "0");
    end
end
