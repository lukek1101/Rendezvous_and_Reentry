function label = bool_label(value)
    if isempty(value) || (isnumeric(value) && isscalar(value) && isnan(value))
        label = "not evaluated";
    elseif value
        label = "on";
    else
        label = "off";
    end
end
