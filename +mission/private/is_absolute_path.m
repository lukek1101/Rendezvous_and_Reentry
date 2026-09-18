function tf = is_absolute_path(path_value)
    text_value = string(path_value);
    tf = startsWith(text_value, filesep) || startsWith(text_value, "\\") || ...
         (strlength(text_value) >= 2 && extractBetween(text_value, 2, 2) == ":");
end
