function root = project_root()
%PROJECT_ROOT Locate repository data independently of the caller's directory.
    root = fileparts(fileparts(mfilename('fullpath')));
end
