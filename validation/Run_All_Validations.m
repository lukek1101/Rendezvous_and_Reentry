function results = Run_All_Validations()
%RUN_ALL_VALIDATIONS Active MATLAB project checks and full mission regression.
% Python archive checks: python -m unittest discover -s validation -p test_*.py
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    addpath(fullfile(root,'validation'));
    results.code = Check_Project_Code();
    results.orbit = Validate_Orbit_Core();
    results.deorbit = Validate_Deorbit_Execution();
    results.reentry_core = Validate_Reentry_Core_Equivalence();
    results.reentry = Validate_Reentry_Propagator();
    results.papers = Validate_Paper_Reproduction();
    results.mission = Validate_Mission_Architecture();
    results.passed = true;
    fprintf('All MATLAB project validations: PASS\n');
end
