function result = Run_Mission(overrides, options)
%RUN_MISSION Run a mission without clearing the caller or requiring figures.
% result = Run_Mission() uses Mission_Config and Mission_Run_Config defaults.
% overrides is a partial Mission_Run_Config struct (unknown fields error).
% options.plot (false), verbose (true), seed ([]), system (partial physical
% configuration) control the runner. A supplied seed restores the caller RNG.
% For isolated studies set overrides.runtime.allow_environment_overrides=false.
% Histories and states use SI units and ECI / [R,V,H] LVLH coordinates.
    if nargin < 1
        overrides = struct();
    end
    if nargin < 2
        options = struct();
    end
    defaults = struct('plot', false, 'verbose', true, 'seed', [], ...
        'system', Mission_Config());
    options = mission.merge_settings(defaults, options, 'options');
    validateattributes(options.plot, {'logical'}, {'scalar'}, mfilename, 'options.plot');
    validateattributes(options.verbose, {'logical'}, {'scalar'}, mfilename, 'options.verbose');
    if ~isempty(options.seed)
        validateattributes(options.seed, {'numeric'}, ...
            {'scalar','integer','nonnegative','finite','<=',2^32-1}, mfilename, 'options.seed');
        caller_rng = rng;
        rng_cleanup = onCleanup(@() rng(caller_rng));
        rng(options.seed, 'twister');
    end
    initial_rng = rng;
    timer = tic;
    if options.verbose
        cfg = mission.configure(options.system, overrides);
        result = mission.run(cfg);
        result.log = '';
    else
        result.log = evalc('cfg = mission.configure(options.system, overrides); result = mission.run(cfg);');
    end
    result.metadata.schema_version = 1;
    result.metadata.elapsed_seconds = toc(timer);
    result.metadata.matlab_version = version;
    result.metadata.initial_rng = initial_rng;
    result.metadata.seed = options.seed;
    if options.plot
        mission.plot_results(result);
    end
end
