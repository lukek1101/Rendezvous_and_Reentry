function result=Run_Nominal_Orbit(overrides,options)
%RUN_NOMINAL_ORBIT Bounded nominal Phase 1 followed by existing Phase 2 only.
% Deliberately stops before deorbit/entry, whose reference coverage is separate.
if nargin<1, overrides=struct(); end
if nargin<2, options=struct(); end
base=struct('python_config',struct('mode',"NONE"), ...
    'phase1',struct('mode',"HOHMANN",'hohmann_method',"NOMINAL_TARGET"));
% Merge with full schema so ordinary partial run overrides remain supported.
preset="APOLLO7_PREFLIGHT_TRIM";
if isfield(options,'preset'), preset=options.preset; end
if isfield(overrides,'reentry') && isfield(overrides.reentry,'vehicle_mode') && ...
        string(overrides.reentry.vehicle_mode)=="SPACEPLANE", preset="HORUS_2B"; end
system=Mission_Config(preset);
if isfield(options,'system'), system=mission.merge_settings(system,options.system); end
base=mission.merge_settings(Mission_Run_Config(system),base);
overrides=mission.merge_settings(base,overrides);
options.stop_after_proximity=true;
result=Run_Mission(overrides,options);
end
