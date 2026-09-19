function vehicle = vehicle(sys,mode,overrides)
%VEHICLE Adapter only. Solvers accept any mass/shape, with no vehicle-name branches.
    if nargin<3, overrides=struct(); end
    mode=upper(string(mode));
    if mode=="CAPSULE"
        shape=sys.reentry_vehicle.shapes.CAPSULE;
        config=sys.reentry_vehicle.capsule;
        mass=config.mass_kg;
        mass_policy="FIXED_SEPARATED";
        config.aoa_profile_mode="CONSTANT_TRIM";
        config.default_aoa_deg=config.trim_aoa_deg;
    elseif mode=="SPACEPLANE"
        shape=sys.reentry_vehicle.shapes.(char(sys.reentry_vehicle.selected_shape));
        config=sys.reentry_vehicle.spaceplane;
        mass=sys.Chaser_Mass_Init;
        mass_policy="REMAINING_STACK";
    else
        error('entry_design:Vehicle','Use CAPSULE/SPACEPLANE or construct a custom vehicle struct.');
    end
    names=fieldnames(config);
    for k=1:numel(names), shape.(names{k})=config.(names{k}); end
    shape.gravity_model=sys.reentry_vehicle.gravity_model;
    scales=fieldnames(sys.reentry_vehicle.uncertainty);
    for k=1:numel(scales), shape.(scales{k})=sys.reentry_vehicle.uncertainty.(scales{k}); end
    vehicle=struct('name',mode,'mass_kg',mass,'shape',shape,'aoa_deg',0, ...
        'heat_coefficient',sys.reentry_vehicle.sutton_graves_k,'deorbit_mass_policy',mass_policy);
    if isfield(shape,'default_aoa_deg'), vehicle.aoa_deg=shape.default_aoa_deg; end
    names=fieldnames(overrides);
    for k=1:numel(names), vehicle.(names{k})=overrides.(names{k}); end
end
