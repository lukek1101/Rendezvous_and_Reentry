function sys = apply(sys,preset)
%APPLY Published properties separate from declared reduced-model assumptions.
preset=upper(string(preset));
if ~any(preset==["APOLLO7_PREFLIGHT_TRIM","ARD","HORUS_2B","LEGACY_CAPSULE_60KG","LEGACY_PAPER_RLV"])
    error('reference_vehicle:Preset','Unknown reference preset: %s',preset);
end
sys.reference=struct('preset',preset,'revision',"2026-09-21-v2-profiles", ...
    'provenance',"docs/REFERENCE_VEHICLE_RESEARCH.md");
sys.contract=struct('schema_version',2,'position_unit',"m",'velocity_unit',"m/s", ...
    'mass_unit',"kg",'time_unit',"s",'angle_unit',"rad unless field ends _deg", ...
    'translation_frame',"EARTH_CENTERED_INERTIAL",'relative_frame',"R_V_H", ...
    'orbital_geometry',"FIXED_XZ_POLAR",'epoch_utc',"UNSPECIFIED", ...
    'epoch_policy',"RELATIVE_TIME_WITH_EXPLICIT_EARTH_ANGLE", ...
    'propulsion_provenance',"PROJECT_ASSUMPTION_NOT_REFERENCE_VEHICLE", ...
    'attitude_policy',"IDENTITY_PLACEHOLDER_NOT_PROPAGATED");
% Legacy policy target is explicit, not reapplied over user FPA settings.
sys.reentry_flight_path_angle=deg2rad(1.16);
sys.reentry_vehicle.capsule.use_paper_entry_conditions=false;
sys.reentry_vehicle.aoa_profile=struct('axis',"",'grid',[], ...
    'values_deg',[],'boundary_policy',"ERROR",'time_offset_s',0,'source',"USER_SUPPLIED");
if preset=="LEGACY_PAPER_RLV"
    % Retain the previous default only as explicitly labeled frozen replay.
    % Four old selectable source definitions have been removed.
    root=fileparts(fileparts(mfilename('fullpath')));
    frozen=load(fullfile(root,'docs','audit_baseline_2026-09-21','baseline.mat'),'result');
    old=frozen.result.config.system.reentry_vehicle;
    sys.reentry_vehicle.spaceplane=old.spaceplane;
    sys.reentry_vehicle.shapes=struct('LEGACY_PAPER_RLV',old.shapes.COMPROMISE);
    sys.reentry_vehicle.shapes.LEGACY_PAPER_RLV.name="LEGACY_PAPER_RLV";
    sys.reentry_vehicle.selected_shape="LEGACY_PAPER_RLV";
    sys.reentry_vehicle.vehicle_mode="SPACEPLANE";
    return;
end
if preset=="LEGACY_CAPSULE_60KG"
    sys.reentry_vehicle.vehicle_mode="CAPSULE";
    sys.reentry_vehicle.shapes.CAPSULE.name="LEGACY_CAPSULE_60KG";
    return;
end
if preset=="ARD"
    sys.reentry_vehicle.vehicle_mode="CAPSULE";
    s=sys.reentry_vehicle.shapes.CAPSULE;
    s.name="ARD_HYPERSONIC_SURROGATE";
    s.reference_area_m2=pi*2.8^2/4; s.nose_radius_m=3.36;
    s.length_m=2.04; s.max_width_m=2.8; s.max_height_m=2.8; s.base_diameter_m=2.8;
    % Approximate AEDB20 curves, A3 Figs29-30, M10-26. These are graphical
    % estimates, NOT a transcribed general aerodynamic database.
    s.ca=1.36; s.cn=-.07;
    s.cd=s.ca*cosd(20)-s.cn*sind(20);
    s.nominal_ld=(s.ca*sind(20)+s.cn*cosd(20))/s.cd;
    s.default_aoa_deg=20; s.valid_mach=[10 26]; s.valid_alpha_deg=[15 25];
    s.aero_model="ARD_FIXED_BODY_HYPERSONIC";
    s.reference_aoa_profile=reference_vehicle.aoa_profile("ARD");
    s.angular_extension="CONSTANT_CA_CN_ROTATED_WITH_AOA_SURROGATE_NOT_MEASURED_AOA_DEPENDENCE";
    s.provenance="A3 pp10-2/3/4; Figs29/30 approximate AEDB20 CA/CN; SURROGATE_ONLY";
    sys.reentry_vehicle.shapes.CAPSULE=s;
    c=sys.reentry_vehicle.capsule;
    c.mass_kg=2800; c.aero_model=s.aero_model; c.cd=s.cd; c.nominal_ld=s.nominal_ld;
    c.trim_aoa_deg=20; c.reference_entry_speed_m_s=7451.65;
    c.reference_entry_fpa_deg=-2.6; c.entry_interface_altitude_m=120e3;
    c.parachute_deploy_speed_m_s=NaN; % hypersonic validity ends before DRS
    c.paper_capsule_mass_kg=NaN; c.paper_entry_condition_scope="ARD_SURROGATE_ONLY";
    c.reference_total_heat_load_J_m2=NaN;
    c.paper_entry_altitude_range_m=[120e3 120e3];
    c.paper_entry_fpa_range_deg=[-2.6 -2.6]; c.paper_entry_range_to_go_m=[];
    c.paper_heating_model="UNAVAILABLE_ARD_TPS_INVERSION_NOT_IMPLEMENTED";
    c.constraints=struct('max_dynamic_pressure_Pa',Inf,'max_g_load',Inf,'max_heat_flux_W_m2',Inf);
    c.use_paper_entry_conditions=false;
    sys.reentry_vehicle.capsule=c;
    sys.reference.entry_mass_kg=2800;
    sys.reference.entry=struct('altitude_m',120e3,'air_speed_m_s',7451.65,'air_fpa_deg',-2.6);
    sys.reentry_flight_path_angle=deg2rad(2.6);
elseif preset=="APOLLO7_PREFLIGHT_TRIM"
    d=reference_vehicle.apollo7_data();
    sys.reference.revision="2026-09-21-apollo7-v1";
    sys.reference.provenance="docs/APOLLO7_MIGRATION.md";
    sys.reentry_vehicle.vehicle_mode="CAPSULE";
    s=struct('name',preset,'reference_area_m2',129.4*.3048^2, ...
        'nose_radius_m',4.694,'length_m',NaN,'max_width_m',154*.0254, ...
        'max_height_m',154*.0254,'base_diameter_m',154*.0254, ...
        'aero_model',preset,'cd',d.cd(end),'nominal_ld',d.cl(end)/d.cd(end), ...
        'default_aoa_deg',d.alpha_deg(end),'valid_mach',[.4 27.72], ...
        'valid_alpha_deg',[min(d.alpha_deg) max(d.alpha_deg)], ...
        'provenance',d.source,'aoa_convention',"180_MINUS_PUBLISHED_APOLLO_BODY_ALPHA", ...
        'geometry_provenance',"Block II nose: NASA 20070025192 p3; area/diameter: SNA-8-D-027(I) Rev2 Table6-1 p6-5", ...
        'reference_aoa_profile',reference_vehicle.aoa_profile(preset));
    sys.reentry_vehicle.shapes.CAPSULE=s;
    c=sys.reentry_vehicle.capsule;
    c.mass_kg=d.mass_kg; c.aero_model=preset; c.cd=s.cd; c.nominal_ld=s.nominal_ld;
    c.trim_aoa_deg=d.alpha_deg(end);
    % Table Ia is inertial/geodetic, not an air-relative spherical constructor.
    c.reference_entry_speed_m_s=NaN; c.reference_entry_fpa_deg=NaN;
    c.entry_interface_altitude_m=121920;
    c.altitude_termination_enabled=true;
    c.parachute_deploy_speed_m_s=NaN;
    c.paper_entry_condition_scope="APOLLO7_TABLE_IA_INERTIAL_NOT_FORCED";
    c.paper_capsule_mass_kg=NaN; c.reference_total_heat_load_J_m2=NaN;
    c.paper_spacecraft_total_mass_kg=NaN; c.paper_satellite_reference_area_m2=NaN;
    c.paper_satellite_cd=NaN; c.paper_capsule_inertia_kg_m2=NaN(3);
    c.paper_entry_cases=[]; c.paper_target_latitude_deg=NaN; c.paper_target_longitude_deg=NaN;
    c.density_uncertainty_fraction=NaN; c.explicit_cd_uncertainty_fraction=NaN;
    c.cd_uncertainty_fraction=NaN; c.ld_recession_uncertainty_fraction=NaN;
    c.uncertainty_provenance="UNSPECIFIED_FOR_APOLLO7_USER_SCALES_REMAIN_EXPLICIT";
    c.paper_entry_altitude_range_m=[]; c.paper_entry_fpa_range_deg=[];
    c.paper_entry_range_to_go_m=[];
    c.paper_heating_model="SUTTON_GRAVES_SURROGATE_NOT_APOLLO_TPS";
    c.constraints=struct('max_dynamic_pressure_Pa',Inf,'max_g_load',Inf,'max_heat_flux_W_m2',Inf);
    c.guidance_activation_drag_g=Inf; c.guidance_cutoff_mach=.4;
    c.ld_bounds=[min(d.cl./d.cd) max(d.cl./d.cd)];
    sys.reentry_vehicle.capsule=c;
    sys.reference.entry_mass_kg=d.mass_kg;
    sys.reference.mass_provenance=d.mass_source;
    sys.reference.mission_mass_policy="EXISTING_PROJECT_CARRIER_PLUS_SEPARATED_REFERENCE_CAPSULE";
    sys.reference.entry=struct('source',"MSC 69-FM-89 Table Ia BET; metadata only", ...
        'geodetic_latitude_deg',29.926,'longitude_deg',-92.444, ...
        'reported_altitude_m',397802.1*.3048,'inertial_speed_m_s',25848.512*.3048, ...
        'inertial_fpa_deg',-2.055,'inertial_azimuth_deg',87.552, ...
        'ground_elapsed_time_s',935608,'interface_policy',"PROJECT_SPHERICAL_400000_FT");
    sys.h_entry_interface=121920;
    sys.reentry_vehicle.terminal_altitude=7620;
    sys.reference.terminal_policy="PROJECT_PRE_PARACHUTE_25000_FT_NO_PARACHUTE_OR_TOUCHDOWN_MODEL";
    sys.reentry_flight_path_angle=deg2rad(2.055); % targeting request, never a state reset
else
    sys.reentry_vehicle.vehicle_mode="SPACEPLANE";
    sys.reentry_vehicle.selected_shape="HORUS_2B";
    s=struct('name',"HORUS_2B_CLEAN_UNTRIMMED",'length_m',25,'max_width_m',13, ...
        'max_height_m',4.5,'base_diameter_m',NaN,'reference_area_m2',110, ...
        'nose_radius_m',.8,'cd',.7,'default_aoa_deg',40, ...
        'aero_model',"HORUS_CLEAN_TABLE",'valid_mach',[1.2 20], ...
        'valid_alpha_deg',[0 45],'provenance',"H1 clean tables; H2 nose radius; no trim correction");
    sys.reentry_vehicle.shapes.HORUS_2B=s;
    sys.reentry_vehicle.shapes.HORUS_2B.reference_aoa_profile=reference_vehicle.aoa_profile("HORUS_2B");
    sys.reentry_vehicle.spaceplane.aero_model=s.aero_model;
    sys.reentry_vehicle.spaceplane.aoa_profile_mode="CONSTANT";
    sys.reentry_vehicle.spaceplane.communication.enabled=false;
    sys.reentry_vehicle.spaceplane.paper_initial_scenarios=[];
    sys.reentry_vehicle.spaceplane.paper_heating_model="UNAVAILABLE_HORUS_THERMAL_REPRODUCTION";
    sys.reference.entry_mass_kg=26029;
    % This starting wet mass is explicitly a mission scenario assumption.
    % Integrated entry retains remaining propagated mass, never restores 26029.
    sys.Chaser_Mass_Init=26029;
    sys.reference.mission_mass_policy="REFERENCE_ENTRY_MASS_USED_AS_INITIAL_WET_MASS_ASSUMPTION";
    sys.reference.entry=struct('altitude_m',122e3,'air_speed_m_s',7435.5, ...
        'air_fpa_deg',-1.43,'latitude_deg',-22.3,'longitude_deg',-106.7,'heading_deg',70.75);
    sys.h_entry_interface=122e3; sys.reentry_flight_path_angle=deg2rad(1.43);
end
sys.reference.validity_policy="ERROR_OUTSIDE_DOMAIN_NO_EXTRAPOLATION";
sys.reentry_vehicle.reference_aoa_profile=reference_vehicle.aoa_profile(preset);
sys.reference.heating="SUTTON_GRAVES_SURROGATE_NOT_FLIGHT_VALIDATED";
end
