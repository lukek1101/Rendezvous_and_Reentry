function c=physics_contract(sys,run)
%PHYSICS_CONTRACT Conservative, explicit compatibility key for offline designs.
c=sys.contract;
c.mu_m3_s2=sys.mu; c.earth_radius_m=sys.Re; c.J2=sys.J2; c.g0_m_s2=sys.g0;
c.inclination_rad=sys.inc;
c.insertion_altitude_m=sys.h_insert; c.target_altitude_m=sys.h_target;
c.initial_chaser_angle_deg=sys.initial_chaser_angle_deg;
c.initial_phase_angle_deg=sys.initial_phase_angle_deg;
c.base_chaser_wet_mass_kg=sys.Chaser_Mass_Init; c.target_mass_kg=sys.Target_Mass;
c.initial_stack_mass_kg=sys.Chaser_Mass_Init;
if sys.reentry_vehicle.vehicle_mode=="CAPSULE" && sys.reentry_vehicle.capsule.add_to_chaser_initial_mass
    c.initial_stack_mass_kg=c.initial_stack_mass_kg+sys.reentry_vehicle.capsule.mass_kg;
end
c.impulse_isp_s=sys.Isp; c.finite_thrust_N=sys.maneuver.finite_burn_thrust;
c.maneuver=sys.maneuver;
c.use_thrust_noise=run.maneuver.use_thrust_noise;
c.phase1_mode=run.phase1.mode;
c.finite_isp_s=sys.maneuver.finite_burn_isp; c.burn_model=sys.maneuver.default_burn_model;
c.atmospheric_drag=sys.environment.atmospheric_drag;
c.drag_scope=run.environment.atmospheric_drag.apply_from_phase;
c.vehicle_preset=sys.reference.preset; c.vehicle_revision=sys.reference.revision;
c.vehicle_properties=sys.reentry_vehicle.shapes;
c.entry_model_settings=sys.reentry_vehicle;
c.physics_source_sha256=struct( ...
    'environment_eom',mission.file_sha256(fullfile(mission.project_root(),'Env_EOM.m')), ...
    'atmosphere',mission.file_sha256(fullfile(mission.project_root(),'Standard_Atmosphere_Density.m')), ...
    'reference_data',mission.file_sha256(fullfile(mission.project_root(),'+reference_vehicle','horus_data.m')), ...
    'reference_presets',mission.file_sha256(fullfile(mission.project_root(),'+reference_vehicle','apply.m')));
c.physics_source_sha256.entry_aerodynamics=mission.file_sha256(fullfile(mission.project_root(),'+reentry_core','aerodynamic_coefficients.m'));
c.physics_source_sha256.entry_commands=mission.file_sha256(fullfile(mission.project_root(),'+reentry_core','resolve_commands.m'));
c.physics_source_sha256.profile_resolution=mission.file_sha256(fullfile(mission.project_root(),'+reference_vehicle','resolve_profile.m'));
c.physics_source_sha256.apollo7_data=mission.file_sha256(fullfile(mission.project_root(),'+reference_vehicle','apollo7_data.m'));
c.earth_angle_at_epoch_deg=sys.reentry_vehicle.spaceplane.communication.earth_fixed_to_eci_angle_at_mission_epoch_deg;
c.desired_relative_position_m=run.phase1.desired_rel_lvlh_m(:);
if string(run.phase1.mode)=="HOHMANN"
    c.hohmann_method=run.phase1.hohmann_method;
    c.nominal_targeting=run.phase1.nominal;
    c.nominal_correction=run.phase1.correction;
end
end
