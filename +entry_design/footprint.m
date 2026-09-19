function out = footprint(sys,vehicle,initial_state,overrides)
%FOOTPRINT Sampled reachable endpoints. Hull is visualization, not feasibility proof.
    if nargin<4, overrides=struct(); end
    c=mission.merge_settings(entry_design.defaults(),overrides);
    profiles=c.bank_profiles_deg;
    validateattributes(profiles,{'numeric'},{'2d','nonempty','finite','real'});
    if size(profiles,2)~=numel(c.speed_fractions)
        error('entry_design:Profiles','Each bank profile must have one value per speed knot.');
    end
    trajectories=cell(size(profiles,1),1);
    for k=1:numel(trajectories)
        trajectories{k}=entry_design.propagate(sys,vehicle,initial_state,profiles(k,:),c);
    end
    feasible=cellfun(@(t) t.feasible,trajectories);
    all_xy=cell2mat(cellfun(@(t) t.range_m,trajectories,'UniformOutput',false));
    latlon=cell2mat(cellfun(@(t) t.latlon_deg,trajectories,'UniformOutput',false));
    points=all_xy(feasible,:); hull=[];
    if size(points,1)>=3 && rank(points-mean(points,1))==2
        hull=convhull(points(:,1),points(:,2));
    end
    out=struct('vehicle',vehicle,'config',c,'initial_state',initial_state(:), ...
        'trajectories',{trajectories},'feasible',feasible,'range_m',all_xy, ...
        'latlon_deg',latlon,'visual_hull_indices',hull,'visual_hull_points_m',points, ...
        'interpretation',"SAMPLED_REACHABLE_ENDPOINTS_NOT_COMPLETE_BOUNDARY", ...
        'endpoint',"ALTITUDE_SURFACE");
    if c.terminal_altitude_m==0
        out.endpoint="GROUND_INTERSECTION_SURROGATE_NO_LANDING_DYNAMICS";
    end
end
