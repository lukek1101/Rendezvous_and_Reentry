function result = target(sys,vehicle,initial_state,target_latlon_deg,overrides,refine)
%TARGET A propagated witness establishes reachability within a stated tolerance.
% Failure to find one is UNRESOLVED, not a mathematical unreachable certificate.
    if nargin<6, refine=true; end
    validateattributes(target_latlon_deg,{'numeric'},{'vector','numel',2,'finite'});
    if abs(target_latlon_deg(1))>90, error('entry_design:Latitude','Invalid latitude.'); end
    fp=entry_design.footprint(sys,vehicle,initial_state,overrides); c=fp.config;
    costs=inf(numel(fp.trajectories),1);
    for k=1:numel(costs), costs(k)=score(fp.trajectories{k}); end
    [best,index]=min(costs); trajectory=fp.trajectories{index};
    evaluations=0;
    if refine && isfinite(best) && best>c.target_tolerance_m
        % Bounded differential correction avoids a saturated tanh seed at bank limits.
        goal=[cosd(target_latlon_deg(1))*cosd(target_latlon_deg(2)); ...
            cosd(target_latlon_deg(1))*sind(target_latlon_deg(2));sind(target_latlon_deg(1))];
        east=[-sind(target_latlon_deg(2));cosd(target_latlon_deg(2));0];
        tangent=[east';cross(goal,east)'];
        for iteration=1:c.target_max_iterations
            profile=trajectory.bank_profile_deg;
            if best<=c.target_tolerance_m || evaluations+numel(profile)+1>c.target_max_evaluations
                break;
            end
            residual=endpoint_error(trajectory); jac=zeros(2,numel(profile));
            for j=1:numel(profile)
                delta=min(1,c.bank_limit_deg/10);
                if profile(j)+delta>c.bank_limit_deg, delta=-delta; end
                trial=profile; trial(j)=trial(j)+delta;
                probe=entry_design.propagate(sys,vehicle,initial_state,trial,c);
                evaluations=evaluations+1;
                jac(:,j)=(endpoint_error(probe)-residual)/delta;
            end
            step=-pinv(jac,1e-4)*residual;
            step=step*min(1,15/max(max(abs(step)),eps));
            improved=false;
            for factor=[1 .5 .25 .125]
                if evaluations>=c.target_max_evaluations, break; end
                trial=max(-c.bank_limit_deg,min(c.bank_limit_deg,profile+factor*step'));
                candidate=entry_design.propagate(sys,vehicle,initial_state,trial,c);
                evaluations=evaluations+1; candidate_cost=score(candidate);
                if candidate_cost<best
                    best=candidate_cost; trajectory=candidate; improved=true; break;
                end
            end
            if ~improved, break; end
        end
    end
    status="UNRESOLVED_BY_SEARCH";
    if best<=c.target_tolerance_m, status="REACHABLE_WITNESS"; end
    result=struct('status',status,'miss_distance_m',best,'target_latlon_deg',target_latlon_deg, ...
        'tolerance_m',c.target_tolerance_m,'trajectory',trajectory,'config',c, ...
        'refinement_evaluations',evaluations,'search_method',"bounded endpoint differential correction");
    function cost=score(t)
        u=[cosd(target_latlon_deg(1))*cosd(target_latlon_deg(2)); ...
           cosd(target_latlon_deg(1))*sind(target_latlon_deg(2));sind(target_latlon_deg(1))];
        v=t.final_ecef_m/norm(t.final_ecef_m);
        cost=sys.Re*atan2(norm(cross(u,v)),dot(u,v));
        if ~t.feasible, cost=inf; end
    end
    function residual=endpoint_error(t)
        residual=sys.Re*tangent*(t.final_ecef_m/norm(t.final_ecef_m)-goal);
    end
end
