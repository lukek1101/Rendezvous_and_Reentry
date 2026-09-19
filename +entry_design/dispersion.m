function result = dispersion(sys,vehicle,initial_state,bank_profile,entry_overrides,uncertainty)
%DISPERSION Fixed-policy terminal dispersion, separate from controllable footprint.
% This is not an IMU/GNSS filter simulation. Initial covariance is in ECI SI units.
    defaults=struct('trials',20,'seed',42,'initial_covariance',zeros(6), ...
        'density_coefficient_of_variation',0,'cd_coefficient_of_variation',0, ...
        'ld_coefficient_of_variation',0);
    u=mission.merge_settings(defaults,uncertainty);
    validateattributes(u.trials,{'numeric'},{'scalar','integer','positive'});
    validateattributes(u.seed,{'numeric'},{'scalar','integer','nonnegative','<=',2^32-1});
    validateattributes(u.initial_covariance,{'numeric'},{'size',[6 6],'finite','real'});
    if norm(u.initial_covariance-u.initial_covariance','fro')>1e-9
        error('entry_design:Covariance','Initial covariance must be symmetric.');
    end
    [vectors,eigenvalues]=eig(u.initial_covariance,'vector');
    if min(eigenvalues)<-1e-10, error('entry_design:Covariance','Covariance must be positive semidefinite.'); end
    root=vectors*diag(sqrt(max(0,eigenvalues)));
    cv=[u.density_coefficient_of_variation u.cd_coefficient_of_variation u.ld_coefficient_of_variation];
    validateattributes(cv,{'numeric'},{'finite','real','nonnegative'});
    caller_rng=rng; cleanup=onCleanup(@() rng(caller_rng)); rng(u.seed,'twister');
    c=mission.merge_settings(entry_design.defaults(),entry_overrides);
    latlon=zeros(u.trials,2); feasible=false(u.trials,1); perturbations=zeros(6,u.trials);
    scales=zeros(u.trials,3); sigma=sqrt(log(1+cv.^2));
    for k=1:u.trials
        perturbations(:,k)=root*randn(6,1);
        scales(k,:)=exp(sigma.*randn(1,3)-sigma.^2/2);
        trial=vehicle; names={'density_scale','cd_scale','ld_scale'};
        for j=1:3
            base=1;
            if isfield(trial.shape,names{j}), base=trial.shape.(names{j}); end
            trial.shape.(names{j})=base*scales(k,j);
        end
        sample=entry_design.propagate(sys,trial,initial_state(:)+perturbations(:,k),bank_profile,c);
        latlon(k,:)=sample.latlon_deg; feasible(k)=sample.feasible;
    end
    result=struct('config',u,'entry_config',c,'latlon_deg',latlon,'feasible',feasible, ...
        'initial_perturbations',perturbations,'parameter_scales',scales, ...
        'interpretation',"FIXED_POLICY_DISPERSION_NOT_REACHABLE_FOOTPRINT_OR_NAVIGATION_FILTER");
end
