function [cd, cl, ld] = aerodynamic_coefficients(shape, aoa_deg, mach)
    aero_model = upper(string(get_field(shape, 'aero_model', "LEGACY_SHAPE_LD")));

    if aero_model=="APOLLO7_PREFLIGHT_TRIM"
        data=reference_vehicle.apollo7_data();
        if ~isscalar(mach) || ~isfinite(mach) || mach<data.mach(1) || mach>data.mach(end)
            error('reference_vehicle:Domain','Apollo 7 trim requires Mach 0.40--27.72.');
        end
        trim=interp1(data.mach,data.alpha_deg,mach,'linear');
        if ~isscalar(aoa_deg) || ~isfinite(aoa_deg) || abs(aoa_deg-trim)>1e-8
            error('reference_vehicle:TrimOnly','Apollo 7 coefficients apply only at the published trim schedule.');
        end
        cd=interp1(data.mach,data.cd,mach,'linear');
        cl=interp1(data.mach,data.cl,mach,'linear');
        ld=cl/cd;
    elseif any(aero_model==["HORUS_CLEAN_TABLE","ARD_FIXED_HYPERSONIC","ARD_FIXED_BODY_HYPERSONIC"])
        if ~isfinite(mach) || ~isfinite(aoa_deg) || ...
                mach<shape.valid_mach(1) || mach>shape.valid_mach(2) || ...
                aoa_deg<shape.valid_alpha_deg(1) || aoa_deg>shape.valid_alpha_deg(2)
            error('reference_vehicle:Domain','%s outside declared Mach/AoA domain (M=%g, alpha=%g deg).',aero_model,mach,aoa_deg);
        end
        if aero_model=="HORUS_CLEAN_TABLE"
            data=reference_vehicle.horus_data();
            % Use only nonzero interpolation weights: missing neighbours must
            % not contaminate an exactly published grid point.
            wm=weights(data.mach,mach); wa=weights(data.alpha_deg,aoa_deg);
            w=wa(:)*wm(:)'; active=w>0;
            if any(~isfinite(data.cd(active))) || any(~isfinite(data.cl(active)))
                error('reference_vehicle:MissingData','HORUS interpolation touches an unpublished cell.');
            end
            cd=sum(w(active).*data.cd(active)); cl=sum(w(active).*data.cl(active));
            ld=cl/cd;
        elseif aero_model=="ARD_FIXED_BODY_HYPERSONIC"
            cd=shape.ca*cosd(aoa_deg)-shape.cn*sind(aoa_deg);
            cl=shape.ca*sind(aoa_deg)+shape.cn*cosd(aoa_deg);
            ld=cl/cd;
        else
            cd=shape.cd; ld=shape.nominal_ld;
        end
    elseif aero_model == "PAPER_RLV_POLYNOMIAL"
        cl_coeff = get_field(shape, 'cl_polynomial', [-0.041065, 0.016292, 0.0002602]);
        cd_coeff = get_field(shape, 'cd_from_cl_polynomial', [0.080505, -0.03026, 0.86495]);
        cl = cl_coeff(1) + cl_coeff(2)*aoa_deg + cl_coeff(3)*aoa_deg^2;
        cd = cd_coeff(1) + cd_coeff(2)*cl + cd_coeff(3)*cl^2;
        cd = max(cd, 1e-6);
        ld = cl / cd;
    elseif aero_model == "PAPER_CAPSULE_REDUCED"
        cd = get_field(shape, 'cd', 1.3);
        ld = get_field(shape, 'nominal_ld', 0.25);
    else
        cd = get_field(shape, 'cd', 1.2);
        ld = lookup_ld(shape, aoa_deg, mach);
    end

    cd = cd * get_field(shape, 'cd_scale', 1.0);
    ld = ld * get_field(shape, 'ld_scale', 1.0);
    cl = cd * ld;
end

function w=weights(grid,x)
w=zeros(size(grid)); k=find(grid==x,1);
if ~isempty(k), w(k)=1; return; end
k=find(grid<x,1,'last'); f=(x-grid(k))/(grid(k+1)-grid(k));
w(k)=1-f; w(k+1)=f;
end
