function d=apollo7_data()
%APOLLO7_DATA MSC 69-FM-89, printed p.21, Table IIb (preflight trim).
% Force coefficients are already wind-axis CL/CD, not CA/CN.
% Preserve source Apollo body-axis alpha; simulator uses its supplement.
d.mach=[.40 .70 1.10 1.20 1.35 1.65 2 2.40 3 4 10 27.72];
d.cl=[.24399 .26368 .49540 .48008 .56442 .55160 .53387 .50892 .48036 .44293 .42994 .39208];
d.cd=[.8531 .9852 1.1684 1.1548 1.2776 1.2641 1.2689 1.2378 1.2131 1.2124 1.2221 1.2813];
d.apollo_alpha_deg=[167.17 164.53 154.76 155.03 153.92 153.09 152.97 153.45 153.97 155.99 156.67 159.66];
d.alpha_deg=180-d.apollo_alpha_deg;
d.published_ld=[.2860 .2676 .4240 .4157 .4418 .4364 .4207 .4111 .3960 .3653 .3518 .3060];
d.source="MSC 69-FM-89 Table IIb p21; PREFLIGHT_TRIM; linear Mach interpolation";
d.mass_kg=12364.1*.45359237;
d.mass_source="MSC 69-FM-89 p8 post-separation weight 12364.1 lb; distinct from preflight force data";
end
