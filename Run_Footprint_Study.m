function results = Run_Footprint_Study(output_dir)
%RUN_FOOTPRINT_STUDY Example only: shared entry conditions for two vehicle models.
% Change vehicle structs/config independently; not calibrated mission predictions.
    root=fileparts(mfilename('fullpath'));
    if nargin<1, output_dir=fullfile(root,'output','footprint'); end
    if ~isfolder(output_dir), mkdir(output_dir); end
    sys=Mission_Config(); c=entry_design.defaults();
    initial=entry_design.initial_state(sys,[0 0],120e3,7500,-3,90,c);
    modes=["SPACEPLANE" "CAPSULE"]; results=struct();
    fig=figure('Visible','off','Color','w','Position',[100 100 1100 450]);
    cleanup=onCleanup(@() close(fig)); tiledlayout(1,2);
    summary=struct();
    for k=1:2
        vehicle=entry_design.vehicle(sys,modes(k));
        fp=entry_design.footprint(sys,vehicle,initial,c);
        results.(modes(k))=fp;
        nexttile; hold on;
        if ~isempty(fp.visual_hull_indices)
            pts=fp.visual_hull_points_m(fp.visual_hull_indices,:)/1000;
            plot(pts(:,1),pts(:,2),'--','Color',[.6 .6 .6]);
        end
        scatter(fp.range_m(fp.feasible,1)/1000,fp.range_m(fp.feasible,2)/1000,40,'filled');
        if any(~fp.feasible)
            scatter(fp.range_m(~fp.feasible,1)/1000,fp.range_m(~fp.feasible,2)/1000,40,'x');
        end
        grid on; xlabel('Downrange (km)'); ylabel('Crossrange (km)');
        title(modes(k)+" — 20 km endpoint");
        summary.(modes(k))=struct('mass_kg',vehicle.mass_kg,'feasible_count',sum(fp.feasible), ...
            'range_m',fp.range_m,'latlon_deg',fp.latlon_deg,'feasible',fp.feasible);
    end
    sgtitle('Sampled reachable endpoints; dashed hull is not a reachability certificate');
    points=[results.SPACEPLANE.range_m(results.SPACEPLANE.feasible,:); ...
        results.CAPSULE.range_m(results.CAPSULE.feasible,:)]/1000;
    if ~isempty(points)
        axes_list=findall(fig,'Type','axes');
        for k=1:numel(axes_list)
            xlim(axes_list(k),[min(points(:,1))-50 max(points(:,1))+50]);
            ylim(axes_list(k),[min(points(:,2))-50 max(points(:,2))+50]);
        end
    end
    exportgraphics(fig,fullfile(output_dir,'footprints.png'),'Resolution',160);
    summary.config=c; summary.initial_state_eci=initial;
    summary.endpoint="20 km altitude; NOT touchdown";
    summary.path_limits="Unbounded study envelope; apply validated thermal/load limits before mission use";
    file=fopen(fullfile(output_dir,'footprint_results.json'),'w');
    if file<0, error('entry_design:Output','Cannot create footprint output.'); end
    file_cleanup=onCleanup(@() fclose(file));
    fprintf(file,'%s',jsonencode(summary,PrettyPrint=true));
    disp(summary);
end
