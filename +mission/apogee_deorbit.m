function [Xc,Xt,dv,fuel,hist,info] = apogee_deorbit(sys,Xc,Xt,a)
%APOGEE_DEORBIT Finite retrograde firings at successive apogee starts.
% Not a retargeted FPA solution. Actual entry state is the downstream boundary.
    mission.validate_apogee_burns(a);
    y=[Xc(1:6);Xt(1:6);Xc(14)]; m0=y(13); elapsed=0;
    if m0<=a.minimum_mass_kg || norm(y(1:3))-sys.Re<=sys.h_entry_interface
        error('mission:DeorbitInitialState','Insufficient initial mass or altitude for deorbit.');
    end
    states=y; times=0; modes="INITIAL";
    burns=struct('start_s',{},'duration_s',{},'delta_v_m_s',{}, ...
        'start_radial_speed_m_s',{},'start_altitude_m',{});
    parts=a.total_delta_v_m_s*a.fractions/sum(a.fractions);
    part=1; remainder=parts(1); count=0; coast_end=-inf;
    status="BUDGET_EXHAUSTED_NO_ENTRY";
    if a.first_burn=="NEXT_APOGEE"
        [found,entered]=advance(a.max_elapsed_s,0,"APOGEE");
        if entered, error('mission:EarlyEntry','Entry before the first apogee burn.'); end
        if ~found, error('mission:NoApogee','No distinct apogee found; circular states should use CURRENT.'); end
    end
    while part<=numel(parts)
        if count>=a.max_burns
            error('mission:BurnLimit','Maximum deorbit burn count exceeded.');
        end
        if count>0
            % Coast through cooldown, then detect the next descending r-dot zero.
            if a.cooldown_s>0
                [~,entered]=advance(min(a.cooldown_s,a.max_elapsed_s-elapsed),0,"COAST");
                if entered, error('mission:EarlyEntry','Entry during cooldown before planned burns completed.'); end
            end
            [found,entered]=advance(a.max_elapsed_s-elapsed,0,"APOGEE");
            if entered, error('mission:EarlyEntry','Entry before the next planned apogee; revise burn allocation.'); end
            if ~found, error('mission:NoApogee','No apogee within the configured horizon.'); end
            if elapsed-coast_end<a.cooldown_s-1e-6
                error('mission:Cooldown','Insufficient cooldown.');
            end
        end
        duration_needed=y(13)*sys.g0*a.isp_s/a.thrust_N* ...
            (1-exp(-remainder/(sys.g0*a.isp_s)));
        duration=min(duration_needed,a.max_burn_duration_s);
        count=count+1; start=elapsed; mass_start=y(13);
        radial=dot(y(1:3),y(4:6))/norm(y(1:3));
        altitude=norm(y(1:3))-sys.Re;
        [~,entered]=advance(duration,a.thrust_N,"BURN");
        delivered=sys.g0*a.isp_s*log(mass_start/y(13));
        burns(count)=struct('start_s',start,'duration_s',elapsed-start, ...
            'delta_v_m_s',delivered,'start_radial_speed_m_s',radial,'start_altitude_m',altitude);
        fprintf('   Apogee schedule burn %d: start %.1f s, duration %.2f s, dV %.3f m/s\n', ...
            count,start,elapsed-start,delivered);
        remainder=max(0,remainder-delivered); coast_end=elapsed;
        if entered
            status="ENTRY_DURING_BURN";
            break;
        end
        if remainder<1e-7
            part=part+1;
            if part<=numel(parts), remainder=parts(part); end
        end
    end
    if status~="ENTRY_DURING_BURN"
        [~,entered]=advance(a.max_elapsed_s-elapsed,0,"COAST");
        if ~entered, error('mission:NoEntry','Burn schedule did not reach entry within the horizon.'); end
        status="ENTRY_AFTER_BURNS";
    end
    Xc(1:6)=y(1:6); Xc(14)=y(13); Xt=y(7:12);
    dv=sys.g0*a.isp_s*log(m0/y(13)); fuel=m0-y(13);
    count_samples=numel(times); rel=zeros(3,count_samples); vr=rel;
    for k=1:count_samples
        [rel(:,k),vr(:,k)]=orbit_core.relative_state(states(1:6,k),states(7:12,k));
    end
    hist=struct('pos',states(1:3,:),'vel',states(4:6,:),'mass',states(13,:), ...
        'time',times,'time_end',elapsed,'rel_pos_lvlh',rel,'rel_vel_lvlh',vr,'mode',modes);
    hist.target_pos=states(7:9,:); hist.target_vel=states(10:12,:);
    hist.rel_pos=states(1:3,:)-states(7:9,:);
    info=struct('mode',"APOGEE_SPLIT_FINITE",'burns',burns,'status',status, ...
        'commanded_delta_v_m_s',a.total_delta_v_m_s,'delivered_delta_v_m_s',dv, ...
        'entry_fpa_deg',reentry_core.flight_path_angle_deg(y(1:3),y(4:6)), ...
        'entry_altitude_m',norm(y(1:3))-sys.Re,'duration_s',elapsed, ...
        'entry_injection_dV',dv,'entry_injection_fuel',fuel,'reentry_coast_time',elapsed-coast_end);

    function [apogee,entered]=advance(duration,thrust,mode)
        if duration<=0 || elapsed+duration>a.max_elapsed_s+1e-6
            error('mission:DeorbitTimeout','No remaining time for the burn schedule.');
        end
        segment_start=elapsed;
        opts=odeset('RelTol',1e-10,'AbsTol',1e-8,'MaxStep',a.max_step_s,'Events',@events);
        [tt,yy,~,~,ie]=ode45(@rhs,[elapsed elapsed+duration],y,opts);
        elapsed=tt(end); y=yy(end,:)';
        states=[states yy(2:end,:)']; times=[times tt(2:end)'];
        modes=[modes repmat(mode,1,numel(tt)-1)];
        apogee=any(ie==2); entered=any(ie==1);
        if any(ie==3), error('mission:FuelLimit','Configured minimum mass reached.'); end
        function dy=rhs(~,s)
            force=-thrust*s(4:6)/norm(s(4:6));
            local_sys=sys; local_sys.Isp=a.isp_s;
            dy=mission.proximity_dynamics(0,s,force,local_sys);
        end
        function [value,terminal,direction]=events(t,s)
            radial_speed=dot(s(1:3),s(4:6))/norm(s(1:3));
            if mode~="APOGEE" || t<=segment_start+1
                radial_speed=-1;
            end
            value=[norm(s(1:3))-sys.Re-sys.h_entry_interface;radial_speed;s(13)-a.minimum_mass_kg];
            terminal=[1;1;1]; direction=[-1;-1;-1];
        end
    end
end
