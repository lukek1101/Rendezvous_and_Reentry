function y = proximity_step(y, force_eci, duration, dt, sys)
%PROXIMITY_STEP RK4 substeps with a held physical force (not acceleration).
    elapsed = 0;
    while elapsed < duration
        h = min(dt,duration-elapsed);
        k1 = mission.proximity_dynamics(0,y,force_eci,sys);
        k2 = mission.proximity_dynamics(0,y+h*k1/2,force_eci,sys);
        k3 = mission.proximity_dynamics(0,y+h*k2/2,force_eci,sys);
        k4 = mission.proximity_dynamics(0,y+h*k3,force_eci,sys);
        y = y+h*(k1+2*k2+2*k3+k4)/6;
        elapsed = elapsed+h;
    end
end
