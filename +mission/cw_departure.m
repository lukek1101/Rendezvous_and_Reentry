function dv = cw_departure(r, v, goal, duration, n)
%CW_DEPARTURE Linear targeting seed; nonlinear execution needs correction.
    s = sin(n*duration); c = cos(n*duration);
    rr = [4-3*c 0 0; 6*(s-n*duration) 1 0; 0 0 c];
    rv = [s 2*(1-c) 0; -2*(1-c) 4*s-3*n*duration 0; 0 0 s]/n;
    if rcond(rv) < 1e-9
        error('mission:SingularTransfer','Transfer time is near a CW singularity.');
    end
    dv = rv\(goal-rr*r)-v;
end
