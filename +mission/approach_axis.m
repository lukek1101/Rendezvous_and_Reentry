function axis=approach_axis(mode)
% Target LVLH: +R outward, +V forward transverse, +H orbit normal.
% Mode names identify the side occupied by the chaser, not closing velocity.
switch string(mode)
    case "+R", axis=[1;0;0];
    case "-R", axis=[-1;0;0];
    case "+V", axis=[0;1;0];
    case "-V", axis=[0;-1;0];
    otherwise, error('mission:ApproachMode','Use +R, -R, +V or -V.');
end
end
