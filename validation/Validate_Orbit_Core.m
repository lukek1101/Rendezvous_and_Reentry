function results = Validate_Orbit_Core()
%VALIDATE_ORBIT_CORE Check gravity against potential and LVLH frame invariants.
    sys = Mission_Config();
    r = [6.5e6;1.1e6;2.2e6];
    gradient = zeros(3,1);
    for k = 1:3
        offset = zeros(3,1);
        offset(k) = 1; % One-metre central finite difference of the potential.
        gradient(k) = (potential(r+offset,sys)-potential(r-offset,sys))/2;
    end
    results.gravity_error = norm(orbit_core.gravity_j2(r,sys)+gradient);
    assert(results.gravity_error < 5e-8, 'J2 acceleration differs from potential gradient.');
    central = sys;
    central.J2 = 0;
    assert(norm(orbit_core.gravity_j2(r,central)+sys.mu*r/norm(r)^3) < 1e-14);

    target = [7e6;0;0;0;7500;0];
    rho = [30;-5000;100];
    omega = [0;0;7500/7e6];
    chaser = target + [rho;cross(omega,rho)];
    [position,velocity,basis] = orbit_core.relative_state(chaser,target);
    assert(norm(position-rho)<1e-12 && norm(velocity)<1e-12);
    assert(norm(basis*basis'-eye(3))<1e-14 && det(basis)>0);
    % An arbitrary rigid rotation must preserve target-frame coordinates.
    axis = [1;2;3]/sqrt(14);
    K = [0 -axis(3) axis(2);axis(3) 0 -axis(1);-axis(2) axis(1) 0];
    Q = eye(3)+sin(0.7)*K+(1-cos(0.7))*K*K;
    [p2,v2] = orbit_core.relative_state([Q*chaser(1:3);Q*chaser(4:6)], ...
        [Q*target(1:3);Q*target(4:6)]);
    assert(norm(p2-position)<1e-8 && norm(v2-velocity)<1e-8);
    results.passed = true;
    fprintf('Orbit core: PASS (potential gradient and rotated LVLH frame)\n');
end

function value = potential(r,sys)
    radius = norm(r);
    value = -sys.mu/radius * (1-sys.J2*(sys.Re/radius)^2 * ...
        0.5*(3*(r(3)/radius)^2-1));
end
