

function sdot = droneSystem(t, s, params)
    % q = [x y z, phi, theta, psi, xdot, ydot, zdot, phi_dot, theta_dot, psi_dot]
    % f = [f1, f2, f3, f4]
    % Nonlinear, tryaint rotation
    
    qd.pos = s(1:3);  qd.vel = s(7:9);
    qd.euler = s(4:6); qd.omega = s(10:12);
        
    % Define reference position or refrence trajectory
    ref = trajectoryGen(t, qd);
    f = innerloopControllers(qd, ref);
    
    sdot = dynamicsRender(t, qd, f, params); 
end