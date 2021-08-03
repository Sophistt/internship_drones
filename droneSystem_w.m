
function sdot = droneSystem_w(t, s, params)
    % q = [x y z, phi, theta, psi, xdot, ydot, zdot, phi_dot, theta_dot, psi_dot]
    % w = [w1, w2, w3, w4]
    % Nonlinear, tryaint rotation
    qd = stateToQd(s);
    
    ref = trajectoryGen(t, qd);
    f = innerloopControllers(qd, ref);
    
    sdot = dynamicsRender_w(t, s, f, params);
end