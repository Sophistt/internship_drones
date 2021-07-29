
function qdot = droneSystem_w(t, q)
    % q = [x y z, phi, theta, psi, xdot, ydot, zdot, phi_dot, theta_dot, psi_dot]
    % w = [w1, w2, w3, w4]
    % Nonlinear, tryaint rotation
    
    qt.pos = q(1:3); qt.ori = q(4:6);
    qt.vel = q(7:9); qt.omega = q(10:12);
    
    % Define reference position or refrence trajectory
    ref = [5; 5; 5; 0];
    
    w = innerloopControllers(qt, ref, t);
    
    qdot = dynamicsRender(qt, t, w);
end