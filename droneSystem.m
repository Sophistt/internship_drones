

function qdot = droneSystem(t, q, env, goal)
    % q = [x y z, phi, theta, psi, xdot, ydot, zdot, phi_dot, theta_dot, psi_dot]
    % f = [f1, f2, f3, f4]
    % Nonlinear, tryaint rotation
    
    qt.pos = q(1:3); qt.ori = q(4:6);
    qt.vel = q(7:9); qt.omega = q(10:12);
        
    % Define reference position or refrence trajectory
    ref = trajectoryGen(t, qt, env, goal);
    
    f = innerloopControllers(qt, ref);
    
    qdot = dynamicsRender(qt, t, f); 
end