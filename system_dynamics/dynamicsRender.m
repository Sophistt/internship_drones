
function qdot = dynamicsRender(t, qd, f, params)
    
    
    %% Align system variables
    % x = qt.pos(1); y = qt.pos(2); z = qt.pos(3);
    % xdot = qt.vel(1); ydot = qt.vel(2); zdot = qt.vel(3);
    phi = qd.euler(1); theta = qd.euler(2); psi = qd.euler(3);

    % Angular velocity rotation matrix
    R = [cos(theta), 0, -cos(phi)*sin(theta);
         0         , 1, sin(phi);
         sin(theta), 0, cos(phi)*cos(theta)];
    temp = R * qd.omega;
    
    phi_dot = temp(1); theta_dot = temp(2); psi_dot = temp(3);
    
    % Control input
    f1 = f(1); f2 = f(2); f3 = f(3); f4 = f(4);
    f_all = f1 + f2 + f3 + f4;
    
    %% Update system dynamics

    % Define ode (x y z)ddot
    xddot = (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) * f_all / params.mass;
    yddot = (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) * f_all / params.mass;
    zddot = cos(phi)*cos(theta)  * f_all / params.mass - params.grav;
    
    % Define ode (phi theta psi)ddot
    phi_ddot   = (theta_dot*psi_dot*(params.Jyy-params.Jzz) ...
                 + params.arm_length/sqrt(2)*(-f1-f2+f3+f4)) / params.Jxx;
    theta_ddot = (psi_dot*phi_dot*(params.Jzz-params.Jxx)   ...
                 + params.arm_length/sqrt(2)*(-f1+f2+f3-f4)) / params.Jyy;
    psi_ddot   = (theta_dot*phi_dot*(params.Jxx-params.Jyy) ...
                 + params.gamma*(f1-f2+f3-f4)) / params.Jzz;
    
    % Assign value
    qdot = [qd.vel; qd.omega; xddot; yddot; zddot; phi_ddot; theta_ddot; psi_ddot];
    
end