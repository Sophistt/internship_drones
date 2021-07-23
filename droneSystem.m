
function qdot = droneSystem(t, q, w)
    % q = [x y z, phi, theta, psi, xdot, ydot, zdot, phi_dot, theta_dot, psi_dot]
    % w = [w1, w2, w3, w4]
    % Nonlinear, tryaint rotation
    
    Jxx = 1.395 * 10^(-5); % [Kg x m^2]
    Jyy = 1.436 * 10^(-5); % [Kg x m^2]
    Jzz = 2.173 * 10^(-5); % [Kg x m^2]
    CT  = 3.1582 * 10^(-10);  % [N/rpm^2]
    CD  = 7.9379  * 10^(-12); % [Nm/rpm^2]
    d = 39.73 * 10^(-3); % [m]
    m = 0.033; % [kg]
    g = 9.81;
    
    x = q(1); y = q(2); z = q(3);
    phi = q(4); theta = q(5); psi = q(6);
    xdot = q(7); ydot = q(8); zdot = q(9);
    R = [cos(theta), 0, -cos(phi)*sin(theta);
         0         , 1, sin(phi);
         sin(theta), 0, cos(phi)*cos(theta)];
    temp = R * [q(10); q(11); q(12)];
    phi_dot = temp(1); theta_dot = temp(2); psi_dot = temp(3);
    
    w1 = w(1); w2 = w(2); w3 = w(3); w4 = w(4);
    w_all = w1^2 + w2^2 + w3^2 + w4^2;
    
    % Define ode (x y z)ddot
    xddot = (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) * CT * w_all / m;
    yddot = (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) * CT * w_all / m;
    zddot = cos(phi)*cos(theta) * CT * w_all / m - g;
    
    % Define ode (phi theta psi)ddot
    phi_ddot   = theta_dot*psi_dot*(Jyy-Jzz) + CT*d/sqrt(2)*(-w1^2-w2^2+w3^2+w4^2);
    theta_ddot = psi_dot*phi_dot*(Jzz-Jxx) + CT*d/sqrt(2)*(-w1^2+w2^2+w3^2-w4^2);
    psi_ddot   = theta_dot*phi_dot*(Jxx-Jyy) + CD*(w1^2-w2^2+w3^2-w4^2);
    
    % Assign value
    qdot = [q(7:12); xddot; yddot; zddot; phi_ddot; theta_ddot; psi_ddot];
end