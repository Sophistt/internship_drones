
function qdot = dynamicsRender(qt, t, w)
    
    %% Define system parameters
    Jxx = 1.395 * 10^(-5); % [Kg x m^2]
    Jyy = 1.436 * 10^(-5); % [Kg x m^2]
    Jzz = 2.173 * 10^(-5); % [Kg x m^2]
    CT  = 3.1582 * 10^(-10);  % [N/rpm^2]
    CD  = 7.9379  * 10^(-12); % [Nm/rpm^2]
    d = 39.73 * 10^(-3); % [m]
    m = 0.033; % [kg]
    g = 9.81;
    
    %% Align system variables
    % x = qt.pos(1); y = qt.pos(2); z = qt.pos(3);
    % xdot = qt.vel(1); ydot = qt.vel(2); zdot = qt.vel(3);
    phi = qt.ori(1); theta = qt.ori(2); psi = qt.ori(3);

    % Angular velocity rotation matrix
    R = [cos(theta), 0, -cos(phi)*sin(theta);
         0         , 1, sin(phi);
         sin(theta), 0, cos(phi)*cos(theta)];
    temp = R * qt.omega;
    
    phi_dot = temp(1); theta_dot = temp(2); psi_dot = temp(3);
    
    % Control input
    w1 = w(1); w2 = w(2); w3 = w(3); w4 = w(4);
    w_all = w1^2 + w2^2 + w3^2 + w4^2;
    
    %% Update system dynamics
    
    % Define ode (x y z)ddot
    xddot = (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) * CT * w_all / m;
    yddot = (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) * CT * w_all / m;
    zddot = cos(phi)*cos(theta) * CT * w_all / m - g;
    
    % Define ode (phi theta psi)ddot
    phi_ddot   = (theta_dot*psi_dot*(Jyy-Jzz) + CT*d/sqrt(2)*(-w1^2-w2^2+w3^2+w4^2)) / Jxx;
    theta_ddot = (psi_dot*phi_dot*(Jzz-Jxx) + CT*d/sqrt(2)*(-w1^2+w2^2+w3^2-w4^2)) / Jyy;
    psi_ddot   = (theta_dot*phi_dot*(Jxx-Jyy) + CD*(w1^2-w2^2+w3^2-w4^2)) / Jzz;
    
    % Assign value
    qdot = [qt.vel; qt.omega; xddot; yddot; zddot; phi_ddot; theta_ddot; psi_ddot];
    
end