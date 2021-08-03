
function f = innerloopControllers(qt, ref)
    % ref: [x; y; z; yaw; xdot; ydot; zdot; yaw_dow]
    % System Parameters
    Jxx = 1.395 * 10^(-5); % [Kg x m^2]
    Jyy = 1.436 * 10^(-5); % [Kg x m^2]
    Jzz = 2.173 * 10^(-5); % [Kg x m^2]
    I = diag([Jxx, Jyy, Jzz]);
    CT  = 3.1582 * 10^(-10);  % [N/rpm^2]
    CD  = 7.9379  * 10^(-12); % [Nm/rpm^2]
    gamma = CD / CT;
    d = 39.73 * 10^(-3); L = d / sqrt(2); % [m]
    m = 0.033; % [kg]
    g = 9.81;
    
    %% Define Parameters of Position Controller
    kpx = 5; kdx = 5; kix = 0;
    kpy = 5; kdy = 5; kiy = 0;
    kpz = 20; kdz = 10; kiz = 0;
    
    %% Hover Position PD Controller
    
    % Note: rdot = rddot = 0 for hover 
    rdes_ddot1 = kpx*(ref(1)-qt.pos(1)) + kdx*(0-qt.vel(1));
    rdes_ddot2 = kpy*(ref(2)-qt.pos(2)) + kdy*(0-qt.vel(2));
    rdes_ddot3 = kpz*(ref(3)-qt.pos(3)) + kdz*(0-qt.vel(3));
    u1         = m*(g+rdes_ddot3);

    phi_des   = (rdes_ddot1*sin(ref(4)) - rdes_ddot2*cos(ref(4)))/g;
    delta_des = (rdes_ddot1*cos(ref(4)) + rdes_ddot2*sin(ref(4)))/g;
    psi_des   = ref(4);
    
    %% Attitude PD Controller
    kp_phi   = 3000; kd_phi   = 300;
    kp_delta = 3000; kd_delta = 300;
    kp_psi   = 3000; kd_psi   = 300;
    
    w_phi   = kp_phi  *(phi_des  -qt.ori(1)) + kd_phi  *(0-qt.omega(1));
    w_delta = kp_delta*(delta_des-qt.ori(2)) + kd_delta*(0-qt.omega(2));
    w_psi   = kp_psi  *(psi_des  -qt.ori(3)) + kd_psi  *(0-qt.omega(3));
    u2      = I * [w_phi; w_delta; w_psi];
    
    % Define transition matrix from u to fi
    Trans = [0.25 -8.8989 -8.8989  9.9466;
             0.25 -8.8989  8.8989 -9.9466;
             0.25  8.8989  8.8989  9.9466;
             0.25  8.8989 -8.8989 -9.9466];
    
    f = Trans * [u1; u2];
end