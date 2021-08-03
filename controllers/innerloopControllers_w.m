
function w = innerloopControllers(qt, ref, t)
    % ref: [x; y; z; yaw; xdot; ydot; zdot; yaw_dow]
    % System Parameters
    CT  = 3.1582 * 10^(-10);  % [N/rpm^2]
    m = 0.033; % [kg]
    g = 9.81;
    wh = sqrt(m*g/(4*CT));
    
    %% Define Parameters of Position Controller
    kpx = 5; kdx = 5; kix = 0;
    kpy = 5; kdy = 5; kiy = 0;
    kpz = 20; kdz = 10; kiz = 0;
    
    %% Hover Position PD Controller
    
    % Note: rdot = rddot = 0 for hover 
    rdes_ddot1 = kpx*(ref(1)-qt.pos(1)) + kdx*(0-qt.vel(1));
    rdes_ddot2 = kpy*(ref(2)-qt.pos(2)) + kdy*(0-qt.vel(2));
    rdes_ddot3 = kpz*(ref(3)-qt.pos(3)) + kdz*(0-qt.vel(3));

    phi_des   = (rdes_ddot1*sin(ref(4)) - rdes_ddot2*cos(ref(4)))/g;
    delta_des = (rdes_ddot1*cos(ref(4)) + rdes_ddot2*sin(ref(4)))/g;
    psi_des   = ref(4);
    wF        = m*rdes_ddot3 / (8*CT*wh);
    
    %% Attitude PD Controller
    kp_phi   = 3000; kd_phi   = 300;
    kp_delta = 3000; kd_delta = 300;
    kp_psi   = 3000; kd_psi   = 300;
    
    w_phi   = kp_phi*(phi_des-qt.ori(1))     + kd_phi*(0-qt.omega(1));
    w_delta = kp_delta*(delta_des-qt.ori(2)) + kd_delta*(0-qt.omega(2));
    w_psi   = kp_psi*(psi_des-qt.ori(3))     + kd_psi*(0-qt.omega(3));
    
    % Define omega rotation matrix
    Romega = [1 -1 -1 1;
              1 -1 1 -1;
              1  1 1  1;
              1 1 -1 -1];
    
    w = Romega * [wh+wF; w_phi; w_delta; w_psi];
end