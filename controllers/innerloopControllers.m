
function f = innerloopControllers(qd, t, qn, params)
    
    %% Define Parameters of Position Controller
    kpx = 5; kdx = 5; kix = 0;
    kpy = 5; kdy = 5; kiy = 0;
    kpz = 20; kdz = 10; kiz = 0;
    
    %% Hover Position PD Controller
    
    % Note: rdot = rddot = 0 for hover 
    rdes_ddot1 = kpx*(qd{qn}.pos_des(1)-qd{qn}.pos(1)) + kdx*(qd{qn}.vel_des(1)-qd{qn}.vel(1));
    rdes_ddot2 = kpy*(qd{qn}.pos_des(2)-qd{qn}.pos(2)) + kdy*(qd{qn}.vel_des(2)-qd{qn}.vel(2));
    rdes_ddot3 = kpz*(qd{qn}.pos_des(3)-qd{qn}.pos(3)) + kdz*(qd{qn}.vel_des(3)-qd{qn}.vel(3));
    u1         = params.mass * (params.grav + rdes_ddot3);

    phi_des   = (rdes_ddot1*sin(qd{qn}.yaw_des) - rdes_ddot2*cos(qd{qn}.yaw_des))/ params.grav;
    theta_des = (rdes_ddot1*cos(qd{qn}.yaw_des) + rdes_ddot2*sin(qd{qn}.yaw_des))/ params.grav;
    psi_des   = qd{qn}.yaw_des;
    
    %% Attitude PD Controller
    kp_phi   = 3000; kd_phi   = 100;
    kp_theta = 3000; kd_theta = 100;
    kp_psi   = 3000; kd_psi   = 10;
    
    w_phi   = kp_phi  * (phi_des  -qd{qn}.euler(1)) + kd_phi  *(0-qd{qn}.omega(1));
    w_theta = kp_theta* (theta_des-qd{qn}.euler(2)) + kd_theta*(0-qd{qn}.omega(2));
    w_psi   = kp_psi  * (psi_des  -qd{qn}.euler(3)) + kd_psi  *(0-qd{qn}.omega(3));
    u2      = params.I* [w_phi; w_theta; w_psi];
    
    % Define transition matrix from u to fi
    Trans = [0.25 -8.8989 -8.8989  9.9466;
             0.25 -8.8989  8.8989 -9.9466;
             0.25  8.8989  8.8989  9.9466;
             0.25  8.8989 -8.8989 -9.9466];
    
    f = Trans * [u1; u2];
end