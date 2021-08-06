

function sdot = droneSystem_euler(t, s, qn, controlhandle, trajhandle, params)
    % q = [x y z, phi, theta, psi, xdot, ydot, zdot, phi_dot, theta_dot, psi_dot]
    % f = [f1, f2, f3, f4]
    % Nonlinear, tryaint rotation
    
    % Attain system params
    qd{qn}.pos = s(1:3);   qd{qn}.vel = s(7:9);
    qd{qn}.euler = s(4:6); qd{qn}.omega = s(10:12);
    
     % Get desired state at time t from trajectory planner
    desired_state = trajhandle(t, qn);
    
    % The desired_state is set in the trajectory generator
    qd{qn}.pos_des      = desired_state.pos;
    qd{qn}.vel_des      = desired_state.vel;
    qd{qn}.acc_des      = desired_state.acc;
    qd{qn}.yaw_des      = desired_state.yaw;
    qd{qn}.yawdot_des   = desired_state.yawdot;
        
    f = controlhandle(qd, t, qn, params);
    
    sdot = dynamicsRender(t, s, f, params); 
end