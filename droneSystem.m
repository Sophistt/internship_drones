
function sdot = droneSystem(t, s, qn, controlhandle, trajhandle, params)
    % q = [x y z, phi, theta, psi, xdot, ydot, zdot, phi_dot, theta_dot, psi_dot]
    % w = [w1, w2, w3, w4]
    % Nonlinear, tryaint rotation
    qd{qn} = stateToQd(s);
    
    % Get desired state at time t from trajectory planner
    desired_state = trajhandle(t, qn);
    
    % The desired_state is set in the trajectory generator
    qd{qn}.pos_des      = desired_state.pos;
    qd{qn}.vel_des      = desired_state.vel;
    qd{qn}.acc_des      = desired_state.acc;
    qd{qn}.yaw_des      = desired_state.yaw;
    qd{qn}.yawdot_des   = desired_state.yawdot;
    
    f = controlhandle(qd, t, qn, params);
    
    sdot = dynamicsRender_w(t, s, f, params);
end