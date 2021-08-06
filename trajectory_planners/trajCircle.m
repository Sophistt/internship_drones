
function [des_state] = trajCircle(t, qn)
    x = 5 * sin(t / 10);
    y = 5 * cos(t / 10);
    z = 5;
    yaw = 0;
    
    des_state.pos = [x;y;z];
    des_state.vel = [0;0;0];
    des_state.acc = [0;0;0];
    des_state.yaw = yaw;
    des_state.yawdot = 0;
end