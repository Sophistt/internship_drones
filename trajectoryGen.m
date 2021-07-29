
function traj = trajectoryGen(t)
    x = 5 * sin(t / 10);
    y = 5 * cos(t / 10);
    z = 5;
    yaw = 0;
    traj = [x; y; z; yaw];
end