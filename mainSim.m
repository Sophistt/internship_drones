clc; clear; close all;
addpath(genpath(pwd)); % Add files;

controlOmega = 0;
%tspan = 0:.1:100;
tspan = [0 100];
y0 = zeros(12, 1);
options = odeset('RelTol',1e-8,'AbsTol',1e-10);

% Define simulation environment
env = envDefine();
goal = 0;

if controlOmega
    [t, y] = ode45(@(t, y) droneSystem_w(t, y), tspan, y0, options);
else
    [t, y] = ode15s(@(t, y) droneSystem(t, y, env, goal), tspan, y0, options);
end


%% visualization
figure;
subplot(3, 1, 1);
plot(t, y(:, 1), 'Linewidth', 1.5);
xlabel("Time [s]")
ylabel("Position [m]")
title("X-axis")
grid on;

subplot(3, 1, 2);
plot(t, y(:, 2), 'Linewidth', 1.5);
xlabel("Time [s]")
ylabel("Position [m]")
title("Y-axis")
grid on;

subplot(3, 1, 3);
plot(t, y(:, 3), 'Linewidth', 1.5);
xlabel("Time [s]")
ylabel("Position [m]")
title("Z-axis")
grid on;

figure;
plot3(y(:,1), y(:,2), y(:,3), 'Linewidth', 1.5);
plotcube([.4 .4 .6],[.8 -.2 0],.8,[1 0 0]);
plotcube([.4 .4 .6],[1.8 -1.2 0],.8,[1 0 0]);
plotcube([.4 .4 .6],[2.8 -.2 0],.8,[1 0 0]);
xlabel("Position X [m]")
ylabel("Position Y [m]")
zlabel("Position Z [m]")
grid on;

