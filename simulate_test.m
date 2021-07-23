clc; clear; close all;

tspan = [0 20];
w_max = 17000; % rpm
w_min  = 16000; 
w = [w_max; w_max; w_min; w_min];
y0 = zeros(12, 1);

options = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t, y] = ode45(@(t, y) droneSystem(t, y, w), tspan, y0, options);


%% visualization
figure;
plot(t, y(:, 3), 'Linewidth', 1.5);

figure;
plot(y(:,2), y(:,3), 'Linewidth', 1.5);

