clc; clear; close all;

x = -0:0.03:4;
y = -2:0.03:2;

des = [3; 0];
obs.pos = [1;1];
obs.shape = [1;1];

Ka  = 0.01;
Kb  = 0.000001;
d0  = 0.2;

[X,Y] = meshgrid(x, y);

dis = zeros(length(x), length(y));
Uobs = zeros(length(x), length(y));
for i = 1:length(x)
    for j = 1:length(y)
        [dis(j,i), ~, ~] = disToObs([x(i),y(j)], obs);
        if dis(j, i) < d0
            Uobs(j,i) = 1/2 * Kb *(1/dis(j,i)-1/d0)^2;
        else
            Uobs(j,i) = 0;
        end
    end
end


Z = 1/2 * Ka * ((X-des(1)).^2 + (Y-des(2)).^2);
Z = Z + Uobs;
surf(X,Y,Z)
hold on;

q = [0; 2];
for i = 1:500
    Fatt_x = - Ka * (q(1)-des(1));
    Fatt_y = - Ka * (q(2)-des(2));
    [dis, dx, dy]    = disToObs(q, obs);
    if dis < d0
        Frep_x = 5e-6 * Kb * (1/dis-1/d0)*(1/dis)^2*((q(1)-dx)/dis);
        Frep_y = 1e-6 * Kb * (1/dis-1/d0)*(1/dis)^2*((q(2)-dy)/dis);
    else
        Frep_x = 0;
        Frep_y = 0;
    end 
    q(1) = q(1) + Fatt_x + 0.03 * Frep_x; 
    q(2) = q(2) + Fatt_y + 0.3 * Frep_y;
    z    = 1/2 * Ka * ((q(1)-des(1)).^2 + (q(2)-des(2)).^2);
    plot3(q(1),q(2),z,'or', 'Linewidth', 2);
end
